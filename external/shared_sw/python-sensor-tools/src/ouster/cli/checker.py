# TODO:
# - borrow dest ip hack from example
# - distutils, proper build

import numpy as np
np.set_printoptions(precision=4)

import argparse
import sys
import time
from datetime import datetime
import collections
import psutil

import ouster.client._sensor as sensor
import ouster.client.lidardata as osl

def log(str, file=sys.stderr):
    print("{} {}".format(datetime.now(), str), file=file, end='')

# Utility class for tracking PPM
#  Maintains a rolling window of counts within eval_window of the 'current' count
#  PPM is estimated as 1E6 * (n_tracked - 1)/(latest - earliest + 1)
class PPMTracker:
    est_ppm = 0
    threshold = 0
    max_est = 0
    n_tracked = 0

    # threshold, int, [R1]:                 Maximum 'valid' ppm (per error checks). Ignored set < 0
    # break_at_max, bool, T/F:              sys.exit(1) if an eval exceeds threshold
    # eval_window, int, [1000, +inf]:       PPM calc's rolling window
    # min_window, int, [0, eval_window]:    Minimum (rolling) window across samples to trigger an error
    def __init__(self, threshold, break_at_max, eval_window, min_window):
        self.counts = collections.deque()
        self.threshold = threshold
        self.break_at_max = break_at_max
        self.eval_window = max(1000, eval_window)
        self.min_window = min(min_window, self.eval_window)

    # Track a count & update est. PPM
    #  If est_ppm > threshold across >=min_window packets, logs and (if break_at_max) exits
    def track(self, new_count):
        # Prepend new count & deque counts older than eval_window
        self.counts.appendleft(new_count)
        while self.counts[0] - self.counts[-1] > self.eval_window:
            self.counts.pop()

        # Crudely estimate PPM for tracker widget
        self.n_tracked+=1
        self.est_ppm = 1E6 * self.n_tracked / new_count
        if len(self.counts) > 1:
            # Estimate PPM
            window = self.counts[0] - self.counts[-1] + 1
            self.est_ppm = 1E6 * (len(self.counts) - 1) / window

            # Evaluate & test/log
            if window > self.min_window:
                self.max_est = max(self.max_est, self.est_ppm)

                if self.threshold > 0 and self.est_ppm > self.threshold and self.break_at_max:
                    log('PPM Error: {:6}/{} counts, est. PPM: {:<.1f} (g.t. thresh: {})\n'.format(
                        len(self.counts), window + 1, round(self.est_ppm, 1), self.threshold))
                    sys.exit(1)

# Utility class for tracking timeout windows
class Timer:
    def __init__(self, timeout, log_msg='', exit=False):
        self.timeout = max(0, timeout)
        if log_msg:
            self.log_msg = ', ' + log_msg
        self.exit = exit
        self.t0 = time.monotonic()

    def age(self):
        return time.monotonic() - self.t0

    def restart(self):
        self.t0 = time.monotonic()

    def timed_out(self):
        if self.timeout and self.age() >= self.timeout:
            log('Timeout exceeded: {} > {}'.format(self.age(), self.timeout)
                + self.log_msg + '\n')
            if self.exit:
                sys.exit(1)
            return True
        return False

class LidarState:
    n_packets = 0

    def __init__(self, pkt_buf, ppm_tracker, lidar_mode, pf):
        self.ppm_err = ppm_tracker
        self.col_res = 1024
        self.scan_hz = 10
        self.pf = pf
        for known_mode in ['512x10','1024x10','2048x10','512x20','1024x20']:
            if lidar_mode == known_mode:
                x_i = known_mode.find('x')
                self.col_res = int(known_mode[:x_i])
                self.scan_hz = int(known_mode[x_i + 1:])

        self.pkt_data       = osl.LidarPacket(pkt_buf, pf)
        self.valid_view     = self.pkt_data.view(osl.ColHeader.VALID)
        self.col_id_view    = self.pkt_data.view(osl.ColHeader.MEASUREMENT_ID)
        self.frame_id_view  = self.pkt_data.view(osl.ColHeader.FRAME_ID)
        self.ticks_view     = self.pkt_data.view(osl.ColHeader.ENCODER_COUNT)
        self.ts_view        = self.pkt_data.view(osl.ColHeader.TIMESTAMP)

        self.range_view     = self.pkt_data.view(osl.ChanField.RANGE)
        self.reflc_view     = self.pkt_data.view(osl.ChanField.REFLECTIVITY)
        self.signl_view     = self.pkt_data.view(osl.ChanField.INTENSITY)
        self.noise_view     = self.pkt_data.view(osl.ChanField.AMBIENT)

        self.reset_last()
        self.reset_scan()

    def reset_last(self):
        self.valid      = None
        self.col_id     = None
        self.frame_id   = None
        self.ticks      = None
        self.ts         = None

    def reset_scan(self):
        self.scan_px_range = np.zeros((self.pf.pixels_per_column, self.col_res))
        self.scan_px_reflc = np.zeros((self.pf.pixels_per_column, self.col_res))
        self.scan_px_signl = np.zeros((self.pf.pixels_per_column, self.col_res))
        self.scan_px_noise = np.zeros((self.pf.pixels_per_column, self.col_res))

    def handle_packet(self):
        self.n_packets += 1

        # Find how many of the packet's columns are in the current scan
        scan_ind = self.col_id_view[0]
        scan_cols = self.col_id_view[-1] - self.col_id_view[0] + 1
        check_scan = False
        if scan_ind + self.pf.columns_per_packet > self.col_res or scan_ind == 0:
            scan_cols = (self.pf.columns_per_packet - self.col_id_view[-1] - 1) % self.pf.columns_per_packet
            check_scan = True

        valid       = self.valid_view
        col_id      = self.col_id_view
        frame_id    = self.frame_id_view
        ticks       = self.ticks_view
        ts          = self.ts_view
        # If populating the scan, prepend the last packet's trailing column(s) (for delta checks)
        if self.valid == 0xffffffff:
            valid       = np.insert(valid,     0, self.valid)
            col_id      = np.insert(col_id,    0, self.col_id)
            frame_id    = np.insert(frame_id,  0, self.frame_id)
            ticks       = np.insert(ticks,     0, self.ticks)
            ts          = np.insert(ts,        0, self.ts)
        pre_str = 'LIDAR pkt {:<9} (cols [{:4}:{:<4}]): '.format(
            self.n_packets, col_id[0], col_id[-1])

        # Evaluate
        error_count = 0
        if not all(valid == 0xffffffff):
            vhex = np.vectorize(hex)
            log(pre_str + 'invalid crc(s): {}\n'.format(vhex(valid)))
            error_count += 1
            self.reset_last()
        else:
            # ticks
            if any(ticks < 0) or any(ticks > self.pf.encoder_ticks_per_rev):
                log(pre_str + 'encoder ticks out of range: {}\n'.format(ticks))
                error_count += 1

            # col_id delta
            d_col_id = np.diff(col_id)
            if any(d_col_id % self.col_res != 1):
                log(pre_str + 'unexpected col_id delta(s): {}\n'.format(d_col_id  % self.col_res))
                error_count += 1

            # frame_id delta
            d_frame_id = np.diff(frame_id)
            if not check_scan and any(d_frame_id != 0):
                log(pre_str + 'nonzero frame_id(={}) delta(s): {}\n'.format(frame_id[0], d_frame_id))
                error_count += 1
            elif check_scan and (any(frame_id[:scan_cols]    != (frame_id[-1] - 1)) % 2**16
                             or  any(frame_id[ scan_cols+1:] != (frame_id[0]  + 1)) % 2**16):
                log(pre_str + 'unexpected scan frame_id(={}) delta(s): {}\n'.format(frame_id[0], d_frame_id))
                error_count += 1

            # ts delta (ms)
            d_ts = np.diff(ts) * 1E-3
            if any(d_ts < 0):
                log(pre_str + 'negative ts delta(s) (ms): {}\n'.format(d_ts))
                error_count += 1
            exp_dt = 10 * self.col_res * self.scan_hz * 1E-3
            if any(d_ts > 2 * exp_dt):
                log(pre_str + 'ts delta(s) g.t. 2x expected ({}ms): {}\n'.format(exp_dt, d_ts))
                error_count += 1

        self.valid    = self.valid_view[-1]
        self.col_id   = self.col_id_view[-1]
        self.frame_id = self.frame_id_view[-1]
        self.ticks    = self.ticks_view[-1]
        self.ts       = self.ts_view[-1]
        if not error_count:
            self.scan_px_range[:,scan_ind:scan_ind + scan_cols] = self.range_view[:,:scan_cols]
            self.scan_px_reflc[:,scan_ind:scan_ind + scan_cols] = self.reflc_view[:,:scan_cols]
            self.scan_px_signl[:,scan_ind:scan_ind + scan_cols] = self.signl_view[:,:scan_cols]
            self.scan_px_noise[:,scan_ind:scan_ind + scan_cols] = self.noise_view[:,:scan_cols]
            if check_scan:
                offset = self.pf.columns_per_packet - scan_cols
                error_count += self.check_scan()
                self.reset_scan()
                if check_scan and offset % self.pf.columns_per_packet:
                    self.scan_px_range[:,:offset] = self.range_view[:,-offset:]
                    self.scan_px_reflc[:,:offset] = self.reflc_view[:,-offset:]
                    self.scan_px_signl[:,:offset] = self.signl_view[:,-offset:]
                    self.scan_px_noise[:,:offset] = self.noise_view[:,-offset:]

        # Update & check ppm_err rate
        if error_count:
            self.ppm_err.track(self.n_packets)

    def check_scan(self):
        error_count = 0
        for scan_px, strng in ((self.scan_px_range, "range"),
                               (self.scan_px_reflc, "reflectivity"),
                               (self.scan_px_signl, "signal"),
                               (self.scan_px_noise, "noise")):
            dead_rows = self.pf.pixels_per_column - np.sum(np.any(np.diff(scan_px, axis=1), axis=1))
            if dead_rows:
                log("LIDAR pkt {:<9}: last scan's {} had {} dead row(s)!\n".format(
                    self.n_packets, strng, dead_rows))
                error_count += 1
        return error_count

class IMUState:
    n_packets = 0

    # state_maxima: Vector of boundary conditions on IMU state
    #     pkt_t :   max period between packets (ms)
    #     ang_V :   max abs. angular velocity (rad/s)
    #     ang_dV:   max abs. angular velocity delta (rad/s^2)
    #     lin_dA:   max abs. linear acceleration delta (g/s)
    def __init__(self, pkt_buf, ppm_tracker, state_maxima, pf):
        self.buf = pkt_buf
        self.ppm_err = ppm_tracker
        self.max_dt, self.max_ang_V, self.max_ang_dV, self.max_lin_dA = state_maxima
        self.pf = pf

    # consume a new packet from the imu, updating & comparing state to (old) self
    def handle_packet(self):
        self.n_packets += 1
        error_count = 0

        pre_str = 'IMU pkt {:<9}: '.format(self.n_packets)

        sys_t = self.pf.imu_sys_ts(self.buf)
        accl_t = self.pf.imu_accel_ts(self.buf)
        gyro_t = self.pf.imu_gyro_ts(self.buf)

        A = [self.pf.imu_la_x(self.buf), self.pf.imu_la_y(self.buf), self.pf.imu_la_z(self.buf)]
        W = [self.pf.imu_av_x(self.buf), self.pf.imu_av_y(self.buf), self.pf.imu_av_z(self.buf)]

        if self.n_packets > 1:
            if sys_t <= self.sys_t:
                log(pre_str + 'sys_t  increment failure: {:3}->{:3} ({:+})\n'.format(
                    sys_t, self.sys_t, sys_t - self.sys_t))
                error_count += 1
            if accl_t <= self.accl_t:
                log(pre_str + 'accl_t increment failure: {:3}->{:3} ({:+})\n'.format(
                    accl_t, self.accl_t, accl_t - self.accl_t))
                error_count += 1
            if gyro_t <= self.gyro_t:
                log(pre_str + 'gyro_t increment failure: {:3}->{:3} ({:+})\n'.format(
                    gyro_t, self.gyro_t, gyro_t - self.gyro_t))
                error_count += 1

            sys_dt = sys_t - self.sys_t
            if sys_dt > 1E6 * self.max_dt:
                log(pre_str + 'sys_dt too slow! {:3} ms >= {:3} ({:<.3f} Hz)\n'.format(
                    sys_dt, 1E6 * self.max_dt, 1E9/sys_dt))
                error_count += 1
            accl_dt = accl_t - self.accl_t
            if accl_dt > 1E6 * self.max_dt:
                log(pre_str + 'accl_dt too slow! {:3} ms >= {:3} ({:<.3f} Hz)\n'.format(
                    accl_dt, 1E6 * self.max_dt, 1E9/accl_dt))
                error_count += 1
            gyro_dt = gyro_t - self.gyro_t
            if gyro_dt > 1E6 * self.max_dt:
                log(pre_str + 'gyro_dt too slow! {:3} ms >= {:3} ({:<.3f} Hz)\n'.format(
                    gyro_dt, 1E6 * self.max_dt, 1E9/gyro_dt))
                error_count += 1

            g_dir = 0
            for i in range(3):
                dW_dt = 1E9 * (W[i] - self.W[i])/sys_dt
                dA_dt = 1E9 * (A[i] - self.A[i])/sys_dt
                if abs(A[i]) > abs(A[g_dir]):
                    g_dir = i
                if abs(W[i]) > self.max_ang_V:
                    log(pre_str + 'angular velocity, |W[{}] = {:+<.3f}| > {}\n'.format(
                        i, W[i], self.max_ang_V))
                    error_count += 1
                if abs(dW_dt) > self.max_ang_dV:
                    log(pre_str + 'angular velocity delta, |dWdt[{}] = {:+<.3f}| > {}\n'.format(
                        i, dW_dt, self.max_ang_dV))
                    error_count += 1
                if abs(dA_dt) > self.max_lin_dA:
                    log(pre_str + 'linear acceleration delta, |dAdt[{}] = {:+<.3f}| > {}\n'.format(
                        i, dA_dt, self.max_lin_dA))
                    error_count += 1
            if g_dir != 2:
                log(pre_str + 'G not pointing along the Z-axis: [{:<+.3f},{:<+.3f},{:<+.3f}]\n'.format(
                    A[0], A[1], A[2]))
                error_count += 1

        self.sys_t = sys_t
        self.accl_t = accl_t
        self.gyro_t = gyro_t
        self.A = A
        self.W = W

        # Update & check ppm_err rate
        if error_count:
            self.ppm_err.track(self.n_packets)

# Utility class to print a spinning widget: | / - \ | / - ...
class StatusTracker:
    i = 0
    def __init__(self, period, ldr_ppm, imu_ppm):
        self.period = max(1, period)
        self.ldr_ppm = ldr_ppm
        self.imu_ppm = imu_ppm
        self.cpu_pct = psutil.cpu_percent()

    # Print | \ - or /, switching which every (period) calls
    def print_status(self):
        if not sys.stdout.isatty():
            return
        self.i = (self.i + 1) % self.period
        if not self.i % self.period:
            self.cpu_pct = psutil.cpu_percent()
        est_ldr_ppm = round(self.ldr_ppm.est_ppm,1)
        est_imu_ppm = round(self.imu_ppm.est_ppm,1)
        print('CPU%: {}\tLIDAR PPM: {:>6}\tIMU PPM: {:>6}'.format(
            self.cpu_pct, est_ldr_ppm, est_imu_ppm), end='\r')


def CheckerParser():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('host_init', nargs='*',
        help='[hostname data_destination]: ' +
        'Connect to sensor \'hostname\' streaming data to \'data_destination\'')
    parser.add_argument('--port_init', nargs = 2, type = int,
        help='[lidar_port imu_port]: ' +
        'Bind & listen to a sensor streaming to \'lidar_port\' & \'imu_port\' ' +
        'in lieu of using hostname & data_destination')
    parser.add_argument('-m', '--lidar_mode', default='1024x10',
        choices = ['512x10','1024x10','2048x10','512x20','1024x20'],
        help = 'Sensor\'s image resolution. Configures sensor on hostname init')
    parser.add_argument('-t','--timeout', type = float, default = 0,
        help = 'Time (seconds) this script runs before halting. Ignored set <= 0')
    parser.add_argument('-s', '--socket_timeout', type = float, default = 0,
        help = 'Timeout (seconds) to hear both lidar & IMU. Ignored set <= 0')
    parser.add_argument('--max_ppm', nargs = 2, type = int, default = [0, 0],
        help = 'Max number of LIDAR or IMU packet errors (per million)\
         accepted. Ignored set < 0')
    parser.add_argument('--ppm_window', type = int, default = 1E6,
        help = 'Maximum number of packets (rolling) evaluated when checking PPM')
    parser.add_argument('--ppm_min_window', nargs = 2, type = int, default = [10000, 10],
        help = 'Minimum number of LIDAR/IMU packets (rolling) needed to trigger a PPM error')
    parser.add_argument('-b', '--break_on_err', action = 'store_true',
        help = 'Set script to exit on any error')
    parser.add_argument('--imu_maxima', nargs = 3, type = float, default = [20, 100, 1000, 100],
        help = 'IMU\'s state\'s abs. maxima: ' +
            'packet period (ms)' +
            'angular velocity (rad/s), ' +
            'angular velocity delta (rad/s*2), ' +
            'linear acceleration delta (g/s)')
    return parser

def run():
    # Prepare running configuration
    parser = CheckerParser()
    args = parser.parse_args()
    run_timer = Timer(args.timeout, 'halting checker.py')

    # Acquire sensor, trying by hostname & then by port numbers
    # TODO: this doesn't quite make sense
    # - if we want a no-tcp init option, need to require metadata file with data_format parameters
    # - if we just want a no-reconfigure no-reinit option, we can read ports from the sensor config over tcp
    cli = None
    if len(args.host_init) == 2:
        cli = sensor.init_client(args.host_init[0], args.host_init[1], sensor.lidar_mode_of_string(args.lidar_mode))
    elif args.port_init and len(args.host_init) == 1:
        cli = sensor.init_client(args.host_init[0], int(args.port_init[0]), int(args.port_init[1]))
        data_format = sensor.default_sensor_info(sensor.lidar_mode_of_string(args.lidar_mode))
    else:
        print('Need to provide either sensor host and dest args or host and ports')
        sys.exit(1)

    if not cli:
        print('Failed to connect by host_init:"{}" or port_init:"{}"\n'.format(args.host_init, args.port_init),
              file=sys.stderr)
        parser.print_help(sys.stderr)
        sys.exit(1)

    meta = sensor.parse_metadata(sensor.get_metadata(cli))
    pf = sensor.get_format(meta)

    # Prepare tracking objects
    lidar_buf = bytearray(pf.lidar_packet_size + 1)
    lidar_ppm = PPMTracker(args.max_ppm[0], args.break_on_err, args.ppm_window, args.ppm_min_window[0])
    lidar_timer = Timer(args.socket_timeout, 'Failed to receive LIDAR packets', exit=True)
    lidar = LidarState(lidar_buf, lidar_ppm, args.lidar_mode, pf)

    imu_buf = bytearray(pf.imu_packet_size + 1)
    imu_ppm = PPMTracker(args.max_ppm[1], args.break_on_err, args.ppm_window, args.ppm_min_window[1])
    imu_timer = Timer(args.socket_timeout, 'Failed to receive IMU packets', exit=True)
    imu = IMUState(imu_buf, imu_ppm, args.imu_maxima, pf)

    # Begin main loop & wait for exit conditions
    log('Listening...\n', file=sys.stdout)
    status = StatusTracker(100, lidar_ppm, imu_ppm)
    psutil.cpu_times_percent()
    try:
        while not (run_timer.timed_out() or lidar_timer.timed_out() or imu_timer.timed_out()):
            st = sensor.poll_client(cli)
            if st & sensor.ERROR:
                log('fatal: Ouster client encountered an error!\n')
                sys.exit(1)
            elif (st & sensor.LIDAR_DATA):
                if sensor.read_lidar_packet(cli, lidar_buf, pf):
                    lidar.handle_packet()
                    lidar_timer.restart()
                elif args.break_on_err:
                    log('fatal: Error encountered reading lidar data!')
                    sys.exit(1)
            elif (st & sensor.IMU_DATA):
                if sensor.read_imu_packet(cli, imu_buf, pf):
                    imu.handle_packet()
                    imu_timer.restart()
                elif args.break_on_err:
                    log('fatal: Error encountered reading IMU data!')
                    sys.exit(1)

            if not args.timeout and (st & sensor.LIDAR_DATA or st & sensor.IMU_DATA):
                status.print_status()

    except KeyboardInterrupt:
        pass

    # Print the run's statistics
    finally:
        n_err = lidar.ppm_err.n_tracked
        n_pkt = max(1, lidar.n_packets)
        log('LIDAR err PPM:\n\tmax: {:<.3f}\n\tnet: {:<.3f}\t({}/{})\n'.format(
            lidar.ppm_err.max_est, 1E6 * n_err/n_pkt, n_err, n_pkt), file=sys.stdout)

        n_err = imu.ppm_err.n_tracked
        n_pkt = max(1, imu.n_packets)
        log('IMU err PPM:\n\tmax: {:<.3f}\n\tnet: {:<.3f}\t({}/{})\n'.format(
            imu.ppm_err.max_est, 1E6 * n_err/n_pkt, n_err, n_pkt), file=sys.stdout)

        cpu_pct = psutil.cpu_times_percent()
        misc = round(sum([cpu_pct.nice, cpu_pct.iowait, cpu_pct.irq, cpu_pct.softirq, cpu_pct.steal,
                          cpu_pct.guest, cpu_pct.guest_nice]),1)
        log('CPU Usage:\n\tUser:\t{:>4}%\n\tSys:\t{:>4}%\n\tIdle:\t{:>4}%\n\tMisc:\t{:>4}%\n\tCurr:\t{:>4}%\n'.format(
            cpu_pct.user, cpu_pct.system, cpu_pct.idle, misc, status.cpu_pct), file=sys.stdout)
