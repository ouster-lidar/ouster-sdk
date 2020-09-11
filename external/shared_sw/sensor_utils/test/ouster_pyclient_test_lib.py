import argparse
import socket
import sys
import os
from datetime import datetime
import time
import tempfile
import shutil
import threading
import numpy as np

import ouster._sensor as ouster_sensor
import ouster._pcap as ouster_pcap
import ouster.lidardata as osl

class Pcap:
    def __init__(self, hostname, dest_ip, path):
        # TODO: only works with gen1 data for now
        self._pf = ouster_sensor.get_format(ouster_sensor.default_sensor_info(ouster_sensor.MODE_1024x10))

        self._hostname = hostname
        self._dest_ip = dest_ip
        self._path = path
        self._playback_thread = None
        self._playback_thread_lock = threading.Lock()
        self._playback_thread_imu = None
        self._playback_thread_imu_lock = threading.Lock()
        self._listener_run = False
        self._listener_thread = None
        self._listener_thread_lock = threading.Lock()
        self._listener_new_lidar_data = False
        self._listener_new_lidar_data_mutex = threading.Lock()
        self._listener_new_lidar_data_cv = threading.Condition(self._listener_new_lidar_data_mutex)
        self._listener_new_imu_data = False
        self._listener_new_imu_data_mutex = threading.Lock()
        self._listener_new_imu_data_cv = threading.Condition(self._listener_new_imu_data_mutex)
        self._lidar_buf = bytearray(self._pf.lidar_packet_size+1)
        self._imu_buf = bytearray(self._pf.imu_packet_size+1)
        self._os_data = osl.OsLidarData(self._lidar_buf, self._pf)
        self._cli = None
        self._data_lock = threading.Lock()
        self._reset_data()
        self._run = False
        self._stepwise_pcap_handle = None
        
        
    def __getstate__(self):
        return {
            'hostname': self._hostname,
            'dest_ip': self._dest_ip,
            'path': self._path,
            'data': self._data}

    def __setstate__(self, state):
        # TODO: only works with gen1 data for now
        self._pf = ouster_sensor.get_format(ouster_sensor.default_sensor_info(ouster_sensor.MODE_1024x10))

        self._hostname = state['hostname']
        self._dest_ip = state['dest_ip']
        self._path = state['path']
        self._data = state['data']
        self._playback_thread = None
        self._playback_thread_lock = threading.Lock()
        self._playback_thread_imu = None
        self._playback_thread_imu_lock = threading.Lock()
        self._listener_run = False
        self._listener_thread = None
        self._listener_thread_lock = threading.Lock()
        self._listener_new_lidar_data = False
        self._listener_new_lidar_data_mutex = threading.Lock()
        self._listener_new_lidar_data_cv = threading.Condition(self._listener_new_lidar_data_mutex)
        self._listener_new_imu_data = False
        self._listener_new_imu_data_mutex = threading.Lock()
        self._listener_new_imu_data_cv = threading.Condition(self._listener_new_lidar_data_mutex)
        self._lidar_buf = bytearray(self._pf.lidar_packet_size+1)
        self._imu_buf = bytearray(self._pf.imu_packet_size+1)
        self._os_data = osl.OsLidarData(self._lidar_buf, self._pf)
        self._cli = None
        self._data_lock = threading.Lock()
        self._run = False
        self._stepwise_pcap_handle = None
        
    def _reset_data(self):
        with self._data_lock:
            self._data = {
                'errors': 0,
                'lidar_packets': [],
                'imu_packets': []}

    def _play_pcap(self, rate=0.5):
        try:
            ouster_pcap.replay_pcap(self._path, self._dest_ip, self._dest_ip, rate)

        except Exception as e:
            print(e)
            sys.exit(1)

        finally:
            with self._playback_thread_lock:
                self._playback_thread = None

    def play_pcap(self, blocking=False, rate=0.5):
        if self._path != None:
            if not blocking:
                with self._playback_thread_lock:
                    if self._playback_thread == None:
                       self._playback_thread = threading.Thread(
                           target=self._play_pcap)
                       self._playback_thread.setDaemon(True)
                       self._playback_thread.start()
            else:
                self._play_pcap()
                
    def init_stepwise_replay(self, dst_lidar_port, dst_imu_port):
        if self._stepwise_pcap_handle == None:
            self._pcap_info = ouster_pcap.replay_get_pcap_info(self._path, 10000)
            self._pcap_port_guess = ouster_pcap.guess_ports(self._pcap_info)
            self._stepwise_pcap_handle = ouster_pcap.replay_initalize(self._path,
                                                                      self._dest_ip,
                                                                      self._dest_ip,
                                                                      self._pcap_port_guess,
                                                                      dst_lidar_port, dst_imu_port)
            
        else:
            raise Exception("stepwise replay already initialized")
        
    def _start_stepwise_lidar_play(self):
        if self._stepwise_pcap_handle == None:
            raise Exception("Stepwise replay not initialized")
        else:
            sent = True
            while sent:
                sent = ouster_pcap.replay_next_lidar_packet(self._stepwise_pcap_handle)
                timeout = 100
                if sent:
                    with self._listener_new_lidar_data_cv:
                        while not self._listener_new_lidar_data:
                            if timeout <= 0:
                                self._listener_new_lidar_data = False
                                break
                            timeout -= 1
                            self._listener_new_lidar_data_cv.wait(0.01)
                        self._listener_new_lidar_data = False
    
    def _start_stepwise_imu_play(self):
        if self._stepwise_pcap_handle == None:
            raise Exception("Stepwise replay not initialized")
        else:
            sent = True
            while sent:
                sent = ouster_pcap.replay_next_imu_packet(self._stepwise_pcap_handle)
                timeout = 100
                if sent:
                    with self._listener_new_imu_data_cv:
                        while not self._listener_new_imu_data:
                            if timeout <= 0:
                                self._listener_new_imu_data = False
                                break
                            timeout -= 1
                            self._listener_new_imu_data_cv.wait(0.01)
                        self._listener_new_imu_data = False
    # Commented out until we replace the pybuffers code  
    # def get_all_packets(self):
    #     if self._stepwise_pcap_handle == None:
    #         raise Exception("Stepwise replay not initialized")
    #     else:
    #         has_next = True
    #         while has_next: 
    #             if (ouster_pcap.get_next_lidar_data(self._stepwise_pcap_handle, self._lidar_buf)):
    #                 self._data['lidar_packets'].append(np.array(self._lidar_buf, copy=True))
    #             else:
    #                 has_next = False
    #         has_next = True
    #         while has_next: 
    #             if (ouster_pcap.get_next_imu_data(self._stepwise_pcap_handle, self._imu_buf)):
    #                 self._data['imu_packets'].append(np.array(self._imu_buf, copy=True))
    #             else:
    #                 has_next = False
                    
    def start_stepwise_play(self, blocking=False):
        if self._stepwise_pcap_handle == None:
            raise Exception("Stepwise replay not initialized")
        else:
            if self._path != None:
                with self._playback_thread_lock:
                    if self._playback_thread == None:
                        self._playback_thread = threading.Thread(
                            target=self._start_stepwise_lidar_play)
                        self._playback_thread.setDaemon(True)
                        self._playback_thread.start()
                    if self._playback_thread_imu == None:
                        self._playback_thread_imu = threading.Thread(
                            target=self._start_stepwise_imu_play)
                        self._playback_thread_imu.setDaemon(True)
                        self._playback_thread_imu.start()

            if blocking:
                self._playback_thread.join()
                self._playback_thread_imu.join()
        
    def _listen_thread(self):
        run = False
        with self._listener_thread_lock:
            run = self._run
        while run:
            st = ouster_sensor.poll_client(self._cli)
            if st & ouster_sensor.ERROR:
                with self._listener_thread_lock:
                    self._run = False
                with self._data_lock:
                    self._data['errors'] += 1
            elif st & ouster_sensor.LIDAR_DATA:
                if (ouster_sensor.read_lidar_packet(self._cli, self._lidar_buf, self._pf)):
                    self._data['lidar_packets'].append(np.array(self._lidar_buf, copy=True))
                    with self._listener_new_lidar_data_cv:
                        try:
                            self._listener_new_lidar_data_cv.notifyAll()
                        except:
                            pass
                        self._listener_new_lidar_data = True
            elif st & ouster_sensor.IMU_DATA:
                if (ouster_sensor.read_imu_packet(self._cli, self._imu_buf, self._pf)):
                    self._data['imu_packets'].append(np.array(self._lidar_buf, copy=True))
                    with self._listener_new_lidar_data_cv:
                        try:
                            self._listener_new_imu_data_cv.notifyAll()
                        except:
                            pass
                        self._listener_new_imu_data = True

            with self._listener_thread_lock:
                run = self._run

    def _get_timestamp_key(self, packet):
        return osl.OsLidarData(packet, self._pf).make_col_timestamp_view()[0]

    def start_listening(self, lidar_port, imu_port):
        if not self._run:
            self._run = True
            with self._listener_thread_lock:
                if self._cli == None:
                    self._cli = ouster_sensor.init_client(self._hostname, int(lidar_port), int(imu_port))
                if self._listener_thread == None:
                   self._reset_data()
                   self._listener_thread = threading.Thread(
                       target=self._listen_thread)
                   self._listener_thread.setDaemon(True)
                   self._listener_thread.start()

    def stop_listening(self):
        with self._listener_thread_lock:
            if self._run:
                self._run = False

        if self._listener_thread != None:
            self._listener_thread.join()
            self._listener_thread = None
        temp_lidar = self._data['lidar_packets']
        temp_imu = self._data['imu_packets']
        self._data['lidar_packets'] = sorted(temp_lidar, key=lambda packet: self._get_timestamp_key(packet))
        self._data['imu_packets'] = sorted(temp_imu, key=lambda packet: self._get_timestamp_key(packet))

    def pytest_compare(self, other, test_object):
        with self._data_lock:
            test_object.assertEqual(self._data['errors'], other._data['errors'])
            test_object.assertEqual(len(self._data['lidar_packets']), len(other._data['lidar_packets']))
            test_object.assertEqual(len(self._data['imu_packets']), len(other._data['imu_packets']))

            a = self._data['lidar_packets']
            b = other._data['lidar_packets']
            for i in range(0, len(a)):
                a_data = osl.OsLidarData(a[i], self._pf)
                b_data = osl.OsLidarData(b[i], self._pf)

                a_subdata = a_data.make_col_measurement_id_view()
                b_subdata = b_data.make_col_measurement_id_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(a_subdata[x], a_subdata[x])

                a_subdata = a_data.make_col_timestamp_view()
                b_subdata = b_data.make_col_timestamp_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(a_subdata[x], a_subdata[x])

                a_subdata = a_data.make_col_frame_id_view()
                b_subdata = b_data.make_col_frame_id_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(a_subdata[x], a_subdata[x])

                a_subdata = a_data.make_pixel_range_view()
                b_subdata = b_data.make_pixel_range_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(len(a_subdata[x]), len(b_subdata[x]))
                    for y in range(0, len(a_subdata[x])):
                        test_object.assertEqual(a_subdata[x][y], a_subdata[x][y])

                a_subdata = a_data.make_pixel_reflectivity_view()
                b_subdata = b_data.make_pixel_reflectivity_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(len(a_subdata[x]), len(b_subdata[x]))
                    for y in range(0, len(a_subdata[x])):
                        test_object.assertEqual(a_subdata[x][y], a_subdata[x][y])

                a_subdata = a_data.make_pixel_signal_view()
                b_subdata = b_data.make_pixel_signal_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(len(a_subdata[x]), len(b_subdata[x]))
                    for y in range(0, len(a_subdata[x])):
                        test_object.assertEqual(a_subdata[x][y], a_subdata[x][y])

                a_subdata = a_data.make_pixel_noise_view()
                b_subdata = b_data.make_pixel_noise_view()
                test_object.assertEqual(len(a_subdata), len(b_subdata))
                for x in range(0, len(a_subdata)):
                    test_object.assertEqual(len(a_subdata[x]), len(b_subdata[x]))
                    for y in range(0, len(a_subdata[x])):
                        test_object.assertEqual(a_subdata[x][y], a_subdata[x][y])

