from typing import Optional
import os
import subprocess
import socket
import psutil
import ipaddress
import threading
import time
import json
import click
from datetime import datetime, timezone
from ouster.sdk.core import SensorInfo, OperatingMode, UDPProfileLidar, TimestampMode
from ouster.sdk._bindings.client import PacketType
from ouster.sdk.util.parsing import scan_to_packets
from ouster.sdk.core.io_types import OusterIoType, io_type
from ouster.cli.plugins.source_util import (SourceCommandContext,
                                            SourceCommandType,
                                            source_multicommand)
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.pcap import PcapPacketSource
from ouster.sdk.bag import BagPacketSource
from ouster.sdk.osf import OsfScanSource
from zeroconf import ServiceInfo, Zeroconf
from flask import Flask, request, jsonify, render_template


# TODO: fix
SENSOR_IDX = 0


def ts_format(ts):
    return datetime.fromtimestamp(ts * 1e-9, tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S.%f %Z')


app = Flask(__name__)


class SensorPacketSource():

    def __init__(self, source_url, cycle):
        self._source_url = source_url
        self._loop = cycle
        self._packet_source = self._init_pkt_src()

    def _init_pkt_src(self):
        from ouster.sdk._bindings.client import SensorPacketSource as SPS
        return SPS([(self._source_url)])

    @property
    def sensor_info(self):
        # HACK: quick prototype, store the value
        # TODO: we need to enable this for other sources
        from copy import deepcopy
        out = deepcopy(self._packet_source.sensor_info)
        out[0].config.udp_port_imu += 2
        out[0].config.udp_port_lidar += 2
        return out

    def __iter__(self):
        while True:
            try:
                for packet in self._packet_source:
                    yield packet
            except StopIteration:
                if self._loop:
                    self._packet_source = self._init_pkt_src()
                else:
                    break


class OsfPacketSource():
    """Converts OSF scans to packets."""
    def __init__(self, source_url, soft_id_check):
        if soft_id_check:
            click.secho("soft_id_check is not supported for OSF sources, flag is ignored!",
                        fg='yellow')
        self._source_url = source_url
        self._scan_source = self._init_pkt_src()

    @property
    def sensor_info(self):
        return self._scan_source.sensor_info

    def _init_pkt_src(self):
        return OsfScanSource(self._source_url)

    def __iter__(self):
        for scan, in self._scan_source:
            packets = scan_to_packets(scan, self.sensor_info[0])
            for packet in packets:
                yield 0, packet


class PacketSourcePacer():

    def __init__(self, packet_source_factory, rate, loop=False):
        self._packet_source_factory = packet_source_factory
        self._packet_source = packet_source_factory()
        self._rate = rate
        self._loop = loop
        self._restart_timestamp = None

    @property
    def sensor_info(self):
        return self._packet_source.sensor_info

    def _restart_source(self):
        """Restart the packet source and reset timing."""
        self._packet_source = self._packet_source_factory()
        self._restart_timestamp = time.monotonic()

    def check_and_clear_restart(self):
        """Check if a restart occurred recently and clear the flag."""
        if self._restart_timestamp is not None:
            restart_time = self._restart_timestamp
            self._restart_timestamp = None
            return restart_time
        return None

    def __iter__(self):
        while True:
            started = False
            pcap_start_time = None
            real_start_time = None

            try:
                for _, packet in self._packet_source:
                    if not started:
                        started = True
                        pcap_start_time = packet.host_timestamp
                        real_start_time = time.perf_counter()
                        # Yield the first packet immediately without delay
                        yield packet
                        continue

                    # Calculate the arrival time based on packet timestamp and rate
                    packet_time_offset = (packet.host_timestamp - pcap_start_time) * 1e-9
                    arrival_time = real_start_time + (packet_time_offset / self._rate)

                    # Sleep until real-time catches up with the packet time
                    while True:
                        current_time = time.perf_counter()
                        sleep_duration = arrival_time - current_time
                        if sleep_duration <= 0:
                            break
                        time.sleep(sleep_duration)

                    yield packet

                # If we reach here, the source has ended naturally
                if self._loop:
                    self._restart_source()
                else:
                    break

            except StopIteration:
                # Handle explicit StopIteration from the source
                if self._loop:
                    self._restart_source()
                else:
                    break


io_type_handlers = {
    OusterIoType.SENSOR: SensorPacketSource,    # TODO: maybe?
    OusterIoType.PCAP: PcapPacketSource,
    OusterIoType.OSF: OsfPacketSource,
    OusterIoType.BAG: BagPacketSource,
    OusterIoType.MCAP: BagPacketSource
}


class mDNSService:

    def __init__(self, sensor_info: SensorInfo):
        self._sensor_info = sensor_info
        service_type = "_roger._tcp.local."
        service_name = f"Ouster Sensor {sensor_info.sn}"
        ipv4_address = socket.inet_aton("127.0.0.1")
        # TODO: enable ipv6 interfaces
        # ipv6_address = socket.inet_pton(socket.AF_INET6, "::1")
        properties = {
            "sn": sensor_info.sn,
            "pn": sensor_info.prod_pn,
            "fw": sensor_info.image_rev
        }
        self._info = ServiceInfo(
            service_type,
            f"{service_name}.{service_type}",
            addresses=[ipv4_address],
            port=sensor_info.config.udp_port_lidar,
            properties=properties,
            server="localhost"
        )
        self._zeroconf = Zeroconf(interfaces=self._active_addresses())
        self.register()

    def _active_addresses(self):
        addresses = psutil.net_if_addrs()
        active_addresses = []
        for interface_name, interface_addresses in addresses.items():
            # Check if the interface is logically 'up'
            stats = psutil.net_if_stats().get(interface_name)
            if not stats or not stats.isup:
                continue
            for snicaddr in interface_addresses:
                address, family = snicaddr.address, snicaddr.family
                if family == socket.AF_INET:
                    # IPv4: Exclude loopback
                    if address != '127.0.0.1':
                        active_addresses.append(address)
                elif family == socket.AF_INET6:
                    # IPv6: Exclude loopback (::1)
                    if address == '::1':
                        continue
                    # Check if is a valid ipaddress
                    try:
                        _ = ipaddress.ip_address(address)
                        active_addresses.append(address)
                    except ValueError:
                        continue
        return active_addresses

    def register(self):
        self._zeroconf.register_service(self._info)
        print("mDNS started.")

    def unregister(self):
        self._zeroconf.unregister_service(self._info)
        self._zeroconf.close()
        print("mDNS stopped.")

    def __del__(self):
        self.unregister()


class HttpServer():

    def __init__(self, sensor, http_port):
        self._sensor = sensor
        self._http_port = http_port

        def _sensor_info_as_json(sensor_info):
            return json.loads(sensor_info.to_json_string())

        @app.route('/api/v1/sensor/metadata', methods=['GET'])
        def get_all_metadata():
            result = _sensor_info_as_json(self._sensor.sensor_info)
            return jsonify(result)

        @app.route('/api/v1/sensor/metadata/<metadata_type>', methods=['GET'])
        def get_sensor_metadata(metadata_type):
            sensor_info_json = _sensor_info_as_json(self._sensor.sensor_info)
            if metadata_type in sensor_info_json:
                return jsonify(sensor_info_json[metadata_type])
            return jsonify({"error": "Metadata not found"}), 404

        @app.route('/api/v1/sensor/config', methods=['GET', 'POST'])
        def sensor_config_handler():
            sensor_info_json = _sensor_info_as_json(self._sensor.sensor_info)
            if request.method == 'GET':
                result = sensor_info_json["config_params"]
                return jsonify(result)
            elif request.method == 'POST':
                sensor_config = {}
                try:
                    sensor_config = json.loads(request.data)
                except Exception:
                    return jsonify({"error": {
                        "title": "500 Internal Server Error",
                        "description": "Command \"set_config_param\" failed: error: Expected 1-2 arguments"}})
                self.configure_sensor(sensor_config, request.remote_addr)
                return jsonify({"message": "Configuration updated successfully"})
            elif request.method == 'DELETE':
                # sensor_config.clear()
                return jsonify({"message": "Configuration deleted successfully"})

        @app.route('/api/v1/sensor/cmd/get_config_param', methods=['GET'])
        def get_config_param():
            sensor_info_json = _sensor_info_as_json(self._sensor.sensor_info)
            config_parameters = {
                "active": sensor_info_json["config_params"],
                "staged": sensor_info_json["config_params"]
            }
            args = request.args.get('args')
            if args in config_parameters:
                return jsonify(config_parameters[args])
            else:
                return jsonify({"error": "Config parameter not found"}), 404

        @app.route('/api/v1/sensor/cmd/set_udp_dest_auto', methods=['GET'])
        def set_udp_dest_auto():
            self._sensor.sensor_info.config.udp_dest = request.remote_addr
            return "{}"

        @app.route('/api/v1/sensor/cmd/set_config_param', methods=['GET', 'POST', 'DELETE'])
        def set_config_param():
            # parse the lidar udp and imu udp from the request
            args = request.args.get('args')
            sensor_config = {}
            try:
                args = args[args.index('{'):]
                sensor_config = json.loads(args)
            except Exception:
                return jsonify({"error": {
                    "title": "500 Internal Server Error",
                    "description": "Command \"set_config_param\" failed: error: Expected 1-2 arguments"}})
            self.configure_sensor(sensor_config, request.remote_addr)
            return "\"set_config_param\""

        @app.route('/api/v1/user/data', methods=['GET', 'PUT', 'DELETE'])
        def user_data():
            if request.method == 'GET':
                user_data = self._sensor.sensor_info.user_data if self._sensor.sensor_info.user_data else "\"\""
                return user_data
            elif request.method == 'PUT':
                new_data = request.json
                self._sensor.sensor_info.user_data = new_data
                return new_data if new_data else "\"\""
            elif request.method == 'DELETE':
                user_data = self._sensor.sensor_info.user_data
                self._sensor.sensor_info.user_data = ""
                return user_data if user_data else "\"\""

        @app.route('/api/v1/sensor/cmd/reinitialize', methods=['GET'])
        def reinitialize():
            return "{}"

        @app.route('/api/v1/system/firmware', methods=['GET'])
        def get_firmware_version():
            sensor_info_json = _sensor_info_as_json(self._sensor.sensor_info)
            return jsonify({"fw": sensor_info_json["sensor_info"]["image_rev"]})

        # TODO: properly populate info from metadata
        @app.route('/api/v1/system/network', methods=['GET'])
        def get_network_info():
            sensor_info = self._sensor.sensor_info
            return jsonify({"duplex": "full",
                            "ipv6": {
                                "link_local": "fe80::be0f:a7ff:fe00:4521/64"},  # hardcoded for now
                            "speed": 1000, "ethaddr": "bc:0f:a7:00:45:21",      # hardcoded for now
                            "speed_override": "null",
                            "hostname": f"os-{sensor_info.sn}",
                            "carrier": "true",
                            "ipv4": {
                                "override": "null",
                                "link_local": f"{sensor_info.config.udp_dest}/16"}
                            })

        # TODO: Use packets from the source to populate timing values instead of realtime
        @app.route('/api/v1/time/system', methods=['GET'])
        def get_system_time():
            response = jsonify({
                "monotonic": time.monotonic(),
                "realtime": datetime.now().timestamp(),
            })
            return response

        @app.route('/api/v1/sensor/cmd/save_config_params', methods=['GET'])
        def save_config_params():
            print("Ignoring the save_config_params operation")
            # Note: does this activate the sensor?
            return "{}"

        @app.route('/api/v1/sensor/telemetry', methods=['GET'])
        def get_sensor_telemetry():
            response = jsonify({
                "phase_lock_status": "DISABLED",
                "input_voltage_mv": "23879",
                "input_current_ma": "693",
                "timestamp_ns": str(int(time.time() * 1e9)),
                "internal_temperature_deg_c": "60.0"
            })
            return response

        def tab_title(tab):
            return f"os-{self._sensor.sensor_info.sn} :: {tab} :: Ouster"

        @app.route('/')
        def dashboard():
            system_info = {
                'Ethernet Address': '00:00:00:00:00:00',
                'IPv4 (Link-Local)': '127.0.0.1/16',
                'IPv6 (Link-Local)': '[::1]/64',
                'Hostname': f"os-{self._sensor.sensor_info.sn}",
                'Serial Number': self._sensor.sensor_info.sn,
                'Part Number': self._sensor.sensor_info.prod_pn,
                'Model': self._sensor.sensor_info.prod_line
            }

            version = self._sensor.sensor_info.get_version()
            firmware_update = {
                'Current Image': self._sensor.sensor_info.image_rev,
                'Current Version': f'v{version.major}.{version.minor}.{version.patch}'
            }

            system_status = {
                'State': self._sensor.sensor_info.config.operating_mode.name,
                'Active Alerts': '0'
            }

            configuration = {
                'Lidar Mode': self._sensor.sensor_info.config.lidar_mode.name,
                'Signal Multiplier': self._sensor.sensor_info.config.signal_multiplier,
                'Active Azimuth Window': self._sensor.sensor_info.config.azimuth_window,
                'UDP Profile Lidar': self._sensor.sensor_info.config.udp_profile_lidar
            }

            return render_template('dashboard.html',
                                   title=tab_title("Dashboard"),
                                   system_info=system_info,
                                   firmware_update=firmware_update,
                                   system_status=system_status,
                                   configuration=configuration)

        @app.route('/diag')
        def diagnostics():
            active_alerts = []
            logged_alerts = []

            return render_template('diagnostics.html',
                                   title=tab_title("Diagnostics"),
                                   active_alerts=active_alerts,
                                   logged_alerts=logged_alerts)

        @app.route('/config')
        def configuration():
            config = self._sensor.sensor_info.config
            config_data = {
                'network': [
                    {
                        'name': 'udp_dest',
                        'label': 'UDP Destination Address',
                        'active': config.udp_dest,
                        'staged': config.udp_dest,
                        'type': 'text',
                        'button': 'Set Local'
                    },
                    {
                        'name': 'udp_port_lidar',
                        'label': 'UDP Port Lidar',
                        'active': config.udp_port_lidar,
                        'staged': config.udp_port_lidar,
                        'type': 'text',
                        'button': 'Reset to default'
                    },
                    {
                        'name': 'udp_port_imu',
                        'label': 'UDP Port IMU',
                        'active': config.udp_port_imu,
                        'staged': config.udp_port_imu,
                        'type': 'text',
                        'button': 'Reset to default'
                    }
                ],
                'mode': [
                    {
                        'name': 'lidar_mode',
                        'immutable': True,
                        'label': 'Lidar Mode',
                        'active': config.lidar_mode.name,
                        'staged': config.lidar_mode.name,
                        'type': 'dropdown',
                        'options': ['512x10', '512x20', '1024x10', '1024x20', '2048x10']
                    },
                    {
                        'name': 'operating_mode',
                        'immutable': False,
                        'label': 'Operating Mode',
                        'active': config.operating_mode.name,
                        'staged': config.operating_mode.name,
                        'type': 'dropdown',
                        'options': ['NORMAL', 'STANDBY']
                    },
                    {
                        'name': 'azimuth_window',
                        'immutable': True,
                        'label': 'Azimuth Window',
                        'active_start': config.azimuth_window[0],
                        'active_end': config.azimuth_window[1],
                        'staged_start': config.azimuth_window[0],
                        'staged_end': config.azimuth_window[1],
                        'type': 'range'
                    },
                    {
                        'name': 'signal_multiplier',
                        'immutable': True,
                        'label': 'Signal Multiplier',
                        'active': config.signal_multiplier,
                        'staged': config.signal_multiplier,
                        'type': 'dropdown',
                        'options': ['1', '2', '3']
                    },
                    {
                        'name': 'udp_profile_lidar',
                        'immutable': True,
                        'label': 'UDP Profile Lidar',
                        'active': config.udp_profile_lidar.name,
                        'staged': config.udp_profile_lidar.name,
                        'type': 'dropdown',
                        'options': [profile.name for profile in UDPProfileLidar.values]
                    }
                ],
                'timing': [
                    {
                        'immutable': True,
                        'label': 'Timestamp Mode',
                        'active': config.timestamp_mode.name,
                        'staged': config.timestamp_mode.name,
                        'type': 'dropdown',
                        'options': [time_mode.name for time_mode in TimestampMode.values]
                    },
                    {
                        'immutable': True,
                        'label': 'Multipurpose IO Mode',
                        'active': config.multipurpose_io_mode,
                        'staged': config.multipurpose_io_mode,
                        'type': 'dropdown',
                        'options': ['OFF', 'INPUT_NMEA_UART', 'OUTPUT_*']
                    },
                    {
                        'immutable': True,
                        'label': 'NMEA In Polarity',
                        'active': config.nmea_in_polarity,
                        'staged': config.nmea_in_polarity,
                        'type': 'dropdown',
                        'options': ['ACTIVE_HIGH', 'ACTIVE_LOW']
                    },
                    {
                        'immutable': True,
                        'label': 'NMEA Ignore Valid Char',
                        'active': config.nmea_ignore_valid_char,
                        'staged': config.nmea_ignore_valid_char,
                        'type': 'checkbox'
                    },
                    {
                        'immutable': True,
                        'label': 'NMEA Baud Rate',
                        'active': config.nmea_baud_rate,
                        'staged': config.nmea_baud_rate,
                        'type': 'dropdown',
                        'options': ['BAUD_9600', 'BAUD_115200']
                    },
                    {
                        'immutable': True,
                        'label': 'NMEA Leap Seconds',
                        'active': config.nmea_leap_seconds,
                        'staged': config.nmea_leap_seconds,
                        'type': 'text'
                    },
                    {
                        'immutable': True,
                        'label': 'Sync Pulse In Polarity',
                        'active': config.sync_pulse_in_polarity,
                        'staged': config.sync_pulse_in_polarity,
                        'type': 'dropdown',
                        'options': ['ACTIVE_HIGH', 'ACTIVE_LOW']
                    },
                    {
                        'immutable': True,
                        'label': 'Sync Pulse Out Polarity',
                        'active': config.sync_pulse_out_polarity,
                        'staged': config.sync_pulse_out_polarity,
                        'type': 'dropdown',
                        'options': ['ACTIVE_HIGH', 'ACTIVE_LOW']
                    },
                    {
                        'immutable': True,
                        'label': 'Sync Pulse Out Frequency',
                        'active': config.sync_pulse_out_frequency,
                        'staged': config.sync_pulse_out_frequency,
                        'type': 'text'
                    },
                    {
                        'immutable': True,
                        'label': 'Sync Pulse Out Angle',
                        'active': config.sync_pulse_out_angle,
                        'staged': config.sync_pulse_out_angle,
                        'type': 'text'
                    },
                    {
                        'immutable': True,
                        'label': 'Sync Pulse Out Pulse Width',
                        'active': config.sync_pulse_out_pulse_width,
                        'staged': config.sync_pulse_out_pulse_width,
                        'type': 'text'
                    },
                    {
                        'immutable': True,
                        'label': 'Phase Lock Enable',
                        'active': config.phase_lock_enable,
                        'staged': config.phase_lock_enable,
                        'type': 'checkbox'
                    },
                    {
                        'immutable': True,
                        'label': 'Phase Lock Offset',
                        'active': config.phase_lock_offset,
                        'staged': config.phase_lock_offset,
                        'type': 'text'
                    }
                ]
            }

            return render_template('configuration.html',
                                   title=tab_title("Configuration"),
                                   config=config_data)

        @app.route('/apply_config', methods=['POST'])
        def apply_config():
            # read udp dest, lidar port, imu port from the form and apply them
            udp_dest = request.form.get('udp_dest')
            if udp_dest:
                self._sensor.sensor_info.config.udp_dest = udp_dest
            lidar_port = request.form.get('udp_port_lidar')
            if lidar_port:
                self._sensor.sensor_info.config.udp_port_lidar = int(lidar_port)
            imu_port = request.form.get('udp_port_imu')
            if imu_port:
                self._sensor.sensor_info.config.udp_port_imu = int(imu_port)
            operating_mode = request.form.get('operating_mode')
            if operating_mode:
                self._sensor.sensor_info.config.operating_mode = OperatingMode.from_string(operating_mode)
            # TODO: display the confirmation message
            return configuration()

        @app.route('/disclaimer')
        def disclaimer():
            return render_template('disclaimer.html',
                                   title=tab_title("Disclaimer"))

        # START THE HTTP SERVER
        self.start()

    def start(self):
        self._service_status = {"status": 0}

        def serve_flask_app(http_port, service_status):
            from waitress import serve
            try:
                serve(app, host="localhost", port=http_port)
            except PermissionError:
                click.secho("Permission error!",
                            fg='red')
                service_status['status'] = -1
            except Exception as e:
                print("Unknown error: " + str(e))
                service_status['status'] = -2

        self._flask_thread = threading.Thread(target=serve_flask_app,
                                              args=(self._http_port, self._service_status,),
                                              daemon=True)

        self._flask_thread.start()
        time.sleep(0.5)
        if "status" in self._service_status and self._service_status["status"] != 0:
            raise RuntimeError("http server failed to start")

        print("http server started")

    def configure_sensor(self, sensor_config, remote_addr):
        if 'udp_dest' in sensor_config:
            udp_dest = remote_addr if sensor_config['udp_dest'] == "@auto" else sensor_config['udp_dest']
            self._sensor.sensor_info.config.udp_dest = udp_dest
        if 'udp_port_lidar' in sensor_config:
            self._sensor.sensor_info.config.udp_port_lidar = sensor_config['udp_port_lidar']
        if 'udp_port_imu' in sensor_config:
            self._sensor.sensor_info.config.udp_port_imu = sensor_config['udp_port_imu']
        if 'operating_mode' in sensor_config:
            self._sensor.sensor_info.config.operating_mode = OperatingMode.from_string(sensor_config['operating_mode'])
        if 'user_data' in sensor_config:
            self._sensor.sensor_info.user_data = sensor_config['user_data']

    def stop(self):
        self._flask_thread.join()
        print("http server stopped")


class ScanSourceUdpReplay():

    def __init__(self, source_url, http_port, loop, rate, soft_id_check, lidar_port=-1,
                 imu_port=-1, udp_dest="127.0.0.1", operating_mode="NORMAL", sensor_sn="",
                 hide_diagnostics=False):

        source_type = io_type(source_url)
        pkt_src_handler = io_type_handlers[source_type]

        # Create a factory function that produces packet sources
        def packet_source_factory():
            return pkt_src_handler(source_url, soft_id_check=soft_id_check)

        self._packet_source = PacketSourcePacer(
                                packet_source_factory,
                                rate=rate,
                                loop=loop)

        self._sensor_info = self._packet_source.sensor_info[SENSOR_IDX]
        # override ports before starting the HTTP server & DNS service
        if lidar_port != -1:
            self._sensor_info.config.udp_port_lidar = lidar_port
        if imu_port != -1:
            self._sensor_info.config.udp_port_imu = imu_port
        # should we override the current udp_dest? by default ?
        self._sensor_info.config.udp_dest = udp_dest
        self._sensor_info.sn = sensor_sn if sensor_sn != "" else self._sensor_info.sn
        self._sensor_info.config.operating_mode = OperatingMode.from_string(operating_mode)

        self._show_diagnostics = not hide_diagnostics
        self._first_diagnostics_message = True

        self._http_server = HttpServer(self, http_port)
        self._mdns = mDNSService(self._sensor_info)
        self.start()

    @property
    def sensor_info(self) -> SensorInfo:
        return self._sensor_info

    def display_diagnostics(self, lines):
        def clear_diagnostics(line_count):
            for _ in range(line_count):
                print('\033[1A\033[2K', end='')

        if self._first_diagnostics_message:
            self._first_diagnostics_message = False
        else:
            clear_diagnostics(len(lines) + 2)
        print("===== Sensor Replay Diagnostics =====")
        diagnostics_msg = '\n*  '.join(lines)
        print(f"*  {diagnostics_msg}")
        print("=====================================")

    def start(self):
        print("starting udp streaming")
        initialized = False
        sock_lidar = 0
        sock_imu = 0
        while not initialized:
            try:
                sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print("sockets initialized")
                initialized = True
            except ConnectionRefusedError:
                print("connection refused")
                time.sleep(0.1)

        # diagnostics
        pkts_count = 0
        last_time = time.monotonic()

        for packet in self._packet_source:
            # Check if a restart occurred and reset timing measurements if so
            restart_time = self._packet_source.check_and_clear_restart()
            if restart_time is not None:
                # Reset the timing measurements to account for restart
                pkts_count = 0
                last_time = time.monotonic()

            # grab current config in case it was changed via HTTP
            operating_mode = self._sensor_info.config.operating_mode
            udp_dest = self._sensor_info.config.udp_dest
            udp_port_lidar = self._sensor_info.config.udp_port_lidar
            udp_port_imu = self._sensor_info.config.udp_port_imu

            if operating_mode == OperatingMode.OPERATING_NORMAL:
                if packet.type == PacketType.Lidar:
                    sock_lidar.sendto(
                        packet.buf, (udp_dest, udp_port_lidar))
                elif packet.type == PacketType.Imu:
                    sock_imu.sendto(
                        packet.buf, (udp_dest, udp_port_imu))
                else:
                    print(f"Unknown packet type {packet.type()}")

            # diagnostics
            if self._show_diagnostics:
                pkts_count += 1
                curr_time = time.monotonic()
                if curr_time - last_time > 1:
                    self.display_diagnostics(
                        [f"Operating Mode: {operating_mode}",
                        f"UDP Destination: {udp_dest}",
                        f"UDP Port (Lidar): {udp_port_lidar}",
                        f"UDP Port (IMU): {udp_port_imu}",
                        f"Host Time: {ts_format(packet.host_timestamp)}",
                        f"{pkts_count} packets/seconds"])
                    pkts_count = 0
                    last_time = curr_time

        sock_lidar.close()
        sock_imu.close()


def get_docker_image_creation_date(image_name):
    try:
        result = subprocess.run(
            ["docker", "inspect", "--format='{{.Created}}'", image_name],
            capture_output=True, text=True, check=True)
        creation_date = result.stdout.strip()
        return creation_date
    except subprocess.CalledProcessError as e:
        if e.returncode == 1:
            return None     # Image not found.
        else:
            click.secho(f"Docker command failed: {e}", fg='red')
            return None
    except FileNotFoundError:
        click.secho("Docker command not found. Is Docker installed?", fg='red')
        return None
    except Exception as e:
        click.secho(f"An unexpected error occurred: {e}", fg='red')
        return None


def build_ouster_cli_sensor_replay_image(ouster_sdk: str):
    from ouster.sdk import __version__
    package_version = __version__

    try:
        import sys
        if sys.version_info < (3, 9):
            import importlib_resources as resources     # type: ignore
        else:
            import importlib.resources as resources

        dockerfile_path = resources.files('ouster.cli') / 'plugins' / 'sensor_replay_dockerfile'

        docker_build_cmd = [
            "docker", "build",
            "-t", "ouster-cli-sensor-replay",
            "--file", str(dockerfile_path)]

        build_target = ""
        if os.path.exists(ouster_sdk) and os.path.isdir(ouster_sdk):
            build_target = "ouster-sdk-local"
        else:
            # if ouster_sdk is not a path and not empty str assume it is a version number
            # TODO: when ouster_sdk is not empty check it represnts a valid version
            ouster_sdk = f"ouster-sdk=={ouster_sdk}" if ouster_sdk else \
                f"ouster-sdk=={package_version}"
            build_target = "ouster-sdk-public"

        docker_build_cmd += [
            "--build-arg", f"OUSTER_SDK={ouster_sdk}",
            "--target", f"{build_target}",
            "."
        ]

        print("docker build command:", " ".join(docker_build_cmd))

        return subprocess.run(docker_build_cmd, check=True)
    except subprocess.CalledProcessError as e:
        click.secho(f"Docker command failed: {e}", fg='red')
        return None
    except FileNotFoundError:
        click.secho("Docker command not found. Is Docker installed?", fg='red')
        return None
    except Exception as e:
        click.secho(f"An unexpected error occurred: {e}", fg='red')
        return None


@click.command
@click.option('-d', '--dockerize', is_flag=True, default=False, show_default=True,
              help="Run the sensor replay inside a Docker container")
@click.option('-p', '--http-port', type=int, default=80, show_default=True,
              help="Change the port used by the sensor HTTP server")
@click.option('--lidar-port', type=int, default=-1, show_default=True,
              help="lidar port")
@click.option('--imu-port', type=int, default=-1, show_default=True,
              help="imu port")
@click.option('--loop', type=bool, default=True, show_default=True,
              help="When playback reaches the end of the file, restart from the begnning")
@click.option('--rate', type=float, default="1.0", show_default=True,
              help="udp packets replay rate.")
@click.option('--ouster-sdk', type=str, default="",
              hidden=True, help="Use local SDK build (hidden option)")
@click.option('--udp-dest', type=str, default="127.0.0.1", show_default=True,
              help="The destination IP to send the replayed packets to.")
@click.option('--operating-mode', type=click.Choice(["NORMAL", "STANDBY"]),
              default="NORMAL", show_default=True, help="Set sensor Operating Mode on startup.")
@click.option('--sensor-sn', type=str, default="", show_default=True,
              help="Override Sensor SN.")
@click.option('--hide-diagnostics', is_flag=True, default=False, show_default=True,
              help="Hide diagnostic information.")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def source_sensor_replay(ctx: SourceCommandContext, dockerize: bool, http_port: int,
                         lidar_port: Optional[int], imu_port: Optional[int],
                         loop: bool, rate: float, ouster_sdk: str, udp_dest: str,
                         operating_mode: str, sensor_sn: str, hide_diagnostics: bool) -> None:
    """
    [BETA] Replay PCAP|BAG|OSF|MCAP as a Sensor
    """

    soft_id_check = ctx.source_options.get("soft_id_check", False)

    if not dockerize:
        try:
            ScanSourceUdpReplay(ctx.source_uri, http_port, loop, rate, soft_id_check, lidar_port, imu_port,
                                udp_dest, operating_mode, sensor_sn, hide_diagnostics)
        except RuntimeError:
            click.secho("Failed to start the service, this is likely due trying to run with the default"
                        " http port (80) and not having the necessary permissions, to resolve this try"
                        " one of the following options:\n"
                        "1) set the --http-port to a value higher than 1024 in combination with one of the following:\n"
                        "   a) (preferred) Set the environment varialbe 'http_proxy' to http://localhost:port (for port"
                        " use the same value you used as the http-port) then launch the client app.\n"
                        "   b) If the above option fail you can use a reverse proxy such as ngix or similar to re-route"
                        " the selected http-port value to port 80 and then launch the client app.\n"
                        "2) If you have docker installed and have followed the linux postinstall guide described in:"
                        " https://docs.docker.com/engine/install/linux-postinstall/ then you may simply pass the option"
                        " --dockerize  to the 'sensor_replay' command, follow the prompts to spin the service in a"
                        " container. Please note that this option isn't compatible with rootless docker mode.",
                        fg="yellow")
        return

    if os.path.exists(ouster_sdk) and \
        (os.path.isabs(ouster_sdk) or not os.path.exists(os.path.join(os.getcwd(), ouster_sdk))):
        click.secho("When using the --ouster-sdk option as a path you should use a relative"
                    " path to the build context", fg='red')
        return

    if get_docker_image_creation_date("ouster-cli-sensor-replay") is None:
        # TODO: check if the date of docker image creation and compare with
        # the modified date of the associated dockerfile, if found that the
        # dockerfile is newer than the image, then prompt to rebuild.
        # helpful for development.
        click.secho("The dockerize flag is set but the docker image is not found."
                    " To use this feature we need to build the assoicated docker image first."
                    " Build the image now (Y/n)?", fg='yellow')
        choice = input("")
        if choice == 'Y':
            build_ouster_cli_sensor_replay_image(ouster_sdk)
            click.secho("Done building the docker image", fg='green')
        else:
            exit(0)

    command = ["docker", "run", "--rm", "--network", "host"]

    source_url = ctx.source_uri or ""
    if io_type(source_url) != OusterIoType.SENSOR:
        abs_source_url = os.path.abspath(source_url)
        abs_dir_url = os.path.dirname(abs_source_url)
        source_name = os.path.basename(source_url)
        source_url = f"/data/{source_name}"
        command += ["--mount", "type=bind,source=" + abs_dir_url + ",target=/data"]

    command += ["ouster-cli-sensor-replay",
                "ouster-cli",
                "source", source_url,
                "sensor_replay",
                "--http-port", str(http_port),
                "--lidar-port", str(lidar_port),
                "--imu-port", str(imu_port),
                "--loop", str(loop),
                "--rate", str(rate)]

    process = subprocess.Popen(command)
    process.wait()


source.commands[OusterIoType.PCAP]['sensor_replay'] = source_sensor_replay
source.commands[OusterIoType.BAG]['sensor_replay'] = source_sensor_replay
source.commands[OusterIoType.MCAP]['sensor_replay'] = source_sensor_replay
source.commands[OusterIoType.OSF]['sensor_replay'] = source_sensor_replay
# source.commands[OusterIoType.SENSOR]['sensor_replay'] = source_sensor_replay  # not supported for now
