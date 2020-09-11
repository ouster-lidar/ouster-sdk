#!/usr/bin/python3

import argparse
import socket
import sys
import os
from datetime import datetime
import time
import tempfile
import shutil
import threading
import pickle

sys.path.append(os.path.dirname(__file__))
from ouster_pyclient_test_lib import ouster_sensor, ouster_pcap, osl, Pcap

parser = argparse.ArgumentParser(
    description=
    'Make the simple ouster_pyclient test file')

parser.add_argument('hostname', type=str, help="the hostname that the pcap was recorded with")
parser.add_argument('cap_file', type=str, help="the pcap file")
parser.add_argument('file_out', type=str, help="file to save the decode pickle as")

args = parser.parse_args()     

p = Pcap(args.hostname, "127.0.0.1", args.cap_file)

p.start_listening(40070, 56195)
time.sleep(5)
p.play_pcap(True, 0.5)
time.sleep(5)
p.stop_listening()

with open(args.file_out, "wb") as f:
    pickle.dump(p, f)
