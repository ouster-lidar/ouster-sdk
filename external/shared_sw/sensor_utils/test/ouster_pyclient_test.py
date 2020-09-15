#!/usr/bin/env python3

import ouster.client._sensor as sensor
import ouster.client.lidardata as osl
import numpy as np
import unittest
import gc
import time
import sys
import pytest
import os
import pickle
import socket
import sys

sys.path.append(os.path.dirname(__file__))
from ouster_pyclient_test_lib import ouster_sensor, ouster_pcap, osl, Pcap

class Wrapper:
    def __init__(self, data):
        self.data = data

# NOTE: Until we move away from winpcap this test uses a terrible hack located in the pcap code
class TestReceiveData(unittest.TestCase):
    @classmethod
    def setup_class(cls):
        with open("test-files/ouster_pyclient_test2.dat", 'rb') as f:
            cls._reference = pickle.load(f)
            
    def test_recieve_data(self):
        # We are grabbing the current local address from the system rather than just using 127.0.0.1
        self._test = Pcap("os1-991912000353", socket.gethostbyname(socket.gethostname()), 
                          os.path.join(os.path.dirname(__file__), "test-files/ouster_pyclient_test2.pcap"))
        self._test.start_listening(7000, 7001)
        time.sleep(5)
        print("Started pcap play")
        print(time.time())
        self._test.init_stepwise_replay(7000, 7001)
        self._test.start_stepwise_play(True)
        print("Stopped pcap play")
        print(time.time())
        time.sleep(5)
        self._test.stop_listening()
        time.sleep(5)
        self._test.pytest_compare(self._reference, self)
    # Commented out until we replace the pybuffers code        
    # def test_read_data(self):
    #     self._test = Pcap("os1-991912000353", socket.gethostbyname(socket.gethostname()), 
    #                       os.path.join(os.path.dirname(__file__), "test-files/ouster_pyclient_test2.pcap"))
    #     print("Started reading")
    #     print(time.time())
    #     self._test.init_stepwise_replay(7000, 7001)
    #     self._test.get_all_packets()
    #     print("Stopped reading")
    #     print(time.time())
    #     self._test.pytest_compare(self._reference, self)


if __name__ == '__main__':
    unittest.main()
