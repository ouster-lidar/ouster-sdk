#!/usr/bin/env python3

import ouster._sensor as sensor
import ouster.lidardata as osl
import pickle
import numpy as np
import unittest

class TestSimpleVerification(unittest.TestCase):
    def setUp(self):
        # TODO: only works with gen1 data for now
        self._pf = sensor.get_format(sensor.default_sensor_info(sensor.MODE_1024x10))

        with open("ouster_lidardata_test.dat", 'rb') as f:
            self._data = pickle.load(f)
            self._os_data = osl.OsLidarData(self._data['data'], self._pf)

    def test_simple_px_range(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_pixel_range_view(),
            np.array(self._data['px_range_data'])))

    def test_simple_px_reflectivity(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_pixel_reflectivity_view(),
            np.array(self._data['px_reflectivity_data'])))

    def test_simple_px_signal(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_pixel_signal_view(),
            np.array(self._data['px_signal_data'])))
        
    def test_simple_px_noise(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_pixel_noise_view(),
            np.array(self._data['px_noise_data'])))
        
    def test_simple_px_timestamp(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_col_timestamp_view(),
            np.array(self._data['timestamp'])))
        
    def test_simple_px_measurement_id(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_col_measurement_id_view(),
            np.array(self._data['measurement_id'])))
        
    def test_simple_px_frame_id(self):
        self.assertTrue(np.array_equal(
            self._os_data.make_col_frame_id_view(),
            np.array(self._data['col_frame_id'])))
        
    def test_simple_px_valid(self):
        array = self._os_data.make_col_valid_view()
        self.assertEqual(len(array), self._pf.columns_per_packet)
        for item in array:
            self.assertEqual(item, 0xFFFFFFFF)

if __name__ == '__main__':
    unittest.main()

