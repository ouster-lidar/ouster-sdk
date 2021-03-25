from ouster import client
import ouster.client.data as osl
import pickle
import numpy as np
from os import path
import unittest

# use paths relative to this file to find test data
DATA_DIR = path.dirname(path.abspath(__file__))


class TestSimpleVerification(unittest.TestCase):
    def setUp(self):
        # TODO: only works with gen1 data for now
        self._pf = client.PacketFormat(
            client.SensorInfo.from_default(client.LidarMode.MODE_1024x10))

        with open(path.join(DATA_DIR, "ouster_lidardata_test.dat"), 'rb') as f:
            self._data = pickle.load(f)
            self._os_data = osl.LidarPacket(self._data['data'], self._pf)

    def test_simple_px_range(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ChanField.RANGE),
            np.array(self._data['px_range_data'])))

    def test_simple_px_reflectivity(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ChanField.REFLECTIVITY),
            np.array(self._data['px_reflectivity_data'])))

    def test_simple_px_signal(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ChanField.INTENSITY),
            np.array(self._data['px_signal_data'])))

    def test_simple_px_noise(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ChanField.AMBIENT),
            np.array(self._data['px_noise_data'])))

    def test_simple_px_timestamp(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ColHeader.TIMESTAMP),
            np.array(self._data['timestamp'])))

    def test_simple_px_measurement_id(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ColHeader.MEASUREMENT_ID),
            np.array(self._data['measurement_id'])))

    def test_simple_px_frame_id(self):
        self.assertTrue(np.array_equal(
            self._os_data.view(osl.ColHeader.FRAME_ID),
            np.array(self._data['col_frame_id'])))

    def test_simple_px_valid(self):
        array = self._os_data.view(osl.ColHeader.VALID)
        self.assertEqual(len(array), self._pf.columns_per_packet)
        for item in array:
            self.assertEqual(item, 0xFFFFFFFF)


if __name__ == '__main__':
    unittest.main()
