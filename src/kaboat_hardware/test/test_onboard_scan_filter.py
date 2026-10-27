"""Check that the boat-structure sector never reaches public /scan."""

import math
import unittest

from sensor_msgs.msg import LaserScan

from kaboat_hardware.onboard_scan_filter import mask_sector


class TestOnboardScanFilter(unittest.TestCase):
    def test_220_to_320_in_published_scan_frame(self):
        raw = LaserScan()
        raw.header.frame_id = 'onboard_laser_frame'
        raw.angle_min = -math.pi
        raw.angle_increment = math.radians(1.0)
        raw.ranges = [2.0] * 361
        raw.intensities = [9.0] * 361

        filtered = mask_sector(raw, 220.0, 320.0)
        for bearing in (219, 220, 270, 320, 321):
            index = (bearing + 180) % 360
            if 220 <= bearing <= 320:
                self.assertTrue(math.isinf(filtered.ranges[index]))
                self.assertEqual(filtered.intensities[index], 0.0)
            else:
                self.assertEqual(filtered.ranges[index], 2.0)
                self.assertEqual(filtered.intensities[index], 9.0)

        self.assertEqual(raw.ranges[index], 2.0)
        self.assertEqual(raw.intensities[index], 9.0)
        self.assertEqual(filtered.header.frame_id, raw.header.frame_id)
        self.assertEqual(filtered.angle_min, raw.angle_min)
        self.assertEqual(filtered.angle_increment, raw.angle_increment)

    def test_negative_increment_and_empty_intensities(self):
        raw = LaserScan()
        raw.angle_min = math.pi
        raw.angle_increment = -math.radians(1.0)
        raw.ranges = [2.0] * 361

        filtered = mask_sector(raw, 220.0, 320.0)
        self.assertTrue(math.isinf(filtered.ranges[180 + 90]))  # -90 deg = 270 deg
        self.assertEqual(filtered.ranges[180], 2.0)  # 0 deg remains valid
        self.assertEqual(len(filtered.intensities), 0)


if __name__ == '__main__':
    unittest.main()
