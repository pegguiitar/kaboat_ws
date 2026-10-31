"""Check that the boat-structure sector and tilted scans never corrupt mapping."""

import math
import unittest

from sensor_msgs.msg import LaserScan

from kaboat_hardware.onboard_scan_filter import (
    invalidate_all_ranges,
    is_tilted,
    mask_sector,
    quaternion_to_roll_pitch,
)


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

    def test_quaternion_to_roll_pitch(self):
        # Identity quaternion -> 0 roll, 0 pitch
        roll, pitch = quaternion_to_roll_pitch(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)

        # 10 deg roll (around X axis)
        angle = math.radians(10.0)
        roll, pitch = quaternion_to_roll_pitch(math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))
        self.assertAlmostEqual(math.degrees(roll), 10.0, places=4)
        self.assertAlmostEqual(math.degrees(pitch), 0.0, places=4)

        # 20 deg pitch (around Y axis)
        angle_p = math.radians(20.0)
        roll, pitch = quaternion_to_roll_pitch(0.0, math.sin(angle_p / 2), 0.0, math.cos(angle_p / 2))
        self.assertAlmostEqual(math.degrees(roll), 0.0, places=4)
        self.assertAlmostEqual(math.degrees(pitch), 20.0, places=4)

    def test_is_tilted_threshold_5_deg(self):
        thresh = math.radians(5.0)

        # 4.9 degrees -> not tilted
        self.assertFalse(is_tilted(math.radians(4.9), math.radians(0.0), thresh))
        self.assertFalse(is_tilted(math.radians(0.0), math.radians(-4.9), thresh))

        # 5.1 degrees -> tilted
        self.assertTrue(is_tilted(math.radians(5.1), math.radians(0.0), thresh))
        self.assertTrue(is_tilted(math.radians(0.0), math.radians(-5.1), thresh))
        self.assertTrue(is_tilted(math.radians(6.0), math.radians(6.0), thresh))

    def test_invalidate_all_ranges(self):
        raw = LaserScan()
        raw.ranges = [1.0, 2.0, 3.0]
        raw.intensities = [10.0, 20.0, 30.0]

        invalidated = invalidate_all_ranges(raw)
        self.assertTrue(all(math.isinf(r) for r in invalidated.ranges))
        self.assertTrue(all(i == 0.0 for i in invalidated.intensities))


if __name__ == '__main__':
    unittest.main()
