import math
import unittest

from kaboat_hardware.thruster_backend import (
    ThrusterConfig, ThrusterMixer, SlewRateLimiter,
    ratio_to_pwm, clamp, DummyBackend
)


class TestThrusterBackend(unittest.TestCase):

    def setUp(self):
        self.config = ThrusterConfig(
            neutral_pwm=1500,
            max_forward_pwm=1900,
            max_reverse_pwm=1100,
            deadband_us=25,
            left_trim=1.0,
            right_trim=1.0,
            invert_left=False,
            invert_right=False,
            max_slew_rate=2.0,
        )

    def test_ratio_to_pwm_neutral(self):
        """0.0 명령은 정확히 중립 1500µs여야 한다."""
        pwm = ratio_to_pwm(0.0, self.config)
        self.assertEqual(pwm, 1500)

    def test_ratio_to_pwm_forward_max(self):
        """+1.0 명령은 최대 전진 1900µs여야 한다."""
        pwm = ratio_to_pwm(1.0, self.config)
        self.assertEqual(pwm, 1900)

    def test_ratio_to_pwm_reverse_max(self):
        """-1.0 명령은 최대 후진 1100µs여야 한다."""
        pwm = ratio_to_pwm(-1.0, self.config)
        self.assertEqual(pwm, 1100)

    def test_ratio_to_pwm_deadband_jump(self):
        """미세 전진/후진 명령 시 ESC 불감대(±25µs)를 건너뛰어 응답해야 한다."""
        # 0보다 큰 미세값은 1500 + 25 = 1525 이상이어야 함
        pwm_fwd_min = ratio_to_pwm(0.001, self.config)
        self.assertGreaterEqual(pwm_fwd_min, 1525)

        # 0보다 작은 미세값은 1500 - 25 = 1475 이하여야 함
        pwm_rev_min = ratio_to_pwm(-0.001, self.config)
        self.assertLessEqual(pwm_rev_min, 1475)

    def test_ratio_to_pwm_clamping(self):
        """±1.0을 초과하는 명령은 ±1.0으로 클램프되어야 한다."""
        self.assertEqual(ratio_to_pwm(1.5, self.config), 1900)
        self.assertEqual(ratio_to_pwm(-2.0, self.config), 1100)

    def test_slew_rate_limiter(self):
        """급격한 출력 변화(가속)를 설정된 속도로 제한해야 한다."""
        limiter = SlewRateLimiter(max_rate=2.0, initial_val=0.0)

        # t=0s, target=1.0 -> 첫 초기화 시점
        val0 = limiter.update(1.0, now=0.0)
        self.assertEqual(val0, 1.0)

        # t=0.1s 에서 target=0.0 -> 최대 감소량: 2.0 * 0.1 = 0.2 -> 0.8
        limiter.reset(initial_val=1.0)
        limiter.update(1.0, now=0.0)
        val1 = limiter.update(0.0, now=0.1)
        self.assertAlmostEqual(val1, 0.8, places=4)

        # t=0.5s (0.4s 경과) -> 추가 감소량: 2.0 * 0.4 = 0.8 -> 0.0 도달
        val2 = limiter.update(0.0, now=0.5)
        self.assertAlmostEqual(val2, 0.0, places=4)

    def test_thruster_mixer_straight(self):
        """순수 전진 명령 시 좌우 PWM이 동일해야 한다."""
        mixer = ThrusterMixer(self.config)
        l_pwm, r_pwm, l_ratio, r_ratio = mixer.mix(linear_x=0.5, angular_z=0.0, now=0.0)
        self.assertEqual(l_pwm, r_pwm)
        self.assertGreater(l_pwm, 1500)
        self.assertAlmostEqual(l_ratio, 0.5, places=4)
        self.assertAlmostEqual(r_ratio, 0.5, places=4)

    def test_thruster_mixer_turn(self):
        """좌회전(angular.z > 0) 시 좌측 감속/후진, 우측 가속이어야 한다."""
        mixer = ThrusterMixer(self.config)
        # 제자리 좌회전 (linear=0, angular=0.3)
        l_pwm, r_pwm, l_ratio, r_ratio = mixer.mix(linear_x=0.0, angular_z=0.3, now=0.0)
        self.assertLess(l_pwm, 1500)     # 좌측 후진
        self.assertGreater(r_pwm, 1500)  # 우측 전진
        self.assertAlmostEqual(l_ratio, -0.3, places=4)
        self.assertAlmostEqual(r_ratio, 0.3, places=4)

    def test_thruster_mixer_trim_and_invert(self):
        """트림 및 반전 플래그가 정상 적용되어야 한다."""
        cfg = ThrusterConfig(
            neutral_pwm=1500, max_forward_pwm=1900, max_reverse_pwm=1100,
            deadband_us=0, left_trim=0.8, right_trim=1.0,
            invert_left=True, invert_right=False, max_slew_rate=10.0
        )
        mixer = ThrusterMixer(cfg)
        l_pwm, r_pwm, l_rat, r_rat = mixer.mix(linear_x=1.0, angular_z=0.0, now=0.0)
        # 좌측: invert -> -1.0 * 0.8 = -0.8 -> 후진
        self.assertAlmostEqual(l_rat, -0.8, places=4)
        self.assertLess(l_pwm, 1500)
        # 우측: +1.0 -> 전진
        self.assertAlmostEqual(r_rat, 1.0, places=4)
        self.assertEqual(r_pwm, 1900)

    def test_dummy_backend(self):
        """더미 백엔드가 에러 없이 PWM을 수신하고 저장해야 한다."""
        dummy = DummyBackend()
        self.assertTrue(dummy.open())
        self.assertTrue(dummy.send_pwm(1650, 1450))
        self.assertEqual(dummy.last_left, 1650)
        self.assertEqual(dummy.last_right, 1450)
        dummy.close()
        self.assertFalse(dummy.is_open)


if __name__ == '__main__':
    unittest.main()

