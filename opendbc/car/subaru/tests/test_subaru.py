import unittest
from types import SimpleNamespace

from opendbc.car.subaru.carcontroller import CarController, ANGLE_FILTER_SPEED_BP, OVERRIDE_SETTLE_FRAMES
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import DBC


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


def _cc(angle, lat_active=True):
  return SimpleNamespace(latActive=lat_active, actuators=SimpleNamespace(steeringAngleDeg=angle))


def _cs(v_ego, angle=0.0, cruise_enabled=True, rate=0.0, brake_pressed=False, steering_pressed=False):
  return SimpleNamespace(out=SimpleNamespace(vEgoRaw=v_ego, steeringAngleDeg=angle, steeringRateDeg=rate, steeringTorque=0.0,
                                             steeringPressed=steering_pressed, brakePressed=brake_pressed,
                                             cruiseState=SimpleNamespace(enabled=cruise_enabled)))


class TestSubaruAngleFilter(unittest.TestCase):
  def setUp(self):
    CP = CarInterface.get_non_essential_params("SUBARU_ASCENT_2023")
    self.CC = CarController(DBC[CP.carFingerprint], CP)

  def test_low_speed_smoothing(self):
    # a step in the request only partially lands the first frame, then converges
    self.CC.lateral_angle(_cc(10.0), _cs(1.0))
    self.assertTrue(0.0 < self.CC.angle_filter.x < 10.0)
    for _ in range(200):
      self.CC.lateral_angle(_cc(10.0), _cs(1.0))
    self.assertAlmostEqual(self.CC.angle_filter.x, 10.0, places=1)

  def test_highway_passthrough(self):
    self.CC.lateral_angle(_cc(7.0), _cs(ANGLE_FILTER_SPEED_BP[-1]))
    self.assertAlmostEqual(self.CC.angle_filter.x, 7.0)

  def test_inactive_tracks_measured(self):
    self.CC.angle_filter.x = 5.0
    self.CC.lateral_angle(_cc(10.0, lat_active=False), _cs(1.0, angle=3.3))
    self.assertEqual(self.CC.angle_filter.x, 3.3)

  def test_brake_overrides_before_cruise_drops(self):
    # cruiseState.enabled lags the brake signal on disengage; don't wait for it
    self.CC.lateral_angle(_cc(10.0), _cs(5.0, angle=3.3, cruise_enabled=True, brake_pressed=True))
    self.assertTrue(self.CC.driver_override)
    self.assertEqual(self.CC.apply_steer_last, 3.3)

  def test_override_requires_settle_to_resume(self):
    self.CC.driver_override = True
    for _ in range(OVERRIDE_SETTLE_FRAMES - 1):
      self.CC.lateral_angle(_cc(0.0), _cs(5.0))
    self.assertTrue(self.CC.driver_override)
    self.CC.lateral_angle(_cc(0.0), _cs(5.0))
    self.assertFalse(self.CC.driver_override)

  def test_override_settle_resets_on_hands_on(self):
    self.CC.driver_override = True
    for _ in range(OVERRIDE_SETTLE_FRAMES - 1):
      self.CC.lateral_angle(_cc(0.0), _cs(5.0))
    self.CC.lateral_angle(_cc(0.0), _cs(5.0, steering_pressed=True))
    self.assertTrue(self.CC.driver_override)
    for _ in range(OVERRIDE_SETTLE_FRAMES - 1):
      self.CC.lateral_angle(_cc(0.0), _cs(5.0))
    self.assertTrue(self.CC.driver_override)
