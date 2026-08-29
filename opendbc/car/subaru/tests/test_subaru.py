import unittest
from types import SimpleNamespace

from opendbc.car.subaru.carcontroller import CarController, ANGLE_FILTER_SPEED_BP
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


def _cs(v_ego, angle=0.0):
  return SimpleNamespace(out=SimpleNamespace(vEgoRaw=v_ego, steeringAngleDeg=angle, steeringTorque=0.0))


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
