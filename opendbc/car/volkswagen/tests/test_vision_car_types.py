import json
import os
import tempfile
import unittest
from unittest import mock

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.volkswagen import vision_car_types as vct
from opendbc.car.volkswagen.mebcan import ACC_HUD_ACTIVE, create_acc_hud_control
from opendbc.car.volkswagen.values import CAR, DBC


class FakeClock:
  def __init__(self, t=100.0):
    self.t = t

  def __call__(self):
    return self.t


class TestVisionCarTypes(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.TemporaryDirectory()
    self.path = os.path.join(self.dir.name, "yolo_lead.json")
    self.clock = FakeClock()
    self.reader = vct.VisionCarTypes(self.path, self.clock)

  def tearDown(self):
    self.dir.cleanup()

  def _write(self, t, lead=3, left=3, right=3):
    with open(self.path, "w") as f:
      json.dump({"t": t, "lead": {"type": lead}, "left": {"type": left}, "right": {"type": right}}, f)

  def test_no_file_means_cars(self):
    self.assertEqual(self.reader.get(), vct.ALL_CARS)

  def test_fresh_file_is_used(self):
    self._write(self.clock.t, lead=2, left=1, right=4)
    self.assertEqual(self.reader.get(), (2, 1, 4))

  def test_stale_file_falls_back_to_cars(self):
    self._write(self.clock.t, lead=2)
    self.assertEqual(self.reader.get(), (2, 3, 3))
    self.clock.t += vct.MAX_AGE + 0.1          # writer stopped
    self.assertEqual(self.reader.get(), vct.ALL_CARS)

  def test_invalid_types_and_garbage_are_cars(self):
    self._write(self.clock.t, lead=9, left=0, right=2)
    self.assertEqual(self.reader.get(), (3, 3, 2))
    with open(self.path, "w") as f:
      f.write("{not json")
    self.clock.t += vct.READ_PERIOD + 1e-3
    self.assertEqual(self.reader.get(), (3, 3, 2))  # keeps the last good read while it is fresh

  def test_reads_at_most_every_read_period(self):
    self._write(self.clock.t, lead=2)
    self.assertEqual(self.reader.get()[0], 2)
    self._write(self.clock.t, lead=1)
    self.assertEqual(self.reader.get()[0], 2)      # same instant: cached
    self.clock.t += vct.READ_PERIOD + 1e-3
    self._write(self.clock.t, lead=1)
    self.assertEqual(self.reader.get()[0], 1)


class TestCarTypesOnCluster(unittest.TestCase):
  def _acc_19(self, car, types, lead_visible=True):
    packer = CANPacker(DBC[car][Bus.pt])
    parser = CANParser(DBC[car][Bus.pt], [("ACC_19", 0)], 0)
    with mock.patch.object(vct.vision_car_types, "get", return_value=types):
      addr, dat, bus = create_acc_hud_control(packer, 0, ACC_HUD_ACTIVE, 100, lead_visible, 2, False, False, 30., 20, False, 0, 0,
                                              True, 0, (25., 40.))
    parser.update([(0, [(addr, dat, bus)])])
    return parser.vl["ACC_19"]

  def test_three_types_land_in_their_fields(self):
    v = self._acc_19(CAR.VOLKSWAGEN_GOLF_MK8, (2, 1, 4))
    self.assertEqual((v["Lead_Type"], v["Lead_Type_Left"], v["Lead_Type_Right"]), (2, 1, 4))

  def test_no_lead_no_lead_type(self):
    v = self._acc_19(CAR.VOLKSWAGEN_GOLF_MK8, (2, 3, 3), lead_visible=False)
    self.assertEqual(v["Lead_Type"], 0)

  def test_stock_icons_without_the_daemon(self):
    v = self._acc_19(CAR.VOLKSWAGEN_GOLF_MK8, vct.ALL_CARS)
    self.assertEqual((v["Lead_Type"], v["Lead_Type_Left"], v["Lead_Type_Right"]), (3, 3, 3))


if __name__ == "__main__":
  unittest.main()
