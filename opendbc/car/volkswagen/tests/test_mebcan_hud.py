import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.values import CAR, DBC
from opendbc.car.volkswagen.mebcan import create_acc_hud_control
from opendbc.car.volkswagen.mebcan import ACC_HUD_ACTIVE, ACC_HUD_ENABLED, LDW_LINE_ACTIVE, LDW_LINE_NONE, LDW_LINE_PASSIVE, \
  get_acc_hud_event, get_lane_line_display


class TestAccHudCurveEvent(unittest.TestCase):
  def _event(self, curve_direction, curve_speed=True, acc_hud=ACC_HUD_ACTIVE):
    return get_acc_hud_event(acc_hud, False, False, 0, 0, curve_speed=curve_speed, curve_direction=curve_direction)

  def test_curve_icon_follows_direction(self):
    self.assertEqual(self._event(-1), 8)  # left curve
    self.assertEqual(self._event(1), 7)   # right curve
    self.assertEqual(self._event(2), 6)   # S-bend

  def test_unknown_direction_keeps_s_bend(self):
    self.assertEqual(self._event(0), 6)
    self.assertEqual(self._event(9), 6)

  def test_no_curve_event_without_curve_or_active_hud(self):
    self.assertEqual(self._event(-1, curve_speed=False), 0)
    self.assertEqual(self._event(-1, acc_hud=ACC_HUD_ENABLED), 0)


class TestLaneLineDisplay(unittest.TestCase):
  def test_hidden_when_not_seen(self):
    self.assertEqual(get_lane_line_display(False, False, False), LDW_LINE_NONE)
    self.assertEqual(get_lane_line_display(False, False, True), LDW_LINE_NONE)

  def test_seen_line_grey_passive_white_active(self):
    self.assertEqual(get_lane_line_display(True, False, False), LDW_LINE_PASSIVE)
    self.assertEqual(get_lane_line_display(True, False, True), LDW_LINE_ACTIVE)

  def test_departure_is_drawn_and_fits_the_2_bit_signal(self):
    for visible in (False, True):
      for active in (False, True):
        value = get_lane_line_display(visible, True, active)
        self.assertEqual(value, LDW_LINE_ACTIVE)
        self.assertLessEqual(value, 3)


class TestNeighbourLeads(unittest.TestCase):
  @staticmethod
  def _radar(left=(0, 0., 0.), right=(0, 0., 0.)):
    v = {"Distance_Status": 0}
    for lane, (oid, dist, lat) in (("Left_Lane", left), ("Right_Lane", right)):
      v[f"{lane}_01_ObjectID"], v[f"{lane}_01_Long_Distance"], v[f"{lane}_01_Lat_Distance"] = oid, dist, lat
      v[f"{lane}_02_ObjectID"], v[f"{lane}_02_Long_Distance"], v[f"{lane}_02_Lat_Distance"] = 0, 0., 0.
    return v

  def test_side_cars_in_their_lanes(self):
    self.assertEqual(CarState.parse_neighbour_leads(self._radar((3, 30., 3.2), (4, 40., -3.6))), (30., 40.))

  def test_cutting_in_car_is_not_a_side_car(self):
    # right lane object already at our axis: the radar has not re-assigned it yet
    self.assertEqual(CarState.parse_neighbour_leads(self._radar(right=(4, 25., -0.5))), (0., 0.))
    self.assertEqual(CarState.parse_neighbour_leads(self._radar(left=(3, 25., 0.4))), (0., 0.))


class TestSideCarsOnCluster(unittest.TestCase):
  def _values(self, acc_control, lead_visible, distance, neighbours):
    packer = CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt])
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_19", 0)], 0)
    addr, dat, bus = create_acc_hud_control(packer, 0, acc_control, 100, lead_visible, 2, False, False, distance, 20, False, 0, 0,
                                            True, 0, neighbours)
    parser.update([(0, [(addr, dat, bus)])])
    return parser.vl["ACC_19"]

  def test_drawn_in_standby_too(self):
    v = self._values(ACC_HUD_ENABLED, False, 0, (30., 40.))
    self.assertAlmostEqual(v["Lead_Distance_Left"], 30., delta=0.2)
    self.assertAlmostEqual(v["Lead_Distance_Right"], 40., delta=0.2)

  def test_side_car_at_the_lead_distance_drawn_once(self):
    v = self._values(ACC_HUD_ACTIVE, True, 25., (60., 27.))
    self.assertAlmostEqual(v["Lead_Distance_Left"], 60., delta=0.2)
    self.assertEqual(v["Lead_Distance_Right"], 0.)

  def test_curve_speed_on_left_and_right_curves(self):
    packer = CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt])
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_19", 0)], 0)
    for event in (6, 7, 8):
      addr, dat, bus = create_acc_hud_control(packer, 0, ACC_HUD_ACTIVE, 100, False, 2, False, False, 0, 20, False, event, 40 / 3.6, True, 0)
      parser.update([(0, [(addr, dat, bus)])])
      self.assertAlmostEqual(parser.vl["ACC_19"]["ACC_Event_Wunschgeschw"], 40., delta=0.5)


if __name__ == "__main__":
  unittest.main()
