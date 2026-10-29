import itertools
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.values import CAR, DBC
from opendbc.car.volkswagen.mebcan import create_acc_hud_control, RADAR_HUD_WARNING_SIGNALS
from opendbc.car.volkswagen.mebcan import ACC_HUD_ACTIVE, ACC_HUD_DISABLED, ACC_HUD_ENABLED, ACC_HUD_OVERRIDE, LDW_LINE_ACTIVE, LDW_LINE_NONE, \
  LDW_LINE_PASSIVE, ACC_EVENTS_ROAD, get_acc_hud_event, get_lane_line_display


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


class TestAccHudRoadEvent(unittest.TestCase):
  @staticmethod
  def _event(road_event, acc_hud=ACC_HUD_ACTIVE, esp_hold=False, curve=False, sl_ahead=False, sl=0):
    return get_acc_hud_event(acc_hud, esp_hold, False, 0, sl, curve_speed=curve, speed_limit_ahead=sl_ahead, curve_direction=-1,
                             road_event=road_event)

  def test_road_events_shown_while_acc_is_on(self):
    for road_event in ACC_EVENTS_ROAD:
      for acc_hud in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE):
        self.assertEqual(self._event(road_event, acc_hud), road_event)

  def test_not_in_standby_and_only_known_values(self):
    self.assertEqual(self._event(10, ACC_HUD_ENABLED), 0)
    self.assertEqual(self._event(10, ACC_HUD_DISABLED), 0)
    for road_event in (1, 2, 3, 6, 11, 12, 13):
      self.assertEqual(self._event(road_event), 0)

  def test_existing_events_win(self):
    self.assertEqual(self._event(14, esp_hold=True), 3)
    self.assertEqual(self._event(14, curve=True), 8)
    self.assertEqual(self._event(14, sl_ahead=True), 4)
    self.assertEqual(self._event(14, sl=1), 5)

  def test_road_event_only_fills_an_empty_slot(self):
    # with no road event every input combination keeps its old event; a road event only replaces "nothing shown"
    for acc_hud, esp_hold, pred, pred_type, sl, curve, sl_ahead in itertools.product(
        (ACC_HUD_DISABLED, ACC_HUD_ENABLED, ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE), (False, True), (False, True), (0, 1), (0, 1), (False, True),
        (False, True)):
      before = get_acc_hud_event(acc_hud, esp_hold, pred, pred_type, sl, curve, sl_ahead, 1)
      self.assertEqual(get_acc_hud_event(acc_hud, esp_hold, pred, pred_type, sl, curve, sl_ahead, 1, 0), before)
      with_road = get_acc_hud_event(acc_hud, esp_hold, pred, pred_type, sl, curve, sl_ahead, 1, 10)
      self.assertEqual(with_road, before if before or acc_hud not in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 10)

  def test_road_event_on_the_wire_without_a_speed(self):
    packer = CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt])
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_19", 0)], 0)
    for road_event in ACC_EVENTS_ROAD:
      addr, dat, bus = create_acc_hud_control(packer, 0, ACC_HUD_ACTIVE, 100, False, 2, False, False, 0, 20, False, road_event, 0, True, 0)
      parser.update([(0, [(addr, dat, bus)])])
      self.assertEqual(parser.vl["ACC_19"]["ACC_Events"], road_event)
      self.assertAlmostEqual(parser.vl["ACC_19"]["ACC_Event_Wunschgeschw"], 327.36, delta=0.5)  # no value, like stock


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


class TestRadarHudWarning(unittest.TestCase):
  # the radar's own ACC_19 at the 2026-10-25 13:42:48 FCW (route 0000021a seg 11), and while it just follows a lead
  RADAR_FCW = {"ACC_Optischer_Fahrerhinweis": 1, "ACC_Akustischer_Fahrerhinweis": 3, "ACC_Warnhinweis": 0, "ACC_EGO_Fahrzeug": 2,
               "ACC_Texte_Zusatzanz_02": 4, "ACC_Texte_Primaeranz_02": 0, "ACC_Display_Prio": 0, "ACC_rel_Objekt_Zusatzanz": 2,
               "ACC_Relevantes_Objekt_02": 2}
  RADAR_FOLLOWING = {"ACC_Optischer_Fahrerhinweis": 0, "ACC_Akustischer_Fahrerhinweis": 0, "ACC_Warnhinweis": 0, "ACC_EGO_Fahrzeug": 1,
                     "ACC_Texte_Zusatzanz_02": 0, "ACC_Texte_Primaeranz_02": 0, "ACC_Display_Prio": 2, "ACC_rel_Objekt_Zusatzanz": 1,
                     "ACC_Relevantes_Objekt_02": 1}

  @staticmethod
  def _values(acc_control, fcw_alert, radar_hud):
    packer = CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt])
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_19", 0)], 0)
    msg = create_acc_hud_control(packer, 0, acc_control, 100, True, 2, False, False, 30., 20, fcw_alert, 0, 0, True, 0, radar_hud=radar_hud)
    parser.update([(0, [msg])])
    return dict(parser.vl["ACC_19"])

  def test_radar_warning_shown_without_openpilot_fcw(self):
    for acc_control in (ACC_HUD_ACTIVE, ACC_HUD_ENABLED):
      with self.subTest(acc_control=acc_control):
        v = self._values(acc_control, False, self.RADAR_FCW)
        self.assertEqual({sig: v[sig] for sig in RADAR_HUD_WARNING_SIGNALS}, self.RADAR_FCW)

  def test_radar_wins_over_openpilot_fcw(self):
    v = self._values(ACC_HUD_ACTIVE, True, self.RADAR_FCW)
    self.assertEqual({sig: v[sig] for sig in RADAR_HUD_WARNING_SIGNALS}, self.RADAR_FCW)
    # its sound ends first: openpilot's does not keep it going
    quiet = {**self.RADAR_FCW, "ACC_Akustischer_Fahrerhinweis": 0}
    self.assertEqual(self._values(ACC_HUD_ACTIVE, True, quiet)["ACC_Akustischer_Fahrerhinweis"], 0)

  def test_openpilot_fcw_when_the_radar_does_not_warn(self):
    for radar_hud in (None, self.RADAR_FOLLOWING):
      with self.subTest(radar_hud=radar_hud):
        v = self._values(ACC_HUD_ACTIVE, True, radar_hud)
        self.assertEqual((v["ACC_Optischer_Fahrerhinweis"], v["ACC_Akustischer_Fahrerhinweis"], v["ACC_EGO_Fahrzeug"],
                          v["ACC_Texte_Zusatzanz_02"], v["ACC_Display_Prio"]), (1, 3, 2, 11, 0))

  def test_radar_states_that_are_not_warnings_change_nothing(self):
    # its override text, its ACC off text and the chime it gives when it sees ACC switch off
    for extra in ({}, {"ACC_Texte_Zusatzanz_02": 3}, {"ACC_Texte_Zusatzanz_02": 1}, {"ACC_Akustischer_Fahrerhinweis": 1}):
      with self.subTest(extra=extra):
        radar_hud = {**self.RADAR_FOLLOWING, **extra}
        for fcw_alert in (False, True):
          self.assertEqual(self._values(ACC_HUD_ACTIVE, fcw_alert, radar_hud), self._values(ACC_HUD_ACTIVE, fcw_alert, None))


  def test_radar_info_text_while_openpilot_is_not_engaged(self):
    # 0x14 "ACC not started": SET, RES or Travel Assist while ACC is off; 0x05/0x0f: cannot engage at a stop/braking
    for acc_control, info, expected in ((ACC_HUD_DISABLED, 0x14, (0x14, 1)), (ACC_HUD_ENABLED, 0x05, (0x05, 1)),
                                        (ACC_HUD_ENABLED, 0x0f, (0x0f, 1)), (ACC_HUD_DISABLED, 0, (0, 3)),
                                        (ACC_HUD_ENABLED, 0, (0, 3)),
                                        # not its fault text: openpilot keeps regulating after a radar fault
                                        (ACC_HUD_DISABLED, 0x02, (0, 3)),
                                        # not once openpilot engaged anyway
                                        (ACC_HUD_ACTIVE, 0x05, (0, 2))):
      with self.subTest(acc_control=acc_control, info=info):
        v = self._values(acc_control, False, {**self.RADAR_FOLLOWING, "ACC_Display_Prio": 1, "ACC_Texte_Info": info})
        self.assertEqual((v["ACC_Texte_Info"], v["ACC_Display_Prio"]), expected)

if __name__ == "__main__":
  unittest.main()
