import unittest

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


if __name__ == "__main__":
  unittest.main()
