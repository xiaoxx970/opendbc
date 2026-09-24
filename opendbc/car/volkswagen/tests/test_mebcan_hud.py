import unittest

from opendbc.car.volkswagen.mebcan import ACC_HUD_ACTIVE, ACC_HUD_ENABLED, get_acc_hud_event


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


if __name__ == "__main__":
  unittest.main()
