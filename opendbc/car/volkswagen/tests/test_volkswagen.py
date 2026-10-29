import random
import re
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen import mebcan
from opendbc.car.volkswagen.carcontroller import HCAMitigation, RADAR_ACC_CANCEL_FRAMES, RADAR_ENGAGE_WAIT_FRAMES
from opendbc.car.volkswagen.carstate import CarState, RADAR_ACC_HUD_TIMEOUT_FRAMES, RADAR_ACC_STATUS_TIMEOUT_FRAMES
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, CanBus, CarControllerParams as CCP, FW_QUERY_CONFIG, VolkswagenFlags, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenCarState(unittest.TestCase):
  @staticmethod
  def car_state():
    car_state = CarState.__new__(CarState)
    car_state.eps_init_complete = True
    car_state.frame = 1000
    car_state.hca_fault_frames = 0
    return car_state

  def test_hca_fault_only_reported_in_drive(self):
    # HCA can briefly report FAULT when shifting to reverse at high steering angles
    for drive_mode, expected_fault in ((False, False), (True, True)):
      with self.subTest(drive_mode=drive_mode):
        car_state = self.car_state()
        for _ in range(CCP.HCA_PERMANENT_FAULT_FRAMES):
          _, permanent_fault, _ = car_state.update_hca_state("FAULT", drive_mode=drive_mode)
        assert permanent_fault == expected_fault

  def test_hca_fault_blip_is_temporary(self):
    # HCA blips FAULT when lateral is handed back at a high steering angle, e.g. a door opening at
    # a standstill. It must not raise a permanent fault, which alerts "LKAS Fault: Restart the Car".
    car_state = self.car_state()

    for _ in range(CCP.HCA_PERMANENT_FAULT_FRAMES - 1):
      temporary_fault, permanent_fault, _ = car_state.update_hca_state("FAULT")
      assert temporary_fault
      assert not permanent_fault

    # the blip clears, and a later isolated blip must not resume the previous count
    temporary_fault, permanent_fault, _ = car_state.update_hca_state("READY")
    assert not temporary_fault
    assert not permanent_fault

    temporary_fault, permanent_fault, _ = car_state.update_hca_state("FAULT")
    assert temporary_fault
    assert not permanent_fault

  def test_hca_fault_latches_when_persistent(self):
    car_state = self.car_state()

    for frame in range(CCP.HCA_PERMANENT_FAULT_FRAMES + 10):
      temporary_fault, permanent_fault, _ = car_state.update_hca_state("FAULT")
      expected_permanent = frame + 1 >= CCP.HCA_PERMANENT_FAULT_FRAMES
      assert permanent_fault == expected_permanent
      assert temporary_fault != expected_permanent

  def test_hca_disabled_is_permanent_immediately(self):
    # DISABLED means the EPS was never configured for Lane Assist, so there is nothing to debounce
    car_state = self.car_state()
    _, permanent_fault, _ = car_state.update_hca_state("DISABLED")
    assert permanent_fault


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"


class TestVolkswagenStandstillSignals(unittest.TestCase):
  @staticmethod
  def make_interface(platform):
    fingerprint = {bus: {} for bus in range(4)}
    cp = CarInterface.get_params(platform, fingerprint, [], False, False, False)
    cp_sp = CarInterface.get_params_sp(cp, platform, fingerprint, [], False, False, False)
    cp_ic = CarInterface.get_params_ic(cp, platform, fingerprint, [], False, False, False)
    return CarInterface(cp, cp_sp, cp_ic), CANPacker(DBC[platform][Bus.pt])

  def test_meb_motion_state_and_gen2_hms(self):
    for platform in (CAR.CUPRA_BORN_MK1, CAR.VOLKSWAGEN_ID3_MK2):
      with self.subTest(platform=platform):
        ci, packer = self.make_interface(platform)
        for motion_state in range(4):
          msg = packer.make_can_msg("ESC_50", 0, {"Motion_State": motion_state})
          ci.update([(motion_state + 1, [msg])])
          self.assertEqual(ci.CS.esp_standstill, motion_state == 3)
          self.assertEqual(ci.CS.esp_hold_confirmation, motion_state == 3 if platform == CAR.CUPRA_BORN_MK1 else False)

        if platform == CAR.VOLKSWAGEN_ID3_MK2:
          for hms_status in (0, 1, 2):
            msg = packer.make_can_msg("VMM_02", 0, {"HMS_Status": hms_status})
            ci.update([(hms_status + 5, [msg])])
            self.assertEqual(ci.CS.esp_hold_confirmation, hms_status == 1)

  def test_mqb_evo_standstill(self):
    # ESC_50.Standstill never reads 1 on a Golf 8, so zero wheel speed counts as standstill too
    ci, packer = self.make_interface(CAR.VOLKSWAGEN_GOLF_MK8)
    wheels = ("VL_Radgeschw", "VR_Radgeschw", "HL_Radgeschw", "HR_Radgeschw")
    for i, (standstill, wheel_speed, expected) in enumerate(((0, 0., True), (1, 0., True), (0, 5., False), (1, 5., True))):
      msgs = [packer.make_can_msg("ESC_50", 0, {"Standstill": standstill}),
              packer.make_can_msg("ESC_51", 0, dict.fromkeys(wheels, wheel_speed))]
      ci.update([(i + 1, msgs)])
      self.assertEqual(ci.CS.esp_standstill, expected, f"{standstill=} {wheel_speed=}")

  def test_mqb_evo_hold_drop_at_standstill_keeps_drive_off(self):
    # 2026-10-26 19:53: ~0.5 s after ANFAHREN the restarted engine came up and the ESP dropped its hold
    # confirmation while the wheels were still at zero. ANFAHREN must survive that until the car moves.
    ci, packer = self.make_interface(CAR.VOLKSWAGEN_GOLF_MK8)
    state = mebcan.SunnypilotMebLongStateMachine(ci.CP, CCP(ci.CP))
    wheels = ("VL_Radgeschw", "VR_Radgeschw", "HL_Radgeschw", "HR_Radgeschw")
    CC = SimpleNamespace(enabled=True, longActive=True, cruiseControl=SimpleNamespace(override=False),
                         actuators=SimpleNamespace(longControlState=LongCtrlState.stopping))

    def step(i, hold, wheel_speed):
      ci.update([(i, [packer.make_can_msg("ESP_21", 0, {"ESP_Haltebestaetigung": hold}),
                      packer.make_can_msg("ESC_51", 0, dict.fromkeys(wheels, wheel_speed))])])
      ci.CS.out.vEgo = ci.CS.out.vEgoRaw
      return state.update(ci.CS, CC, 0.5)[2]

    self.assertEqual(step(1, 1, 0.), mebcan.ACC_HMS_HOLD)
    CC.actuators.longControlState = LongCtrlState.pid
    self.assertEqual(step(2, 1, 0.), mebcan.ACC_HMS_RELEASE)
    for i in range(3, 6):
      self.assertEqual(step(i, 0, 0.), mebcan.ACC_HMS_RELEASE)
    # the wheels turn: drive-off is done and the hold is ramped away as before
    self.assertEqual(step(6, 0, 1.), mebcan.ACC_HMS_RAMP_RELEASE)

  def test_mqb_evo_rain_sensor_and_wiper_stalk(self):
    ci, packer = self.make_interface(CAR.VOLKSWAGEN_GOLF_MK8)
    stalks = ({}, {"WH_Intervall": 1}, {"WH_WischerStufe1": 1}, {"WH_WischerStufe2": 1}, {"WH_Tipwischen": 1})
    for i, (stalk, expected) in enumerate(zip(stalks, (0, 1, 2, 3, 0), strict=True)):
      msgs = [packer.make_can_msg("RLS_01", 0, {"RS_Regenmenge": 10 * i, "RS_Wischergeschwindigkeit": i}),
              packer.make_can_msg("SMLS_01", 0, stalk)]
      ci.update([(i + 1, msgs)])
      self.assertEqual(ci.CS.out_ic.rainAmount, 10 * i)
      self.assertEqual(ci.CS.out_ic.rainWiperRequest, i)
      self.assertEqual(ci.CS.out_ic.wiperSwitch, expected)


class TestRadarAccHud(unittest.TestCase):
  @staticmethod
  def make_interface(alpha_long=True, gateway=True):
    fingerprint = {bus: {} for bus in range(4)}
    if gateway:
      fingerprint[1][0x520] = 8  # Airbag_02 on bus 1: gateway harness
    cp = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK8, fingerprint, [], alpha_long, False, False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.VOLKSWAGEN_GOLF_MK8, fingerprint, [], alpha_long, False, False)
    cp_ic = CarInterface.get_params_ic(cp, CAR.VOLKSWAGEN_GOLF_MK8, fingerprint, [], alpha_long, False, False)
    return CarInterface(cp, cp_sp, cp_ic), CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt])

  def test_read_only_with_openpilot_long_on_the_gateway(self):
    for alpha_long, gateway in ((True, True), (False, True), (True, False)):
      with self.subTest(alpha_long=alpha_long, gateway=gateway):
        ci, _ = self.make_interface(alpha_long, gateway)
        self.assertEqual(ci.CS.radar_acc_hud_available, alpha_long and gateway)

  def test_radar_values_kept_while_it_sends(self):
    ci, packer = self.make_interface()
    radar = packer.make_can_msg("ACC_19", CanBus(ci.CP).cam, {"ACC_Texte_Zusatzanz_02": 4, "ACC_Optischer_Fahrerhinweis": 1})
    ci.update([(1, [radar])])
    self.assertEqual(ci.CS.radar_acc_hud["ACC_Texte_Zusatzanz_02"], 4)
    for t in range(2, RADAR_ACC_HUD_TIMEOUT_FRAMES + 1):
      ci.update([(t, [])])
    self.assertEqual(ci.CS.radar_acc_hud["ACC_Optischer_Fahrerhinweis"], 1)
    ci.update([(RADAR_ACC_HUD_TIMEOUT_FRAMES + 1, [])])
    self.assertIsNone(ci.CS.radar_acc_hud)  # the radar went quiet: never hold on to its last warning

  def test_missing_radar_hud_does_not_invalidate_the_parser(self):
    ci, packer = self.make_interface()
    self.assertIsNone(ci.CS.radar_acc_hud)
    state = ci.can_parsers[Bus.cam].message_states[packer.dbc.name_to_msg["ACC_19"].address]
    self.assertTrue(state.ignore_alive)


class TestEngageButtons(unittest.TestCase):
  # The radar engages its own ACC on the press and refuses it while braking on the move; it faults when TSK disagrees
  def setUp(self):
    self.ci, self.packer = TestRadarAccHud.make_interface()
    self.t = 0

  def step(self, brake=0, speed=50., radar_status=2, **buttons):
    self.t += 1
    wheels = dict.fromkeys(("VL_Radgeschw", "VR_Radgeschw", "HL_Radgeschw", "HR_Radgeschw"), speed)
    msgs = [self.packer.make_can_msg("GRA_ACC_01", CanBus(self.ci.CP).pt, buttons),
            self.packer.make_can_msg("Motor_51", CanBus(self.ci.CP).pt, {"TSK_Status": 2}),
            self.packer.make_can_msg("Motor_14", CanBus(self.ci.CP).pt, {"MO_Fahrer_bremst": brake}),
            self.packer.make_can_msg("ESC_51", CanBus(self.ci.CP).pt, wheels)]
    if radar_status is not None:
      msgs.append(self.packer.make_can_msg("ACC_18", CanBus(self.ci.CP).cam, {"ACC_Status_ACC": radar_status}))
    return self.ci.update([(self.t, msgs)])[0]

  def test_engage_on_press(self):
    for button in ("GRA_Tip_Setzen", "GRA_Tip_Wiederaufnahme", "GRA_TravelAssist"):
      with self.subTest(button=button):
        self.setUp()
        self.step()
        self.assertTrue(self.step(**{button: 1}).buttonEnable)
        self.assertFalse(self.step(**{button: 1}).buttonEnable)
        self.assertFalse(self.step().buttonEnable)  # the release does not engage again

  def test_press_while_braking_on_the_move(self):
    # the press is still reported, where the brake makes it a "Pedal Pressed" no entry; the release does not engage
    for button in ("GRA_Tip_Setzen", "GRA_Tip_Wiederaufnahme", "GRA_TravelAssist"):
      with self.subTest(button=button):
        self.setUp()
        self.step(brake=1)
        CS = self.step(brake=1, **{button: 1})
        self.assertTrue(CS.buttonEnable and CS.brakePressed and not CS.standstill)
        self.assertFalse(self.step(**{button: 1}).buttonEnable)
        self.assertFalse(self.step().buttonEnable)

  def test_press_while_holding_the_brake_at_a_stop(self):
    self.step(brake=1, speed=0.)
    CS = self.step(brake=1, speed=0., GRA_Tip_Setzen=1)
    self.assertTrue(CS.buttonEnable and CS.standstill)

  def test_speed_buttons_do_not_engage(self):
    self.step()
    for button in ("GRA_Tip_Hoch", "GRA_Tip_Runter"):
      self.assertFalse(self.step(**{button: 1}).buttonEnable)
      self.assertFalse(self.step().buttonEnable)

  def test_radar_acc_status_reported(self):
    self.step()
    for radar_status, fault in ((2, False), (3, False), (5, False), (6, True), (7, True)):
      self.step(radar_status=radar_status)
      self.assertEqual(self.ci.CS.out_ic.radarAccStatus, radar_status)
      self.assertEqual(self.ci.CS.out_ic.radarAccFault, fault)
    for _ in range(RADAR_ACC_STATUS_TIMEOUT_FRAMES):
      self.step(radar_status=None)
    self.assertEqual(self.ci.CS.out_ic.radarAccStatus, 0)  # the radar went quiet
    self.assertFalse(self.ci.CS.out_ic.radarAccFault)

  def test_cancels_a_radar_engagement_openpilot_did_not_follow(self):
    CC, CC_SP, CC_IC = structs.CarControl(), structs.CarControlSP(), structs.CarControlIC()

    def cancel_sent(radar_status, enabled=False, **buttons):
      self.step(radar_status=radar_status, COUNTER=self.t % 16, **buttons)
      CC.enabled = enabled
      _, sends = self.ci.apply(CC.as_reader(), CC_SP, CC_IC, self.t * 10_000_000)
      gra = [m for m in sends if m[0] == self.packer.dbc.name_to_msg["GRA_ACC_01"].address]
      return any(m[1][1] & 0x20 for m in gra)  # GRA_Abbrechen, bit 13

    self.step()
    # radar and openpilot engaged together: nothing to do
    self.assertFalse(any(cancel_sent(3, enabled=True) for _ in range(RADAR_ACC_CANCEL_FRAMES + 5)))
    self.assertFalse(cancel_sent(2))
    # the radar engaged and openpilot did not, with the key held or not
    sent = [cancel_sent(3, GRA_Tip_Setzen=1) for _ in range(RADAR_ACC_CANCEL_FRAMES + 2)]
    self.assertFalse(any(sent[:RADAR_ACC_CANCEL_FRAMES]))
    self.assertTrue(sent[-1])
    # not once the radar's ACC has faulted
    self.assertFalse(any(cancel_sent(7) for _ in range(RADAR_ACC_CANCEL_FRAMES + 2)))

  def test_acc_waits_for_the_radar(self):
    # 2026-10-27 route 223: Travel Assist at a stop, TSK regulated at +0.08 s, the radar was still in standby and faulted
    CC, CC_SP, CC_IC = structs.CarControl(), structs.CarControlSP(), structs.CarControlIC()
    acc_18 = self.packer.dbc.name_to_msg["ACC_18"].address
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_18", float("nan"))], CanBus(self.ci.CP).pt)

    def sent_status(radar_status, enabled=True, **buttons):
      self.step(radar_status=radar_status, **buttons)
      CC.enabled = CC.longActive = enabled
      _, sends = self.ci.apply(CC.as_reader(), CC_SP, CC_IC, self.t * 10_000_000)
      msgs = [m for m in sends if m[0] == acc_18]
      if msgs:
        parser.update([(self.t * 10_000_000, msgs)])
      return parser.vl["ACC_18"]["ACC_Status_ACC"]

    for _ in range(5):
      sent_status(2, enabled=False)
    statuses = [sent_status(2, GRA_TravelAssist=1)] + [sent_status(2, GRA_TravelAssist=1) for _ in range(10)]
    self.assertTrue(all(st == 2 for st in statuses), statuses)  # standby while the radar has not engaged
    for _ in range(3):
      sent_status(3)
    self.assertEqual(sent_status(3), 3)
    # once engaged together, the radar's own transitions do not hold openpilot back
    self.assertEqual(sent_status(4), 3)

    # the radar never follows: openpilot regulates anyway after the wait rather than leave the driver without ACC
    for _ in range(5):
      sent_status(2, enabled=False)
    statuses = [sent_status(2, GRA_Tip_Wiederaufnahme=1)] + [sent_status(2) for _ in range(RADAR_ENGAGE_WAIT_FRAMES + 5)]
    self.assertEqual(statuses.index(3), RADAR_ENGAGE_WAIT_FRAMES)
    self.assertTrue(all(st == 3 for st in statuses[RADAR_ENGAGE_WAIT_FRAMES:]))

    # no wait once the radar's ACC has faulted
    for _ in range(5):
      sent_status(7, enabled=False)
    sent_status(7, GRA_Tip_Wiederaufnahme=1)
    self.assertEqual(sent_status(7), 3)

  def test_travel_assist_cluster_notification(self):
    # stock camera after a Travel Assist press: key press flag, then the enable request the cluster shows a text for
    CC, CC_SP, CC_IC = structs.CarControl(), structs.CarControlSP(), structs.CarControlIC()
    ta_01 = self.packer.dbc.name_to_msg["TA_01"].address
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("TA_01", float("nan"))], CanBus(self.ci.CP).pt)
    self.ci.CS.travel_assist_available = True

    def sent(enabled=True, **buttons):
      self.step(radar_status=3 if enabled else 2, **buttons)
      self.ci.CS.travel_assist_available = True
      CC.enabled = CC.longActive = enabled
      _, sends = self.ci.apply(CC.as_reader(), CC_SP, CC_IC, self.t * 10_000_000)
      msgs = [m for m in sends if m[0] == ta_01]
      if msgs:
        parser.update([(self.t * 10_000_000, msgs)])
      return int(parser.vl["TA_01"]["Travel_Assist_Press_Ack"]), int(parser.vl["TA_01"]["Travel_Assist_Request"])

    for _ in range(5):
      sent(enabled=False)
    seq = [sent(GRA_TravelAssist=1)] + [sent() for _ in range(200)]
    self.assertTrue(seq[20][0] and not seq[20][1])
    self.assertEqual(seq[50], (1, 4))
    self.assertEqual(seq[100], (0, 4))
    self.assertEqual(seq[150], (0, 0))
    # nothing for a press while already engaged, as stock
    seq = [sent(GRA_TravelAssist=1)] + [sent() for _ in range(200)]
    self.assertEqual(set(seq), {(0, 0)})
    # nothing for SET
    for _ in range(5):
      sent(enabled=False)
    seq = [sent(GRA_Tip_Setzen=1)] + [sent() for _ in range(200)]
    self.assertEqual(set(seq), {(0, 0)})

  def test_engage_display_prio(self):
    # stock radar ACC_19: medium display prio for 3 s from the engagement, then the active prio
    CC, CC_SP, CC_IC = structs.CarControl(), structs.CarControlSP(), structs.CarControlIC()
    acc_19 = self.packer.dbc.name_to_msg["ACC_19"].address
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_19", float("nan"))], CanBus(self.ci.CP).pt)

    def prio(enabled=True, **buttons):
      self.step(radar_status=3 if enabled else 2, **buttons)
      CC.enabled = CC.longActive = enabled
      _, sends = self.ci.apply(CC.as_reader(), CC_SP, CC_IC, self.t * 10_000_000)
      msgs = [m for m in sends if m[0] == acc_19]
      if msgs:
        parser.update([(self.t * 10_000_000, msgs)])
      return int(parser.vl["ACC_19"]["ACC_Display_Prio"])

    for _ in range(10):
      prio(enabled=False)
    seq = [prio(GRA_Tip_Setzen=1)] + [prio() for _ in range(400)]
    self.assertEqual(seq[100], 1)
    self.assertEqual(seq[280], 1)
    self.assertEqual(seq[320], 2)
    self.assertEqual(seq[400], 2)

  def test_ready_to_start_only_after_braking_to_a_stop(self):
    # stock radar ACC_19: "ACC ready to start" (ACC_Events 3) for 2.5 s once its ACC has braked the car into a hold, not
    # after an engagement at a stop, where it would cover the engage text; ACC_18's hold does not depend on it
    CC, CC_SP, CC_IC = structs.CarControl(), structs.CarControlSP(), structs.CarControlIC()
    acc_18 = self.packer.dbc.name_to_msg["ACC_18"].address
    acc_19 = self.packer.dbc.name_to_msg["ACC_19"].address
    parser = CANParser(DBC[CAR.VOLKSWAGEN_GOLF_MK8][Bus.pt], [("ACC_18", float("nan")), ("ACC_19", float("nan"))],
                       CanBus(self.ci.CP).pt)

    def sent(enabled=True, held=True):
      self.step(radar_status=3 if enabled else 2, speed=0. if held else 30.)
      self.ci.CS.esp_hold_confirmation = held
      CC.enabled = CC.longActive = enabled
      CC.actuators.longControlState = LongCtrlState.stopping if held else LongCtrlState.pid
      _, sends = self.ci.apply(CC.as_reader(), CC_SP, CC_IC, self.t * 10_000_000)
      msgs = [m for m in sends if m[0] in (acc_18, acc_19)]
      if msgs:
        parser.update([(self.t * 10_000_000, msgs)])
      return int(parser.vl["ACC_19"]["ACC_Events"]), int(parser.vl["ACC_18"]["ACC_Anforderung_HMS"])

    # engaging while held at a stop: no ready to start
    for _ in range(10):
      sent(enabled=False)
    seq = [sent() for _ in range(400)]
    self.assertNotIn(3, [event for event, _ in seq])

    # openpilot brakes the car to a stop: ready to start for 2.5 s of the hold, then gone while it keeps holding
    for _ in range(50):
      sent(held=False)
    seq = [sent() for _ in range(400)]
    events = [event for event, _ in seq]
    self.assertTrue(all(event == 3 for event in events[10:240]), events[:20])
    self.assertTrue(all(event == 0 for event in events[260:]))
    self.assertEqual({hms for _, hms in seq[10:240]}, {hms for _, hms in seq[260:]})


class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_flag_values_are_unique(self):
    values = [flag.value for flag in VolkswagenFlags.__members__.values()]
    self.assertEqual(len(values), len(set(values)))

  def test_bsm_does_not_change_platform(self):
    fingerprint = {bus: {} for bus in range(4)}
    fingerprint[0][0x24C] = 8  # MEB_Side_Assist_01
    meb_cp = CarInterface.get_params(CAR.CUPRA_BORN_MK1, fingerprint, [], False, False, False)
    self.assertTrue(meb_cp.flags & VolkswagenFlags.HAS_BSM)
    self.assertFalse(meb_cp.flags & VolkswagenFlags.MQB_EVO)

    fingerprint[0].clear()
    mqb_evo_cp = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK8, fingerprint, [], False, False, False)
    self.assertTrue(mqb_evo_cp.flags & VolkswagenFlags.MQB_EVO)
    self.assertFalse(mqb_evo_cp.flags & VolkswagenFlags.HAS_BSM)

  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    platforms = list(CAR)
    for i, platform in enumerate(platforms):
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # Platforms may share chassis codes when their model years disambiguate the VIN.
        for comp in platforms[i + 1:]:
          if not (platform.config.wmis & comp.config.wmis and
                  platform.config.chassis_codes & comp.config.chassis_codes):
            continue

          model_years_overlap = (not platform.config.model_years or not comp.config.model_years or
                                 bool(platform.config.model_years & comp.config.model_years))
          assert not model_years_overlap, f"Ambiguous VIN attributes: {platform} and {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      for wmi in WMI:
        for chassis_code in platform.config.chassis_codes | {"00"}:
          for model_year in platform.config.model_years | {"0"}:
            with self.subTest(platform=platform.name, wmi=wmi, chassis_code=chassis_code, model_year=model_year):
              vin = ["0"] * 17
              vin[0:3] = wmi
              vin[6:8] = chassis_code
              vin[9] = model_year
              vin = "".join(vin)

              # Check a few FW cases - expected, unexpected
              for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
                should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                                (not platform.config.model_years or model_year in platform.config.model_years) and
                                radar_fw in all_radar_fw)

                live_fws = {(0x757, None): [radar_fw]}
                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

                expected_matches = {platform} if should_match else set()
                assert expected_matches == matches, "Bad match"
