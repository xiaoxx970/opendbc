import random
import re
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import DT_CTRL, structs
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen import mebcan
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenCarState(unittest.TestCase):
  def test_car_not_ready_schema(self):
    car_state = structs.CarState()
    assert not car_state.carNotReady

  def test_hca_fault_only_reported_in_drive(self):
    car_state = CarState.__new__(CarState)
    car_state.eps_init_complete = True
    car_state.frame = 1000

    for drive_mode, expected_fault in ((False, False), (True, True)):
      with self.subTest(drive_mode=drive_mode):
        _, permanent_fault, _ = car_state.update_hca_state("FAULT", drive_mode=drive_mode)
        assert permanent_fault == expected_fault


class TestVolkswagenDbc(unittest.TestCase):
  def test_acc_event_speed_signal(self):
    for dbc in ("vw_meb_generated", "vw_meb_2024_generated", "vw_mqbevo_generated", "vw_mqbevo_2024_generated"):
      with self.subTest(dbc=dbc):
        signals = CANParser(dbc, [("ACC_19", 0)], 0).vl["ACC_19"]
        assert "ACC_Event_Wunschgeschw" in signals

  def test_gen2_blinker_control_without_unknown_signal(self):
    packer = CANPacker("vw_meb_2024_generated")
    stock_values = {"EA_Blinken": 0, "EA_Texte": 3}
    mebcan.create_blinker_control(packer, 0, stock_values, {"EA_Funktionsstatus": 0}, True, False, True)


class TestVolkswagenLongitudinalControl(unittest.TestCase):
  def get_hold_type(self, **kwargs):
    values = {
      "main_switch_on": True,
      "acc_faulted": False,
      "long_active": True,
      "starting": False,
      "stopping": False,
      "esp_hold": True,
      "override": True,
      "override_begin": True,
      "long_disabling": False,
      "hold_for_engine_start": False,
      "previous_hold_type": mebcan.ACC_HMS_HOLD,
      "just_reengaged": False,
      "v_ego": 0.,
    }
    return mebcan.get_acc_hold_type(**(values | kwargs))

  def test_override_waits_for_engine_start(self):
    assert self.get_hold_type(hold_for_engine_start=True) == mebcan.ACC_HMS_HOLD
    assert self.get_hold_type(hold_for_engine_start=False) == mebcan.ACC_HMS_RAMP_RELEASE

  def test_override_release_sequence(self):
    assert self.get_hold_type(override=False, stopping=True, hold_for_engine_start=False) == mebcan.ACC_HMS_HOLD
    assert self.get_hold_type(override_begin=False, previous_hold_type=mebcan.ACC_HMS_RAMP_RELEASE,
                              v_ego=mebcan.HOLD_RELEASE_SPEED - 0.01) == mebcan.ACC_HMS_RAMP_RELEASE
    assert self.get_hold_type(override_begin=False, previous_hold_type=mebcan.ACC_HMS_RAMP_RELEASE,
                              v_ego=mebcan.HOLD_RELEASE_SPEED) == mebcan.ACC_HMS_NO_REQUEST


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

class TestVolkswagenPlatformConfigs(unittest.TestCase):
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
    radar_ecu = (Ecu.fwdRadar, 0x757, None)
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus.get(radar_ecu, ())})

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
                has_radar_fw = bool(FW_VERSIONS.get(platform, {}).get(radar_ecu))
                should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                                (not platform.config.model_years or model_year in platform.config.model_years) and
                                radar_fw in all_radar_fw and has_radar_fw)

                live_fws = {(0x757, None): [radar_fw]}
                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

                expected_matches = {platform} if should_match else set()
                assert expected_matches == matches, "Bad match"
