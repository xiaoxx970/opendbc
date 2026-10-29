import numpy as np
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs, make_tester_present_msg
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.carstate import RADAR_ACC_ENGAGED, RADAR_ACC_FAULTED
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlagBits
from opendbc.car.volkswagen.mebutils import LongControlJerk, LongControlLimit

from opendbc.sunnypilot.car.volkswagen.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Stock VW hides the distance popup 2 s after the last button is released. Keep in step with
# openpilot's selfdrive/car/distance_display.py, which decides what those presses do.
DISTANCE_POPUP_FRAMES = 200
# Before going quiet, end HCA_03 the way the camera does: 10 standby frames still flagged high send rate, then
# 5 announcing the drop to its 1 Hz idle rate. Stopping right after an active frame leaves the EPS waiting for
# the next 20 ms frame, and the camera's next idle frame can be a second away.
HCA_STANDBY_HIGH_RATE_FRAMES = 10
HCA_STANDBY_FRAMES = 15
DISTANCE_STEP_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise)
# The radar faults its own ACC when it regulates while TSK stays in standby for more than 0.4 s. Both engage on the press,
# openpilot a few frames later: cancel the radar when openpilot has not followed by now.
RADAR_ACC_CANCEL_FRAMES = 15
# openpilot's ACC waits this long after engaging for the radar's own ACC to engage too, then regulates anyway: the radar
# engages 0.01-0.23 s after the press, and an ACC openpilot can use beats keeping the radar's engine restart
RADAR_ENGAGE_WAIT_FRAMES = 50
# The stock camera's TA_01 after a Travel Assist press that engages (stock drives 08-08, 08-24): key press flag from
# +0.12 to +0.7 s, enable request from +0.3 to +1.35 s. Inferred to be what shows "Travel Assist on" on the cluster.
TRAVEL_ASSIST_PRESSED_FRAMES = (12, 70)
TRAVEL_ASSIST_REQUEST_FRAMES = (30, 135)

# stock radar ACC_19: display prio 1 for 3.00 s from the engage frame (19 stock engagements: 2.99-3.01 s)
ACC_HUD_ENGAGE_PRIO_FRAMES = 300
# stock radar ACC_19: "ACC ready to start" (ACC_Events 3) for 2.50 s once its own ACC has braked the car into a hold, never
# after an engagement at a stop (2026-10-29: 9 of 11 stock stops exactly 2.50 s, none in 27 of 29 engagements at a stop)
ACC_HUD_STOP_TEXT_FRAMES = 250


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP, CP_IC):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP, CP_IC)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.flags = VolkswagenFlagBits(CP.flags)
    self.aeb_available = not self.flags.pq
    # On the gateway the camera is left in place: while openpilot is not steering it stays off HCA_03 and LDW_02,
    # and panda forwards the camera's instead, so stock Lane Assist and Side Assist keep working
    self.stock_lane_assist = self.flags.meb_or_mqb_evo and \
                             CP.networkLocation == structs.CarParams.NetworkLocation.gateway
    self.hca_standby_frames = HCA_STANDBY_FRAMES

    if self.flags.pq:
      self.CCS = pqcan
    elif self.flags.mlb:
      self.CCS = mlbcan
    elif self.flags.meb_or_mqb_evo:
      self.CCS = mebcan
      self.meb_long_state = mebcan.SunnypilotMebLongStateMachine(self.CP, self.CCP)
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.accel_last = 0.
    self.long_jerk_control = LongControlJerk(dt=(DT_CTRL * self.CCP.ACC_CONTROL_STEP)) if self.flags.meb_or_mqb_evo else None
    self.long_limit_control = LongControlLimit(dt=(DT_CTRL * self.CCP.ACC_CONTROL_STEP)) if self.flags.meb_or_mqb_evo else None
    self.gra_acc_counter_last = None
    self.radar_acc_unfollowed_frames = 0
    self.radar_engage_wait_frames = 0  # frames openpilot's ACC has waited for the radar's, 0 = not waiting
    self.enabled_prev = False
    self.hca_mitigation = HCAMitigation(self.CCP)
    self.klr_counter_last = None
    self.lead_distance_bars_last = None
    self.distance_buttons = set()  # distance and +/- presses that belong to the distance popup, until released
    self.distance_button_frame = -DISTANCE_POPUP_FRAMES  # last release of one of them
    self.distance_bar_frame = -DISTANCE_POPUP_FRAMES  # last change of the distance
    self.speed_limit_last = 0
    self.speed_limit_changed_timer = -400  # never treat boot as a freshly detected speed limit
    self.radar_disabled_warning_timer = 0
    self.hide_ea_error = False
    self.enabled_last = False
    self.acc_hud_engaged_last = False
    self.acc_hud_engage_frame = 0
    self.esp_hold_last = False
    self.acc_hud_stop_frame = -ACC_HUD_STOP_TEXT_FRAMES
    self.travel_assist_press_frame_last = None
    self.travel_assist_notify_frame = None

  def travel_assist_notify(self, CS, CC) -> tuple[bool, bool]:
    # replay the stock camera's TA_01 after a Travel Assist press openpilot engaged on; like stock, a press while
    # already engaged shows nothing
    if CS.travel_assist_press_frame != self.travel_assist_press_frame_last:
      self.travel_assist_press_frame_last = CS.travel_assist_press_frame
      self.travel_assist_notify_frame = None if self.enabled_last else CS.travel_assist_press_frame
    self.enabled_last = CC.enabled
    if not CC.enabled or self.travel_assist_notify_frame is None:
      return False, False
    frames = CS.frame - self.travel_assist_notify_frame
    return (TRAVEL_ASSIST_PRESSED_FRAMES[0] <= frames < TRAVEL_ASSIST_PRESSED_FRAMES[1],
            TRAVEL_ASSIST_REQUEST_FRAMES[0] <= frames < TRAVEL_ASSIST_REQUEST_FRAMES[1])

  def update(self, CC, CC_SP, CC_IC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # copy custom data to carstate
    CS.force_rhd_for_bsm = CC_IC.forceRHDForBSM
    CS.enable_predicative_speed_limit = CC_IC.cruiseSpeedLimitPredicative
    CS.enable_pred_react_to_speed_limits = CC_IC.cruiseSpeedLimitPredReactToSL
    CS.enable_pred_react_to_curves = CC_IC.cruiseSpeedLimitPredReactToCurves

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.flags.meb_or_mqb_evo:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if CC.latActive:
          hca_enabled = True
          # no closed loop correction for FORD as long as the current curvature car signal is verified
          steer_correction = CS.out_ic.steeringCurvature - CC.currentCurvature + CC_IC.rollCompensation if not self.flags.ford_car else 0.
          apply_curvature = actuators.curvature + steer_correction
          apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                    CS.out_ic.steeringCurvature, CC.latActive, self.CCP.STEER_STEP)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(abs(CS.out.steeringTorque), [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                                          [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)

        else:
          if self.steering_power_last > 0: # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            apply_curvature = float(np.clip(CS.out_ic.steeringCurvature, -self.CCP.CURVATURE_LIMITS.CURVATURE_MAX, self.CCP.CURVATURE_LIMITS.CURVATURE_MAX)) # synchronize with current curvature
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0. # inactive curvature
            steering_power = 0

        if not self.stock_lane_assist:
          can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        elif hca_enabled:
          self.hca_standby_frames = 0
          can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        elif self.hca_standby_frames < HCA_STANDBY_FRAMES:
          high_send_rate = self.hca_standby_frames < HCA_STANDBY_HIGH_RATE_FRAMES
          self.hca_standby_frames += 1
          can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power,
                                                            high_send_rate=high_send_rate))
        CS.op_steering_requested = hca_enabled or not self.stock_lane_assist
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power
        
      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.
        
        apply_torque = 0
        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
        
        apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
        hca_enabled = apply_torque != 0
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

    # Emergency Assist mitigation
    if self.flags.meb_or_mqb_evo:
      if self.flags.stock_klr_present:
        # send capacitive steering wheel hands-on message to keep ACC resume active and control Emergency Assist
        # MEB Emergency Assist brake jerks after 30s of continued hands-off time.
        # We send the stock wheeltouch message to start the stock DM timer when openpilot latches the critical driver monitoring alert
        klr_send_ready = CS.klr_stock_values["COUNTER"] != self.klr_counter_last
        if klr_send_ready:
          lat_active = CC.latActive and not CC.driverMonitoringEscalation
          can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, lat_active, CS.klr_stock_values))
          can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, lat_active, CS.klr_stock_values))
        self.klr_counter_last = CS.klr_stock_values["COUNTER"]
    else:
      if self.flags.stock_hca_present:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Blinker Controls ************************************************** #
    # "Wechselblinken" has to be allowed in assistance blinker functions in gateway
    # "Wechselblinken" means switching between hazards and one sided indicators for every indicator cycle (VW MEB full cycle: 0.8 seconds, 1st normal, 2nd hazards)
    # user input has hgher prio than EA indicating, post cycle handover is done via actual indicator signal if EA would already request
    # signaling indicators for 1 frame to trigger the first non hazard cycle, retrigger after the car signals a fully ended cycle
    if self.flags.meb_or_mqb_evo and self.flags.stock_ea_present:
      if self.frame % 2 == 0:
        blinker_active = CS.left_blinker_active or CS.right_blinker_active
        left_blinker = CC.leftBlinker if not blinker_active else False
        right_blinker = CC.rightBlinker if not blinker_active else False
        can_sends.append(mebcan.create_blinker_control(self.packer_pt, self.CAN.pt, CS.ea_hud_stock_values, CS.ea_control_stock_values, left_blinker, right_blinker, self.hide_ea_error))
    
    # **** Acceleration Controls ******************************************** #

    if self.flags.meb_or_mqb_evo and CS.out_ic.radarDisableFailed:
      self.meb_long_state.reset()
    
    # The radar's own ACC faults when TSK regulates while it stays in standby for 0.2 s. It engages 0.01-0.03 s after a
    # SET/RES press on the move, but up to 0.23 s after Travel Assist and later still at a stop (2026-10-27 14:19:40,
    # route 223: TSK at +0.08 s, radar still standby, fault at +0.28 s). Keep openpilot's ACC_18 in standby after it
    # engages until the radar has engaged, for at most RADAR_ENGAGE_WAIT_FRAMES.
    radar_acc_healthy = CS.radar_acc_status is not None and CS.radar_acc_status not in RADAR_ACC_FAULTED
    if not CC.enabled or not radar_acc_healthy or CS.radar_acc_status in RADAR_ACC_ENGAGED or \
       self.radar_engage_wait_frames >= RADAR_ENGAGE_WAIT_FRAMES:
      self.radar_engage_wait_frames = 0
    elif not self.enabled_prev or self.radar_engage_wait_frames:
      self.radar_engage_wait_frames += 1
    self.enabled_prev = CC.enabled
    long_cc = CC if not self.radar_engage_wait_frames else SimpleNamespace(
      enabled=False, longActive=False, cruiseControl=CC.cruiseControl, actuators=CC.actuators,
    )

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl and not CS.out_ic.radarDisableFailed:
      stopping = actuators.longControlState == LongCtrlState.stopping
        
      if self.flags.meb_or_mqb_evo:
        accel_request = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX))
        accel, acc_control, acc_hold_type, braking_to_stop, leaving_standstill, held = self.meb_long_state.update(CS, long_cc, accel_request)

        critical_state = hud_control.visualAlert == VisualAlert.fcw
        if CC_IC.longComfortMode:
          driver_override = CS.out.gasPressed and not CS.out.brakePressed
          self.long_jerk_control.update(self.meb_long_state.acc_enabled, driver_override,
                                        CC_IC.hudLeadDistance, hud_control.leadVisible,
                                        accel_request if self.meb_long_state.acc_enabled and not driver_override else 0.0, critical_state)
          self.long_limit_control.update(self.meb_long_state.acc_enabled, CS.out.vEgoRaw, hud_control.setSpeed,
                                         CC_IC.hudLeadDistance, hud_control.leadVisible, critical_state)

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, self.CP, CS.acc_type,
                                                           self.meb_long_state.acc_enabled,
                                                           self.long_jerk_control.get_jerk_up() if CC_IC.longComfortMode else self.CCP.JERK_LIMIT,
                                                           self.long_jerk_control.get_jerk_down() if CC_IC.longComfortMode else self.CCP.JERK_LIMIT,
                                                           self.long_limit_control.get_upper_limit() if CC_IC.longComfortMode else 0.,
                                                           self.long_limit_control.get_lower_limit() if CC_IC.longComfortMode else 0.,
                                                           accel, acc_control, acc_hold_type, braking_to_stop, leaving_standstill, held,
                                                           CS.out.vEgoRaw * CV.MS_TO_KPH, CS.travel_assist_available,
                                                           self.meb_long_state.start_stop_info,
                                                           self.travel_assist_notify(CS, CC),
                                                           CS.acc_stock_values if self.flags.mqb_evo and CS.out.standstill else None))

      else:
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < 0.25)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))
      self.accel_last = accel

    #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))
        
    # **** Radar disable **************************************************** #
    # Send radar replacement messages cruise state relevant
    # Disables Autonomous Emergency Braking (AEB), Front Collision Warning (FCW), Emergency Assist (EA),
    # Predicative Speed Control, (for MQBevo Traffic Sign Detection)
    # Dash warnings for critical deactivations are shown for several seconds
    
    if self.flags.disable_radar and self.CP.openpilotLongitudinalControl and not CS.out_ic.radarDisableFailed:
      if self.flags.meb_or_mqb_evo:
        if self.radar_disabled_warning_timer < 600: # display critical hud warnings for some seconds
          self.radar_disabled_warning_timer += 1
        else:
          self.hide_ea_error = True # block EA error after several seconds
          
        if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
          can_sends.append(make_tester_present_msg(0x700, self.CAN.pt, suppress_response=True)) # Tester Present to keep the programming session
          can_sends.append(self.CCS.create_aeb_control(self.packer_pt, self.CAN.pt, self.CP)) # AEB Control (1 Hz)
          
        if self.frame % self.CCP.AEB_HUD_STEP == 0:
          can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, self.CAN.pt, self.radar_disabled_warning_timer < 600)) # AEB HUD (5 Hz), show deactivation for several seconds

        if self.frame % 4 == 0:
          can_sends.append(self.CCS.create_radar_objects(self.packer_pt, self.CAN.pt)) # Radar Objects (25 Hz)

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]

      # a take-over alert still goes out after openpilot stopped steering
      stock_ldw = self.stock_lane_assist and not CC.latActive and hud_alert == 0
      if self.flags.meb_or_mqb_evo:
        if not stock_ldw:
          sound_alert = self.CCP.LDW_SOUNDS["Chime"] if hud_alert == self.CCP.LDW_MESSAGES["laneAssistTakeOver"] and not CC_IC.disableCarSteerAlerts else self.CCP.LDW_SOUNDS["None"]
          can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, self.CP, CS.ldw_stock_values, CC.latActive,
                                                           CS.out.steeringPressed, hud_alert, hud_control, sound_alert))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    # Distance popup like stock VW: a distance press shows it, +/- pressed while it is shown adjust
    # the distance instead of the set speed, and it closes 2 s after the last of those is released
    if CS.out.cruiseState.available:
      for b in CS.out.buttonEvents:
        showing = bool(self.distance_buttons) or self.frame - self.distance_button_frame < DISTANCE_POPUP_FRAMES
        if b.pressed and (b.type == ButtonType.gapAdjustCruise or (b.type in DISTANCE_STEP_BUTTONS and showing)):
          self.distance_buttons.add(b.type)
        elif not b.pressed and b.type in self.distance_buttons:
          self.distance_buttons.discard(b.type)
          self.distance_button_frame = self.frame
    else:
      self.distance_buttons.clear()
      self.distance_button_frame = -DISTANCE_POPUP_FRAMES

    # also show a change made elsewhere, but not on the first frame after boot
    if self.lead_distance_bars_last is not None and hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl and not CS.out_ic.radarDisableFailed:
      if self.flags.meb_or_mqb_evo:
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        show_distance_bars = bool(self.distance_buttons) or self.frame - self.distance_button_frame < DISTANCE_POPUP_FRAMES or \
                             self.frame - self.distance_bar_frame < DISTANCE_POPUP_FRAMES
        gap = max(8, CS.out.vEgo * CC_IC.hudLeadFollowTime)
        distance = max(8, CC_IC.hudLeadDistance) if CC_IC.hudLeadDistance != 0 else 0

        # Use the same state as ACC_18 so HUD and drivetrain status cannot diverge.
        acc_hud_status = self.meb_long_state.acc_status
        # stock radar: medium display prio for 3 s from the engagement, which keeps the cluster's engage text up
        acc_engaged = acc_hud_status in (mebcan.ACC_HUD_ACTIVE, mebcan.ACC_HUD_OVERRIDE)
        if acc_engaged and not self.acc_hud_engaged_last:
          self.acc_hud_engage_frame = self.frame
        self.acc_hud_engaged_last = acc_engaged
        engage_prio = acc_engaged and self.frame - self.acc_hud_engage_frame < ACC_HUD_ENGAGE_PRIO_FRAMES
        # the hold began while openpilot's ACC was regulating, so it braked the car to this stop: show ready to start
        if CS.esp_hold_confirmation and not self.esp_hold_last and acc_hud_status == mebcan.ACC_HUD_ACTIVE:
          self.acc_hud_stop_frame = self.frame
        self.esp_hold_last = CS.esp_hold_confirmation
        acc_hud_stop_text = CS.esp_hold_confirmation and self.frame - self.acc_hud_stop_frame < ACC_HUD_STOP_TEXT_FRAMES

        sl_predicative_active = True if CC_IC.cruiseSpeedLimitPredicative and CS.out_ic.cruiseSpeedLimitPredicative != 0 else False

        # Emulate the stock pACC cluster prompts from openpilot's own data sources:
        # - camera speed limit (VZE_04 -> cruiseSpeedLimit) has priority, then sunnypilot's map speed limit
        # - a newly detected/changed limit is shown for a few seconds (independent of EnableSpeedLimitControl,
        #   that toggle only gates whether cruise.py adopts the limit as the set speed)
        # - an upcoming map speed limit is shown as "speed limit ahead" while approaching it
        # - vision curve control (SCC-V) slowing for a curve is shown as "curve" with its target speed
        camera_limit = CS.out_ic.cruiseSpeedLimit
        map_limit = CC_IC.hudSpeedLimit if CC_IC.hudSpeedLimitFromMap else 0
        sl_current = camera_limit if camera_limit != 0 else map_limit
        if sl_current != 0 and self.speed_limit_last != sl_current:
          self.speed_limit_changed_timer = self.frame
        self.speed_limit_last = sl_current
        sl_active = self.frame - self.speed_limit_changed_timer < 400
        sl_ahead_active = CC_IC.hudSpeedLimitFromMap and CC_IC.hudSpeedLimitAhead and CC_IC.hudSpeedLimit != 0
        curve_active = CC_IC.hudCurveSpeed != 0 and CC_IC.hudCurveSpeed < hud_control.setSpeed

        if curve_active:
          speed_limit = CC_IC.hudCurveSpeed
        elif sl_predicative_active:
          speed_limit = CS.out_ic.cruiseSpeedLimitPredicative
        elif sl_ahead_active:
          speed_limit = CC_IC.hudSpeedLimit
        elif sl_active:
          speed_limit = sl_current
        else:
          speed_limit = 0

        acc_hud_event = self.CCS.get_acc_hud_event(acc_hud_status, acc_hud_stop_text, sl_predicative_active, CS.speed_limit_predicative_type, sl_active,
                                                   curve_active, sl_ahead_active, CC_IC.hudCurveDirection)

        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                         hud_control.leadVisible, hud_control.leadDistanceBars + 1, show_distance_bars,
                                                         CS.esp_hold_confirmation, distance, gap, fcw_alert, acc_hud_event, speed_limit,
                                                         self.flags.mqb_evo, self.frame // self.CCP.ACC_HUD_STEP,
                                                         CS.neighbour_lead_distance, CS.radar_acc_hud, engage_prio))

      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
        # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    # The radar engaged its own ACC but openpilot did not (a no entry). Cancel it, as the driver would, before its
    # comparison with TSK faults it for the rest of the drive.
    if CS.radar_acc_status in RADAR_ACC_ENGAGED and not CC.enabled:
      self.radar_acc_unfollowed_frames += 1
    else:
      self.radar_acc_unfollowed_frames = 0
    radar_acc_cancel = self.radar_acc_unfollowed_frames > RADAR_ACC_CANCEL_FRAMES

    gra_send_ready = CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready:
      bus_send = self.CAN.pt if self.flags.pq else self.CAN.ext
      if self.CP.pcmCruise:
        if CC.cruiseControl.cancel or CC.cruiseControl.resume:
          can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, bus_send, CS.gra_stock_values,
                                                               cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))
        else: # Intelligent Cruise Button Management
          can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, bus_send))
      elif radar_acc_cancel:
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, bus_send, CS.gra_stock_values, cancel=True))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last
    new_actuators.speed = actuators.speed

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
