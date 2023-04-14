from common.log import Loger
from cereal import car, log
from common.conversions import Conversions as CV
from common.numpy_fast import interp, clip
from common.realtime import DT_CTRL
import math
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, AccState, CanBus, CarControllerParams, CruiseButtons, CC_ONLY_CAR
from selfdrive.controls.lib.drive_helpers import apply_deadzone
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
import cereal.messaging as messaging
from common.params import Params
from selfdrive.ntune import ntune_scc_get, ntune_scc_enabled

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
VisualAlert = car.CarControl.HUDControl.VisualAlert
NetworkLocation = car.CarParams.NetworkLocation
LongCtrlState = car.CarControl.Actuators.LongControlState
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15


def actuator_hystereses(final_pedal, pedal_steady):
  # hyst params... TODO: move these to VehicleParams
  pedal_hyst_gap = 0.01    # don't change pedal command for small oscillations within this value

  # for small pedal oscillations within pedal_hyst_gap, don't change the pedal command
  if math.isclose(final_pedal, 0.0):
    pedal_steady = 0.
  elif final_pedal > pedal_steady + pedal_hyst_gap:
    pedal_steady = final_pedal - pedal_hyst_gap
  elif final_pedal < pedal_steady - pedal_hyst_gap:
    pedal_steady = final_pedal + pedal_hyst_gap
  final_pedal = pedal_steady

  return final_pedal, pedal_steady


class CarController:

  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_gas = 0
    self.apply_brake = 0

    self.frame = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.cancel_counter = 0

    self.lka_steering_cmd_counter = 0
    self.sent_lka_steering_cmd = False
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams(self.CP)

    # DisableDisengageOnGas
    self.disengage_on_gas = not Params().get_bool("DisableDisengageOnGas")

    # stop at Stopsignal
    self.stopsign_enabled = ntune_scc_enabled('StopAtStopSign')
    self.sm = messaging.SubMaster(['controlsState', 'longitudinalPlan', 'radarState'])
    self.log = Loger()
    self.e2e_standstill_enable = Params().get_bool("DepartChimeAtResume")
    self.e2e_standstill = False
    self.e2e_standstill_stat = False
    self.e2e_standstill_timer = 0

    self.packer = CANPacker(dbc_name)
    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint]['pt'])
    self.packer_obj = CANPacker(DBC[self.CP.carFingerprint]['radar'])
    self.packer_ch = CANPacker(DBC[self.CP.carFingerprint]['chassis'])

  def update(self, CC, CS, enabled):
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # Send CAN commands.
    can_sends = []

    # Steering (Active: 50Hz, inactive: 10Hz)
    # Attempt to sync with camera on startup at 50Hz, first few msgs are blocked
    init_lka_counter = not self.sent_lka_steering_cmd and self.CP.networkLocation == NetworkLocation.fwdCamera
    steer_step = self.params.INACTIVE_STEER_STEP
    if CC.latActive or init_lka_counter:
      steer_step = self.params.ACTIVE_STEER_STEP

    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.loopback_lka_steering_cmd_updated:
      self.lka_steering_cmd_counter += 1
      self.sent_lka_steering_cmd = True
    elif (self.frame - self.last_steer_frame) >= steer_step:
      # Initialize ASCMLKASteeringCmd counter using the camera until we get a msg on the bus
      if init_lka_counter:
        self.lka_steering_cmd_counter = CS.cam_lka_steering_cmd_counter + 1
      lkas_enabled = (CC.latActive or CS.pause_long_on_gas_press) and CS.lkMode and CS.out.vEgo > self.params.MIN_STEER_SPEED
      if CC.latActive:
        new_steer = int(round(actuators.steer * self.params.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      self.last_steer_frame = self.frame
      self.apply_steer_last = apply_steer
      idx = self.lka_steering_cmd_counter % 4
      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    if self.CP.openpilotLongitudinalControl:
      # Gas/regen, brakes, and UI commands - all at 25Hz
      if (self.frame % 4) == 0:
        if not CC.longActive or CS.pause_long_on_gas_press:
          # ASCM sends max regen when not enabled
          self.apply_gas = self.params.INACTIVE_REGEN
          self.apply_brake = 0
        else:
          self.apply_gas = int(round(interp(actuators.accel, self.params.GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
          self.apply_brake = int(round(interp(actuators.accel, self.params.BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))

        idx = (self.frame // 4) % 4

        if CS.cruiseMain and not CC.longActive and CS.autoHold and CS.autoHoldActive and \
             not CS.out.gasPressed and CS.out.gearShifter in ['drive','low'] and \
             CS.out.vEgo < 0.01 and not CS.regenPaddlePressed and CS.autoholdBrakeStart:
          # Auto Hold State
          car_stopping = self.apply_gas < self.params.ZERO_GAS
          at_full_stop = CS.out.standstill and car_stopping
          friction_brake_bus = CanBus.CHASSIS
          # GM Camera exceptions
          # TODO: can we always check the longControlState?
          if self.CP.networkLocation == NetworkLocation.fwdCamera:
            friction_brake_bus = CanBus.POWERTRAIN
          near_stop = (CS.out.vEgo < self.params.NEAR_STOP_BRAKE_PHASE) and car_stopping
          can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, friction_brake_bus, self.apply_brake, idx, CC.enabled, near_stop, at_full_stop, self.CP))
          CS.autoHoldActivated = True
        else:  
          if CS.out.gasPressed:
            at_full_stop = False
            near_stop = False
            car_stopping = False
          else:
            at_full_stop = CC.longActive and CS.out.standstill
            near_stop = CC.longActive and (CS.out.vEgo < self.params.NEAR_STOP_BRAKE_PHASE)
          friction_brake_bus = CanBus.CHASSIS
          # GM Camera exceptions
          # TODO: can we always check the longControlState?
          if self.CP.networkLocation == NetworkLocation.fwdCamera:
            at_full_stop = at_full_stop and actuators.longControlState == LongCtrlState.stopping
            friction_brake_bus = CanBus.POWERTRAIN

          # GasRegenCmdActive needs to be 1 to avoid cruise faults. It describes the ACC state, not actuation
          can_sends.append(gmcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, self.apply_gas, idx, CC.enabled and CS.out.cruiseState.enabled, at_full_stop))
          can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, friction_brake_bus, self.apply_brake, idx, CC.enabled, near_stop, at_full_stop, self.CP))
          CS.autoHoldActivated = False


        # Send dashboard UI commands (ACC status), 25hz
        send_fcw = hud_alert == VisualAlert.fcw
        follow_level = CS.get_follow_level()
        can_sends.append(gmcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, CC.enabled and CS.out.cruiseState.enabled, \
                   hud_v_cruise * CV.MS_TO_KPH, hud_control.leadVisible, send_fcw, follow_level))

      if CC.longActive:
        can_sends.append(gmcan.create_buttons(self.packer_pt, CanBus.CAMERA, CS.buttons_counter, CruiseButtons.RES_ACCEL))

      elif CC.longActive:
        can_sends.append(gmcan.create_buttons(self.packer_pt, CanBus.CAMERA, CS.buttons_counter, CruiseButtons.DECEL_SET))
			        
      # Radar needs to know current speed and yaw rate (50hz),
      # and that ADAS is alive (10hz)
      if not self.CP.radarOffCan:
        tt = self.frame * DT_CTRL
        time_and_headlights_step = 10
        if self.frame % time_and_headlights_step == 0:
          idx = (self.frame // time_and_headlights_step) % 4
          can_sends.append(gmcan.create_adas_time_status(CanBus.OBSTACLE, int((tt - self.start_time) * 60), idx))
          can_sends.append(gmcan.create_adas_headlights_status(self.packer_obj, CanBus.OBSTACLE))

        speed_and_accelerometer_step = 2
        if self.frame % speed_and_accelerometer_step == 0:
          idx = (self.frame // speed_and_accelerometer_step) % 4
          can_sends.append(gmcan.create_adas_steering_status(CanBus.OBSTACLE, idx))
          can_sends.append(gmcan.create_adas_accelerometer_speed_status(CanBus.OBSTACLE, CS.out.vEgo, idx))

      if self.CP.networkLocation == NetworkLocation.gateway and self.frame % self.params.ADAS_KEEPALIVE_STEP == 0:
        can_sends += gmcan.create_adas_keepalive(CanBus.POWERTRAIN)

    else:
      # While car is braking, cancel button causes ECM to enter a soft disable state with a fault status.
      # A delayed cancellation allows camera to cancel and avoids a fault when user depresses brake quickly
      self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

      # Stock longitudinal, integrated at camera
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.04:
        if self.cancel_counter > CAMERA_CANCEL_DELAY_FRAMES:
          self.last_button_frame = self.frame
          can_sends.append(gmcan.create_buttons(self.packer_pt, CanBus.CAMERA, CS.buttons_counter, CruiseButtons.CANCEL))

    if self.CP.networkLocation == NetworkLocation.fwdCamera and self.CP.carFingerprint not in CC_ONLY_CAR:
      # Silence "Take Steering" alert sent by camera, forward PSCMStatus with HandsOffSWlDetectionStatus=1
      if self.frame % 10 == 0:
        can_sends.append(gmcan.create_pscm_status(self.packer_pt, CanBus.CAMERA, CS.pscm_status))

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)

    # SW_GMLAN not yet on cam harness, no HUD alerts
    if self.CP.networkLocation != NetworkLocation.fwdCamera and (self.frame % self.params.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last):
      steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake

    self.frame += 1
    return new_actuators, can_sends

    #opkr
    if self.e2e_standstill_enable:
      try:
        self.sm.update(0)

        if self.e2e_standstill:
          self.e2e_standstill_timer += 1
          if self.e2e_standstill_timer > 500:
            self.e2e_standstill = False
            self.e2e_standstill_timer = 0
        elif CS.v_Ego > 0:
          self.e2e_standstill = False
          self.e2e_standstill_stat = False
          self.e2e_standstill_timer = 0
        elif self.e2e_standstill_stat and self.sm['longitudinalPlan'].trafficState != 1 and CS.v_Ego == 0:
          self.e2e_standstill = True
          self.e2e_standstill_stat = False
          self.e2e_standstill_timer = 0
        elif self.sm['longitudinalPlan'].trafficState == 1 and self.sm['longitudinalPlan'].stopLine[12] < 10 and CS.v_Ego == 0:
          self.e2e_standstill_timer += 1
          if self.e2e_standstill_timer > 300:
            self.e2e_standstill_timer = 101
            self.e2e_standstill_stat = True
        else:
          self.e2e_standstill_timer = 0
      except:
        pass