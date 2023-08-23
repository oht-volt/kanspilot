    if self.CP.carFingerprint in CC_ONLY_CAR:
      ret.accFaulted = False
      ret.cruiseState.speed = pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"] * CV.KPH_TO_MS
      ret.cruiseState.enabled = pt_cp.vl["ECMCruiseControl"]["CruiseActive"] != 0import copy
from cereal import car

from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import DT_CTRL
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import sec_since_boot
from openpilot.common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, STEER_THRESHOLD, GMFlags, CC_ONLY_CAR, CAMERA_ACC_CAR
from openpilot.common.params import Params
import cereal.messaging as messaging

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
GearShifter = car.CarState.GearShifter
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS
CLUSTER_SAMPLE_RATE = 20  # frames

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL2"]["PRNDL2"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0

    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0
    self.vEgo = 0

    #3Bar Distance
    self.prev_distance_button = 0
    self.prev_lkas_enabled = 0
    self.lkas_enabled = 0
    self.distance_button = 0
    self.follow_level = 2
    self.lkMode = True
    #bellow 5lines for Autohold
    self.autoHold = False
    self.autoHoldActive = False
    self.autoHoldActivated = False
    self.lastAutoHoldTime = 0.0
    self.sessionInitTime = sec_since_boot()
    self.regenPaddlePressed = False
    self.cruiseMain = False

    #Engine Rpm
    self.engineRPM = 0

    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.is_metric = False

    self.buttons_counter = 0

    self.single_pedal_mode = False
    self.pedal_steady = 0.
    self.distance_button_pressed = False
    self.gasPressed = False
    #standstill checker
    #self.prev_standstill_status = False
    #self.standstill_status = False
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.totalDistance = 0.0
    self.speedLimitDistance = 0

  def update(self, pt_cp, cam_cp, loopback_cp, chassis_cp): # line for brake light & GM: EPS fault workaround (#22404)
    ret = car.CarState.new_message()

    self.distance_button_pressed = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"] != 0
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    ret.cruiseButtons = self.cruise_buttons
    self.buttons_counter = pt_cp.vl["ASCMSteeringButton"]["RollingCounter"]
    self.pscm_status = copy.copy(pt_cp.vl["PSCMStatus"])
    moving_forward = pt_cp.vl["EBCMWheelSpdRear"]["MovingForward"] != 0
    self.moving_backward = (pt_cp.vl["EBCMWheelSpdRear"]["MovingBackward"] != 0) and not moving_forward

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]["RollingCounter"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["ASCMLKASteeringCmd"]["RollingCounter"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera and not self.CP.flags & GMFlags.NO_CAMERA.value:
      self.pt_lka_steering_cmd_counter = pt_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
      self.cam_lka_steering_cmd_counter = cam_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
    self.is_metric = Params().get_bool("IsMetric")
    self.speed_conv_to_ms = CV.KPH_TO_MS * 1.609344 if self.is_metric else CV.MPH_TO_MS

    # 4 lines for 3Bar Distance
    self.prev_lkas_enabled = self.lkas_enabled
    self.lkas_enabled = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    self.prev_distance_button = self.distance_button
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]

    cluSpeed = pt_cp.vl["ECMVehicleSpeed"]["VehicleSpeed"]
    ret.vEgoCluster = cluSpeed * self.speed_conv_to_ms

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    # ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]) * (106./100.)
    # ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    vEgoRawClu = cluSpeed * self.speed_conv_to_ms
    vEgoClu, aEgoClu = self.update_clu_speed_kf(vEgoRawClu)

    vEgoRawWheel = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]) * (106./100.)
    vEgoRawWheel = interp(vEgoRawWheel, [0., 10.], [(vEgoRawWheel + vEgoRawClu) / 2., vEgoRawWheel])
    vEgoWheel, aEgoWheel = self.update_speed_kf(vEgoRawWheel)

    if self.use_cluster_speed:
      ret.vEgoRaw = vEgoRawClu
      ret.vEgo = vEgoClu
      ret.aEgo = aEgoClu
    else:
      ret.vEgoRaw = vEgoRawWheel
      ret.vEgo = vEgoWheel
      ret.aEgo = aEgoWheel

    ret.vCluRatio = (ret.vEgoCluster / vEgoClu) if (vEgoClu > 3. and ret.vEgoCluster > 3.) else 1.0

    self.vEgo = ret.vEgo

    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = pt_cp.vl["ECMVehicleSpeed"]["VehicleSpeed"]
      self.cluster_speed_counter = 0
    ret.vEgoCluster = self.cluster_speed * self.speed_conv_to_ms

    ret.standstill = ret.wheelSpeeds.rl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    if pt_cp.vl["ECMPRDNL2"]["ManualMode"] == 1:
      ret.gearShifter = self.parse_gear_shifter("T")
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL2"]["PRNDL2"], None))

    if self.CP.flags & GMFlags.NO_CAMERA.value:
      ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    else:
      ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["BrakePressed"] != 0
    else:
      # Some Volt 2016-17 have loose brake pedal push rod retainers which causes the ECM to believe
      # that the brake is being intermittently pressed without user interaction.
      # To avoid a cruise fault we need to use a conservative brake position threshold
      # https://static.nhtsa.gov/odi/tsbs/2017/MC-10137629-9999.pdf
      ret.brakePressed = ret.brake >= 8

    # Regen braking is braking
    if self.CP.transmissionType == TransmissionType.direct:
      ret.regenBraking = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      self.single_pedal_mode = ret.gearShifter == GearShifter.low or pt_cp.vl["EVDriveMode"]["SinglePedalModeActive"] == 1

    if self.CP.enableGasInterceptor:
      ret.gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      threshold = 15 if self.CP.carFingerprint in CAMERA_ACC_CAR else 4
      ret.gasPressed = ret.gas > threshold
    else:
      ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
      ret.gasPressed = ret.gas > 1e-5
    self.gasPressed = ret.gasPressed

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = True #abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerFaultTemporary = self.lkas_status == 2
    ret.steerFaultPermanent = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    ret.parkingBrake = pt_cp.vl["VehicleIgnitionAlt"]["ParkBrake"] == 1
    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]

    ret.brakePressed = ret.brake > 1e-5

    # Regen braking is braking
    if self.CP.transmissionType == TransmissionType.direct:
      self.regenPaddlePressed = bool(pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"])
      ret.brakePressed = ret.brakePressed or self.regenPaddlePressed
      ret.regenBraking = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0

    ret.cruiseState.enabled = self.pcm_acc_status != AccState.OFF
    ret.cruiseState.standstill = False # self.pcm_acc_status == AccState.STANDSTILL

    # bellow 1 line for AutoHold
    self.cruiseMain = ret.cruiseState.available
    if ret.cruiseState.enabled:
      ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.MPH_TO_MS
    else:
      ret.cruiseState.speed = 0
    if self.CP.networkLocation == NetworkLocation.fwdCamera and not self.CP.flags & GMFlags.NO_CAMERA.value:
      if self.CP.carFingerprint not in CC_ONLY_CAR:
        ret.cruiseState.speed = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
      ret.stockAeb = cam_cp.vl["AEBCmd"]["AEBCmdActive"] != 0
      # openpilot controls nonAdaptive when not pcmCruise
      if self.CP.pcmCruise:
        ret.cruiseState.nonAdaptive = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCCruiseState"] not in (2, 3)
    if self.CP.carFingerprint in CC_ONLY_CAR:
      ret.accFaulted = False
      ret.cruiseState.speed = pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"] * CV.KPH_TO_MS
      ret.cruiseState.enabled = pt_cp.vl["ECMCruiseControl"]["CruiseActive"] != 0

    #Cruise Gap
    ret.cruiseGap = self.follow_level

    self.engineRPM = pt_cp.vl["ECMEngineStatus"]["EngineRPM"]
    ret.cruiseState.pcmMode = False

    # bellow line for Brake Light
    ret.brakeLights = chassis_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakePressure"] != 0 or ret.brakePressed

    # bellow Lines are for Autohold
    self.autoHold = True
    # autohold on ui icon
    #if self.CP.enableAutoHold:
    ret.brakeHoldActive = self.autoHoldActivated

    #여기서부터 아래 265라인까지 모두 현기차에만 해당됨. 
    self.totalDistance += ret.vEgo * DT_CTRL
    ret.totalDistance = self.totalDistance
    #맨아래 speedLimit, Distance 값만 road_speed_limiter에 주기 위한 것임
    if self.CP.naviCluster == 1:
      if ret.speedLimit>0:
        if self.speedLimitDistance <= self.totalDistance:
          self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6  #일반적으로 속도*6M 시점에 안내하는것으로 보임.
        self.speedLimitDistance = max(self.totalDistance+1, self.speedLimitDistance) #구간또는 거리가 벗어난경우에는 1M를 유지함.
      else:
        self.speedLimitDistance = self.totalDistance
      ret.speedLimitDistance = self.speedLimitDistance - self.totalDistance
    else:
      ret.speedLimit = 0
      ret.speedLimitDistance = 0

    return ret

  def get_follow_level(self):
    return self.follow_level

  @staticmethod
  def get_cam_can_parser(CP):
    messages = []
    if CP.networkLocation == NetworkLocation.fwdCamera and not CP.flags & GMFlags.NO_CAMERA.value:
      messages += [
        ("AEBCmd", 10),
        ("ASCMLKASteeringCmd", 10),
      ]
      if CP.carFingerprint not in CC_ONLY_CAR:
        messages += [
          ("ASCMActiveCruiseControlStatus", 25),
        ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ("BCMTurnSignals", 1),
      ("ECMPRDNL2", 10),
      ("PSCMStatus", 10),
      ("ESPStatus", 10),
      ("BCMDoorBeltStatus", 10),
      ("VehicleIgnitionAlt", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("EBCMFrictionBrakeStatus", 20),
      ("AcceleratorPedal2", 33),
      ("ASCMSteeringButton", 33),
      ("ECMEngineStatus", 100),
      ("PSCMSteeringAngle", 100),
      ("ECMAcceleratorPos", 80),
      ("EBCMBrakePedalPosition", 100),
      ("ECMVehicleSpeed", 20),
      ("ASCMActiveCruiseControlStatus", 25),

    ]

    # Used to read back last counter sent to PT by camera
    if CP.networkLocation == NetworkLocation.fwdCamera:
      messages += [
        ("ASCMLKASteeringCmd", 0),
      ]
      if CP.flags & GMFlags.NO_CAMERA.value:
        messages.remove(("ECMAcceleratorPos", 80))
        messages.append(("EBCMBrakePedalPosition", 100))

    if CP.transmissionType == TransmissionType.direct:
      messages += [
        ("EBCMRegenPaddle", 50),
        ("EVDriveMode", 0),
      ]

    if CP.carFingerprint in CC_ONLY_CAR:
      messages += [
        ("ECMCruiseControl", 10),
      ]

    if CP.enableGasInterceptor:
      messages += [
        ("GAS_SENSOR", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    messages = [
      ("ASCMLKASteeringCmd", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus.LOOPBACK)

## all bellows are for Brake Light
  @staticmethod
  def get_chassis_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    messages = [
      # sig_name, sig_address, default
      ("FrictionBrakePressure", 100),
    ]
    return CANParser(DBC[CP.carFingerprint]["chassis"], messages, CanBus.CHASSIS, enforce_checks=False)
