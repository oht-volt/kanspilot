import copy
from cereal import car

from common.numpy_fast import interp
from common.realtime import DT_CTRL
from common.conversions import Conversions as CV
from common.realtime import sec_since_boot
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, STEER_THRESHOLD, CruiseButtons
from common.params import Params
import cereal.messaging as messaging

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
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

    # lead_distance
    self.lead_distance = 0
    self.sm = messaging.SubMaster(["radarState"])
    self.buttons_counter = 0
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

    self.sm.update(0)
    if self.sm.updated["radarState"]:
      self.lead_distance = 0.0
      lead = self.sm["radarState"].leadOne
      if lead is not None and lead.status:
        self.lead_distance = lead.dRel
        #print("Dist_dRel={:.1f}".format(self.lead_distance))
        ret.diffDistance = self.lead_distance

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    ret.cruiseButtons = self.cruise_buttons
    self.buttons_counter = pt_cp.vl["ASCMSteeringButton"]["RollingCounter"]
    self.pscm_status = copy.copy(pt_cp.vl["PSCMStatus"])
    self.moving_backward = pt_cp.vl["EBCMWheelSpdRear"]["MovingBackward"] != 0

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]["RollingCounter"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["ASCMLKASteeringCmd"]["RollingCounter"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
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
    #This brake position value disengages stock ACC, use it to avoid control mismatch.
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0

    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["BrakePressed"] != 0
    else:
      # Some Volt 2016-17 have loose brake pedal push rod retainers which causes the ECM to believe
      # that the brake is being intermittently pressed without user interaction.
      # To avoid a cruise fault we need to use a conservative brake position threshold
      # https://static.nhtsa.gov/odi/tsbs/2017/MC-10137629-9999.pdf
      ret.brakePressed = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] >= 8
    if ret.brake < 10/0xd0:
      ret.brake = 0.

    ret.tpms.fl = 36 #TODO:
    ret.tpms.fr = 36 #TODO:
    ret.tpms.rl = 36 #TODO:
    ret.tpms.rr = 36 #TODO:

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

    #standstill check
    #self.prev_standstill_status = self.standstill_status
    #self.standstill_status = ret.cruiseState.standstill # self.pcm_acc_status == AccState.STANDSTILL
    #print("standstill={}".format(self.standstill_status))

    # bellow 1 line for AutoHold
    self.cruiseMain = ret.cruiseState.available
    if ret.cruiseState.enabled:
      ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.MPH_TO_MS
    else:
      ret.cruiseState.speed = 0
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.cruiseState.speed = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
      ret.stockAeb = cam_cp.vl["AEBCmd"]["AEBCmdActive"] != 0
      # openpilot controls nonAdaptive when not pcmCruise
      if self.CP.pcmCruise:
        ret.cruiseState.nonAdaptive = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCCruiseState"] not in (2, 3)

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

    #���⼭���� �Ʒ� 265���α��� ��� ���������� �ش��. 
    self.totalDistance += ret.vEgo * DT_CTRL
    ret.totalDistance = self.totalDistance
    #�ǾƷ� speedLimit, Distance ���� road_speed_limiter�� �ֱ� ���� ����
    if self.CP.naviCluster == 1:
      if ret.speedLimit>0:
        if self.speedLimitDistance <= self.totalDistance:
          self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6  #�Ϲ������� �ӵ�*6M ������ �ȳ��ϴ°����� ����.
        self.speedLimitDistance = max(self.totalDistance+1, self.speedLimitDistance) #�����Ǵ� �Ÿ��� �����쿡�� 1M�� ������.
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
    signals = []
    checks = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("AEBCmdActive", "AEBCmd"),
        ("RollingCounter", "ASCMLKASteeringCmd"),
        ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
        ("ACCCruiseState", "ASCMActiveCruiseControlStatus"),
      ]
      checks += [
        ("AEBCmd", 10),
        ("ASCMLKASteeringCmd", 10),
        ("ASCMActiveCruiseControlStatus", 25),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("BrakePedalPosition", "EBCMBrakePedalPosition"),
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("TurnSignals", "BCMTurnSignals"),
      ("AcceleratorPedal2", "AcceleratorPedal2"),
      ("CruiseState", "AcceleratorPedal2"),
      ("ACCButtons", "ASCMSteeringButton"),
      ("LKAButton", "ASCMSteeringButton", 0),
      ("RollingCounter", "ASCMSteeringButton"),
      ("DistanceButton", "ASCMSteeringButton"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("MovingBackward", "EBCMWheelSpdRear"),
      ("FrictionBrakeUnavailable", "EBCMFrictionBrakeStatus"),
      ("PRNDL2", "ECMPRDNL2"),
      ("ManualMode", "ECMPRDNL2"),
      ("LKADriverAppldTrq", "PSCMStatus"),
      ("LKATorqueDelivered", "PSCMStatus"),
      ("LKATorqueDeliveredStatus", "PSCMStatus"),
      ("HandsOffSWlDetectionStatus", "PSCMStatus"),
      ("HandsOffSWDetectionMode", "PSCMStatus"),
      ("LKATotalTorqueDelivered", "PSCMStatus"),
      ("PSCMStatusChecksum", "PSCMStatus"),
      ("RollingCounter", "PSCMStatus"),
      ("TractionControlOn", "ESPStatus"),
      ("ParkBrake", "VehicleIgnitionAlt"),
      ("CruiseMainOn", "ECMEngineStatus"),
      ("BrakePressed", "ECMEngineStatus"),
      ("VehicleSpeed", "ECMVehicleSpeed"),
      ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
      ("EngineRPM", "ECMEngineStatus"),
    ]

    checks = []

    # Used to read back last counter sent to PT by camera
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("RollingCounter", "ASCMLKASteeringCmd"),
      ]
      checks += [
        ("ASCMLKASteeringCmd", 0),
      ]

    if CP.transmissionType == TransmissionType.direct:
      signals.append(("RegenPaddle", "EBCMRegenPaddle"))
      checks.append(("EBCMRegenPaddle", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN, enforce_checks=False)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd"),
    ]

    checks = []

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK, enforce_checks=False)

## all bellows are for Brake Light
  @staticmethod
  def get_chassis_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("FrictionBrakePressure", "EBCMFrictionBrakeStatus"),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]["chassis"], signals, checks, CanBus.CHASSIS, enforce_checks=False)
