from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.leapmotor.values import DBC, CarControllerParams

GearShifter = structs.CarState.GearShifter
GEARS = {0: GearShifter.park, 1: GearShifter.reverse, 2: GearShifter.neutral, 3: GearShifter.drive}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.steering_angle_last = 0.0
    self.lcc_state = 7

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # speed
    self.parse_wheel_speeds(ret,
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FR'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RR'],
    )
    ret.vEgoCluster = cp.vl['CLUSTER']['DISPLAY_SPEED'] * CV.KPH_TO_MS
    ret.standstill = ret.vEgoRaw < 0.01
    ret.yawRate = cp.vl['IMU_CANDIDATE']['YAW_RATE'] * CV.DEG_TO_RAD

    # pedals
    ret.gasPressed = cp.vl['VEHICLE_STATE']['ACCEL_PEDAL'] > 0
    ret.brakePressed = bool(cp.vl['VEHICLE_STATE']['BRAKE_PRESSED'])

    # steering: the EPS speed field has no sign, so take the sign from the angle change
    angle = cp.vl['EPS_STEERING']['STEER_ANGLE']
    sign = 1.0 if angle > self.steering_angle_last else (-1.0 if angle < self.steering_angle_last else 0.0)
    ret.steeringAngleDeg = angle
    ret.steeringRateDeg = sign * cp.vl['EPS_STEERING']['STEER_SPEED_ABS']
    self.steering_angle_last = angle
    ret.steeringTorque = cp.vl['EPS_STEERING']['STEER_TORQUE_CANDIDATE']
    ret.steeringTorqueEps = cp.vl['EPS_STEERING']['EPS_ASSIST_TORQUE_CANDIDATE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)

    # gear
    ret.gearShifter = GEARS.get(int(cp.vl['VEHICLE_STATE']['GEAR']), GearShifter.unknown)

    # body
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl['BODY_CONTROL']['BLINKER_LEFT'] == 1,
                                                                      cp.vl['BODY_CONTROL']['BLINKER_RIGHT'] == 1)
    ret.doorOpen = bool(cp.vl['BODY_CONTROL']['DRIVER_DOOR_OPEN'])
    ret.seatbeltUnlatched = not bool(cp.vl['LONG_ACCEL_CANDIDATE']['SEATBELT_BUCKLED'])

    # stock ACC / LCC, read from the camera side
    ret.cruiseState.enabled = bool(cp.vl['POWERTRAIN_STATE']['ASSIST_ENGAGED'])
    ret.cruiseState.available = ret.cruiseState.enabled  # TODO: no "ACC available" signal found yet
    ret.cruiseState.speed = cp_cam.vl['ADAS_LKA']['ACC_SET_SPEED'] * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = ret.cruiseState.speed
    ret.cruiseState.standstill = ret.cruiseState.enabled and ret.standstill
    self.lcc_state = int(cp_cam.vl['ADAS_LKA']['LCC_STATE'])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
