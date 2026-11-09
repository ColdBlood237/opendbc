from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.leapmotor import leapmotorcan
from opendbc.car.leapmotor.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # the safety model's vehicle-model limits are computed for this platform
  from opendbc.car.leapmotor.interface import CarInterface
  return CarInterface.get_non_essential_params("LEAPMOTOR_B10")


class CarController(CarControllerBase):
  """Replaces the camera's ADAS_LKA (0x83) with openpilot's angle request.

  Nothing reaches the car while the interface stays dashcamOnly with noOutput safety. Before that changes,
  the camera-unplug test must confirm the camera sends 0x83, and the first transmit is the owner's decision.
  """

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle_last = 0.
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if self.frame % CarControllerParams.STEER_STEP == 0 and CS.adas_lka is not None:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
      can_sends.append(leapmotorcan.create_steering_control(self.packer, CS.adas_lka, self.apply_angle_last, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
