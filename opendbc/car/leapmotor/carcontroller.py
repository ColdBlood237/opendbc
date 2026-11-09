from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  """Read-only draft: sends nothing until a safety model and the steering frame are validated."""

  def update(self, CC, CS, now_nanos):
    self.frame += 1
    return CC.actuators.as_builder(), []
