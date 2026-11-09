from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1
  # Raw driver torque from EPS_STEERING.STEER_TORQUE_CANDIDATE: about 1-3 hands off, 21 holding full lock.
  # Unit unknown; threshold to be tuned.
  STEER_DRIVER_ALLOWANCE = 8


@dataclass
class LeapmotorCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Centering Control (LCC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class LeapmotorPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'leapmotor_b10',
  })


class CAR(Platforms):
  # mass and wheelbase: Italian brochure, B10 Life with the 56.2 kWh Pro battery (the 67.1 kWh ProMax is 1845 kg).
  # steerRatio 15.2 is measured from slow figure-eights (steering angle vs. yaw rate / speed).
  LEAPMOTOR_B10 = LeapmotorPlatformConfig(
    [LeapmotorCarDocs("Leapmotor B10 2025")],
    CarSpecs(mass=1780, wheelbase=2.735, steerRatio=15.2),
  )


# The car's own gateway queries these identifiers on 0x7DF at every ignition, and the EPS (0x73D), brake (0x73E)
# and front camera (0x75E) answer. Physical request addresses are assumed to be the reply minus 8: UNCONFIRMED.
PART_NUMBER_REQUEST = bytes([0x22, 0xF1, 0x87])
PART_NUMBER_RESPONSE = bytes([0x62, 0xF1, 0x87])

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, PART_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, PART_NUMBER_RESPONSE],
      whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdCamera],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
