#!/usr/bin/env python3
import os
import time
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.hyundai.values import DBC

<<<<<<< HEAD
def get_radar_can_parser(CP):
  signals = [
    # sig_name, sig_address, default
    ("ACC_ObjStatus", "SCC11", 0),
    ("ACC_ObjLatPos", "SCC11", 0),
    ("ACC_ObjDist", "SCC11", 0),
    ("ACC_ObjRelSpd", "SCC11", 0),
  ]
  checks = [
    # address, frequency
    ("SCC11", 50),
  ]
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)
=======
>>>>>>> 3b0189d2a3e43374de226deb576ae49d8bcc69ac


class RadarInterface(RadarInterfaceBase):
  pass