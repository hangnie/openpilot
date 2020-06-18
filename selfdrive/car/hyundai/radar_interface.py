#!/usr/bin/env python3
import os
import time
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.hyundai.values import DBC


class RadarInterface(RadarInterfaceBase):
  pass