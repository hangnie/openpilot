# functions common among cars
from common.numpy_fast import clip

# kg of standard extra cargo to count for drive, gas, etc...
STD_CARGO_KG = 136.

def gen_empty_fingerprint():
  return {i: {} for i in range(0, 4)}

# FIXME: hardcoding honda civic 2016 touring params so they can be used to
# scale unknown params for other cars
class CivicParams:
  MASS = 1326. + STD_CARGO_KG
  WHEELBASE = 2.70
  CENTER_TO_FRONT = WHEELBASE * 0.4
  CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
  ROTATIONAL_INERTIA = 2500
  TIRE_STIFFNESS_FRONT = 192150
  TIRE_STIFFNESS_REAR = 202500

# TODO: get actual value, for now starting with reasonable value for
# civic and scaling by mass and wheelbase
def scale_rot_inertia(mass, wheelbase):
  return CivicParams.ROTATIONAL_INERTIA * mass * wheelbase ** 2 / (CivicParams.MASS * CivicParams.WHEELBASE ** 2)

# TODO: start from empirically derived lateral slip stiffness for the civic and scale by
# mass and CG position, so all cars will have approximately similar dyn behaviors
def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
  center_to_rear = wheelbase - center_to_front
  tire_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / CivicParams.MASS * \
                         (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)

  tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / CivicParams.MASS * \
                        (center_to_front / wheelbase) / (CivicParams.CENTER_TO_FRONT / CivicParams.WHEELBASE)

  return tire_stiffness_front, tire_stiffness_rear

def dbc_dict(pt_dbc, radar_dbc, chassis_dbc=None):
  return {'pt': pt_dbc, 'radar': radar_dbc, 'chassis': chassis_dbc}


def apply_std_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + (LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  driver_min_torque = -LIMITS.STEER_MAX + (-LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                                    apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                                    min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


def apply_toyota_steer_torque_limits(apply_torque, apply_torque_last, motor_torque, LIMITS):
  # limits due to comparison of commanded torque VS motor reported torque
  max_lim = min(max(motor_torque + LIMITS.STEER_ERROR_MAX, LIMITS.STEER_ERROR_MAX), LIMITS.STEER_MAX)
  min_lim = max(min(motor_torque - LIMITS.STEER_ERROR_MAX, -LIMITS.STEER_ERROR_MAX), -LIMITS.STEER_MAX)

  apply_torque = clip(apply_torque, min_lim, max_lim)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque,
                        max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque,
                        apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


def crc8_pedal(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size-1, -1, -1):
    crc ^= data[i]
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_gas_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 0, values)


def is_ecu_disconnected(fingerprint, fingerprint_list, ecu_fingerprint, car, ecu):
  # is_ecu_disconnected(i40 핑거프린터, 전체핑거프린터, {3: {832, 1156, 1191, 1342]}, HYUNDAI I40 2012 1.7D, 3):
  # check if a stock ecu is disconnected by looking for specific CAN msgs in the fingerprint
  # return True if the reference car fingerprint contains the ecu fingerprint msg and
  # fingerprint does not contains messages normally sent by a given ecu
  # (지문에서 특정 CAN msgs를 찾아 스톡 ECU가 분리되었는지 점검하십시오.)
  # (참조 차량 지문에 ecu 지문 메시지가 포함되어 있으면 True를 반환하고)
  # (지정된 ecu가 일반적으로 보낸 메시지를 지문에 포함하지 않습니다)
  
  # for 함수 : 순서형 자료(list)를 반복 (for x family => family 라는 목록에서 각각의 항목을 x에 넣음)
  # any() 함수 : 인자중에 True(1)가 하나라도 있으면 True
  # in 함수 : 특정한 문자열 안에 찾고자 하는 문자열이 있으면 True ('전체 문자열(Welcome)' in '찾고자하는 문자열(come)' = True)

  print("===[init]> fingerprint      : ", fingerprint)
  # print("===[init]> fingerprint_list : ", fingerprint_list)
  print("===[init]> ecu_fingerprint  : ", ecu_fingerprint)
  print("===[init]> car              : ", car)
  print("===[init]> ecu              : ", ecu)
  
  ecu_in_car = False
  print("===[init]> ecu_in_car1      : ", ecu_in_car)

  for car_finger in fingerprint_list[car]:   # 전체 핑거프린트 중 내 차량의 핑거크린트를 찾아 항목 하나씩 꺼내여 car_finger 에 넣음
    print("===[init]> car_finger       : ", car_finger)
    if any(msg in car_finger for msg in ecu_fingerprint[ecu]):
      ecu_in_car = True
      print("===[init]> ecu_in_car2      : ", ecu_in_car)

   
    #for msg in ecu_fingerprint[ecu]:         # ecu 핑거프린트 항목 중 하나씩 꺼내어 msg에 넣고 
    #  print("===[init]> msg              : ", msg)

      #if any(car_finger in msg):             # any, in 함수로 msg와 아까 꺼낸 car_finger와 비교
      #  ecu_in_car = True
      #  print("===[init]> msg              : ", msg)
      #  print("===[init]> ecu_in_car2      : ", ecu_in_car)
    
    print("=== for end ===")
    

  return ecu_in_car and not any(msg in fingerprint for msg in ecu_fingerprint[ecu])


def make_can_msg(addr, dat, bus):
  return [addr, 0, dat, bus]

