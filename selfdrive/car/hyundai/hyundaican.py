import crcmod
from selfdrive.car.hyundai.values import CAR, CHECKSUM

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

def create_lkas11(packer, car_fingerprint, bus, apply_steer, steer_req, cnt, enabled, lkas11, hud_alert,
                                   lane_visible, left_lane_depart, right_lane_depart, keep_stock=False):
  values = {
    "CF_Lkas_Bca_R": lkas11["CF_Lkas_Bca_R"] if keep_stock else 3,
    #"CF_Lkas_LdwsSysState": 3 if steer_req else lane_visible,
    "CF_Lkas_LdwsSysState": 3 if enabled else 1,
    "CF_Lkas_SysWarning": hud_alert,
    "CF_Lkas_LdwsLHWarning": lkas11["CF_Lkas_LdwsLHWarning"],
    "CF_Lkas_LdwsRHWarning": lkas11["CF_Lkas_LdwsRHWarning"],
    "CF_Lkas_HbaLamp": lkas11["CF_Lkas_HbaLamp"] if keep_stock else 0,
    "CF_Lkas_FcwBasReq": lkas11["CF_Lkas_FcwBasReq"] if keep_stock else 0,
    "CR_Lkas_StrToqReq": apply_steer,
    "CF_Lkas_ActToi": steer_req,
    "CF_Lkas_ToiFlt": 0,
    "CF_Lkas_HbaSysState": lkas11["CF_Lkas_HbaSysState"] if keep_stock else 1,
    "CF_Lkas_FcwOpt": lkas11["CF_Lkas_FcwOpt"] if keep_stock else 0,
    "CF_Lkas_HbaOpt": lkas11["CF_Lkas_HbaOpt"] if keep_stock else 3,
    "CF_Lkas_MsgCount": cnt,
    "CF_Lkas_FcwSysState": lkas11["CF_Lkas_FcwSysState"] if keep_stock else 0,
    "CF_Lkas_FcwCollisionWarning": lkas11["CF_Lkas_FcwCollisionWarning"] if keep_stock else 0,
    "CF_Lkas_FusionState": lkas11["CF_Lkas_FusionState"] if keep_stock else 0,
    "CF_Lkas_Chksum": 0,
    "CF_Lkas_FcwOpt_USM": lkas11["CF_Lkas_FcwOpt_USM"] if keep_stock else 2,
    "CF_Lkas_LdwsOpt_USM": lkas11["CF_Lkas_LdwsOpt_USM"] if keep_stock else 3,
  }
  if car_fingerprint == CAR.GENESIS:
    values["CF_Lkas_Bca_R"] = 2
    values["CF_Lkas_HbaSysState"] = lkas11["CF_Lkas_HbaSysState"] if keep_stock else 0
    values["CF_Lkas_HbaOpt"] = lkas11["CF_Lkas_HbaOpt"] if keep_stock else 1
    values["CF_Lkas_FcwOpt_USM"] = lkas11["CF_Lkas_FcwOpt_USM"] if keep_stock else 2
    values["CF_Lkas_LdwsOpt_USM"] = lkas11["CF_Lkas_LdwsOpt_USM"] if keep_stock else 0
  if car_fingerprint == CAR.K5:
    values["CF_Lkas_Bca_R"] = 0
    values["CF_Lkas_HbaOpt"] = lkas11["CF_Lkas_HbaOpt"] if keep_stock else 1
    values["CF_Lkas_FcwOpt_USM"] = lkas11["CF_Lkas_FcwOpt_USM"] if keep_stock else 1
    values["CF_Lkas_LdwsOpt_USM"] = lkas11["CF_Lkas_LdwsOpt_USM"] if keep_stock else 3
  if car_fingerprint == CAR.SANTAFE:
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1
    values["CF_Lkas_LdwsOpt_USM"] = 2
    values["CF_Lkas_SysWarning"] = 0
  if car_fingerprint in [CAR.SELTOS, CAR.PALISADE]:
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 4 if hud_alert else 0

  dat = packer.make_can_msg("LKAS11", 0, values)[2]

  if car_fingerprint in CHECKSUM["crc8"]:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif car_fingerprint in CHECKSUM["6B"]:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", bus, values)

def create_clu11(packer, bus, clu11, button, speed, cnt):
  values = {
    "CF_Clu_CruiseSwState": button,
    "CF_Clu_CruiseSwMain": clu11["CF_Clu_CruiseSwMain"],
    "CF_Clu_SldMainSW": clu11["CF_Clu_SldMainSW"],
    "CF_Clu_ParityBit1": clu11["CF_Clu_ParityBit1"],
    "CF_Clu_VanzDecimal": clu11["CF_Clu_VanzDecimal"],
    "CF_Clu_Vanz": speed,
    "CF_Clu_SPEED_UNIT": clu11["CF_Clu_SPEED_UNIT"],
    "CF_Clu_DetentOut": clu11["CF_Clu_DetentOut"],
    "CF_Clu_RheostatLevel": clu11["CF_Clu_RheostatLevel"],
    "CF_Clu_CluInfo": clu11["CF_Clu_CluInfo"],
    "CF_Clu_AmpInfo": clu11["CF_Clu_AmpInfo"],
    "CF_Clu_AliveCnt1": cnt,
  }

  return packer.make_can_msg("CLU11", bus, values)
def create_scc11(packer, enabled, count, sccEmulation, scc11):
  if sccEmulation:
    values = {
      "MainMode_ACC": 1,
      "SCCInfoDisplay": 0,
      "AliveCounterACC": count,
      "VSetDis": 0,  # km/h velosity
      "ObjValid": 0,
      "DriverAlertDisplay": 0,
      "TauGapSet": 4,
      "Navi_SCC_Curve_Status": 0,
      "Navi_SCC_Curve_Act": 0,
      "Navi_SCC_Camera_Act": 0,
      "Navi_SCC_Camera_Status": 0,
      "ACC_ObjStatus": 0,
      "ACC_ObjDist": 150,
      "ACC_ObjLatPos":0,
      "ACC_ObjRelSpd":0,
    }
  else: 
    values = {
      "MainMode_ACC": 1, #scc11["MainMode_ACC"], #0,
      "SCCInfoDisplay": scc11["MainMode_ACC"], #0,
      "AliveCounterACC": count,
      "VSetDis": scc11["VSetDis"], #0,  # km/h velosity
      "ObjValid": 0,
      "DriverAlertDisplay": 0,
      "TauGapSet": 4, #scc11["TauGapSet"],
      "Navi_SCC_Curve_Status": 0,
      "Navi_SCC_Curve_Act": 0,
      "Navi_SCC_Camera_Act": 0,
      "Navi_SCC_Camera_Status": 0,
      "ACC_ObjStatus": 0,
      "ACC_ObjDist": scc11["ACC_ObjDist"],
      "ACC_ObjLatPos":0,
      "ACC_ObjRelSpd":0,
    }

  
  return packer.make_can_msg("SCC11", 0, values)


def create_scc12(packer, apply_accel, enabled, cnt, sccEmulation, scc12):
  if sccEmulation:
    values = {
      "CF_VSM_Prefill": 0,
      "CF_VSM_DecCmdAct": 0,
      "CF_VSM_HBACmd": 0,
      "CF_VSM_Warn": 0,
      "CF_VSM_Stat": 0,
      "CF_VSM_BeltCmd": 0,
      "ACCFailInfo": 0,
      "ACCMode": enabled,
      "StopReq": 0,
      "CR_VSM_DecCmd": 0,
      "aReqMax": apply_accel+3.0 if enabled else 0,
      "TakeOverReq": 0,
      "PreFill": 0,
      "aReqMin": apply_accel+3.0 if enabled else -10.23,
      "CF_VSM_ConfMode": 0,
      "AEB_Failinfo": 0,
      "AEB_Status": 0,
      "AEB_CmdAct": 0,
      "AEB_StopReq": 0,
      "CR_VSM_Alive": cnt,
      "CR_VSM_ChkSum": 0,
    }
  else:
    values = {
      "CF_VSM_Prefill": scc12["CF_VSM_Prefill"],
      "CF_VSM_DecCmdAct": scc12["CF_VSM_DecCmdAct"],
      "CF_VSM_HBACmd": scc12["CF_VSM_HBACmd"],
      "CF_VSM_Warn": scc12["CF_VSM_Warn"],
      "CF_VSM_Stat": scc12["CF_VSM_Stat"],
      "CF_VSM_BeltCmd": scc12["CF_VSM_BeltCmd"],
      "ACCFailInfo": scc12["ACCFailInfo"],
      "ACCMode": 1, #scc12["ACCMode"],
      "StopReq": scc12["StopReq"],
      "CR_VSM_DecCmd": scc12["CR_VSM_DecCmd"],
      "aReqMax": apply_accel if enabled and scc12["ACCMode"] == 1 else scc12["aReqMax"],
      "TakeOverReq": scc12["TakeOverReq"],
      "PreFill": scc12["PreFill"],
      "aReqMin": apply_accel if enabled and scc12["ACCMode"] == 1 else scc12["aReqMin"],
      "CF_VSM_ConfMode": scc12["CF_VSM_ConfMode"],
      "AEB_Failinfo": scc12["AEB_Failinfo"],
      "AEB_Status": scc12["AEB_Status"],
      "AEB_CmdAct": scc12["AEB_CmdAct"],
      "AEB_StopReq": scc12["AEB_StopReq"],
      "CR_VSM_Alive": cnt,
      "CR_VSM_ChkSum": 0,
    }
  dat = packer.make_can_msg("SCC12", 0, values)[2]
  values["CR_VSM_ChkSum"] = 16 - sum([sum(divmod(i, 16)) for i in dat]) % 16

  return packer.make_can_msg("SCC12", 0, values)

def create_scc13(packer, sccEmulation):
  values = {
    "SCCDrvModeRValue" : 2,
    "SCC_Equip" : 1,
    "AebDrvSetStatus" : 0,
  }
  return packer.make_can_msg("SCC13", 0, values)




def create_mdps12(packer, car_fingerprint, cnt, mdps12):
  values = {
    "CR_Mdps_StrColTq": mdps12["CR_Mdps_StrColTq"],
    "CF_Mdps_Def": mdps12["CF_Mdps_Def"],
    "CF_Mdps_ToiActive": 0,
    "CF_Mdps_ToiUnavail": 1,
    "CF_Mdps_MsgCount2": cnt,
    "CF_Mdps_Chksum2": 0,
    "CF_Mdps_ToiFlt": mdps12["CF_Mdps_ToiFlt"],
    "CF_Mdps_SErr": mdps12["CF_Mdps_SErr"],
    "CR_Mdps_StrTq": mdps12["CR_Mdps_StrTq"],
    "CF_Mdps_FailStat": mdps12["CF_Mdps_FailStat"],
    "CR_Mdps_OutTq": mdps12["CR_Mdps_OutTq"],
  }

  dat = packer.make_can_msg("MDPS12", 2, values)[2]
  checksum = sum(dat) % 256
  values["CF_Mdps_Chksum2"] = checksum

  return packer.make_can_msg("MDPS12", 2, values)

def create_lfa_mfa(packer, cnt, enabled):
  values = {
    "ACTIVE": enabled,
  }

  # ACTIVE 1 = Green steering wheel icon

  # LFA_USM 2 & 3 = LFA cancelled, fast loud beeping
  # LFA_USM 0 & 1 = No mesage

  # LFA_SysWarning 1 = "Switching to HDA", short beep
  # LFA_SysWarning 2 = "Switching to Smart Cruise control", short beep
  # LFA_SysWarning 3 =  LFA error

  # ACTIVE2: nothing
  # HDA_USM: nothing

  return packer.make_can_msg("LFAHDA_MFC", 0, values)
