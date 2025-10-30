#ifndef BOX_IDS_HPP
#define BOX_IDS_HPP

#include <cstdint>

namespace msp {

/**
 * @brief Flight mode box IDs from Betaflight.
 */

enum BoxId : std::uint8_t {
  // ARM flag
  BOXARM = 0,
  // FLIGHT_MODE
  BOXANGLE,
  BOXHORIZON,
  BOXMAG,
  BOXALTHOLD,
  BOXHEADFREE,
  BOXCHIRP,
  BOXPASSTHRU,
  BOXFAILSAFE,
  BOXPOSHOLD,
  BOXGPSRESCUE,
  BOXID_FLIGHTMODE_LAST = BOXGPSRESCUE,

  // When new flight modes are added, the parameter group version for
  // 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented
  // to ensure that the RC modes configuration is reset.

  // RCMODE flags
  BOXANTIGRAVITY,
  BOXHEADADJ,
  BOXCAMSTAB,
  BOXBEEPERON,
  BOXLEDLOW,
  BOXCALIB,
  BOXOSD,
  BOXTELEMETRY,
  BOXSERVO1,
  BOXSERVO2,
  BOXSERVO3,
  BOXBLACKBOX,
  BOXAIRMODE,
  BOX3D,
  BOXFPVANGLEMIX,
  BOXBLACKBOXERASE,
  BOXCAMERA1,
  BOXCAMERA2,
  BOXCAMERA3,
  BOXCRASHFLIP,
  BOXPREARM,
  BOXBEEPGPSCOUNT,
  BOXVTXPITMODE,
  BOXPARALYZE,
  BOXUSER1,
  BOXUSER2,
  BOXUSER3,
  BOXUSER4,
  BOXPIDAUDIO,
  BOXACROTRAINER,
  BOXVTXCONTROLDISABLE,
  BOXLAUNCHCONTROL,
  BOXMSPOVERRIDE,
  BOXSTICKCOMMANDDISABLE,
  BOXBEEPERMUTE,
  BOXREADY,
  BOXLAPTIMERRESET,
  CHECKBOX_ITEM_COUNT
};

;

/**
 * @brief Get the name of a box ID.
 */
inline const char *getBoxName(BoxId boxId) {
  switch (boxId) {
  case BOXARM:
    return "ARM";
  case BOXANGLE:
    return "ANGLE";
  case BOXHORIZON:
    return "HORIZON";
  case BOXMAG:
    return "MAG";
  case BOXALTHOLD:
    return "ALTHOLD";
  case BOXHEADFREE:
    return "HEADFREE";
  case BOXCHIRP:
    return "CHIRP";
  case BOXPASSTHRU:
    return "PASSTHRU";
  case BOXFAILSAFE:
    return "FAILSAFE";
  case BOXPOSHOLD:
    return "POSHOLD";
  case BOXGPSRESCUE:
    return "GPSRESCUE";
  case BOXANTIGRAVITY:
    return "ANTIGRAVITY";
  case BOXHEADADJ:
    return "HEADADJ";
  case BOXCAMSTAB:
    return "CAMSTAB";
  case BOXBEEPERON:
    return "BEEPERON";
  case BOXLEDLOW:
    return "LEDLOW";
  case BOXCALIB:
    return "CALIB";
  case BOXOSD:
    return "OSD";
  case BOXTELEMETRY:
    return "TELEMETRY";
  case BOXSERVO1:
    return "SERVO1";
  case BOXSERVO2:
    return "SERVO2";
  case BOXSERVO3:
    return "SERVO3";
  case BOXBLACKBOX:
    return "BLACKBOX";
  case BOXAIRMODE:
    return "AIRMODE";
  case BOX3D:
    return "3D";
  case BOXFPVANGLEMIX:
    return "FPVANGLEMIX";
  case BOXBLACKBOXERASE:
    return "BLACKBOXERASE";
  case BOXCAMERA1:
    return "CAMERA1";
  case BOXCAMERA2:
    return "CAMERA2";
  case BOXCAMERA3:
    return "CAMERA3";
  case BOXCRASHFLIP:
    return "CRASHFLIP";
  case BOXPREARM:
    return "PREARM";
  case BOXBEEPGPSCOUNT:
    return "BEEPGPSCOUNT";
  case BOXVTXPITMODE:
    return "VTXPITMODE";
  case BOXPARALYZE:
    return "PARALYZE";
  case BOXUSER1:
    return "USER1";
  case BOXUSER2:
    return "USER2";
  case BOXUSER3:
    return "USER3";
  case BOXUSER4:
    return "USER4";
  case BOXPIDAUDIO:
    return "PIDAUDIO";
  case BOXACROTRAINER:
    return "ACROTRAINER";
  case BOXVTXCONTROLDISABLE:
    return "VTXCONTROLDISABLE";
  case BOXLAUNCHCONTROL:
    return "LAUNCHCONTROL";
  case BOXMSPOVERRIDE:
    return "MSPOVERRIDE";
  case BOXSTICKCOMMANDDISABLE:
    return "STICKCOMMANDDISABLE";
  case BOXBEEPERMUTE:
    return "BEEPERMUTE";
  case BOXREADY:
    return "READY";
  case BOXLAPTIMERRESET:
    return "LAPTIMERRESET";
  default:
    return "UNKNOWN";
  }
}

} // namespace msp

#endif // !BOX_IDS_HPP
