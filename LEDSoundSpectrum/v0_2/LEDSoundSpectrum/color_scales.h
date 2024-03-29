/**
 * @file color_scales.h
 * @brief color scale array data macros
 * @copyright Chase Vara, 2024
 **/
#ifndef _COLOR_SCALES_H_
#define _COLOR_SCALES_H_

#define NUM_COLOR_SCALES 3
#define NUM_COLORS 32
#define COLOR_SCALES_ARRAY {HEAT_SCALE_HEX32, ORANGE_SCALE_HEX32, SUNSET_SCALE_HEX32}

#define HEAT_SCALE_HEX32 {\
  0x000003,\
  0x000016,\
  0x000029,\
  0x00003D,\
  0x000F50,\
  0x002764,\
  0x004577,\
  0x00688B,\
  0x008E9E,\
  0x00B1B1,\
  0x00C5C5,\
  0x00D8C6,\
  0x00ECB4,\
  0x02FFA0,\
  0x00FF79,\
  0x00FF4D,\
  0x00FF1F,\
  0x00FF00,\
  0x12FF00,\
  0x40FF00,\
  0x72FF00,\
  0xA7FF00,\
  0xE0FF00,\
  0xFFFF00,\
  0xFFFF00,\
  0xFFED00,\
  0xFFB100,\
  0xFF7100,\
  0xFF2D00,\
  0xFF0000,\
  0xFF0000,\
  0xFF0000,\
}

#define ORANGE_SCALE_HEX32 {\
  0x030000,\
  0x0A0100,\
  0x120200,\
  0x1A0300,\
  0x220500,\
  0x2A0701,\
  0x320A02,\
  0x3A0D02,\
  0x421003,\
  0x4A1405,\
  0x511806,\
  0x591C07,\
  0x612009,\
  0x69250B,\
  0x712A0D,\
  0x792F0F,\
  0x813512,\
  0x893B14,\
  0x914017,\
  0x98471A,\
  0xA04D1D,\
  0xA85320,\
  0xB05A24,\
  0xB86127,\
  0xC0682B,\
  0xC86F2F,\
  0xD07633,\
  0xD87D37,\
  0xDF843C,\
  0xE78C40,\
  0xEF9345,\
  0xF79B4A,\
}

#define SUNSET_SCALE_HEX32 {\
  0x000004,\
  0x030104,\
  0x070104,\
  0x0a0204,\
  0x0d0203,\
  0x100303,\
  0x140403,\
  0x170403,\
  0x1a0503,\
  0x1e0603,\
  0x210603,\
  0x240703,\
  0x270702,\
  0x2b0802,\
  0x2e0902,\
  0x310902,\
  0x350a02,\
  0x380a02,\
  0x3b0b02,\
  0x3f0c02,\
  0x420c01,\
  0x450d01,\
  0x480d01,\
  0x4c0e01,\
  0x4f0f01,\
  0x520f01,\
  0x561001,\
  0x591101,\
  0x5c1100,\
  0x5f1200,\
  0x631200,\
  0x661300\
}

#endif  // _COLOR_SCALES_H_