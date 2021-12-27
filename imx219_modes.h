/*
Copyright (c) 2017, Raspberry Pi Foundation
Copyright (c) 2017, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// These values are copied from https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=162722&p=1052339
// Raspberry Pi can not discuss these settings as we have information from
// Sony under NDA.

// REQUESTS FOR SUPPORT ABOUT THESE REGISTER VALUES WILL
// BE IGNORED BY PI TOWERS.

#ifndef IMX219MODES_H_
#define IMX219MODES_H_

// clang-format off
// We want to preserve 1 register per line.

struct sensor_regs imx219_8MPix[] = {
    // 3280x2464@15 FPS
    {0x0100, 0x00},  // 0=OFF, 1=Stream, 2=MAX
    {0x30EB, 0x05},
    {0x30EB, 0x0C},
    {0x300A, 0xFF},
    {0x300B, 0xFF},
    {0x30EB, 0x05},
    {0x30EB, 0x09},
    {0x0114, 0x01},  // CSI MIPI Lanes [1:0] (0x01=2, 0x03=4)
    {0x0128, 0x00},  // DPHY_CNTRL
    {0x012A, 0x18},  // EXCK_FREQ [15:8]
    {0x012B, 0x00},  // EXCK_FREQ [7:0]

    // Frame Bank A register group, 0x0154 - 0x018D

    {0x0157, 0x00},  // Analog Gain
    {0x0158, 0x01},  // Digital Gain [15:8]
    {0x0159, 0x00},  // Digital Gain [7:0]
    {0x015A, 0x01},  // Shutter/Coarse Integration Time [15:8]
    {0x015B, 0x00},  // Shutter/Coarse Integration Time [7:0]
    // Frame length unit is {1, 2} lines in binning modes {{0, 1, 2}, 3}
    // Here it is 0x9c8 = 2540
    {0x0160, 0x09},  // Frame Length [15:8]
    {0x0161, 0xC8},  // Frame Length [7:0]
    // Line length in units of pixels
    // Here it is 0xd78 = 3448
    {0x0162, 0x0D},  // Line Length [15:8]
    {0x0163, 0x78},  // Line Length [7:0]
    // x addresss start: x addresss of the top left corner pixel
    // Here it is 0
    {0x0164, 0x00},  // x addresss start [11:8]
    {0x0165, 0x00},  // x addresss start [7:0]
    // x address end: x addresss of the bottom right corner pixel
    // Here it is 0xccf = 3279
    {0x0166, 0x0C},  // x addresss end [11:8]
    {0x0167, 0xCF},  // x addresss end [7:0]
    // y addresss start: y addresss of the top left corner pixel
    // Here it is 0
    {0x0168, 0x00},  // y addresss start [11:8]
    {0x0169, 0x00},  // y addresss start [7:0]
    // y address end: y addresss of the bottom right corner pixel
    // Here it is 099f = 2463
    {0x016A, 0x09},  // y addresss end [11:8]
    {0x016B, 0x9F},  // y addresss end [7:0]
    // Image width (x-direction)
    // Here it is 0xcd0 = 3280 pixels
    {0x016C, 0x0C},  // Image width [11:8]
    {0x016D, 0xD0},  // Image width [7:0]
    // Image height (y-direction)
    // Here it is 0x9a0 = 2464 pixels
    {0x016E, 0x09},  // Image height [11:8]
    {0x016F, 0xA0},  // Image height [7:0]
    // X and Y increments for odd pixels
    {0x0170, 0x01},  // X_ODD_INC [2:0]
    {0x0171, 0x01},  // Y_ODD_INC [2:0]
    {0x0172, 0x03},  // Image orientation, bit 0: horizontal, bit 1: vertical
    // Binning modes:
    // 0: none, 1: x2, 2: x4, 3: x2 analog
    {0x0174, 0x00},  // Binning Mode H_A (horizontal)
    {0x0175, 0x00},  // Binning Mode V_A (vertical)
    // Binning calculation modes:
    // 0: average, 1: sum
    {0x0176, 0x00},  // Binning Calculation Mode, horizontal
    {0x0177, 0x00},  // Binning Calculation Mode, vertical
    {0x0189, 0x00},  // Analog gain, short exposure
    // Coarse integration time, short exposure
    {0x018A, 0x01},  // Coarse Integration time [15:8]
    {0x018B, 0xF4},  // Coarse Integration time [7:0]
    {0x018C, 0x0A},  // CSI-2 Data Format [15:8]
    {0x018D, 0x0A},  // CSI-2 Data Format [7:0]

    // Frame Bank B register group, 0x0254 - 0x028D - not set here

    // Clock setup registers 0x0300 - 0x0627

    {0x0301, 0x05},  // VTPXCK_DIV
    {0x0303, 0x01},  // VTSYCK_DIV
    {0x0304, 0x03},  // PREPLLCK_VT_DIV [3:0]
    {0x0305, 0x03},  // PREPLLCK_OP_DIV [3:0]
    {0x0306, 0x00},  // PLL_VT_MPY [10:8]
    {0x0307, 0x2B},  // PLL_VT_MPY [7:0]
    {0x0309, 0x0A},  // OPPXCK_DIV [4:0]
    {0x030B, 0x01},  // OPSYCK_DIV
    {0x030C, 0x00},  // PLL_OP_MPY [10:8]
    {0x030D, 0x55},  // PLL_OP_MPY [7:0]
    {0x4540, 0x00},  // CIS Tuning ?
    {0x455E, 0x00},  // CIS Tuning ?
    {0x4713, 0x30},  // CIS Tuning ?
    {0x471E, 0x4B},  // CIS Tuning ?
    {0x4750, 0x14},  // CIS Tuning ?
    {0x4767, 0x0F},  // CIS Tuning ?
    {0x478B, 0x10},  // CIS Tuning ?
    {0x478F, 0x10},  // CIS Tuning ?
    {0x4797, 0x0E},  // CIS Tuning ?
    {0x479B, 0x0E},  // CIS Tuning ?
    {0x47B4, 0x14},  // CIS Tuning ?
    {0x0100, 0x01}};


// Register setting provided on
// https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=109137&start=350#p1259865
// with the addreg macro removed.
struct sensor_regs imx219_mode1[] =
{
    // 1920x1080
    {0x0100, 0x00},
    {0x30eb, 0x05},
    {0x30eb, 0x0c},
    {0x300a, 0xff},
    {0x300b, 0xff},
    {0x30eb, 0x05},
    {0x30eb, 0x09},
    {0x0114, 0x01},
    {0x0128, 0x00},
    {0x012a, 0x18},
    {0x012b, 0x00},
    {0x0162, 0x0d},
    {0x0163, 0x78},
    {0x0164, 0x02},
    {0x0165, 0xa8},
    {0x0166, 0x0a},
    {0x0167, 0x27},
    {0x0168, 0x02},
    {0x0169, 0xb4},
    {0x016a, 0x06},
    {0x016b, 0xeb},
    {0x016c, 0x07},
    {0x016d, 0x80},
    {0x016e, 0x04},
    {0x016f, 0x38},
    {0x0170, 0x01},
    {0x0171, 0x01},
    {0x0174, 0x00},
    {0x0175, 0x00},
    {0x018c, 0x0a},
    {0x018d, 0x0a},
    {0x0301, 0x05},
    {0x0303, 0x01},
    {0x0304, 0x03},
    {0x0305, 0x03},
    {0x0306, 0x00},
    {0x0307, 0x39},
    {0x0309, 0x0a},
    {0x030b, 0x01},
    {0x030c, 0x00},
    {0x030d, 0x72},
    {0x455e, 0x00},
    {0x471e, 0x4b},
    {0x4767, 0x0f},
    {0x4750, 0x14},
    {0x4540, 0x00},
    {0x47b4, 0x14},
    {0x4713, 0x30},
    {0x478b, 0x10},
    {0x478f, 0x10},
    {0x4793, 0x10},
    {0x4797, 0x0e},
    {0x479b, 0x0e},

    {0x0172, 0x03},
    {0x0157, 0x00},
    {0x0160, 0x06},
    {0x0161, 0xe3},
    {0x0162, 0x0d},
    {0x0163, 0x78},
    {0x015a, 0x00},
    {0x015b, 0x31},
    {0x0100, 0x01},
};

struct sensor_regs imx219_mode2[] =
{
    // 3280x2464
    {0x0100, 0x00},
    {0x30eb, 0x0c},
    {0x30eb, 0x05},
    {0x300a, 0xff},
    {0x300b, 0xff},
    {0x30eb, 0x05},
    {0x30eb, 0x09},
    {0x0114, 0x01},
    {0x0128, 0x00},
    {0x012a, 0x18},
    {0x012b, 0x00},
    {0x0164, 0x00},
    {0x0165, 0x00},
    {0x0166, 0x0c},
    {0x0167, 0xcf},
    {0x0168, 0x00},
    {0x0169, 0x00},
    {0x016a, 0x09},
    {0x016b, 0x9f},
    {0x016c, 0x0c},
    {0x016d, 0xd0},
    {0x016e, 0x09},
    {0x016f, 0xa0},
    {0x0170, 0x01},
    {0x0171, 0x01},
    {0x0174, 0x00},
    {0x0175, 0x00},
    {0x018c, 0x0a},
    {0x018d, 0x0a},
    {0x0301, 0x05},
    {0x0303, 0x01},
    {0x0304, 0x03},
    {0x0305, 0x03},
    {0x0306, 0x00},
    {0x0307, 0x39},
    {0x0309, 0x0a},
    {0x030b, 0x01},
    {0x030c, 0x00},
    {0x030d, 0x72},
    {0x0624, 0x0c},
    {0x0625, 0xd0},
    {0x0626, 0x09},
    {0x0627, 0xa0},
    {0x455e, 0x00},
    {0x471e, 0x4b},
    {0x4767, 0x0f},
    {0x4750, 0x14},
    {0x4540, 0x00},
    {0x47b4, 0x14},
    {0x4713, 0x30},
    {0x478b, 0x10},
    {0x478f, 0x10},
    {0x4793, 0x10},
    {0x4797, 0x0e},
    {0x479b, 0x0e},

    {0x0172, 0x03},
    {0x0157, 0x00},
    {0x0160, 0x0d},
    {0x0161, 0xc6},
    {0x0162, 0x0d},
    {0x0163, 0x78},
    {0x015a, 0x00},
    {0x015b, 0x31},
    {0x0100, 0x01},
};

struct sensor_regs imx219_mode3[] =
{
    // 3280x2464
    {0x0100, 0x00},
    {0x30eb, 0x0c},
    {0x30eb, 0x05},
    {0x300a, 0xff},
    {0x300b, 0xff},
    {0x30eb, 0x05},
    {0x30eb, 0x09},
    {0x0114, 0x01},
    {0x0128, 0x00},
    {0x012a, 0x18},
    {0x012b, 0x00},
    {0x0164, 0x00},
    {0x0165, 0x00},
    {0x0166, 0x0c},
    {0x0167, 0xcf},
    {0x0168, 0x00},
    {0x0169, 0x00},
    {0x016a, 0x09},
    {0x016b, 0x9f},
    {0x016c, 0x0c},
    {0x016d, 0xd0},
    {0x016e, 0x09},
    {0x016f, 0xa0},
    {0x0170, 0x01},
    {0x0171, 0x01},
    {0x0174, 0x00},
    {0x0175, 0x00},
    {0x018c, 0x0a},
    {0x018d, 0x0a},
    {0x0301, 0x05},
    {0x0303, 0x01},
    {0x0304, 0x03},
    {0x0305, 0x03},
    {0x0306, 0x00},
    {0x0307, 0x39},
    {0x0309, 0x0a},
    {0x030b, 0x01},
    {0x030c, 0x00},
    {0x030d, 0x72},
    {0x0624, 0x0c},
    {0x0625, 0xd0},
    {0x0626, 0x09},
    {0x0627, 0xa0},
    {0x455e, 0x00},
    {0x471e, 0x4b},
    {0x4767, 0x0f},
    {0x4750, 0x14},
    {0x4540, 0x00},
    {0x47b4, 0x14},
    {0x4713, 0x30},
    {0x478b, 0x10},
    {0x478f, 0x10},
    {0x4793, 0x10},
    {0x4797, 0x0e},
    {0x479b, 0x0e},
    {0x0172, 0x03},
    {0x0157, 0x00},
    {0x0160, 0x0d},
    {0x0161, 0xc6},
    {0x0162, 0x0d},
    {0x0163, 0x78},
    {0x015a, 0x00},
    {0x015b, 0x31},
    {0x0100, 0x01},
};

struct sensor_regs imx219_mode4[] =
{
    // 1640x1232, 2x2 binning
    {0x0100, 0x00},
    {0x30eb, 0x0c},
    {0x30eb, 0x05},
    {0x300a, 0xff},
    {0x300b, 0xff},
    {0x30eb, 0x05},
    {0x30eb, 0x09},
    {0x0114, 0x01},
    {0x0128, 0x00},
    {0x012a, 0x18},
    {0x012b, 0x00},
    {0x0164, 0x00},
    {0x0165, 0x00},
    {0x0166, 0x0c},
    {0x0167, 0xcf},
    {0x0168, 0x00},
    {0x0169, 0x00},
    {0x016a, 0x09},
    {0x016b, 0x9f},
    {0x016c, 0x06},
    {0x016d, 0x68},
    {0x016e, 0x04},
    {0x016f, 0xd0},
    {0x0170, 0x01},
    {0x0171, 0x01},
    {0x0174, 0x01},
    {0x0175, 0x01},
    {0x018c, 0x0a},
    {0x018d, 0x0a},
    {0x0301, 0x05},
    {0x0303, 0x01},
    {0x0304, 0x03},
    {0x0305, 0x03},
    {0x0306, 0x00},
    {0x0307, 0x39},
    {0x0309, 0x0a},
    {0x030b, 0x01},
    {0x030c, 0x00},
    {0x030d, 0x72},
    {0x455e, 0x00},
    {0x471e, 0x4b},
    {0x4767, 0x0f},
    {0x4750, 0x14},
    {0x4540, 0x00},
    {0x47b4, 0x14},
    {0x4713, 0x30},
    {0x478b, 0x10},
    {0x478f, 0x10},
    {0x4793, 0x10},
    {0x4797, 0x0e},
    {0x479b, 0x0e},
    {0x0172, 0x03},
    {0x0157, 0x00},
    {0x0160, 0x05},
    {0x0161, 0x2a},
    {0x0162, 0x0d},
    {0x0163, 0x78},
    {0x015a, 0x00},
    {0x015b, 0x31},
    {0x0100, 0x01},
};

struct sensor_regs imx219_mode5[] =
{
    // 1640x922 with 2x2 binning
    {0x0100, 0x00},
    {0x30eb, 0x0c},
    {0x30eb, 0x05},
    {0x300a, 0xff},
    {0x300b, 0xff},
    {0x30eb, 0x05},
    {0x30eb, 0x09},
    {0x0114, 0x01},
    {0x0128, 0x00},
    {0x012a, 0x18},
    {0x012b, 0x00},
    {0x0164, 0x00},
    {0x0165, 0x00},
    {0x0166, 0x0c},
    {0x0167, 0xcf},
    {0x0168, 0x01},
    {0x0169, 0x36},
    {0x016a, 0x08},
    {0x016b, 0x69},
    {0x016c, 0x06},
    {0x016d, 0x68},
    {0x016e, 0x03},
    {0x016f, 0x9a},
    {0x0170, 0x01},
    {0x0171, 0x01},
    {0x0174, 0x01},
    {0x0175, 0x01},
    {0x018c, 0x0a},
    {0x018d, 0x0a},
    {0x0301, 0x05},
    {0x0303, 0x01},
    {0x0304, 0x03},
    {0x0305, 0x03},
    {0x0306, 0x00},
    {0x0307, 0x39},
    {0x0309, 0x0a},
    {0x030b, 0x01},
    {0x030c, 0x00},
    {0x030d, 0x72},
    {0x455e, 0x00},
    {0x471e, 0x4b},
    {0x4767, 0x0f},
    {0x4750, 0x14},
    {0x4540, 0x00},
    {0x47b4, 0x14},
    {0x4713, 0x30},
    {0x478b, 0x10},
    {0x478f, 0x10},
    {0x4793, 0x10},
    {0x4797, 0x0e},
    {0x479b, 0x0e},
    {0x0172, 0x03},
    {0x0157, 0x00},
    {0x0160, 0x05},
    {0x0161, 0x2a},
    {0x0162, 0x0d},
    {0x0163, 0x78},
    {0x015a, 0x00},
    {0x015b, 0x31},
    {0x0100, 0x01},
};

struct sensor_regs imx219_mode6[] =
{
    // 1280x720 with 2x2 binning
    {0x0100, 0x00},
    {0x30eb, 0x05},
    {0x30eb, 0x0c},
    {0x300a, 0xff},
    {0x300b, 0xff},
    {0x30eb, 0x05},
    {0x30eb, 0x09},
    {0x0114, 0x01},
    {0x0128, 0x00},
    {0x012a, 0x18},
    {0x012b, 0x00},
    {0x0164, 0x01},
    {0x0165, 0x68},
    {0x0166, 0x0b},
    {0x0167, 0x67},
    {0x0168, 0x02},
    {0x0169, 0x00},
    {0x016a, 0x07},
    {0x016b, 0x9f},
    {0x016c, 0x05},
    {0x016d, 0x00},
    {0x016e, 0x02},
    {0x016f, 0xd0},
    {0x0170, 0x01},
    {0x0171, 0x01},
    {0x0174, 0x03},
    {0x0175, 0x03},
    {0x018c, 0x0a},
    {0x018d, 0x0a},
    {0x0301, 0x05},
    {0x0303, 0x01},
    {0x0304, 0x03},
    {0x0305, 0x03},
    {0x0306, 0x00},
    {0x0307, 0x39},
    {0x0309, 0x0a},
    {0x030b, 0x01},
    {0x030c, 0x00},
    {0x030d, 0x72},
    {0x455e, 0x00},
    {0x471e, 0x4b},
    {0x4767, 0x0f},
    {0x4750, 0x14},
    {0x4540, 0x00},
    {0x47b4, 0x14},
    {0x4713, 0x30},
    {0x478b, 0x10},
    {0x478f, 0x10},
    {0x4793, 0x10},
    {0x4797, 0x0e},
    {0x479b, 0x0e},
    {0x0172, 0x03},
    {0x0157, 0x00},
    {0x0160, 0x02},
    {0x0161, 0x39},
    {0x0162, 0x0d},
    {0x0163, 0xe7},
    {0x015a, 0x00},
    {0x015b, 0x2f},
    {0x0100, 0x01},
};

struct sensor_regs imx219_mode7[] =
{
    // 640x480 with 2x2 binning
    {0x0100, 0x00},  // 0=OFF, 1=Stream, 2=MAX
    {0x30EB, 0x05},
    {0x30EB, 0x0C},
    {0x300A, 0xFF},
    {0x300B, 0xFF},
    {0x30EB, 0x05},
    {0x30EB, 0x09},
    {0x0114, 0x01},  // CSI MIPI Lanes [1:0] (0x01=2, 0x03=4)
    {0x0128, 0x00},  // DPHY_CNTRL
    {0x012A, 0x18},  // EXCK_FREQ [15:8]
    {0x012B, 0x00},  // EXCK_FREQ [7:0]

    // Frame Bank A register group, 0x0154 - 0x018D

    {0x0157, 0x00},  // Analog Gain
    // Digital Gain (0x0158, 0x0159) is not set here.  Why?
    {0x015A, 0x00},  // Shutter/Coarse Integration Time [15:8]
    {0x015B, 0x2f},  // Shutter/Coarse Integration Time [7:0]
    // Frame length unit is {1, 2} lines in binning modes {{0, 1, 2}, 3}
    // Here it is 0x239 = 569 (that's 1138 lines)
    {0x0160, 0x02},  // Frame Length [15:8]
    {0x0161, 0x39},  // Frame Length [7:0]
    // Line length in units of pixels
    // Here it is 0xde7 = 3559
    {0x0162, 0x0D},  // Line Length [15:8]
    {0x0163, 0xe7},  // Line Length [7:0]
    // x addresss start: x addresss of the top left corner pixel
    // Here it is 0x3e8 = 1000 = (3280 - 640 * 2)/2
    // The value is chosen so that the image is centered on the sensor.
    {0x0164, 0x03},  // x addresss start [11:8]
    {0x0165, 0xe8},  // x addresss start [7:0]
    // x address end: x addresss of the bottom right corner pixel
    // Here it is 0x8e7 = 2279 = 1000 (x start) + 640 (width) * 2 (for binning) - 1
    {0x0166, 0x08},  // x addresss end [11:8]
    {0x0167, 0xe7},  // x addresss end [7:0]
    // y addresss start: y addresss of the top left corner pixel
    // Here it is 0x2f0 = 752
    {0x0168, 0x02},  // y addresss start [11:8]
    {0x0169, 0xf0},  // y addresss start [7:0]
    // y address end: y addresss of the bottom right corner pixel
    // Here it is 0x6af = 1711 = 752 (y start) + 480 (height) * 2 (for binning) - 1
    {0x016A, 0x06},  // y addresss end [11:8]
    {0x016B, 0xaF},  // y addresss end [7:0]
    // Image width (x-direction)
    // Here it is 0x280 = 640
    {0x016C, 0x02},  // Image width [11:8]
    {0x016D, 0x80},  // Image width [7:0]
    // Image height (y-direction)
    // Here it is 0x1e0 = 480
    {0x016E, 0x01},  // Image height [11:8]
    {0x016F, 0xe0},  // Image height [7:0]
    // X and Y increments for odd pixels
    {0x0170, 0x01},  // X_ODD_INC [2:0]
    {0x0171, 0x01},  // Y_ODD_INC [2:0]
    {0x0172, 0x03},  // Image orientation, bit 0: horizontal, bit 1: vertical
    // Binning modes:
    // 0: none, 1: 2x2, 2: 4x4, 3: 2x2 analog
    {0x0174, 0x03},  // Binning Mode H_A (horizontal)
    {0x0175, 0x03},  // Binning Mode V_A (vertical)
    // Binning calculation modes:
    // 0: average, 1: sum
    {0x0176, 0x00},  // Binning Calculation Mode, horizontal
    {0x0177, 0x00},  // Binning Calculation Mode, vertical
    {0x0189, 0x00},  // Analog gain, short exposure
    // Coarse integration time, short exposure
    {0x018A, 0x01},  // Coarse Integration time [15:8]
    {0x018B, 0xF4},  // Coarse Integration time [7:0]
    {0x018C, 0x0A},  // CSI-2 Data Format [15:8]
    {0x018D, 0x0A},  // CSI-2 Data Format [7:0]

    // Frame Bank B register group, 0x0254 - 0x028D - not set here

    // Clock setup registers 0x0300 - 0x0627

    {0x0301, 0x05},  // VTPXCK_DIV
    {0x0303, 0x01},  // VTSYCK_DIV
    {0x0304, 0x03},  // PREPLLCK_VT_DIV [3:0]
    {0x0305, 0x03},  // PREPLLCK_OP_DIV [3:0]
    {0x0306, 0x00},  // PLL_VT_MPY [10:8]
    {0x0307, 0x39},  // PLL_VT_MPY [7:0]
    {0x0309, 0x0A},  // OPPXCK_DIV [4:0]
    {0x030B, 0x01},  // OPSYCK_DIV
    {0x030C, 0x00},  // PLL_OP_MPY [10:8]
    {0x030D, 0x72},  // PLL_OP_MPY [7:0]
    {0x4540, 0x00},
    {0x455E, 0x00},
    {0x4713, 0x30},
    {0x471E, 0x4B},
    {0x4750, 0x14},
    {0x4767, 0x0F},
    {0x478B, 0x10},
    {0x478F, 0x10},
    {0x4797, 0x0E},
    {0x479B, 0x0E},
    {0x47B4, 0x14},
    {0x0100, 0x01}
};

// clang-format on

// TODO binning is not defined here. It should be added and taken into account when crop is calculated.
struct mode_def imx219_modes[] = {
	{
	    .regs = imx219_8MPix,
	    .num_regs = NUM_ELEMENTS(imx219_8MPix),
	    .width = 3280,
	    .height = 2464,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 2504,
	    .line_time_ns = 18904,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode1,
	    .num_regs = NUM_ELEMENTS(imx219_mode1),
	    .width = 1920,
	    .height = 1080,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 1084,
	    .line_time_ns = 18904,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode2,
	    .num_regs = NUM_ELEMENTS(imx219_mode2),
	    .width = 3280,
	    .height = 2464,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 2468,
	    .line_time_ns = 18904,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode3,
	    .num_regs = NUM_ELEMENTS(imx219_mode3),
	    .width = 3280,
	    .height = 2464,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 2468,
	    .line_time_ns = 18904,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode4,
	    .num_regs = NUM_ELEMENTS(imx219_mode4),
	    .width = 1640,
	    .height = 1232,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 1236,
	    .line_time_ns = 18904,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode5,
	    .num_regs = NUM_ELEMENTS(imx219_mode5),
	    .width = 1640,
	    .height = 922,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 926,
	    .line_time_ns = 18904,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode6,
	    .num_regs = NUM_ELEMENTS(imx219_mode6),
	    .width = 1280,
	    .height = 720,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 724,
	    .line_time_ns = 19517,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
	{
	    .regs = imx219_mode7,
	    .num_regs = NUM_ELEMENTS(imx219_mode7),
	    .width = 640,
	    .height = 480,
	    .encoding = 0,
	    .order = BAYER_ORDER_BGGR,
	    .native_bit_depth = 10,
	    .image_id = 0x2B,
	    .data_lanes = 2,
	    .min_vts = 484,
	    .line_time_ns = 19517,
	    .timing = { 0, 0, 0, 0, 0 },
	    .term = { 0, 0 },
	    .black_level = 66,
	},
};

// From
// https://android.googlesource.com/kernel/bcm/+/android-bcm-tetra-3.10-lollipop-wear-release/drivers/media/video/imx219.c
struct sensor_regs imx219_stop[] = {
	// to power down
	{ 0x0100, 0x00 }, // disable streaming
};

static int imx219_set_crop(const struct sensor_def *const sensor, struct mode_def *const sensor_mode,
			   const struct raspiraw_crop *const cfg)
{
// TODO This does not take binning into account.  binning member needs to be the in mode_def first.
#define WIDTH  (3280)
#define HEIGHT (2464)

	// All members of cfg default to -1 if the user did not set them.
	// Check for validity.
	if ((cfg->height == 0) || (cfg->width == 0))
	{
		// 0 is not a valid value for either.
		return -1;
	}
	if ((cfg->height > HEIGHT) || (cfg->width > WIDTH))
	{
		// These are the maximum values, can't exceed them.
		return -1;
	}
	if ((cfg->left >= 0) && (cfg->left + cfg->width > WIDTH))
	{
		// Falling off the right edge of the sensor.
		return -1;
	}
	if ((cfg->top >= 0) && (cfg->top + cfg->height > WIDTH))
	{
		// Falling off the bottom edge of the sensor.
		return -1;
	}

	if (cfg->hinc >= 0)
	{
		modReg(sensor_mode, 0x0170, 0, 2, cfg->hinc, EQUAL);
	}

	if (cfg->vinc >= 0)
	{
		modReg(sensor_mode, 0x0171, 0, 2, cfg->vinc, EQUAL);
	}

	if (cfg->width > 0)
	{
		sensor_mode->width = cfg->width;
		modReg(sensor_mode, 0x016C, 0, 3, cfg->width >> 8, EQUAL);
		modReg(sensor_mode, 0x016D, 0, 7, cfg->width & 0xFF, EQUAL);
	}

	if (cfg->height > 0)
	{
		sensor_mode->height = cfg->height;
		modReg(sensor_mode, 0x016E, 0, 3, cfg->height >> 8, EQUAL);
		modReg(sensor_mode, 0x016F, 0, 7, cfg->height & 0xFF, EQUAL);
	}

	// Set the start and end pixels in both dimensions.  If the user
	// did not provide left or top offset, then assume that we should
	// center the image.
	{
		int x_start = 0;
		if (cfg->left >= 0)
		{
			x_start = cfg->left;
		}
		else
		{
			x_start = (WIDTH - sensor_mode->width) / 2;
		}
		modReg(sensor_mode, 0x0164, 0, 3, x_start >> 8, EQUAL);
		modReg(sensor_mode, 0x0165, 0, 7, x_start & 0xFF, EQUAL);
		const int x_end = x_start + sensor_mode->width - 1;
		modReg(sensor_mode, 0x0166, 0, 3, x_end >> 8, EQUAL);
		modReg(sensor_mode, 0x0167, 0, 7, x_end & 0xFF, EQUAL);
	}
	{
		int y_start = 0;
		if (cfg->top >= 0)
		{
			y_start = cfg->top;
		}
		else
		{
			y_start = (HEIGHT - sensor_mode->height) / 2;
		}
		modReg(sensor_mode, 0x0168, 0, 3, y_start >> 8, EQUAL);
		modReg(sensor_mode, 0x0169, 0, 7, y_start & 0xFF, EQUAL);
		const int y_end = y_start + sensor_mode->height - 1;
		modReg(sensor_mode, 0x016A, 0, 3, y_end >> 8, EQUAL);
		modReg(sensor_mode, 0x016B, 0, 7, y_end & 0xFF, EQUAL);
	}

	return 0;

#undef WIDTH
#undef HEIGHT
}

// ID, exposure, and gain register settings taken from
// https://android.googlesource.com/kernel/bcm/+/android-bcm-tetra-3.10-lollipop-wear-release/drivers/media/video/imx219.c
// Flip settings taken from
// https://github.com/rellimmot/Sony-IMX219-Raspberry-Pi-V2-CMOS/blob/master/imx219mipiraw_Sensor.c#L585
struct sensor_def imx219 = {
	.name = "imx219",
	.modes = imx219_modes,
	.num_modes = NUM_ELEMENTS(imx219_modes),
	.stop = imx219_stop,
	.num_stop_regs = NUM_ELEMENTS(imx219_stop),

	.i2c_addr = 0x10,
	.i2c_addressing = 2,
	.i2c_ident_length = 2,
	.i2c_ident_reg = 0x0000,
	.i2c_ident_value = 0x1902, // 0x0219 bytes reversed

	.vflip_reg = 0x172,
	.vflip_reg_bit = 1,
	.hflip_reg = 0x172,
	.hflip_reg_bit = 0,

	.exposure_reg = 0x015A,
	.exposure_reg_num_bits = 16,

	.vts_reg = 0x0160,
	.vts_reg_num_bits = 16,

	.gain_reg = 0x0157,
	.gain_reg_num_bits = 8, // Only valid up to 230.

	.set_crop = imx219_set_crop,
};

#endif
