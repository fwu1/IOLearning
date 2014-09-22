/**
 * Arduino - CameraOV7670 implementation.
 * 
 * CameraOV7670.h
 * 
 * The class CameraOV7670.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#include "Camera.h"
#include <OutputStream.h>
#include <Wire.h>

#ifndef __ARDUINO_DRIVER_CAMERA_OV7670_H__
#define __ARDUINO_DRIVER_CAMERA_OV7670_H__ 1

// CCIR656 enable
#define OV7670_COM1_CCIR656            0x40

// Soft sleep mode
#define OV7670_COM2_SSLEEP             0x10

// Byte swap
#define OV7670_COM3_SWAP               0x40

// Enable scaling
#define OV7670_COM3_SCALEEN            0x08

// Enable downsamp/crop/window
#define OV7670_COM3_DCWEN              0x04

// Use external clock directly
#define OV7670_CLKRC_EXT               0x40

// Mask for internal clock scale
#define OV7670_CLKRC_SCALE             0x3f

// Reset
#define OV7670_COM7_RESET              0x80

// 
#define OV7670_COM7_FMT_MASK           0x38

// 
#define OV7670_COM7_FMT_VGA            0x00

// CIF format
#define OV7670_COM7_FMT_CIF            0x20

// QVGA format
#define OV7670_COM7_FMT_QVGA           0x10

// QCIF format
#define OV7670_COM7_FMT_QCIF           0x08

// bits 0 and 2 - RGB format
#define OV7670_COM7_RGB                0x04

// YUV
#define OV7670_COM7_YUV                0x00

// Bayer format
#define OV7670_COM7_BAYER              0x01

// Processed bayer
#define OV7670_COM7_PBAYER             0x05

// Enable fast AGC/AEC
#define OV7670_COM8_FASTAEC            0x80

// Unlimited AEC step size
#define OV7670_COM8_AECSTEP            0x40

// Band filter enable
#define OV7670_COM8_BFILT              0x20

// Auto gain enable
#define OV7670_COM8_AGC                0x04

// White balance enable
#define OV7670_COM8_AWB                0x02

// Auto exposure enable
#define OV7670_COM8_AEC                0x01

// HSYNC instead of HREF
#define OV7670_COM10_HSYNC             0x40

// Suppress PCLK on horiz blank
#define OV7670_COM10_PCLK_HB           0x20

// Reverse HREF
#define OV7670_COM10_HREF_REV          0x08

// VSYNC on clock leading edge
#define OV7670_COM10_VS_LEAD           0x04

// VSYNC negative
#define OV7670_COM10_VS_NEG            0x02

// HSYNC negative
#define OV7670_COM10_HS_NEG            0x01

// Mirror image
#define OV7670_MVFP_MIRROR             0x20

// Vertical flip
#define OV7670_MVFP_FLIP               0x10

// UYVY or VYUY - see com13
#define OV7670_TSLB_YLAST              0x04

// NIght mode enable
#define OV7670_COM11_NIGHT             0x80

// Two bit NM frame rate
#define OV7670_COM11_NMFR              0x60

// Auto detect 50/60 Hz
#define OV7670_COM11_HZAUTO            0x10

// Manual 50Hz select
#define OV7670_COM11_50HZ              0x08

// Exp
#define OV7670_COM11_EXP               0x02

// HREF always
#define OV7670_COM12_HREF              0x80

// Gamma enable
#define OV7670_COM13_GAMMA             0x80

// UV saturation auto adjustment
#define OV7670_COM13_UVSAT             0x40

// V before U - w/TSLB
#define OV7670_COM13_UVSWAP            0x01

// DCW/PCLK-scale enable
#define OV7670_COM14_DCWEN             0x10

// Data range 10 to F0
#define OV7670_COM15_R10F0             0x00

// Data range 01 to FE
#define OV7670_COM15_R01FE             0x80

// Data range 00 to FF
#define OV7670_COM15_R00FF             0xc0

// RGB565 output
#define OV7670_COM15_RGB565            0x10

// RGB555 output
#define OV7670_COM15_RGB555            0x30

// AWB gain enable
#define OV7670_COM16_AWBGAIN           0x08

// AEC window - must match COM4
#define OV7670_COM17_AECWIN            0xc0

// DSP Color bar
#define OV7670_COM17_CBAR              0x08

// Length
#define OV7670_CMATRIX_LEN             0x06

// Black pixel correction enable
#define OV7670_R76_BLKPCOR             0x80

// White pixel correction enable
#define OV7670_R76_WHTPCOR             0x40

// Turn on RGB444, overrides 5x5
#define OV7670_RGB444_ENABLE           0x02

// Empty nibble at end
#define OV7670_RGB444_RGBX             0x01


class CameraOV7670 : public Camera {
private:

    unsigned char (*read)();

    unsigned char vsyncPin;

    unsigned char hsyncPin;

    unsigned char address;
public:

    enum Register {

        // Gain lower 8 bits (rest in vref)
        REG_GAIN = 0x00,

        // blue gain
        REG_BLUE = 0x01,

        // red gain
        REG_RED = 0x02,

        // Pieces of GAIN, VSTART, VSTOP
        REG_VREF = 0x03,

        // Control 1
        REG_COM1 = 0x04,

        // U/B Average level
        REG_BAVE = 0x05,

        // Y/Gb Average level
        REG_GBAVE = 0x06,

        // AEC MS 5 bits
        REG_AECHH = 0x07,

        // V/R Average level
        REG_RAVE = 0x08,

        // Control 2
        REG_COM2 = 0x09,

        // Product ID MSB
        REG_PID = 0x0a,

        // Product ID LSB
        REG_VER = 0x0b,

        // Control 3
        REG_COM3 = 0x0c,

        // Control 4
        REG_COM4 = 0x0d,

        // All "reserved"
        REG_COM5 = 0x0e,

        // Control 6
        REG_COM6 = 0x0f,

        // More bits of AEC value
        REG_AECH = 0x10,

        // Clocl control
        REG_CLKRC = 0x11,

        // Control 7
        REG_COM7 = 0x12,

        // Control 8
        REG_COM8 = 0x13,

        // Control 9  - gain ceiling
        REG_COM9 = 0x14,

        // Control 10
        REG_COM10 = 0x15,

        // Horiz start high bits
        REG_HSTART = 0x17,

        // Horiz stop high bits
        REG_HSTOP = 0x18,

        // Vert start high bits
        REG_VSTART = 0x19,

        // Vert stop high bits
        REG_VSTOP = 0x1a,

        // Pixel delay after HREF
        REG_PSHFT = 0x1b,

        // Manuf. ID high
        REG_MIDH = 0x1c,

        // Manuf. ID low
        REG_MIDL = 0x1d,

        // Mirror / vflip
        REG_MVFP = 0x1e,

        // AGC upper limit
        REG_AEW = 0x24,

        // AGC lower limit
        REG_AEB = 0x25,

        // AGC/AEC fast mode op region
        REG_VPT = 0x26,

        // HSYNC rising edge delay
        REG_HSYST = 0x30,

        // HSYNC falling edge delay
        REG_HSYEN = 0x31,

        // HREF pieces
        REG_HREF = 0x32,

        // Lots of stuff
        REG_TSLB = 0x3a,

        // Control 11
        REG_COM11 = 0x3b,

        // Control 12
        REG_COM12 = 0x3c,

        // Control 13
        REG_COM13 = 0x3d,

        // Control 14
        REG_COM14 = 0x3e,

        // Edge enhancement factor
        REG_EDGE = 0x3f,

        // Control 15
        REG_COM15 = 0x40,

        // Control 16
        REG_COM16 = 0x41,

        // Control 17
        REG_COM17 = 0x42,

        // CMatrix base
        REG_CMATRIX_BASE = 0x4f,

        // CMatrix sign
        REG_CMATRIX_SIGN = 0x58,

        // Brightness
        REG_BRIGHT = 0x55,

        // Contrast control
        REG_CONTRAS = 0x56,

        // Fix gain control
        REG_GFIX = 0x69,

        // OV's name
        REG_R76 = 0x76,

        // RGB 444 control
        REG_RGB444 = 0x8c,

        // Hist AEC/AGC control 1
        REG_HAECC1 = 0x9f,

        // Hist AEC/AGC control 2
        REG_HAECC2 = 0xa0,

        // 50hz banding step limit
        REG_BD50MAX = 0xa5,

        // Hist AEC/AGC control 3
        REG_HAECC3 = 0xa6,

        // Hist AEC/AGC control 4
        REG_HAECC4 = 0xa7,

        // Hist AEC/AGC control 5
        REG_HAECC5 = 0xa8,

        // Hist AEC/AGC control 6
        REG_HAECC6 = 0xa9,

        // Hist AEC/AGC control 7
        REG_HAECC7 = 0xaa,

        // 60hz banding step limit
        REG_BD60MAX = 0xab
    };

    /**
     * Public constructor.
     *
     * @param read				The reader function.
     * @param vsyncPin			The vertical sync pin number.
     * @param hsyncPin			The horizontal sync pin number.
     * @param pclkPin			The clock pin number.
     */
    CameraOV7670(unsigned char (*read)(), unsigned char vsyncPin,
            unsigned char hsyncPin);

    /**
     * Clears the buffers.
     */
    void clearBuffers();

    /**
     * Returns a frame.
     * 
     * @param out 				The frame out.
     * @return 					The frame size.
     */
    int readFrame(OutputStream *out);
};

#endif /* __ARDUINO_DRIVER_CAMERA_OV7670_H__ */
