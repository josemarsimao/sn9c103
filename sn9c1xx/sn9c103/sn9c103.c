/***************************************************************************
 * V4L2 driver for SN9C1xx PC Camera Controllers                           *
 *                                                                         *
 * Copyright (C) 2004-2008 by Luca Risolia <luca.risolia@studio.unibo.it>  *
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU General Public License as published by    *
 * the Free Software Foundation; either version 2 of the License, or       *
 * (at your option) any later version.                                     *
 *                                                                         *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU General Public License for more details.                            *
 *                                                                         *
 * You should have received a copy of the GNU General Public License       *
 * along with this program; if not, write to the Free Software             *
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.               *
 ***************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/compiler.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/stat.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/page-flags.h>
#include <asm/byteorder.h>
#include <asm/page.h>
#include <asm/uaccess.h>

//#include "sn9c102.h"
#include <linux/version.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/stddef.h>
#include <linux/kref.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>







///***************************
///#include "sn9c102_config.h"
///***************************


#include <linux/types.h>
#include <linux/jiffies.h>

#define SN9C102_DEBUG
#define SN9C102_DEBUG_LEVEL       2
#define SN9C102_ENABLE_SYSFS
#define SN9C102_MAX_DEVICES       64
#define SN9C102_MAX_OPENS         0
#define SN9C102_PRESERVE_IMGSCALE 0
#define SN9C102_FORCE_MUNMAP      0
#define SN9C102_MAX_FRAMES        32
#define SN9C102_URBS              2
#define SN9C102_ISO_PACKETS       7
#define SN9C102_ALTERNATE_SETTING 8
#define SN9C102_URB_TIMEOUT       msecs_to_jiffies(2 * SN9C102_ISO_PACKETS)
#define SN9C102_CTRL_TIMEOUT      300
#define SN9C102_FRAME_TIMEOUT     0

/*****************************************************************************/

static const u8 SN9C102_Y_QTABLE0[64] = {
	 8,   5,   5,   8,  12,  20,  25,  30,
	 6,   6,   7,   9,  13,  29,  30,  27,
	 7,   6,   8,  12,  20,  28,  34,  28,
	 7,   8,  11,  14,  25,  43,  40,  31,
	 9,  11,  18,  28,  34,  54,  51,  38,
	12,  17,  27,  32,  40,  52,  56,  46,
	24,  32,  39,  43,  51,  60,  60,  50,
	36,  46,  47,  49,  56,  50,  51,  49
};

static const u8 SN9C102_UV_QTABLE0[64] = {
	 8,   9,  12,  23,  49,  49,  49,  49,
	 9,  10,  13,  33,  49,  49,  49,  49,
	12,  13,  28,  49,  49,  49,  49,  49,
	23,  33,  49,  49,  49,  49,  49,  49,
	49,  49,  49,  49,  49,  49,  49,  49,
	49,  49,  49,  49,  49,  49,  49,  49,
	49,  49,  49,  49,  49,  49,  49,  49,
	49,  49,  49,  49,  49,  49,  49,  49
};

static const u8 SN9C102_Y_QTABLE1[64] = {
	16,  11,  10,  16,  24,  40,  51,  61,
	12,  12,  14,  19,  26,  58,  60,  55,
	14,  13,  16,  24,  40,  57,  69,  56,
	14,  17,  22,  29,  51,  87,  80,  62,
	18,  22,  37,  56,  68, 109, 103,  77,
	24,  35,  55,  64,  81, 104, 113,  92,
	49,  64,  78,  87, 103, 121, 120, 101,
	72,  92,  95,  98, 112, 100, 103,  99
};

static const u8 SN9C102_UV_QTABLE1[64] = {
	17,  18,  24,  47,  99,  99,  99,  99,
	18,  21,  26,  66,  99,  99,  99,  99,
	24,  26,  56,  99,  99,  99,  99,  99,
	47,  66,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99
};





///***********************************
///#include "sensors/sn9c102_sensor.h"
///***********************************

#include <linux/usb.h>
#include <linux/videodev2.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/errno.h>
#include <asm/types.h>

struct sn9c102_device;
struct sn9c102_sensor;

/*****************************************************************************/

/*
   OVERVIEW.
   This is a small interface that allows you to add support for any CCD/CMOS
   image sensors connected to the SN9C1XX bridges. The entire API is documented
   below. In the most general case, to support a sensor there are three steps
   you have to follow:
   1) define the main "sn9c102_sensor" structure by setting the basic fields;
   2) write a probing function to be called by the core module when the USB
      camera is recognized, then add both the USB ids and the name of that
      function to the two corresponding tables in sn9c102_devtable.h;
   3) implement the methods that you want/need (and fill the rest of the main
      structure accordingly).
   "sn9c102_pas106b.c" is an example of all this stuff. Remember that you do
   NOT need to touch the source code of the core module for the things to work
   properly, unless you find bugs or flaws in it. Finally, do not forget to
   read the V4L2 API for completeness.
*/

/*****************************************************************************/

enum sn9c102_bridge {
	BRIDGE_SN9C101 = 0x01,
	BRIDGE_SN9C102 = 0x02,
	BRIDGE_SN9C103 = 0x04,
	BRIDGE_SN9C105 = 0x08,
	BRIDGE_SN9C120 = 0x10,
};

/* Return the bridge name */
extern enum sn9c102_bridge sn9c102_get_bridge(struct sn9c102_device* cam);

/* Return a pointer the sensor struct attached to the camera */
extern struct sn9c102_sensor* sn9c102_get_sensor(struct sn9c102_device* cam);

/* Identify a device */
extern struct sn9c102_device*
sn9c102_match_id(struct sn9c102_device* cam, const struct usb_device_id *id);

/* Attach a probed sensor to the camera. */
extern void
sn9c102_attach_sensor(struct sn9c102_device* cam,
                      const struct sn9c102_sensor* sensor);

/*
   Read/write routines: they always return -1 on error, 0 or the read value
   otherwise. NOTE that a real read operation is not supported by the SN9C1XX
   chip for some of its registers. To work around this problem, a pseudo-read
   call is provided instead: it returns the last successfully written value
   on the register (0 if it has never been written), the usual -1 on error.
*/

/* The "try" I2C I/O versions are used when probing the sensor */
extern int sn9c102_i2c_try_write(struct sn9c102_device*,
                                 const struct sn9c102_sensor*, u8 address,
                                 u8 value);
extern int sn9c102_i2c_try_read(struct sn9c102_device*,
                                const struct sn9c102_sensor*, u8 address);

/*
   These must be used if and only if the sensor doesn't implement the standard
   I2C protocol. There are a number of good reasons why you must use the
   single-byte versions of these functions: do not abuse. The first function
   writes n bytes, from data0 to datan, to registers 0x09 - 0x09+n of SN9C1XX
   chip. The second one programs the registers 0x09 and 0x10 with data0 and
   data1, and places the n bytes read from the sensor register table in the
   buffer pointed by 'buffer'. Both the functions return -1 on error; the write
   version returns 0 on success, while the read version returns the first read
   byte.
*/
extern int sn9c102_i2c_try_raw_write(struct sn9c102_device* cam,
                                     const struct sn9c102_sensor* sensor, u8 n,
                                     u8 data0, u8 data1, u8 data2, u8 data3,
                                     u8 data4, u8 data5);
extern int sn9c102_i2c_try_raw_read(struct sn9c102_device* cam,
                                    const struct sn9c102_sensor* sensor,
                                    u8 data0, u8 data1, u8 n, u8 buffer[]);

/* To be used after the sensor struct has been attached to the camera struct */
extern int sn9c102_i2c_write(struct sn9c102_device*, u8 address, u8 value);
extern int sn9c102_i2c_read(struct sn9c102_device*, u8 address);

/* I/O on registers in the bridge. Could be used by the sensor methods too */
extern int sn9c102_read_reg(struct sn9c102_device*, u16 index);
extern int sn9c102_pread_reg(struct sn9c102_device*, u16 index);
extern int sn9c102_write_reg(struct sn9c102_device*, u8 value, u16 index);
extern int sn9c102_write_regs(struct sn9c102_device*, const u8 valreg[][2],
                              int count);
/*
   Write multiple registers with constant values. For example:
   sn9c102_write_const_regs(cam, {0x00, 0x14}, {0x60, 0x17}, {0x0f, 0x18});
   Register adresses must be < 256.
*/
#define sn9c102_write_const_regs(sn9c102_device, data...)                     \
	({ static const u8 _valreg[][2] = {data};                             \
	sn9c102_write_regs(sn9c102_device, _valreg, ARRAY_SIZE(_valreg)); })

/*****************************************************************************/

enum sn9c102_i2c_sysfs_ops {
	SN9C102_I2C_READ = 0x01,
	SN9C102_I2C_WRITE = 0x02,
};

enum sn9c102_i2c_frequency { /* sensors may support both the frequencies */
	SN9C102_I2C_100KHZ = 0x01,
	SN9C102_I2C_400KHZ = 0x02,
};

enum sn9c102_i2c_interface {
	SN9C102_I2C_2WIRES,
	SN9C102_I2C_3WIRES,
};

#define SN9C102_MAX_CTRLS (V4L2_CID_LASTP1-V4L2_CID_BASE+10)

struct sn9c102_sensor {
	char name[32], /* sensor name */
	     maintainer[64]; /* name of the mantainer <email> */

	enum sn9c102_bridge supported_bridge; /* supported SN9C1xx bridges */

	/* Supported operations through the 'sysfs' interface */
	enum sn9c102_i2c_sysfs_ops sysfs_ops;

	/*
	   These sensor capabilities must be provided if the SN9C1XX controller
	   needs to communicate through the sensor serial interface by using
	   at least one of the i2c functions available.
	*/
	enum sn9c102_i2c_frequency frequency;
	enum sn9c102_i2c_interface interface;

	/*
	   This identifier must be provided if the image sensor implements
	   the standard I2C protocol.
	*/
	u8 i2c_slave_id; /* reg. 0x09 */

	/*
	   NOTE: Where not noted,most of the functions below are not mandatory.
	         Set to null if you do not implement them. If implemented,
	         they must return 0 on success, the proper error otherwise.
	*/

	int (*init)(struct sn9c102_device* cam);
	/*
	   This function will be called after the sensor has been attached.
	   It should be used to initialize the sensor only, but may also
	   configure part of the SN9C1XX chip if necessary. You don't need to
	   setup picture settings like brightness, contrast, etc.. here, if
	   the corrisponding controls are implemented (see below), since
	   they are adjusted in the core driver by calling the set_ctrl()
	   method after init(), where the arguments are the default values
	   specified in the v4l2_queryctrl list of supported controls;
	   Same suggestions apply for other settings, _if_ the corresponding
	   methods are present; if not, the initialization must configure the
	   sensor according to the default configuration structures below.
	*/

	struct v4l2_queryctrl qctrl[SN9C102_MAX_CTRLS];
	/*
	   Optional list of default controls, defined as indicated in the
	   V4L2 API. Menu type controls are not handled by this interface.
	*/

	int (*get_ctrl)(struct sn9c102_device* cam, struct v4l2_control* ctrl);
	int (*set_ctrl)(struct sn9c102_device* cam,
	                const struct v4l2_control* ctrl);
	/*
	   You must implement at least the set_ctrl method if you have defined
	   the list above. The returned value must follow the V4L2
	   specifications for the VIDIOC_G|C_CTRL ioctls. V4L2_CID_H|VCENTER
	   are not supported by this driver, so do not implement them. Also,
	   you don't have to check whether the passed values are out of bounds,
	   given that this is done by the core module.
	*/

	struct v4l2_cropcap cropcap;
	/*
	   Think the image sensor as a grid of R,G,B monochromatic pixels
	   disposed according to a particular Bayer pattern, which describes
	   the complete array of pixels, from (0,0) to (xmax, ymax). We will
	   use this coordinate system from now on. It is assumed the sensor
	   chip can be programmed to capture/transmit a subsection of that
	   array of pixels: we will call this subsection "active window".
	   It is not always true that the largest achievable active window can
	   cover the whole array of pixels. The V4L2 API defines another
	   area called "source rectangle", which, in turn, is a subrectangle of
	   the active window. The SN9C1XX chip is always programmed to read the
	   source rectangle.
	   The bounds of both the active window and the source rectangle are
	   specified in the cropcap substructures 'bounds' and 'defrect'.
	   By default, the source rectangle should cover the largest possible
	   area. Again, it is not always true that the largest source rectangle
	   can cover the entire active window, although it is a rare case for
	   the hardware we have. The bounds of the source rectangle _must_ be
	   multiple of 16 and must use the same coordinate system as indicated
	   before; their centers shall align initially.
	   If necessary, the sensor chip must be initialized during init() to
	   set the bounds of the active sensor window; however, by default, it
	   usually covers the largest achievable area (maxwidth x maxheight)
	   of pixels, so no particular initialization is needed, if you have
	   defined the correct default bounds in the structures.
	   See the V4L2 API for further details.
	   NOTE: once you have defined the bounds of the active window
	         (struct cropcap.bounds) you must not change them.anymore.
	   Only 'bounds' and 'defrect' fields are mandatory, other fields
	   will be ignored.
	*/

	int (*set_crop)(struct sn9c102_device* cam,
	                const struct v4l2_rect* rect);
	/*
	   To be called on VIDIOC_C_SETCROP. The core module always calls a
	   default routine which configures the appropriate SN9C1XX regs (also
	   scaling), but you may need to override/adjust specific stuff.
	   'rect' contains width and height values that are multiple of 16: in
	   case you override the default function, you always have to program
	   the chip to match those values; on error return the corresponding
	   error code without rolling back.
	   NOTE: in case, you must program the SN9C1XX chip to get rid of
	         blank pixels or blank lines at the _start_ of each line or
	         frame after each HSYNC or VSYNC, so that the image starts with
	         real RGB data (see regs 0x12, 0x13) (having set H_SIZE and,
	         V_SIZE you don't have to care about blank pixels or blank
	         lines at the end of each line or frame).
	*/

	struct v4l2_pix_format pix_format;
	/*
	   What you have to define here are: 1) initial 'width' and 'height' of
	   the target rectangle 2) the initial 'pixelformat', which can be
	   either V4L2_PIX_FMT_SN9C10X,V4L2_PIX_FMT_JPEG (for compressed video)
	   or V4L2_PIX_FMT_SBGGR8 3) 'priv', which we'll be used to indicate
	   the number of bits per pixel for uncompressed video, 8 or 9 (despite
	   the current value of 'pixelformat').
	   NOTE 1: both 'width' and 'height' _must_ be either 1/1 or 1/2 or 1/4
	           of cropcap.defrect.width and cropcap.defrect.height. I
	           suggest 1/1.
	   NOTE 2: The initial compression quality is defined by the first bit
	           of reg 0x17 during the initialization of the image sensor.
	   NOTE 3: as said above, you have to program the SN9C1XX chip to get
	           rid of any blank pixels, so that the output of the sensor
	           matches the RGB bayer sequence (i.e. BGBGBG...GRGRGR).
	*/

	int (*set_pix_format)(struct sn9c102_device* cam,
	                      const struct v4l2_pix_format* pix);
	/*
	   To be called on VIDIOC_S_FMT, when switching from the SBGGR8 to
	   SN9C10X pixel format or viceversa. On error return the corresponding
	   error code without rolling back.
	*/

	/*
	   Do NOT write to the data below, it's READ ONLY. It is used by the
	   core module to store successfully updated values of the above
	   settings, for rollbacks..etc..in case of errors during atomic I/O
	*/
	struct v4l2_queryctrl _qctrl[SN9C102_MAX_CTRLS];
	struct v4l2_rect _rect;
};

/*****************************************************************************/

/* Private ioctl's for control settings supported by some image sensors */
#define SN9C102_V4L2_CID_DAC_MAGNITUDE (V4L2_CID_PRIVATE_BASE + 0)
#define SN9C102_V4L2_CID_GREEN_BALANCE (V4L2_CID_PRIVATE_BASE + 1)
#define SN9C102_V4L2_CID_RESET_LEVEL (V4L2_CID_PRIVATE_BASE + 2)
#define SN9C102_V4L2_CID_PIXEL_BIAS_VOLTAGE (V4L2_CID_PRIVATE_BASE + 3)
#define SN9C102_V4L2_CID_GAMMA (V4L2_CID_PRIVATE_BASE + 4)
#define SN9C102_V4L2_CID_BAND_FILTER (V4L2_CID_PRIVATE_BASE + 5)
#define SN9C102_V4L2_CID_BRIGHT_LEVEL (V4L2_CID_PRIVATE_BASE + 6)


///*****************************
///#include "sn9c102_devtable.h"
///*****************************


#include <linux/usb.h>

struct sn9c102_device;

/*
   Each SN9C1xx camera has proper PID/VID identifiers.
   SN9C103, SN9C105, SN9C120 support multiple interfaces, but we only have to
   handle the video class interface.
*/
#define SN9C102_USB_DEVICE(vend, prod, bridge)                                \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE |                           \
	               USB_DEVICE_ID_MATCH_INT_CLASS,                         \
	.idVendor = (vend),                                                   \
	.idProduct = (prod),                                                  \
	.bInterfaceClass = 0xff,                                              \
	.driver_info = (bridge)

static const struct usb_device_id sn9c102_id_table[] = {
	/* SN9C101 and SN9C102 */
	{ SN9C102_USB_DEVICE(0x0c45, 0x6001, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6005, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6007, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6009, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6011, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x600d, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6019, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6024, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6025, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6028, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6029, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x602a, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x602b, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x602c, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x602d, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x602e, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6030, BRIDGE_SN9C102), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x603f, BRIDGE_SN9C102), },
	/* SN9C103 */
	{ SN9C102_USB_DEVICE(0x0c45, 0x6080, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6082, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6083, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6088, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x608a, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x608b, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x608c, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x608e, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x608f, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60a0, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60a2, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60a3, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60a8, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60aa, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ab, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ac, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ae, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60af, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60b0, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60b2, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60b3, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60b8, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ba, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60bb, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60bc, BRIDGE_SN9C103), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60be, BRIDGE_SN9C103), },
	/* SN9C105 */
	{ SN9C102_USB_DEVICE(0x045e, 0x00f5, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x045e, 0x00f7, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0471, 0x0327, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0471, 0x0328, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x06f8, 0x3004, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60c0, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60c2, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60c8, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60cc, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ea, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ec, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60ef, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60fa, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60fb, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60fc, BRIDGE_SN9C105), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x60fe, BRIDGE_SN9C105), },
	/* SN9C120 */
	{ SN9C102_USB_DEVICE(0x0458, 0x7025, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6102, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6108, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x610f, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x612a, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x612c, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6130, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x6138, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x613a, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x613b, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x613c, BRIDGE_SN9C120), },
	{ SN9C102_USB_DEVICE(0x0c45, 0x613e, BRIDGE_SN9C120), },
	{ }
};


enum sn9c102_frame_state {
	F_UNUSED,
	F_QUEUED,
	F_GRABBING,
	F_DONE,
	F_ERROR,
};

struct sn9c102_frame_t {
	void* bufmem;
	struct v4l2_buffer buf;
	enum sn9c102_frame_state state;
	struct list_head frame;
	unsigned long vma_use_count;
};

enum sn9c102_dev_state {
	DEV_INITIALIZED = 0x01,
	DEV_DISCONNECTED = 0x02,
	DEV_MISCONFIGURED = 0x04,
};

enum sn9c102_io_method {
	IO_NONE = 0,
	IO_READ,
	IO_MMAP,
};

enum sn9c102_stream_state {
	STREAM_OFF = 0,
	STREAM_INTERRUPT,
	STREAM_ON,
};

typedef char sn9c102_sof_header_t[62];

struct sn9c102_sof_t {
	sn9c102_sof_header_t header;
	u16 bytesread;
};

struct sn9c102_sysfs_attr {
	u16 reg, i2c_reg;
	sn9c102_sof_header_t frame_header;
};

struct sn9c102_module_param {
	u8 force_munmap, max_opens;
	u16 frame_timeout;
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static DEFINE_MUTEX(sn9c102_sysfs_lock);
#endif

static DEFINE_MUTEX(sn9c102_dev_lock);

struct sn9c102_device {
	struct video_device* v4ldev;

	enum sn9c102_bridge bridge;
	struct sn9c102_sensor sensor;

	struct usb_device* usbdev;
	struct urb* urb[SN9C102_URBS];
	void* transfer_buffer[SN9C102_URBS];
	u8* control_buffer;

	struct sn9c102_frame_t *frame_current, frame[SN9C102_MAX_FRAMES];
	struct list_head inqueue, outqueue;
	u32 frame_count, nbuffers, nreadbuffers;

	enum sn9c102_io_method io;
	enum sn9c102_stream_state stream;

	struct v4l2_jpegcompression compression;

	struct sn9c102_sysfs_attr sysfs;
	struct sn9c102_sof_t sof;
	u16 reg[384];

	struct sn9c102_module_param module_param;

	struct file* priority;
	struct kref kref;
	enum sn9c102_dev_state state;
	u8 users;

	struct completion probe;
	struct mutex fileop_mutex;
	spinlock_t queue_lock;
	wait_queue_head_t wait_open, wait_frame, wait_stream;
};

/*****************************************************************************/

struct sn9c102_device*
sn9c102_match_id(struct sn9c102_device* cam, const struct usb_device_id *id)
{
	return usb_match_id(usb_ifnum_to_if(cam->usbdev, 0), id) ? cam : NULL;
}


void
sn9c102_attach_sensor(struct sn9c102_device* cam,
                      const struct sn9c102_sensor* sensor)
{
	memcpy(&cam->sensor, sensor, sizeof(struct sn9c102_sensor));
}


enum sn9c102_bridge
sn9c102_get_bridge(struct sn9c102_device* cam)
{
	return cam->bridge;
}


struct sn9c102_sensor* sn9c102_get_sensor(struct sn9c102_device* cam)
{
	return &cam->sensor;
}

/*****************************************************************************/

#undef DBG
#undef KDBG
#ifdef SN9C102_DEBUG
#	define DBG(level, fmt, args...)                                       \
do {                                                                          \
	if (debug >= (level)) {                                               \
		if ((level) == 1)                                             \
			dev_err(&cam->usbdev->dev, fmt "\n", ## args);        \
		else if ((level) == 2)                                        \
			dev_info(&cam->usbdev->dev, fmt "\n", ## args);       \
		else if ((level) >= 3)                                        \
			dev_info(&cam->usbdev->dev, "[%s:%d] " fmt "\n",      \
			         __func__, __LINE__ , ## args);               \
	}                                                                     \
} while (0)
#	define V4LDBG(level, name, cmd)                                       \
do {                                                                          \
	if (debug >= (level))                                                 \
		/*v4l_print_ioctl(name, cmd);                                   \ */ \
		v4l_printk_ioctl(name, cmd);                                   \
} while (0)
#	define KDBG(level, fmt, args...)                                      \
do {                                                                          \
	if (debug >= (level)) {                                               \
		if ((level) == 1 || (level) == 2)                             \
			pr_info("sn9c102: " fmt "\n", ## args);               \
		else if ((level) == 3)                                        \
			pr_debug("sn9c102: [%s:%d] " fmt "\n",                \
			         __func__, __LINE__ , ## args);               \
	}                                                                     \
} while (0)
#else
#	define DBG(level, fmt, args...) do {;} while(0)
#	define V4LDBG(level, name, cmd) do {;} while(0)
#	define KDBG(level, fmt, args...) do {;} while(0)
#endif

#undef PDBG
#define PDBG(fmt, args...)                                                    \
dev_info(&cam->usbdev->dev, "[%s:%s:%d] " fmt "\n", __FILE__, __func__,       \
         __LINE__ , ## args)

#undef PDBGG
#define PDBGG(fmt, args...) do {;} while(0) /* placeholder */








/*
   Probing functions: on success, you must attach the sensor to the camera
   by calling sn9c102_attach_sensor().
   To enable the I2C communication, you might need to perform a really basic
   initialization of the SN9C1XX chip.
   Functions must return 0 on success, the appropriate error otherwise.
*/


static int ov7630_init(struct sn9c102_device* cam)
{
	int err = 0;

	switch (sn9c102_get_bridge(cam)) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
		err = sn9c102_write_const_regs(cam, {0x00, 0x14}, {0x64, 0x17},
		                               {0x0f, 0x18}, {0x50, 0x19});

		err += sn9c102_i2c_write(cam, 0x12, 0x8d);
		err += sn9c102_i2c_write(cam, 0x12, 0x7c);
		err += sn9c102_i2c_write(cam, 0x11, 0x01);
		err += sn9c102_i2c_write(cam, 0x13, 0x01);
		err += sn9c102_i2c_write(cam, 0x15, 0x34);
		err += sn9c102_i2c_write(cam, 0x16, 0x03);
		err += sn9c102_i2c_write(cam, 0x17, 0x1c);
		err += sn9c102_i2c_write(cam, 0x18, 0xbd);
		err += sn9c102_i2c_write(cam, 0x19, 0x06);
		err += sn9c102_i2c_write(cam, 0x1a, 0xf6);
		err += sn9c102_i2c_write(cam, 0x1b, 0x04);
		err += sn9c102_i2c_write(cam, 0x20, 0x44);
		err += sn9c102_i2c_write(cam, 0x23, 0xee);
		err += sn9c102_i2c_write(cam, 0x26, 0xa0);
		err += sn9c102_i2c_write(cam, 0x27, 0x9a);
		err += sn9c102_i2c_write(cam, 0x28, 0xa0);
		err += sn9c102_i2c_write(cam, 0x29, 0x30);
		err += sn9c102_i2c_write(cam, 0x2f, 0x3d);
		err += sn9c102_i2c_write(cam, 0x30, 0x24);
		err += sn9c102_i2c_write(cam, 0x32, 0x86);
		err += sn9c102_i2c_write(cam, 0x60, 0xa9);
		err += sn9c102_i2c_write(cam, 0x61, 0x4a);
		err += sn9c102_i2c_write(cam, 0x65, 0x00);
		err += sn9c102_i2c_write(cam, 0x69, 0x38);
		err += sn9c102_i2c_write(cam, 0x6f, 0x88);
		err += sn9c102_i2c_write(cam, 0x70, 0x0b);
		err += sn9c102_i2c_write(cam, 0x71, 0x00);
		err += sn9c102_i2c_write(cam, 0x74, 0x21);
		err += sn9c102_i2c_write(cam, 0x75, 0x8e);
		err += sn9c102_i2c_write(cam, 0x7d, 0xf7);
		break;
	case BRIDGE_SN9C103:
		err = sn9c102_write_const_regs(cam, {0x46, 0x01},
		                               {0x44, 0x02}, {0x00, 0x03},
		                               {0x1a, 0x04}, {0x20, 0x05},
		                               {0x20, 0x06}, {0x20, 0x07},
		                               {0x80, 0x08}, {0x21, 0x09},
		                               {0x00, 0x0a}, {0x00, 0x0b},
		                               {0x00, 0x0c}, {0x00, 0x0d},
		                               {0x00, 0x0e}, {0x00, 0x0f},
		                               {0x03, 0x10}, {0x00, 0x11},
		                               {0x01, 0x12}, {0x01, 0x13},
		                               {0x0a, 0x14}, {0x28, 0x15},
		                               {0x1e, 0x16}, {0x60, 0x17},
		                               {0x8f, 0x18}, {0x20, 0x19},
		                               {0x1d, 0x1a}, {0x10, 0x1b},
		                               {0x02, 0x1c}, {0x03, 0x1d},
		                               {0x0f, 0x1e}, {0x0c, 0x1f},
		                               {0x00, 0x20}, {0x10, 0x21},
		                               {0x20, 0x22}, {0x30, 0x23},
		                               {0x40, 0x24}, {0x50, 0x25},
		                               {0x60, 0x26}, {0x70, 0x27},
		                               {0x80, 0x28}, {0x90, 0x29},
		                               {0xa0, 0x2a}, {0xb0, 0x2b},
		                               {0xc0, 0x2c}, {0xd0, 0x2d},
		                               {0xe0, 0x2e}, {0xf0, 0x2f},
		                               {0xff, 0x30});

		err += sn9c102_i2c_write(cam, 0x12, 0x8d);
		err += sn9c102_i2c_write(cam, 0x12, 0x0d);
		err += sn9c102_i2c_write(cam, 0x15, 0x34);
		err += sn9c102_i2c_write(cam, 0x11, 0x01);
		err += sn9c102_i2c_write(cam, 0x1b, 0x04);
		err += sn9c102_i2c_write(cam, 0x20, 0x44);
		err += sn9c102_i2c_write(cam, 0x23, 0xee);
		err += sn9c102_i2c_write(cam, 0x26, 0xa0);
		err += sn9c102_i2c_write(cam, 0x27, 0x9a);
		err += sn9c102_i2c_write(cam, 0x28, 0xa0);
		err += sn9c102_i2c_write(cam, 0x29, 0x30);
		err += sn9c102_i2c_write(cam, 0x2f, 0x3d);
		err += sn9c102_i2c_write(cam, 0x30, 0x24);
		err += sn9c102_i2c_write(cam, 0x32, 0x86);
		err += sn9c102_i2c_write(cam, 0x60, 0xa9);
		err += sn9c102_i2c_write(cam, 0x61, 0x4a);
		err += sn9c102_i2c_write(cam, 0x65, 0x00);
		err += sn9c102_i2c_write(cam, 0x69, 0x38);
		err += sn9c102_i2c_write(cam, 0x6f, 0x88);
		err += sn9c102_i2c_write(cam, 0x70, 0x0b);
		err += sn9c102_i2c_write(cam, 0x71, 0x00);
		err += sn9c102_i2c_write(cam, 0x74, 0x21);
		err += sn9c102_i2c_write(cam, 0x75, 0x8e);
		err += sn9c102_i2c_write(cam, 0x7d, 0xf7);
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		err = sn9c102_write_const_regs(cam, {0x40, 0x02}, {0x00, 0x03},
	                               {0x1a, 0x04}, {0x03, 0x10},
	                               {0x0a, 0x14}, {0xe2, 0x17},
	                               {0x0b, 0x18}, {0x00, 0x19},
	                               {0x1d, 0x1a}, {0x10, 0x1b},
	                               {0x02, 0x1c}, {0x03, 0x1d},
	                               {0x0f, 0x1e}, {0x0c, 0x1f},
	                               {0x00, 0x20}, {0x24, 0x21},
	                               {0x3b, 0x22}, {0x47, 0x23},
	                               {0x60, 0x24}, {0x71, 0x25},
	                               {0x80, 0x26}, {0x8f, 0x27},
	                               {0x9d, 0x28}, {0xaa, 0x29},
	                               {0xb8, 0x2a}, {0xc4, 0x2b},
	                               {0xd1, 0x2c}, {0xdd, 0x2d},
	                               {0xe8, 0x2e}, {0xf4, 0x2f},
	                               {0xff, 0x30}, {0x00, 0x3f},
	                               {0xc7, 0x40}, {0x01, 0x41},
	                               {0x44, 0x42}, {0x00, 0x43},
	                               {0x44, 0x44}, {0x00, 0x45},
	                               {0x44, 0x46}, {0x00, 0x47},
	                               {0xc7, 0x48}, {0x01, 0x49},
	                               {0xc7, 0x4a}, {0x01, 0x4b},
	                               {0xc7, 0x4c}, {0x01, 0x4d},
	                               {0x44, 0x4e}, {0x00, 0x4f},
	                               {0x44, 0x50}, {0x00, 0x51},
	                               {0x44, 0x52}, {0x00, 0x53},
	                               {0xc7, 0x54}, {0x01, 0x55},
	                               {0xc7, 0x56}, {0x01, 0x57},
	                               {0xc7, 0x58}, {0x01, 0x59},
	                               {0x44, 0x5a}, {0x00, 0x5b},
	                               {0x44, 0x5c}, {0x00, 0x5d},
	                               {0x44, 0x5e}, {0x00, 0x5f},
	                               {0xc7, 0x60}, {0x01, 0x61},
	                               {0xc7, 0x62}, {0x01, 0x63},
	                               {0xc7, 0x64}, {0x01, 0x65},
	                               {0x44, 0x66}, {0x00, 0x67},
	                               {0x44, 0x68}, {0x00, 0x69},
	                               {0x44, 0x6a}, {0x00, 0x6b},
	                               {0xc7, 0x6c}, {0x01, 0x6d},
	                               {0xc7, 0x6e}, {0x01, 0x6f},
	                               {0xc7, 0x70}, {0x01, 0x71},
	                               {0x44, 0x72}, {0x00, 0x73},
	                               {0x44, 0x74}, {0x00, 0x75},
	                               {0x44, 0x76}, {0x00, 0x77},
	                               {0xc7, 0x78}, {0x01, 0x79},
	                               {0xc7, 0x7a}, {0x01, 0x7b},
	                               {0xc7, 0x7c}, {0x01, 0x7d},
	                               {0x44, 0x7e}, {0x00, 0x7f},
	                               {0x17, 0x84}, {0x00, 0x85},
	                               {0x2e, 0x86}, {0x00, 0x87},
	                               {0x09, 0x88}, {0x00, 0x89},
	                               {0xe8, 0x8a}, {0x0f, 0x8b},
	                               {0xda, 0x8c}, {0x0f, 0x8d},
	                               {0x40, 0x8e}, {0x00, 0x8f},
	                               {0x37, 0x90}, {0x00, 0x91},
	                               {0xcf, 0x92}, {0x0f, 0x93},
	                               {0xfa, 0x94}, {0x0f, 0x95},
	                               {0x00, 0x96}, {0x00, 0x97},
	                               {0x00, 0x98}, {0x66, 0x99},
	                               {0x00, 0x9a}, {0x40, 0x9b},
	                               {0x20, 0x9c}, {0x00, 0x9d},
	                               {0x00, 0x9e}, {0x00, 0x9f},
	                               {0x2d, 0xc0}, {0x2d, 0xc1},
	                               {0x3a, 0xc2}, {0x00, 0xc3},
	                               {0x04, 0xc4}, {0x3f, 0xc5},
	                               {0x00, 0xc6}, {0x00, 0xc7},
	                               {0x50, 0xc8}, {0x3c, 0xc9},
	                               {0x28, 0xca}, {0xd8, 0xcb},
	                               {0x14, 0xcc}, {0xec, 0xcd},
	                               {0x32, 0xce}, {0xdd, 0xcf},
	                               {0x32, 0xd0}, {0xdd, 0xd1},
	                               {0x6a, 0xd2}, {0x50, 0xd3},
	                               {0x60, 0xd4}, {0x00, 0xd5},
	                               {0x00, 0xd6});

		err += sn9c102_i2c_write(cam, 0x12, 0x80);
		err += sn9c102_i2c_write(cam, 0x12, 0x48);
		err += sn9c102_i2c_write(cam, 0x01, 0x80);
		err += sn9c102_i2c_write(cam, 0x02, 0x80);
		err += sn9c102_i2c_write(cam, 0x03, 0x80);
		err += sn9c102_i2c_write(cam, 0x04, 0x10);
		err += sn9c102_i2c_write(cam, 0x05, 0x20);
		err += sn9c102_i2c_write(cam, 0x06, 0x80);
		err += sn9c102_i2c_write(cam, 0x11, 0x00);
		err += sn9c102_i2c_write(cam, 0x0c, 0x20);
		err += sn9c102_i2c_write(cam, 0x0d, 0x20);
		err += sn9c102_i2c_write(cam, 0x15, 0x80);
		err += sn9c102_i2c_write(cam, 0x16, 0x03);
		err += sn9c102_i2c_write(cam, 0x17, 0x1b);
		err += sn9c102_i2c_write(cam, 0x18, 0xbd);
		err += sn9c102_i2c_write(cam, 0x19, 0x05);
		err += sn9c102_i2c_write(cam, 0x1a, 0xf6);
		err += sn9c102_i2c_write(cam, 0x1b, 0x04);
		err += sn9c102_i2c_write(cam, 0x21, 0x1b);
		err += sn9c102_i2c_write(cam, 0x22, 0x00);
		err += sn9c102_i2c_write(cam, 0x23, 0xde);
		err += sn9c102_i2c_write(cam, 0x24, 0x10);
		err += sn9c102_i2c_write(cam, 0x25, 0x8a);
		err += sn9c102_i2c_write(cam, 0x26, 0xa0);
		err += sn9c102_i2c_write(cam, 0x27, 0xca);
		err += sn9c102_i2c_write(cam, 0x28, 0xa2);
		err += sn9c102_i2c_write(cam, 0x29, 0x74);
		err += sn9c102_i2c_write(cam, 0x2a, 0x88);
		err += sn9c102_i2c_write(cam, 0x2b, 0x34);
		err += sn9c102_i2c_write(cam, 0x2c, 0x88);
		err += sn9c102_i2c_write(cam, 0x2e, 0x00);
		err += sn9c102_i2c_write(cam, 0x2f, 0x00);
		err += sn9c102_i2c_write(cam, 0x30, 0x00);
		err += sn9c102_i2c_write(cam, 0x32, 0xc2);
		err += sn9c102_i2c_write(cam, 0x33, 0x08);
		err += sn9c102_i2c_write(cam, 0x4c, 0x40);
		err += sn9c102_i2c_write(cam, 0x4d, 0xf3);
		err += sn9c102_i2c_write(cam, 0x60, 0x05);
		err += sn9c102_i2c_write(cam, 0x61, 0x4a);
		err += sn9c102_i2c_write(cam, 0x62, 0x12);
		err += sn9c102_i2c_write(cam, 0x63, 0x57);
		err += sn9c102_i2c_write(cam, 0x64, 0x73);
		err += sn9c102_i2c_write(cam, 0x65, 0x00);
		err += sn9c102_i2c_write(cam, 0x66, 0x55);
		err += sn9c102_i2c_write(cam, 0x67, 0x01);
		err += sn9c102_i2c_write(cam, 0x68, 0xac);
		err += sn9c102_i2c_write(cam, 0x69, 0x38);
		err += sn9c102_i2c_write(cam, 0x6f, 0x1f);
		err += sn9c102_i2c_write(cam, 0x70, 0x01);
		err += sn9c102_i2c_write(cam, 0x71, 0x00);
		err += sn9c102_i2c_write(cam, 0x72, 0x10);
		err += sn9c102_i2c_write(cam, 0x73, 0x50);
		err += sn9c102_i2c_write(cam, 0x74, 0x20);
		err += sn9c102_i2c_write(cam, 0x76, 0x01);
		err += sn9c102_i2c_write(cam, 0x77, 0xf3);
		err += sn9c102_i2c_write(cam, 0x78, 0x90);
		err += sn9c102_i2c_write(cam, 0x79, 0x98);
		err += sn9c102_i2c_write(cam, 0x7a, 0x98);
		err += sn9c102_i2c_write(cam, 0x7b, 0x00);
		err += sn9c102_i2c_write(cam, 0x7c, 0x38);
		err += sn9c102_i2c_write(cam, 0x7d, 0xff);
		break;
	default:
		break;
	}

	return err;
}


static int ov7630_get_ctrl(struct sn9c102_device* cam,
                           struct v4l2_control* ctrl)
{
	enum sn9c102_bridge bridge = sn9c102_get_bridge(cam);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x10)) < 0)
			return -EIO;
		break;
	case V4L2_CID_RED_BALANCE:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102)
			ctrl->value = (sn9c102_pread_reg(cam, 0x10)
			               & 0x0F) << 3;
		else if (bridge == BRIDGE_SN9C105 || bridge == BRIDGE_SN9C120)
			ctrl->value = sn9c102_pread_reg(cam, 0x05);
		else
			ctrl->value = sn9c102_pread_reg(cam, 0x07);
		break;
	case V4L2_CID_BLUE_BALANCE:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102)
			ctrl->value = ((sn9c102_pread_reg(cam, 0x10)
			                & 0xF0) >> 4) << 3;
		else
			ctrl->value = sn9c102_pread_reg(cam, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102)
			ctrl->value = sn9c102_pread_reg(cam, 0x11) << 3;
		else if (bridge == BRIDGE_SN9C105 || bridge == BRIDGE_SN9C120)
			ctrl->value = sn9c102_pread_reg(cam, 0x07);
		else
			ctrl->value = sn9c102_pread_reg(cam, 0x05);
		break;
	case V4L2_CID_GAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x00)) < 0)
			return -EIO;
		ctrl->value &= 0x3f;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x0c)) < 0)
			return -EIO;
		ctrl->value &= 0x3f;
		break;
	case V4L2_CID_WHITENESS:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x0d)) < 0)
			return -EIO;
		ctrl->value &= 0x3f;
		break;
	case V4L2_CID_AUTOGAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x13)) < 0)
			return -EIO;
		ctrl->value &= 0x01;
		break;
	case V4L2_CID_VFLIP:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x75)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x80) ? 1 : 0;
		break;
	case V4L2_CID_BRIGHTNESS:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x06)) < 0)
			return -EIO;
		break;
	case SN9C102_V4L2_CID_GAMMA:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x14)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x04) ? 1 : 0;
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x2d)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x04) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int ov7630_set_ctrl(struct sn9c102_device* cam,
                           const struct v4l2_control* ctrl)
{
	enum sn9c102_bridge bridge = sn9c102_get_bridge(cam);
	u8 rb_gain;
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		err += sn9c102_i2c_write(cam, 0x10, ctrl->value);
		break;
	case V4L2_CID_RED_BALANCE:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102) {
			rb_gain = sn9c102_pread_reg(cam, 0x10);
			rb_gain = (rb_gain & 0xf0) | (ctrl->value >> 3);
			err += sn9c102_write_reg(cam, rb_gain, 0x10);
		} else if (bridge == BRIDGE_SN9C105 ||
		           bridge == BRIDGE_SN9C120)
			err += sn9c102_write_reg(cam, ctrl->value, 0x05);
		else
			err += sn9c102_write_reg(cam, ctrl->value, 0x07);
		break;
	case V4L2_CID_BLUE_BALANCE:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102) {
			rb_gain = sn9c102_pread_reg(cam, 0x10);
			rb_gain = (rb_gain & 0x0f) | ((ctrl->value >> 3) << 4);
			err += sn9c102_write_reg(cam, rb_gain, 0x10);
		} else
			err += sn9c102_write_reg(cam, ctrl->value, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102)
			err += sn9c102_write_reg(cam, ctrl->value >> 3, 0x11);
		else if (bridge == BRIDGE_SN9C105 || bridge == BRIDGE_SN9C120)
			err += sn9c102_write_reg(cam, ctrl->value, 0x07);
		else
			err += sn9c102_write_reg(cam, ctrl->value, 0x05);
		break;
	case V4L2_CID_GAIN:
		err += sn9c102_i2c_write(cam, 0x00, ctrl->value);
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		err += sn9c102_i2c_write(cam, 0x0c, ctrl->value);
		break;
	case V4L2_CID_WHITENESS:
		err += sn9c102_i2c_write(cam, 0x0d, ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		if (bridge == BRIDGE_SN9C101 || bridge == BRIDGE_SN9C102)
			err += sn9c102_i2c_write(cam, 0x13, ctrl->value);
		else
			err += sn9c102_i2c_write(cam, 0x13, ctrl->value |
			                                    (ctrl->value<<1));
		break;
	case V4L2_CID_VFLIP:
		err += sn9c102_i2c_write(cam, 0x75, 0x0e | (ctrl->value << 7));
		break;
	case V4L2_CID_BRIGHTNESS:
		err += sn9c102_i2c_write(cam, 0x06, ctrl->value);
		break;
	case SN9C102_V4L2_CID_GAMMA:
		err += sn9c102_i2c_write(cam, 0x14, ctrl->value << 2);
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		err += sn9c102_i2c_write(cam, 0x2d, 0x81 | (ctrl->value << 2));
		break;
	default:
		return -EINVAL;
	}

	return err ? -EIO : 0;
}


static int ov7630_set_crop(struct sn9c102_device* cam,
                           const struct v4l2_rect* rect)
{
	struct sn9c102_sensor* s = sn9c102_get_sensor(cam);
	int err = 0;
	u8 h_start = 0, v_start = (u8)(rect->top - s->cropcap.bounds.top) + 1;

	switch (sn9c102_get_bridge(cam)) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
	case BRIDGE_SN9C103:
		h_start = (u8)(rect->left - s->cropcap.bounds.left) + 1;
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		h_start = (u8)(rect->left - s->cropcap.bounds.left) + 4;
		break;
	default:
		break;
	}

	err += sn9c102_write_reg(cam, h_start, 0x12);
	err += sn9c102_write_reg(cam, v_start, 0x13);

	return err;
}


static int ov7630_set_pix_format(struct sn9c102_device* cam,
                                 const struct v4l2_pix_format* pix)
{
	int err = 0;

	switch (sn9c102_get_bridge(cam)) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
	case BRIDGE_SN9C103:
		if (pix->pixelformat == V4L2_PIX_FMT_SBGGR8)
			err += sn9c102_write_reg(cam, 0x50, 0x19);
		else
			err += sn9c102_write_reg(cam, 0x20, 0x19);
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		if (pix->pixelformat == V4L2_PIX_FMT_SBGGR8) {
			err += sn9c102_write_reg(cam, 0xe5, 0x17);
			err += sn9c102_i2c_write(cam, 0x11, 0x04);
		} else {
			err += sn9c102_write_reg(cam, 0xe2, 0x17);
			err += sn9c102_i2c_write(cam, 0x11, 0x02);
		}
		break;
	default:
		break;
	}

	return err;
}


static const struct sn9c102_sensor ov7630 = {
	.name = "OV7630",
	.maintainer = "Luca Risolia <luca.risolia@studio.unibo.it>",
	.supported_bridge = BRIDGE_SN9C101 | BRIDGE_SN9C102 | BRIDGE_SN9C103 |
	                    BRIDGE_SN9C105 | BRIDGE_SN9C120,
	.sysfs_ops = SN9C102_I2C_READ | SN9C102_I2C_WRITE,
	.frequency = SN9C102_I2C_100KHZ,
	.interface = SN9C102_I2C_2WIRES,
	.i2c_slave_id = 0x21,
	.init = &ov7630_init,
	.qctrl = {
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "global gain",
			.minimum = 0x00,
			.maximum = 0x3f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "brightness",
			.minimum = 0x00,
			.maximum = 0xff,
			.step = 0x01,
			.default_value = 0x80,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_GREEN_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "green balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x10,
			.flags = 0,
		},
		{
			.id = V4L2_CID_RED_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "red balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_BLUE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "blue balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x00,
			.maximum = 0xff,
			.step = 0x01,
			.default_value = 0x60,
			.flags = 0,
		},
		{
			.id = V4L2_CID_WHITENESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance background: red",
			.minimum = 0x00,
			.maximum = 0x3f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance background: blue",
			.minimum = 0x00,
			.maximum = 0x3f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_AUTOGAIN,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto gain-exposure",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x01,
			.flags = 0,
		},
		{
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "vertical flip",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x01,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_BAND_FILTER,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "band filter",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_GAMMA,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "rgb gamma",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
	},
	.get_ctrl = &ov7630_get_ctrl,
	.set_ctrl = &ov7630_set_ctrl,
	.cropcap = {
		.bounds = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
		.defrect = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
	},
	.set_crop = &ov7630_set_crop,
	.pix_format = {
		.width = 640,
		.height = 480,
		.pixelformat = V4L2_PIX_FMT_JPEG,
		.priv = 8,
	},
	.set_pix_format = &ov7630_set_pix_format
};


int sn9c102_probe_ov7630(struct sn9c102_device* cam)
{
	int pid, ver, err = 0;

	switch (sn9c102_get_bridge(cam)) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
		err = sn9c102_write_const_regs(cam, {0x01, 0x01}, {0x00, 0x01},
		                               {0x28, 0x17});
		break;
	case BRIDGE_SN9C103: /* do _not_ change anything! */
		err = sn9c102_write_const_regs(cam, {0x09, 0x01}, {0x44, 0x02},
		                               {0x44, 0x01}, {0x68, 0x17},
		                               {0x44, 0x01});
		pid = sn9c102_i2c_try_read(cam, &ov7630, 0x0a);
		if (err || pid < 0) /* try a different initialization */
			err += sn9c102_write_const_regs(cam, {0x01, 0x01},
			                                {0x00, 0x01});
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		err = sn9c102_write_const_regs(cam, {0x01, 0xf1}, {0x00, 0xf1},
		                               {0x29, 0x01}, {0x74, 0x02},
		                               {0x0e, 0x01}, {0x44, 0x01});
		break;
	default:
		break;
	}

	pid = sn9c102_i2c_try_read(cam, &ov7630, 0x0a);
	ver = sn9c102_i2c_try_read(cam, &ov7630, 0x0b);
	if (err || pid < 0 || ver < 0)
		return -EIO;
	if (pid != 0x76 || ver != 0x31)
		return -ENODEV;
	sn9c102_attach_sensor(cam, &ov7630);

	return 0;
}


static int ov7648_init(struct sn9c102_device* cam)
{
	int err = 0;

	err = sn9c102_write_const_regs(cam, {0x40, 0x02}, {0x00, 0x03},
	                               {0x1a, 0x04}, {0x03, 0x10},
	                               {0x0a, 0x14}, {0xe2, 0x17},
	                               {0x0b, 0x18}, {0x00, 0x19},
	                               {0x1d, 0x1a}, {0x10, 0x1b},
	                               {0x02, 0x1c}, {0x03, 0x1d},
	                               {0x0f, 0x1e}, {0x0c, 0x1f},
	                               {0x00, 0x20}, {0x24, 0x21},
	                               {0x3b, 0x22}, {0x47, 0x23},
	                               {0x60, 0x24}, {0x71, 0x25},
	                               {0x80, 0x26}, {0x8f, 0x27},
	                               {0x9d, 0x28}, {0xaa, 0x29},
	                               {0xb8, 0x2a}, {0xc4, 0x2b},
	                               {0xd1, 0x2c}, {0xdd, 0x2d},
	                               {0xe8, 0x2e}, {0xf4, 0x2f},
	                               {0xff, 0x30}, {0x00, 0x3f},
	                               {0xc7, 0x40}, {0x01, 0x41},
	                               {0x44, 0x42}, {0x00, 0x43},
	                               {0x44, 0x44}, {0x00, 0x45},
	                               {0x44, 0x46}, {0x00, 0x47},
	                               {0xc7, 0x48}, {0x01, 0x49},
	                               {0xc7, 0x4a}, {0x01, 0x4b},
	                               {0xc7, 0x4c}, {0x01, 0x4d},
	                               {0x44, 0x4e}, {0x00, 0x4f},
	                               {0x44, 0x50}, {0x00, 0x51},
	                               {0x44, 0x52}, {0x00, 0x53},
	                               {0xc7, 0x54}, {0x01, 0x55},
	                               {0xc7, 0x56}, {0x01, 0x57},
	                               {0xc7, 0x58}, {0x01, 0x59},
	                               {0x44, 0x5a}, {0x00, 0x5b},
	                               {0x44, 0x5c}, {0x00, 0x5d},
	                               {0x44, 0x5e}, {0x00, 0x5f},
	                               {0xc7, 0x60}, {0x01, 0x61},
	                               {0xc7, 0x62}, {0x01, 0x63},
	                               {0xc7, 0x64}, {0x01, 0x65},
	                               {0x44, 0x66}, {0x00, 0x67},
	                               {0x44, 0x68}, {0x00, 0x69},
	                               {0x44, 0x6a}, {0x00, 0x6b},
	                               {0xc7, 0x6c}, {0x01, 0x6d},
	                               {0xc7, 0x6e}, {0x01, 0x6f},
	                               {0xc7, 0x70}, {0x01, 0x71},
	                               {0x44, 0x72}, {0x00, 0x73},
	                               {0x44, 0x74}, {0x00, 0x75},
	                               {0x44, 0x76}, {0x00, 0x77},
	                               {0xc7, 0x78}, {0x01, 0x79},
	                               {0xc7, 0x7a}, {0x01, 0x7b},
	                               {0xc7, 0x7c}, {0x01, 0x7d},
	                               {0x44, 0x7e}, {0x00, 0x7f},
	                               {0x17, 0x84}, {0x00, 0x85},
	                               {0x2e, 0x86}, {0x00, 0x87},
	                               {0x09, 0x88}, {0x00, 0x89},
	                               {0xe8, 0x8a}, {0x0f, 0x8b},
	                               {0xda, 0x8c}, {0x0f, 0x8d},
	                               {0x40, 0x8e}, {0x00, 0x8f},
	                               {0x37, 0x90}, {0x00, 0x91},
	                               {0xcf, 0x92}, {0x0f, 0x93},
	                               {0xfa, 0x94}, {0x0f, 0x95},
	                               {0x00, 0x96}, {0x00, 0x97},
	                               {0x00, 0x98}, {0x66, 0x99},
	                               {0x00, 0x9a}, {0x40, 0x9b},
	                               {0x20, 0x9c}, {0x00, 0x9d},
	                               {0x00, 0x9e}, {0x00, 0x9f},
	                               {0x2d, 0xc0}, {0x2d, 0xc1},
	                               {0x3a, 0xc2}, {0x00, 0xc3},
	                               {0x04, 0xc4}, {0x3f, 0xc5},
	                               {0x00, 0xc6}, {0x00, 0xc7},
	                               {0x50, 0xc8}, {0x3c, 0xc9},
	                               {0x28, 0xca}, {0xd8, 0xcb},
	                               {0x14, 0xcc}, {0xec, 0xcd},
	                               {0x32, 0xce}, {0xdd, 0xcf},
	                               {0x32, 0xd0}, {0xdd, 0xd1},
	                               {0x6a, 0xd2}, {0x50, 0xd3},
	                               {0x60, 0xd4}, {0x00, 0xd5},
	                               {0x00, 0xd6});

	err += sn9c102_i2c_write(cam, 0x12, 0x80);
	err += sn9c102_i2c_write(cam, 0x12, 0x0c);
	err += sn9c102_i2c_write(cam, 0x19, 0x02);
	err += sn9c102_i2c_write(cam, 0x28, 0xa2);
	err += sn9c102_i2c_write(cam, 0x03, 0xa4);
	err += sn9c102_i2c_write(cam, 0x04, 0x30);
	err += sn9c102_i2c_write(cam, 0x05, 0x88);
	err += sn9c102_i2c_write(cam, 0x06, 0x60);
	err += sn9c102_i2c_write(cam, 0x24, 0xa0);
	err += sn9c102_i2c_write(cam, 0x25, 0x80);
	err += sn9c102_i2c_write(cam, 0x2a, 0x11);
	err += sn9c102_i2c_write(cam, 0x2d, 0x05);
	err += sn9c102_i2c_write(cam, 0x60, 0xa6);
	err += sn9c102_i2c_write(cam, 0x6d, 0x33);
	err += sn9c102_i2c_write(cam, 0x6e, 0x22);

	return err;
}


static int ov7648_get_ctrl(struct sn9c102_device* cam,
                           struct v4l2_control* ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x10)) < 0)
			return -EIO;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		if ((ctrl->value = sn9c102_read_reg(cam, 0x02)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x04) ? 1 : 0;
		break;
	case V4L2_CID_RED_BALANCE:
		ctrl->value = sn9c102_read_reg(cam, 0x05);
		break;
	case V4L2_CID_BLUE_BALANCE:
		ctrl->value = sn9c102_read_reg(cam, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		ctrl->value = sn9c102_read_reg(cam, 0x07);
		break;
	case V4L2_CID_GAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x00)) < 0)
			return -EIO;
		ctrl->value &= 0x3f;
		break;
	case V4L2_CID_AUTOGAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x13)) < 0)
			return -EIO;
		ctrl->value &= 0x01;
		break;
	case V4L2_CID_VFLIP:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x75)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x80) ? 1 : 0;
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x2d)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x02) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int ov7648_set_ctrl(struct sn9c102_device* cam,
                           const struct v4l2_control* ctrl)
{
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		err += sn9c102_i2c_write(cam, 0x10, ctrl->value);
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		err += sn9c102_write_reg(cam, 0x43 | (ctrl->value << 2), 0x02);
		break;
	case V4L2_CID_RED_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x05);
		break;
	case V4L2_CID_BLUE_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x07);
		break;
	case V4L2_CID_GAIN:
		err += sn9c102_i2c_write(cam, 0x00, ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		err += sn9c102_i2c_write(cam, 0x13, 0xa0 | ctrl->value |
		                                    (ctrl->value << 1));
		break;
	case V4L2_CID_VFLIP:
		err += sn9c102_i2c_write(cam, 0x75, 0x0e | (ctrl->value << 7));
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		err += sn9c102_i2c_write(cam, 0x2d, ctrl->value << 2);
		break;
	default:
		return -EINVAL;
	}

	return err ? -EIO : 0;
}


static int ov7648_set_crop(struct sn9c102_device* cam,
                           const struct v4l2_rect* rect)
{
	struct sn9c102_sensor* s = sn9c102_get_sensor(cam);
	int err = 0;
	u8 h_start = (u8)(rect->left - s->cropcap.bounds.left) + 2,
	   v_start = (u8)(rect->top - s->cropcap.bounds.top) + 1;

	err += sn9c102_write_reg(cam, h_start, 0x12);
	err += sn9c102_write_reg(cam, v_start, 0x13);

	return err;
}


static int ov7648_set_pix_format(struct sn9c102_device* cam,
                                 const struct v4l2_pix_format* pix)
{
	int err = 0;

	if (pix->pixelformat == V4L2_PIX_FMT_SBGGR8) {
		err += sn9c102_write_reg(cam, 0xe5, 0x17);
		err += sn9c102_i2c_write(cam, 0x11, 0x04);
	} else {
		err += sn9c102_write_reg(cam, 0xe2, 0x17);
		err += sn9c102_i2c_write(cam, 0x11, 0x01);
	}

	return err;
}


static const struct sn9c102_sensor ov7648 = {
	.name = "OV7648",
	.maintainer = "Luca Risolia <luca.risolia@studio.unibo.it>",
	.supported_bridge = BRIDGE_SN9C105 | BRIDGE_SN9C120,
	.sysfs_ops = SN9C102_I2C_READ | SN9C102_I2C_WRITE,
	.frequency = SN9C102_I2C_100KHZ,
	.interface = SN9C102_I2C_2WIRES,
	.i2c_slave_id = 0x21,
	.init = &ov7648_init,
	.qctrl = {
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "global gain",
			.minimum = 0x00,
			.maximum = 0x3f,
			.step = 0x01,
			.default_value = 0x14,
			.flags = 0,
		},
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x00,
			.maximum = 0xff,
			.step = 0x01,
			.default_value = 0x60,
			.flags = 0,
		},
		{
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "led",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		{
			.id = V4L2_CID_RED_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "red balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_BLUE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "blue balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_AUTOGAIN,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto gain-exposure",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x01,
			.flags = 0,
		},
		{
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "vertical flip",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_GREEN_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "green balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_BAND_FILTER,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "band filter",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
	},
	.get_ctrl = &ov7648_get_ctrl,
	.set_ctrl = &ov7648_set_ctrl,
	.cropcap = {
		.bounds = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
		.defrect = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
	},
	.set_crop = &ov7648_set_crop,
	.pix_format = {
		.width = 640,
		.height = 480,
		.pixelformat = V4L2_PIX_FMT_JPEG,
		.priv = 8,
	},
	.set_pix_format = &ov7648_set_pix_format
};


int sn9c102_probe_ov7648(struct sn9c102_device* cam)
{
	int pid, ver, err;

	err = sn9c102_write_const_regs(cam, {0x01, 0xf1}, {0x00, 0xf1},
	                               {0x29, 0x01}, {0x74, 0x02},
	                               {0x0e, 0x01}, {0x44, 0x01});

	pid = sn9c102_i2c_try_read(cam, &ov7648, 0x0a);
	ver = sn9c102_i2c_try_read(cam, &ov7648, 0x0b);
	if (err || pid < 0 || ver < 0)
		return -EIO;
	if (pid != 0x76 || ver != 0x48)
		return -ENODEV;

	sn9c102_attach_sensor(cam, &ov7648);

	return 0;
}


static int ov7660_init(struct sn9c102_device* cam)
{
	int err = 0;

	err = sn9c102_write_const_regs(cam, {0x40, 0x02}, {0x00, 0x03},
	                               {0x1a, 0x04}, {0x03, 0x10},
	                               {0x08, 0x14}, {0x20, 0x17},
	                               {0x8b, 0x18}, {0x00, 0x19},
	                               {0x1d, 0x1a}, {0x10, 0x1b},
	                               {0x02, 0x1c}, {0x03, 0x1d},
	                               {0x0f, 0x1e}, {0x0c, 0x1f},
	                               {0x00, 0x20}, {0x29, 0x21},
	                               {0x40, 0x22}, {0x54, 0x23},
	                               {0x66, 0x24}, {0x76, 0x25},
	                               {0x85, 0x26}, {0x94, 0x27},
	                               {0xa1, 0x28}, {0xae, 0x29},
	                               {0xbb, 0x2a}, {0xc7, 0x2b},
	                               {0xd3, 0x2c}, {0xde, 0x2d},
	                               {0xea, 0x2e}, {0xf4, 0x2f},
	                               {0xff, 0x30}, {0x00, 0x3f},
	                               {0xc7, 0x40}, {0x01, 0x41},
	                               {0x44, 0x42}, {0x00, 0x43},
	                               {0x44, 0x44}, {0x00, 0x45},
	                               {0x44, 0x46}, {0x00, 0x47},
	                               {0xc7, 0x48}, {0x01, 0x49},
	                               {0xc7, 0x4a}, {0x01, 0x4b},
	                               {0xc7, 0x4c}, {0x01, 0x4d},
	                               {0x44, 0x4e}, {0x00, 0x4f},
	                               {0x44, 0x50}, {0x00, 0x51},
	                               {0x44, 0x52}, {0x00, 0x53},
	                               {0xc7, 0x54}, {0x01, 0x55},
	                               {0xc7, 0x56}, {0x01, 0x57},
	                               {0xc7, 0x58}, {0x01, 0x59},
	                               {0x44, 0x5a}, {0x00, 0x5b},
	                               {0x44, 0x5c}, {0x00, 0x5d},
	                               {0x44, 0x5e}, {0x00, 0x5f},
	                               {0xc7, 0x60}, {0x01, 0x61},
	                               {0xc7, 0x62}, {0x01, 0x63},
	                               {0xc7, 0x64}, {0x01, 0x65},
	                               {0x44, 0x66}, {0x00, 0x67},
	                               {0x44, 0x68}, {0x00, 0x69},
	                               {0x44, 0x6a}, {0x00, 0x6b},
	                               {0xc7, 0x6c}, {0x01, 0x6d},
	                               {0xc7, 0x6e}, {0x01, 0x6f},
	                               {0xc7, 0x70}, {0x01, 0x71},
	                               {0x44, 0x72}, {0x00, 0x73},
	                               {0x44, 0x74}, {0x00, 0x75},
	                               {0x44, 0x76}, {0x00, 0x77},
	                               {0xc7, 0x78}, {0x01, 0x79},
	                               {0xc7, 0x7a}, {0x01, 0x7b},
	                               {0xc7, 0x7c}, {0x01, 0x7d},
	                               {0x44, 0x7e}, {0x00, 0x7f},
	                               {0x14, 0x84}, {0x00, 0x85},
	                               {0x27, 0x86}, {0x00, 0x87},
	                               {0x07, 0x88}, {0x00, 0x89},
	                               {0xec, 0x8a}, {0x0f, 0x8b},
	                               {0xd8, 0x8c}, {0x0f, 0x8d},
	                               {0x3d, 0x8e}, {0x00, 0x8f},
	                               {0x3d, 0x90}, {0x00, 0x91},
	                               {0xcd, 0x92}, {0x0f, 0x93},
	                               {0xf7, 0x94}, {0x0f, 0x95},
	                               {0x0c, 0x96}, {0x00, 0x97},
	                               {0x00, 0x98}, {0x66, 0x99},
	                               {0x05, 0x9a}, {0x00, 0x9b},
	                               {0x04, 0x9c}, {0x00, 0x9d},
	                               {0x08, 0x9e}, {0x00, 0x9f},
	                               {0x2d, 0xc0}, {0x2d, 0xc1},
	                               {0x3a, 0xc2}, {0x05, 0xc3},
	                               {0x04, 0xc4}, {0x3f, 0xc5},
	                               {0x00, 0xc6}, {0x00, 0xc7},
	                               {0x50, 0xc8}, {0x3C, 0xc9},
	                               {0x28, 0xca}, {0xd8, 0xcb},
	                               {0x14, 0xcc}, {0xec, 0xcd},
	                               {0x32, 0xce}, {0xdd, 0xcf},
	                               {0x32, 0xd0}, {0xdd, 0xd1},
	                               {0x6a, 0xd2}, {0x50, 0xd3},
	                               {0x00, 0xd4}, {0x00, 0xd5},
	                               {0x00, 0xd6});

	err += sn9c102_i2c_write(cam, 0x12, 0x80);
	err += sn9c102_i2c_write(cam, 0x11, 0x09);
	err += sn9c102_i2c_write(cam, 0x00, 0x0A);
	err += sn9c102_i2c_write(cam, 0x01, 0x80);
	err += sn9c102_i2c_write(cam, 0x02, 0x80);
	err += sn9c102_i2c_write(cam, 0x03, 0x00);
	err += sn9c102_i2c_write(cam, 0x04, 0x00);
	err += sn9c102_i2c_write(cam, 0x05, 0x08);
	err += sn9c102_i2c_write(cam, 0x06, 0x0B);
	err += sn9c102_i2c_write(cam, 0x07, 0x00);
	err += sn9c102_i2c_write(cam, 0x08, 0x1C);
	err += sn9c102_i2c_write(cam, 0x09, 0x01);
	err += sn9c102_i2c_write(cam, 0x0A, 0x76);
	err += sn9c102_i2c_write(cam, 0x0B, 0x60);
	err += sn9c102_i2c_write(cam, 0x0C, 0x00);
	err += sn9c102_i2c_write(cam, 0x0D, 0x08);
	err += sn9c102_i2c_write(cam, 0x0E, 0x04);
	err += sn9c102_i2c_write(cam, 0x0F, 0x6F);
	err += sn9c102_i2c_write(cam, 0x10, 0x20);
	err += sn9c102_i2c_write(cam, 0x11, 0x03);
	err += sn9c102_i2c_write(cam, 0x12, 0x05);
	err += sn9c102_i2c_write(cam, 0x13, 0xC7);
	err += sn9c102_i2c_write(cam, 0x14, 0x2C);
	err += sn9c102_i2c_write(cam, 0x15, 0x00);
	err += sn9c102_i2c_write(cam, 0x16, 0x02);
	err += sn9c102_i2c_write(cam, 0x17, 0x10);
	err += sn9c102_i2c_write(cam, 0x18, 0x60);
	err += sn9c102_i2c_write(cam, 0x19, 0x02);
	err += sn9c102_i2c_write(cam, 0x1A, 0x7B);
	err += sn9c102_i2c_write(cam, 0x1B, 0x02);
	err += sn9c102_i2c_write(cam, 0x1C, 0x7F);
	err += sn9c102_i2c_write(cam, 0x1D, 0xA2);
	err += sn9c102_i2c_write(cam, 0x1E, 0x01);
	err += sn9c102_i2c_write(cam, 0x1F, 0x0E);
	err += sn9c102_i2c_write(cam, 0x20, 0x05);
	err += sn9c102_i2c_write(cam, 0x21, 0x05);
	err += sn9c102_i2c_write(cam, 0x22, 0x05);
	err += sn9c102_i2c_write(cam, 0x23, 0x05);
	err += sn9c102_i2c_write(cam, 0x24, 0x68);
	err += sn9c102_i2c_write(cam, 0x25, 0x58);
	err += sn9c102_i2c_write(cam, 0x26, 0xD4);
	err += sn9c102_i2c_write(cam, 0x27, 0x80);
	err += sn9c102_i2c_write(cam, 0x28, 0x80);
	err += sn9c102_i2c_write(cam, 0x29, 0x30);
	err += sn9c102_i2c_write(cam, 0x2A, 0x00);
	err += sn9c102_i2c_write(cam, 0x2B, 0x00);
	err += sn9c102_i2c_write(cam, 0x2C, 0x80);
	err += sn9c102_i2c_write(cam, 0x2D, 0x00);
	err += sn9c102_i2c_write(cam, 0x2E, 0x00);
	err += sn9c102_i2c_write(cam, 0x2F, 0x0E);
	err += sn9c102_i2c_write(cam, 0x30, 0x08);
	err += sn9c102_i2c_write(cam, 0x31, 0x30);
	err += sn9c102_i2c_write(cam, 0x32, 0xB4);
	err += sn9c102_i2c_write(cam, 0x33, 0x00);
	err += sn9c102_i2c_write(cam, 0x34, 0x07);
	err += sn9c102_i2c_write(cam, 0x35, 0x84);
	err += sn9c102_i2c_write(cam, 0x36, 0x00);
	err += sn9c102_i2c_write(cam, 0x37, 0x0C);
	err += sn9c102_i2c_write(cam, 0x38, 0x02);
	err += sn9c102_i2c_write(cam, 0x39, 0x43);
	err += sn9c102_i2c_write(cam, 0x3A, 0x00);
	err += sn9c102_i2c_write(cam, 0x3B, 0x0A);
	err += sn9c102_i2c_write(cam, 0x3C, 0x6C);
	err += sn9c102_i2c_write(cam, 0x3D, 0x99);
	err += sn9c102_i2c_write(cam, 0x3E, 0x0E);
	err += sn9c102_i2c_write(cam, 0x3F, 0x41);
	err += sn9c102_i2c_write(cam, 0x40, 0xC1);
	err += sn9c102_i2c_write(cam, 0x41, 0x22);
	err += sn9c102_i2c_write(cam, 0x42, 0x08);
	err += sn9c102_i2c_write(cam, 0x43, 0xF0);
	err += sn9c102_i2c_write(cam, 0x44, 0x10);
	err += sn9c102_i2c_write(cam, 0x45, 0x78);
	err += sn9c102_i2c_write(cam, 0x46, 0xA8);
	err += sn9c102_i2c_write(cam, 0x47, 0x60);
	err += sn9c102_i2c_write(cam, 0x48, 0x80);
	err += sn9c102_i2c_write(cam, 0x49, 0x00);
	err += sn9c102_i2c_write(cam, 0x4A, 0x00);
	err += sn9c102_i2c_write(cam, 0x4B, 0x00);
	err += sn9c102_i2c_write(cam, 0x4C, 0x00);
	err += sn9c102_i2c_write(cam, 0x4D, 0x00);
	err += sn9c102_i2c_write(cam, 0x4E, 0x00);
	err += sn9c102_i2c_write(cam, 0x4F, 0x46);
	err += sn9c102_i2c_write(cam, 0x50, 0x36);
	err += sn9c102_i2c_write(cam, 0x51, 0x0F);
	err += sn9c102_i2c_write(cam, 0x52, 0x17);
	err += sn9c102_i2c_write(cam, 0x53, 0x7F);
	err += sn9c102_i2c_write(cam, 0x54, 0x96);
	err += sn9c102_i2c_write(cam, 0x55, 0x40);
	err += sn9c102_i2c_write(cam, 0x56, 0x40);
	err += sn9c102_i2c_write(cam, 0x57, 0x40);
	err += sn9c102_i2c_write(cam, 0x58, 0x0F);
	err += sn9c102_i2c_write(cam, 0x59, 0xBA);
	err += sn9c102_i2c_write(cam, 0x5A, 0x9A);
	err += sn9c102_i2c_write(cam, 0x5B, 0x22);
	err += sn9c102_i2c_write(cam, 0x5C, 0xB9);
	err += sn9c102_i2c_write(cam, 0x5D, 0x9B);
	err += sn9c102_i2c_write(cam, 0x5E, 0x10);
	err += sn9c102_i2c_write(cam, 0x5F, 0xF0);
	err += sn9c102_i2c_write(cam, 0x60, 0x05);
	err += sn9c102_i2c_write(cam, 0x61, 0x60);
	err += sn9c102_i2c_write(cam, 0x62, 0x00);
	err += sn9c102_i2c_write(cam, 0x63, 0x00);
	err += sn9c102_i2c_write(cam, 0x64, 0x50);
	err += sn9c102_i2c_write(cam, 0x65, 0x30);
	err += sn9c102_i2c_write(cam, 0x66, 0x00);
	err += sn9c102_i2c_write(cam, 0x67, 0x80);
	err += sn9c102_i2c_write(cam, 0x68, 0x7A);
	err += sn9c102_i2c_write(cam, 0x69, 0x90);
	err += sn9c102_i2c_write(cam, 0x6A, 0x80);
	err += sn9c102_i2c_write(cam, 0x6B, 0x0A);
	err += sn9c102_i2c_write(cam, 0x6C, 0x30);
	err += sn9c102_i2c_write(cam, 0x6D, 0x48);
	err += sn9c102_i2c_write(cam, 0x6E, 0x80);
	err += sn9c102_i2c_write(cam, 0x6F, 0x74);
	err += sn9c102_i2c_write(cam, 0x70, 0x64);
	err += sn9c102_i2c_write(cam, 0x71, 0x60);
	err += sn9c102_i2c_write(cam, 0x72, 0x5C);
	err += sn9c102_i2c_write(cam, 0x73, 0x58);
	err += sn9c102_i2c_write(cam, 0x74, 0x54);
	err += sn9c102_i2c_write(cam, 0x75, 0x4C);
	err += sn9c102_i2c_write(cam, 0x76, 0x40);
	err += sn9c102_i2c_write(cam, 0x77, 0x38);
	err += sn9c102_i2c_write(cam, 0x78, 0x34);
	err += sn9c102_i2c_write(cam, 0x79, 0x30);
	err += sn9c102_i2c_write(cam, 0x7A, 0x2F);
	err += sn9c102_i2c_write(cam, 0x7B, 0x2B);
	err += sn9c102_i2c_write(cam, 0x7C, 0x03);
	err += sn9c102_i2c_write(cam, 0x7D, 0x07);
	err += sn9c102_i2c_write(cam, 0x7E, 0x17);
	err += sn9c102_i2c_write(cam, 0x7F, 0x34);
	err += sn9c102_i2c_write(cam, 0x80, 0x41);
	err += sn9c102_i2c_write(cam, 0x81, 0x4D);
	err += sn9c102_i2c_write(cam, 0x82, 0x58);
	err += sn9c102_i2c_write(cam, 0x83, 0x63);
	err += sn9c102_i2c_write(cam, 0x84, 0x6E);
	err += sn9c102_i2c_write(cam, 0x85, 0x77);
	err += sn9c102_i2c_write(cam, 0x86, 0x87);
	err += sn9c102_i2c_write(cam, 0x87, 0x95);
	err += sn9c102_i2c_write(cam, 0x88, 0xAF);
	err += sn9c102_i2c_write(cam, 0x89, 0xC7);
	err += sn9c102_i2c_write(cam, 0x8A, 0xDF);
	err += sn9c102_i2c_write(cam, 0x8B, 0x99);
	err += sn9c102_i2c_write(cam, 0x8C, 0x99);
	err += sn9c102_i2c_write(cam, 0x8D, 0xCF);
	err += sn9c102_i2c_write(cam, 0x8E, 0x20);
	err += sn9c102_i2c_write(cam, 0x8F, 0x26);
	err += sn9c102_i2c_write(cam, 0x90, 0x10);
	err += sn9c102_i2c_write(cam, 0x91, 0x0C);
	err += sn9c102_i2c_write(cam, 0x92, 0x25);
	err += sn9c102_i2c_write(cam, 0x93, 0x00);
	err += sn9c102_i2c_write(cam, 0x94, 0x50);
	err += sn9c102_i2c_write(cam, 0x95, 0x50);
	err += sn9c102_i2c_write(cam, 0x96, 0x00);
	err += sn9c102_i2c_write(cam, 0x97, 0x01);
	err += sn9c102_i2c_write(cam, 0x98, 0x10);
	err += sn9c102_i2c_write(cam, 0x99, 0x40);
	err += sn9c102_i2c_write(cam, 0x9A, 0x40);
	err += sn9c102_i2c_write(cam, 0x9B, 0x20);
	err += sn9c102_i2c_write(cam, 0x9C, 0x00);
	err += sn9c102_i2c_write(cam, 0x9D, 0x99);
	err += sn9c102_i2c_write(cam, 0x9E, 0x7F);
	err += sn9c102_i2c_write(cam, 0x9F, 0x00);
	err += sn9c102_i2c_write(cam, 0xA0, 0x00);
	err += sn9c102_i2c_write(cam, 0xA1, 0x00);

	return err;
}


static int ov7660_get_ctrl(struct sn9c102_device* cam,
                           struct v4l2_control* ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x10)) < 0)
			return -EIO;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		if ((ctrl->value = sn9c102_read_reg(cam, 0x02)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x04) ? 1 : 0;
		break;
	case V4L2_CID_RED_BALANCE:
		if ((ctrl->value = sn9c102_read_reg(cam, 0x05)) < 0)
			return -EIO;
		ctrl->value &= 0x7f;
		break;
	case V4L2_CID_BLUE_BALANCE:
		if ((ctrl->value = sn9c102_read_reg(cam, 0x06)) < 0)
			return -EIO;
		ctrl->value &= 0x7f;
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		if ((ctrl->value = sn9c102_read_reg(cam, 0x07)) < 0)
			return -EIO;
		ctrl->value &= 0x7f;
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x3b)) < 0)
			return -EIO;
		ctrl->value &= 0x08;
		break;
	case V4L2_CID_GAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x00)) < 0)
			return -EIO;
		ctrl->value &= 0x1f;
		break;
	case V4L2_CID_AUTOGAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x13)) < 0)
			return -EIO;
		ctrl->value &= 0x01;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int ov7660_set_ctrl(struct sn9c102_device* cam,
                           const struct v4l2_control* ctrl)
{
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		err += sn9c102_i2c_write(cam, 0x10, ctrl->value);
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		err += sn9c102_write_reg(cam, 0x43 | (ctrl->value << 2), 0x02);
		break;
	case V4L2_CID_RED_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x05);
		break;
	case V4L2_CID_BLUE_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x07);
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		err += sn9c102_i2c_write(cam, ctrl->value << 3, 0x3b);
		break;
	case V4L2_CID_GAIN:
		err += sn9c102_i2c_write(cam, 0x00, 0x60 + ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		err += sn9c102_i2c_write(cam, 0x13, 0xc0 |
		                                    (ctrl->value * 0x07));
		break;
	default:
		return -EINVAL;
	}

	return err ? -EIO : 0;
}


static int ov7660_set_crop(struct sn9c102_device* cam,
                           const struct v4l2_rect* rect)
{
	struct sn9c102_sensor* s = sn9c102_get_sensor(cam);
	int err = 0;
	u8 h_start = (u8)(rect->left - s->cropcap.bounds.left) + 1,
	   v_start = (u8)(rect->top - s->cropcap.bounds.top) + 1;

	err += sn9c102_write_reg(cam, h_start, 0x12);
	err += sn9c102_write_reg(cam, v_start, 0x13);

	return err;
}


static int ov7660_set_pix_format(struct sn9c102_device* cam,
                                 const struct v4l2_pix_format* pix)
{
	int r0, err = 0;

	r0 = sn9c102_pread_reg(cam, 0x01);

	if (pix->pixelformat == V4L2_PIX_FMT_JPEG) {
		err += sn9c102_write_reg(cam, r0 | 0x40, 0x01);
		err += sn9c102_write_reg(cam, 0xa2, 0x17);
		err += sn9c102_i2c_write(cam, 0x11, 0x00);
	} else {
		err += sn9c102_write_reg(cam, r0 | 0x40, 0x01);
		err += sn9c102_write_reg(cam, 0xa2, 0x17);
		err += sn9c102_i2c_write(cam, 0x11, 0x0d);
	}

	return err;
}


static const struct sn9c102_sensor ov7660 = {
	.name = "OV7660",
	.maintainer = "Luca Risolia <luca.risolia@studio.unibo.it>",
	.supported_bridge = BRIDGE_SN9C105 | BRIDGE_SN9C120,
	.sysfs_ops = SN9C102_I2C_READ | SN9C102_I2C_WRITE,
	.frequency = SN9C102_I2C_100KHZ,
	.interface = SN9C102_I2C_2WIRES,
	.i2c_slave_id = 0x21,
	.init = &ov7660_init,
	.qctrl = {
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "global gain",
			.minimum = 0x00,
			.maximum = 0x1f,
			.step = 0x01,
			.default_value = 0x09,
			.flags = 0,
		},
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x00,
			.maximum = 0xff,
			.step = 0x01,
			.default_value = 0x27,
			.flags = 0,
		},
		{
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "led",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		{
			.id = V4L2_CID_RED_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "red balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x14,
			.flags = 0,
		},
		{
			.id = V4L2_CID_BLUE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "blue balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x14,
			.flags = 0,
		},
		{
			.id = V4L2_CID_AUTOGAIN,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto gain-exposure",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x01,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_GREEN_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "green balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x14,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_BAND_FILTER,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "band filter",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
	},
	.get_ctrl = &ov7660_get_ctrl,
	.set_ctrl = &ov7660_set_ctrl,
	.cropcap = {
		.bounds = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
		.defrect = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
	},
	.set_crop = &ov7660_set_crop,
	.pix_format = {
		.width = 640,
		.height = 480,
		.pixelformat = V4L2_PIX_FMT_JPEG,
		.priv = 8,
	},
	.set_pix_format = &ov7660_set_pix_format
};


int sn9c102_probe_ov7660(struct sn9c102_device* cam)
{
	int pid, ver, err;

	err = sn9c102_write_const_regs(cam, {0x01, 0xf1}, {0x00, 0xf1},
	                               {0x01, 0x01}, {0x00, 0x01},
	                               {0x28, 0x17});

	pid = sn9c102_i2c_try_read(cam, &ov7660, 0x0a);
	ver = sn9c102_i2c_try_read(cam, &ov7660, 0x0b);
	if (err || pid < 0 || ver < 0)
		return -EIO;
	if (pid != 0x76 || ver != 0x60)
		return -ENODEV;

	sn9c102_attach_sensor(cam, &ov7660);

	return 0;
}




/*
   Add the above entries to this table. Be sure to add the entry in the right
   place, since, on failure, the next probing routine is called according to
   the order of the list below, from top to bottom.
*/
static int (*sn9c102_sensor_table[])(struct sn9c102_device*) = {
	&sn9c102_probe_ov7630, /* strong detection based on SENSOR ids */
	&sn9c102_probe_ov7648, /* strong detection based on SENSOR ids */
	&sn9c102_probe_ov7660, /* strong detection based on SENSOR ids */
};

/*****************************************************************************/













#define SN9C102_MODULE_NAME     "V4L2 driver for SN9C1xx PC Camera Controllers"
#define SN9C102_MODULE_ALIAS    "sn9c1xx"
#define SN9C102_MODULE_AUTHOR   "(C) 2004-2008 Luca Risolia"
#define SN9C102_AUTHOR_EMAIL    "<luca.risolia@studio.unibo.it>"
#define SN9C102_MODULE_LICENSE  "GPL"
#define SN9C102_MODULE_VERSION  "1:1.50"
#define SN9C102_MODULE_VERSION_CODE  KERNEL_VERSION(1, 1, 50)

/*****************************************************************************/

MODULE_DEVICE_TABLE(usb, sn9c102_id_table);

MODULE_AUTHOR(SN9C102_MODULE_AUTHOR " " SN9C102_AUTHOR_EMAIL);
MODULE_DESCRIPTION(SN9C102_MODULE_NAME);
MODULE_ALIAS(SN9C102_MODULE_ALIAS);
MODULE_VERSION(SN9C102_MODULE_VERSION);
MODULE_LICENSE(SN9C102_MODULE_LICENSE);

static short video_nr[] = {[0 ... SN9C102_MAX_DEVICES-1] = -1};
module_param_array(video_nr, short, NULL, 0644);
MODULE_PARM_DESC(video_nr,
                 " <-1|n[,...]>"
                 "\nSpecify V4L2 minor mode number."
                 "\n-1 = use next available (default)"
                 "\n n = use minor number n (integer >= 0)"
                 "\nYou can specify up to "__MODULE_STRING(SN9C102_MAX_DEVICES)
                 " cameras this way."
                 "\nFor example:"
                 "\nvideo_nr=-1,2,-1 would assign minor number 2 to"
                 "\nthe second camera and use auto for the first"
                 "\none and for every other camera."
                 "\n");

static /*short - josemar*/unsigned max_opens[] = {[0 ... SN9C102_MAX_DEVICES-1] = SN9C102_MAX_OPENS};
module_param_array(max_opens, uint, NULL, 0644);
MODULE_PARM_DESC(max_opens,
                 " <0|n[,...]>"
                 "\nMaximum number of allowed device opens at a given time."
                 "\n0 for unlimited opens"
                 "\nDefault value is "__MODULE_STRING(SN9C102_MAX_OPENS)"."
                 "\n");

static /*short - josemar*/bool force_munmap[] = {[0 ... SN9C102_MAX_DEVICES-1] =
                               SN9C102_FORCE_MUNMAP};
module_param_array(force_munmap, bool, NULL, 0644);
MODULE_PARM_DESC(force_munmap,
                 " <0|1[,...]>"
                 "\nForce the application to unmap previously"
                 "\nmapped buffer memory before calling any VIDIOC_S_CROP or"
                 "\nVIDIOC_S_FMT ioctl's. Not all the applications support"
                 "\nthis feature. This parameter is specific for each"
                 "\ndetected camera."
                 "\n0 = do not force memory unmapping"
                 "\n1 = force memory unmapping (save memory)"
                 "\nDefault value is "__MODULE_STRING(SN9C102_FORCE_MUNMAP)"."
                 "\n");

static unsigned int frame_timeout[] = {[0 ... SN9C102_MAX_DEVICES-1] =
                                       SN9C102_FRAME_TIMEOUT};
module_param_array(frame_timeout, uint, NULL, 0644);
MODULE_PARM_DESC(frame_timeout,
                 " <0|n[,...]>"
                 "\nTimeout for a video frame in seconds before"
                 "\nreturning an I/O error; 0 for infinity."
                 "\nThis parameter is specific for each detected camera."
                 "\nDefault value is "__MODULE_STRING(SN9C102_FRAME_TIMEOUT)"."
                 "\n");

#ifdef SN9C102_DEBUG
static unsigned short debug = SN9C102_DEBUG_LEVEL;
module_param(debug, ushort, 0644);
MODULE_PARM_DESC(debug,
                 " <n>"
                 "\nDebugging information level, from 0 to 3:"
                 "\n0 = none (use carefully)"
                 "\n1 = critical errors"
                 "\n2 = significant informations"
                 "\n3 = more verbose messages"
                 "\nLevel 3 is useful for testing only."
                 "\nDefault value is "__MODULE_STRING(SN9C102_DEBUG_LEVEL)"."
                 "\n");
#endif

/*****************************************************************************/

static u32
sn9c102_request_buffers(struct sn9c102_device* cam, u32 count,
                        enum sn9c102_io_method io)
{
	struct v4l2_pix_format* p = &(cam->sensor.pix_format);
	struct v4l2_rect* r = &(cam->sensor.cropcap.bounds);
	size_t imagesize = cam->module_param.force_munmap || io == IO_READ ?
	                   (p->width * p->height * p->priv) / 8 :
	                   (r->width * r->height * p->priv) / 8;
	void* buff = NULL;
	u32 i;

	if (count > SN9C102_MAX_FRAMES)
		count = SN9C102_MAX_FRAMES;

	if (cam->bridge == BRIDGE_SN9C105 || cam->bridge == BRIDGE_SN9C120)
		imagesize += 589; /* length of JPEG header */

	cam->nbuffers = count;
	while (cam->nbuffers > 0) {
		if ((buff = vmalloc_32_user(cam->nbuffers *
		                            PAGE_ALIGN(imagesize))))
			break;
		cam->nbuffers--;
	}

	for (i = 0; i < cam->nbuffers; i++) {
		cam->frame[i].bufmem = buff + i*PAGE_ALIGN(imagesize);
		cam->frame[i].buf.index = i;
		cam->frame[i].buf.m.offset = i*PAGE_ALIGN(imagesize);
		cam->frame[i].buf.length = imagesize;
		cam->frame[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam->frame[i].buf.sequence = 0;
		cam->frame[i].buf.field = V4L2_FIELD_NONE;
		cam->frame[i].buf.memory = V4L2_MEMORY_MMAP;
		cam->frame[i].buf.flags = 0;
	}

	return cam->nbuffers;
}


static void sn9c102_release_buffers(struct sn9c102_device* cam)
{
	if (cam->nbuffers) {
		vfree(cam->frame[0].bufmem);
		cam->nbuffers = 0;
	}
	cam->frame_current = NULL;
}


static void sn9c102_empty_framequeues(struct sn9c102_device* cam)
{
	u32 i;

	INIT_LIST_HEAD(&cam->inqueue);
	INIT_LIST_HEAD(&cam->outqueue);

	for (i = 0; i < SN9C102_MAX_FRAMES; i++) {
		cam->frame[i].state = F_UNUSED;
		cam->frame[i].buf.bytesused = 0;
	}
}


static void sn9c102_requeue_outqueue(struct sn9c102_device* cam)
{
	struct sn9c102_frame_t *i;

	list_for_each_entry(i, &cam->outqueue, frame) {
		i->state = F_QUEUED;
		list_add(&i->frame, &cam->inqueue);
	}

	INIT_LIST_HEAD(&cam->outqueue);
}


static void sn9c102_queue_unusedframes(struct sn9c102_device* cam)
{
	unsigned long lock_flags;
	u32 i;

	for (i = 0; i < cam->nbuffers; i++)
		if (cam->frame[i].state == F_UNUSED) {
			cam->frame[i].state = F_QUEUED;
			spin_lock_irqsave(&cam->queue_lock, lock_flags);
			list_add_tail(&cam->frame[i].frame, &cam->inqueue);
			spin_unlock_irqrestore(&cam->queue_lock, lock_flags);
		}
}

/*****************************************************************************/

/*
   Write a sequence of count value/register pairs. Returns -1 after the first
   failed write, or 0 for no errors.
*/
int sn9c102_write_regs(struct sn9c102_device* cam, const u8 valreg[][2],
                       int count)
{
	struct usb_device* udev = cam->usbdev;
	u8* buff = cam->control_buffer;
	int i, res;

	for (i = 0; i < count; i++) {
		u8 index = valreg[i][1];

		/*
		   index is a u8, so it must be <256 and can't be out of range.
		   If we put in a check anyway, gcc annoys us with a warning
		   hat our check is useless. People get all uppity when they
		   see warnings in the kernel compile.
		*/

		*buff = valreg[i][0];

		res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x08,
		                      0x41, index, 0, buff, 1,
		                      SN9C102_CTRL_TIMEOUT);

		if (res < 0) {
			DBG(3, "Failed to write a register (value 0x%02X, "
			       "index 0x%02X, error %d)", *buff, index, res);
			return -1;
		}

		cam->reg[index] = *buff;
	}

	return 0;
}


int sn9c102_write_reg(struct sn9c102_device* cam, u8 value, u16 index)
{
	struct usb_device* udev = cam->usbdev;
	u8* buff = cam->control_buffer;
	int res;

	if (index >= ARRAY_SIZE(cam->reg))
		return -1;

	*buff = value;

	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x08, 0x41,
	                      index, 0, buff, 1, SN9C102_CTRL_TIMEOUT);
	if (res < 0) {
		DBG(3, "Failed to write a register (value 0x%02X, index "
		       "0x%02X, error %d)", value, index, res);
		return -1;
	}

	cam->reg[index] = value;

	return 0;
}


/* NOTE: with the SN9C10[123] reading some registers always returns 0 */
int sn9c102_read_reg(struct sn9c102_device* cam, u16 index)
{
	struct usb_device* udev = cam->usbdev;
	u8* buff = cam->control_buffer;
	int res;

	res = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x00, 0xc1, index, 0, buff, 1, SN9C102_CTRL_TIMEOUT);
	if (res < 0)
		DBG(3, "Failed to read a register (index 0x%02X, error %d)",
		    index, res);

	return (res >= 0) ? (int)(*buff) : -1;
}


int sn9c102_pread_reg(struct sn9c102_device* cam, u16 index)
{
	if (index >= ARRAY_SIZE(cam->reg))
		return -1;

	return cam->reg[index];
}


static int
sn9c102_i2c_wait(struct sn9c102_device* cam,
                 const struct sn9c102_sensor* sensor)
{
	int i, r;

	for (i = 1; i <= 5; i++) {
		r = sn9c102_read_reg(cam, 0x08);
		if (r < 0)
			return -EIO;
		if (r & 0x04)
			return 0;
		if (sensor->frequency & SN9C102_I2C_400KHZ)
			udelay(5*16);
		else
			udelay(16*16);
	}
	return -EBUSY;
}


static int
sn9c102_i2c_detect_read_error(struct sn9c102_device* cam,
                              const struct sn9c102_sensor* sensor)
{
	int r , err = 0;

	r = sn9c102_read_reg(cam, 0x08);
	if (r < 0)
		err += r;

	if (cam->bridge == BRIDGE_SN9C101 || cam->bridge == BRIDGE_SN9C102) {
		if (!(r & 0x08))
			err += -1;
	} else {
		if (r & 0x08)
			err += -1;
	}

	return err ? -EIO : 0;
}


static int
sn9c102_i2c_detect_write_error(struct sn9c102_device* cam,
                               const struct sn9c102_sensor* sensor)
{
	int r;
	r = sn9c102_read_reg(cam, 0x08);
	return (r < 0 || (r >= 0 && (r & 0x08))) ? -EIO : 0;
}


int
sn9c102_i2c_try_raw_read(struct sn9c102_device* cam,
                         const struct sn9c102_sensor* sensor, u8 data0,
                         u8 data1, u8 n, u8 buffer[])
{
	struct usb_device* udev = cam->usbdev;
	u8* data = cam->control_buffer;
	int i = 0, err = 0, res;

	/* Write cycle */
	data[0] = ((sensor->interface == SN9C102_I2C_2WIRES) ? 0x80 : 0) |
	          ((sensor->frequency & SN9C102_I2C_400KHZ) ? 0x01 : 0) | 0x10;
	data[1] = data0; /* I2C slave id */
	data[2] = data1; /* address */
	data[7] = 0x10;
	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x08, 0x41,
	                      0x08, 0, data, 8, SN9C102_CTRL_TIMEOUT);
	if (res < 0)
		err += res;

	err += sn9c102_i2c_wait(cam, sensor);

	/* Read cycle - n bytes */
	data[0] = ((sensor->interface == SN9C102_I2C_2WIRES) ? 0x80 : 0) |
	          ((sensor->frequency & SN9C102_I2C_400KHZ) ? 0x01 : 0) |
	          (n << 4) | 0x02;
	data[1] = data0;
	data[7] = 0x10;
	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x08, 0x41,
	                      0x08, 0, data, 8, SN9C102_CTRL_TIMEOUT);
	if (res < 0)
		err += res;

	err += sn9c102_i2c_wait(cam, sensor);

	/* The first read byte will be placed in data[4] */
	res = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x00, 0xc1,
	                      0x0a, 0, data, 5, SN9C102_CTRL_TIMEOUT);
	if (res < 0)
		err += res;

	err += sn9c102_i2c_detect_read_error(cam, sensor);

	PDBGG("I2C read: address 0x%02X, first read byte: 0x%02X", data1,
	      data[4]);

	if (err) {
		DBG(3, "I2C read failed for %s image sensor", sensor->name);
		return -1;
	}

	if (buffer)
		for (i = 0; i < n && i < 5; i++)
			buffer[n-i-1] = data[4-i];

	return (int)data[4];
}


int
sn9c102_i2c_try_raw_write(struct sn9c102_device* cam,
                          const struct sn9c102_sensor* sensor, u8 n, u8 data0,
                          u8 data1, u8 data2, u8 data3, u8 data4, u8 data5)
{
	struct usb_device* udev = cam->usbdev;
	u8* data = cam->control_buffer;
	int err = 0, res;

	/* Write cycle. It usually is address + value */
	data[0] = ((sensor->interface == SN9C102_I2C_2WIRES) ? 0x80 : 0) |
	          ((sensor->frequency & SN9C102_I2C_400KHZ) ? 0x01 : 0)
	          | ((n - 1) << 4);
	data[1] = data0;
	data[2] = data1;
	data[3] = data2;
	data[4] = data3;
	data[5] = data4;
	data[6] = data5;
	data[7] = 0x17;
	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x08, 0x41,
	                      0x08, 0, data, 8, SN9C102_CTRL_TIMEOUT);
	if (res < 0)
		err += res;

	err += sn9c102_i2c_wait(cam, sensor);
	err += sn9c102_i2c_detect_write_error(cam, sensor);

	if (err)
		DBG(3, "I2C write failed for %s image sensor", sensor->name);

	PDBGG("I2C raw write: %u bytes, data0 = 0x%02X, data1 = 0x%02X, "
	      "data2 = 0x%02X, data3 = 0x%02X, data4 = 0x%02X, data5 = 0x%02X",
	      n, data0, data1, data2, data3, data4, data5);

	return err ? -1 : 0;
}


int
sn9c102_i2c_try_read(struct sn9c102_device* cam,
                     const struct sn9c102_sensor* sensor, u8 address)
{
	return sn9c102_i2c_try_raw_read(cam, sensor, sensor->i2c_slave_id,
	                                address, 1, NULL);
}


int
sn9c102_i2c_try_write(struct sn9c102_device* cam,
                      const struct sn9c102_sensor* sensor, u8 address, u8 value)
{
	return sn9c102_i2c_try_raw_write(cam, sensor, 3,
	                                 sensor->i2c_slave_id, address,
	                                 value, 0, 0, 0);
}


int sn9c102_i2c_read(struct sn9c102_device* cam, u8 address)
{
	return sn9c102_i2c_try_read(cam, &cam->sensor, address);
}


int sn9c102_i2c_write(struct sn9c102_device* cam, u8 address, u8 value)
{
	return sn9c102_i2c_try_write(cam, &cam->sensor, address, value);
}


static int sn9c102_reset_imagepipe(struct sn9c102_device* cam)
{
	int err = 0, r;

	r = sn9c102_read_reg(cam, 0x01);
	err += sn9c102_write_reg(cam, r & 0xfb, 0x01);
	err += sn9c102_write_reg(cam, r | 0x04, 0x01);

	return (err || r < 0) ? -EIO : 0;
}

/*****************************************************************************/

static size_t sn9c102_sof_length(struct sn9c102_device* cam)
{
	switch (cam->bridge) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
		return 12;
	case BRIDGE_SN9C103:
		return 18;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		return 62;
	}

	return 0;
}


static void*
sn9c102_find_sof_header(struct sn9c102_device* cam, void* mem, size_t len)
{
	static const char marker[6] = {0xff, 0xff, 0x00, 0xc4, 0xc4, 0x96};
	const char *m = mem;
	size_t soflen = 0, i, j;

	soflen = sn9c102_sof_length(cam);

 	for (i = 0; i < len; i++) {
		size_t b;

		/* Read the variable part of the header */
		if (unlikely(cam->sof.bytesread >= sizeof(marker))) {
			cam->sof.header[cam->sof.bytesread] = *(m+i);
			if (++cam->sof.bytesread == soflen) {
				cam->sof.bytesread = 0;
				return mem + i;
			}
			continue;
		}

		/* Search for the SOF marker (fixed part) in the header */
		for (j = 0, b=cam->sof.bytesread; j+b < sizeof(marker); j++) {
			if (unlikely(i+j == len))
				return NULL;
			if (*(m+i+j) == marker[cam->sof.bytesread]) {
				cam->sof.header[cam->sof.bytesread] = *(m+i+j);
				if (++cam->sof.bytesread == sizeof(marker)) {
					PDBGG("Bytes to analyze: %zd. SOF "
					      "starts at byte #%zd", len, i);
					i += j+1;
					break;
				}
			} else {
				cam->sof.bytesread = 0;
				break;
			}
		}
	}

	return NULL;
}


static void*
sn9c102_find_eof_header(struct sn9c102_device* cam, void* mem, size_t len)
{
	static const u8 eof_header[4][4] = {
		{0x00, 0x00, 0x00, 0x00},
		{0x40, 0x00, 0x00, 0x00},
		{0x80, 0x00, 0x00, 0x00},
		{0xc0, 0x00, 0x00, 0x00},
	};
	size_t i, j;

	/* The EOF header does not exist in compressed data */
	if (cam->sensor.pix_format.pixelformat == V4L2_PIX_FMT_SN9C10X ||
	    cam->sensor.pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
		return NULL;

	/*
	   The EOF header might cross the packet boundary, but this is not a
	   problem, since the end of a frame is determined by checking its size
	   in the first place.
	*/
	for (i = 0; (len >= 4) && (i <= len - 4); i++)
		for (j = 0; j < ARRAY_SIZE(eof_header); j++)
			if (!memcmp(mem + i, eof_header[j], 4))
				return mem + i;

	return NULL;
}


static void
sn9c102_write_jpegheader(struct sn9c102_device* cam, struct sn9c102_frame_t* f)
{
	static const u8 jpeg_header[589] = {
		0xff, 0xd8, 0xff, 0xdb, 0x00, 0x84, 0x00, 0x06, 0x04, 0x05,
		0x06, 0x05, 0x04, 0x06, 0x06, 0x05, 0x06, 0x07, 0x07, 0x06,
		0x08, 0x0a, 0x10, 0x0a, 0x0a, 0x09, 0x09, 0x0a, 0x14, 0x0e,
		0x0f, 0x0c, 0x10, 0x17, 0x14, 0x18, 0x18, 0x17, 0x14, 0x16,
		0x16, 0x1a, 0x1d, 0x25, 0x1f, 0x1a, 0x1b, 0x23, 0x1c, 0x16,
		0x16, 0x20, 0x2c, 0x20, 0x23, 0x26, 0x27, 0x29, 0x2a, 0x29,
		0x19, 0x1f, 0x2d, 0x30, 0x2d, 0x28, 0x30, 0x25, 0x28, 0x29,
		0x28, 0x01, 0x07, 0x07, 0x07, 0x0a, 0x08, 0x0a, 0x13, 0x0a,
		0x0a, 0x13, 0x28, 0x1a, 0x16, 0x1a, 0x28, 0x28, 0x28, 0x28,
		0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
		0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
		0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
		0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
		0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0xff, 0xc4, 0x01, 0xa2,
		0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
		0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x01,
		0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03,
		0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x10, 0x00,
		0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04,
		0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04,
		0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
		0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23,
		0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62,
		0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25,
		0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38,
		0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
		0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
		0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76,
		0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
		0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
		0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa,
		0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
		0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3,
		0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3,
		0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3,
		0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0x11, 0x00, 0x02,
		0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04,
		0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
		0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
		0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1,
		0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
		0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19,
		0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
		0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
		0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
		0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76,
		0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
		0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
		0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
		0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
		0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
		0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3,
		0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
		0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xff, 0xc0, 0x00, 0x11,
		0x08, 0x01, 0xe0, 0x02, 0x80, 0x03, 0x01, 0x21, 0x00, 0x02,
		0x11, 0x01, 0x03, 0x11, 0x01, 0xff, 0xda, 0x00, 0x0c, 0x03,
		0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3f, 0x00
	};
	u8 *pos = f->bufmem;

	memcpy(pos, jpeg_header, sizeof(jpeg_header));
	if (cam->compression.quality == 0) {
		memcpy(pos + 7, SN9C102_Y_QTABLE0, 64);
		memcpy(pos + 8 + 64, SN9C102_UV_QTABLE0, 64);
	} else if (cam->compression.quality == 1) {
		memcpy(pos + 7, SN9C102_Y_QTABLE1, 64);
		memcpy(pos + 8 + 64, SN9C102_UV_QTABLE1, 64);
	}
	*(pos + 564) = cam->sensor.pix_format.width & 0xFF;
	*(pos + 563) = (cam->sensor.pix_format.width >> 8) & 0xFF;
	*(pos + 562) = cam->sensor.pix_format.height & 0xFF;
	*(pos + 561) = (cam->sensor.pix_format.height >> 8) & 0xFF;

	f->buf.bytesused += sizeof(jpeg_header);
}


static void sn9c102_urb_complete(struct urb *urb)
{
	struct sn9c102_device* cam = urb->context;
	struct sn9c102_frame_t** f;
	size_t imagesize, soflen;
	u8 i;
	int err = 0;

	if (urb->status == -ENOENT)
		return;

	f = &cam->frame_current;

	if (cam->stream == STREAM_INTERRUPT) {
		cam->stream = STREAM_OFF;
		if ((*f))
			(*f)->state = F_QUEUED;
		cam->sof.bytesread = 0;
		DBG(3, "Stream interrupted by application");
		wake_up(&cam->wait_stream);
	}

	if (cam->state & DEV_DISCONNECTED)
		return;

	if (cam->state & DEV_MISCONFIGURED) {
		wake_up_interruptible(&cam->wait_frame);
		return;
	}

	if (cam->stream == STREAM_OFF || list_empty(&cam->inqueue))
		goto resubmit_urb;

	if (!(*f))
		(*f) = list_entry(cam->inqueue.next, struct sn9c102_frame_t,
		                  frame);

	imagesize = (cam->sensor.pix_format.width *
	             cam->sensor.pix_format.height *
	             cam->sensor.pix_format.priv) / 8;
	if (cam->sensor.pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
		imagesize += 589; /* length of the JPEG header */
	soflen = sn9c102_sof_length(cam);

	for (i = 0; i < urb->number_of_packets; i++) {
		unsigned int img, len;
		int status;
		void *pos, *sof, *eof;

		len = urb->iso_frame_desc[i].actual_length;
		status = urb->iso_frame_desc[i].status;
		pos = urb->iso_frame_desc[i].offset + urb->transfer_buffer;

		if (status) {
			DBG(3, "Error %u in isochronous frame", status);
			(*f)->state = F_ERROR;
			cam->sof.bytesread = 0;
			continue;
		}

		PDBGG("Isochrnous frame: length %u, #%u i", len, i);

redo:
		sof = sn9c102_find_sof_header(cam, pos, len);
		if (likely(!sof)) {
			eof = sn9c102_find_eof_header(cam, pos, len);
			if ((*f)->state == F_GRABBING) {
end_of_frame:
				img = len;

				if (eof)
					img = (eof > pos) ? eof - pos - 1 : 0;

				if ((*f)->buf.bytesused + img > imagesize) {
					u32 b;
					b = (*f)->buf.bytesused + img -
					    imagesize;
					img = imagesize - (*f)->buf.bytesused;
					PDBGG("Expected EOF not found: video "
					      "frame cut");
					if (eof)
						DBG(3, "Exceeded limit: +%u "
						       "bytes", (unsigned)(b));
				}

				memcpy((*f)->bufmem + (*f)->buf.bytesused, pos, img);

				if ((*f)->buf.bytesused == 0)
					/* do_gettimeofday(&(*f)->buf.timestamp); - josemar*/
					{
                        struct timespec64 tm;
                        ktime_get_ts64(&tm);
                        (*f)->buf.timestamp.tv_sec = tm.tv_sec;
                        (*f)->buf.timestamp.tv_usec = 1000 * tm.tv_nsec;
					}


				(*f)->buf.bytesused += img;

				if ((*f)->buf.bytesused == imagesize ||
				    ((cam->sensor.pix_format.pixelformat ==
				      V4L2_PIX_FMT_SN9C10X ||
				      cam->sensor.pix_format.pixelformat ==
				      V4L2_PIX_FMT_JPEG) && eof)) {
					u32 b;

					b = (*f)->buf.bytesused;
					(*f)->state = F_DONE;
					(*f)->buf.sequence= ++cam->frame_count;

					spin_lock(&cam->queue_lock);
					list_move_tail(&(*f)->frame,
					               &cam->outqueue);
					if (!list_empty(&cam->inqueue))
						(*f) = list_entry(
						        cam->inqueue.next,
						        struct sn9c102_frame_t,
						        frame );
					else
						(*f) = NULL;
					spin_unlock(&cam->queue_lock);

					memcpy(cam->sysfs.frame_header,
					       cam->sof.header, soflen);

					DBG(3, "Video frame captured: %lu "
					       "bytes", (unsigned long)(b));

					if (!(*f))
						goto resubmit_urb;

				} else if (eof) {
					(*f)->state = F_ERROR;
					DBG(3, "Not expected EOF after %lu "
					       "bytes of image data",
					    (unsigned long)
					    ((*f)->buf.bytesused));
				}

				if (sof) /* (1) */
					goto start_of_frame;

			} else if (eof) {
				DBG(3, "EOF without SOF");
				continue;

			} else {
				PDBGG("Ignoring pointless isochronous frame");
				continue;
			}

		} else if ((*f)->state == F_QUEUED || (*f)->state == F_ERROR) {
start_of_frame:
			(*f)->state = F_GRABBING;
			(*f)->buf.bytesused = 0;
			len -= (sof - pos);
			pos = sof;
			if (cam->sensor.pix_format.pixelformat ==
			    V4L2_PIX_FMT_JPEG)
				sn9c102_write_jpegheader(cam, (*f));
			DBG(3, "SOF detected: new video frame");
			if (len)
				goto redo;

		} else if ((*f)->state == F_GRABBING) {
			eof = sn9c102_find_eof_header(cam, pos, len);
			if (eof && eof < sof)
				goto end_of_frame; /* (1) */
			else {
				if (cam->sensor.pix_format.pixelformat ==
				    V4L2_PIX_FMT_SN9C10X ||
				    cam->sensor.pix_format.pixelformat ==
				    V4L2_PIX_FMT_JPEG) {
					if (sof - pos >= soflen) {
						eof = sof - soflen;
					} else { /* remove header */
						eof = pos;
						(*f)->buf.bytesused -=
						        (soflen - (sof - pos));
					}
					goto end_of_frame;
				} else {
					DBG(3, "SOF before expected EOF after "
					       "%lu bytes of image data",
					    (unsigned long)
					    ((*f)->buf.bytesused));
					goto start_of_frame;
				}
			}
		}
	}

resubmit_urb:
	urb->dev = cam->usbdev;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0 && err != -EPERM) {
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "usb_submit_urb() failed");
	}

	wake_up_interruptible(&cam->wait_frame);
}


static int sn9c102_start_transfer(struct sn9c102_device* cam)
{
	struct usb_device *udev = cam->usbdev;
	struct urb* urb;
	struct usb_host_interface* altsetting = usb_altnum_to_altsetting(
	                                            usb_ifnum_to_if(udev, 0),
	                                            SN9C102_ALTERNATE_SETTING);
	const unsigned int psz = le16_to_cpu(altsetting->
	                                     endpoint[0].desc.wMaxPacketSize);
	s8 i, j;
	int err = 0;

	for (i = 0; i < SN9C102_URBS; i++) {
		cam->transfer_buffer[i] = kzalloc(SN9C102_ISO_PACKETS * psz,
		                                  GFP_KERNEL);
		if (!cam->transfer_buffer[i]) {
			err = -ENOMEM;
			DBG(1, "Not enough memory");
			goto free_buffers;
		}
	}

	for (i = 0; i < SN9C102_URBS; i++) {
		urb = usb_alloc_urb(SN9C102_ISO_PACKETS, GFP_KERNEL);
		cam->urb[i] = urb;
		if (!urb) {
			err = -ENOMEM;
			DBG(1, "usb_alloc_urb() failed");
			goto free_urbs;
		}
		urb->dev = udev;
		urb->context = cam;
		urb->pipe = usb_rcvisocpipe(udev, 1);
		urb->transfer_flags = URB_ISO_ASAP;
		urb->number_of_packets = SN9C102_ISO_PACKETS;
		urb->complete = sn9c102_urb_complete;
		urb->transfer_buffer = cam->transfer_buffer[i];
		urb->transfer_buffer_length = psz * SN9C102_ISO_PACKETS;
		urb->interval = 1;
		for (j = 0; j < SN9C102_ISO_PACKETS; j++) {
			urb->iso_frame_desc[j].offset = psz * j;
			urb->iso_frame_desc[j].length = psz;
		}
	}

	if ((err = sn9c102_reset_imagepipe(cam)))
		goto free_urbs;

	err = usb_set_interface(udev, 0, SN9C102_ALTERNATE_SETTING);
	if (err) {
		DBG(1, "usb_set_interface() failed");
		goto free_urbs;
	}

	cam->frame_current = NULL;
	cam->sof.bytesread = 0;

	for (i = 0; i < SN9C102_URBS; i++) {
		err = usb_submit_urb(cam->urb[i], GFP_KERNEL);
		if (err) {
			for (j = i-1; j >= 0; j--)
				usb_kill_urb(cam->urb[j]);
			DBG(1, "usb_submit_urb() failed, error %d", err);
			goto free_urbs;
		}
	}

	return 0;

free_urbs:
	for (i = 0; (i < SN9C102_URBS) && cam->urb[i]; i++)
		usb_free_urb(cam->urb[i]);

free_buffers:
	for (i = 0; (i < SN9C102_URBS) && cam->transfer_buffer[i]; i++)
		kfree(cam->transfer_buffer[i]);

	return err;
}


static int sn9c102_stop_transfer(struct sn9c102_device* cam)
{
	struct usb_device *udev = cam->usbdev;
	s8 i;
	int err = 0;

	if (cam->state & DEV_DISCONNECTED)
		return 0;

	for (i = SN9C102_URBS-1; i >= 0; i--) {
		usb_kill_urb(cam->urb[i]);
		usb_free_urb(cam->urb[i]);
		kfree(cam->transfer_buffer[i]);
	}

	cam->stream = STREAM_OFF;

	err = usb_set_interface(udev, 0, 0); /* 0 Mb/s */
	if (err)
		DBG(3, "usb_set_interface() failed");

	return err;
}


static int sn9c102_stream_interrupt(struct sn9c102_device* cam)
{
	long timeout;

	cam->stream = STREAM_INTERRUPT;
	timeout = wait_event_timeout(cam->wait_stream,
	                             (cam->stream == STREAM_OFF) ||
	                             (cam->state & DEV_DISCONNECTED),
	                             SN9C102_URB_TIMEOUT);
	if (cam->state & DEV_DISCONNECTED)
		return -ENODEV;
	else if (cam->stream != STREAM_OFF) {
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "URB timeout reached. The camera is misconfigured. "
		       "To use it, close and open /dev/video%d again.",
		    cam->v4ldev->minor);
		return -EIO;
	}

	return 0;
}

/*****************************************************************************/

#if defined(CONFIG_VIDEO_ADV_DEBUG) && defined(SN9C102_ENABLE_SYSFS)
static u16 sn9c102_strtou16(const char* buff, size_t len, ssize_t* count)
{
	char str[7];
	char* endp;
	unsigned long val;

	if (len < 6) {
		strncpy(str, buff, len);
		str[len] = '\0';
	} else {
		strncpy(str, buff, 6);
		str[6] = '\0';
	}

	val = simple_strtoul(str, &endp, 0);

	*count = 0;
	if (val <= 0xffff)
		*count = (ssize_t)(endp - str);
	if ((*count) && (len == *count+1) && (buff[*count] == '\n'))
		*count += 1;

	return (u16)val;
}

/*
   NOTE 1: being inside one of the following methods implies that the v4l
           device exists for sure (see kobjects and reference counters)
   NOTE 2: buffers are PAGE_SIZE long
*/

static ssize_t sn9c102_show_reg(struct device* cd,
                                struct device_attribute *attr, char* buf)
{
	struct sn9c102_device* cam;
	ssize_t count;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	count = sprintf(buf, "%u\n", cam->sysfs.reg);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t
sn9c102_store_reg(struct device* cd, struct device_attribute *attr,
                  const char* buf, size_t len)
{
	struct sn9c102_device* cam;
	u16 index;
	ssize_t count;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	index = sn9c102_strtou16(buf, len, &count);
	if (index >= ARRAY_SIZE(cam->reg) || !count) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EINVAL;
	}

	cam->sysfs.reg = index;

	DBG(2, "Moved SN9C1XX register index to 0x%02X", cam->sysfs.reg);
	DBG(3, "Written bytes: %zd", count);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t sn9c102_show_val(struct device* cd,
                                struct device_attribute *attr, char* buf)
{
	struct sn9c102_device* cam;
	ssize_t count;
	int val;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	if ((val = sn9c102_read_reg(cam, cam->sysfs.reg)) < 0) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EIO;
	}

	count = sprintf(buf, "%d\n", val);

	DBG(3, "Read bytes: %zd, value: %d", count, val);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t
sn9c102_store_val(struct device* cd,  struct device_attribute *attr,
                  const char* buf, size_t len)
{
	struct sn9c102_device* cam;
	u16 value;
	ssize_t count;
	int err;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	value = sn9c102_strtou16(buf, len, &count);
	if (!count) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EINVAL;
	}

	err = sn9c102_write_reg(cam, value, cam->sysfs.reg);
	if (err) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EIO;
	}

	DBG(2, "Written SN9C1XX reg. 0x%02X, val. 0x%02X",
	    cam->sysfs.reg, value);
	DBG(3, "Written bytes: %zd", count);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t sn9c102_show_i2c_reg(struct device* cd,
                                    struct device_attribute *attr, char* buf)
{
	struct sn9c102_device* cam;
	ssize_t count;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	count = sprintf(buf, "%u\n", cam->sysfs.i2c_reg);

	DBG(3, "Read bytes: %zd", count);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t
sn9c102_store_i2c_reg(struct device* cd,  struct device_attribute *attr,
                      const char* buf, size_t len)
{
	struct sn9c102_device* cam;
	u16 index;
	ssize_t count;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	index = sn9c102_strtou16(buf, len, &count);
	if (!count) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EINVAL;
	}

	cam->sysfs.i2c_reg = index;

	DBG(2, "Moved sensor register index to 0x%02X", cam->sysfs.i2c_reg);
	DBG(3, "Written bytes: %zd", count);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t sn9c102_show_i2c_val(struct device* cd,
                                    struct device_attribute *attr, char* buf)
{
	struct sn9c102_device* cam;
	ssize_t count;
	int val;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	if (!(cam->sensor.sysfs_ops & SN9C102_I2C_READ)) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENOSYS;
	}

	if ((val = sn9c102_i2c_read(cam, cam->sysfs.i2c_reg)) < 0) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EIO;
	}

	count = sprintf(buf, "%d\n", val);

	DBG(3, "Read bytes: %zd, value: %d", count, val);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t
sn9c102_store_i2c_val(struct device* cd,  struct device_attribute *attr,
                      const char* buf, size_t len)
{
	struct sn9c102_device* cam;
	u16 value;
	ssize_t count;
	int err;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	if (!(cam->sensor.sysfs_ops & SN9C102_I2C_WRITE)) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENOSYS;
	}

	value = sn9c102_strtou16(buf, len, &count);
	if (!count) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EINVAL;
	}

	err = sn9c102_i2c_write(cam, cam->sysfs.i2c_reg, value);
	if (err) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -EIO;
	}

	DBG(2, "Written sensor reg. 0x%02X, val. 0x%02X",
	    cam->sysfs.i2c_reg, value);
	DBG(3, "Written bytes: %zd", count);

	mutex_unlock(&sn9c102_sysfs_lock);

	return count;
}


static ssize_t
sn9c102_store_green(struct device* cd,  struct device_attribute *attr,
                    const char* buf, size_t len)
{
	struct sn9c102_device* cam;
	enum sn9c102_bridge bridge;
	ssize_t res = 0;
	u16 value;
	ssize_t count;

	if (mutex_lock_interruptible(&sn9c102_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&sn9c102_sysfs_lock);
		return -ENODEV;
	}

	bridge = cam->bridge;

	mutex_unlock(&sn9c102_sysfs_lock);

	value = sn9c102_strtou16(buf, len, &count);
	if (!count)
		return -EINVAL;

	switch (bridge) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
		if (value > 0x0f)
			return -EINVAL;
		if ((res = sn9c102_store_reg(cd, attr, "0x11", 4)) >= 0)
			res = sn9c102_store_val(cd, attr, buf, len);
		break;
	case BRIDGE_SN9C103:
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		if (value > 0x7f)
			return -EINVAL;
		if ((res = sn9c102_store_reg(cd, attr, "0x07", 4)) >= 0)
			res = sn9c102_store_val(cd, attr, buf, len);
		break;
	}

	return res;
}


static ssize_t
sn9c102_store_blue(struct device* cd,  struct device_attribute *attr,
                   const char* buf, size_t len)
{
	ssize_t res = 0;
	u16 value;
	ssize_t count;

	value = sn9c102_strtou16(buf, len, &count);
	if (!count || value > 0x7f)
		return -EINVAL;

	if ((res = sn9c102_store_reg(cd, attr, "0x06", 4)) >= 0)
		res = sn9c102_store_val(cd, attr, buf, len);

	return res;
}


static ssize_t
sn9c102_store_red(struct device* cd,  struct device_attribute *attr,
                  const char* buf, size_t len)
{
	ssize_t res = 0;
	u16 value;
	ssize_t count;

	value = sn9c102_strtou16(buf, len, &count);
	if (!count || value > 0x7f)
		return -EINVAL;

	if ((res = sn9c102_store_reg(cd, attr, "0x05", 4)) >= 0)
		res = sn9c102_store_val(cd, attr, buf, len);

	return res;
}


static ssize_t sn9c102_show_frame_header(struct device* cd,
                                         struct device_attribute *attr,
                                         char* buf)
{
	struct sn9c102_device* cam;
	ssize_t count;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam)
		return -ENODEV;

	count = sizeof(cam->sysfs.frame_header);
	memcpy(buf, cam->sysfs.frame_header, count);

	DBG(3, "Frame header, read bytes: %zd", count);

	return count;
}


static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR,
                   sn9c102_show_reg, sn9c102_store_reg);
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR,
                   sn9c102_show_val, sn9c102_store_val);
static DEVICE_ATTR(i2c_reg, S_IRUGO | S_IWUSR,
                   sn9c102_show_i2c_reg, sn9c102_store_i2c_reg);
static DEVICE_ATTR(i2c_val, S_IRUGO | S_IWUSR,
                   sn9c102_show_i2c_val, sn9c102_store_i2c_val);
static DEVICE_ATTR(green, /*S_IWUGO - josemar*/S_IWUSR|S_IWGRP, NULL, sn9c102_store_green);
static DEVICE_ATTR(blue, /*S_IWUGO - josemar*/S_IWUSR|S_IWGRP, NULL, sn9c102_store_blue);
static DEVICE_ATTR(red, /*S_IWUGO - josemar*/S_IWUSR|S_IWGRP, NULL, sn9c102_store_red);
static DEVICE_ATTR(frame_header, S_IRUGO, sn9c102_show_frame_header, NULL);


static int sn9c102_create_sysfs(struct sn9c102_device* cam)
{
	/* struct device *classdev = &(cam->v4ldev->class_dev); - josemar */
   	struct device *classdev = &(cam->v4ldev->dev);
	int err = 0;

	if ((err = device_create_file(classdev, &dev_attr_reg)))
		goto err_out;
	if ((err = device_create_file(classdev, &dev_attr_val)))
		goto err_reg;
	if ((err = device_create_file(classdev, &dev_attr_frame_header)))
		goto err_val;

	if (cam->sensor.sysfs_ops) {
		if ((err = device_create_file(classdev, &dev_attr_i2c_reg)))
			goto err_frame_header;
		if ((err = device_create_file(classdev, &dev_attr_i2c_val)))
			goto err_i2c_reg;
	}

	if (cam->bridge == BRIDGE_SN9C101 || cam->bridge == BRIDGE_SN9C102) {
		if ((err = device_create_file(classdev, &dev_attr_green)))
			goto err_i2c_val;
	} else {
		if ((err = device_create_file(classdev, &dev_attr_blue)))
			goto err_i2c_val;
		if ((err = device_create_file(classdev, &dev_attr_red)))
			goto err_blue;
	}

	return 0;

err_blue:
	device_remove_file(classdev, &dev_attr_blue);
err_i2c_val:
	if (cam->sensor.sysfs_ops)
		device_remove_file(classdev, &dev_attr_i2c_val);
err_i2c_reg:
	if (cam->sensor.sysfs_ops)
		device_remove_file(classdev, &dev_attr_i2c_reg);
err_frame_header:
	device_remove_file(classdev, &dev_attr_frame_header);
err_val:
	device_remove_file(classdev, &dev_attr_val);
err_reg:
	device_remove_file(classdev, &dev_attr_reg);
err_out:
	return err;
}
#endif /* CONFIG_VIDEO_ADV_DEBUG && SN9C102_ENABLE_SYSFS */

/*****************************************************************************/

static int
sn9c102_set_pix_format(struct sn9c102_device* cam, struct v4l2_pix_format* pix)
{
	int err = 0;

	if (pix->pixelformat == V4L2_PIX_FMT_SN9C10X ||
	    pix->pixelformat == V4L2_PIX_FMT_JPEG) {
		switch (cam->bridge) {
		case BRIDGE_SN9C101:
		case BRIDGE_SN9C102:
		case BRIDGE_SN9C103:
			err += sn9c102_write_reg(cam, cam->reg[0x18] | 0x80,
			                         0x18);
			break;
		case BRIDGE_SN9C105:
		case BRIDGE_SN9C120:
			err += sn9c102_write_reg(cam, cam->reg[0x18] & 0x7f,
			                         0x18);
			break;
		}
	} else {
		switch (cam->bridge) {
		case BRIDGE_SN9C101:
		case BRIDGE_SN9C102:
		case BRIDGE_SN9C103:
			err += sn9c102_write_reg(cam, cam->reg[0x18] & 0x7f,
			                         0x18);
			break;
		case BRIDGE_SN9C105:
		case BRIDGE_SN9C120:
			err += sn9c102_write_reg(cam, cam->reg[0x18] | 0x80,
			                         0x18);
			break;
		}
	}

	return err ? -EIO : 0;
}


static int
sn9c102_set_compression(struct sn9c102_device* cam,
                        struct v4l2_jpegcompression* compression)
{
	int i, err = 0;

	switch (cam->bridge) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
	case BRIDGE_SN9C103:
		if (compression->quality == 0)
			err += sn9c102_write_reg(cam, cam->reg[0x17] | 0x01,
			                         0x17);
		else if (compression->quality == 1)
			err += sn9c102_write_reg(cam, cam->reg[0x17] & 0xfe,
			                         0x17);
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		if (compression->quality == 0) {
			for (i = 0; i <= 63; i++) {
				err += sn9c102_write_reg(cam,
				                         SN9C102_Y_QTABLE1[i],
				                         0x100 + i);
				err += sn9c102_write_reg(cam,
				                         SN9C102_UV_QTABLE1[i],
				                         0x140 + i);
			}
			err += sn9c102_write_reg(cam, cam->reg[0x18] & 0xbf,
			                         0x18);
		} else if (compression->quality == 1) {
			for (i = 0; i <= 63; i++) {
				err += sn9c102_write_reg(cam,
				                         SN9C102_Y_QTABLE1[i],
				                         0x100 + i);
				err += sn9c102_write_reg(cam,
				                         SN9C102_UV_QTABLE1[i],
				                         0x140 + i);
			}
			err += sn9c102_write_reg(cam, cam->reg[0x18] | 0x40,
			                         0x18);
		}
		break;
	}

	return err ? -EIO : 0;
}


static int sn9c102_set_scale(struct sn9c102_device* cam, u8 scale)
{
	u8 r = 0;
	int err = 0;

	if (scale == 1)
		r = cam->reg[0x18] & 0xcf;
	else if (scale == 2) {
		r = cam->reg[0x18] & 0xcf;
		r |= 0x10;
	} else if (scale == 4)
		r = cam->reg[0x18] | 0x20;

	err += sn9c102_write_reg(cam, r, 0x18);
	if (err)
		return -EIO;

	PDBGG("Scaling factor: %u", scale);

	return 0;
}


static int sn9c102_set_crop(struct sn9c102_device* cam, struct v4l2_rect* rect)
{
	struct sn9c102_sensor* s = &cam->sensor;
	u8 h_start = (u8)(rect->left - s->cropcap.bounds.left),
	   v_start = (u8)(rect->top - s->cropcap.bounds.top),
	   h_size = (u8)(rect->width / 16),
	   v_size = (u8)(rect->height / 16);
	int err = 0;

	err += sn9c102_write_reg(cam, h_start, 0x12);
	err += sn9c102_write_reg(cam, v_start, 0x13);
	err += sn9c102_write_reg(cam, h_size, 0x15);
	err += sn9c102_write_reg(cam, v_size, 0x16);
	if (err)
		return -EIO;

	PDBGG("h_start, v_start, h_size, v_size %u %u %u %u", h_start, v_start,
	      h_size, v_size);

	return 0;
}


static int sn9c102_init(struct sn9c102_device* cam)
{
	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_control ctrl;
	struct v4l2_queryctrl *qctrl;
	struct v4l2_rect* rect;
	u8 i = 0;
	int err = 0;

	if (!(cam->state & DEV_INITIALIZED)) {
		init_waitqueue_head(&cam->wait_open);
		memcpy(s->_qctrl, s->qctrl, sizeof(s->qctrl));
		memcpy(&(s->_rect), &(s->cropcap.defrect),
		       sizeof(struct v4l2_rect));
	}

	qctrl = s->_qctrl;
	rect = &(s->_rect);

	if (s->init) {
		err = s->init(cam);
		if (err) {
			DBG(3, "Sensor initialization failed");
			return err;
		}
	}

	if (!(cam->state & DEV_INITIALIZED))
		if (cam->bridge == BRIDGE_SN9C101 ||
		    cam->bridge == BRIDGE_SN9C102 ||
		    cam->bridge == BRIDGE_SN9C103) {
			if (s->pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
				s->pix_format.pixelformat= V4L2_PIX_FMT_SBGGR8;
			cam->compression.quality =  cam->reg[0x17] & 0x01 ?
			                            0 : 1;
		} else {
			if (s->pix_format.pixelformat == V4L2_PIX_FMT_SN9C10X)
				s->pix_format.pixelformat = V4L2_PIX_FMT_JPEG;
			cam->compression.quality =  cam->reg[0x18] & 0x40 ?
			                            0 : 1;
			err += sn9c102_set_compression(cam, &cam->compression);
		}
	else
		err += sn9c102_set_compression(cam, &cam->compression);
	err += sn9c102_set_scale(cam, rect->width / s->pix_format.width);
	err += sn9c102_set_crop(cam, rect);
	err += sn9c102_set_pix_format(cam, &s->pix_format);
	if (s->set_pix_format)
		err += s->set_pix_format(cam, &s->pix_format);
	if (err)
		return err;

	if (s->pix_format.pixelformat == V4L2_PIX_FMT_SN9C10X ||
	    s->pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
		DBG(3, "Compressed video format is active, quality %d",
		    cam->compression.quality);
	else
		DBG(3, "Uncompressed video format is active");

	if (s->set_crop)
		if ((err = s->set_crop(cam, rect))) {
			DBG(3, "set_crop() failed");
			return err;
		}

	if (s->set_ctrl) {
		for (i = 0; i < ARRAY_SIZE(s->qctrl); i++)
			if (s->qctrl[i].id != 0 &&
			    !(s->qctrl[i].flags & V4L2_CTRL_FLAG_DISABLED)) {
				ctrl.id = s->qctrl[i].id;
				ctrl.value = qctrl[i].default_value;
				err = s->set_ctrl(cam, &ctrl);
				if (err) {
					DBG(3, "Set %s control failed",
					    s->qctrl[i].name);
					return err;
				}
				DBG(3, "Image sensor supports '%s' control",
				    s->qctrl[i].name);
			}
	}

	/* Enable video */
	if (!(cam->reg[0x01] & 0x04)) {
		err = sn9c102_write_reg(cam, cam->reg[0x01] | 0x04, 0x01);
		if (err) {
			DBG(1, "I/O hardware error");
			return -EIO;
		}
	}

	if (!(cam->state & DEV_INITIALIZED)) {
		mutex_init(&cam->fileop_mutex);
		spin_lock_init(&cam->queue_lock);
		init_waitqueue_head(&cam->wait_frame);
		init_waitqueue_head(&cam->wait_stream);
		sn9c102_empty_framequeues(cam);
		cam->nreadbuffers = 2;
		cam->state |= DEV_INITIALIZED;
	}

	DBG(2, "Initialization succeeded");
	return 0;
}

/*****************************************************************************/

static void sn9c102_release_resources(struct kref *kref)
{
	struct sn9c102_device *cam;

#if defined(CONFIG_VIDEO_ADV_DEBUG) && defined(SN9C102_ENABLE_SYSFS)
	mutex_lock(&sn9c102_sysfs_lock);
#endif
	cam = container_of(kref, struct sn9c102_device, kref);


	v4l2_device_disconnect(cam->v4ldev->v4l2_dev);
	v4l2_device_unregister(cam->v4ldev->v4l2_dev);
    kfree(cam->v4ldev->v4l2_dev);

	DBG(2, "V4L2 device /dev/video%d deregistered", cam->v4ldev->minor);
	video_set_drvdata(cam->v4ldev, NULL);
	video_unregister_device(cam->v4ldev);
	usb_put_dev(cam->usbdev);
	kfree(cam->control_buffer);
	kfree(cam);

#if defined(CONFIG_VIDEO_ADV_DEBUG) && defined(SN9C102_ENABLE_SYSFS)
	mutex_unlock(&sn9c102_sysfs_lock);
#endif
}


/*static int sn9c102_open(struct inode* inode, struct file* filp) - josemar */
static int sn9c102_open(struct file* filp)
{
	struct sn9c102_device* cam;
	int err = 0;

	/*
	   A mutex_trylock() in open() is the only safe way to prevent race
	   conditions with disconnect(), one close() and multiple (not
	   necessarily simultaneous) attempts to open(). For example, it
	   prevents from waiting for a second access, while the device
	   structure is being deallocated, after a possible disconnect() and
	   during a following close() holding the write lock: given that, after
	   this deallocation, no access will be possible anymore; using the
	   non-trylock version would have let open() gain the access to the
	   device structure improperly.
	   For this reason the lock must also not be per-device.
	*/
	if (!mutex_trylock(&sn9c102_dev_lock))
		return -EAGAIN;

	cam = video_get_drvdata(video_devdata(filp));


	if (mutex_lock_interruptible(&cam->fileop_mutex)) {
		mutex_unlock(&sn9c102_dev_lock);
		return -ERESTARTSYS;
	}

	if (wait_for_completion_interruptible(&cam->probe)) {
		err = -ERESTARTSYS;
		goto exit;
	}

	if (cam->state & DEV_DISCONNECTED) {
		DBG(1, "Device not present");
		err = -ENODEV;
		goto exit;
	}

	kref_get(&cam->kref);

	if ((cam->users && cam->users == cam->module_param.max_opens) ||
	    waitqueue_active(&cam->wait_open)) {
		DBG(2, "The maximum number of allowed simultaneous opens for "
		        "device /dev/video%d has been reached",
		    cam->v4ldev->minor);
		/*
		   open() must follow the open flags and should block
		   eventually while the device is in use.
		*/
		if ((filp->f_flags & O_NONBLOCK) ||
		    (filp->f_flags & O_NDELAY)) {
			err = -EWOULDBLOCK;
			goto out;
		}
		DBG(2, "A blocking open() has been requested. Wait for the "
		       "device to be released...");
		mutex_unlock(&cam->fileop_mutex);
		mutex_unlock(&sn9c102_dev_lock);
		err = wait_event_interruptible_exclusive(cam->wait_open,
		                                (cam->state & DEV_DISCONNECTED)
		                                         || !cam->users);
		mutex_lock(&sn9c102_dev_lock);
		mutex_lock(&cam->fileop_mutex);
		if (err)
			goto out;
		if (cam->state & DEV_DISCONNECTED) {
			err = -ENODEV;
			goto out;
		}
	}

	if (cam->state & DEV_MISCONFIGURED) {
		err = sn9c102_init(cam);
		if (err) {
			DBG(1, "Initialization failed again. "
			       "I will retry on next open().");
			goto out;
		}
		cam->state &= ~DEV_MISCONFIGURED;
	}

	if (!cam->users)
		if ((err = sn9c102_start_transfer(cam)))
			goto out;

	filp->private_data = cam;
	cam->users++;

	DBG(3, "Video device /dev/video%d is open, actual users: %d",
	    cam->v4ldev->minor, cam->users);

out:
	if (err)
		kref_put(&cam->kref, sn9c102_release_resources);

exit:
	mutex_unlock(&cam->fileop_mutex);
	mutex_unlock(&sn9c102_dev_lock);

	return err;
}


/* static int sn9c102_release(struct inode* inode, struct file* filp) - josemar */
static int sn9c102_release(struct file* filp)
{
	struct sn9c102_device* cam;

	mutex_lock(&sn9c102_dev_lock);

	cam = video_get_drvdata(video_devdata(filp));




	mutex_lock(&cam->fileop_mutex);

	if (!(--cam->users))
		sn9c102_stop_transfer(cam);

	if (cam->priority == filp) {
		sn9c102_empty_framequeues(cam);
		sn9c102_release_buffers(cam);
		cam->priority = NULL;
		cam->io = IO_NONE;
		cam->frame_count = 0;
	}

	DBG(3, "Video device /dev/video%d closed, actual users: %d",
	    cam->v4ldev->minor, cam->users);

	wake_up_interruptible_nr(&cam->wait_open, 1);

	kref_put(&cam->kref, sn9c102_release_resources);

	mutex_unlock(&cam->fileop_mutex);
	mutex_unlock(&sn9c102_dev_lock);

	return 0;
}


static ssize_t
sn9c102_read(struct file* filp, char __user * buf, size_t count, loff_t* f_pos)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
	struct sn9c102_frame_t* f, * i;
	unsigned long lock_flags;
	long timeout;
	int err = 0;

	if (mutex_lock_interruptible(&cam->fileop_mutex))
		return -ERESTARTSYS;

	if (cam->state & DEV_DISCONNECTED) {
		DBG(1, "Device not present");
		mutex_unlock(&cam->fileop_mutex);
		return -ENODEV;
	}

	if (cam->state & DEV_MISCONFIGURED) {
		DBG(1, "The camera is misconfigured. Close and open it "
		       "again.");
		mutex_unlock(&cam->fileop_mutex);
		return -EIO;
	}

	if (cam->io == IO_MMAP) {
		DBG(3, "Close and open the device again to choose "
		       "the read method");
		mutex_unlock(&cam->fileop_mutex);
		return -EBUSY;
	}

	if (cam->io == IO_READ && cam->priority != filp) {
		DBG(2, "Device /dev/video%d is already in use for streaming",
		    cam->v4ldev->minor);
		mutex_unlock(&cam->fileop_mutex);
		return -EBUSY;
	}

	if (cam->io == IO_NONE) {
		if (!sn9c102_request_buffers(cam,cam->nreadbuffers, IO_READ)) {
			DBG(1, "read() failed, not enough memory");
			mutex_unlock(&cam->fileop_mutex);
			return -ENOMEM;
		}
		cam->priority = filp;
		cam->io = IO_READ;
		cam->stream = STREAM_ON;
	}

	if (list_empty(&cam->inqueue)) {
		if (!list_empty(&cam->outqueue))
			sn9c102_empty_framequeues(cam);
		sn9c102_queue_unusedframes(cam);
	}

	if (!count) {
		mutex_unlock(&cam->fileop_mutex);
		return 0;
	}

	if (list_empty(&cam->outqueue)) {
		if (filp->f_flags & O_NONBLOCK) {
			mutex_unlock(&cam->fileop_mutex);
			return -EAGAIN;
		}
		if (!cam->module_param.frame_timeout) {
			err = wait_event_interruptible
			      ( cam->wait_frame,
			        (!list_empty(&cam->outqueue)) ||
			        (cam->state & DEV_DISCONNECTED) ||
			        (cam->state & DEV_MISCONFIGURED) );
			if (err) {
				mutex_unlock(&cam->fileop_mutex);
				return err;
			}
		} else {
			timeout = wait_event_interruptible_timeout
			          ( cam->wait_frame,
			            (!list_empty(&cam->outqueue)) ||
			            (cam->state & DEV_DISCONNECTED) ||
			            (cam->state & DEV_MISCONFIGURED),
			            cam->module_param.frame_timeout *
			            1000 * msecs_to_jiffies(1) );
			if (timeout < 0) {
				mutex_unlock(&cam->fileop_mutex);
				return timeout;
			} else if (timeout == 0 &&
			           !(cam->state & DEV_DISCONNECTED)) {
				DBG(1, "Video frame timeout elapsed");
				mutex_unlock(&cam->fileop_mutex);
				return -EIO;
			}
		}
		if (cam->state & DEV_DISCONNECTED) {
			mutex_unlock(&cam->fileop_mutex);
			return -ENODEV;
		}
		if (cam->state & DEV_MISCONFIGURED) {
			mutex_unlock(&cam->fileop_mutex);
			return -EIO;
		}
	}

	f = list_entry(cam->outqueue.prev, struct sn9c102_frame_t, frame);

	if (count > f->buf.bytesused)
		count = f->buf.bytesused;

	if (copy_to_user(buf, f->bufmem, count)) {
		err = -EFAULT;
		goto exit;
	}
	*f_pos += count;

exit:
	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	list_for_each_entry(i, &cam->outqueue, frame)
		i->state = F_UNUSED;
	INIT_LIST_HEAD(&cam->outqueue);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	sn9c102_queue_unusedframes(cam);

	PDBGG("Frame #%lu, bytes read: %zu",
	      (unsigned long)f->buf.index, count);

	mutex_unlock(&cam->fileop_mutex);

	return count;
}


static unsigned int sn9c102_poll(struct file *filp, poll_table *wait)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
	unsigned int mask = 0;

	if (mutex_lock_interruptible(&cam->fileop_mutex))
		return POLLERR;

	if (cam->state & DEV_DISCONNECTED) {
		DBG(1, "Device not present");
		goto error;
	}

	if (cam->state & DEV_MISCONFIGURED) {
		DBG(1, "The camera is misconfigured. Close and open it "
		       "again.");
		goto error;
	}

	if (cam->io == IO_READ && cam->priority != filp) {
		DBG(2, "Device /dev/video%d is already in use for streaming",
		    cam->v4ldev->minor);
		goto error;
	}

	if (cam->io == IO_NONE) {
		if (!sn9c102_request_buffers(cam, cam->nreadbuffers,
		                             IO_READ)) {
			DBG(1, "poll() failed, not enough memory");
			goto error;
		}
		cam->priority = filp;
		cam->io = IO_READ;
		cam->stream = STREAM_ON;
	}

	if (cam->io == IO_READ)
		if (list_empty(&cam->inqueue) && cam->nbuffers > 1) {
			sn9c102_empty_framequeues(cam);
			sn9c102_queue_unusedframes(cam);
		}

	poll_wait(filp, &cam->wait_frame, wait);

	if (!list_empty(&cam->outqueue))
		mask |= POLLIN | POLLRDNORM;

	mutex_unlock(&cam->fileop_mutex);

	return mask;

error:
	mutex_unlock(&cam->fileop_mutex);
	return POLLERR;
}


static void sn9c102_vm_open(struct vm_area_struct* vma)
{
	struct sn9c102_frame_t* f = vma->vm_private_data;
	f->vma_use_count++;
}


static void sn9c102_vm_close(struct vm_area_struct* vma)
{
	/* NOTE: buffers are not freed here */
	struct sn9c102_frame_t* f = vma->vm_private_data;
	f->vma_use_count--;
}


static struct vm_operations_struct sn9c102_vm_ops = {
	.open = sn9c102_vm_open,
	.close = sn9c102_vm_close,
};


static int sn9c102_mmap(struct file* filp, struct vm_area_struct *vma)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
	unsigned long size = vma->vm_end - vma->vm_start,
	              start = vma->vm_start;
	void *pos;
	u32 i;

	if (mutex_lock_interruptible(&cam->fileop_mutex))
		return -ERESTARTSYS;

	if (cam->state & DEV_DISCONNECTED) {
		DBG(1, "Device not present");
		mutex_unlock(&cam->fileop_mutex);
		return -ENODEV;
	}

	if (cam->state & DEV_MISCONFIGURED) {
		DBG(1, "The camera is misconfigured. Close and open it "
		       "again.");
		mutex_unlock(&cam->fileop_mutex);
		return -EIO;
	}

	if (!(vma->vm_flags & (VM_WRITE | VM_READ))) {
		mutex_unlock(&cam->fileop_mutex);
		return -EACCES;
	}

	if (cam->io != IO_MMAP ||
	    size != PAGE_ALIGN(cam->frame[0].buf.length)) {
		mutex_unlock(&cam->fileop_mutex);
		return -EINVAL;
	}

	if (cam->io == IO_MMAP && cam->priority != filp) {
		DBG(2, "Device /dev/video%d is already in use for streaming",
		    cam->v4ldev->minor);
		mutex_unlock(&cam->fileop_mutex);
		return -EBUSY;
	}

	for (i = 0; i < cam->nbuffers; i++) {
		if ((cam->frame[i].buf.m.offset>>PAGE_SHIFT) == vma->vm_pgoff)
			break;
	}
	if (i == cam->nbuffers) {
		mutex_unlock(&cam->fileop_mutex);
		return -EINVAL;
	}

	vma->vm_flags |= VM_IO;
	/*vma->vm_flags |= VM_RESERVED; - josemar - It is no longer declared in mmflags.h */

	pos = cam->frame[i].bufmem;
	while (size > 0) { /* size is page-aligned */
		if (vm_insert_page(vma, start, vmalloc_to_page(pos))) {
			mutex_unlock(&cam->fileop_mutex);
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vma->vm_ops = &sn9c102_vm_ops;
	vma->vm_private_data = &cam->frame[i];
	sn9c102_vm_open(vma);

	mutex_unlock(&cam->fileop_mutex);

	return 0;
}

/*****************************************************************************/

static int
//sn9c102_vidioc_querycap(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_querycap(struct file *filp, void *fh, struct v4l2_capability *cap)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_capability xcap = {
		.driver = "sn9c102",
		.version = SN9C102_MODULE_VERSION_CODE,
		.capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
		                V4L2_CAP_STREAMING,
	};


	strlcpy(xcap.card, cam->v4ldev->name, sizeof(xcap.card));
	if (usb_make_path(cam->usbdev, xcap.bus_info, sizeof(xcap.bus_info)) < 0)
		/*strlcpy(xcap.bus_info, cam->usbdev->dev.bus_id, - josemar */
		strlcpy(xcap.bus_info, cam->usbdev->dev.kobj.name,
		        sizeof(xcap.bus_info));

	if (copy_to_user(cap, &xcap, sizeof(xcap)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_enuminput(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_enuminput(struct file *filp, void *fh, struct v4l2_input *inp)
{
    //struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_input i;

	if (copy_from_user(&i, inp, sizeof(i)))
		return -EFAULT;

	if (i.index)
		return -EINVAL;

	memset(&i, 0, sizeof(i));
	strcpy(i.name, "Camera");
	i.type = V4L2_INPUT_TYPE_CAMERA;

	if (copy_to_user(inp, &i, sizeof(i)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_g_input(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_g_input(struct file *filp, void *fh, unsigned int *i)
{
    //struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	unsigned int index = 0;

	if (copy_to_user(i, &index, sizeof(index)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_s_input(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_s_input(struct file *filp, void *fh, unsigned int i)
{
    //struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	unsigned int index;

	if (copy_from_user(&index, &i, sizeof(index)))
		return -EFAULT;

	if (index != 0)
		return -EINVAL;

	return 0;
}


static int
//sn9c102_vidioc_query_ctrl(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_query_ctrl(struct file *filp, void *fh, struct v4l2_queryctrl *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_queryctrl qc;
	u8 i;

	if (copy_from_user(&qc, a, sizeof(qc)))
		return -EFAULT;

	for (i = 0; i < ARRAY_SIZE(s->qctrl); i++)
		if (qc.id && qc.id == s->qctrl[i].id) {
			memcpy(&qc, &(s->qctrl[i]), sizeof(qc));
			if (copy_to_user(a, &qc, sizeof(qc)))
				return -EFAULT;
			return 0;
		}

	return -EINVAL;
}


static int
//sn9c102_vidioc_g_ctrl(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_g_ctrl(struct file *filp, void *fh, struct v4l2_control *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
    void __user * arg;

	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_control ctrl;
	int err = 0;
	u8 i;

	if (!s->get_ctrl && !s->set_ctrl)
		return -EINVAL;

	if (copy_from_user(&ctrl, arg, sizeof(ctrl)))
		return -EFAULT;

	if (!s->get_ctrl) {
		for (i = 0; i < ARRAY_SIZE(s->qctrl); i++)
			if (ctrl.id && ctrl.id == s->qctrl[i].id) {
				ctrl.value = s->_qctrl[i].default_value;
				goto exit;
			}
		return -EINVAL;
	} else
		err = s->get_ctrl(cam, &ctrl);

exit:
	if (copy_to_user(arg, &ctrl, sizeof(ctrl)))
		return -EFAULT;

	PDBGG("VIDIOC_G_CTRL: id %lu, value %lu",
	      (unsigned long)ctrl.id, (unsigned long)ctrl.value);

	return err;
}


static int
//sn9c102_vidioc_s_ctrl(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_s_ctrl(struct file *filp, void *fh, struct v4l2_control *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
    void __user * arg;

	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_control ctrl;
	u8 i;
	int err = 0;

	if (!s->set_ctrl)
		return -EINVAL;

	if (copy_from_user(&ctrl, arg, sizeof(ctrl)))
		return -EFAULT;

	for (i = 0; i < ARRAY_SIZE(s->qctrl); i++)
		if (ctrl.id == s->qctrl[i].id) {
			if (s->qctrl[i].flags & V4L2_CTRL_FLAG_DISABLED)
				return -EINVAL;
			if (ctrl.value < s->qctrl[i].minimum ||
			    ctrl.value > s->qctrl[i].maximum)
				return -ERANGE;
			ctrl.value -= ctrl.value % s->qctrl[i].step;
			break;
		}

	if ((err = s->set_ctrl(cam, &ctrl)))
		return err;

	s->_qctrl[i].default_value = ctrl.value;

	PDBGG("VIDIOC_S_CTRL: id %lu, value %lu",
	      (unsigned long)ctrl.id, (unsigned long)ctrl.value);

	return 0;
}


static int
//sn9c102_vidioc_cropcap(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_cropcap(struct file *filp, void *fh, int buf_type, struct v4l2_fract *aspect)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_cropcap* cc = &(cam->sensor.cropcap);

	cc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cc->pixelaspect.numerator = 1;
	cc->pixelaspect.denominator = 1;

	if (copy_to_user(aspect, cc, sizeof(*cc)))
		return -EFAULT;

	return 0;
}


static int
sn9c102_vidioc_g_crop(struct file *filp, void *fh, struct v4l2_crop *argp)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_crop crop = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	};

	memcpy(&(crop.c), &(s->_rect), sizeof(struct v4l2_rect));

	if (copy_to_user(argp, &crop, sizeof(crop)))
		return -EFAULT;

	return 0;
}


static int
sn9c102_vidioc_s_crop(struct file *filp, void *fh, struct v4l2_crop *argp)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_crop crop;
	struct v4l2_rect* rect;
	struct v4l2_rect* bounds = &(s->cropcap.bounds);
	struct v4l2_pix_format* pix_format = &(s->pix_format);
	u8 scale;
	const enum sn9c102_stream_state stream = cam->stream;
	const u32 nbuffers = cam->nbuffers;
	u32 i;
	int err = 0;

	if (copy_from_user(&crop, argp, sizeof(crop)))
		return -EFAULT;

	rect = &(crop.c);

	if (crop.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (cam->module_param.force_munmap)
		for (i = 0; i < cam->nbuffers; i++)
			if (cam->frame[i].vma_use_count) {
				DBG(3, "VIDIOC_S_CROP failed. "
				       "Unmap the buffers first.");
				return -EBUSY;
			}

	// * Preserve R,G or B origin
	rect->left = (s->_rect.left & 1L) ? rect->left | 1L : rect->left & ~1L;
	rect->top = (s->_rect.top & 1L) ? rect->top | 1L : rect->top & ~1L;

	if (rect->width < 48)
		rect->width = 48;
	if (rect->height < 48)
		rect->height = 48;
	if (rect->width > bounds->width)
		rect->width = bounds->width;
	if (rect->height > bounds->height)
		rect->height = bounds->height;
	if (rect->left < bounds->left)
		rect->left = bounds->left;
	if (rect->top < bounds->top)
		rect->top = bounds->top;
	if (rect->left + rect->width > bounds->left + bounds->width)
		rect->left = bounds->left+bounds->width - rect->width;
	if (rect->top + rect->height > bounds->top + bounds->height)
		rect->top = bounds->top+bounds->height - rect->height;

	rect->width &= ~15L;
	rect->height &= ~15L;

	if (SN9C102_PRESERVE_IMGSCALE) {
		// * Calculate the actual scaling factor
		u32 a, b;
		a = rect->width * rect->height;
		b = pix_format->width * pix_format->height;
		scale = b ? (u8)((a / b) < 4 ? 1 : ((a / b) < 16 ? 2 : 4)) : 1;
	} else
		scale = 1;

	if (cam->stream == STREAM_ON)
		if ((err = sn9c102_stream_interrupt(cam)))
			return err;

	if (copy_to_user(argp, &crop, sizeof(crop))) {
		cam->stream = stream;
		return -EFAULT;
	}

	if (cam->module_param.force_munmap || cam->io == IO_READ)
		sn9c102_release_buffers(cam);

	err = sn9c102_set_crop(cam, rect);
	if (s->set_crop)
		err += s->set_crop(cam, rect);
	err += sn9c102_set_scale(cam, scale);

	if (err) { // * atomic, no rollback in ioctl()
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "VIDIOC_S_CROP failed because of hardware problems. To "
		       "use the camera, close and open /dev/video%d again.",
		    cam->v4ldev->minor);
		return -EIO;
	}

	s->pix_format.width = rect->width/scale;
	s->pix_format.height = rect->height/scale;
	memcpy(&(s->_rect), rect, sizeof(*rect));

	if ((cam->module_param.force_munmap || cam->io == IO_READ) &&
	    nbuffers != sn9c102_request_buffers(cam, nbuffers, cam->io)) {
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "VIDIOC_S_CROP failed because of not enough memory. To "
		       "use the camera, close and open /dev/video%d again.",
		    cam->v4ldev->minor);
		return -ENOMEM;
	}

	if (cam->io == IO_READ)
		sn9c102_empty_framequeues(cam);
	else if (cam->module_param.force_munmap)
		sn9c102_requeue_outqueue(cam);

	cam->stream = stream;

	return 0;
}




static int
//sn9c102_vidioc_enum_framesizes(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_enum_framesizes(struct file *filp, void *fh, struct v4l2_frmsizeenum *fsize)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, fsize, sizeof(frmsize)))
		return -EFAULT;

	if (frmsize.index != 0)
		return -EINVAL;

	switch (cam->bridge) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
	case BRIDGE_SN9C103:
		if (frmsize.pixel_format != V4L2_PIX_FMT_SN9C10X &&
		    frmsize.pixel_format != V4L2_PIX_FMT_SBGGR8)
			return -EINVAL;
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		if (frmsize.pixel_format != V4L2_PIX_FMT_JPEG &&
		    frmsize.pixel_format != V4L2_PIX_FMT_SBGGR8)
			return -EINVAL;
		break;
	}

	frmsize.type = V4L2_FRMSIZE_TYPE_STEPWISE;
	frmsize.stepwise.min_width = frmsize.stepwise.step_width = 48;
	frmsize.stepwise.min_height = frmsize.stepwise.step_height = 48;
	frmsize.stepwise.max_width = cam->sensor.cropcap.bounds.width;
	frmsize.stepwise.max_height = cam->sensor.cropcap.bounds.height;
	memset(&frmsize.reserved, 0, sizeof(frmsize.reserved));

	if (copy_to_user(fsize, &frmsize, sizeof(frmsize)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_enum_fmt(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_enum_fmt(struct file *filp, void *fh, struct v4l2_fmtdesc *f)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_fmtdesc fmtd;

	if (copy_from_user(&fmtd, f, sizeof(fmtd)))
		return -EFAULT;

	if (fmtd.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (fmtd.index == 0) {
		strcpy(fmtd.description, "bayer rgb");
		fmtd.pixelformat = V4L2_PIX_FMT_SBGGR8;
		fmtd.flags = 0;
	} else if (fmtd.index == 1) {
		switch (cam->bridge) {
		case BRIDGE_SN9C101:
		case BRIDGE_SN9C102:
		case BRIDGE_SN9C103:
			strcpy(fmtd.description, "compressed");
			fmtd.pixelformat = V4L2_PIX_FMT_SN9C10X;
			break;
		case BRIDGE_SN9C105:
		case BRIDGE_SN9C120:
			strcpy(fmtd.description, "JPEG");
			fmtd.pixelformat = V4L2_PIX_FMT_JPEG;
			break;
		}
		fmtd.flags = V4L2_FMT_FLAG_COMPRESSED;
	} else
		return -EINVAL;

	fmtd.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	memset(&fmtd.reserved, 0, sizeof(fmtd.reserved));

	if (copy_to_user(f, &fmtd, sizeof(fmtd)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_g_fmt(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_g_fmt(struct file *filp, void *fh, struct v4l2_format *f)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_format format;
	struct v4l2_pix_format* pfmt = &(cam->sensor.pix_format);

	if (copy_from_user(&format, f, sizeof(format)))
		return -EFAULT;

	if (format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pfmt->colorspace = (pfmt->pixelformat == V4L2_PIX_FMT_JPEG) ?
	                   V4L2_COLORSPACE_JPEG : V4L2_COLORSPACE_SRGB;
	pfmt->bytesperline = (pfmt->pixelformat == V4L2_PIX_FMT_SN9C10X ||
	                      pfmt->pixelformat == V4L2_PIX_FMT_JPEG)
	                     ? 0 : (pfmt->width * pfmt->priv) / 8;
	pfmt->sizeimage = pfmt->height * ((pfmt->width*pfmt->priv)/8);
	pfmt->field = V4L2_FIELD_NONE;
	memcpy(&(format.fmt.pix), pfmt, sizeof(*pfmt));

	if (copy_to_user(f, &format, sizeof(format)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_try_s_fmt(struct sn9c102_device* cam, unsigned int cmd, void __user * arg)
sn9c102_vidioc_try_s_fmt(struct file *filp, void *fh, struct v4l2_format *f)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
    unsigned int cmd = *((unsigned int*)fh);


	struct sn9c102_sensor* s = &cam->sensor;
	struct v4l2_format format;
	struct v4l2_pix_format* pix;
	struct v4l2_pix_format* pfmt = &(s->pix_format);
	struct v4l2_rect* bounds = &(s->cropcap.bounds);
	struct v4l2_rect rect;
	u8 scale;
	const enum sn9c102_stream_state stream = cam->stream;
	const u32 nbuffers = cam->nbuffers;
	u32 i;
	int err = 0;

	if (copy_from_user(&format, f, sizeof(format)))
		return -EFAULT;

	pix = &(format.fmt.pix);

	if (format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memcpy(&rect, &(s->_rect), sizeof(rect));

	{ /* calculate the actual scaling factor */
		u32 a, b;
		a = rect.width * rect.height;
		b = pix->width * pix->height;
		scale = b ? (u8)((a / b) < 4 ? 1 : ((a / b) < 16 ? 2 : 4)) : 1;
	}

	rect.width = scale * pix->width;
	rect.height = scale * pix->height;

	if (rect.width < 48)
		rect.width = 48;
	if (rect.height < 48)
		rect.height = 48;
	if (rect.width > bounds->left + bounds->width - rect.left)
		rect.width = bounds->left + bounds->width - rect.left;
	if (rect.height > bounds->top + bounds->height - rect.top)
		rect.height = bounds->top + bounds->height - rect.top;

	rect.width &= ~15L;
	rect.height &= ~15L;

	{ /* adjust the scaling factor */
		u32 a, b;
		a = rect.width * rect.height;
		b = pix->width * pix->height;
		scale = b ? (u8)((a / b) < 4 ? 1 : ((a / b) < 16 ? 2 : 4)) : 1;
	}

	pix->width = rect.width / scale;
	pix->height = rect.height / scale;

	switch (cam->bridge) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
	case BRIDGE_SN9C103:
		if (pix->pixelformat != V4L2_PIX_FMT_SN9C10X &&
		    pix->pixelformat != V4L2_PIX_FMT_SBGGR8)
			pix->pixelformat = pfmt->pixelformat;
		break;
	case BRIDGE_SN9C105:
	case BRIDGE_SN9C120:
		if (pix->pixelformat != V4L2_PIX_FMT_JPEG &&
		    pix->pixelformat != V4L2_PIX_FMT_SBGGR8)
			pix->pixelformat = pfmt->pixelformat;
		break;
	}
	pix->priv = pfmt->priv; /* bpp */
	pix->colorspace = (pix->pixelformat == V4L2_PIX_FMT_JPEG) ?
	                  V4L2_COLORSPACE_JPEG : V4L2_COLORSPACE_SRGB;
	pix->bytesperline = (pix->pixelformat == V4L2_PIX_FMT_SN9C10X ||
	                     pix->pixelformat == V4L2_PIX_FMT_JPEG)
	                    ? 0 : (pix->width * pix->priv) / 8;
	pix->sizeimage = pix->height * ((pix->width * pix->priv) / 8);
	pix->field = V4L2_FIELD_NONE;

	if (cmd == VIDIOC_TRY_FMT) {
		if (copy_to_user(f, &format, sizeof(format)))
			return -EFAULT;
		return 0;
	}

	if (cam->module_param.force_munmap)
		for (i = 0; i < cam->nbuffers; i++)
			if (cam->frame[i].vma_use_count) {
				DBG(3, "VIDIOC_S_FMT failed. Unmap the "
				       "buffers first.");
				return -EBUSY;
			}

	if (cam->stream == STREAM_ON)
		if ((err = sn9c102_stream_interrupt(cam)))
			return err;

	if (copy_to_user(f, &format, sizeof(format))) {
		cam->stream = stream;
		return -EFAULT;
	}

	if (cam->module_param.force_munmap  || cam->io == IO_READ)
		sn9c102_release_buffers(cam);

	err += sn9c102_set_pix_format(cam, pix);
	err += sn9c102_set_crop(cam, &rect);
	if (s->set_pix_format)
		err += s->set_pix_format(cam, pix);
	if (s->set_crop)
		err += s->set_crop(cam, &rect);
	err += sn9c102_set_scale(cam, scale);

	if (err) { /* atomic, no rollback in ioctl() */
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "VIDIOC_S_FMT failed because of hardware problems. To "
		       "use the camera, close and open /dev/video%d again.",
		    cam->v4ldev->minor);
		return -EIO;
	}

	memcpy(pfmt, pix, sizeof(*pix));
	memcpy(&(s->_rect), &rect, sizeof(rect));

	if ((cam->module_param.force_munmap  || cam->io == IO_READ) &&
	    nbuffers != sn9c102_request_buffers(cam, nbuffers, cam->io)) {
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "VIDIOC_S_FMT failed because of not enough memory. To "
		       "use the camera, close and open /dev/video%d again.",
		    cam->v4ldev->minor);
		return -ENOMEM;
	}

	if (cam->io == IO_READ)
		sn9c102_empty_framequeues(cam);
	else if (cam->module_param.force_munmap)
		sn9c102_requeue_outqueue(cam);

	cam->stream = stream;

	return 0;
}


static int
//sn9c102_vidioc_g_jpegcomp(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_g_jpegcomp(struct file *filp, void *fh, struct v4l2_jpegcompression *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	if (copy_to_user(a, &cam->compression, sizeof(cam->compression)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_s_jpegcomp(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_s_jpegcomp(struct file *filp, void *fh, const struct v4l2_jpegcompression *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_jpegcompression jc;
	const enum sn9c102_stream_state stream = cam->stream;
	int err = 0;

	if (copy_from_user(&jc, a, sizeof(jc)))
		return -EFAULT;

	if (jc.quality != 0 && jc.quality != 1)
		return -EINVAL;

	if (cam->stream == STREAM_ON)
		if ((err = sn9c102_stream_interrupt(cam)))
			return err;

	err += sn9c102_set_compression(cam, &jc);
	if (err) { /* atomic, no rollback in ioctl() */
		cam->state |= DEV_MISCONFIGURED;
		DBG(1, "VIDIOC_S_JPEGCOMP failed because of hardware "
		       "problems. To use the camera, close and open "
		       "/dev/video%d again.", cam->v4ldev->minor);
		return -EIO;
	}

	cam->compression.quality = jc.quality;

	cam->stream = stream;

	return 0;
}


static int
//sn9c102_vidioc_reqbufs(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_reqbufs(struct file *filp, void *fh, struct v4l2_requestbuffers *b)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_requestbuffers rb;
	u32 i;
	int err;

	if (copy_from_user(&rb, b, sizeof(rb)))
		return -EFAULT;

	if (rb.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    rb.memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	if (cam->io == IO_READ) {
		DBG(3, "Close and open the device again to choose the mmap "
		       "I/O method");
		return -EBUSY;
	}

	for (i = 0; i < cam->nbuffers; i++)
		if (cam->frame[i].vma_use_count) {
			DBG(3, "VIDIOC_REQBUFS failed. Previous buffers are "
			       "still mapped.");
			return -EBUSY;
		}

	if (cam->stream == STREAM_ON)
		if ((err = sn9c102_stream_interrupt(cam)))
			return err;

	sn9c102_empty_framequeues(cam);

	sn9c102_release_buffers(cam);
	if (rb.count)
		rb.count = sn9c102_request_buffers(cam, rb.count, IO_MMAP);

	if (copy_to_user(b, &rb, sizeof(rb))) {
		sn9c102_release_buffers(cam);
		cam->io = IO_NONE;
		cam->priority = NULL;
		return -EFAULT;
	}

	if (rb.count)
		cam->io = IO_MMAP;
	else {
		cam->io = IO_NONE;
		cam->priority = NULL;
	}

	return 0;
}


static int
//sn9c102_vidioc_querybuf(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_querybuf(struct file *filp, void *fh, struct v4l2_buffer *b)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_buffer xb;

	if (copy_from_user(&xb, b, sizeof(xb)))
		return -EFAULT;

	if (xb.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    xb.index >= cam->nbuffers || cam->io != IO_MMAP)
		return -EINVAL;

	memcpy(&xb, &cam->frame[xb.index].buf, sizeof(xb));

	if (cam->frame[xb.index].vma_use_count)
		xb.flags |= V4L2_BUF_FLAG_MAPPED;

	if (cam->frame[xb.index].state == F_DONE)
		xb.flags |= V4L2_BUF_FLAG_DONE;
	else if (cam->frame[xb.index].state != F_UNUSED)
		xb.flags |= V4L2_BUF_FLAG_QUEUED;

	if (copy_to_user(b, &xb, sizeof(xb)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_qbuf(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_qbuf(struct file *filp, void *fh, struct v4l2_buffer *b)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));


	struct v4l2_buffer xb;
	unsigned long lock_flags;

	if (copy_from_user(&xb, b, sizeof(xb)))
		return -EFAULT;

	if (xb.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    xb.index >= cam->nbuffers || cam->io != IO_MMAP)
		return -EINVAL;

	if (cam->frame[xb.index].state != F_UNUSED)
		return -EINVAL;

	cam->frame[xb.index].state = F_QUEUED;

	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	list_add_tail(&cam->frame[xb.index].frame, &cam->inqueue);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	PDBGG("Frame #%lu queued", (unsigned long)xb.index);

	return 0;
}


static int
//sn9c102_vidioc_dqbuf(struct sn9c102_device* cam, struct file* filp, void __user * arg)
sn9c102_vidioc_dqbuf(struct file *filp, void *fh, struct v4l2_buffer *b)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_buffer xb;
	struct sn9c102_frame_t *f;
	unsigned long lock_flags;
	long timeout;
	int err = 0;

	if (copy_from_user(&xb, b, sizeof(xb)))
		return -EFAULT;

	if (xb.type != V4L2_BUF_TYPE_VIDEO_CAPTURE || cam->io != IO_MMAP)
		return -EINVAL;

	if (list_empty(&cam->outqueue)) {
		if (cam->stream == STREAM_OFF)
			return -EINVAL;
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (!cam->module_param.frame_timeout) {
			err = wait_event_interruptible
			      ( cam->wait_frame,
			        (!list_empty(&cam->outqueue)) ||
			        (cam->state & DEV_DISCONNECTED) ||
			        (cam->state & DEV_MISCONFIGURED) );
			if (err)
				return err;
		} else {
			timeout = wait_event_interruptible_timeout
			          ( cam->wait_frame,
			            (!list_empty(&cam->outqueue)) ||
			            (cam->state & DEV_DISCONNECTED) ||
			            (cam->state & DEV_MISCONFIGURED),
			            cam->module_param.frame_timeout *
			            1000 * msecs_to_jiffies(1) );
			if (timeout < 0)
				return timeout;
			else if (timeout == 0 &&
			         !(cam->state & DEV_DISCONNECTED)) {
				DBG(1, "Video frame timeout elapsed");
				return -EIO;
			}
		}
		if (cam->state & DEV_DISCONNECTED)
			return -ENODEV;
		if (cam->state & DEV_MISCONFIGURED)
			return -EIO;
	}

	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	f = list_entry(cam->outqueue.next, struct sn9c102_frame_t, frame);
	list_del(cam->outqueue.next);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	f->state = F_UNUSED;

	memcpy(&xb, &f->buf, sizeof(xb));
	if (f->vma_use_count)
		xb.flags |= V4L2_BUF_FLAG_MAPPED;

	if (copy_to_user(b, &xb, sizeof(xb)))
		return -EFAULT;

	PDBGG("Frame #%lu dequeued", (unsigned long)f->buf.index);

	return 0;
}


static int
//sn9c102_vidioc_streamon(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_streamon(struct file *filp, void *fh, enum v4l2_buf_type i)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	int type;

	if (copy_from_user(&type, &i, sizeof(type)))
		return -EFAULT;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE || cam->io != IO_MMAP)
		return -EINVAL;

	cam->stream = STREAM_ON;

	DBG(3, "Stream on");

	return 0;
}


static int
//sn9c102_vidioc_streamoff(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_streamoff(struct file *filp, void *fh, enum v4l2_buf_type i)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	int type, err;

	if (copy_from_user(&type, &i, sizeof(type)))
		return -EFAULT;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE || cam->io != IO_MMAP)
		return -EINVAL;

	if (cam->stream == STREAM_ON)
		if ((err = sn9c102_stream_interrupt(cam)))
			return err;

	sn9c102_empty_framequeues(cam);

	DBG(3, "Stream off");

	return 0;
}


static int
//sn9c102_vidioc_g_parm(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_g_parm(struct file *filp, void *fh, struct v4l2_streamparm *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_streamparm sp;

	if (copy_from_user(&sp, a, sizeof(sp)))
		return -EFAULT;

	if (sp.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	sp.parm.capture.extendedmode = 0;
	sp.parm.capture.readbuffers = cam->nreadbuffers;

	if (copy_to_user(a, &sp, sizeof(sp)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_s_parm(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_s_parm(struct file *filp, void *fh, struct v4l2_streamparm *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));

	struct v4l2_streamparm sp;

	if (copy_from_user(&sp, a, sizeof(sp)))
		return -EFAULT;

	if (sp.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	sp.parm.capture.extendedmode = 0;

	if (sp.parm.capture.readbuffers <= 1)
		sp.parm.capture.readbuffers = cam->nreadbuffers;

	if (sp.parm.capture.readbuffers > SN9C102_MAX_FRAMES)
		sp.parm.capture.readbuffers = SN9C102_MAX_FRAMES;

	if (copy_to_user(a, &sp, sizeof(sp)))
		return -EFAULT;

	cam->nreadbuffers = sp.parm.capture.readbuffers;

	return 0;
}


static int
//sn9c102_vidioc_enumaudio(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_enumaudio(struct file *filp, void *fh, struct v4l2_audio *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
    void __user * arg;

	struct v4l2_audio audio;

	if (cam->bridge != BRIDGE_SN9C103 && cam->bridge != BRIDGE_SN9C105)
		return -EINVAL;

	if (copy_from_user(&audio, arg, sizeof(audio)))
		return -EFAULT;

	if (audio.index != 0)
		return -EINVAL;

	strcpy(audio.name, "Microphone");
	audio.capability = 0;
	audio.mode = 0;

	if (copy_to_user(arg, &audio, sizeof(audio)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_g_audio(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_g_audio(struct file *filp, void *fh, struct v4l2_audio *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
    void __user * arg;

	struct v4l2_audio audio;

	if (cam->bridge != BRIDGE_SN9C103 && cam->bridge != BRIDGE_SN9C105)
		return -EINVAL;

	if (copy_from_user(&audio, arg, sizeof(audio)))
		return -EFAULT;

	memset(&audio, 0, sizeof(audio));
	strcpy(audio.name, "Microphone");

	if (copy_to_user(arg, &audio, sizeof(audio)))
		return -EFAULT;

	return 0;
}


static int
//sn9c102_vidioc_s_audio(struct sn9c102_device* cam, void __user * arg)
sn9c102_vidioc_s_audio(struct file *filp, void *fh, const struct v4l2_audio *a)
{
    struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
    void __user * arg;

	struct v4l2_audio audio;

	if (cam->bridge != BRIDGE_SN9C103 && cam->bridge != BRIDGE_SN9C105)
		return -EINVAL;

	if (copy_from_user(&audio, arg, sizeof(audio)))
		return -EFAULT;

	if (audio.index != 0)
		return -EINVAL;

	return 0;
}


const struct v4l2_ioctl_ops ioctl_ops = {
    .vidioc_querycap = sn9c102_vidioc_querycap,
    .vidioc_enum_fmt_vid_cap = sn9c102_vidioc_enum_fmt,
    .vidioc_enum_fmt_vid_overlay = 0,
    .vidioc_enum_fmt_vid_out = 0,
    .vidioc_enum_fmt_sdr_cap = 0,
    .vidioc_enum_fmt_sdr_out = 0,
    .vidioc_enum_fmt_meta_cap = 0,
    .vidioc_enum_fmt_meta_out = 0,
    .vidioc_g_fmt_vid_cap = sn9c102_vidioc_g_fmt,
    .vidioc_g_fmt_vid_overlay = 0,
    .vidioc_g_fmt_vid_out = 0,
    .vidioc_g_fmt_vid_out_overlay = 0,
    .vidioc_g_fmt_vbi_cap = 0,
    .vidioc_g_fmt_vbi_out = 0,
    .vidioc_g_fmt_sliced_vbi_cap = 0,
    .vidioc_g_fmt_sliced_vbi_out = 0,
    .vidioc_g_fmt_vid_cap_mplane = 0,
    .vidioc_g_fmt_vid_out_mplane = 0,
    .vidioc_g_fmt_sdr_cap = 0,
    .vidioc_g_fmt_sdr_out = 0,
    .vidioc_g_fmt_meta_cap = 0,
    .vidioc_g_fmt_meta_out = 0,
    .vidioc_s_fmt_vid_cap = sn9c102_vidioc_try_s_fmt,
    .vidioc_s_fmt_vid_overlay = 0,
    .vidioc_s_fmt_vid_out = 0,
    .vidioc_s_fmt_vid_out_overlay = 0,
    .vidioc_s_fmt_vbi_cap = 0,
    .vidioc_s_fmt_vbi_out = 0,
    .vidioc_s_fmt_sliced_vbi_cap = 0,
    .vidioc_s_fmt_sliced_vbi_out = 0,
    .vidioc_s_fmt_vid_cap_mplane = 0,
    .vidioc_s_fmt_vid_out_mplane = 0,
    .vidioc_s_fmt_sdr_cap = 0,
    .vidioc_s_fmt_sdr_out = 0,
    .vidioc_s_fmt_meta_cap = 0,
    .vidioc_s_fmt_meta_out = 0,
    .vidioc_try_fmt_vid_cap = sn9c102_vidioc_try_s_fmt,
    .vidioc_try_fmt_vid_overlay = 0,
    .vidioc_try_fmt_vid_out = 0,
    .vidioc_try_fmt_vid_out_overlay = 0,
    .vidioc_try_fmt_vbi_cap = 0,
    .vidioc_try_fmt_vbi_out = 0,
    .vidioc_try_fmt_sliced_vbi_cap = 0,
    .vidioc_try_fmt_sliced_vbi_out = 0,
    .vidioc_try_fmt_vid_cap_mplane = 0,
    .vidioc_try_fmt_vid_out_mplane = 0,
    .vidioc_try_fmt_sdr_cap = 0,
    .vidioc_try_fmt_sdr_out = 0,
    .vidioc_try_fmt_meta_cap = 0,
    .vidioc_try_fmt_meta_out = 0,
    .vidioc_reqbufs = sn9c102_vidioc_reqbufs,
    .vidioc_querybuf = sn9c102_vidioc_querybuf,
    .vidioc_qbuf = sn9c102_vidioc_qbuf,
    .vidioc_expbuf = 0,
    .vidioc_dqbuf = sn9c102_vidioc_dqbuf,
    .vidioc_create_bufs = 0,
    .vidioc_prepare_buf = 0,
    .vidioc_overlay = 0,
    .vidioc_g_fbuf = 0,
    .vidioc_s_fbuf = 0,
    .vidioc_streamon = sn9c102_vidioc_streamon,
    .vidioc_streamoff = sn9c102_vidioc_streamoff,
    .vidioc_g_std = 0,
    .vidioc_s_std = 0,
    .vidioc_querystd = 0,
    .vidioc_enum_input = sn9c102_vidioc_enuminput,
    .vidioc_g_input = sn9c102_vidioc_g_input,
    .vidioc_s_input = sn9c102_vidioc_s_input,
    .vidioc_enum_output = 0,
    .vidioc_g_output = 0,
    .vidioc_s_output = 0,
    .vidioc_queryctrl = sn9c102_vidioc_query_ctrl,
    .vidioc_query_ext_ctrl = 0,
    .vidioc_g_ctrl = sn9c102_vidioc_g_ctrl,
    .vidioc_s_ctrl = sn9c102_vidioc_s_ctrl,
    .vidioc_g_ext_ctrls = 0,
    .vidioc_s_ext_ctrls = 0,
    .vidioc_try_ext_ctrls = 0,
    .vidioc_querymenu = 0,
    .vidioc_enumaudio = sn9c102_vidioc_enumaudio,
    .vidioc_g_audio = sn9c102_vidioc_g_audio,
    .vidioc_s_audio = sn9c102_vidioc_s_audio,
    .vidioc_enumaudout = 0,
    .vidioc_g_audout = 0,
    .vidioc_s_audout = 0,
    .vidioc_g_modulator = 0,
    .vidioc_s_modulator = 0,
    .vidioc_g_pixelaspect = sn9c102_vidioc_cropcap,  /* verify */
    .vidioc_g_selection = 0,
    .vidioc_s_selection = 0,
    .vidioc_g_jpegcomp = sn9c102_vidioc_g_jpegcomp,
    .vidioc_s_jpegcomp = sn9c102_vidioc_s_jpegcomp,
    .vidioc_g_enc_index = 0,
    .vidioc_encoder_cmd = 0,
    .vidioc_try_encoder_cmd = 0,
    .vidioc_decoder_cmd = 0,
    .vidioc_try_decoder_cmd = 0,
    .vidioc_g_parm = sn9c102_vidioc_g_parm,
    .vidioc_s_parm = sn9c102_vidioc_s_parm,
    .vidioc_g_tuner = 0,
    .vidioc_s_tuner = 0,
    .vidioc_g_frequency = 0,
    .vidioc_s_frequency = 0,
    .vidioc_enum_freq_bands = 0,
    .vidioc_g_sliced_vbi_cap = 0,
    .vidioc_log_status = 0,
    .vidioc_s_hw_freq_seek = 0,
#ifdef CONFIG_VIDEO_ADV_DEBUG
    .vidioc_g_register = 0,
    .vidioc_s_register = 0,
    .vidioc_g_chip_info = 0,
#endif
    .vidioc_enum_framesizes = sn9c102_vidioc_enum_framesizes,
    .vidioc_enum_frameintervals = 0,
    .vidioc_s_dv_timings = 0,
    .vidioc_g_dv_timings = 0,
    .vidioc_query_dv_timings = 0,
    .vidioc_enum_dv_timings = 0,
    .vidioc_dv_timings_cap = 0,
    .vidioc_g_edid = 0,
    .vidioc_s_edid = 0,
    .vidioc_subscribe_event = 0,
    .vidioc_unsubscribe_event = 0,
    .vidioc_default = 0
};
















/* static int sn9c102_ioctl_v4l2(struct inode* inode, struct file* filp, - josemar */

static int sn9c102_ioctl_v4l2(struct file* filp, unsigned int cmd, void __user * arg)
{
	//struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp)); - remover


	switch (cmd) {

	case VIDIOC_QUERYCAP:
		return sn9c102_vidioc_querycap(filp, NULL, arg);

	case VIDIOC_ENUMINPUT:
		return sn9c102_vidioc_enuminput(filp, NULL, arg);

	case VIDIOC_G_INPUT:
		return sn9c102_vidioc_g_input(filp, NULL, arg);

	case VIDIOC_S_INPUT:
		return sn9c102_vidioc_s_input(filp, NULL, (unsigned long)arg);

	case VIDIOC_QUERYCTRL:
		return sn9c102_vidioc_query_ctrl(filp, NULL, arg);

	case VIDIOC_G_CTRL:
		return sn9c102_vidioc_g_ctrl(filp, NULL, arg);

	case VIDIOC_S_CTRL:
		return sn9c102_vidioc_s_ctrl(filp, NULL, arg);

	case VIDIOC_CROPCAP:
		return sn9c102_vidioc_cropcap(filp, NULL, 0, arg);

	case VIDIOC_G_CROP:
		return sn9c102_vidioc_g_crop(filp, NULL, arg);	// verify josemar

	case VIDIOC_S_CROP:
		return sn9c102_vidioc_s_crop(filp, NULL, arg);	// verify josemar

	case VIDIOC_ENUM_FRAMESIZES:
		return sn9c102_vidioc_enum_framesizes(filp, NULL, arg);

	case VIDIOC_ENUM_FMT:
		return sn9c102_vidioc_enum_fmt(filp, NULL, arg);

	case VIDIOC_G_FMT:
		return sn9c102_vidioc_g_fmt(filp, NULL, arg);

	case VIDIOC_TRY_FMT:
	case VIDIOC_S_FMT:
		return sn9c102_vidioc_try_s_fmt(filp, NULL, arg);

	case VIDIOC_G_JPEGCOMP:
		return sn9c102_vidioc_g_jpegcomp(filp, NULL, arg);

	case VIDIOC_S_JPEGCOMP:
		return sn9c102_vidioc_s_jpegcomp(filp, NULL, arg);

	case VIDIOC_REQBUFS:
		return sn9c102_vidioc_reqbufs(filp, NULL, arg);

	case VIDIOC_QUERYBUF:
		return sn9c102_vidioc_querybuf(filp, NULL, arg);

	case VIDIOC_QBUF:
		return sn9c102_vidioc_qbuf(filp, NULL, arg);

	case VIDIOC_DQBUF:
		return sn9c102_vidioc_dqbuf(filp, NULL, arg);

	case VIDIOC_STREAMON:
		return sn9c102_vidioc_streamon(filp, NULL, (enum v4l2_buf_type)arg);

	case VIDIOC_STREAMOFF:
		return sn9c102_vidioc_streamoff(filp, NULL, (enum v4l2_buf_type)arg);

	case VIDIOC_G_PARM:
		return sn9c102_vidioc_g_parm(filp, NULL, arg);

	case VIDIOC_S_PARM:
		return sn9c102_vidioc_s_parm(filp, NULL, arg);

	case VIDIOC_ENUMAUDIO:
		return sn9c102_vidioc_enumaudio(filp, NULL, arg);

	case VIDIOC_G_AUDIO:
		return sn9c102_vidioc_g_audio(filp, NULL, arg);

	case VIDIOC_S_AUDIO:
		return sn9c102_vidioc_s_audio(filp, NULL, arg);

	case VIDIOC_G_STD:
	case VIDIOC_S_STD:
	case VIDIOC_QUERYSTD:
	case VIDIOC_ENUMSTD:
	case VIDIOC_QUERYMENU:
	case VIDIOC_ENUM_FRAMEINTERVALS:
		return -EINVAL;

	default:
		return -EINVAL;

	}

    return 0;

}




/*static int sn9c102_ioctl(struct inode* inode, struct file* filp, - josemar */

static long int sn9c102_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	struct sn9c102_device* cam = video_get_drvdata(video_devdata(filp));
	int err = 0;

	if (mutex_lock_interruptible(&cam->fileop_mutex))
		return -ERESTARTSYS;

	if (cam->state & DEV_DISCONNECTED) {
		DBG(1, "Device not present");
		mutex_unlock(&cam->fileop_mutex);
		return -ENODEV;
	}

	if (cam->state & DEV_MISCONFIGURED) {
		DBG(1, "The camera is misconfigured. Close and open it "
		       "again.");
		mutex_unlock(&cam->fileop_mutex);
		return -EIO;
	}

	V4LDBG(3, "sn9c102", cmd);

	switch (cmd) {
	case VIDIOC_S_CROP:
	case VIDIOC_S_FMT:
	case VIDIOC_REQBUFS:
		if (cam->priority && cam->priority != filp) {
			DBG(2, "Device /dev/video%d is already in use for "
			       "streaming", cam->v4ldev->minor);
			err = -EBUSY;
		} else {
			// * err = sn9c102_ioctl_v4l2(inode, filp, cmd, - josemar
			err = sn9c102_ioctl_v4l2(filp, cmd, (void __user *)arg);
			if (!err)
				if (cmd != VIDIOC_REQBUFS ||
				    (cmd == VIDIOC_REQBUFS &&
				     cam->io == IO_MMAP))
					cam->priority = filp;
		}
		break;
	default:
		// * err = sn9c102_ioctl_v4l2(inode, filp, cmd, (void __user *)arg); - josemar
		err = sn9c102_ioctl_v4l2(filp, cmd, (void __user *)arg);
		break;
	}

	mutex_unlock(&cam->fileop_mutex);

	return err;
}


/*****************************************************************************/

/* static const struct file_operations sn9c102_fops = { - josemar */
static const struct v4l2_file_operations sn9c102_fops = {
	.owner = THIS_MODULE,
	.open = sn9c102_open,
	.release = sn9c102_release,
	.unlocked_ioctl = sn9c102_ioctl, //video_ioctl2,
	.read = sn9c102_read,
	.mmap = sn9c102_mmap,
	.poll = sn9c102_poll,

#ifdef CONFIG_COMPAT
    .compat_ioctl32 = 0,
#endif
	.get_unmapped_area = 0,
	.write = 0,

};

/*****************************************************************************/





/* It exists a single interface only. We do not need to validate anything. */
/* Called if device is plugged */
static int sn9c102_usb_probe(struct usb_interface* intf, const struct usb_device_id* id) {

	struct usb_device *udev = interface_to_usbdev(intf);
	struct sn9c102_device* cam;
	struct v4l2_device* v4dv = 0;
	static unsigned int dev_nr = 0;
	unsigned int i;
	int err = 0, r;

/// ******************
	if (!(cam = kzalloc(sizeof(struct sn9c102_device), GFP_KERNEL)))
		return -ENOMEM;


/// ******************
	if (!(cam->control_buffer = kzalloc(8, GFP_KERNEL))) {
		DBG(1, "kzalloc() failed");
		err = -ENOMEM;
		goto fail;
	}

/// ******************
	cam->usbdev = udev;



/// ******************
	if (!(v4dv = kzalloc(sizeof(struct v4l2_device), GFP_KERNEL) )) {
		DBG(1, "v4l2_dev failed");
		err = -ENOMEM;
		goto fail;
	}


/// ******************
	err = v4l2_device_register(&intf->dev, v4dv);
	if (err){
        DBG(1, "Problema com v4l2_device_register");
		goto fail;
    }else{

        DBG(1, "Passou por v4l2_device_register");

    }


/// ******************
	if (!(cam->v4ldev = video_device_alloc())) {
		DBG(1, "video_device_alloc() failed");
		err = -ENOMEM;
		goto fail;
	}


	r = sn9c102_read_reg(cam, 0x00);
	if (r < 0 || (r != 0x10 && r != 0x11 && r != 0x12)) {
		DBG(1, "Sorry, this is not a SN9C1xx-based camera "
		       "(vid:pid 0x%04X:0x%04X)", id->idVendor, id->idProduct);
		err = -ENODEV;
		goto fail;
	}

	cam->bridge = id->driver_info;
	switch (cam->bridge) {
	case BRIDGE_SN9C101:
	case BRIDGE_SN9C102:
		DBG(2, "SN9C10[12] PC Camera Controller detected "
		       "(vid:pid 0x%04X:0x%04X)", id->idVendor, id->idProduct);
		break;
	case BRIDGE_SN9C103:
		DBG(2, "SN9C103 PC Camera Controller detected "
		       "(vid:pid 0x%04X:0x%04X)", id->idVendor, id->idProduct);
		break;
	case BRIDGE_SN9C105:
		DBG(2, "SN9C105 PC Camera Controller detected "
		       "(vid:pid 0x%04X:0x%04X)", id->idVendor, id->idProduct);
		break;
	case BRIDGE_SN9C120:
		DBG(2, "SN9C120 PC Camera Controller detected "
		       "(vid:pid 0x%04X:0x%04X)", id->idVendor, id->idProduct);
		break;
	}

	for  (i = 0; i < ARRAY_SIZE(sn9c102_sensor_table); i++) {
		err = sn9c102_sensor_table[i](cam);
		if (!err)
			break;
	}

	if (!err) {
		DBG(2, "%s image sensor detected", cam->sensor.name);
		DBG(3, "Support for %s maintained by %s",
		    cam->sensor.name, cam->sensor.maintainer);
	} else {
		DBG(1, "No supported image sensor detected for this bridge");
		err = -ENODEV;
		goto fail;
	}

	if (!(cam->bridge & cam->sensor.supported_bridge)) {
		DBG(1, "Bridge not supported");
		err = -ENODEV;
		goto fail;
	}

	cam->module_param.force_munmap = force_munmap[dev_nr];
	cam->module_param.frame_timeout = frame_timeout[dev_nr];
	cam->module_param.max_opens = max_opens[dev_nr];

	if (sn9c102_init(cam)) {
		DBG(1, "Initialization failed");
		cam->state |= DEV_MISCONFIGURED;
	}




#if defined(CONFIG_MEDIA_CONTROLLER)


    //cam->v4ldev->entity = ;
    //cam->v4ldev->intf_devnode = ;
    //cam->v4ldev->pipe = ;

#endif

    strcpy(cam->v4ldev->name, "SN9C1xx PC Camera");
	cam->v4ldev->fops = &sn9c102_fops;
	cam->v4ldev->ioctl_ops = &ioctl_ops;
	cam->v4ldev->release = video_device_release;

	cam->v4ldev->v4l2_dev = v4dv;
	cam->v4ldev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;


	/* sets private data from struct video_device */
	video_set_drvdata(cam->v4ldev, cam);

    //cam->v4ldev->dev = udev->dev;
    // CREATED WHEN REGISTERING //cam->v4ldev->cdev = 0;

    cam->v4ldev->dev_parent = NULL;
    cam->v4ldev->ctrl_handler = NULL;
    //cam->v4ldev->queue = NULL;
    cam->v4ldev->prio = NULL;


    // CREATED WHEN REGISTERING //cam->v4ldev->vfl_type = VFL_TYPE_GRABBER;
    //cam->v4ldev->vfl_dir = 0;
	cam->v4ldev->minor = video_nr[dev_nr];
	// CREATED WHEN REGISTERING //cam->v4ldev->num = dev_nr;
	// CREATED WHEN REGISTERING //cam->v4ldev->flags = 0;
	// CREATED WHEN REGISTERING //cam->v4ldev->index = 0;

	// CREATED WHEN REGISTERING //memset(&cam->v4ldev->fh_lock, 0, sizeof(cam->v4ldev->fh_lock));
	// CREATED WHEN REGISTERING //memset(&cam->v4ldev->fh_list, 0, sizeof(cam->v4ldev->fh_list));

	//cam->v4ldev->dev_debug = 0;
	//cam->v4ldev->tvnorms = 0;


    //cam->v4ldev->lock =  0;


	init_completion(&cam->probe);

	err = video_register_device(cam->v4ldev, VFL_TYPE_GRABBER, -1 /*video_nr[dev_nr]*/);




	if (err) {
		DBG(1, "V4L2 device registration failed");
		if (err == -ENFILE && video_nr[dev_nr] == -1)
			DBG(1, "Free /dev/videoX node not found");
		video_nr[dev_nr] = -1;
		dev_nr = (dev_nr < SN9C102_MAX_DEVICES-1) ? dev_nr+1 : 0;
		complete_all(&cam->probe);
		goto fail;
	}

	DBG(2, "V4L2 device registered as /dev/video%d", cam->v4ldev->minor);



	dev_nr = (dev_nr < SN9C102_MAX_DEVICES-1) ? dev_nr+1 : 0;

	if (cam->state & DEV_MISCONFIGURED)
		DBG(2, "The initialization will retried on open()");

#if defined(CONFIG_VIDEO_ADV_DEBUG) && defined(SN9C102_ENABLE_SYSFS)
	err = sn9c102_create_sysfs(cam);
	if (!err)
		DBG(2, "Optional device control through 'sysfs' "
		       "interface ready");
	else
		DBG(2, "Failed to create optional 'sysfs' interface for "
		       "device controlling. Error #%d", err);
#else
	DBG(2, "Optional device control through 'sysfs' interface disabled");
	DBG(3, "Compile the kernel with the 'CONFIG_VIDEO_ADV_DEBUG' "
	       "configuration option to enable it.");
#endif

	usb_set_intfdata(intf, cam);
	kref_init(&cam->kref);
	usb_get_dev(cam->usbdev);

	complete_all(&cam->probe);

	return 0;

fail:
	if (cam) {
		kfree(cam->control_buffer);

		if (cam->v4ldev){
            if(cam->v4ldev->v4l2_dev){
                v4l2_device_unregister(cam->v4ldev->v4l2_dev);
                kfree(cam->v4ldev->v4l2_dev);
                v4dv = 0;
            }
            if(v4dv){
                kfree(v4dv);
                v4dv = 0;
            }
			video_device_release(cam->v4ldev);
        }
		kfree(cam);
	}
	return err;
}

/* Called when device is unplugged */
static void sn9c102_usb_disconnect(struct usb_interface* intf)
{
	struct sn9c102_device* cam;

	mutex_lock(&sn9c102_dev_lock);

	cam = usb_get_intfdata(intf);

	DBG(2, "Disconnecting %s...", cam->v4ldev->name);

	if (cam->users) {
		DBG(2, "Device /dev/video%d is in use! Deregistration and "
		       "memory deallocation are deferred.",
		    cam->v4ldev->minor);
		cam->state |= DEV_MISCONFIGURED;
		sn9c102_stop_transfer(cam);
		cam->state |= DEV_DISCONNECTED;
		wake_up_interruptible(&cam->wait_frame);
		wake_up(&cam->wait_stream);
	} else
		cam->state |= DEV_DISCONNECTED;

	wake_up_interruptible_all(&cam->wait_open);

	kref_put(&cam->kref, sn9c102_release_resources);

	mutex_unlock(&sn9c102_dev_lock);
}


static struct usb_driver sn9c102_usb_driver = {
	.name = "sn9c102",
	.probe = sn9c102_usb_probe,
	.disconnect = sn9c102_usb_disconnect,
	.id_table = sn9c102_id_table,
};

/*****************************************************************************/

/* Called when module is loaded */
static int __init sn9c102_module_init(void)
{
	int err = 0;

	KDBG(2, SN9C102_MODULE_NAME " v" SN9C102_MODULE_VERSION);
	KDBG(3, SN9C102_MODULE_AUTHOR);
	
	err = usb_register(&sn9c102_usb_driver);

	if (err < 0){
		
		KDBG(1, "usb_register() failed");
		return -1;

	}

	return 0;
}

/* Called when module is unloaded */
static void __exit sn9c102_module_exit(void)
{
	usb_deregister(&sn9c102_usb_driver);
}


module_init(sn9c102_module_init);
module_exit(sn9c102_module_exit);
