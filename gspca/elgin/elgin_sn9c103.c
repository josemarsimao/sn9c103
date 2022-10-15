#define MODULE_NAME "gspca_elgin_sn9c103"

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/io.h>
#include <asm/page.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>


#include <linux/input.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include <linux/mutex.h>


// Josemar:
// ********************************************************************************
#define DEBUG_103 1
#if DEBUG_103

    int elgcnt = 0;
    #define KDBG(func, fmt, args...) do { pr_info("Elgin_sn9c103: " func fmt "\n", ## args); elgcnt++; } while(0);

#else

    #define KDBG(func, fmt, args...)

#endif // DEBUG_103



/* GSPCA debug codes */

#define D_PROBE  1
#define D_CONF   2
#define D_STREAM 3
#define D_FRAM   4
#define D_PACK   5
#define D_USBI   6
#define D_USBO   7


int gspca_debug;

#define gspca_dbg(gspca_dev, level, fmt, ...)			\
	v4l2_dbg(level, gspca_debug, &(gspca_dev)->v4l2_dev,	\
		 fmt, ##__VA_ARGS__)

#define gspca_err(gspca_dev, fmt, ...)				\
	v4l2_err(&(gspca_dev)->v4l2_dev, fmt, ##__VA_ARGS__)

#define GSPCA_MAX_FRAMES 16	/* maximum number of video frame buffers */
/* image transfers */
#define MAX_NURBS 4		/* max number of URBs */


/* used to list framerates supported by a camera mode (resolution) */
struct framerates {
	const u8 *rates;
	int nrates;
};


/* device information - set at probe time */
struct cam {
	const struct v4l2_pix_format *cam_mode;	/* size nmodes */
	const struct framerates *mode_framerates; /* must have size nmodes,
						   * just like cam_mode */
	u32 bulk_size;		/* buffer size when image transfer by bulk */
	u32 input_flags;	/* value for ENUM_INPUT status flags */
	u8 nmodes;		/* size of cam_mode */
	u8 no_urb_create;	/* don't create transfer URBs */
	u8 bulk_nurbs;		/* number of URBs in bulk mode
				 * - cannot be > MAX_NURBS
				 * - when 0 and bulk_size != 0 means
				 *   1 URB and submit done by subdriver */
	u8 bulk;		/* image transfer by 0:isoc / 1:bulk */
	u8 npkt;		/* number of packets in an ISOC message
				 * 0 is the default value: 32 packets */
	u8 needs_full_bandwidth;/* Set this flag to notify the bandwidth calc.
				 * code that the cam fills all image buffers to
				 * the max, even when using compression. */
};



struct gspca_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};


static inline struct gspca_buffer *to_gspca_buffer(struct vb2_buffer *vb2)
{
    KDBG("to_gspca_buffer"," - %d",elgcnt)

	return container_of(vb2, struct gspca_buffer, vb.vb2_buf);
}

struct gspca_dev {

	// Ambos
	struct video_device vdev;	/* !! must be the first item */

	struct module *module;		/* subdriver handling the device */
	struct v4l2_device v4l2_dev;

	// Ambos
	struct usb_device *dev;

#if IS_ENABLED(CONFIG_INPUT)
	struct input_dev *input_dev;
	char phys[64];			/* physical device path */
#endif

	struct cam cam;				/* device information */
	const struct sd_desc *sd_desc;		/* subdriver description */
	struct v4l2_ctrl_handler ctrl_handler;

	/* autogain and exposure or gain control cluster, these are global as
	   the autogain/exposure functions in autogain_functions.c use them */
	struct {
		struct v4l2_ctrl *autogain;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *gain;
		int exp_too_low_cnt, exp_too_high_cnt;
	};

#define USB_BUF_SZ 64
	__u8 *usb_buf;				/* buffer for USB exchanges */

	// Ambos
	struct urb *urb[MAX_NURBS];

#if IS_ENABLED(CONFIG_INPUT)
	struct urb *int_urb;
#endif

	u8 *image;				/* image being filled */
	u32 image_len;				/* current length of image */
	__u8 last_packet_type;
	__s8 empty_packet;		/* if (-1) don't check empty packets */
	bool streaming;

	__u8 curr_mode;			/* current camera mode */
	struct v4l2_pix_format pixfmt;	/* current mode parameters */
	__u32 sequence;			/* frame sequence number */

	struct vb2_queue queue;

	spinlock_t qlock;

	// Ambos
	struct list_head buf_list;

	wait_queue_head_t wq;		/* wait queue */

	// Ambos
	struct mutex usb_lock;		/* usb exchange protection */

	int usb_err;			/* USB error - protected by usb_lock */
	u16 pkt_size;			/* ISOC packet size */
#ifdef CONFIG_PM
	char frozen;			/* suspend - resume */
#endif
	bool present;
	char memory;			/* memory type (V4L2_MEMORY_xxx) */
	__u8 iface;			/* USB interface number */
	__u8 alt;			/* USB alternate setting */
	int xfer_ep;			/* USB transfer endpoint address */
	u8 audio;			/* presence of audio device */

	/* (*) These variables are proteced by both usb_lock and queue_lock,
	   that is any code setting them is holding *both*, which means that
	   any code getting them needs to hold at least one of them */
};



/* subdriver operations */
typedef int (*cam_op) (struct gspca_dev *);
typedef void (*cam_v_op) (struct gspca_dev *);
typedef int (*cam_cf_op) (struct gspca_dev *, const struct usb_device_id *);
typedef int (*cam_get_jpg_op) (struct gspca_dev *, struct v4l2_jpegcompression *);
typedef int (*cam_set_jpg_op) (struct gspca_dev *, const struct v4l2_jpegcompression *);
typedef int (*cam_get_reg_op) (struct gspca_dev *, struct v4l2_dbg_register *);
typedef int (*cam_set_reg_op) (struct gspca_dev *, const struct v4l2_dbg_register *);
typedef int (*cam_chip_info_op) (struct gspca_dev *, struct v4l2_dbg_chip_info *);
typedef void (*cam_streamparm_op) (struct gspca_dev *, struct v4l2_streamparm *);
typedef void (*cam_pkt_op) (struct gspca_dev *gspca_dev, u8 *data, int len);
typedef int (*cam_int_pkt_op) (struct gspca_dev *gspca_dev,	u8 *data, int len);
typedef void (*cam_format_op) (struct gspca_dev *gspca_dev,	struct v4l2_format *fmt);
typedef int (*cam_frmsize_op) (struct gspca_dev *gspca_dev,	struct v4l2_frmsizeenum *fsize);

/* subdriver description */
struct sd_desc {
/* information */
	const char *name;	/* sub-driver name */
/* mandatory operations */
	cam_cf_op config;	/* called on probe */
	cam_op init;		/* called on probe and resume */
	cam_op init_controls;	/* called on probe */
	cam_v_op probe_error;	/* called if probe failed, do cleanup here */
	cam_op start;		/* called on stream on after URBs creation */
	cam_pkt_op pkt_scan;
/* optional operations */
	cam_op isoc_init;	/* called on stream on before getting the EP */
	cam_op isoc_nego;	/* called when URB submit failed with NOSPC */
	cam_v_op stopN;		/* called on stream off - main alt */
	cam_v_op stop0;		/* called on stream off & disconnect - alt 0 */
	cam_v_op dq_callback;	/* called when a frame has been dequeued */
	cam_get_jpg_op get_jcomp;
	cam_set_jpg_op set_jcomp;
	cam_streamparm_op get_streamparm;
	cam_streamparm_op set_streamparm;
	cam_format_op try_fmt;
	cam_frmsize_op enum_framesizes;
#ifdef CONFIG_VIDEO_ADV_DEBUG
	cam_set_reg_op set_register;
	cam_get_reg_op get_register;
	cam_chip_info_op get_chip_info;
#endif
#if IS_ENABLED(CONFIG_INPUT)
	cam_int_pkt_op int_pkt_scan;
	/* other_input makes the gspca core create gspca_dev->input even when
	   int_pkt_scan is NULL, for cams with non interrupt driven buttons */
	u8 other_input;
#endif
};

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type {
	DISCARD_PACKET,
	FIRST_PACKET,
	INTER_PACKET,
	LAST_PACKET
};






/* auto gain and exposure algorithm based on the knee algorithm described here:
   http://ytse.tricolour.net/docs/LowLightOptimization.html

   Returns 0 if no changes were made, 1 if the gain and or exposure settings
   where changed. */
int gspca_expo_autogain(struct gspca_dev *gspca_dev, int avg_lum, int desired_avg_lum, int deadzone, int gain_knee, int exposure_knee)
{
    KDBG("gspca_expo_autogain"," - %d",elgcnt)

/*
	s32 gain, orig_gain, exposure, orig_exposure;
	int i, steps, retval = 0;

	if (v4l2_ctrl_g_ctrl(gspca_dev->autogain) == 0)
		return 0;

	orig_gain = gain = v4l2_ctrl_g_ctrl(gspca_dev->gain);
	orig_exposure = exposure = v4l2_ctrl_g_ctrl(gspca_dev->exposure);
*/
	/* If we are of a multiple of deadzone, do multiple steps to reach the
	   desired lumination fast (with the risc of a slight overshoot) */

/*
	steps = abs(desired_avg_lum - avg_lum) / deadzone;

	gspca_dbg(gspca_dev, D_FRAM, "autogain: lum: %d, desired: %d, steps: %d\n",
		  avg_lum, desired_avg_lum, steps);

	for (i = 0; i < steps; i++) {
		if (avg_lum > desired_avg_lum) {
			if (gain > gain_knee)
				gain--;
			else if (exposure > exposure_knee)
				exposure--;
			else if (gain > gspca_dev->gain->default_value)
				gain--;
			else if (exposure > gspca_dev->exposure->minimum)
				exposure--;
			else if (gain > gspca_dev->gain->minimum)
				gain--;
			else
				break;
		} else {
			if (gain < gspca_dev->gain->default_value)
				gain++;
			else if (exposure < exposure_knee)
				exposure++;
			else if (gain < gain_knee)
				gain++;
			else if (exposure < gspca_dev->exposure->maximum)
				exposure++;
			else if (gain < gspca_dev->gain->maximum)
				gain++;
			else
				break;
		}
	}

	if (gain != orig_gain) {
		v4l2_ctrl_s_ctrl(gspca_dev->gain, gain);
		retval = 1;
	}
	if (exposure != orig_exposure) {
		v4l2_ctrl_s_ctrl(gspca_dev->exposure, exposure);
		retval = 1;
	}

	if (retval)
		gspca_dbg(gspca_dev, D_FRAM, "autogain: changed gain: %d, expo: %d\n",
			  gain, exposure);
	return retval;
*/
/*
	int i, steps, retval = 0;
	s32 gain, orig_gain, exposure, orig_exposure;
	orig_gain = gain = v4l2_ctrl_g_ctrl(gspca_dev->gain);
	orig_exposure = exposure = v4l2_ctrl_g_ctrl(gspca_dev->exposure);



	gain = gain + (gspca_dev->gain->minimum + gspca_dev->gain->maximum)/2;
	exposure = exposure + (gspca_dev->exposure->minimum + gspca_dev->exposure->maximum)/2;



	if (gain != orig_gain) {
		v4l2_ctrl_s_ctrl(gspca_dev->gain, gain);
		retval = 1;
	}
	if (exposure != orig_exposure) {
		v4l2_ctrl_s_ctrl(gspca_dev->exposure, 210);
		retval = 1;
	}

	i = 20;

	steps = i;

	v4l2_ctrl_s_ctrl(gspca_dev->exposure, 10);
	retval = 1;

	return retval;
*/
	return 0;

}

/* Autogain + exposure algorithm for cameras with a coarse exposure control
   (usually this means we can only control the clockdiv to change exposure)
   As changing the clockdiv so that the fps drops from 30 to 15 fps for
   example, will lead to a huge exposure change (it effectively doubles),
   this algorithm normally tries to only adjust the gain (between 40 and
   80 %) and if that does not help, only then changes exposure. This leads
   to a much more stable image then using the knee algorithm which at
   certain points of the knee graph will only try to adjust exposure,
   which leads to oscillating as one exposure step is huge.

   Returns 0 if no changes were made, 1 if the gain and or exposure settings
   where changed. */
int gspca_coarse_grained_expo_autogain(struct gspca_dev *gspca_dev, int avg_lum, int desired_avg_lum, int deadzone)
{
    KDBG("gspca_coarse_grained_expo_autogain"," - %d",elgcnt)
/*

	s32 gain_low, gain_high, gain, orig_gain, exposure, orig_exposure;
	int steps, retval = 0;

	if (v4l2_ctrl_g_ctrl(gspca_dev->autogain) == 0)
		return 0;

	orig_gain = gain = v4l2_ctrl_g_ctrl(gspca_dev->gain);
	orig_exposure = exposure = v4l2_ctrl_g_ctrl(gspca_dev->exposure);

	gain_low  = (s32)(gspca_dev->gain->maximum - gspca_dev->gain->minimum) /
		    5 * 2 + gspca_dev->gain->minimum;
	gain_high = (s32)(gspca_dev->gain->maximum - gspca_dev->gain->minimum) /
		    5 * 4 + gspca_dev->gain->minimum;

	// If we are of a multiple of deadzone, do multiple steps to reach the	   desired lumination fast (with the risc of a slight overshoot)

	steps = (desired_avg_lum - avg_lum) / deadzone;

	gspca_dbg(gspca_dev, D_FRAM, "autogain: lum: %d, desired: %d, steps: %d\n",
		  avg_lum, desired_avg_lum, steps);

	if ((gain + steps) > gain_high &&
	    exposure < gspca_dev->exposure->maximum) {
		gain = gain_high;
		gspca_dev->exp_too_low_cnt++;
		gspca_dev->exp_too_high_cnt = 0;
	} else if ((gain + steps) < gain_low &&
		   exposure > gspca_dev->exposure->minimum) {
		gain = gain_low;
		gspca_dev->exp_too_high_cnt++;
		gspca_dev->exp_too_low_cnt = 0;
	} else {
		gain += steps;
		if (gain > gspca_dev->gain->maximum)
			gain = gspca_dev->gain->maximum;
		else if (gain < gspca_dev->gain->minimum)
			gain = gspca_dev->gain->minimum;
		gspca_dev->exp_too_high_cnt = 0;
		gspca_dev->exp_too_low_cnt = 0;
	}

	if (gspca_dev->exp_too_high_cnt > 3) {
		exposure--;
		gspca_dev->exp_too_high_cnt = 0;
	} else if (gspca_dev->exp_too_low_cnt > 3) {
		exposure++;
		gspca_dev->exp_too_low_cnt = 0;
	}

	if (gain != orig_gain) {
		v4l2_ctrl_s_ctrl(gspca_dev->gain, gain);
		retval = 1;
	}
	if (exposure != orig_exposure) {
		v4l2_ctrl_s_ctrl(gspca_dev->exposure, exposure);
		retval = 1;
	}

	if (retval)
		gspca_dbg(gspca_dev, D_FRAM, "autogain: changed gain: %d, expo: %d\n",
			  gain, exposure);
	return retval;
*/
    return 0;
}




/*
 * Main USB camera driver
 *
 * Copyright (C) 2008-2011 Jean-François Moine <http://moinejf.free.fr>
 *
 * Camera button input handling by Márton Németh
 * Copyright (C) 2009-2010 Márton Németh <nm127@freemail.hu>
 */

//#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#define GSPCA_VERSION	"2.14.0"



#if IS_ENABLED(CONFIG_INPUT)
#include <linux/input.h>
#include <linux/usb/input.h>
#endif

/* global values */
#define DEF_NURBS 3		/* default number of URBs */
#if DEF_NURBS > MAX_NURBS
#error "DEF_NURBS too big"
#endif


MODULE_AUTHOR("Jean-François Moine <http://moinejf.free.fr>");
MODULE_DESCRIPTION("GSPCA USB Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(GSPCA_VERSION);

int gspca_debug;


static void PDEBUG_MODE(struct gspca_dev *gspca_dev, int debug, char *txt, __u32 pixfmt, int w, int h)
{
    KDBG("PDEBUG_MODE"," - %d",elgcnt)

	if ((pixfmt >> 24) >= '0' && (pixfmt >> 24) <= 'z') {
		gspca_dbg(gspca_dev, debug, "%s %c%c%c%c %dx%d\n",
			  txt,
			  pixfmt & 0xff,
			  (pixfmt >> 8) & 0xff,
			  (pixfmt >> 16) & 0xff,
			  pixfmt >> 24,
			  w, h);
	} else {
		gspca_dbg(gspca_dev, debug, "%s 0x%08x %dx%d\n",
			  txt,
			  pixfmt,
			  w, h);
	}
}

/* specific memory types - !! should be different from V4L2_MEMORY_xxx */
#define GSPCA_MEMORY_NO 0	/* V4L2_MEMORY_xxx starts from 1 */
#define GSPCA_MEMORY_READ 7

/*
 * Input and interrupt endpoint handling functions
 */
#if IS_ENABLED(CONFIG_INPUT)
static void int_irq(struct urb *urb)
{
    KDBG("int_irq"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = (struct gspca_dev *) urb->context;
	int ret;

	ret = urb->status;
	switch (ret) {
	case 0:
		if (gspca_dev->sd_desc->int_pkt_scan(gspca_dev,
		    urb->transfer_buffer, urb->actual_length) < 0) {
			gspca_err(gspca_dev, "Unknown packet received\n");
		}
		break;

	case -ENOENT:
	case -ECONNRESET:
	case -ENODEV:
	case -ESHUTDOWN:
		/* Stop is requested either by software or hardware is gone,
		 * keep the ret value non-zero and don't resubmit later.
		 */
		break;

	default:
		gspca_err(gspca_dev, "URB error %i, resubmitting\n",
			  urb->status);
		urb->status = 0;
		ret = 0;
	}

	if (ret == 0) {
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret < 0)
			pr_err("Resubmit URB failed with error %i\n", ret);
	}
}

// DEVICE PLUGGEDED - 12
static int gspca_input_connect(struct gspca_dev *dev)
{
    KDBG("gspca_input_connect"," - %d",elgcnt)

	struct input_dev *input_dev;
	int err = 0;

	dev->input_dev = NULL;
	if (dev->sd_desc->int_pkt_scan || dev->sd_desc->other_input)  {
		input_dev = input_allocate_device();
		if (!input_dev)
			return -ENOMEM;

		usb_make_path(dev->dev, dev->phys, sizeof(dev->phys));
		strlcat(dev->phys, "/input0", sizeof(dev->phys));

		input_dev->name = dev->sd_desc->name;
		input_dev->phys = dev->phys;

		usb_to_input_id(dev->dev, &input_dev->id);

		input_dev->evbit[0] = BIT_MASK(EV_KEY);
		input_dev->keybit[BIT_WORD(KEY_CAMERA)] = BIT_MASK(KEY_CAMERA);
		input_dev->dev.parent = &dev->dev->dev;

		err = input_register_device(input_dev);
		if (err) {
			pr_err("Input device registration failed with error %i\n",
			       err);
			input_dev->dev.parent = NULL;
			input_free_device(input_dev);
		} else {
			dev->input_dev = input_dev;
		}
	}

	return err;
}

// DEVICE PLUGGEDED - 14
static int alloc_and_submit_int_urb(struct gspca_dev *gspca_dev, struct usb_endpoint_descriptor *ep)
{
    KDBG("alloc_and_submit_int_urb"," - %d",elgcnt)

	unsigned int buffer_len;
	int interval;
	struct urb *urb;
	struct usb_device *dev;
	void *buffer = NULL;
	int ret = -EINVAL;

	buffer_len = le16_to_cpu(ep->wMaxPacketSize);
	interval = ep->bInterval;
	gspca_dbg(gspca_dev, D_CONF, "found int in endpoint: 0x%x, buffer_len=%u, interval=%u\n",
		  ep->bEndpointAddress, buffer_len, interval);

	dev = gspca_dev->dev;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		ret = -ENOMEM;
		goto error;
	}

	buffer = usb_alloc_coherent(dev, buffer_len,
				GFP_KERNEL, &urb->transfer_dma);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_buffer;
	}
	usb_fill_int_urb(urb, dev,
		usb_rcvintpipe(dev, ep->bEndpointAddress),
		buffer, buffer_len,
		int_irq, (void *)gspca_dev, interval);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret < 0) {
		gspca_err(gspca_dev, "submit int URB failed with error %i\n",
			  ret);
		goto error_submit;
	}
	gspca_dev->int_urb = urb;
	return ret;

error_submit:
	usb_free_coherent(dev,
			  urb->transfer_buffer_length,
			  urb->transfer_buffer,
			  urb->transfer_dma);
error_buffer:
	usb_free_urb(urb);
error:
	return ret;
}

// DEVICE PLUGGEDED - 13
static void gspca_input_create_urb(struct gspca_dev *gspca_dev)
{
    KDBG("gspca_input_create_urb"," - %d",elgcnt)

	struct usb_interface *intf;
	struct usb_host_interface *intf_desc;
	struct usb_endpoint_descriptor *ep;
	int i;

	if (gspca_dev->sd_desc->int_pkt_scan)  {
		intf = usb_ifnum_to_if(gspca_dev->dev, gspca_dev->iface);
		intf_desc = intf->cur_altsetting;
		for (i = 0; i < intf_desc->desc.bNumEndpoints; i++) {
			ep = &intf_desc->endpoint[i].desc;
			if (usb_endpoint_dir_in(ep) &&
			    usb_endpoint_xfer_int(ep)) {

				alloc_and_submit_int_urb(gspca_dev, ep);
				break;
			}
		}
	}
}

static void gspca_input_destroy_urb(struct gspca_dev *gspca_dev)
{
    KDBG("gspca_input_destroy_urb"," - %d",elgcnt)

	struct urb *urb;

	urb = gspca_dev->int_urb;
	if (urb) {
		gspca_dev->int_urb = NULL;
		usb_kill_urb(urb);
		usb_free_coherent(gspca_dev->dev,
				  urb->transfer_buffer_length,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		usb_free_urb(urb);
	}
}
#else
static inline void gspca_input_destroy_urb(struct gspca_dev *gspca_dev)
{
}

static inline void gspca_input_create_urb(struct gspca_dev *gspca_dev)
{
}

static inline int gspca_input_connect(struct gspca_dev *dev)
{
	return 0;
}
#endif

/*
 * fill a video frame from an URB and resubmit
 */
static void fill_frame(struct gspca_dev *gspca_dev,	struct urb *urb)
{
    KDBG("fill_frame"," - %d",elgcnt)

	u8 *data;		/* address of data in the iso message */
	int i, len, st;
	cam_pkt_op pkt_scan;

	if (urb->status != 0) {
		if (urb->status == -ESHUTDOWN)
			return;		/* disconnection */
#ifdef CONFIG_PM
		if (gspca_dev->frozen)
			return;
#endif
		gspca_err(gspca_dev, "urb status: %d\n", urb->status);
		urb->status = 0;
		goto resubmit;
	}
	pkt_scan = gspca_dev->sd_desc->pkt_scan;
	for (i = 0; i < urb->number_of_packets; i++) {
		len = urb->iso_frame_desc[i].actual_length;

		/* check the packet status and length */
		st = urb->iso_frame_desc[i].status;
		if (st) {
			gspca_dbg(gspca_dev, D_PACK, "ISOC data error: [%d] len=%d, status=%d\n",
			       i, len, st);
			gspca_dev->last_packet_type = DISCARD_PACKET;
			continue;
		}
		if (len == 0) {
			if (gspca_dev->empty_packet == 0)
				gspca_dev->empty_packet = 1;
			continue;
		}

		/* let the packet be analyzed by the subdriver */
		gspca_dbg(gspca_dev, D_PACK, "packet [%d] o:%d l:%d\n",
			  i, urb->iso_frame_desc[i].offset, len);
		data = (u8 *) urb->transfer_buffer
					+ urb->iso_frame_desc[i].offset;
		pkt_scan(gspca_dev, data, len);
	}

resubmit:
	if (!gspca_dev->streaming)
		return;
	/* resubmit the URB */
	st = usb_submit_urb(urb, GFP_ATOMIC);
	if (st < 0)
		pr_err("usb_submit_urb() ret %d\n", st);
}

/*
 * ISOC message interrupt from the USB device
 *
 * Analyse each packet and call the subdriver for copy to the frame buffer.
 */
static void isoc_irq(struct urb *urb)
{
    KDBG("isoc_irq"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = (struct gspca_dev *) urb->context;

	gspca_dbg(gspca_dev, D_PACK, "isoc irq\n");
	if (!gspca_dev->streaming)
		return;
	fill_frame(gspca_dev, urb);
}

/*
 * bulk message interrupt from the USB device
 */
static void bulk_irq(struct urb *urb)
{
    KDBG("bulk_irq"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = (struct gspca_dev *) urb->context;
	int st;

	gspca_dbg(gspca_dev, D_PACK, "bulk irq\n");
	if (!gspca_dev->streaming)
		return;
	switch (urb->status) {
	case 0:
		break;
	case -ESHUTDOWN:
		return;		/* disconnection */
	default:
#ifdef CONFIG_PM
		if (gspca_dev->frozen)
			return;
#endif
		gspca_err(gspca_dev, "urb status: %d\n", urb->status);
		urb->status = 0;
		goto resubmit;
	}

	gspca_dbg(gspca_dev, D_PACK, "packet l:%d\n", urb->actual_length);
	gspca_dev->sd_desc->pkt_scan(gspca_dev,
				urb->transfer_buffer,
				urb->actual_length);

resubmit:
	if (!gspca_dev->streaming)
		return;
	/* resubmit the URB */
	if (gspca_dev->cam.bulk_nurbs != 0) {
		st = usb_submit_urb(urb, GFP_ATOMIC);
		if (st < 0)
			pr_err("usb_submit_urb() ret %d\n", st);
	}
}

/*
 * add data to the current frame
 *
 * This function is called by the subdrivers at interrupt level.
 *
 * To build a frame, these ones must add
 *	- one FIRST_PACKET
 *	- 0 or many INTER_PACKETs
 *	- one LAST_PACKET
 * DISCARD_PACKET invalidates the whole frame.
 */
void gspca_frame_add(struct gspca_dev *gspca_dev, enum gspca_packet_type packet_type, const u8 *data, int len)
{
    KDBG("gspca_frame_add"," - %d",elgcnt)

	struct gspca_buffer *buf;
	unsigned long flags;

	gspca_dbg(gspca_dev, D_PACK, "add t:%d l:%d\n",	packet_type, len);

	spin_lock_irqsave(&gspca_dev->qlock, flags);
	buf = list_first_entry_or_null(&gspca_dev->buf_list,
				       typeof(*buf), list);
	spin_unlock_irqrestore(&gspca_dev->qlock, flags);

	if (packet_type == FIRST_PACKET) {
		/* if there is no queued buffer, discard the whole frame */
		if (!buf) {
			gspca_dev->last_packet_type = DISCARD_PACKET;
			gspca_dev->sequence++;
			return;
		}
		gspca_dev->image = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
		gspca_dev->image_len = 0;
	} else {
		switch (gspca_dev->last_packet_type) {
		case DISCARD_PACKET:
			if (packet_type == LAST_PACKET) {
				gspca_dev->last_packet_type = packet_type;
				gspca_dev->image = NULL;
				gspca_dev->image_len = 0;
			}
			return;
		case LAST_PACKET:
			return;
		}
	}

	/* append the packet to the frame buffer */
	if (len > 0) {
		if (gspca_dev->image_len + len > PAGE_ALIGN(gspca_dev->pixfmt.sizeimage)) {
			gspca_err(gspca_dev, "frame overflow %d > %d\n",
				  gspca_dev->image_len + len,
				  PAGE_ALIGN(gspca_dev->pixfmt.sizeimage));
			packet_type = DISCARD_PACKET;
		} else {
/* !! image is NULL only when last pkt is LAST or DISCARD
			if (gspca_dev->image == NULL) {
				pr_err("gspca_frame_add() image == NULL\n");
				return;
			}
 */
			memcpy(gspca_dev->image + gspca_dev->image_len,
				data, len);
			gspca_dev->image_len += len;
		}
	}
	gspca_dev->last_packet_type = packet_type;

	/* if last packet, invalidate packet concatenation until
	 * next first packet, wake up the application and advance
	 * in the queue */
	if (packet_type == LAST_PACKET) {
		if (gspca_dev->image_len > gspca_dev->pixfmt.sizeimage)
			gspca_dev->image_len = gspca_dev->pixfmt.sizeimage;
		spin_lock_irqsave(&gspca_dev->qlock, flags);
		list_del(&buf->list);
		spin_unlock_irqrestore(&gspca_dev->qlock, flags);
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
				      gspca_dev->image_len);
		buf->vb.sequence = gspca_dev->sequence++;
		buf->vb.field = V4L2_FIELD_NONE;
		gspca_dbg(gspca_dev, D_FRAM, "frame complete len:%d\n",
			  gspca_dev->image_len);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		gspca_dev->image = NULL;
		gspca_dev->image_len = 0;
	}
}

static void destroy_urbs(struct gspca_dev *gspca_dev)
{
    KDBG("destroy_urbs"," - %d",elgcnt)

	struct urb *urb;
	unsigned int i;

	gspca_dbg(gspca_dev, D_STREAM, "kill transfer\n");

	/* Killing all URBs guarantee that no URB completion
	 * handler is running. Therefore, there shouldn't
	 * be anyone trying to access gspca_dev->urb[i]
	 */
	for (i = 0; i < MAX_NURBS; i++)
		usb_kill_urb(gspca_dev->urb[i]);

	gspca_dbg(gspca_dev, D_STREAM, "releasing urbs\n");
	for (i = 0; i < MAX_NURBS; i++) {
		urb = gspca_dev->urb[i];
		if (!urb)
			continue;
		gspca_dev->urb[i] = NULL;
		usb_free_coherent(gspca_dev->dev,
				  urb->transfer_buffer_length,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		usb_free_urb(urb);
	}
}





static int gspca_set_alt0(struct gspca_dev *gspca_dev)
{
    KDBG("gspca_set_alt0"," - %d",elgcnt)

	int ret;

	if (gspca_dev->alt == 0)
		return 0;
	ret = usb_set_interface(gspca_dev->dev, gspca_dev->iface, 0);
	if (ret < 0)
		pr_err("set alt 0 err %d\n", ret);
	return ret;
}

/*
 * look for an input transfer endpoint in an alternate setting.
 *
 * If xfer_ep is invalid, return the first valid ep found, otherwise
 * look for exactly the ep with address equal to xfer_ep.
 */
static struct usb_host_endpoint *alt_xfer(struct usb_host_interface *alt, int xfer, int xfer_ep)
{
    KDBG("usb_host_endpoint"," - %d",elgcnt)

	struct usb_host_endpoint *ep;
	int i, attr;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		ep = &alt->endpoint[i];
		attr = ep->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
		if (attr == xfer
		    && ep->desc.wMaxPacketSize != 0
		    && usb_endpoint_dir_in(&ep->desc)
		    && (xfer_ep < 0 || ep->desc.bEndpointAddress == xfer_ep))
			return ep;
	}
	return NULL;
}

/* compute the minimum bandwidth for the current transfer */
static u32 which_bandwidth(struct gspca_dev *gspca_dev)
{
    KDBG("which_bandwidth"," - %d",elgcnt)

	u32 bandwidth;

	/* get the (max) image size */
	bandwidth = gspca_dev->pixfmt.sizeimage;

	/* if the image is compressed, estimate its mean size */
	if (!gspca_dev->cam.needs_full_bandwidth &&
	    bandwidth < gspca_dev->pixfmt.width *
				gspca_dev->pixfmt.height)
		bandwidth = bandwidth * 3 / 8;	/* 0.375 */

	/* estimate the frame rate */
	if (gspca_dev->sd_desc->get_streamparm) {
		struct v4l2_streamparm parm;

		gspca_dev->sd_desc->get_streamparm(gspca_dev, &parm);
		bandwidth *= parm.parm.capture.timeperframe.denominator;
		bandwidth /= parm.parm.capture.timeperframe.numerator;
	} else {

		/* don't hope more than 15 fps with USB 1.1 and
		 * image resolution >= 640x480 */
		if (gspca_dev->pixfmt.width >= 640
		 && gspca_dev->dev->speed == USB_SPEED_FULL)
			bandwidth *= 15;		/* 15 fps */
		else
			bandwidth *= 30;		/* 30 fps */
	}

	gspca_dbg(gspca_dev, D_STREAM, "min bandwidth: %d\n", bandwidth);
	return bandwidth;
}


/* endpoint table */
#define MAX_ALT 16
struct ep_tb_s {
	u32 alt;
	u32 bandwidth;
};

/*
 * build the table of the endpoints
 * and compute the minimum bandwidth for the image transfer
 */
static int build_isoc_ep_tb(struct gspca_dev *gspca_dev, struct usb_interface *intf, struct ep_tb_s *ep_tb)
{
    KDBG("build_isoc_ep_tb"," - %d",elgcnt)

	struct usb_host_endpoint *ep;
	int i, j, nbalt, psize, found;
	u32 bandwidth, last_bw;

	nbalt = intf->num_altsetting;
	if (nbalt > MAX_ALT)
		nbalt = MAX_ALT;	/* fixme: should warn */

	/* build the endpoint table */
	i = 0;
	last_bw = 0;
	for (;;) {
		ep_tb->bandwidth = 2000 * 2000 * 120;
		found = 0;
		for (j = 0; j < nbalt; j++) {
			ep = alt_xfer(&intf->altsetting[j],
				      USB_ENDPOINT_XFER_ISOC,
				      gspca_dev->xfer_ep);
			if (ep == NULL)
				continue;
			if (ep->desc.bInterval == 0) {
				pr_err("alt %d iso endp with 0 interval\n", j);
				continue;
			}
			psize = le16_to_cpu(ep->desc.wMaxPacketSize);
			psize = (psize & 0x07ff) * (1 + ((psize >> 11) & 3));
			bandwidth = psize * 1000;
			if (gspca_dev->dev->speed == USB_SPEED_HIGH
			 || gspca_dev->dev->speed >= USB_SPEED_SUPER)
				bandwidth *= 8;
			bandwidth /= 1 << (ep->desc.bInterval - 1);
			if (bandwidth <= last_bw)
				continue;
			if (bandwidth < ep_tb->bandwidth) {
				ep_tb->bandwidth = bandwidth;
				ep_tb->alt = j;
				found = 1;
			}
		}
		if (!found)
			break;
		gspca_dbg(gspca_dev, D_STREAM, "alt %d bandwidth %d\n",
			  ep_tb->alt, ep_tb->bandwidth);
		last_bw = ep_tb->bandwidth;
		i++;
		ep_tb++;
	}

	/*
	 * If the camera:
	 * has a usb audio class interface (a built in usb mic); and
	 * is a usb 1 full speed device; and
	 * uses the max full speed iso bandwidth; and
	 * and has more than 1 alt setting
	 * then skip the highest alt setting to spare bandwidth for the mic
	 */
	if (gspca_dev->audio &&
			gspca_dev->dev->speed == USB_SPEED_FULL &&
			last_bw >= 1000000 &&
			i > 1) {
		gspca_dbg(gspca_dev, D_STREAM, "dev has usb audio, skipping highest alt\n");
		i--;
		ep_tb--;
	}

	/* get the requested bandwidth and start at the highest atlsetting */
	bandwidth = which_bandwidth(gspca_dev);
	ep_tb--;
	while (i > 1) {
		ep_tb--;
		if (ep_tb->bandwidth < bandwidth)
			break;
		i--;
	}
	return i;
}

/*
 * create the URBs for image transfer
 */
static int create_urbs(struct gspca_dev *gspca_dev, struct usb_host_endpoint *ep)
{
    KDBG("create_urbs"," - %d",elgcnt)

	struct urb *urb;
	int n, nurbs, i, psize, npkt, bsize;

	/* calculate the packet size and the number of packets */
	psize = le16_to_cpu(ep->desc.wMaxPacketSize);

	if (!gspca_dev->cam.bulk) {		/* isoc */

		/* See paragraph 5.9 / table 5-11 of the usb 2.0 spec. */
		if (gspca_dev->pkt_size == 0)
			psize = (psize & 0x07ff) * (1 + ((psize >> 11) & 3));
		else
			psize = gspca_dev->pkt_size;
		npkt = gspca_dev->cam.npkt;
		if (npkt == 0)
			npkt = 32;		/* default value */
		bsize = psize * npkt;
		gspca_dbg(gspca_dev, D_STREAM,
			  "isoc %d pkts size %d = bsize:%d\n",
			  npkt, psize, bsize);
		nurbs = DEF_NURBS;
	} else {				/* bulk */
		npkt = 0;
		bsize = gspca_dev->cam.bulk_size;
		if (bsize == 0)
			bsize = psize;
		gspca_dbg(gspca_dev, D_STREAM, "bulk bsize:%d\n", bsize);
		if (gspca_dev->cam.bulk_nurbs != 0)
			nurbs = gspca_dev->cam.bulk_nurbs;
		else
			nurbs = 1;
	}

	for (n = 0; n < nurbs; n++) {
		urb = usb_alloc_urb(npkt, GFP_KERNEL);
		if (!urb)
			return -ENOMEM;
		gspca_dev->urb[n] = urb;
		urb->transfer_buffer = usb_alloc_coherent(gspca_dev->dev,
						bsize,
						GFP_KERNEL,
						&urb->transfer_dma);

		if (urb->transfer_buffer == NULL) {
			pr_err("usb_alloc_coherent failed\n");
			return -ENOMEM;
		}
		urb->dev = gspca_dev->dev;
		urb->context = gspca_dev;
		urb->transfer_buffer_length = bsize;
		if (npkt != 0) {		/* ISOC */
			urb->pipe = usb_rcvisocpipe(gspca_dev->dev,
						    ep->desc.bEndpointAddress);
			urb->transfer_flags = URB_ISO_ASAP
					| URB_NO_TRANSFER_DMA_MAP;
			urb->interval = 1 << (ep->desc.bInterval - 1);
			urb->complete = isoc_irq;
			urb->number_of_packets = npkt;
			for (i = 0; i < npkt; i++) {
				urb->iso_frame_desc[i].length = psize;
				urb->iso_frame_desc[i].offset = psize * i;
			}
		} else {		/* bulk */
			urb->pipe = usb_rcvbulkpipe(gspca_dev->dev,
						ep->desc.bEndpointAddress);
			urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
			urb->complete = bulk_irq;
		}
	}
	return 0;
}

/* Note: both the queue and the usb locks should be held when calling this */
static void gspca_stream_off(struct gspca_dev *gspca_dev)
{
    KDBG("gspca_stream_off"," - %d",elgcnt)

	gspca_dev->streaming = false;
	gspca_dev->usb_err = 0;
	if (gspca_dev->sd_desc->stopN)
		gspca_dev->sd_desc->stopN(gspca_dev);
	destroy_urbs(gspca_dev);
	gspca_input_destroy_urb(gspca_dev);
	gspca_set_alt0(gspca_dev);
	if (gspca_dev->present)
		gspca_input_create_urb(gspca_dev);
	if (gspca_dev->sd_desc->stop0)
		gspca_dev->sd_desc->stop0(gspca_dev);
	gspca_dbg(gspca_dev, D_STREAM, "stream off OK\n");
}

/*
 * start the USB transfer
 */
static int gspca_init_transfer(struct gspca_dev *gspca_dev)
{
    KDBG("gspca_init_transfer"," - %d",elgcnt)

	struct usb_interface *intf;
	struct usb_host_endpoint *ep;
	struct urb *urb;
	struct ep_tb_s ep_tb[MAX_ALT];
	int n, ret, xfer, alt, alt_idx;

	/* reset the streaming variables */
	gspca_dev->image = NULL;
	gspca_dev->image_len = 0;
	gspca_dev->last_packet_type = DISCARD_PACKET;

	gspca_dev->usb_err = 0;

	/* do the specific subdriver stuff before endpoint selection */
	intf = usb_ifnum_to_if(gspca_dev->dev, gspca_dev->iface);
	gspca_dev->alt = gspca_dev->cam.bulk ? intf->num_altsetting : 0;
	if (gspca_dev->sd_desc->isoc_init) {
		ret = gspca_dev->sd_desc->isoc_init(gspca_dev);
		if (ret < 0)
			return ret;
	}
	xfer = gspca_dev->cam.bulk ? USB_ENDPOINT_XFER_BULK
				   : USB_ENDPOINT_XFER_ISOC;

	/* if bulk or the subdriver forced an altsetting, get the endpoint */
	if (gspca_dev->alt != 0) {
		gspca_dev->alt--;	/* (previous version compatibility) */
		ep = alt_xfer(&intf->altsetting[gspca_dev->alt], xfer,
			      gspca_dev->xfer_ep);
		if (ep == NULL) {
			pr_err("bad altsetting %d\n", gspca_dev->alt);
			return -EIO;
		}
		ep_tb[0].alt = gspca_dev->alt;
		alt_idx = 1;
	} else {
		/* else, compute the minimum bandwidth
		 * and build the endpoint table */
		alt_idx = build_isoc_ep_tb(gspca_dev, intf, ep_tb);
		if (alt_idx <= 0) {
			pr_err("no transfer endpoint found\n");
			return -EIO;
		}
	}

	/* set the highest alternate setting and
	 * loop until urb submit succeeds */
	gspca_input_destroy_urb(gspca_dev);

	gspca_dev->alt = ep_tb[--alt_idx].alt;
	alt = -1;
	for (;;) {
		if (alt != gspca_dev->alt) {
			alt = gspca_dev->alt;
			if (intf->num_altsetting > 1) {
				ret = usb_set_interface(gspca_dev->dev,
							gspca_dev->iface,
							alt);
				if (ret < 0) {
					if (ret == -ENOSPC)
						goto retry; /*fixme: ugly*/
					pr_err("set alt %d err %d\n", alt, ret);
					goto out;
				}
			}
		}
		if (!gspca_dev->cam.no_urb_create) {
			gspca_dbg(gspca_dev, D_STREAM, "init transfer alt %d\n",
				  alt);
			ret = create_urbs(gspca_dev,
				alt_xfer(&intf->altsetting[alt], xfer,
					 gspca_dev->xfer_ep));
			if (ret < 0) {
				destroy_urbs(gspca_dev);
				goto out;
			}
		}

		/* clear the bulk endpoint */
		if (gspca_dev->cam.bulk)
			usb_clear_halt(gspca_dev->dev, gspca_dev->urb[0]->pipe);

		/* start the cam */
		ret = gspca_dev->sd_desc->start(gspca_dev);
		if (ret < 0) {
			destroy_urbs(gspca_dev);
			goto out;
		}
		v4l2_ctrl_handler_setup(gspca_dev->vdev.ctrl_handler);
		gspca_dev->streaming = true;

		/* some bulk transfers are started by the subdriver */
		if (gspca_dev->cam.bulk && gspca_dev->cam.bulk_nurbs == 0)
			break;

		/* submit the URBs */
		for (n = 0; n < MAX_NURBS; n++) {
			urb = gspca_dev->urb[n];
			if (urb == NULL)
				break;
			ret = usb_submit_urb(urb, GFP_KERNEL);
			if (ret < 0)
				break;
		}
		if (ret >= 0)
			break;			/* transfer is started */

		/* something when wrong
		 * stop the webcam and free the transfer resources */
		gspca_stream_off(gspca_dev);
		if (ret != -ENOSPC) {
			pr_err("usb_submit_urb alt %d err %d\n",
			       gspca_dev->alt, ret);
			goto out;
		}

		/* the bandwidth is not wide enough
		 * negotiate or try a lower alternate setting */
retry:
		gspca_err(gspca_dev, "alt %d - bandwidth not wide enough, trying again\n",
			  alt);
		msleep(20);	/* wait for kill complete */
		if (gspca_dev->sd_desc->isoc_nego) {
			ret = gspca_dev->sd_desc->isoc_nego(gspca_dev);
			if (ret < 0)
				goto out;
		} else {
			if (alt_idx <= 0) {
				pr_err("no transfer endpoint found\n");
				ret = -EIO;
				goto out;
			}
			gspca_dev->alt = ep_tb[--alt_idx].alt;
		}
	}
out:
	gspca_input_create_urb(gspca_dev);
	return ret;
}

// DEVICE PLUGGEDED - 8
static void gspca_set_default_mode(struct gspca_dev *gspca_dev)
{
    KDBG("gspca_set_default_mode"," - %d",elgcnt)

	int i;

	i = gspca_dev->cam.nmodes - 1;	/* take the highest mode */
	gspca_dev->curr_mode = i;
	gspca_dev->pixfmt = gspca_dev->cam.cam_mode[i];

	/* does nothing if ctrl_handler == NULL */
	v4l2_ctrl_handler_setup(gspca_dev->vdev.ctrl_handler);
}

static int wxh_to_mode(struct gspca_dev *gspca_dev,	int width, int height, u32 pixelformat)
{
    KDBG("wxh_to_mode"," - %d",elgcnt)

	int i;

	for (i = 0; i < gspca_dev->cam.nmodes; i++) {
		if (width == gspca_dev->cam.cam_mode[i].width
		    && height == gspca_dev->cam.cam_mode[i].height
		    && pixelformat == gspca_dev->cam.cam_mode[i].pixelformat)
			return i;
	}
	return -EINVAL;
}

static int wxh_to_nearest_mode(struct gspca_dev *gspca_dev,	int width, int height, u32 pixelformat)
{
    KDBG("wxh_to_nearest_mode"," - %d",elgcnt)

	int i;

	for (i = gspca_dev->cam.nmodes; --i >= 0; ) {
		if (width >= gspca_dev->cam.cam_mode[i].width
		    && height >= gspca_dev->cam.cam_mode[i].height
		    && pixelformat == gspca_dev->cam.cam_mode[i].pixelformat)
			return i;
	}
	for (i = gspca_dev->cam.nmodes; --i > 0; ) {
		if (width >= gspca_dev->cam.cam_mode[i].width
		    && height >= gspca_dev->cam.cam_mode[i].height)
			break;
	}
	return i;
}

/*
 * search a mode with the right pixel format
 */
static int gspca_get_mode(struct gspca_dev *gspca_dev, int mode, int pixfmt)
{
    KDBG("gspca_get_mode"," - %d",elgcnt)

	int modeU, modeD;

	modeU = modeD = mode;
	while ((modeU < gspca_dev->cam.nmodes) || modeD >= 0) {
		if (--modeD >= 0) {
			if (gspca_dev->cam.cam_mode[modeD].pixelformat
								== pixfmt)
				return modeD;
		}
		if (++modeU < gspca_dev->cam.nmodes) {
			if (gspca_dev->cam.cam_mode[modeU].pixelformat
								== pixfmt)
				return modeU;
		}
	}
	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vidioc_g_chip_info(struct file *file, void *priv, struct v4l2_dbg_chip_info *chip)
{
    KDBG("vidioc_g_chip_info"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	if (gspca_dev->sd_desc->get_chip_info)
		return gspca_dev->sd_desc->get_chip_info(gspca_dev, chip);
	return chip->match.addr ? -EINVAL : 0;
}

static int vidioc_g_register(struct file *file, void *priv,	struct v4l2_dbg_register *reg)
{
    KDBG("vidioc_g_register"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->get_register(gspca_dev, reg);
}

static int vidioc_s_register(struct file *file, void *priv,	const struct v4l2_dbg_register *reg)
{
    KDBG("vidioc_s_register"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->set_register(gspca_dev, reg);
}
#endif

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv, struct v4l2_fmtdesc *fmtdesc)
{
    KDBG("vidioc_enum_fmt_vid_cap"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);
	int i, j, index;
	__u32 fmt_tb[8];

	/* give an index to each format */
	index = 0;
	for (i = gspca_dev->cam.nmodes; --i >= 0; ) {
		fmt_tb[index] = gspca_dev->cam.cam_mode[i].pixelformat;
		j = 0;
		for (;;) {
			if (fmt_tb[j] == fmt_tb[index])
				break;
			j++;
		}
		if (j == index) {
			if (fmtdesc->index == index)
				break;		/* new format */
			index++;
			if (index >= ARRAY_SIZE(fmt_tb))
				return -EINVAL;
		}
	}
	if (i < 0)
		return -EINVAL;		/* no more format */

	fmtdesc->pixelformat = fmt_tb[index];
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *_priv,	struct v4l2_format *fmt)
{
    KDBG("vidioc_g_fmt_vid_cap"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);
	u32 priv = fmt->fmt.pix.priv;

	fmt->fmt.pix = gspca_dev->pixfmt;
	/* some drivers use priv internally, so keep the original value */
	fmt->fmt.pix.priv = priv;
	return 0;
}

static int try_fmt_vid_cap(struct gspca_dev *gspca_dev, struct v4l2_format *fmt)
{
    KDBG("try_fmt_vid_cap"," - %d",elgcnt)

	int w, h, mode, mode2;

	w = fmt->fmt.pix.width;
	h = fmt->fmt.pix.height;

	PDEBUG_MODE(gspca_dev, D_CONF, "try fmt cap",
		    fmt->fmt.pix.pixelformat, w, h);

	/* search the nearest mode for width and height */
	mode = wxh_to_nearest_mode(gspca_dev, w, h, fmt->fmt.pix.pixelformat);

	/* OK if right palette */
	if (gspca_dev->cam.cam_mode[mode].pixelformat
						!= fmt->fmt.pix.pixelformat) {

		/* else, search the closest mode with the same pixel format */
		mode2 = gspca_get_mode(gspca_dev, mode,
					fmt->fmt.pix.pixelformat);
		if (mode2 >= 0)
			mode = mode2;
	}
	fmt->fmt.pix = gspca_dev->cam.cam_mode[mode];
	if (gspca_dev->sd_desc->try_fmt) {
		/* pass original resolution to subdriver try_fmt */
		fmt->fmt.pix.width = w;
		fmt->fmt.pix.height = h;
		gspca_dev->sd_desc->try_fmt(gspca_dev, fmt);
	}
	return mode;			/* used when s_fmt */
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *_priv, struct v4l2_format *fmt)
{
    KDBG("vidioc_try_fmt_vid_cap"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);
	u32 priv = fmt->fmt.pix.priv;

	if (try_fmt_vid_cap(gspca_dev, fmt) < 0)
		return -EINVAL;
	/* some drivers use priv internally, so keep the original value */
	fmt->fmt.pix.priv = priv;
	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *_priv, struct v4l2_format *fmt)
{
    KDBG("vidioc_s_fmt_vid_cap"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);
	u32 priv = fmt->fmt.pix.priv;
	int mode;

	if (vb2_is_busy(&gspca_dev->queue))
		return -EBUSY;

	mode = try_fmt_vid_cap(gspca_dev, fmt);
	if (mode < 0)
		return -EINVAL;

	gspca_dev->curr_mode = mode;
	if (gspca_dev->sd_desc->try_fmt)
		/* subdriver try_fmt can modify format parameters */
		gspca_dev->pixfmt = fmt->fmt.pix;
	else
		gspca_dev->pixfmt = gspca_dev->cam.cam_mode[mode];
	/* some drivers use priv internally, so keep the original value */
	fmt->fmt.pix.priv = priv;
	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
    KDBG("vidioc_enum_framesizes"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);
	int i;
	__u32 index = 0;

	if (gspca_dev->sd_desc->enum_framesizes)
		return gspca_dev->sd_desc->enum_framesizes(gspca_dev, fsize);

	for (i = 0; i < gspca_dev->cam.nmodes; i++) {
		if (fsize->pixel_format !=
				gspca_dev->cam.cam_mode[i].pixelformat)
			continue;

		if (fsize->index == index) {
			fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			fsize->discrete.width =
				gspca_dev->cam.cam_mode[i].width;
			fsize->discrete.height =
				gspca_dev->cam.cam_mode[i].height;
			return 0;
		}
		index++;
	}

	return -EINVAL;
}

static int vidioc_enum_frameintervals(struct file *filp, void *priv, struct v4l2_frmivalenum *fival)
{
    KDBG("vidioc_enum_frameintervals"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(filp);
	int mode;
	__u32 i;

	mode = wxh_to_mode(gspca_dev, fival->width, fival->height,
			   fival->pixel_format);
	if (mode < 0)
		return -EINVAL;

	if (gspca_dev->cam.mode_framerates == NULL ||
			gspca_dev->cam.mode_framerates[mode].nrates == 0)
		return -EINVAL;

	if (fival->pixel_format !=
			gspca_dev->cam.cam_mode[mode].pixelformat)
		return -EINVAL;

	for (i = 0; i < gspca_dev->cam.mode_framerates[mode].nrates; i++) {
		if (fival->index == i) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator =
				gspca_dev->cam.mode_framerates[mode].rates[i];
			return 0;
		}
	}

	return -EINVAL;
}

static void gspca_release(struct v4l2_device *v4l2_device)
{
    KDBG("gspca_release"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = container_of(v4l2_device, struct gspca_dev, v4l2_dev);

	v4l2_ctrl_handler_free(gspca_dev->vdev.ctrl_handler);
	v4l2_device_unregister(&gspca_dev->v4l2_dev);
	kfree(gspca_dev->usb_buf);
	kfree(gspca_dev);
}

// DEVICE PLUGGEDED - 15
static int vidioc_querycap(struct file *file, void  *priv, struct v4l2_capability *cap)
{
    KDBG("vidioc_querycap"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	strscpy((char *)cap->driver, gspca_dev->sd_desc->name,
		sizeof(cap->driver));
	if (gspca_dev->dev->product != NULL) {
		strscpy((char *)cap->card, gspca_dev->dev->product,
			sizeof(cap->card));
	} else {
		snprintf((char *) cap->card, sizeof cap->card,
			"USB Camera (%04x:%04x)",
			le16_to_cpu(gspca_dev->dev->descriptor.idVendor),
			le16_to_cpu(gspca_dev->dev->descriptor.idProduct));
	}
	usb_make_path(gspca_dev->dev, (char *) cap->bus_info,
			sizeof(cap->bus_info));
	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,	struct v4l2_input *input)
{
    KDBG("vidioc_enum_input"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	if (input->index != 0)
		return -EINVAL;
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->status = gspca_dev->cam.input_flags;
	strscpy(input->name, gspca_dev->sd_desc->name,
		sizeof input->name);
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
    KDBG("vidioc_g_input"," - %d",elgcnt)

	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
    KDBG("vidioc_s_input"," - %d",elgcnt)

	if (i > 0)
		return -EINVAL;
	return 0;
}

static int vidioc_g_jpegcomp(struct file *file, void *priv, struct v4l2_jpegcompression *jpegcomp)
{
    KDBG("vidioc_g_jpegcomp"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->get_jcomp(gspca_dev, jpegcomp);
}

static int vidioc_s_jpegcomp(struct file *file, void *priv, const struct v4l2_jpegcompression *jpegcomp)
{
    KDBG("vidioc_s_jpegcomp"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->set_jcomp(gspca_dev, jpegcomp);
}

static int vidioc_g_parm(struct file *filp, void *priv,	struct v4l2_streamparm *parm)
{
    KDBG("vidioc_g_parm"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(filp);

	parm->parm.capture.readbuffers = gspca_dev->queue.min_buffers_needed;

	if (!gspca_dev->sd_desc->get_streamparm)
		return 0;

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	gspca_dev->usb_err = 0;
	gspca_dev->sd_desc->get_streamparm(gspca_dev, parm);
	return gspca_dev->usb_err;
}

static int vidioc_s_parm(struct file *filp, void *priv, struct v4l2_streamparm *parm)
{
    KDBG("vidioc_s_parm"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = video_drvdata(filp);

	parm->parm.capture.readbuffers = gspca_dev->queue.min_buffers_needed;

	if (!gspca_dev->sd_desc->set_streamparm) {
		parm->parm.capture.capability = 0;
		return 0;
	}

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	gspca_dev->usb_err = 0;
	gspca_dev->sd_desc->set_streamparm(gspca_dev, parm);
	return gspca_dev->usb_err;
}

static int gspca_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers, unsigned int *nplanes, unsigned int sizes[], struct device *alloc_devs[])
{
    KDBG("gspca_queue_setup"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vq);
	unsigned int size = PAGE_ALIGN(gspca_dev->pixfmt.sizeimage);

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int gspca_buffer_prepare(struct vb2_buffer *vb)
{
    KDBG("gspca_buffer_prepare"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = PAGE_ALIGN(gspca_dev->pixfmt.sizeimage);

	if (vb2_plane_size(vb, 0) < size) {
		gspca_err(gspca_dev, "buffer too small (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	return 0;
}

static void gspca_buffer_finish(struct vb2_buffer *vb)
{
    KDBG("gspca_buffer_finish"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vb->vb2_queue);

	if (!gspca_dev->sd_desc->dq_callback)
		return;

	gspca_dev->usb_err = 0;
	if (gspca_dev->present)
		gspca_dev->sd_desc->dq_callback(gspca_dev);
}

static void gspca_buffer_queue(struct vb2_buffer *vb)
{
    KDBG("gspca_buffer_queue"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct gspca_buffer *buf = to_gspca_buffer(vb);
	unsigned long flags;

	spin_lock_irqsave(&gspca_dev->qlock, flags);
	list_add_tail(&buf->list, &gspca_dev->buf_list);
	spin_unlock_irqrestore(&gspca_dev->qlock, flags);
}

static void gspca_return_all_buffers(struct gspca_dev *gspca_dev, enum vb2_buffer_state state)
{
    KDBG("gspca_return_all_buffers"," - %d",elgcnt)

	struct gspca_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&gspca_dev->qlock, flags);
	list_for_each_entry_safe(buf, node, &gspca_dev->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&gspca_dev->qlock, flags);
}

static int gspca_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    KDBG("gspca_start_streaming"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vq);
	int ret;

	gspca_dev->sequence = 0;

	ret = gspca_init_transfer(gspca_dev);
	if (ret)
		gspca_return_all_buffers(gspca_dev, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void gspca_stop_streaming(struct vb2_queue *vq)
{
    KDBG("gspca_stop_streaming"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vq);

	gspca_stream_off(gspca_dev);

	/* Release all active buffers */
	gspca_return_all_buffers(gspca_dev, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops gspca_qops = {
	.queue_setup		= gspca_queue_setup,
	.buf_prepare		= gspca_buffer_prepare,
	.buf_finish		= gspca_buffer_finish,
	.buf_queue		= gspca_buffer_queue,
	.start_streaming	= gspca_start_streaming,
	.stop_streaming		= gspca_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static const struct v4l2_file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static const struct v4l2_ioctl_ops dev_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,
	.vidioc_enum_input	= vidioc_enum_input,
	.vidioc_g_input		= vidioc_g_input,
	.vidioc_s_input		= vidioc_s_input,
	.vidioc_g_jpegcomp	= vidioc_g_jpegcomp,
	.vidioc_s_jpegcomp	= vidioc_s_jpegcomp,
	.vidioc_g_parm		= vidioc_g_parm,
	.vidioc_s_parm		= vidioc_s_parm,
	.vidioc_enum_framesizes = vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = vidioc_enum_frameintervals,
	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,
	.vidioc_expbuf		= vb2_ioctl_expbuf,
	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_chip_info	= vidioc_g_chip_info,
	.vidioc_g_register	= vidioc_g_register,
	.vidioc_s_register	= vidioc_s_register,
#endif
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct video_device gspca_template = {
	.name = "gspca main driver",
	.fops = &dev_fops,
	.ioctl_ops = &dev_ioctl_ops,
	.release = video_device_release_empty, /* We use v4l2_dev.release */
};

/*
 * probe and create a new gspca device
 *
 * This function must be called by the sub-driver when it is
 * called for probing a new device.
 */
 // DEVICE PLUGGEDED - 2
int gspca_dev_probe2(struct usb_interface *intf, const struct usb_device_id *id, const struct sd_desc *sd_desc,	int dev_size, struct module *module)
{
    KDBG("gspca_dev_probe2"," - %d",elgcnt)

	struct gspca_dev *gspca_dev;
	struct usb_device *dev = interface_to_usbdev(intf);
	struct vb2_queue *q;
	int ret;

	pr_info("%s-" GSPCA_VERSION " probing %04x:%04x\n", sd_desc->name, id->idVendor, id->idProduct);

	/* create the device */
	if (dev_size < sizeof *gspca_dev)
		dev_size = sizeof *gspca_dev;
/// ******************
	gspca_dev = kzalloc(dev_size, GFP_KERNEL);
	if (!gspca_dev) {
		pr_err("couldn't kzalloc gspca struct\n");
		return -ENOMEM;
	}

/// ******************
	gspca_dev->usb_buf = kzalloc(USB_BUF_SZ, GFP_KERNEL);
	if (!gspca_dev->usb_buf) {
		pr_err("out of memory\n");
		ret = -ENOMEM;
		goto out;
	}


/// ******************
	gspca_dev->dev = dev;



	gspca_dev->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	gspca_dev->xfer_ep = -1;

	/* check if any audio device */
	if (dev->actconfig->desc.bNumInterfaces != 1) {
		int i;
		struct usb_interface *intf2;

		for (i = 0; i < dev->actconfig->desc.bNumInterfaces; i++) {
			intf2 = dev->actconfig->interface[i];
			if (intf2 != NULL
			 && intf2->altsetting != NULL
			 && intf2->altsetting->desc.bInterfaceClass ==
					 USB_CLASS_AUDIO) {
				gspca_dev->audio = 1;
				break;
			}
		}
	}


/// ******************
	gspca_dev->v4l2_dev.release = gspca_release;


/// ******************
	ret = v4l2_device_register(&intf->dev, &gspca_dev->v4l2_dev);
	if (ret)
		goto out;
	gspca_dev->present = true;
	gspca_dev->sd_desc = sd_desc;
	gspca_dev->empty_packet = -1;	/* don't check the empty packets */



/// ******************
	gspca_dev->vdev = gspca_template; /* .name, .fops, .ioctl_ops, .release */



	gspca_dev->vdev.v4l2_dev = &gspca_dev->v4l2_dev;
	gspca_dev->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;

	video_set_drvdata(&gspca_dev->vdev, gspca_dev);

	gspca_dev->module = module;

	mutex_init(&gspca_dev->usb_lock);
	gspca_dev->vdev.lock = &gspca_dev->usb_lock;
	init_waitqueue_head(&gspca_dev->wq);

	/* Initialize the vb2 queue */
	q = &gspca_dev->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->drv_priv = gspca_dev;
	q->buf_struct_size = sizeof(struct gspca_buffer);
	q->ops = &gspca_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &gspca_dev->usb_lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto out;
	gspca_dev->vdev.queue = q;

	INIT_LIST_HEAD(&gspca_dev->buf_list);
	spin_lock_init(&gspca_dev->qlock);

	/* configure the subdriver and initialize the USB device */
	ret = sd_desc->config(gspca_dev, id);
	if (ret < 0)
		goto out;
	ret = sd_desc->init(gspca_dev);
	if (ret < 0)
		goto out;
	if (sd_desc->init_controls)
		ret = sd_desc->init_controls(gspca_dev);
	if (ret < 0)
		goto out;
	gspca_set_default_mode(gspca_dev);

	ret = gspca_input_connect(gspca_dev);
	if (ret)
		goto out;

#ifdef CONFIG_VIDEO_ADV_DEBUG
	if (!gspca_dev->sd_desc->get_register)
		v4l2_disable_ioctl(&gspca_dev->vdev, VIDIOC_DBG_G_REGISTER);
	if (!gspca_dev->sd_desc->set_register)
		v4l2_disable_ioctl(&gspca_dev->vdev, VIDIOC_DBG_S_REGISTER);
#endif
	if (!gspca_dev->sd_desc->get_jcomp)
		v4l2_disable_ioctl(&gspca_dev->vdev, VIDIOC_G_JPEGCOMP);
	if (!gspca_dev->sd_desc->set_jcomp)
		v4l2_disable_ioctl(&gspca_dev->vdev, VIDIOC_S_JPEGCOMP);

	/* init video stuff */
	ret = video_register_device(&gspca_dev->vdev,  /*VFL_TYPE_VIDEO*/VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		pr_err("video_register_device err %d\n", ret);
		goto out;
	}

	usb_set_intfdata(intf, gspca_dev);
	gspca_dbg(gspca_dev, D_PROBE, "%s created\n", video_device_node_name(&gspca_dev->vdev));

	gspca_input_create_urb(gspca_dev);

	return 0;
out:
#if IS_ENABLED(CONFIG_INPUT)
	if (gspca_dev->input_dev)
		input_unregister_device(gspca_dev->input_dev);
#endif
	v4l2_ctrl_handler_free(gspca_dev->vdev.ctrl_handler);
	v4l2_device_unregister(&gspca_dev->v4l2_dev);
	if (sd_desc->probe_error)
		sd_desc->probe_error(gspca_dev);
	kfree(gspca_dev->usb_buf);
	kfree(gspca_dev);
	return ret;
}

/* same function as the previous one, but check the interface */
// DEVICE PLUGGEDED - 1
int gspca_dev_probe(struct usb_interface *intf,	const struct usb_device_id *id,	const struct sd_desc *sd_desc, int dev_size, struct module *module) {

    KDBG("gspca_dev_probe"," - %d",elgcnt)

	struct usb_device *dev = interface_to_usbdev(intf);
	/* we don't handle multi-config cameras */
	if (dev->descriptor.bNumConfigurations != 1) {
		pr_err("%04x:%04x too many config\n", id->idVendor, id->idProduct);
		return -ENODEV;
	}

	/* the USB video interface must be the first one */
	if (dev->actconfig->desc.bNumInterfaces != 1 && intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	return gspca_dev_probe2(intf, id, sd_desc, dev_size, module);
}

/*
 * USB disconnection
 *
 * This function must be called by the sub-driver
 * when the device disconnects, after the specific resources are freed.
 */
void gspca_disconnect(struct usb_interface *intf) {

    KDBG("gspca_disconnect"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = usb_get_intfdata(intf);
#if IS_ENABLED(CONFIG_INPUT)
	struct input_dev *input_dev;
#endif

	gspca_dbg(gspca_dev, D_PROBE, "%s disconnect\n", video_device_node_name(&gspca_dev->vdev));

	mutex_lock(&gspca_dev->usb_lock);
	gspca_dev->present = false;
	destroy_urbs(gspca_dev);
	gspca_input_destroy_urb(gspca_dev);

	vb2_queue_error(&gspca_dev->queue);

#if IS_ENABLED(CONFIG_INPUT)
	input_dev = gspca_dev->input_dev;
	if (input_dev) {
		gspca_dev->input_dev = NULL;
		input_unregister_device(input_dev);
	}
#endif

	v4l2_device_disconnect(&gspca_dev->v4l2_dev);
	video_unregister_device(&gspca_dev->vdev);

	mutex_unlock(&gspca_dev->usb_lock);

	/* (this will call gspca_release() immediately or on last close) */
	v4l2_device_put(&gspca_dev->v4l2_dev);
}

#ifdef CONFIG_PM
int gspca_suspend(struct usb_interface *intf, pm_message_t message)
{
    KDBG("gspca_suspend"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = usb_get_intfdata(intf);

	gspca_input_destroy_urb(gspca_dev);

	if (!vb2_start_streaming_called(&gspca_dev->queue))
		return 0;

	mutex_lock(&gspca_dev->usb_lock);
	gspca_dev->frozen = 1;		/* avoid urb error messages */
	gspca_dev->usb_err = 0;
	if (gspca_dev->sd_desc->stopN)
		gspca_dev->sd_desc->stopN(gspca_dev);
	destroy_urbs(gspca_dev);
	gspca_set_alt0(gspca_dev);
	if (gspca_dev->sd_desc->stop0)
		gspca_dev->sd_desc->stop0(gspca_dev);
	mutex_unlock(&gspca_dev->usb_lock);

	return 0;
}


int gspca_resume(struct usb_interface *intf)
{
    KDBG("gspca_resume"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = usb_get_intfdata(intf);
	int streaming, ret = 0;

	mutex_lock(&gspca_dev->usb_lock);
	gspca_dev->frozen = 0;
	gspca_dev->usb_err = 0;
	gspca_dev->sd_desc->init(gspca_dev);
	/*
	 * Most subdrivers send all ctrl values on sd_start and thus
	 * only write to the device registers on s_ctrl when streaming ->
	 * Clear streaming to avoid setting all ctrls twice.
	 */
	streaming = vb2_start_streaming_called(&gspca_dev->queue);
	if (streaming)
		ret = gspca_init_transfer(gspca_dev);
	else
		gspca_input_create_urb(gspca_dev);
	mutex_unlock(&gspca_dev->usb_lock);

	return ret;
}

#endif














/* specific webcam descriptor */
struct sd {
	struct gspca_dev gspca_dev;	/* !! must be the first item */

	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *plfreq;

	atomic_t avg_lum;
	int prev_avg_lum;
	int exposure_knee;
	int header_read;
	u8 header[12]; /* Header without sof marker */

	unsigned char autogain_ignore_frames;
	unsigned char frames_to_drop;

	__u8 bridge;			/* Type of bridge */
#define BRIDGE_101 0
#define BRIDGE_102 0 /* We make no difference between 101 and 102 */
#define BRIDGE_103 1

	__u8 sensor;			/* Type of image sensor chip */
#define SENSOR_HV7131D 0
#define SENSOR_HV7131R 1
#define SENSOR_OV6650 2
#define SENSOR_OV7630 3
#define SENSOR_PAS106 4
#define SENSOR_PAS202 5
#define SENSOR_TAS5110C 6
#define SENSOR_TAS5110D 7
#define SENSOR_TAS5130CXX 8
	__u8 reg11;
};



typedef const __u8 sensor_init_t[8];

struct sensor_data {
	const __u8 *bridge_init;
	sensor_init_t *sensor_init;
	int sensor_init_size;
	int flags;
	__u8 sensor_addr;
};


/* sensor_data flags */
#define F_SIF		0x01	/* sif or vga */

/* priv field of struct v4l2_pix_format flags (do not use low nibble!) */
#define MODE_RAW 0x10		/* raw bayer mode */
#define MODE_REDUCED_SIF 0x20	/* vga mode (320x240 / 160x120) on sif cam */

#define COMP 0xc7		/* 0x87 //0x07 */
#define COMP1 0xc9		/* 0x89 //0x09 */

#define MCK_INIT 0x63
#define MCK_INIT1 0x20		/*fixme: Bayer - 0x50 for JPEG ??*/

//#define SYS_CLK 0x04

#define SENS(bridge, sensor, _flags, _sensor_addr) \
{ \
	.bridge_init = bridge, \
	.sensor_init = sensor, \
	.sensor_init_size = sizeof(sensor), \
	.flags = _flags, .sensor_addr = _sensor_addr \
}


/* We calculate the autogain at the end of the transfer of a frame, at this
   moment a frame with the old settings is being captured and transmitted. So
   if we adjust the gain or exposure we must ignore at least the next frame for
   the new settings to come into effect before doing any other adjustments. */
#define AUTOGAIN_IGNORE_FRAMES 1


static const struct v4l2_pix_format vga_mode[] = {
	{160, 120, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,
		.bytesperline = 160,
		.sizeimage = 160 * 120,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 2 | MODE_RAW},
	{160, 120, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 160,
		.sizeimage = 160 * 120 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 2},
	{320, 240, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 320,
		.sizeimage = 320 * 240 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1},
	{640, 480, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 640,
		.sizeimage = 640 * 480 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0},
};


static const struct v4l2_pix_format sif_mode[] = {
	{160, 120, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,
		.bytesperline = 160,
		.sizeimage = 160 * 120,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1 | MODE_RAW | MODE_REDUCED_SIF},
	{160, 120, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 160,
		.sizeimage = 160 * 120 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1 | MODE_REDUCED_SIF},
	{176, 144, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,
		.bytesperline = 176,
		.sizeimage = 176 * 144,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1 | MODE_RAW},
	{176, 144, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 176,
		.sizeimage = 176 * 144 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1},
	{320, 240, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 320,
		.sizeimage = 320 * 240 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0 | MODE_REDUCED_SIF},
	{352, 288, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 352,
		.sizeimage = 352 * 288 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0},
};


static const __u8 initHv7131d[] = {
	0x04, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x11, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x02, 0x02, 0x00,
	0x28, 0x1e, 0x60, 0x8e, 0x42,
};
static const __u8 hv7131d_sensor_init[][8] = {
	{0xa0, 0x11, 0x01, 0x04, 0x00, 0x00, 0x00, 0x17},
	{0xa0, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x17},
	{0xa0, 0x11, 0x28, 0x00, 0x00, 0x00, 0x00, 0x17},
	{0xa0, 0x11, 0x30, 0x30, 0x00, 0x00, 0x00, 0x17}, /* reset level */
	{0xa0, 0x11, 0x34, 0x02, 0x00, 0x00, 0x00, 0x17}, /* pixel bias volt */
};

static const __u8 initHv7131r[] = {
	0x46, 0x77, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x11, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x02, 0x01, 0x00,
	0x28, 0x1e, 0x60, 0x8a, 0x20,
};
static const __u8 hv7131r_sensor_init[][8] = {
	{0xc0, 0x11, 0x31, 0x38, 0x2a, 0x2e, 0x00, 0x10},
	{0xa0, 0x11, 0x01, 0x08, 0x2a, 0x2e, 0x00, 0x10},
	{0xb0, 0x11, 0x20, 0x00, 0xd0, 0x2e, 0x00, 0x10},
	{0xc0, 0x11, 0x25, 0x03, 0x0e, 0x28, 0x00, 0x16},
	{0xa0, 0x11, 0x30, 0x10, 0x0e, 0x28, 0x00, 0x15},
};
static const __u8 initOv6650[] = {
	0x44, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x01, 0x0a, 0x16, 0x12, 0x68, 0x8b,
	0x10,
};
static const __u8 ov6650_sensor_init[][8] = {
	/* Bright, contrast, etc are set through SCBB interface.
	 * AVCAP on win2 do not send any data on this controls. */
	/* Anyway, some registers appears to alter bright and constrat */

	/* Reset sensor */
	{0xa0, 0x60, 0x12, 0x80, 0x00, 0x00, 0x00, 0x10},
	/* Set clock register 0x11 low nibble is clock divider */
	{0xd0, 0x60, 0x11, 0xc0, 0x1b, 0x18, 0xc1, 0x10},
	/* Next some unknown stuff */
	{0xb0, 0x60, 0x15, 0x00, 0x02, 0x18, 0xc1, 0x10},
/*	{0xa0, 0x60, 0x1b, 0x01, 0x02, 0x18, 0xc1, 0x10},
		 * THIS SET GREEN SCREEN
		 * (pixels could be innverted in decode kind of "brg",
		 * but blue wont be there. Avoid this data ... */
	{0xd0, 0x60, 0x26, 0x01, 0x14, 0xd8, 0xa4, 0x10}, /* format out? */
	{0xd0, 0x60, 0x26, 0x01, 0x14, 0xd8, 0xa4, 0x10},
	{0xa0, 0x60, 0x30, 0x3d, 0x0a, 0xd8, 0xa4, 0x10},
	/* Enable rgb brightness control */
	{0xa0, 0x60, 0x61, 0x08, 0x00, 0x00, 0x00, 0x10},
	/* HDG: Note windows uses the line below, which sets both register 0x60
	   and 0x61 I believe these registers of the ov6650 are identical as
	   those of the ov7630, because if this is true the windows settings
	   add a bit additional red gain and a lot additional blue gain, which
	   matches my findings that the windows settings make blue much too
	   blue and red a little too red.
	{0xb0, 0x60, 0x60, 0x66, 0x68, 0xd8, 0xa4, 0x10}, */
	/* Some more unknown stuff */
	{0xa0, 0x60, 0x68, 0x04, 0x68, 0xd8, 0xa4, 0x10},
	{0xd0, 0x60, 0x17, 0x24, 0xd6, 0x04, 0x94, 0x10}, /* Clipreg */
};

static const __u8 initOv7630[] = {
	0x04, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,	/* r01 .. r08 */
	0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* r09 .. r10 */
	0x00, 0x01, 0x01, 0x0a,				/* r11 .. r14 */
	0x28, 0x1e,			/* H & V sizes     r15 .. r16 */
	0x68, 0x8f, MCK_INIT1,				/* r17 .. r19 */
};
static const __u8 ov7630_sensor_init[][8] = {
	{0xa0, 0x21, 0x12, 0x80, 0x00, 0x00, 0x00, 0x10},
	{0xb0, 0x21, 0x01, 0x77, 0x3a, 0x00, 0x00, 0x10},
/*	{0xd0, 0x21, 0x12, 0x7c, 0x01, 0x80, 0x34, 0x10},	   jfm */
	{0xd0, 0x21, 0x12, 0x5c, 0x00, 0x80, 0x34, 0x10},	/* jfm */
	{0xa0, 0x21, 0x1b, 0x04, 0x00, 0x80, 0x34, 0x10},
	{0xa0, 0x21, 0x20, 0x44, 0x00, 0x80, 0x34, 0x10},
	{0xa0, 0x21, 0x23, 0xee, 0x00, 0x80, 0x34, 0x10},
	{0xd0, 0x21, 0x26, 0xa0, 0x9a, 0xa0, 0x30, 0x10},
	{0xb0, 0x21, 0x2a, 0x80, 0x00, 0xa0, 0x30, 0x10},
	{0xb0, 0x21, 0x2f, 0x3d, 0x24, 0xa0, 0x30, 0x10},
	{0xa0, 0x21, 0x32, 0x86, 0x24, 0xa0, 0x30, 0x10},
	{0xb0, 0x21, 0x60, 0xa9, 0x4a, 0xa0, 0x30, 0x10},
/*	{0xb0, 0x21, 0x60, 0xa9, 0x42, 0xa0, 0x30, 0x10},	 * jfm */
	{0xa0, 0x21, 0x65, 0x00, 0x42, 0xa0, 0x30, 0x10},
	{0xa0, 0x21, 0x69, 0x38, 0x42, 0xa0, 0x30, 0x10},
	{0xc0, 0x21, 0x6f, 0x88, 0x0b, 0x00, 0x30, 0x10},
	{0xc0, 0x21, 0x74, 0x21, 0x8e, 0x00, 0x30, 0x10},
	{0xa0, 0x21, 0x7d, 0xf7, 0x8e, 0x00, 0x30, 0x10},
	{0xd0, 0x21, 0x17, 0x1c, 0xbd, 0x06, 0xf6, 0x10},
};

static const __u8 initPas106[] = {
	0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04, 0x01, 0x00,
	0x16, 0x12, 0x24, COMP1, MCK_INIT1,
};
/* compression 0x86 mckinit1 0x2b */

/* "Known" PAS106B registers:
  0x02 clock divider
  0x03 Variable framerate bits 4-11
  0x04 Var framerate bits 0-3, one must leave the 4 msb's at 0 !!
       The variable framerate control must never be set lower then 300,
       which sets the framerate at 90 / reg02, otherwise vsync is lost.
  0x05 Shutter Time Line Offset, this can be used as an exposure control:
       0 = use full frame time, 255 = no exposure at all
       Note this may never be larger then "var-framerate control" / 2 - 2.
       When var-framerate control is < 514, no exposure is reached at the max
       allowed value for the framerate control value, rather then at 255.
  0x06 Shutter Time Pixel Offset, like reg05 this influences exposure, but
       only a very little bit, leave at 0xcd
  0x07 offset sign bit (bit0 1 > negative offset)
  0x08 offset
  0x09 Blue Gain
  0x0a Green1 Gain
  0x0b Green2 Gain
  0x0c Red Gain
  0x0e Global gain
  0x13 Write 1 to commit settings to sensor
*/

static const __u8 pas106_sensor_init[][8] = {
	/* Pixel Clock Divider 6 */
	{ 0xa1, 0x40, 0x02, 0x04, 0x00, 0x00, 0x00, 0x14 },
	/* Frame Time MSB (also seen as 0x12) */
	{ 0xa1, 0x40, 0x03, 0x13, 0x00, 0x00, 0x00, 0x14 },
	/* Frame Time LSB (also seen as 0x05) */
	{ 0xa1, 0x40, 0x04, 0x06, 0x00, 0x00, 0x00, 0x14 },
	/* Shutter Time Line Offset (also seen as 0x6d) */
	{ 0xa1, 0x40, 0x05, 0x65, 0x00, 0x00, 0x00, 0x14 },
	/* Shutter Time Pixel Offset (also seen as 0xb1) */
	{ 0xa1, 0x40, 0x06, 0xcd, 0x00, 0x00, 0x00, 0x14 },
	/* Black Level Subtract Sign (also seen 0x00) */
	{ 0xa1, 0x40, 0x07, 0xc1, 0x00, 0x00, 0x00, 0x14 },
	/* Black Level Subtract Level (also seen 0x01) */
	{ 0xa1, 0x40, 0x08, 0x06, 0x00, 0x00, 0x00, 0x14 },
	{ 0xa1, 0x40, 0x08, 0x06, 0x00, 0x00, 0x00, 0x14 },
	/* Color Gain B Pixel 5 a */
	{ 0xa1, 0x40, 0x09, 0x05, 0x00, 0x00, 0x00, 0x14 },
	/* Color Gain G1 Pixel 1 5 */
	{ 0xa1, 0x40, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x14 },
	/* Color Gain G2 Pixel 1 0 5 */
	{ 0xa1, 0x40, 0x0b, 0x04, 0x00, 0x00, 0x00, 0x14 },
	/* Color Gain R Pixel 3 1 */
	{ 0xa1, 0x40, 0x0c, 0x05, 0x00, 0x00, 0x00, 0x14 },
	/* Color GainH  Pixel */
	{ 0xa1, 0x40, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x14 },
	/* Global Gain */
	{ 0xa1, 0x40, 0x0e, 0x0e, 0x00, 0x00, 0x00, 0x14 },
	/* Contrast */
	{ 0xa1, 0x40, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x14 },
	/* H&V synchro polarity */
	{ 0xa1, 0x40, 0x10, 0x06, 0x00, 0x00, 0x00, 0x14 },
	/* ?default */
	{ 0xa1, 0x40, 0x11, 0x06, 0x00, 0x00, 0x00, 0x14 },
	/* DAC scale */
	{ 0xa1, 0x40, 0x12, 0x06, 0x00, 0x00, 0x00, 0x14 },
	/* ?default */
	{ 0xa1, 0x40, 0x14, 0x02, 0x00, 0x00, 0x00, 0x14 },
	/* Validate Settings */
	{ 0xa1, 0x40, 0x13, 0x01, 0x00, 0x00, 0x00, 0x14 },
};

static const __u8 initPas202[] = {
	0x44, 0x44, 0x21, 0x30, 0x00, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x06, 0x03, 0x0a,
	0x28, 0x1e, 0x20, 0x89, 0x20,
};

/* "Known" PAS202BCB registers:
  0x02 clock divider
  0x04 Variable framerate bits 6-11 (*)
  0x05 Var framerate  bits 0-5, one must leave the 2 msb's at 0 !!
  0x07 Blue Gain
  0x08 Green Gain
  0x09 Red Gain
  0x0b offset sign bit (bit0 1 > negative offset)
  0x0c offset
  0x0e Unknown image is slightly brighter when bit 0 is 0, if reg0f is 0 too,
       leave at 1 otherwise we get a jump in our exposure control
  0x0f Exposure 0-255, 0 = use full frame time, 255 = no exposure at all
  0x10 Master gain 0 - 31
  0x11 write 1 to apply changes
  (*) The variable framerate control must never be set lower then 500
      which sets the framerate at 30 / reg02, otherwise vsync is lost.
*/
static const __u8 pas202_sensor_init[][8] = {
	/* Set the clock divider to 4 -> 30 / 4 = 7.5 fps, we would like
	   to set it lower, but for some reason the bridge starts missing
	   vsync's then */
	{0xa0, 0x40, 0x02, 0x04, 0x00, 0x00, 0x00, 0x10},
	{0xd0, 0x40, 0x04, 0x07, 0x34, 0x00, 0x09, 0x10},
	{0xd0, 0x40, 0x08, 0x01, 0x00, 0x00, 0x01, 0x10},
	{0xd0, 0x40, 0x0c, 0x00, 0x0c, 0x01, 0x32, 0x10},
	{0xd0, 0x40, 0x10, 0x00, 0x01, 0x00, 0x63, 0x10},
	{0xa0, 0x40, 0x15, 0x70, 0x01, 0x00, 0x63, 0x10},
	{0xa0, 0x40, 0x18, 0x00, 0x01, 0x00, 0x63, 0x10},
	{0xa0, 0x40, 0x11, 0x01, 0x01, 0x00, 0x63, 0x10},
	{0xa0, 0x40, 0x03, 0x56, 0x01, 0x00, 0x63, 0x10},
	{0xa0, 0x40, 0x11, 0x01, 0x01, 0x00, 0x63, 0x10},
};

static const __u8 initTas5110c[] = {
	0x44, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x11, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x45, 0x09, 0x0a,
	0x16, 0x12, 0x60, 0x86, 0x2b,
};
/* Same as above, except a different hstart */
static const __u8 initTas5110d[] = {
	0x44, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x11, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x41, 0x09, 0x0a,
	0x16, 0x12, 0x60, 0x86, 0x2b,
};
/* tas5110c is 3 wire, tas5110d is 2 wire (regular i2c) */
static const __u8 tas5110c_sensor_init[][8] = {
	{0x30, 0x11, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x10},
	{0x30, 0x11, 0x02, 0x20, 0xa9, 0x00, 0x00, 0x10},
};
/* Known TAS5110D registers
 * reg02: gain, bit order reversed!! 0 == max gain, 255 == min gain
 * reg03: bit3: vflip, bit4: ~hflip, bit7: ~gainboost (~ == inverted)
 *        Note: writing reg03 seems to only work when written together with 02
 */
static const __u8 tas5110d_sensor_init[][8] = {
	{0xa0, 0x61, 0x9a, 0xca, 0x00, 0x00, 0x00, 0x17}, /* reset */
};

static const __u8 initTas5130[] = {
	0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x11, 0x00, 0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, 0x00, 0x68, 0x0c, 0x0a,
	0x28, 0x1e, 0x60, COMP, MCK_INIT,
};
static const __u8 tas5130_sensor_init[][8] = {
/*	{0x30, 0x11, 0x00, 0x40, 0x47, 0x00, 0x00, 0x10},
					* shutter 0x47 short exposure? */
	{0x30, 0x11, 0x00, 0x40, 0x01, 0x00, 0x00, 0x10},
					/* shutter 0x01 long exposure */
	{0x30, 0x11, 0x02, 0x20, 0x70, 0x00, 0x00, 0x10},
};


static const struct sensor_data sensor_data[] = {
	SENS(initHv7131d, hv7131d_sensor_init, 0, 0),
	SENS(initHv7131r, hv7131r_sensor_init, 0, 0),
	SENS(initOv6650, ov6650_sensor_init, F_SIF, 0x60),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initPas106, pas106_sensor_init, F_SIF, 0),
	SENS(initPas202, pas202_sensor_init, 0, 0),
	SENS(initTas5110c, tas5110c_sensor_init, F_SIF, 0),
	SENS(initTas5110d, tas5110d_sensor_init, F_SIF, 0),
	SENS(initTas5130, tas5130_sensor_init, 0, 0),
};



/* get one byte in gspca_dev->usb_buf */
// DEVICE PLUGGEDED - 4
static void reg_r(struct gspca_dev *gspca_dev,  __u16 value) {

    KDBG("reg_r"," - %d",elgcnt)

	int res;

	if (gspca_dev->usb_err < 0)
		return;

	res = usb_control_msg(gspca_dev->dev,
			usb_rcvctrlpipe(gspca_dev->dev, 0),
			0,			/* request */
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			value,
			0,			/* index */
			gspca_dev->usb_buf, 1,
			500);

	if (res < 0) {
		dev_err(gspca_dev->v4l2_dev.dev,
			"Error reading register %02x: %d\n", value, res);
		gspca_dev->usb_err = res;
		/*
		 * Make sure the result is zeroed to avoid uninitialized
		 * values.
		 */
		gspca_dev->usb_buf[0] = 0;
	}
}

// DEVICE PLUGGEDED - 6
static void reg_w(struct gspca_dev *gspca_dev, __u16 value, const __u8 *buffer, int len) {

    KDBG("reg_w"," - %d",elgcnt)

	int res;

	if (gspca_dev->usb_err < 0)
		return;

	memcpy(gspca_dev->usb_buf, buffer, len);
	res = usb_control_msg(gspca_dev->dev,
			usb_sndctrlpipe(gspca_dev->dev, 0),
			0x08,			/* request */
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			value,
			0,			/* index */
			gspca_dev->usb_buf, len,
			500);

	if (res < 0) {
		dev_err(gspca_dev->v4l2_dev.dev, "Error writing register %02x: %d\n", value, res);
		gspca_dev->usb_err = res;
	}
}

static void i2c_w(struct gspca_dev *gspca_dev, const u8 *buf) {

    KDBG("i2c_w"," - %d",elgcnt)

	int retry = 60;

	if (gspca_dev->usb_err < 0)
		return;

	/* is i2c ready */
	reg_w(gspca_dev, 0x08, buf, 8);
	while (retry--) {
		if (gspca_dev->usb_err < 0)
			return;
		msleep(1);
		reg_r(gspca_dev, 0x08);
		if (gspca_dev->usb_buf[0] & 0x04) {
			if (gspca_dev->usb_buf[0] & 0x08) {
				dev_err(gspca_dev->v4l2_dev.dev, "i2c error writing %8ph\n", buf);
				gspca_dev->usb_err = -EIO;
			}
			return;
		}
	}

	dev_err(gspca_dev->v4l2_dev.dev, "i2c write timeout\n");
	gspca_dev->usb_err = -EIO;
}

static void i2c_w_vector(struct gspca_dev *gspca_dev, const __u8 buffer[][8], int len) {

    KDBG("i2c_w_vector"," - %d",elgcnt)

	for (;;) {
		if (gspca_dev->usb_err < 0)
			return;
		i2c_w(gspca_dev, *buffer);
		len -= 8;
		if (len <= 0)
			break;
		buffer++;
	}
}

static void setbrightness(struct gspca_dev *gspca_dev)
{
    KDBG("setbrightness"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;

	switch (sd->sensor) {
	case  SENSOR_OV6650:
	case  SENSOR_OV7630: {
		__u8 i2cOV[] =
			{0xa0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x10};

		/* change reg 0x06 */
		i2cOV[1] = sensor_data[sd->sensor].sensor_addr;
		i2cOV[3] = sd->brightness->val;
		i2c_w(gspca_dev, i2cOV);
		break;
	}
	case SENSOR_PAS106:
	case SENSOR_PAS202: {
		__u8 i2cpbright[] =
			{0xb0, 0x40, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x16};
		__u8 i2cpdoit[] =
			{0xa0, 0x40, 0x11, 0x01, 0x00, 0x00, 0x00, 0x16};

		/* PAS106 uses reg 7 and 8 instead of b and c */
		if (sd->sensor == SENSOR_PAS106) {
			i2cpbright[2] = 7;
			i2cpdoit[2] = 0x13;
		}

		if (sd->brightness->val < 127) {
			/* change reg 0x0b, signreg */
			i2cpbright[3] = 0x01;
			/* set reg 0x0c, offset */
			i2cpbright[4] = 127 - sd->brightness->val;
		} else
			i2cpbright[4] = sd->brightness->val - 127;

		i2c_w(gspca_dev, i2cpbright);
		i2c_w(gspca_dev, i2cpdoit);
		break;
	}
	default:
		break;
	}
}

static void setgain(struct gspca_dev *gspca_dev)
{
    KDBG("setgain"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;
	u8 gain = gspca_dev->gain->val;

	switch (sd->sensor) {
	case SENSOR_HV7131D: {
		__u8 i2c[] = {0xc0, 0x11, 0x31, 0x00, 0x00, 0x00, 0x00, 0x17};

		i2c[3] = 0x3f - gain;
		i2c[4] = 0x3f - gain;
		i2c[5] = 0x3f - gain;

		i2c_w(gspca_dev, i2c);
		break;
	}
	case SENSOR_TAS5110C:
	case SENSOR_TAS5130CXX: {
		__u8 i2c[] = {0x30, 0x11, 0x02, 0x20, 0x70, 0x00, 0x00, 0x10};

		i2c[4] = 255 - gain;
		i2c_w(gspca_dev, i2c);
		break;
	}
	case SENSOR_TAS5110D: {
		__u8 i2c[] = {
			0xb0, 0x61, 0x02, 0x00, 0x10, 0x00, 0x00, 0x17 };
		gain = 255 - gain;
		/* The bits in the register are the wrong way around!! */
		i2c[3] |= (gain & 0x80) >> 7;
		i2c[3] |= (gain & 0x40) >> 5;
		i2c[3] |= (gain & 0x20) >> 3;
		i2c[3] |= (gain & 0x10) >> 1;
		i2c[3] |= (gain & 0x08) << 1;
		i2c[3] |= (gain & 0x04) << 3;
		i2c[3] |= (gain & 0x02) << 5;
		i2c[3] |= (gain & 0x01) << 7;
		i2c_w(gspca_dev, i2c);
		break;
	}
	case SENSOR_OV6650:
	case SENSOR_OV7630: {
		__u8 i2c[] = {0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};

		/*
		 * The ov7630's gain is weird, at 32 the gain drops to the
		 * same level as at 16, so skip 32-47 (of the 0-63 scale).
		 */
		if (sd->sensor == SENSOR_OV7630 && gain >= 32)
			gain += 16;

		i2c[1] = sensor_data[sd->sensor].sensor_addr;
		i2c[3] = gain;
		i2c_w(gspca_dev, i2c);
		break;
	}
	case SENSOR_PAS106:
	case SENSOR_PAS202: {
		__u8 i2cpgain[] =
			{0xa0, 0x40, 0x10, 0x00, 0x00, 0x00, 0x00, 0x15};
		__u8 i2cpcolorgain[] =
			{0xc0, 0x40, 0x07, 0x00, 0x00, 0x00, 0x00, 0x15};
		__u8 i2cpdoit[] =
			{0xa0, 0x40, 0x11, 0x01, 0x00, 0x00, 0x00, 0x16};

		/* PAS106 uses different regs (and has split green gains) */
		if (sd->sensor == SENSOR_PAS106) {
			i2cpgain[2] = 0x0e;
			i2cpcolorgain[0] = 0xd0;
			i2cpcolorgain[2] = 0x09;
			i2cpdoit[2] = 0x13;
		}

		i2cpgain[3] = gain;
		i2cpcolorgain[3] = gain >> 1;
		i2cpcolorgain[4] = gain >> 1;
		i2cpcolorgain[5] = gain >> 1;
		i2cpcolorgain[6] = gain >> 1;

		i2c_w(gspca_dev, i2cpgain);
		i2c_w(gspca_dev, i2cpcolorgain);
		i2c_w(gspca_dev, i2cpdoit);
		break;
	}
	default:
		if (sd->bridge == BRIDGE_103) {
			u8 buf[3] = { gain, gain, gain }; /* R, G, B */
			reg_w(gspca_dev, 0x05, buf, 3);
		} else {
			u8 buf[2];
			buf[0] = gain << 4 | gain; /* Red and blue */
			buf[1] = gain; /* Green */
			reg_w(gspca_dev, 0x10, buf, 2);
		}
	}
}

static void setexposure(struct gspca_dev *gspca_dev)
{
    KDBG("setexposure"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;

	switch (sd->sensor) {
	case SENSOR_HV7131D: {
		/* Note the datasheet wrongly says line mode exposure uses reg
		   0x26 and 0x27, testing has shown 0x25 + 0x26 */
		__u8 i2c[] = {0xc0, 0x11, 0x25, 0x00, 0x00, 0x00, 0x00, 0x17};
		u16 reg = gspca_dev->exposure->val;

		i2c[3] = reg >> 8;
		i2c[4] = reg & 0xff;
		i2c_w(gspca_dev, i2c);
		break;
	}
	case SENSOR_TAS5110C:
	case SENSOR_TAS5110D: {
		/* register 19's high nibble contains the sn9c10x clock divider
		   The high nibble configures the no fps according to the
		   formula: 60 / high_nibble. With a maximum of 30 fps */
		u8 reg = gspca_dev->exposure->val;

		reg = (reg << 4) | 0x0b;
		reg_w(gspca_dev, 0x19, &reg, 1);
		break;
	}
	case SENSOR_OV6650:
	case SENSOR_OV7630: {
		/* The ov6650 / ov7630 have 2 registers which both influence
		   exposure, register 11, whose low nibble sets the nr off fps
		   according to: fps = 30 / (low_nibble + 1)

		   The fps configures the maximum exposure setting, but it is
		   possible to use less exposure then what the fps maximum
		   allows by setting register 10. register 10 configures the
		   actual exposure as quotient of the full exposure, with 0
		   being no exposure at all (not very useful) and reg10_max
		   being max exposure possible at that framerate.

		   The code maps our 0 - 510 ms exposure ctrl to these 2
		   registers, trying to keep fps as high as possible.
		*/
		__u8 i2c[] = {0xb0, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10};
		int reg10, reg11, reg10_max;

		/* ov6645 datasheet says reg10_max is 9a, but that uses
		   tline * 2 * reg10 as formula for calculating texpo, the
		   ov6650 probably uses the same formula as the 7730 which uses
		   tline * 4 * reg10, which explains why the reg10max we've
		   found experimentally for the ov6650 is exactly half that of
		   the ov6645. The ov7630 datasheet says the max is 0x41. */
		if (sd->sensor == SENSOR_OV6650) {
			reg10_max = 0x4d;
			i2c[4] = 0xc0; /* OV6650 needs non default vsync pol */
		} else
			reg10_max = 0x41;

		reg11 = (15 * gspca_dev->exposure->val + 999) / 1000;

		if (reg11 < 1)
			reg11 = 1;
		else if (reg11 > 16)
			reg11 = 16;

		/* In 640x480, if the reg11 has less than 4, the image is
		   unstable (the bridge goes into a higher compression mode
		   which we have not reverse engineered yet). */
		if (gspca_dev->pixfmt.width == 640 && reg11 < 4)
			reg11 = 1;  // josemar: valor original 4, porém framerate ficava 8. Porém, se setado para 1,FR = 30

		/* frame exposure time in ms = 1000 * reg11 / 30    ->
		reg10 = (gspca_dev->exposure->val / 2) * reg10_max
				/ (1000 * reg11 / 30) */
		reg10 = (gspca_dev->exposure->val * 15 * reg10_max)
				/ (1000 * reg11);

		/* Don't allow this to get below 10 when using autogain, the
		   steps become very large (relatively) when below 10 causing
		   the image to oscillate from much too dark, to much too bright
		   and back again. */
		if (gspca_dev->autogain->val && reg10 < 10)
			reg10 = 10;
		else if (reg10 > reg10_max)
			reg10 = reg10_max;

		/* Write reg 10 and reg11 low nibble */
		i2c[1] = sensor_data[sd->sensor].sensor_addr;
		i2c[3] = reg10;
		i2c[4] |= reg11 - 1;

		/* If register 11 didn't change, don't change it */
		if (sd->reg11 == reg11)
			i2c[0] = 0xa0;

		i2c_w(gspca_dev, i2c);
		if (gspca_dev->usb_err == 0)
			sd->reg11 = reg11;
		break;
	}
	case SENSOR_PAS202: {
		__u8 i2cpframerate[] =
			{0xb0, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00, 0x16};
		__u8 i2cpexpo[] =
			{0xa0, 0x40, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x16};
		const __u8 i2cpdoit[] =
			{0xa0, 0x40, 0x11, 0x01, 0x00, 0x00, 0x00, 0x16};
		int framerate_ctrl;

		/* The exposure knee for the autogain algorithm is 200
		   (100 ms / 10 fps on other sensors), for values below this
		   use the control for setting the partial frame expose time,
		   above that use variable framerate. This way we run at max
		   framerate (640x480@7.5 fps, 320x240@10fps) until the knee
		   is reached. Using the variable framerate control above 200
		   is better then playing around with both clockdiv + partial
		   frame exposure times (like we are doing with the ov chips),
		   as that sometimes leads to jumps in the exposure control,
		   which are bad for auto exposure. */
		if (gspca_dev->exposure->val < 200) {
			i2cpexpo[3] = 255 - (gspca_dev->exposure->val * 255)
						/ 200;
			framerate_ctrl = 500;
		} else {
			/* The PAS202's exposure control goes from 0 - 4095,
			   but anything below 500 causes vsync issues, so scale
			   our 200-1023 to 500-4095 */
			framerate_ctrl = (gspca_dev->exposure->val - 200)
							* 1000 / 229 +  500;
		}

		i2cpframerate[3] = framerate_ctrl >> 6;
		i2cpframerate[4] = framerate_ctrl & 0x3f;
		i2c_w(gspca_dev, i2cpframerate);
		i2c_w(gspca_dev, i2cpexpo);
		i2c_w(gspca_dev, i2cpdoit);
		break;
	}
	case SENSOR_PAS106: {
		__u8 i2cpframerate[] =
			{0xb1, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00, 0x14};
		__u8 i2cpexpo[] =
			{0xa1, 0x40, 0x05, 0x00, 0x00, 0x00, 0x00, 0x14};
		const __u8 i2cpdoit[] =
			{0xa1, 0x40, 0x13, 0x01, 0x00, 0x00, 0x00, 0x14};
		int framerate_ctrl;

		/* For values below 150 use partial frame exposure, above
		   that use framerate ctrl */
		if (gspca_dev->exposure->val < 150) {
			i2cpexpo[3] = 150 - gspca_dev->exposure->val;
			framerate_ctrl = 300;
		} else {
			/* The PAS106's exposure control goes from 0 - 4095,
			   but anything below 300 causes vsync issues, so scale
			   our 150-1023 to 300-4095 */
			framerate_ctrl = (gspca_dev->exposure->val - 150) * 1000 / 230 + 300;
		}

		i2cpframerate[3] = framerate_ctrl >> 4;
		i2cpframerate[4] = framerate_ctrl & 0x0f;
		i2c_w(gspca_dev, i2cpframerate);
		i2c_w(gspca_dev, i2cpexpo);
		i2c_w(gspca_dev, i2cpdoit);
		break;
	}
	default:
		break;
	}
}

static void setfreq(struct gspca_dev *gspca_dev)
{
    KDBG("setfreq"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV6650 || sd->sensor == SENSOR_OV7630) {
		/* Framerate adjust register for artificial light 50 hz flicker
		   compensation, for the ov6650 this is identical to ov6630
		   0x2b register, see ov6630 datasheet.
		   0x4f / 0x8a -> (30 fps -> 25 fps), 0x00 -> no adjustment */
		__u8 i2c[] = {0xa0, 0x00, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x10};
		switch (sd->plfreq->val) {
		default:
/*		case 0:			 * no filter*/
/*		case 2:			 * 60 hz */
			i2c[3] = 0;
			break;
		case 1:			/* 50 hz */
			i2c[3] = (sd->sensor == SENSOR_OV6650)? 0x4f : 0x8a;
			break;
		}
		i2c[1] = sensor_data[sd->sensor].sensor_addr;
		i2c_w(gspca_dev, i2c);
	}
}

static void do_autogain(struct gspca_dev *gspca_dev)
{
    KDBG("do_autogain"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;
	int deadzone, desired_avg_lum, avg_lum;

	avg_lum = atomic_read(&sd->avg_lum);
	if (avg_lum == -1)
		return;

	if (sd->autogain_ignore_frames > 0) {
		sd->autogain_ignore_frames--;
		return;
	}

	/* SIF / VGA sensors have a different autoexposure area and thus
	   different avg_lum values for the same picture brightness */
	if (sensor_data[sd->sensor].flags & F_SIF) {
		deadzone = 500;
		/* SIF sensors tend to overexpose, so keep this small */
		desired_avg_lum = 5000;
	} else {
		deadzone = 1500;
		desired_avg_lum = 13000;
	}

	if (sd->brightness)
		desired_avg_lum = sd->brightness->val * desired_avg_lum / 127;

	if (gspca_dev->exposure->maximum < 500) {
		if (gspca_coarse_grained_expo_autogain(gspca_dev, avg_lum, desired_avg_lum, deadzone))
			sd->autogain_ignore_frames = AUTOGAIN_IGNORE_FRAMES;
	} else {
		int gain_knee = (s32)gspca_dev->gain->maximum * 9 / 10;
		if (gspca_expo_autogain(gspca_dev, avg_lum, desired_avg_lum, deadzone, gain_knee, sd->exposure_knee))
			sd->autogain_ignore_frames = AUTOGAIN_IGNORE_FRAMES;
	}
}



/* this function is called at probe time */
// DEVICE PLUGGEDED - 3
static int sd_config(struct gspca_dev *gspca_dev, const struct usb_device_id *id) {

    KDBG("sd_config"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam;

	reg_r(gspca_dev, 0x00);
	if (gspca_dev->usb_buf[0] != 0x10)
		return -ENODEV;

	/* copy the webcam info from the device id */
	sd->sensor = id->driver_info >> 8;
	sd->bridge = id->driver_info & 0xff;

	cam = &gspca_dev->cam;
	if (!(sensor_data[sd->sensor].flags & F_SIF)) {
		cam->cam_mode = vga_mode;
		cam->nmodes = ARRAY_SIZE(vga_mode);
	} else {
		cam->cam_mode = sif_mode;
		cam->nmodes = ARRAY_SIZE(sif_mode);
	}
	cam->npkt = 36;			/* 36 packets per ISOC message */

	return 0;
}

/* this function is called at probe and resume time */
// DEVICE PLUGGEDED - 5
static int sd_init(struct gspca_dev *gspca_dev) {

    KDBG("sd_init"," - %d",elgcnt)

	const __u8 stop = 0x09; /* Disable stream turn of LED */

	reg_w(gspca_dev, 0x01, &stop, 1);

	return gspca_dev->usb_err;
}

// DEVICE PLUGGEDED - 9 - 11
static int sd_s_ctrl(struct v4l2_ctrl *ctrl)
{
    KDBG("sd_s_ctrl"," - %d",elgcnt)

	struct gspca_dev *gspca_dev = container_of(ctrl->handler, struct gspca_dev, ctrl_handler);
	struct sd *sd = (struct sd *)gspca_dev;

	gspca_dev->usb_err = 0;

	if (ctrl->id == V4L2_CID_AUTOGAIN && ctrl->is_new && ctrl->val) {
		/* when switching to autogain set defaults to make sure
		   we are on a valid point of the autogain gain /
		   exposure knee graph, and give this change time to
		   take effect before doing autogain. */
		gspca_dev->gain->val = gspca_dev->gain->default_value;
		gspca_dev->exposure->val = gspca_dev->exposure->default_value;
		sd->autogain_ignore_frames = AUTOGAIN_IGNORE_FRAMES;
	}

	if (!gspca_dev->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		setbrightness(gspca_dev);
		break;
	case V4L2_CID_AUTOGAIN:
		if (gspca_dev->exposure->is_new || (ctrl->is_new && ctrl->val))
			setexposure(gspca_dev);
		if (gspca_dev->gain->is_new || (ctrl->is_new && ctrl->val))
			setgain(gspca_dev);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		setfreq(gspca_dev);
		break;
	default:
		return -EINVAL;
	}
	return gspca_dev->usb_err;
}



static const struct v4l2_ctrl_ops sd_ctrl_ops = {
	.s_ctrl = sd_s_ctrl,
};



/* this function is called at probe time */
// DEVICE PLUGGEDED - 7
static int sd_init_controls(struct gspca_dev *gspca_dev)
{
    KDBG("sd_init_controls"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;
	struct v4l2_ctrl_handler *hdl = &gspca_dev->ctrl_handler;

	gspca_dev->vdev.ctrl_handler = hdl;
	v4l2_ctrl_handler_init(hdl, 5);

	if (sd->sensor == SENSOR_OV6650 || sd->sensor == SENSOR_OV7630 || sd->sensor == SENSOR_PAS106 || sd->sensor == SENSOR_PAS202)
		sd->brightness = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 255, 1, 127);

	/* Gain range is sensor dependent */
	switch (sd->sensor) {
	case SENSOR_OV6650:
	case SENSOR_PAS106:
	case SENSOR_PAS202:
		gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 31, 1, 15);
		break;
	case SENSOR_OV7630:
		gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 47, 1, 31);
		break;
	case SENSOR_HV7131D:
		gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 63, 1, 31);
		break;
	case SENSOR_TAS5110C:
	case SENSOR_TAS5110D:
	case SENSOR_TAS5130CXX:
		gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 255, 1, 127);
		break;
	default:
		if (sd->bridge == BRIDGE_103) {
			gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 127, 1, 63);
		} else {
			gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 15, 1, 7);
		}
	}

	/* Exposure range is sensor dependent, and not all have exposure */
	switch (sd->sensor) {
	case SENSOR_HV7131D:
		gspca_dev->exposure = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_EXPOSURE, 0, 8191, 1, 482);
		sd->exposure_knee = 964;
		break;
	case SENSOR_OV6650:
	case SENSOR_OV7630:
	case SENSOR_PAS106:
	case SENSOR_PAS202:
		gspca_dev->exposure = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_EXPOSURE, 0, 1023, 1, 66);
		sd->exposure_knee = 200;
		break;
	case SENSOR_TAS5110C:
	case SENSOR_TAS5110D:
		gspca_dev->exposure = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_EXPOSURE, 2, 15, 1, 2);
		break;
	}

	if (gspca_dev->exposure) {
		gspca_dev->autogain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	}

	if (sd->sensor == SENSOR_OV6650 || sd->sensor == SENSOR_OV7630)
		sd->plfreq = v4l2_ctrl_new_std_menu(hdl, &sd_ctrl_ops, V4L2_CID_POWER_LINE_FREQUENCY, V4L2_CID_POWER_LINE_FREQUENCY_60HZ, 0, V4L2_CID_POWER_LINE_FREQUENCY_DISABLED);

	if (hdl->error) {
		pr_err("Could not initialize controls\n");
		return hdl->error;
	}

	if (gspca_dev->autogain)
		v4l2_ctrl_auto_cluster(3, &gspca_dev->autogain, 0, false);

	return 0;
}


/* -- start the camera -- */
static int sd_start(struct gspca_dev *gspca_dev)
{
    KDBG("sd_start"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam = &gspca_dev->cam;
	int i, mode;
	__u8 regs[0x31]; // josemar: Em sn9n102.pdf tem a descricao de 0x1Fh registradores. Sugere que regs pertence ao controlador sn9c102/103

	mode = cam->cam_mode[gspca_dev->curr_mode].priv & 0x07;
	/* Copy registers 0x01 - 0x19 from the template */
	memcpy(&regs[0x01], sensor_data[sd->sensor].bridge_init, 0x19);
	/* Set the mode */
	regs[0x18] |= mode << 4;

	/* Set bridge gain to 1.0 */
	if (sd->bridge == BRIDGE_103) {
		regs[0x05] = 0x20; /* Red */
		regs[0x06] = 0x20; /* Green */
		regs[0x07] = 0x20; /* Blue */
	} else {
		regs[0x10] = 0x00; /* Red and blue */
		regs[0x11] = 0x00; /* Green */
	}

	/* Setup pixel numbers and auto exposure window */
	if (sensor_data[sd->sensor].flags & F_SIF) {
		regs[0x1a] = 0x14; /* HO_SIZE 640, makes no sense */
		regs[0x1b] = 0x0a; /* VO_SIZE 320, makes no sense */
		regs[0x1c] = 0x02; /* AE H-start 64 */
		regs[0x1d] = 0x02; /* AE V-start 64 */
		regs[0x1e] = 0x09; /* AE H-end 288 */
		regs[0x1f] = 0x07; /* AE V-end 224 */
	} else {
		regs[0x1a] = 0x1d; /* HO_SIZE 960, makes no sense */
		regs[0x1b] = 0x10; /* VO_SIZE 512, makes no sense */
		regs[0x1c] = 0x05; /* AE H-start 160 */
		regs[0x1d] = 0x03; /* AE V-start 96 */
		regs[0x1e] = 0x0f; /* AE H-end 480 */
		regs[0x1f] = 0x0c; /* AE V-end 384 */
	}

	/* Setup the gamma table (only used with the sn9c103 bridge) */
	for (i = 0; i < 16; i++)
		regs[0x20 + i] = i * 16;
	regs[0x20 + i] = 255;

	/* Special cases where some regs depend on mode or bridge */
	switch (sd->sensor) {
	case SENSOR_TAS5130CXX:
		/* FIXME / TESTME
		   probably not mode specific at all most likely the upper
		   nibble of 0x19 is exposure (clock divider) just as with
		   the tas5110, we need someone to test this. */
		regs[0x19] = mode ? 0x23 : 0x43;
		break;
	case SENSOR_OV7630:
		/* FIXME / TESTME for some reason with the 101/102 bridge the
		   clock is set to 12 Mhz (reg1 == 0x04), rather then 24.
		   Also the hstart needs to go from 1 to 2 when using a 103,
		   which is likely related. This does not seem right. */
		if (sd->bridge == BRIDGE_103) {
			regs[0x01] = 0x44; /* Select 24 Mhz clock */
			regs[0x12] = 0x02; /* Set hstart to 2 */
		}
		break;
	case SENSOR_PAS202:
		/* For some unknown reason we need to increase hstart by 1 on
		   the sn9c103, otherwise we get wrong colors (bayer shift). */
		if (sd->bridge == BRIDGE_103)
			regs[0x12] += 1;
		break;
	}
	/* Disable compression when the raw bayer format has been selected */
	if (cam->cam_mode[gspca_dev->curr_mode].priv & MODE_RAW)
		regs[0x18] &= ~0x80;

	/* Vga mode emulation on SIF sensor? */
	if (cam->cam_mode[gspca_dev->curr_mode].priv & MODE_REDUCED_SIF) {
		regs[0x12] += 16;	/* hstart adjust */
		regs[0x13] += 24;	/* vstart adjust */
		regs[0x15]  = 320 / 16; /* hsize */
		regs[0x16]  = 240 / 16; /* vsize */
	}

	/* reg 0x01 bit 2 video transfert on */
	reg_w(gspca_dev, 0x01, &regs[0x01], 1);
	/* reg 0x17 SensorClk enable inv Clk 0x60 */
	reg_w(gspca_dev, 0x17, &regs[0x17], 1);
	/* Set the registers from the template */
	reg_w(gspca_dev, 0x01, &regs[0x01], (sd->bridge == BRIDGE_103) ? 0x30 : 0x1f); // josemar tamanho da tabela de registradores: 0x1F = SN9C102 e 0x30 = SN9C103

	/* Init the sensor */
	i2c_w_vector(gspca_dev, sensor_data[sd->sensor].sensor_init, sensor_data[sd->sensor].sensor_init_size);

	/* Mode / bridge specific sensor setup */
	switch (sd->sensor) {
	case SENSOR_PAS202: {
		const __u8 i2cpclockdiv[] = {0xa0, 0x40, 0x02, 0x03, 0x00, 0x00, 0x00, 0x10};
		/* clockdiv from 4 to 3 (7.5 -> 10 fps) when in low res mode */
		if (mode)
			i2c_w(gspca_dev, i2cpclockdiv);
		break;
	    }
	case SENSOR_OV7630:
		/* FIXME / TESTME We should be able to handle this identical
		   for the 101/102 and the 103 case */
		if (sd->bridge == BRIDGE_103) {
			const __u8 i2c[] = { 0xa0, 0x21, 0x13, 0x80, 0x00, 0x00, 0x00, 0x10 };
			i2c_w(gspca_dev, i2c);
		}
		break;
	}
	/* H_size V_size 0x28, 0x1e -> 640x480. 0x16, 0x12 -> 352x288 */
	reg_w(gspca_dev, 0x15, &regs[0x15], 2);
	/* compression register */
	reg_w(gspca_dev, 0x18, &regs[0x18], 1);
	/* H_start */
	reg_w(gspca_dev, 0x12, &regs[0x12], 1);
	/* V_START */
	reg_w(gspca_dev, 0x13, &regs[0x13], 1);
	/* reset 0x17 SensorClk enable inv Clk 0x60 */
				/*fixme: ov7630 [17]=68 8f (+20 if 102)*/
	reg_w(gspca_dev, 0x17, &regs[0x17], 1);
	/*MCKSIZE ->3 */	/*fixme: not ov7630*/
	reg_w(gspca_dev, 0x19, &regs[0x19], 1);
	/* AE_STRX AE_STRY AE_ENDX AE_ENDY */
	reg_w(gspca_dev, 0x1c, &regs[0x1c], 4);
	/* Enable video transfert */
	reg_w(gspca_dev, 0x01, &regs[0x01], 1);
	/* Compression */
	reg_w(gspca_dev, 0x18, &regs[0x18], 2);
	msleep(20);

	sd->reg11 = -1;

	setgain(gspca_dev);
	setbrightness(gspca_dev);
	setexposure(gspca_dev);
	setfreq(gspca_dev);

	sd->frames_to_drop = 0;
	sd->autogain_ignore_frames = 0;
	gspca_dev->exp_too_high_cnt = 0;
	gspca_dev->exp_too_low_cnt = 0;
	atomic_set(&sd->avg_lum, -1);
	return gspca_dev->usb_err;
}

static void sd_stopN(struct gspca_dev *gspca_dev)
{
    KDBG("sd_stopN"," - %d",elgcnt)

	sd_init(gspca_dev);
}

static u8* find_sof(struct gspca_dev *gspca_dev, u8 *data, int len)
{
    KDBG("find_sof"," - %d",elgcnt)

	struct sd *sd = (struct sd *) gspca_dev;
	int i, header_size = (sd->bridge == BRIDGE_103) ? 18 : 12;

	/* frames start with:
	 *	ff ff 00 c4 c4 96	synchro
	 *	00		(unknown)
	 *	xx		(frame sequence / size / compression)
	 *	(xx)		(idem - extra byte for sn9c103)
	 *	ll mm		brightness sum inside auto exposure
	 *	ll mm		brightness sum outside auto exposure
	 *	(xx xx xx xx xx)	audio values for snc103
	 */
	for (i = 0; i < len; i++) {
		switch (sd->header_read) {
		case 0:
			if (data[i] == 0xff)
				sd->header_read++;
			break;
		case 1:
			if (data[i] == 0xff)
				sd->header_read++;
			else
				sd->header_read = 0;
			break;
		case 2:
			if (data[i] == 0x00)
				sd->header_read++;
			else if (data[i] != 0xff)
				sd->header_read = 0;
			break;
		case 3:
			if (data[i] == 0xc4)
				sd->header_read++;
			else if (data[i] == 0xff)
				sd->header_read = 1;
			else
				sd->header_read = 0;
			break;
		case 4:
			if (data[i] == 0xc4)
				sd->header_read++;
			else if (data[i] == 0xff)
				sd->header_read = 1;
			else
				sd->header_read = 0;
			break;
		case 5:
			if (data[i] == 0x96)
				sd->header_read++;
			else if (data[i] == 0xff)
				sd->header_read = 1;
			else
				sd->header_read = 0;
			break;
		default:
			sd->header[sd->header_read - 6] = data[i];
			sd->header_read++;
			if (sd->header_read == header_size) {
				sd->header_read = 0;
				return data + i + 1;
			}
		}
	}
	return NULL;
}




static void sd_pkt_scan(struct gspca_dev *gspca_dev,
			u8 *data,			/* isoc packet */
			int len)			/* iso packet length */
{
    KDBG("sd_pkt_scan"," - %d",elgcnt)

	int fr_h_sz = 0, lum_offset = 0, len_after_sof = 0;
	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam = &gspca_dev->cam;
	u8 *sof;

	sof = find_sof(gspca_dev, data, len);
	if (sof) {
		if (sd->bridge == BRIDGE_103) {
			fr_h_sz = 18;
			lum_offset = 3;
		} else {
			fr_h_sz = 12;
			lum_offset = 2;
		}

		len_after_sof = len - (sof - data);
		len = (sof - data) - fr_h_sz;
		if (len < 0)
			len = 0;
	}

	if (cam->cam_mode[gspca_dev->curr_mode].priv & MODE_RAW) {
		/* In raw mode we sometimes get some garbage after the frame
		   ignore this */
		int used;
		int size = cam->cam_mode[gspca_dev->curr_mode].sizeimage;

		used = gspca_dev->image_len;
		if (used + len > size)
			len = size - used;
	}

	gspca_frame_add(gspca_dev, INTER_PACKET, data, len);

	if (sof) {
		int  lum = sd->header[lum_offset] + (sd->header[lum_offset + 1] << 8);

		/* When exposure changes midway a frame we
		   get a lum of 0 in this case drop 2 frames
		   as the frames directly after an exposure
		   change have an unstable image. Sometimes lum
		   *really* is 0 (cam used in low light with
		   low exposure setting), so do not drop frames
		   if the previous lum was 0 too. */
		if (lum == 0 && sd->prev_avg_lum != 0) {
			lum = -1;
			sd->frames_to_drop = 2;
			sd->prev_avg_lum = 0;
		} else
			sd->prev_avg_lum = lum;
		atomic_set(&sd->avg_lum, lum);

		if (sd->frames_to_drop)
			sd->frames_to_drop--;
		else
			gspca_frame_add(gspca_dev, LAST_PACKET, NULL, 0);

		gspca_frame_add(gspca_dev, FIRST_PACKET, sof, len_after_sof);
	}
}

#if IS_ENABLED(CONFIG_INPUT)
static int sd_int_pkt_scan(struct gspca_dev *gspca_dev,
			u8 *data,		/* interrupt packet data */
			int len)		/* interrupt packet length */
{
    KDBG("sd_int_pkt_scan"," - %d",elgcnt)

	int ret = -EINVAL;

	if (len == 1 && data[0] == 1) {
		input_report_key(gspca_dev->input_dev, KEY_CAMERA, 1);
		input_sync(gspca_dev->input_dev);
		input_report_key(gspca_dev->input_dev, KEY_CAMERA, 0);
		input_sync(gspca_dev->input_dev);
		ret = 0;
	}

	return ret;
}
#endif




/* sub-driver description */

static const struct sd_desc sd_desc = {
	.name = MODULE_NAME,
	.config = sd_config,
	.init = sd_init,
	.init_controls = sd_init_controls,
	.start = sd_start,
	.stopN = sd_stopN,
	.pkt_scan = sd_pkt_scan,
	.dq_callback = do_autogain,
#if IS_ENABLED(CONFIG_INPUT)
	.int_pkt_scan = sd_int_pkt_scan,
#endif
};


/* -- module initialisation -- */
#define SB(sensor, bridge) .driver_info = (SENSOR_ ## sensor << 8) | BRIDGE_ ## bridge


static const struct usb_device_id device_table[] = {
	{USB_DEVICE(0x0c45, 0x608f), SB(OV7630, 103)},
	{}
};
MODULE_DEVICE_TABLE(usb, device_table);



/* -- device connect -- */
// DEVICE PLUGGEDED - 0
static int sd_probe(struct usb_interface *intf,	const struct usb_device_id *id)
{
    KDBG("sd_probe"," - %d",elgcnt)

	return gspca_dev_probe(intf, id, &sd_desc, sizeof(struct sd), THIS_MODULE);
}




static struct usb_driver sd_driver = {
	.name = MODULE_NAME,
	.id_table = device_table,
	.probe = sd_probe,
	.disconnect = gspca_disconnect,
#ifdef CONFIG_PM
	.suspend = gspca_suspend,
	.resume = gspca_resume,
	.reset_resume = gspca_resume,
#endif
};




module_param_named(debug, gspca_debug, int, 0644);
MODULE_PARM_DESC(debug, "1:probe 2:config 3:stream 4:frame 5:packet 6:usbi 7:usbo");



// ATENÇÃO:     ESTA OPÇÃO "module_usb_driver(sd_driver)" PERMITIU O LINK ENTRE OUTROS MÓDULOS DE DRIVER AUTOMATICAMENTE
/**************************/
module_usb_driver(sd_driver);
/**************************/




/*     OBSERVAÇOES   */
/*


[ 5828.218070] Elgin_sn9c103: sd_probe - 0
[ 5828.218072] Elgin_sn9c103: gspca_dev_probe - 1
[ 5828.218072] Elgin_sn9c103: gspca_dev_probe2 - 2
[ 5828.218076] Elgin_sn9c103: sd_config - 3
[ 5828.218077] Elgin_sn9c103: reg_r - 4
[ 5828.218871] Elgin_sn9c103: sd_init - 5
[ 5828.218873] Elgin_sn9c103: reg_w - 6
[ 5828.218967] Elgin_sn9c103: sd_init_controls - 7
[ 5828.218977] Elgin_sn9c103: gspca_set_default_mode - 8
[ 5828.218978] Elgin_sn9c103: sd_s_ctrl - 9
[ 5828.218979] Elgin_sn9c103: sd_s_ctrl - 10
[ 5828.218980] Elgin_sn9c103: sd_s_ctrl - 11
[ 5828.218981] Elgin_sn9c103: gspca_input_connect - 12
[ 5828.219880] Elgin_sn9c103: gspca_input_create_urb - 13
[ 5828.219882] Elgin_sn9c103: alloc_and_submit_int_urb - 14
[ 5828.249934] Elgin_sn9c103: vidioc_querycap - 15





*/
