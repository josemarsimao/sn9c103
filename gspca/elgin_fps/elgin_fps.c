/* elgin_fps: cópia de trabalho de elgin_sn9c103.c, ajustada ao kernel 7.0:
 *   - vb2_queue.min_buffers_needed  -> min_queued_buffers
 *   - removidos .wait_prepare/.wait_finish do vb2_ops (o vb2 usa q->lock)
 * Não compila mais em kernels antigos (5.4): para esses, use a pasta elgin/. */
#define MODULE_NAME "gspca_elgin_fps"

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

#include <linux/version.h>


// Josemar:
// ********************************************************************************
#define DEBUG_103 1	/* 0 = remove todas as mensagens KDBG na compilacao */
#if DEBUG_103

    int elgcnt = 0;	/* numero de mensagens KDBG ja impressas */

    /* nivel do log, mudavel com o modulo carregado (/sys/module/gspca_elgin_fps/parameters/dbg_level):
     *   0 = nada; 1 = so eventos (abrir, iniciar/parar captura, fps, sensor); 2 = tudo, inclusive cada
     *   acesso a registrador e cada controle (o ganho automatico gera muitas mensagens por segundo) */
    static int dbg_level = 1;
    module_param(dbg_level, int, 0644);
    MODULE_PARM_DESC(dbg_level, "log do driver: 0 = nada, 1 = eventos (padrao), 2 = tudo (cada registrador e controle)");

    /* imprime a mensagem (nivel info) se o dbg_level atual for >= n */
    #define KDBG_N(n, dev, func, fmt, args...) do { if (dbg_level >= (n)) { dev_info(dev, "Elgin_sn9c103: " func fmt "\n", ## args); elgcnt++; } } while(0);
    #define KDBG(dev, func, fmt, args...)  KDBG_N(1, dev, func, fmt, ## args)	/* eventos */
    #define KDBG2(dev, func, fmt, args...) KDBG_N(2, dev, func, fmt, ## args)	/* frequentes */

#else

    #define KDBG(dev, func, fmt, args...)
    #define KDBG2(dev, func, fmt, args...)

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

/* teste: 1 = nao pular o alt mais alto (1003 KB/s) mesmo com audio USB */
static int top_alt = 1;
module_param(top_alt, int, 0644);
MODULE_PARM_DESC(top_alt, "1 (padrao) = usa o alt mais alto (1003 KB/s) mesmo com audio USB; 0 = pula o alt mais alto como o gspca");

/* dispositivo atual, para o acesso generico a registradores (regw/regr) */
static struct gspca_dev *elg_dev;

/* nomes da ponte e do sensor (definidos mais adiante), usados no querycap e no enum_input */
static void elg_card_name(struct gspca_dev *gspca_dev, char *buf, size_t len);
static void elg_input_name(struct gspca_dev *gspca_dev, char *buf, size_t len);




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

	s32 gain, orig_gain, exposure, orig_exposure;
	int i, steps, retval = 0;

	if (v4l2_ctrl_g_ctrl(gspca_dev->autogain) == 0)
		return 0;

	orig_gain = gain = v4l2_ctrl_g_ctrl(gspca_dev->gain);
	orig_exposure = exposure = v4l2_ctrl_g_ctrl(gspca_dev->exposure);

	// If we are of a multiple of deadzone, do multiple steps to reach the
	// desired lumination fast (with the risc of a slight overshoot)


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

	// If we are of a multiple of deadzone, do multiple steps to reach the
	// desired lumination fast (with the risc of a slight overshoot)

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


/* versao do modulo: "elgin-AAAAMMDDHHMM"; o numero vem da compilacao (umilmod.sh -> make ELGIN_BUILD=...) */
#ifndef ELGIN_BUILD
#define ELGIN_BUILD "dev"
#endif
#define GSPCA_VERSION	"elgin-" ELGIN_BUILD



#if IS_ENABLED(CONFIG_INPUT)
#include <linux/input.h>
#include <linux/usb/input.h>
#endif

/* global values */
#define DEF_NURBS 3		/* default number of URBs */
#if DEF_NURBS > MAX_NURBS
#error "DEF_NURBS too big"
#endif


MODULE_AUTHOR("Original: Jean-François Moine <http://moinejf.free.fr>");
MODULE_AUTHOR("Modificador: Josemar Simão <josemars@ifes.edu.br>");
MODULE_AUTHOR("Revisor: Claude (Anthropic) <noreply@anthropic.com>");
MODULE_DESCRIPTION("GSPCA USB Elgin Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(GSPCA_VERSION);

int gspca_debug;


static void PDEBUG_MODE(struct gspca_dev *gspca_dev, int debug, char *txt, __u32 pixfmt, int w, int h)
{
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

static int gspca_input_connect(struct gspca_dev *dev)
{
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
			pr_err("Input device registration failed with error %i\n", err);
			input_dev->dev.parent = NULL;
			input_free_device(input_dev);
		} else {
			dev->input_dev = input_dev;
		}
	}

	return err;
}

static int alloc_and_submit_int_urb(struct gspca_dev *gspca_dev, struct usb_endpoint_descriptor *ep)
{
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

static void gspca_input_create_urb(struct gspca_dev *gspca_dev)
{
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

	gspca_dev->sd_desc->pkt_scan(gspca_dev,	urb->transfer_buffer, urb->actual_length);

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
	struct gspca_buffer *buf;
	unsigned long flags;

	gspca_dbg(gspca_dev, D_PACK, "add t:%d l:%d\n",	packet_type, len);

	spin_lock_irqsave(&gspca_dev->qlock, flags);
	buf = list_first_entry_or_null(&gspca_dev->buf_list, typeof(*buf), list);
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
			gspca_err(gspca_dev, "frame overflow %d > %d\n", gspca_dev->image_len + len, PAGE_ALIGN(gspca_dev->pixfmt.sizeimage));
			packet_type = DISCARD_PACKET;
		} else {
/* !! image is NULL only when last pkt is LAST or DISCARD
			if (gspca_dev->image == NULL) {
				pr_err("gspca_frame_add() image == NULL\n");
				return;
			}
 */
			memcpy(gspca_dev->image + gspca_dev->image_len,	data, len);
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
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, gspca_dev->image_len);
		buf->vb.sequence = gspca_dev->sequence++;
		buf->vb.field = V4L2_FIELD_NONE;
		gspca_dbg(gspca_dev, D_FRAM, "frame complete len:%d\n", gspca_dev->image_len);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		gspca_dev->image = NULL;
		gspca_dev->image_len = 0;
	}
}

static void destroy_urbs(struct gspca_dev *gspca_dev)
{
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
	u32 bandwidth;

	/* get the (max) image size */
	bandwidth = gspca_dev->pixfmt.sizeimage;

	/* if the image is compressed, estimate its mean size */
	if (!gspca_dev->cam.needs_full_bandwidth && bandwidth < gspca_dev->pixfmt.width * gspca_dev->pixfmt.height)
		bandwidth = bandwidth * 3 / 8;	/* 0.375 */

	/* estimate the frame rate */
	if (gspca_dev->sd_desc->get_streamparm) {
		struct v4l2_streamparm parm;

		gspca_dev->sd_desc->get_streamparm(gspca_dev, &parm);
		bandwidth *= parm.parm.capture.timeperframe.denominator;
		bandwidth /= parm.parm.capture.timeperframe.numerator;
		/* Margem de 30%: o fluxo cru (BA81) perde linhas quando ocupa ~96% do alt escolhido (BA81 320x240
		 * a 10 fps pedia 768 KB/s e caia no alt 6 de 800 KB/s: quadros cortados). Com a margem sobe um alt. */
		bandwidth += bandwidth * 3 / 10;
	} else {

		/* don't hope more than 15 fps with USB 1.1 and
		 * image resolution >= 640x480 */
		if (gspca_dev->pixfmt.width >= 640 && gspca_dev->dev->speed == USB_SPEED_FULL)
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
			ep = alt_xfer(&intf->altsetting[j], USB_ENDPOINT_XFER_ISOC, gspca_dev->xfer_ep);
			if (ep == NULL)
				continue;
			if (ep->desc.bInterval == 0) {
				pr_err("alt %d iso endp with 0 interval\n", j);
				continue;
			}
			psize = le16_to_cpu(ep->desc.wMaxPacketSize);
			psize = (psize & 0x07ff) * (1 + ((psize >> 11) & 3));
			bandwidth = psize * 1000;
			if (gspca_dev->dev->speed == USB_SPEED_HIGH || gspca_dev->dev->speed >= USB_SPEED_SUPER)
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
		gspca_dbg(gspca_dev, D_STREAM, "alt %d bandwidth %d\n", ep_tb->alt, ep_tb->bandwidth);
		last_bw = ep_tb->bandwidth;
		i++;
		ep_tb++;

		KDBG(gspca_dev->v4l2_dev.dev, "bandwidth found: %d"," - %d",last_bw, elgcnt)
	}

	/*
	 * If the camera:
	 * has a usb audio class interface (a built in usb mic); and
	 * is a usb 1 full speed device; and
	 * uses the max full speed iso bandwidth; and
	 * and has more than 1 alt setting
	 * then skip the highest alt setting to spare bandwidth for the mic
	 */
	if (!top_alt && gspca_dev->audio && gspca_dev->dev->speed == USB_SPEED_FULL && last_bw >= 1000000 && i > 1) {
		gspca_dbg(gspca_dev, D_STREAM, "dev has usb audio, skipping highest alt\n");

		KDBG(gspca_dev->v4l2_dev.dev, "dev has usb audio, skipping highest alt"," - %d",elgcnt)

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
			urb->pipe = usb_rcvisocpipe(gspca_dev->dev, ep->desc.bEndpointAddress);

			urb->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;

			urb->interval = 1 << (ep->desc.bInterval - 1);
			urb->complete = isoc_irq;
			urb->number_of_packets = npkt;
			for (i = 0; i < npkt; i++) {
				urb->iso_frame_desc[i].length = psize;
				urb->iso_frame_desc[i].offset = psize * i;
			}
		} else {		/* bulk */
			urb->pipe = usb_rcvbulkpipe(gspca_dev->dev, ep->desc.bEndpointAddress);
			urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
			urb->complete = bulk_irq;
		}
	}
	return 0;
}

/* Note: both the queue and the usb locks should be held when calling this */
static void gspca_stream_off(struct gspca_dev *gspca_dev)
{
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
	struct usb_interface *intf;
	struct usb_host_endpoint *ep;
	struct urb *urb;
	struct ep_tb_s ep_tb[MAX_ALT];
	int n, ret, xfer, alt, alt_idx;

	KDBG(gspca_dev->v4l2_dev.dev, "gspca_init_transfer"," - %d",elgcnt)

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
	xfer = gspca_dev->cam.bulk ? USB_ENDPOINT_XFER_BULK : USB_ENDPOINT_XFER_ISOC;

	/* if bulk or the subdriver forced an altsetting, get the endpoint */
	if (gspca_dev->alt != 0) {
		gspca_dev->alt--;	/* (previous version compatibility) */
		ep = alt_xfer(&intf->altsetting[gspca_dev->alt], xfer, gspca_dev->xfer_ep);
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
				ret = usb_set_interface(gspca_dev->dev,	gspca_dev->iface, alt);
				if (ret < 0) {
					if (ret == -ENOSPC)
						goto retry; /*fixme: ugly*/
					pr_err("set alt %d err %d\n", alt, ret);
					goto out;
				}
			}
		}
		if (!gspca_dev->cam.no_urb_create) {
			gspca_dbg(gspca_dev, D_STREAM, "init transfer alt %d\n", alt);
			ret = create_urbs(gspca_dev, alt_xfer(&intf->altsetting[alt], xfer, gspca_dev->xfer_ep));
			if (ret < 0) {
				destroy_urbs(gspca_dev);
				goto out;
			}
		}

		/* clear the bulk endpoint */
		if (gspca_dev->cam.bulk)
			usb_clear_halt(gspca_dev->dev, gspca_dev->urb[0]->pipe);

		/******************* start the cam *************************/
		ret = gspca_dev->sd_desc->start(gspca_dev);
		/// *********************************************************

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
			pr_err("usb_submit_urb alt %d err %d\n", gspca_dev->alt, ret);
			goto out;
		}

		/* the bandwidth is not wide enough
		 * negotiate or try a lower alternate setting */
retry:
		gspca_err(gspca_dev, "alt %d - bandwidth not wide enough, trying again\n", alt);
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

static void gspca_set_default_mode(struct gspca_dev *gspca_dev)
{
	int i;

	i = gspca_dev->cam.nmodes - 1;	/* take the highest mode */
	gspca_dev->curr_mode = i;
	gspca_dev->pixfmt = gspca_dev->cam.cam_mode[i]; // josemar: Escolher o modo padrão a aprtir dos modo incluídos na tabela.

	/* does nothing if ctrl_handler == NULL */
	v4l2_ctrl_handler_setup(gspca_dev->vdev.ctrl_handler);
}

static int wxh_to_mode(struct gspca_dev *gspca_dev,	int width, int height, u32 pixelformat)
{
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
	int modeU, modeD;

	modeU = modeD = mode;
	while ((modeU < gspca_dev->cam.nmodes) || modeD >= 0) {
		if (--modeD >= 0) {
			if (gspca_dev->cam.cam_mode[modeD].pixelformat == pixfmt)
				return modeD;
		}
		if (++modeU < gspca_dev->cam.nmodes) {
			if (gspca_dev->cam.cam_mode[modeU].pixelformat == pixfmt)
				return modeU;
		}
	}
	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vidioc_g_chip_info(struct file *file, void *priv, struct v4l2_dbg_chip_info *chip)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	if (gspca_dev->sd_desc->get_chip_info)
		return gspca_dev->sd_desc->get_chip_info(gspca_dev, chip);
	return chip->match.addr ? -EINVAL : 0;
}

static int vidioc_g_register(struct file *file, void *priv,	struct v4l2_dbg_register *reg)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->get_register(gspca_dev, reg);
}

static int vidioc_s_register(struct file *file, void *priv,	const struct v4l2_dbg_register *reg)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->set_register(gspca_dev, reg);
}
#endif

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv, struct v4l2_fmtdesc *fmtdesc)
{
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
	struct gspca_dev *gspca_dev = video_drvdata(file);
	u32 priv = fmt->fmt.pix.priv;

	fmt->fmt.pix = gspca_dev->pixfmt;
	/* some drivers use priv internally, so keep the original value */
	fmt->fmt.pix.priv = priv;
	return 0;
}

static int try_fmt_vid_cap(struct gspca_dev *gspca_dev, struct v4l2_format *fmt)
{
	int w, h, mode, mode2;

	w = fmt->fmt.pix.width;
	h = fmt->fmt.pix.height;

	PDEBUG_MODE(gspca_dev, D_CONF, "try fmt cap", fmt->fmt.pix.pixelformat, w, h);

	/* search the nearest mode for width and height */
	mode = wxh_to_nearest_mode(gspca_dev, w, h, fmt->fmt.pix.pixelformat);

	/* OK if right palette */
	if (gspca_dev->cam.cam_mode[mode].pixelformat != fmt->fmt.pix.pixelformat) {

		/* else, search the closest mode with the same pixel format */
		mode2 = gspca_get_mode(gspca_dev, mode, fmt->fmt.pix.pixelformat);
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
	struct gspca_dev *gspca_dev = video_drvdata(file);
	int i;
	__u32 index = 0;

	if (gspca_dev->sd_desc->enum_framesizes)
		return gspca_dev->sd_desc->enum_framesizes(gspca_dev, fsize);

	for (i = 0; i < gspca_dev->cam.nmodes; i++) {
		if (fsize->pixel_format != gspca_dev->cam.cam_mode[i].pixelformat)
			continue;

		if (fsize->index == index) {
			fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			fsize->discrete.width =	gspca_dev->cam.cam_mode[i].width;
			fsize->discrete.height = gspca_dev->cam.cam_mode[i].height;
			return 0;
		}
		index++;
	}

	return -EINVAL;
}

static int vidioc_enum_frameintervals(struct file *filp, void *priv, struct v4l2_frmivalenum *fival)
{
	struct gspca_dev *gspca_dev = video_drvdata(filp);
	int mode;
	__u32 i;

	mode = wxh_to_mode(gspca_dev, fival->width, fival->height, fival->pixel_format);
	if (mode < 0)
		return -EINVAL;

	if (gspca_dev->cam.mode_framerates == NULL || gspca_dev->cam.mode_framerates[mode].nrates == 0)
		return -EINVAL;

	if (fival->pixel_format != gspca_dev->cam.cam_mode[mode].pixelformat)
		return -EINVAL;

	for (i = 0; i < gspca_dev->cam.mode_framerates[mode].nrates; i++) {
		if (fival->index == i) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = gspca_dev->cam.mode_framerates[mode].rates[i];
			return 0;
		}
	}

	return -EINVAL;
}

static void gspca_release(struct v4l2_device *v4l2_device)
{
	struct gspca_dev *gspca_dev = container_of(v4l2_device, struct gspca_dev, v4l2_dev);

	v4l2_ctrl_handler_free(gspca_dev->vdev.ctrl_handler);
	v4l2_device_unregister(&gspca_dev->v4l2_dev);
	kfree(gspca_dev->usb_buf);
	kfree(gspca_dev);
}

static int vidioc_querycap(struct file *file, void  *priv, struct v4l2_capability *cap)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	strscpy((char *)cap->driver, gspca_dev->sd_desc->name, sizeof(cap->driver));

	elg_card_name(gspca_dev, (char *) cap->card, sizeof(cap->card));

	usb_make_path(gspca_dev->dev, (char *) cap->bus_info, sizeof(cap->bus_info));

	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,	struct v4l2_input *input)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	if (input->index != 0)
		return -EINVAL;
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->status = gspca_dev->cam.input_flags;
	elg_input_name(gspca_dev, input->name, sizeof input->name);
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;
	return 0;
}

static int vidioc_g_jpegcomp(struct file *file, void *priv, struct v4l2_jpegcompression *jpegcomp)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->get_jcomp(gspca_dev, jpegcomp);
}

static int vidioc_s_jpegcomp(struct file *file, void *priv, const struct v4l2_jpegcompression *jpegcomp)
{
	struct gspca_dev *gspca_dev = video_drvdata(file);

	gspca_dev->usb_err = 0;
	return gspca_dev->sd_desc->set_jcomp(gspca_dev, jpegcomp);
}

static int vidioc_g_parm(struct file *filp, void *priv,	struct v4l2_streamparm *parm)
{
	struct gspca_dev *gspca_dev = video_drvdata(filp);

	parm->parm.capture.readbuffers = gspca_dev->queue.min_queued_buffers;

	if (!gspca_dev->sd_desc->get_streamparm)
		return 0;

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	gspca_dev->usb_err = 0;
	gspca_dev->sd_desc->get_streamparm(gspca_dev, parm);
	return gspca_dev->usb_err;
}

static int vidioc_s_parm(struct file *filp, void *priv, struct v4l2_streamparm *parm)
{
	struct gspca_dev *gspca_dev = video_drvdata(filp);

	parm->parm.capture.readbuffers = gspca_dev->queue.min_queued_buffers;

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
	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = PAGE_ALIGN(gspca_dev->pixfmt.sizeimage);

	if (vb2_plane_size(vb, 0) < size) {
		gspca_err(gspca_dev, "buffer too small (%lu < %lu)\n", vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	return 0;
}

static void gspca_buffer_finish(struct vb2_buffer *vb)
{
	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vb->vb2_queue);

	if (!gspca_dev->sd_desc->dq_callback)
		return;

	gspca_dev->usb_err = 0;
	if (gspca_dev->present)
		gspca_dev->sd_desc->dq_callback(gspca_dev);
}

static void gspca_buffer_queue(struct vb2_buffer *vb)
{
	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct gspca_buffer *buf = to_gspca_buffer(vb);
	unsigned long flags;

	spin_lock_irqsave(&gspca_dev->qlock, flags);
	list_add_tail(&buf->list, &gspca_dev->buf_list);
	spin_unlock_irqrestore(&gspca_dev->qlock, flags);
}

static void gspca_return_all_buffers(struct gspca_dev *gspca_dev, enum vb2_buffer_state state)
{
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
	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vq);
	int ret;

	KDBG(gspca_dev->v4l2_dev.dev, "gspca_start_streaming"," - %d",elgcnt)

	gspca_dev->sequence = 0;

	ret = gspca_init_transfer(gspca_dev);
	if (ret)
		gspca_return_all_buffers(gspca_dev, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void gspca_stop_streaming(struct vb2_queue *vq)
{
	struct gspca_dev *gspca_dev = vb2_get_drv_priv(vq);

	KDBG(gspca_dev->v4l2_dev.dev, "gspca_stop_streaming"," - %d",elgcnt)

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


static void reg_r5(struct gspca_dev *gspca_dev,  __u16 value);
static void reg_r(struct gspca_dev *gspca_dev,  __u16 value);
static void reg_w(struct gspca_dev *gspca_dev, __u16 value, const __u8 *buffer, int len);
static void i2c_r(struct gspca_dev *gspca_dev, u8 reg, int len);


/*
 * probe and create a new gspca device
 *
 * This function must be called by the sub-driver when it is
 * called for probing a new device.
 */
 int gspca_dev_probe2(struct usb_interface *intf, const struct usb_device_id *id, const struct sd_desc *sd_desc,	int dev_size, struct module *module)
{
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
	q->min_queued_buffers = 2;
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
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,6,19)
    ret = video_register_device(&gspca_dev->vdev, VFL_TYPE_GRABBER, -1);
#else
    ret = video_register_device(&gspca_dev->vdev, VFL_TYPE_VIDEO, -1);
#endif
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
int gspca_dev_probe(struct usb_interface *intf,	const struct usb_device_id *id,	const struct sd_desc *sd_desc, int dev_size, struct module *module) {

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

	struct gspca_dev *gspca_dev = usb_get_intfdata(intf);
#if IS_ENABLED(CONFIG_INPUT)
	struct input_dev *input_dev;
#endif

	gspca_dbg(gspca_dev, D_PROBE, "%s disconnect\n", video_device_node_name(&gspca_dev->vdev));

	if (elg_dev == gspca_dev)
		elg_dev = NULL;

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
	struct v4l2_ctrl *contrast;	/* contraste: na curva da ponte (regs 0x20-0x30) junto com a gama; 128 = sem mudanca */
	struct v4l2_ctrl *awb;		/* balanco de branco automatico pelo sensor (COMA bit 2) */
	struct v4l2_ctrl *rgain;	/* ganhos R, G e B da ponte (regs 0x05/0x06/0x07, por posicao no mosaico) */
	struct v4l2_ctrl *ggain;
	struct v4l2_ctrl *bgain;

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

	u8 fps;			/* taxa pedida via S_PARM (0 = padrão do modo) */
	int fps_mode;		/* modo a que 'fps' se refere */
	struct v4l2_ctrl *black;	/* nivel de preto da ponte (reg 0x14, OFFSET) */
	struct v4l2_ctrl *gamma;	/* curva de gama da ponte (regs 0x20 a 0x30): 0 a 8 */
	struct v4l2_ctrl *hflip;	/* espelho horizontal (COMA bit 6 do sensor) */
	struct v4l2_ctrl *vflip;	/* inversao vertical (COMN 0x75 bit 7 do sensor) */
	struct v4l2_ctrl *bandf;	/* filtro de banda (COMJ 0x2D bit 2 do sensor) */
	struct v4l2_ctrl *ae_target;	/* alvo de luminosidade do ganho automatico (127 = padrao) */
	struct v4l2_ctrl *abg_blue;	/* alvo do AWB, fundo azul (sensor 0x0C) */
	struct v4l2_ctrl *abg_red;	/* alvo do AWB, fundo vermelho (sensor 0x0D) */
	__u8 vstart;			/* V_START base (reg 0x13 da ponte) */
	struct v4l2_ctrl *testpat;	/* padrao de teste: 0 desligado, 1 ponte (0x17 bit 4), 2 barras do sensor */
	__u8 hstart;			/* H_START base (sem a correcao do espelho) */
	__u8 reg17;			/* valor de 0x17 escrito no inicio (sem o bit de teste) */
	struct v4l2_ctrl *dynfps;	/* "dynamic framerate": liga a trava de banda do S910 */
	u8 fps_cap;		/* taxa maxima imposta pela trava de banda (0 = sem limite) */
	u32 link_bw;		/* B/s do alt em uso (wMaxPacketSize x 1000) */
	atomic_t frame_len;	/* tamanho do ultimo quadro (bytes), preenchido no sd_pkt_scan */
	int size_peak;		/* maximo (com decaimento lento) do tamanho dos quadros */
	u8 guard_hi, guard_lo;	/* contadores de histerese da trava */
	u8 hi;			/* 1 = modo de 60 fps (S910 160x120), fixado no sd_start */
	int cap_mode;		/* modo em que o limite fps_cap foi criado */
	u16 ae_area;		/* area da janela de AE em blocos de 32x32 (117 = janela padrao) */
	u8 pivo;		/* ponto de apoio do contraste aplicado na curva (tom medio da cena, 0 = ainda nao medido) */
	u8 pivo_espera;		/* quadros ate poder regravar a curva */
	int pivo_f;		/* tom medio filtrado, x16 */
	u32 fps_logged;		/* ultima taxa (fps, reg 0x11, ajuste fino) mostrada no log; 0 = nenhuma */
	struct v4l2_ctrl *ae_win;	/* menu: janela de AE (padrao do modo, tela cheia, centro, metades, personalizada) */
	struct v4l2_ctrl *ae_sx, *ae_sy, *ae_ex, *ae_ey;	/* janela personalizada (regs 0x1C a 0x1F) */
	u8 ae_def[4];		/* janela de AE padrao do modo (regs 0x1C a 0x1F) */
	u8 ae_w, ae_h;		/* tamanho da janela do sensor em blocos de 32 (reg 0x15 / 2, reg 0x16 / 2) */
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
#define MODE_QVGA 0x40		/* 80x60: janela 320x240 da ponte (sensor em QVGA) reduzida a 1/4 */
#define MODE_CIF 0x80		/* CIF 352x288 e QCIF 176x144: janela 352x288 centrada no quadro VGA do sensor */

#define COMP 0xc7		/* 0x87 //0x07 */
#define COMP1 0xc9		/* 0x89 //0x09 */

#define MCK_INIT 0x63


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


/* Os 4 formatos Bayer brutos: a fase do mosaico e escolhida pela paridade de H_START/V_START da ponte
 * (deslocamento de uma coluna e/ou de uma linha sobre o BGGR); ver sd_start. Os 3 blocos de fase vem ANTES
 * do BA81 para que a enumeracao de formatos continue S910, BA81, e so depois GBRG, GRBG, RGGB. */
#define RAWM(w, h, f, pv) \
	{ w, h, f, V4L2_FIELD_NONE, .bytesperline = w, .sizeimage = (w) * (h), \
	  .colorspace = V4L2_COLORSPACE_SRGB, .priv = pv }
#define PHASE_MODES(f) \
	RAWM(80, 60, f, 2 | MODE_RAW | MODE_QVGA), RAWM(160, 120, f, 2 | MODE_RAW), \
	RAWM(176, 144, f, 1 | MODE_RAW | MODE_CIF), RAWM(320, 240, f, 1 | MODE_RAW), \
	RAWM(352, 288, f, 0 | MODE_RAW | MODE_CIF), RAWM(640, 480, f, 0 | MODE_RAW)

static const struct v4l2_pix_format vga_mode[] = {
	PHASE_MODES(V4L2_PIX_FMT_SGBRG8),	/* H_START + 1 */
	PHASE_MODES(V4L2_PIX_FMT_SGRBG8),	/* V_START + 1 */
	PHASE_MODES(V4L2_PIX_FMT_SRGGB8),	/* H_START + 1 e V_START + 1 */
//josemar: Novos modos podem ser adicionados aqui. O ".priv" é utilizado para definir a resolução no registro 0x18h. Preciso entender melhor como é formado o tamanho da imagem
// josemar: Lembrando que CIF(Common Intermediate Format) é referente a um padrão de tamanho de imagem (PAL/NTSC) correspondente a 352x288 pixels e seus multiplos:  QCIF, 2CIF, 4CIF, 16CIF
//    {80,   60, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,        // josemar: Adicionado por mim
//		.bytesperline = 80,
//		.sizeimage = 80 * 60,
//		.colorspace = V4L2_COLORSPACE_SRGB,
//		.priv = 0 | MODE_RAW},
	{80, 60, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,
		.bytesperline = 80,
		.sizeimage = 80 * 60,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 2 | MODE_RAW | MODE_QVGA},
	{160, 120, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,
		.bytesperline = 160,
		.sizeimage = 160 * 120,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 2 | MODE_RAW},
	{176, 144, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,	/* QCIF: janela CIF com escala 1/2 */
		.bytesperline = 176,
		.sizeimage = 176 * 144,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1 | MODE_RAW | MODE_CIF},
    {320, 240, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,        // josemar: Adicionado por mim
		.bytesperline = 320,
		.sizeimage = 320 * 240,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1 | MODE_RAW},
	{352, 288, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,	/* CIF: janela 352x288 sem escala */
		.bytesperline = 352,
		.sizeimage = 352 * 288,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0 | MODE_RAW | MODE_CIF},
    {640, 480, V4L2_PIX_FMT_SBGGR8, V4L2_FIELD_NONE,        // josemar: Adicionado por mim
		.bytesperline = 640,
		.sizeimage = 640 * 480,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0 | MODE_RAW},
//    {80,   60, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,        // josemar: Adicionado por mim
//		.bytesperline = 80,
//		.sizeimage = 80 * 60 * 5 / 4,
//		.colorspace = V4L2_COLORSPACE_SRGB,
//		.priv = 1 | MODE_RAW},
	{80, 60, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 80,
		.sizeimage = 80 * 60 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 2 | MODE_QVGA},
	{160, 120, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 160,
		.sizeimage = 160 * 120 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 2},
	{176, 144, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,	/* QCIF: janela CIF com escala 1/2 */
		.bytesperline = 176,
		.sizeimage = 176 * 144 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1 | MODE_CIF},
	{320, 240, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 320,
		.sizeimage = 320 * 240 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 1},
	{352, 288, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,	/* CIF: janela 352x288 sem escala */
		.bytesperline = 352,
		.sizeimage = 352 * 288 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0 | MODE_CIF},
	{640, 480, V4L2_PIX_FMT_SN9C10X, V4L2_FIELD_NONE,
		.bytesperline = 640,
		.sizeimage = 640 * 480 * 5 / 4,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.priv = 0},
};


/* Taxas de quadros por modo (VIDIOC_ENUM_FRAMEINTERVALS), na mesma ordem de
 * vga_mode[]: 160/320/640 em BA81 (RAW) e depois em S910 (comprimido).
 * No sensor, fps real = 30/(n+1): 30, 15, 10, 7,5 (anunciado como 8), ...
 * Os limites vêm dos testes: o link USB 1.1 comporta ~900 KB/s; acima disso
 * o bridge corta o quadro e a imagem fica deformada.
 * O 60 fps do 160x120 ainda não está implementado, por isso não é anunciado. */
static const u8 fps_80[]       = { 60, 30, 15, 10, 8 };	/* 80x60 RAW e S910: sensor em QVGA (como o 160x120 de 60 fps) */
static const u8 fps_160[]      = { 30, 15, 10, 8 };	/* 160x120 RAW e S910 */
static const u8 fps_qcif_raw[] = { 12, 8, 6 };	/* 176x144 RAW: 25 KB por quadro; 30 fps passaria de 90% do alt */
static const u8 fps_cif_raw[]  = { 4, 3, 2 };		/* 352x288 RAW: 101 KB por quadro; 5 fps = 507 KB/s */
static const u8 fps_s910_cif[] = { 30, 15, 10, 8 };	/* 176x144 e 352x288 S910 (a trava de banda reduz se faltar canal) */
static const u8 fps_s910_160[] = { 60, 30, 15, 10, 8 };	/* 60: sensor em QVGA entrelacado */
static const u8 fps_s910_320[] = { 30, 15, 10, 8 };	/* 30 só em cena clara */
static const u8 fps_s910_640[] = { 14, 13, 10, 8 };	/* 14 e 13: ajuste fino (reg 0x2A/0x2B) + alt 8; 8 = 7,5 */
static const u8 fps_raw_320[]  = { 10, 8 };
static const u8 fps_raw_640[]  = { 2 };

#define FRATES(a)	{ .rates = (a), .nrates = ARRAY_SIZE(a) }

#define PHASE_FRATES \
	FRATES(fps_80), FRATES(fps_160), FRATES(fps_qcif_raw), FRATES(fps_raw_320), FRATES(fps_cif_raw), FRATES(fps_raw_640)

static const struct framerates vga_framerates[] = {
	PHASE_FRATES, PHASE_FRATES, PHASE_FRATES,	/* GBRG, GRBG, RGGB (mesmos limites do BA81) */
	FRATES(fps_80),			/* 80x60 RAW */
	FRATES(fps_160),		/* 160x120 RAW */
	FRATES(fps_qcif_raw),		/* 176x144 RAW */
	FRATES(fps_raw_320),	/* 320x240 RAW */
	FRATES(fps_cif_raw),		/* 352x288 RAW */
	FRATES(fps_raw_640),	/* 640x480 RAW */
	FRATES(fps_80),			/* 80x60 S910 */
	FRATES(fps_s910_160),		/* 160x120 S910 */
	FRATES(fps_s910_cif),		/* 176x144 S910 */
	FRATES(fps_s910_320),	/* 320x240 S910 */
	FRATES(fps_s910_cif),		/* 352x288 S910 */
	FRATES(fps_s910_640),	/* 640x480 S910 */
};

/* Taxa usada quando o aplicativo não pede nenhuma (a que sempre funcionou) */
static const u8 vga_fps_default[] = { 30, 30, 12, 10, 4, 2,  30, 30, 12, 10, 4, 2,  30, 30, 12, 10, 4, 2,
				      30, 30, 12, 10, 4, 2,  30, 30, 30, 15, 15, 10 };


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

#define MCK_INIT1 0x20		/*fixme: Bayer - 0x50 for JPEG ??*/


#if(0)
static const __u8 initOv7630[] = {
	0x04,   // r01
	0x44,   // r02
	0x00,   // r03
	0x00,   // r04
	0x00,   // r05
	0x00,   // r06
	0x00,   // r07
	0x80,   // r08
	0x21,   // r09
	0x00,   // r0A
	0x00,   // r0B
	0x00,   // r0C
	0x00,   // r0D
	0x00,   // r0E
	0x00,   // r0F
	0x00,   // r10
	0x00,   // r11
	0x01,   // r12
	0x01,   // r13
	0x0a,   // r14
	0x28,   // r15
	0x1f,   // r16 			//  H & V sizes     r15 .. r16
	0x64,   // r17  [0x17]=0x68
	0x8f,   // r18
	0x20, //MCK_INIT1,   // r19
};
#else
static const __u8 initOv7630[] = {
	0x00,   // r01
	0x00,   // r02
	0x00,   // r03
	0x00,   // r04
	0x00,   // r05
	0x00,   // r06
	0x00,   // r07
	0x00,   // r08
	0x00,   // r09
	0x00,   // r0A
	0x00,   // r0B
	0x00,   // r0C
	0x00,   // r0D
	0x00,   // r0E
	0x00,   // r0F
	0x00,   // r10
	0x00,   // r11
	0x00,   // r12
	0x00,   // r13
	0x00,   // r14
	0x00,   // r15
	0x00,   // r16 			//  H & V sizes     r15 .. r16
	0x00,   // r17  [0x17]=0x68
	0x00,   // r18
	0x00, //MCK_INIT1,   // r19
};

#endif


// tabela "original" do sensor: usada com stock_sensor=1


static const __u8 ov7630_sensor_init_stock[][8] = {
//  {i2cC, slID, addr,   D0,   D1,   D2,   D3, CRT }
	{0xa0, 0x21, 0x12, 0x80, 0x00, 0x00, 0x00, 0x10},// josemar: este faz um soft reset
	{0xa0, 0x21, 0x00, 0x80, 0x00, 0x00, 0x00, 0x10},// josemar: [0x00] = 0x80
	{0xb0, 0x21, 0x01, 0x80, 0x80, 0x00, 0x00, 0x10},// [0x01]=0x77; [0x02]=0x3a
    {0xa0, 0x21, 0x03, 0x80, 0x00, 0x00, 0x00, 0x10},// josemar: [0x03] = 0x80
    {0xa0, 0x21, 0x04, 0x00, 0x00, 0x00, 0x00, 0x10},// josemar: [0x06] = 0x80
    {0xa0, 0x21, 0x05, 0x20, 0x00, 0x00, 0x00, 0x10},// josemar: [0x06] = 0x80
	{0xa0, 0x21, 0x06, 0x80, 0x00, 0x00, 0x00, 0x10},// josemar: [0x06] = 0x80
	{0xa0, 0x21, 0x0c, 0x20, 0x00, 0x00, 0x00, 0x10},// josemar: [0x06] = 0x80
	{0xa0, 0x21, 0x0d, 0x20, 0x00, 0x00, 0x00, 0x10},// josemar: [0x06] = 0x80
	{0xa0, 0x21, 0x10, 0x50, 0x00, 0x00, 0x00, 0x10},// josemar: [0x10] = 0x41
	{0xa0, 0x21, 0x11, 0x01, 0x00, 0x00, 0x00, 0x10},// josemar: [0x11] = 0x00
	{0xa0, 0x21, 0x12, 0x3c, 0x00, 0x00, 0x00, 0x10},// [0x12]=0x5c; [0x13]=0x00; [0x14]=0x80; [0x15]=0x34
	{0xa0, 0x21, 0x13, 0x83, 0x00, 0x00, 0x00, 0x10},// [0x12]=0x5c; [0x13]=0x00; [0x14]=0x80; [0x15]=0x34
	{0xa0, 0x21, 0x14, 0x80, 0x00, 0x00, 0x00, 0x10},// [0x12]=0x5c; [0x13]=0x00; [0x14]=0x80; [0x15]=0x34
	{0xa0, 0x21, 0x15, 0x2d, 0x00, 0x00, 0x00, 0x10},// [0x12]=0x5c; [0x13]=0x00; [0x14]=0x80; [0x15]=0x34
	{0xa0, 0x21, 0x16, 0x03, 0x00, 0x00, 0x00, 0x10},// josemar: [0x16] = 0x03
	{0xd0, 0x21, 0x17, 0x1a, 0xbb, 0x06, 0xf6, 0x10},// [0x17]=0x1c; [0x18]=0xbd; [0x19]=0x06; [0x1A]=0xf6
	{0xa0, 0x21, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x10},
	{0xa0, 0x21, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x10},
	{0xa0, 0x21, 0x20, 0x44, 0x00, 0x00, 0x00, 0x10},
	{0xa0, 0x21, 0x23, 0xdc, 0x00, 0x80, 0x34, 0x10},
	{0xa0, 0x21, 0x26, 0xa0, 0x00, 0x00, 0x00, 0x10},   // josemar: desabilitado produz FPS=60
    {0xa0, 0x21, 0x27, 0x9a, 0x00, 0x00, 0x00, 0x10},
    {0xa0, 0x21, 0x28, 0xa0, 0x00, 0x00, 0x00, 0x10},
    {0xa0, 0x21, 0x29, 0x74, 0x00, 0x00, 0x00, 0x10},   // aqui tem 2x
	{0xb0, 0x21, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x10},
	{0xa0, 0x21, 0x2d, 0x81, 0x00, 0x00, 0x00, 0x10},   // QVGA -> 60fps
	{0xb0, 0x21, 0x2f, 0x3d, 0x24, 0xa0, 0x30, 0x10},
	{0xa0, 0x21, 0x32, 0x06, 0x24, 0xa0, 0x30, 0x10},
	{0xb0, 0x21, 0x60, 0xa9, 0x4a, 0xa0, 0x30, 0x10},
	{0xa0, 0x21, 0x65, 0x00, 0x00, 0x00, 0x00, 0x10},
	{0xa0, 0x21, 0x69, 0x38, 0x00, 0x00, 0x00, 0x10},
	{0xc0, 0x21, 0x6f, 0x88, 0x0b, 0x00, 0x00, 0x10},
	{0xc0, 0x21, 0x74, 0x21, 0x0e, 0x00, 0x00, 0x10},
	{0xa0, 0x21, 0x7d, 0x77, 0x00, 0x00, 0x00, 0x10},

};
/*
// josemar: configura no sensor
setfreq: [0x2b] = 0x8a
setexpo: [0x10] =           [0x11] =
setgain: [0x00] = gain
    // josemar: configura no controlador Sn9c103
    u8 buf[3] = { gain, gain, gain }; // R, G, B
	reg_w(gspca_dev, 0x05, buf, 3);
setbrig: [0x06] = bright_value


*/


// tabela capturada do Windows: padrão (stock_sensor=0)

static const __u8 ov7630_sensor_init[][8] = {
//  {i2cC, slID, addr,   D0,   D1,   D2,   D3, CRT }
	{0xA0, 0x21, 0x12, 0x80, 0x00, 0x00, 0x00, 0x10},
    {0xD0, 0x21, 0x12, 0x78, 0x00, 0x80, 0x34, 0x10},
    {0xA0, 0x21, 0x11, 0x01, 0x00, 0x80, 0x34, 0x10},
    {0xB0, 0x21, 0x01, 0x77, 0x3A, 0x80, 0x34, 0x10},
    {0xA0, 0x21, 0x1B, 0x04, 0x3A, 0x80, 0x34, 0x10},
    {0xA0, 0x21, 0x20, 0x44, 0x3A, 0x80, 0x34, 0x10},
    {0xA0, 0x21, 0x23, 0xEE, 0x3A, 0x80, 0x34, 0x10},
    {0xD0, 0x21, 0x26, 0xA0, 0x9A, 0xA0, 0x30, 0x10},
    {0xB0, 0x21, 0x2A, 0x80, 0x00, 0xA0, 0x30, 0x10},
    {0xB0, 0x21, 0x2F, 0x3D, 0x24, 0xA0, 0x30, 0x10},
    {0xA0, 0x21, 0x32, 0x86, 0x24, 0xA0, 0x30, 0x10},
    {0xB0, 0x21, 0x60, 0x29, 0x4A, 0xA0, 0x30, 0x10},	// 0x61 = 0x4A: SPCB bit 3 (BRTEN) ligado, como no sonixb; o
							// Windows grava 0x42, que desliga o brilho (0x06) na imagem crua
    {0xA0, 0x21, 0x65, 0x00, 0x42, 0xA0, 0x30, 0x10},
    {0xA0, 0x21, 0x69, 0x38, 0x42, 0xA0, 0x30, 0x10},
    {0xC0, 0x21, 0x6F, 0x88, 0x0B, 0x00, 0x30, 0x10},
    {0xC0, 0x21, 0x74, 0x21, 0x8E, 0x00, 0x30, 0x10},
    {0xA0, 0x21, 0x7D, 0xF7, 0x8E, 0x00, 0x30, 0x10},
    {0xD0, 0x21, 0x17, 0x1C, 0xBD, 0x06, 0xF6, 0x10},
    {0xA0, 0x21, 0x10, 0x83, 0xBD, 0x06, 0xF6, 0x16},
    {0xA0, 0x21, 0x76, 0x00, 0xBD, 0x06, 0xF6, 0x16},
    {0xA0, 0x21, 0x11, 0x00, 0xBD, 0x06, 0xF6, 0x16},
    {0xA0, 0x21, 0x00, 0x10, 0xBD, 0x06, 0xF6, 0x15},
    {0xB0, 0x21, 0x2A, 0x80, 0x60, 0x06, 0xF6, 0x1D},

};














// josemar: A ponte "bridge" será inicializada de diferentes formas dependendo do sensor utilizazdo. Além disso, ainda há a inicialização do sensor.
static const struct sensor_data sensor_data[] = {
//  SENS(brigde, sensor, flags, sensor_addr)
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
	SENS(initOv7630, ov7630_sensor_init, 0, 0x21),
};

/// *************************************************************************************************************
/// *************************************************************************************************************
/// *************************************************************************************************************



static void reg_r5(struct gspca_dev *gspca_dev,  __u16 value) {



	int res;

	KDBG2(gspca_dev->v4l2_dev.dev, "reg_r5"," - %d",elgcnt)


	if (gspca_dev->usb_err < 0)
		return;

	res = usb_control_msg(gspca_dev->dev,
			usb_rcvctrlpipe(gspca_dev->dev, 0),
			0,			/* request */
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			value,
			0,			/* index */
			gspca_dev->usb_buf, 5,
			500);

	if (res < 0) {
		dev_err(gspca_dev->v4l2_dev.dev, "Error reading register %02x: %d\n", value, res);
		gspca_dev->usb_err = res;
		/*
		 * Make sure the result is zeroed to avoid uninitialized
		 * values.
		 */
		gspca_dev->usb_buf[0] = 0;
	}
}

/* get one byte in gspca_dev->usb_buf */
static void reg_r(struct gspca_dev *gspca_dev,  __u16 value)
{
	int res;

    KDBG2(gspca_dev->v4l2_dev.dev, "reg_r"," - %d",elgcnt)

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
		dev_err(gspca_dev->v4l2_dev.dev, "Error reading register %02x: %d\n", value, res);
		gspca_dev->usb_err = res;
		/*
		 * Make sure the result is zeroed to avoid uninitialized
		 * values.
		 */
		gspca_dev->usb_buf[0] = 0;
	}
}

static void reg_w(struct gspca_dev *gspca_dev, __u16 value, const __u8 *buffer, int len)
{
	int res;

    KDBG2(gspca_dev->v4l2_dev.dev, "reg_w"," - %d",elgcnt)

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

static void i2c_w(struct gspca_dev *gspca_dev, const u8 *buf)
{
	int retry = 60;

    KDBG2(gspca_dev->v4l2_dev.dev, "i2c_w"," - %d",elgcnt)

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


/* Como i2c_w, mas ignora o bit de erro (0x08): o ciclo que so grava o endereco do registrador
 * (primeira etapa da leitura) deixa esse bit ligado no SN9C103 sem ser erro de verdade
 * (o driver Risolia tambem so confere o erro depois do ciclo de leitura). */
static void i2c_w_ign(struct gspca_dev *gspca_dev, const u8 *buf)
{
	int retry = 60;

	if (gspca_dev->usb_err < 0)
		return;
	reg_w(gspca_dev, 0x08, buf, 8);
	while (retry--) {
		if (gspca_dev->usb_err < 0)
			return;
		msleep(1);
		reg_r(gspca_dev, 0x08);
		if (gspca_dev->usb_buf[0] & 0x04)
			return;
	}
	dev_err(gspca_dev->v4l2_dev.dev, "i2c read-address timeout\n");
	gspca_dev->usb_err = -EIO;
}

static u8 i2c_raw[5];	/* ultimos 5 bytes crus lidos da porta I2C (diagnostico) */

static void i2c_r(struct gspca_dev *gspca_dev, u8 reg, int len)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u8 mode[8];

	KDBG2(gspca_dev->v4l2_dev.dev, "i2c_r"," - %d",elgcnt)

    // josemar: primeiro faz uma escrita para somente um registro
    //          que é o endereço
    //          depois faz uma leitura de até cinco registros
	mode[0] = 0x80 | 0x10; // i2c command = a0 (100 kHz)
	mode[1] = sensor_data[sd->sensor].sensor_addr;
	mode[2] = reg;
	mode[3] = 0;
	mode[4] = 0;
	mode[5] = 0;
	mode[6] = 0;
	mode[7] = 0x10;
	i2c_w_ign(gspca_dev, mode);
	msleep(10);
	mode[0] = (0x80 | (len << 4)) | 0x02;
	i2c_w_ign(gspca_dev, mode);	/* o erro so vale depois de ler o dado (Risolia) */
	msleep(10);
	reg_r5(gspca_dev, 0x0a);
	/* com 1 byte lido, o dado vem em usb_buf[4] (Risolia); copia para [0], que os chamadores usam */
	memcpy(i2c_raw, gspca_dev->usb_buf, 5);
	gspca_dev->usb_buf[0] = gspca_dev->usb_buf[4];
}

static void i2c_w_vector(struct gspca_dev *gspca_dev, const __u8 buffer[][8], int len)
{

    KDBG2(gspca_dev->v4l2_dev.dev, "i2c_w_vector"," - %d",elgcnt)

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

/// *************************************************************************************************************
/// *************************************************************************************************************
/// *************************************************************************************************************

static int fps_reg11_n(struct gspca_dev *gspca_dev);

static void setbrightness(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;

    KDBG2(gspca_dev->v4l2_dev.dev, "setbrightness"," - %d",elgcnt)

	switch (sd->sensor) {

	case  SENSOR_OV7630: {
		__u8 i2cOV[] =
			{0xa0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x10};

		/* change reg 0x06 */
		i2cOV[1] = sensor_data[sd->sensor].sensor_addr;
		i2cOV[3] = sd->brightness->val;
		i2c_w(gspca_dev, i2cOV);
		break;
	}



	default:
		break;
	}
}

static void setgain(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u8 gain = gspca_dev->gain->val;

    KDBG2(gspca_dev->v4l2_dev.dev, "setgain"," - %d",elgcnt)

	switch (sd->sensor) {

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

		// josemar
		// Read sensor ID
/*
		i2c_r(gspca_dev, 0x0a, 5);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor PID "," - %d",gspca_dev->usb_buf[0]);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor VER "," - %d",gspca_dev->usb_buf[1]);

        i2c_r(gspca_dev, 0x0b, 5);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor PID "," - %d",gspca_dev->usb_buf[0]);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor VER "," - %d",gspca_dev->usb_buf[1]);

        i2c_r(gspca_dev, 0x1c, 5);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor PID "," - %d",gspca_dev->usb_buf[0]);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor VER "," - %d",gspca_dev->usb_buf[1]);

        i2c_r(gspca_dev, 0x1d, 5);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor PID "," - %d",gspca_dev->usb_buf[0]);
        KDBG(gspca_dev->v4l2_dev.dev, "Sendor VER "," - %d",gspca_dev->usb_buf[1]);
*/
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
	struct sd *sd = (struct sd *) gspca_dev;

    KDBG2(gspca_dev->v4l2_dev.dev, "setexposure"," - %d",elgcnt)

	switch (sd->sensor) {

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

		reg11 = fps_reg11_n(gspca_dev) + 1;	/* taxa da tabela de fps (ou do parametro sensor_reg11) */
		if (reg11 < 1)
			reg11 = 1;
		else if (reg11 > 16)
			reg11 = 16;

		/* In 640x480, if the reg11 has less than 4, the image is
		   unstable (the bridge goes into a higher compression mode
		   which we have not reverse engineered yet). */
		if (0 && gspca_dev->pixfmt.width == 640 && reg11 < 4)	/* desligado: a taxa vem da tabela de fps */
            // ***************************************************************
			reg11 = 4;  // josemar: valor original 4, porém framerate ficava 8. Porém, se setado para 1,FR = 30
            // ***************************************************************

		/* frame exposure time in ms = 1000 * reg11 / 30    ->
		reg10 = (gspca_dev->exposure->val / 2) * reg10_max
				/ (1000 * reg11 / 30) */
		reg10 = (gspca_dev->exposure->val * 15 * reg10_max)	/ (1000 * reg11);

		/* Don't allow this to get below 10 when using autogain, the
		   steps become very large (relatively) when below 10 causing
		   the image to oscillate from much too dark, to much too bright
		   and back again. */
		if (gspca_dev->autogain && gspca_dev->autogain->val && reg10 < 10)
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



	default:
		break;
	}
}

static void setfreq(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;

    KDBG2(gspca_dev->v4l2_dev.dev, "setfreq"," - %d",elgcnt)

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

/* ---- Parâmetros de teste A/B (podem ser mudados em
 *      /sys/module/gspca_elgin_fps/parameters/ ; valem no próximo start) ---- */
static int stock_regs;
module_param(stock_regs, int, 0644);
MODULE_PARM_DESC(stock_regs, "1 = registradores do bridge originais (0 = capturados do Windows)");

static int stock_sensor;
module_param(stock_sensor, int, 0644);
MODULE_PARM_DESC(stock_sensor, "1 = init do sensor original (0 = capturado do Windows)");

static int sensor_reg11 = -1;
module_param(sensor_reg11, int, 0644);
MODULE_PARM_DESC(sensor_reg11, "valor gravado no reg 0x11 do sensor apos iniciar: fps = 30/(valor+1); -1 = nao altera");

static int reg17 = -1;
module_param(reg17, int, 0644);
MODULE_PARM_DESC(reg17, "valor do reg 0x17 do bridge (clock do sensor + qualidade da compressao); -1 = nao altera");

static int reg19 = -1;
module_param(reg19, int, 0644);
MODULE_PARM_DESC(reg19, "valor do reg 0x19 do bridge (modo de sync / MCK_SIZE); -1 = nao altera");

static int lq_sel = -1;
module_param(lq_sel, int, 0644);
MODULE_PARM_DESC(lq_sel, "bit 0 do reg 0x17: 1 = compressao de baixa qualidade, 0 = alta; -1 = nao altera");

/* ---- Taxa de quadros (VIDIOC_G_PARM / VIDIOC_S_PARM) ---- */

/* Escolhe, entre as taxas do modo atual, a maior que não passa de 'want';
 * se todas passarem, a menor disponível. */
static int pick_rate(struct gspca_dev *gspca_dev, int want)
{
	const struct framerates *fr;
	int i;

	if (!gspca_dev->cam.mode_framerates)
		return 30;
	fr = &gspca_dev->cam.mode_framerates[gspca_dev->curr_mode];
	if (!fr->nrates)
		return 30;
	for (i = 0; i < fr->nrates; i++)
		if (fr->rates[i] <= want)
			return fr->rates[i];
	return fr->rates[fr->nrates - 1];
}

/* Taxa em uso: a pedida via S_PARM para o modo atual ou o padrão do modo. */
static int cur_fps(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int mode = gspca_dev->curr_mode;
	int want;

	if (sd->fps_mode != mode) {    /* mudou de modo: volta ao padrao dele */
		sd->fps_mode = mode;
		sd->fps = 0;
		sd->fps_cap = 0;
	}
	if (!gspca_dev->cam.mode_framerates)
		return 30;
	want = sd->fps ? sd->fps : vga_fps_default[mode];
	if (sd->fps_cap && sd->cap_mode == mode && want <= 30 && sd->fps_cap < want)
		want = sd->fps_cap;	/* limite da trava de banda: so no modo em que nasceu e abaixo de 60 fps */
	return pick_rate(gspca_dev, want);
}

/* taxa PEDIDA (S_PARM ou padrao do modo), sem o limite da trava */
static int req_fps(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;

	return pick_rate(gspca_dev, sd->fps ? sd->fps : vga_fps_default[gspca_dev->curr_mode]);
}

/* Valor n do reg 0x11 do sensor (fps = 30 / (n + 1)) para a taxa em uso */
static int fps_reg11_n(struct gspca_dev *gspca_dev)
{
	int fps = cur_fps(gspca_dev);
	int n = (30 + fps / 2) / fps - 1;

	if (n < 0)
		n = 0;
	if (n > 63)
		n = 63;
	if (sensor_reg11 >= 0)
		n = sensor_reg11;
	return n;
}

/* Grava a taxa escolhida no sensor: reg 0x11 (divisor do clock),
 * fps = 30 / (n + 1). O parâmetro sensor_reg11 >= 0 tem prioridade (testes). */
/* Ajuste fino de taxa do sensor (reg 0x2A/0x2B = FRARH/FRARL): N de 10 bits, cada passo
 * diminui a taxa em ~0,105% (medido). Calcula o N para chegar a 'fps' partindo da taxa base
 * 30/(n+1) do divisor do reg 0x11. Retorna 0 se a taxa pedida for >= a base. */
static int fps_fine_n(int fps, int n)
{
	int base_milli = 30000 / (n + 1);
	int diff = base_milli - fps * 1000;
	int fine;

	if (diff <= 0)
		return 0;
	fine = (diff * 952) / base_milli;
	return fine > 1023 ? 1023 : fine;
}

/* grava a taxa de quadros no sensor (reg 0x11 e ajuste fino 0x2A/0x2B). O ganho automatico chama esta
 * funcao a cada ajuste de exposicao: no log nivel 1 so aparece quando a taxa muda (repeticoes no nivel 2) */
static void apply_fps(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u32 key;
	int fps = cur_fps(gspca_dev);
	int n = fps_reg11_n(gspca_dev);
	int fine = (sensor_reg11 >= 0) ? 0 : fps_fine_n(fps, n);
	__u8 i2c11[8] = { 0xa0, 0x21, 0x11, 0x00, 0x00, 0x00, 0x00, 0x10 };

	i2c11[3] = n;
	key = ((u32) fps << 20) | ((u32) n << 12) | fine;	/* fine <= 1023 e n <= 255: nao se sobrepoem */
	if (key != sd->fps_logged) {
		KDBG(gspca_dev->v4l2_dev.dev, "apply_fps"," fps=%d reg11=%d fino=%d - %d", fps, n, fine, elgcnt)
		sd->fps_logged = key;
	} else {
		KDBG2(gspca_dev->v4l2_dev.dev, "apply_fps"," fps=%d reg11=%d fino=%d - %d", fps, n, fine, elgcnt)
	}
	i2c_w(gspca_dev, i2c11);

	if (fine) {
		/* reg 0x2A: bit 7 liga o ajuste, bits 6:5 = 2 bits altos; reg 0x2B = byte baixo */
		__u8 i2cf[8] = { 0xb0, 0x21, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x10 };

		i2cf[3] = 0x80 | (((fine >> 8) & 3) << 5);
		i2cf[4] = fine & 0xff;
		i2c_w(gspca_dev, i2cf);
	} else {
		setfreq(gspca_dev);    /* devolve o reg 0x2B ao valor do controle de frequencia da rede */
	}
}

static void sd_get_streamparm(struct gspca_dev *gspca_dev, struct v4l2_streamparm *parm)
{
	struct v4l2_captureparm *cp = &parm->parm.capture;

	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = cur_fps(gspca_dev);
}

static void sd_set_streamparm(struct gspca_dev *gspca_dev, struct v4l2_streamparm *parm)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct v4l2_captureparm *cp = &parm->parm.capture;
	u32 num = cp->timeperframe.numerator;
	u32 den = cp->timeperframe.denominator;
	int want = 30;
	int nova;

	if (num && den)
		want = (den + num / 2) / num;	/* arredonda: 59,94 -> 60 */

	sd->fps_mode = gspca_dev->curr_mode;
	nova = pick_rate(gspca_dev, want);
	if (gspca_dev->streaming && ((nova > 30) != !!sd->hi))
		nova = sd->hi ? 60 : 30;	/* entrar/sair dos 60 fps exige reiniciar a captura */
	sd->fps = nova;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = sd->fps;

	if (gspca_dev->streaming)
		apply_fps(gspca_dev);
}

/* ---- Acesso generico a registradores (investigar configuracoes nao documentadas) ----
 * Um item tem a forma TIPO:REG=VALOR, onde TIPO e  b = ponte SN9C103  ou  s = sensor OV7630.
 *   regw        (escrita)  aplica na hora:    echo "b:0x1a=0x1d" | sudo tee .../parameters/regw
 *   regr        (leitura)  le um registrador: echo "s:0x0a" | sudo tee .../parameters/regr ; cat .../parameters/regr
 *   regs_start  lista de itens (separados por virgula), reaplicada no FIM de todo inicio de captura
 * Obs.: a ponte devolve 0 em varios registradores; a leitura do sensor funciona.
 */
static char regs_start[256];
module_param_string(regs_start, regs_start, sizeof(regs_start), 0644);
MODULE_PARM_DESC(regs_start, "itens TIPO:REG=VALOR separados por virgula, reaplicados no fim de cada inicio de captura (b = ponte, s = sensor)");

static void reg_item_write(struct gspca_dev *gspca_dev, char kind, unsigned int reg, unsigned int val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (kind == 'b') {
		__u8 v = val;

		reg_w(gspca_dev, reg, &v, 1);
	} else {
		__u8 i2c[8] = { 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };

		i2c[1] = sensor_data[sd->sensor].sensor_addr;
		i2c[2] = reg;
		i2c[3] = val;
		i2c_w(gspca_dev, i2c);
	}
}

/* aplica uma lista de itens TIPO:REG=VALOR; nao mexe em usb_err (quem chama decide) */
static void reg_list_apply(struct gspca_dev *gspca_dev, const char *list)
{
	char buf[sizeof(regs_start)];
	char *p = buf, *tok;
	char kind;
	unsigned int reg, val;

	strscpy(buf, list, sizeof(buf));
	while ((tok = strsep(&p, ", ;\n\t")) != NULL) {
		if (!*tok)
			continue;
		if (sscanf(tok, "%c:%i=%i", &kind, &reg, &val) != 3) {
			dev_err(gspca_dev->v4l2_dev.dev, "registrador: item invalido '%s'\n", tok);
			continue;
		}
		if (kind == 'B')
			kind = 'b';
		else if (kind == 'S')
			kind = 's';
		if (kind != 'b' && kind != 's') {
			dev_err(gspca_dev->v4l2_dev.dev, "registrador: tipo invalido em '%s'\n", tok);
			continue;
		}
		dev_info(gspca_dev->v4l2_dev.dev, "registrador: %c:0x%02x = 0x%02x\n", kind, reg, val);
		reg_item_write(gspca_dev, kind, reg, val);
	}
}

static int regw_set(const char *val, const struct kernel_param *kp)
{
	struct gspca_dev *g = elg_dev;
	int ret;

	if (!g)
		return -ENODEV;
	mutex_lock(&g->usb_lock);
	if (!g->present) {
		mutex_unlock(&g->usb_lock);
		return -ENODEV;
	}
	g->usb_err = 0;
	reg_list_apply(g, val);
	ret = g->usb_err;
	g->usb_err = 0;
	mutex_unlock(&g->usb_lock);
	return ret;
}

static const struct kernel_param_ops regw_ops = {
	.set = regw_set,
};
module_param_cb(regw, &regw_ops, NULL, 0200);
MODULE_PARM_DESC(regw, "escrita imediata TIPO:REG=VALOR (b = ponte, s = sensor)");

static char regr_out[64] = "(nada lido ainda)";

static int regr_set(const char *val, const struct kernel_param *kp)
{
	struct gspca_dev *g = elg_dev;
	char kind;
	unsigned int reg;

	if (!g)
		return -ENODEV;
	if (sscanf(val, "%c:%i", &kind, &reg) != 2)
		return -EINVAL;
	if (kind == 'B')
		kind = 'b';
	else if (kind == 'S')
		kind = 's';
	if (kind != 'b' && kind != 's')
		return -EINVAL;
	mutex_lock(&g->usb_lock);
	if (!g->present) {
		mutex_unlock(&g->usb_lock);
		return -ENODEV;
	}
	g->usb_err = 0;
	if (kind == 'b')
		reg_r(g, reg);
	else
		i2c_r(g, reg, 1);
	if (g->usb_err < 0)
		snprintf(regr_out, sizeof(regr_out), "%c:0x%02x erro %d", kind, reg, g->usb_err);
	else
		snprintf(regr_out, sizeof(regr_out), "%c:0x%02x = 0x%02x  (cru: %02x %02x %02x %02x %02x)", kind, reg, g->usb_buf[0],
			 i2c_raw[0], i2c_raw[1], i2c_raw[2], i2c_raw[3], i2c_raw[4]);
	g->usb_err = 0;
	mutex_unlock(&g->usb_lock);
	return 0;
}

static int regr_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%s\n", regr_out);
}

static const struct kernel_param_ops regr_ops = {
	.set = regr_set,
	.get = regr_get,
};
module_param_cb(regr, &regr_ops, NULL, 0644);
MODULE_PARM_DESC(regr, "escreva TIPO:REG (ex.: s:0x0a) e leia o arquivo para ver o valor");

/* ---- Nomes da ponte e do sensor, e ganchos padrao VIDIOC_DBG_* ---- */

static const char *bridge_name(u8 bridge)
{
	return bridge == BRIDGE_103 ? "SN9C103" : "SN9C102";
}

static const char *sensor_name(u8 sensor)
{
	static const char * const nomes[] = {
		[SENSOR_HV7131D]   = "HV7131D",
		[SENSOR_HV7131R]   = "HV7131R",
		[SENSOR_OV6650]    = "OV6650",
		[SENSOR_OV7630]    = "OV7630",
		[SENSOR_PAS106]    = "PAS106",
		[SENSOR_PAS202]    = "PAS202",
		[SENSOR_TAS5110C]  = "TAS5110C",
		[SENSOR_TAS5110D]  = "TAS5110D",
		[SENSOR_TAS5130CXX] = "TAS5130CXX",
	};

	return sensor < ARRAY_SIZE(nomes) && nomes[sensor] ? nomes[sensor] : "sensor?";
}

/* campo 'card' do VIDIOC_QUERYCAP (32 bytes, 31 caracteres): marca + (ponte + sensor) */
static void elg_card_name(struct gspca_dev *gspca_dev, char *buf, size_t len)
{
	struct sd *sd = (struct sd *) gspca_dev;

	snprintf(buf, len, "Elgin camera (%s + %s)", bridge_name(sd->bridge), sensor_name(sd->sensor));
}

/* nome da entrada (VIDIOC_ENUMINPUT) */
static void elg_input_name(struct gspca_dev *gspca_dev, char *buf, size_t len)
{
	struct sd *sd = (struct sd *) gspca_dev;

	snprintf(buf, len, "%s + %s", bridge_name(sd->bridge), sensor_name(sd->sensor));
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
/* Chips do dispositivo (VIDIOC_DBG_G_CHIP_INFO): indice 0 = ponte, 1 = sensor.
 * Use com:  v4l2-dbg -d /dev/videoN --list-chips
 *           v4l2-dbg -d /dev/videoN -c 1 -g 0x0a      (le o PID do sensor)
 * Exige um kernel com CONFIG_VIDEO_ADV_DEBUG e privilegio de root. */
static int sd_chip_info(struct gspca_dev *gspca_dev, struct v4l2_dbg_chip_info *chip)
{
	struct sd *sd = (struct sd *) gspca_dev;

	switch (chip->match.addr) {
	case 0:
		strscpy(chip->name, bridge_name(sd->bridge), sizeof(chip->name));
		return 0;
	case 1:
		strscpy(chip->name, sensor_name(sd->sensor), sizeof(chip->name));
		return 0;
	}
	return -EINVAL;
}

static int sd_dbg_g_register(struct gspca_dev *gspca_dev, struct v4l2_dbg_register *reg)
{
	if (reg->reg > 0xff)
		return -EINVAL;
	switch (reg->match.addr) {
	case 0:
		reg_r(gspca_dev, reg->reg);          /* a ponte devolve 0 em varios registradores */
		break;
	case 1:
		i2c_r(gspca_dev, reg->reg, 1);
		break;
	default:
		return -EINVAL;
	}
	reg->size = 1;
	reg->val = gspca_dev->usb_buf[0];
	return gspca_dev->usb_err;
}

static int sd_dbg_s_register(struct gspca_dev *gspca_dev, const struct v4l2_dbg_register *reg)
{
	if (reg->reg > 0xff || reg->val > 0xff)
		return -EINVAL;
	switch (reg->match.addr) {
	case 0:
		reg_item_write(gspca_dev, 'b', reg->reg, reg->val);
		break;
	case 1:
		reg_item_write(gspca_dev, 's', reg->reg, reg->val);
		break;
	default:
		return -EINVAL;
	}
	return gspca_dev->usb_err;
}
#endif /* CONFIG_VIDEO_ADV_DEBUG */

/* Taxa real em milli-Hz: 30000/(n+1) com o ajuste fino (cada passo ~0,105%) aplicado */
static int fps_actual_milli(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int fps = cur_fps(gspca_dev);
	int n = fps_reg11_n(gspca_dev);
	int fine = (sensor_reg11 >= 0) ? 0 : fps_fine_n(fps, n);
	int base = 30000 / (n + 1);

	if (sd->hi)                    /* modo de 60 fps: o sensor entrelacado dobra a taxa */
		base *= 2;
	return base / 100 * (100000 - fine * 105) / 1000;
}

/* Trava de banda do S910 (controle "dynamic framerate"). O tamanho do quadro comprimido cresce com o
 * ruido (cena escura, ganho alto) e, perto de ~90% do alt, o canal satura e a imagem desliza. O que
 * deforma sao os quadros GRANDES, nao a media: por isso a decisao usa o MAXIMO recente do tamanho dos
 * quadros (decai ~0,4% por quadro: memoria de ~20 s). Ocupacao = maximo x taxa real / banda do alt em uso.
 *  - passa de 90%: conta +1 (e -1 a cada avaliacao abaixo); com 6 pontos desce um degrau da tabela
 *  - com a taxa limitada e a previsao no degrau acima < 80% por 96 avaliacoes: sobe um degrau (ate a pedida)
 * O limite persiste entre as capturas e some ao mudar de modo ou com o controle desligado.
 * Roda em do_autogain(), que tem contexto de processo e o usb_lock. */
static void rate_guard(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	const struct framerates *fr;
	int size, cur, want, i, cand, util_pm;

	if (sd->hi)					/* modo de 60 fps: o canal esta com ~30%, a trava nao se aplica */
		return;
	if (sd->fps_cap && sd->cap_mode != gspca_dev->curr_mode)
		sd->fps_cap = 0;
	if (!sd->link_bw || !gspca_dev->cam.mode_framerates ||
		(gspca_dev->cam.cam_mode[gspca_dev->curr_mode].priv & MODE_RAW))
		return;                                    /* so o S910 (comprimido) varia de tamanho */
	if (!sd->dynfps || !sd->dynfps->val) {
		if (sd->fps_cap) {                         /* controle desligado: volta a taxa pedida */
			sd->fps_cap = 0;
			apply_fps(gspca_dev);
		}
		return;
	}
	size = atomic_read(&sd->frame_len);
	if (size <= 0)
		return;
	sd->size_peak -= sd->size_peak / 256;          /* decaimento lento */
	if (size > sd->size_peak)
		sd->size_peak = size;

	fr = &gspca_dev->cam.mode_framerates[gspca_dev->curr_mode];
	cur = cur_fps(gspca_dev);
	want = req_fps(gspca_dev);
	util_pm = (int) div_u64((u64) sd->size_peak * fps_actual_milli(gspca_dev), sd->link_bw);

	if (util_pm > 900) {                           /* canal perto de saturar */
		sd->guard_lo = 0;
		if (++sd->guard_hi < 6)
			return;
		sd->guard_hi = 0;
		for (i = 0; i < fr->nrates; i++) {         /* tabela em ordem decrescente: primeira menor que a atual */
			if (fr->rates[i] < cur) {
				sd->fps_cap = fr->rates[i];
				sd->cap_mode = gspca_dev->curr_mode;
				dev_info(gspca_dev->v4l2_dev.dev,
						 "trava de banda: ocupacao (pico) %d,%d%% -> %d fps\n",
						 util_pm / 10, util_pm % 10, sd->fps_cap);
				apply_fps(gspca_dev);
				break;
			}
		}
		return;
	}
	if (sd->guard_hi)                              /* tolera uma avaliacao abaixo: desconta 1 */
		sd->guard_hi--;
	if (!sd->fps_cap)
		return;
	/* ha folga? degrau acima da taxa atual, sem passar da pedida */
	cand = 0;
	for (i = 0; i < fr->nrates; i++)
		if (fr->rates[i] > cur && fr->rates[i] <= want && fr->rates[i] <= 30 && (!cand || fr->rates[i] < cand))
			cand = fr->rates[i];
	if (cand && (int) div_u64((u64) sd->size_peak * (cand * 1000), sd->link_bw) < 800) {
		if (++sd->guard_lo >= 96) {
			sd->guard_lo = 0;
			sd->fps_cap = (cand >= want) ? 0 : cand;
			dev_info(gspca_dev->v4l2_dev.dev, "trava de banda: folga -> %d fps\n", cand);
			apply_fps(gspca_dev);
		}
	} else {
		sd->guard_lo = 0;
	}
}

/* Ganho e exposicao automaticos que PRESERVAM a taxa de quadros.
 * Mesmo algoritmo de "joelho" do gspca_expo_autogain(), mas a exposicao fica limitada ao
 * tempo de quadro ('exposure_max'): acima disso o sensor nao expoe mais (reg10 satura) e
 * o algoritmo gastaria quadros subindo uma exposicao sem efeito. A taxa nunca e mexida. */
static int elg_expo_autogain(struct gspca_dev *gspca_dev, int avg_lum, int desired_avg_lum,
							 int deadzone, int gain_knee, int exposure_knee, int exposure_max)
{
	s32 gain, orig_gain, exposure, orig_exposure;
	int i, steps, retval = 0;

	if (v4l2_ctrl_g_ctrl(gspca_dev->autogain) == 0)
		return 0;

	orig_gain = gain = v4l2_ctrl_g_ctrl(gspca_dev->gain);
	orig_exposure = exposure = v4l2_ctrl_g_ctrl(gspca_dev->exposure);
	if (exposure_knee > exposure_max)
		exposure_knee = exposure_max;
	if (exposure > exposure_max)
		exposure = exposure_max;

	/* se estiver longe do alvo, faz varios passos de uma vez (com risco de passar um pouco) */
	steps = abs(desired_avg_lum - avg_lum) / deadzone;
	for (i = 0; i < steps; i++) {
		if (avg_lum > desired_avg_lum) {                      /* escurecer */
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
		} else {                                              /* clarear */
			if (gain < gspca_dev->gain->default_value)
				gain++;
			else if (exposure < exposure_knee)
				exposure++;
			else if (gain < gain_knee)
				gain++;
			else if (exposure < exposure_max)
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
	return retval;
}

static void contraste_adapta(struct gspca_dev *gspca_dev);

static void do_autogain(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int deadzone, desired_avg_lum, avg_lum, exposure_max;

	rate_guard(gspca_dev);
	contraste_adapta(gspca_dev);		/* o contraste acompanha o tom medio da cena (com ou sem automatico) */

	if (!gspca_dev->autogain || !gspca_dev->gain || !gspca_dev->exposure)
		return;
	if (v4l2_ctrl_g_ctrl(gspca_dev->autogain) == 0)
		return;

	avg_lum = atomic_read(&sd->avg_lum);
	if (avg_lum == -1)
		return;

	if (sd->autogain_ignore_frames > 0) {
		sd->autogain_ignore_frames--;
		return;
	}

	/* o avg_lum do cabecalho depende da janela de AE (regs 0x1C-0x1F = 02 03 0F 0C) */
	if (sensor_data[sd->sensor].flags & F_SIF) {
		deadzone = 500;
		desired_avg_lum = 5000;
	} else {
		deadzone = 1500;
		desired_avg_lum = 13000;
	}

	/* alvo de luminosidade do ganho automatico: controle proprio "AE target" (o brilho agora age no
	 * sensor, reg 0x06 com BRTEN ligado, e nao se mistura com o automatico) */
	if (sd->ae_target)
		desired_avg_lum = sd->ae_target->val * desired_avg_lum / 127;
	/* o avg_lum e' a soma de luminosidade da janela de AE: o alvo acompanha a area (117 = padrao) */
	if (sd->ae_area) {
		desired_avg_lum = desired_avg_lum * sd->ae_area / 117;
		deadzone = deadzone * sd->ae_area / 117;
	}

	/* exposicao maxima util = tempo de quadro: o setexposure() mapeia a exposicao (unidades de
	 * 0,5 ms) em reg10 com reg11 vindo da taxa; reg10 satura em ~67 unidades por passo de reg11 */
	exposure_max = 67 * (fps_reg11_n(gspca_dev) + 1);
	if (sd->hi)
		exposure_max /= 2;		/* 60 fps: o tempo de quadro e' metade */
	if (exposure_max > gspca_dev->exposure->maximum)
		exposure_max = gspca_dev->exposure->maximum;

	if (elg_expo_autogain(gspca_dev, avg_lum, desired_avg_lum, deadzone,
						  (s32) gspca_dev->gain->maximum * 9 / 10, sd->exposure_knee, exposure_max))
		sd->autogain_ignore_frames = AUTOGAIN_IGNORE_FRAMES;
}

/// *************************************************************************************************************
/// *************************************************************************************************************
/// *************************************************************************************************************





/* this function is called at probe time */
static int sd_config(struct gspca_dev *gspca_dev, const struct usb_device_id *id)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam;

    KDBG(gspca_dev->v4l2_dev.dev, "sd_config"," - %d",elgcnt)

	reg_r(gspca_dev, 0x00);
	if (gspca_dev->usb_buf[0] != 0x10)  // josemar: no endereço 00h do controlador tem um Sonix PC Cam chip ID = 10h
		return -ENODEV;

	/* copy the webcam info from the device id */ // josemar: proveniente da tabela dos dispositivos aceitos pelo driver
	sd->sensor = id->driver_info >> 8;
	sd->bridge = id->driver_info & 0xff;

	cam = &gspca_dev->cam;
	if (!(sensor_data[sd->sensor].flags & F_SIF)) {
		cam->cam_mode = vga_mode;
		cam->nmodes = ARRAY_SIZE(vga_mode);
		BUILD_BUG_ON(ARRAY_SIZE(vga_framerates) != ARRAY_SIZE(vga_mode));
		BUILD_BUG_ON(ARRAY_SIZE(vga_fps_default) != ARRAY_SIZE(vga_mode));
		cam->mode_framerates = vga_framerates;
	} else {
		cam->cam_mode = sif_mode;
		cam->nmodes = ARRAY_SIZE(sif_mode);
	}
	cam->npkt = 36;			/* 36 packets per ISOC message */

	return 0;
}

/* this function is called at probe and resume time */
static int sd_init(struct gspca_dev *gspca_dev)
{
	const __u8 stop = 0x09; /* Disable stream turn of LED */

    KDBG(gspca_dev->v4l2_dev.dev, "sd_init"," - %d",elgcnt)
	elg_dev = gspca_dev;

	reg_w(gspca_dev, 0x01, &stop, 1);  // josemar: bit0 = 1: desliga o sensor; bit3 é o output do led

	return gspca_dev->usb_err;
}


/* Correcao do H_START com o espelho ligado. Medido (testa_controles.py): com +1 a fase do mosaico saia trocada,
 * entao o sensor ja' inverte a paridade ao espelhar e nao precisa de correcao */
#define HFLIP_HSTART_ADJ 0
#define VFLIP_VSTART_ADJ 0	/* a medir com testa_novos.py: +1 se a fase Bayer trocar */

/* Curvas de gama da ponte: 17 pontos (regs 0x20 a 0x30), out = 255 * (i/16)^(1/g), g = 0,6 + 0,2 k. k = 2 e' a linear (padrao) */
static const __u8 gamma_tb[9][17] = {
	{   0,   3,   8,  16,  25,  37,  50,  64,  80,  98, 117, 137, 158, 180, 204, 229, 255 },
	{   0,   8,  19,  31,  45,  60,  75,  91, 107, 124, 142, 160, 178, 197, 216, 235, 255 },
	{   0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255 },
	{   0,  25,  45,  63,  80,  97, 113, 128, 143, 158, 172, 187, 201, 214, 228, 242, 255 },
	{   0,  35,  58,  77,  95, 111, 127, 141, 155, 169, 182, 195, 208, 220, 232, 244, 255 },
	{   0,  45,  70,  90, 107, 123, 138, 152, 165, 178, 190, 202, 213, 224, 235, 245, 255 },
	{   0,  55,  80, 101, 118, 134, 148, 161, 174, 185, 196, 207, 217, 227, 237, 246, 255 },
	{   0,  64,  90, 110, 128, 143, 156, 169, 180, 191, 202, 211, 221, 230, 239, 247, 255 },
	{   0,  72,  99, 119, 136, 150, 163, 175, 186, 196, 206, 215, 224, 232, 240, 248, 255 },
};

static const char * const test_menu[] = { "Disabled", "Bridge test image", "Sensor color bars" };

/* tom medio da cena (0..255) a partir da soma de luminosidade do cabecalho (janela de AE): tom ~ soma x
 * PIVO_K / (area x 256). Calibracao inicial: com o alvo padrao (soma 13000 em 117 blocos) a media medida
 * na janela foi ~51 (NOTAS, validacao do ganho automatico de 2026-10-11) -> 0,46 */
#define PIVO_K 118

/* contraste em S com as pontas presas (0 -> 0, 255 -> 255) e apoio no tom medio p: abaixo de p,
 * y = p.x / (k.p - (k-1).x); acima, o espelho. A inclinacao em p e' k: k > 1 escurece abaixo e clareia
 * acima de p; k < 1 aproxima os tons de p sem perder o preto e o branco. Contas inteiras (k em 1/256). */
static int curva_s(int x, int p, int kq)
{
	int a, b;

	if (kq == 256)
		return x;
	if (x <= p)
		return p * x * 256 / (kq * p - (kq - 256) * x);
	a = 255 - x;
	b = 255 - p;
	return 255 - b * a * 256 / (kq * b - (kq - 256) * a);
}

/* curva da ponte (regs 0x20 a 0x30, 17 pontos, entrada i x 16): primeiro o contraste em S apoiado no tom
 * medio da cena (sd->pivo), depois a gama da tabela gamma_tb (interpolada). contraste 128 = sem mudanca
 * (k = 1); 255 -> k = 3; 0 -> k = 1/3. Substitui o contraste do sensor (reg 0x05), que so age em YUV. */
static void curva(struct sd *sd, __u8 *tb)
{
	int i, x, y, gi, c = sd->contrast ? sd->contrast->val : 128;
	int p = sd->pivo ? sd->pivo : 64;
	int kq = c >= 128 ? 256 + (c - 128) * 512 / 127 : 256 - (128 - c) * 171 / 128;
	const __u8 *g = gamma_tb[sd->gamma ? sd->gamma->val : 2];

	p = p < 16 ? 16 : p > 224 ? 224 : p;
	for (i = 0; i < 17; i++) {
		x = i == 16 ? 255 : i * 16;
		y = curva_s(x, p, kq);				/* contraste */
		y = y < 0 ? 0 : y > 255 ? 255 : y;
		gi = y >> 4;					/* gama: interpolacao entre os pontos da tabela */
		y = (gi >= 16 || y >= 255) ? g[16] : g[gi] + ((int)g[gi + 1] - g[gi]) * (y & 15) / 16;
		tb[i] = y < 0 ? 0 : y > 255 ? 255 : y;
	}
}

/* grava a curva (gama + contraste) na ponte, ao vivo */
static void setgamma(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 tb[17];

	curva(sd, tb);
	reg_w(gspca_dev, 0x20, tb, sizeof(tb));
}

/* contraste adaptativo: acompanha o tom medio da cena (soma de luminosidade do cabecalho, a cada quadro)
 * e regrava a curva quando ele muda mais de 8 niveis, no maximo a cada 10 quadros. Com contraste 128
 * (sem mudanca) nao faz nada. Roda em do_autogain (contexto de processo, depois de cada quadro). */
static void contraste_adapta(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int lum, p, pn;

	if (!sd->contrast || sd->contrast->val == 128 || !sd->ae_area)
		return;
	lum = atomic_read(&sd->avg_lum);
	if (lum < 0)
		return;
	p = lum * PIVO_K / (sd->ae_area * 256);
	p = p < 16 ? 16 : p > 224 ? 224 : p;
	if (!sd->pivo_f)
		sd->pivo_f = p * 16;
	sd->pivo_f += (p * 16 - sd->pivo_f) / 4;	/* filtro: sem saltos de quadro a quadro */
	if (sd->pivo_espera) {
		sd->pivo_espera--;
		return;
	}
	pn = sd->pivo_f / 16;
	if (sd->pivo && abs(pn - sd->pivo) <= 8)
		return;
	sd->pivo = pn;
	sd->pivo_espera = 10;
	setgamma(gspca_dev);
	KDBG2(gspca_dev->v4l2_dev.dev, "contraste_adapta", " tom medio %d (soma %d, area %d) - %d", pn, lum, sd->ae_area, elgcnt)
}

static void setblack(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 v = sd->black->val;

	reg_w(gspca_dev, 0x14, &v, 1);
}

/* COMA (0x12) do sensor: base 0x78 (init do Windows, bit 6 ligado). HFLIP=1 desliga o bit 6; barras de cor = bit 1 */
/* (o #define HFLIP_HSTART_ADJ fica acima, antes de gamma_tb) */
static void setcoma(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 i2c[8] = { 0xa0, 0x00, 0x12, 0x78, 0x00, 0x00, 0x00, 0x10 };
	__u8 hs;

	if (sd->sensor != SENSOR_OV7630 || !sd->hflip || !sd->testpat)
		return;
	i2c[1] = sensor_data[sd->sensor].sensor_addr;
	if (sd->hflip->val)
		i2c[3] &= ~0x40;
	if (sd->testpat->val == 2)
		i2c[3] |= 0x02;
	if (sd->awb && sd->awb->val)
		i2c[3] |= 0x04;		/* AWB: balanco de branco automatico do sensor */
	i2c_w(gspca_dev, i2c);
	/* o espelho troca a fase do mosaico Bayer: H_START +1 mantem o BGGR */
	hs = sd->hstart + (sd->hflip->val ? HFLIP_HSTART_ADJ : 0);
	reg_w(gspca_dev, 0x12, &hs, 1);
}

/* inversao vertical: COMN (0x75) bit 7, resto 0x0e como no init; V_START da ponte corrige a fase Bayer */
static void setvflip(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 i2c[8] = { 0xa0, 0x00, 0x75, 0x0e, 0x00, 0x00, 0x00, 0x10 };
	__u8 vs;

	if (sd->sensor != SENSOR_OV7630 || !sd->vflip)
		return;
	i2c[1] = sensor_data[sd->sensor].sensor_addr;
	if (sd->vflip->val)
		i2c[3] |= 0x80;
	i2c_w(gspca_dev, i2c);
	vs = sd->vstart + (sd->vflip->val ? VFLIP_VSTART_ADJ : 0);
	reg_w(gspca_dev, 0x13, &vs, 1);
}

/* filtro de banda: COMJ (0x2D) bit 2; base 0x81 (0xc1 no modo 'hi', como no init) */
static void setbandf(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 i2c[8] = { 0xa0, 0x00, 0x2d, 0x81, 0x00, 0x00, 0x00, 0x10 };

	if (sd->sensor != SENSOR_OV7630 || !sd->bandf)
		return;
	i2c[1] = sensor_data[sd->sensor].sensor_addr;
	if (sd->hi)
		i2c[3] = 0xc1;
	if (sd->bandf->val)
		i2c[3] |= 0x04;
	i2c_w(gspca_dev, i2c);
}

/* alvos do AWB: 'fundo' azul (0x0C) e vermelho (0x0D), 0..63 */
static void setawbt(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 i2c[8] = { 0xb0, 0x00, 0x0c, 0x20, 0x20, 0x00, 0x00, 0x10 };

	if (sd->sensor != SENSOR_OV7630 || !sd->abg_blue || !sd->abg_red)
		return;
	i2c[1] = sensor_data[sd->sensor].sensor_addr;
	i2c[3] = sd->abg_blue->val;
	i2c[4] = sd->abg_red->val;
	i2c_w(gspca_dev, i2c);
}

static void settest(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 v = sd->reg17;

	if (sd->testpat->val == 1)
		v |= 0x10;
	reg_w(gspca_dev, 0x17, &v, 1);
	setcoma(gspca_dev);
}

#define V4L2_CID_AWB_BG (V4L2_CID_USER_BASE + 0x1f02)	/* controles proprios: alvos do AWB */
#define V4L2_CID_AWB_RG (V4L2_CID_USER_BASE + 0x1f03)
#define V4L2_CID_RED_GAIN_BR (V4L2_CID_USER_BASE + 0x1f04)	/* controles proprios: ganhos R, G e B da ponte */
#define V4L2_CID_GREEN_GAIN_BR (V4L2_CID_USER_BASE + 0x1f05)
#define V4L2_CID_BLUE_GAIN_BR (V4L2_CID_USER_BASE + 0x1f06)
#define V4L2_CID_AE_WINDOW (V4L2_CID_USER_BASE + 0x1f07)	/* controles proprios: janela da exposicao automatica */
#define V4L2_CID_AE_START_X (V4L2_CID_USER_BASE + 0x1f08)
#define V4L2_CID_AE_START_Y (V4L2_CID_USER_BASE + 0x1f09)
#define V4L2_CID_AE_END_X (V4L2_CID_USER_BASE + 0x1f0a)
#define V4L2_CID_AE_END_Y (V4L2_CID_USER_BASE + 0x1f0b)
#define V4L2_CID_AE_TARGET (V4L2_CID_USER_BASE + 0x1f0c)	/* controle proprio: alvo do ganho automatico */

static const char * const ae_win_menu[] = {
	"Mode default", "Full", "Center", "Left half", "Right half", "Top half", "Bottom half", "Custom"
};

static u8 ae_clamp(int v, int max)
{
	return v < 0 ? 0 : v > max ? max : v;
}

/* calcula a janela de AE (regs 0x1C a 0x1F, blocos de 32 px) a partir do menu; limites dos registradores
 * (mascaras medidas): STRX 7, STRY 7, ENDX 31, ENDY 15 */
static void ae_window(struct sd *sd, u8 *v)
{
	int w = sd->ae_w, h = sd->ae_h;
	int sx = sd->ae_def[0], sy = sd->ae_def[1], ex = sd->ae_def[2], ey = sd->ae_def[3];

	switch (sd->ae_win ? sd->ae_win->val : 0) {
	case 1: sx = 0; sy = 0; ex = w; ey = h; break;
	case 2: sx = w / 4; sy = h / 4; ex = w - w / 4; ey = h - h / 4; break;
	case 3: sx = 0; sy = 0; ex = w / 2; ey = h; break;
	case 4: sx = w / 2; sy = 0; ex = w; ey = h; break;
	case 5: sx = 0; sy = 0; ex = w; ey = h / 2; break;
	case 6: sx = 0; sy = h / 2; ex = w; ey = h; break;
	case 7:
		sx = sd->ae_sx->val; sy = sd->ae_sy->val; ex = sd->ae_ex->val; ey = sd->ae_ey->val;
		break;
	}
	v[0] = ae_clamp(sx, 7);
	v[1] = ae_clamp(sy, 7);
	v[2] = ae_clamp(ex, 31);
	v[3] = ae_clamp(ey, 15);
}

/* area em blocos (o avg_lum do chip e' a SOMA da luminosidade da janela: o alvo do ganho automatico
 * acompanha a area, ver do_autogain); janela invertida ou vazia conta como 1 */
static void ae_set_area(struct sd *sd, const u8 *v)
{
	int a = (v[2] - v[0]) * (v[3] - v[1]);

	sd->ae_area = (v[2] > v[0] && v[3] > v[1] && a > 0) ? a : 1;
}

static void setaewin(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u8 v[4];

	if (!sd->ae_win)
		return;
	ae_window(sd, v);
	ae_set_area(sd, v);
	reg_w(gspca_dev, 0x1c, v, 4);
	sd->autogain_ignore_frames = AUTOGAIN_IGNORE_FRAMES;	/* deixa a exposicao se acomodar */
}

/* Balanco de branco do sensor: AWB ligado (COMA bit 2). Desligado: ganhos R/B do sensor nos valores do init do
 * Windows (reg 0x01 = 0x77, 0x02 = 0x3a); o ajuste manual fica nos ganhos da ponte (setgains). */
static void setwb(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 i2c[8] = { 0xb0, 0x00, 0x01, 0x77, 0x3a, 0x00, 0x00, 0x10 };	/* 2 bytes: regs 0x01 e 0x02 */

	if (!sd->awb)
		return;
	setcoma(gspca_dev);			/* bit AWB */
	if (!sd->awb->val) {
		i2c[1] = sensor_data[sd->sensor].sensor_addr;
		i2c_w(gspca_dev, i2c);
	}
}

/* Ganhos da ponte: 0x05 age no pixel (1,0), 0x06 no (0,1) e 0x07 nos dois diagonais (0,0) e (1,1) do mosaico
 * (por POSICAO, nao por cor). Por formato:
 *   GBRG: 0x05 = R, 0x06 = B, 0x07 = G      GRBG: 0x05 = B, 0x06 = R, 0x07 = G
 *   BGGR e RGGB: 0x05 e 0x06 = os dois G, 0x07 = R e B juntos (nao separaveis: segue o ganho R; o B nao age) */
static void setgains(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u8 g[3], r, gr, b;

	if (!sd->rgain || !sd->ggain || !sd->bgain)
		return;
	r = sd->rgain->val;
	gr = sd->ggain->val;
	b = sd->bgain->val;
	switch (gspca_dev->pixfmt.pixelformat) {
	case V4L2_PIX_FMT_SGBRG8:
		g[0] = r; g[1] = b; g[2] = gr;
		break;
	case V4L2_PIX_FMT_SGRBG8:
		g[0] = b; g[1] = r; g[2] = gr;
		break;
	default:				/* BGGR (BA81 e S910) e RGGB */
		g[0] = g[1] = gr; g[2] = r;
		break;
	}
	reg_w(gspca_dev, 0x05, g, 3);
}

static int sd_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gspca_dev *gspca_dev = container_of(ctrl->handler, struct gspca_dev, ctrl_handler);
	struct sd *sd = (struct sd *)gspca_dev;

    KDBG2(gspca_dev->v4l2_dev.dev, "sd_s_ctrl"," - %d",elgcnt)

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

	/* os quatro numeros da janela personalizada so valem com o menu em "Custom": nos outros ficam inativos */
	if (ctrl->id == V4L2_CID_AE_WINDOW && sd->ae_sx) {
		bool custom = ctrl->val == 7;

		v4l2_ctrl_activate(sd->ae_sx, custom);
		v4l2_ctrl_activate(sd->ae_sy, custom);
		v4l2_ctrl_activate(sd->ae_ex, custom);
		v4l2_ctrl_activate(sd->ae_ey, custom);
	}

	if (!gspca_dev->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		setbrightness(gspca_dev);
		KDBG2(gspca_dev->v4l2_dev.dev, "sd_s_ctrl - setbrightness"," - %d",elgcnt)
		break;
	case V4L2_CID_AUTOGAIN:
		if (gspca_dev->exposure->is_new || (ctrl->is_new && ctrl->val)){
			setexposure(gspca_dev);
			apply_fps(gspca_dev);
			KDBG2(gspca_dev->v4l2_dev.dev, "sd_s_ctrl - setexposure"," - %d",elgcnt)
        }
		if (gspca_dev->gain->is_new || (ctrl->is_new && ctrl->val)){
			setgain(gspca_dev);
			KDBG2(gspca_dev->v4l2_dev.dev, "sd_s_ctrl - setgain"," - %d",elgcnt)
        }
		break;
	case V4L2_CID_EXPOSURE:			/* sem o automatico, ganho e exposicao chegam separados */
		setexposure(gspca_dev);
		apply_fps(gspca_dev);
		break;
	case V4L2_CID_GAIN:
		setgain(gspca_dev);
		break;
	case V4L2_CID_EXPOSURE_AUTO_PRIORITY:
		if (!ctrl->val && sd->fps_cap) {
			sd->fps_cap = 0;
			apply_fps(gspca_dev);
		}
		break;
	case V4L2_CID_CONTRAST:
		setgamma(gspca_dev);			/* contraste entra na curva da ponte */
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		setwb(gspca_dev);
		break;
	case V4L2_CID_RED_GAIN_BR:
	case V4L2_CID_GREEN_GAIN_BR:
	case V4L2_CID_BLUE_GAIN_BR:
		setgains(gspca_dev);
		break;
	case V4L2_CID_BLACK_LEVEL:
		setblack(gspca_dev);
		break;
	case V4L2_CID_GAMMA:
		setgamma(gspca_dev);
		break;
	case V4L2_CID_HFLIP:
		setcoma(gspca_dev);
		break;
	case V4L2_CID_VFLIP:
		setvflip(gspca_dev);
		break;
	case V4L2_CID_BAND_STOP_FILTER:
		setbandf(gspca_dev);
		break;
	case V4L2_CID_AE_TARGET:
		sd->autogain_ignore_frames = 0;	/* o automatico passa a buscar o alvo novo ja no proximo quadro */
		break;
	case V4L2_CID_AWB_BG:
	case V4L2_CID_AWB_RG:
		setawbt(gspca_dev);
		break;
	case V4L2_CID_AE_WINDOW:
	case V4L2_CID_AE_START_X:
	case V4L2_CID_AE_START_Y:
	case V4L2_CID_AE_END_X:
	case V4L2_CID_AE_END_Y:
		setaewin(gspca_dev);
		break;
	case V4L2_CID_TEST_PATTERN:
		settest(gspca_dev);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		setfreq(gspca_dev);
		apply_fps(gspca_dev);	/* o ajuste fino de taxa usa o mesmo reg 0x2B */
		KDBG2(gspca_dev->v4l2_dev.dev, "sd_s_ctrl - setfreq"," - %d",elgcnt)
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
static int sd_init_controls(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct v4l2_ctrl_handler *hdl = &gspca_dev->ctrl_handler;

    KDBG(gspca_dev->v4l2_dev.dev, "sd_init_controls"," - %d",elgcnt)

	gspca_dev->vdev.ctrl_handler = hdl;
	v4l2_ctrl_handler_init(hdl, 8);

	// josemar - Definition ****
	/// struct v4l2_ctrl * v4l2_ctrl_new_std(struct v4l2_ctrl_handler * hdl, const struct v4l2_ctrl_ops * ops, u32 id, s64 min, s64 max, u64 step, s64 def)
	// ***************

	if (sd->sensor == SENSOR_OV6650 || sd->sensor == SENSOR_OV7630 || sd->sensor == SENSOR_PAS106 || sd->sensor == SENSOR_PAS202)
		sd->brightness = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 255, 1, 127);
	if (sd->sensor == SENSOR_OV7630) {
		/* alvo do ganho automatico, separado do brilho (lido em do_autogain; nao grava registrador) */
		static const struct v4l2_ctrl_config aet_cfg = {
			.ops = &sd_ctrl_ops, .id = V4L2_CID_AE_TARGET, .name = "AE target",
			.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 255, .step = 1, .def = 127,
		};
		sd->ae_target = v4l2_ctrl_new_custom(hdl, &aet_cfg, NULL);
	}

	/* Gain range is sensor dependent */
	switch (sd->sensor) {

	case SENSOR_OV7630:
		gspca_dev->gain = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAIN, 0, 47, 1, 31);
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

	case SENSOR_OV7630:
		gspca_dev->exposure = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_EXPOSURE, 0, 1023, 1, 66);
		sd->exposure_knee = 200;
		break;


	}

	/* controle de ganho automatico (V4L2_CID_AUTOGAIN) removido em 2026-10-17 a pedido do Josemar: ganho e
	 * exposicao ficam sempre manuais. Sem ele (gspca_dev->autogain = NULL), do_autogain so roda a trava de
	 * banda e o contraste adaptativo. */

	if (sd->sensor == SENSOR_OV6650 || sd->sensor == SENSOR_OV7630)
		sd->plfreq = v4l2_ctrl_new_std_menu(hdl, &sd_ctrl_ops, V4L2_CID_POWER_LINE_FREQUENCY, V4L2_CID_POWER_LINE_FREQUENCY_60HZ, 0, V4L2_CID_POWER_LINE_FREQUENCY_DISABLED);

	/* balanco de branco manual (ganhos de cor do proprio sensor) */
	/* "Auto Exposure, Dynamic Framerate": deixa o driver baixar a taxa quando o canal USB satura (S910) */
	sd->dynfps = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0, 1, 1, 1);
	if (sd->sensor == SENSOR_OV7630) {	/* ganhos de cor do sensor; padrao = init do Windows */
		/* contraste pela curva da ponte (128 = sem mudanca); a saturacao do sensor (reg 0x03) saiu:
		 * so age em YUV, e na imagem crua a saturacao so e' possivel na aplicacao */
		sd->contrast = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_CONTRAST, 0, 255, 1, 128);
		sd->awb = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
		{
			static const struct v4l2_ctrl_config rg_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_RED_GAIN_BR, .name = "Red gain",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 127, .step = 1, .def = 0x20,
			};
			static const struct v4l2_ctrl_config gg_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_GREEN_GAIN_BR, .name = "Green gain",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 127, .step = 1, .def = 0x20,
			};
			static const struct v4l2_ctrl_config bg_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_BLUE_GAIN_BR, .name = "Blue gain",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 127, .step = 1, .def = 0x20,
			};

			sd->rgain = v4l2_ctrl_new_custom(hdl, &rg_cfg, NULL);
			sd->ggain = v4l2_ctrl_new_custom(hdl, &gg_cfg, NULL);
			sd->bgain = v4l2_ctrl_new_custom(hdl, &bg_cfg, NULL);
		}
	}
	if (sd->sensor == SENSOR_OV7630) {
		sd->black = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_BLACK_LEVEL, 0, 255, 1, 0x0a);
		sd->gamma = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_GAMMA, 0, 8, 1, 2);
		sd->hflip = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
		sd->vflip = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
		sd->bandf = v4l2_ctrl_new_std(hdl, &sd_ctrl_ops, V4L2_CID_BAND_STOP_FILTER, 0, 1, 1, 0);
		{
			static const struct v4l2_ctrl_config bg_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AWB_BG, .name = "AWB Target Blue",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 63, .step = 1, .def = 0x20,
			};
			static const struct v4l2_ctrl_config rg_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AWB_RG, .name = "AWB Target Red",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 63, .step = 1, .def = 0x20,
			};

			sd->abg_blue = v4l2_ctrl_new_custom(hdl, &bg_cfg, NULL);
			sd->abg_red = v4l2_ctrl_new_custom(hdl, &rg_cfg, NULL);
		}
		{
			static const struct v4l2_ctrl_config aw_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AE_WINDOW, .name = "AE window",
				.type = V4L2_CTRL_TYPE_MENU, .min = 0, .max = ARRAY_SIZE(ae_win_menu) - 1, .def = 0,
				.qmenu = ae_win_menu,
			};
			static const struct v4l2_ctrl_config sx_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AE_START_X, .name = "AE custom start X",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 7, .step = 1, .def = 1,
			};
			static const struct v4l2_ctrl_config sy_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AE_START_Y, .name = "AE custom start Y",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 7, .step = 1, .def = 2,
			};
			static const struct v4l2_ctrl_config ex_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AE_END_X, .name = "AE custom end X",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 31, .step = 1, .def = 7,
			};
			static const struct v4l2_ctrl_config ey_cfg = {
				.ops = &sd_ctrl_ops, .id = V4L2_CID_AE_END_Y, .name = "AE custom end Y",
				.type = V4L2_CTRL_TYPE_INTEGER, .min = 0, .max = 15, .step = 1, .def = 6,
			};

			sd->ae_win = v4l2_ctrl_new_custom(hdl, &aw_cfg, NULL);
			sd->ae_sx = v4l2_ctrl_new_custom(hdl, &sx_cfg, NULL);
			sd->ae_sy = v4l2_ctrl_new_custom(hdl, &sy_cfg, NULL);
			sd->ae_ex = v4l2_ctrl_new_custom(hdl, &ex_cfg, NULL);
			sd->ae_ey = v4l2_ctrl_new_custom(hdl, &ey_cfg, NULL);
			v4l2_ctrl_activate(sd->ae_sx, false);	/* menu comeca em "Mode default" */
			v4l2_ctrl_activate(sd->ae_sy, false);
			v4l2_ctrl_activate(sd->ae_ex, false);
			v4l2_ctrl_activate(sd->ae_ey, false);
		}
		sd->testpat = v4l2_ctrl_new_std_menu_items(hdl, &sd_ctrl_ops, V4L2_CID_TEST_PATTERN, ARRAY_SIZE(test_menu) - 1, 0, 0, test_menu);
	}
	if (hdl->error) {
		pr_err("Could not initialize controls\n");
		return hdl->error;
	}

	if (gspca_dev->autogain)		/* so se o controle automatico voltar a existir */
		v4l2_ctrl_auto_cluster(3, &gspca_dev->autogain, 0, false);

	return 0;
}


/* -- start the camera -- */
static int sd_start(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam = &gspca_dev->cam;
	int mode;
	__u8 regs[0x31]; // josemar: Em sn9n102.pdf tem a descricao de 0x1Fh registradores. Sugere que regs pertence ao controlador sn9c102/103
	const __u8 i2c[] = { 0xa0, 0x21, 0x13, 0x80, 0x00, 0x00, 0x00, 0x10 };

	KDBG(gspca_dev->v4l2_dev.dev, "sd_start"," - %d",elgcnt)

	mode = cam->cam_mode[gspca_dev->curr_mode].priv & 0x07;
	sd->hi = req_fps(gspca_dev) > 30;		/* modo de 60 fps (so existe na tabela do S910 160x120) */
	/* Copy registers 0x01 - 0x19 from the template */
	memcpy(&regs[0x01], sensor_data[sd->sensor].bridge_init, 0x19);



if (stock_regs) {   /* valores "originais" (antigo #if(0)) */

    /* Special cases where some regs depend on mode or bridge */
	/* FIXME / TESTME for some reason with the 101/102 bridge the
	   clock is set to 12 Mhz (reg1 == 0x04), rather then 24.
	   Also the hstart needs to go from 1 to 2 when using a 103,
	   which is likely related. This does not seem right. */
	/// ***** VIR *****
	regs[0x01] = 0x44; /* Select 24 Mhz clock */
	/// ***** VIR *****

	regs[0x02] = 0x44; // r02

	regs[0x03] = 0x00; // r03
	regs[0x04] = 0x00; // r04

	/* Set bridge gain to 1.0 */
	regs[0x05] = 0x20;  // r05 Red
	regs[0x06] = 0x20;  // r06 Green
	regs[0x07] = 0x20;  // r07 Blue

	regs[0x08] = 0x80; // r08
	regs[0x09] = 0x21; // r09
	regs[0x0a] = 0x00; // r0a
	regs[0x0b] = 0x00; // r0b
	regs[0x0c] = 0x00; // r0c
	regs[0x0d] = 0x00; // r0d
	regs[0x0e] = 0x00; // r0e
	regs[0x0f] = 0x00; // r0f

    regs[0x10] = 0x00; // r10
	regs[0x11] = 0x00; // r11

    regs[0x12] = 0x02; // Set hstart to 2 - r12 // em 640x480 e com vlaor 5 ainda aparece um linha escura vetical no lado direito; com 4 ainda aparece resquicios da interpolação
    regs[0x13] = 0x00; // r13
    regs[0x14] = 0x00; // r14  - offset = 0; Quanto maior o valor mais escuro a imagem. Se fica mais claro, a imagem parece ser mais borrada
    regs[0x15] = 0x28; // r15  - deve ser 640 / 16
    regs[0x16] = 0x1e; // r16  - deve ser 480 / 16

    /* Vga mode emulation on SIF sensor? */
	if (0/*cam->cam_mode[gspca_dev->curr_mode].priv & MODE_REDUCED_SIF*/) {
		regs[0x12] += 16;	/* hstart adjust */
		regs[0x13] += 24;	/* vstart adjust */
		regs[0x15]  = 320 / 16; /* hsize */
		regs[0x16]  = 240 / 16; /* vsize */
	}



	/// ***** VIR *****
    regs[0x17] = 0x6c; // r17           aqui controla o frame rate  48MHz, 24MHz, 12MHz
    /// ***** VIR *****


	/* Set the mode */
	/* Disable compression when the raw bayer format has been selected */
	/// ***** VIR *****
	regs[0x18] = (0x8f | (mode << 4)) & ((cam->cam_mode[gspca_dev->curr_mode].priv & MODE_RAW)?0x7f:0xff);
	/// ***** VIR *****

	//KDBG(gspca_dev->v4l2_dev.dev, "regs[0x18] = ","0x%02x - %d",regs[0x18], elgcnt)

    /// ***** VIR *****
	regs[0x19] = 0x2b; // r19      // aqui tem 2x;  aqui controla o frame rate quando utilizando a freq do sistema
    /// ***** VIR *****



} else {            /* valores capturados do Windows */
    	/// ***** VIR *****
	regs[0x01] = 0x44; /* Select 24 Mhz clock */
	/// ***** VIR *****

	regs[0x02] = 0x44; // r02

	regs[0x03] = 0x00; // r03
	regs[0x04] = 0x1A; // r04

	/* Set bridge gain to 1.0 */
	regs[0x05] = 0x20;  // r05 Red
	regs[0x06] = 0x20;  // r06 Green
	regs[0x07] = 0x20;  // r07 Blue

	regs[0x08] = 0x80; // r08
	regs[0x09] = 0x21; // r09
	regs[0x0a] = 0x00; // r0a
	regs[0x0b] = 0x00; // r0b
	regs[0x0c] = 0x00; // r0c
	regs[0x0d] = 0x00; // r0d
	regs[0x0e] = 0x00; // r0e
	regs[0x0f] = 0x00; // r0f

    regs[0x10] = 0x03; // r10
	regs[0x11] = 0x00; // r11

    regs[0x12] = 0x02; // hstart 2 (par) com vstart 1 (impar): mosaico BGGR como o BA81 anuncia (com hstart 1 saia GBRG, medido com cor solida) - r12 // em 640x480 e com vlaor 5 ainda aparece um linha escura vetical no lado direito; com 4 ainda aparece resquicios da interpolação
    regs[0x13] = 0x01; // r13
    regs[0x14] = 0x0A; // r14  - offset = 0; Quanto maior o valor mais escuro a imagem. Se fica mais claro, a imagem parece ser mais borrada
    regs[0x15] = 0x28; // r15  - janela 640 / 16 (o modo menor é reduzido pelo bridge via reg 0x18). Era 0x16 (352 de largura): imagem deformada
    regs[0x16] = 0x1e; // r16  - janela 480 / 16. Era 0x12 (288 de altura)

    /* Vga mode emulation on SIF sensor? */
	if (0/*cam->cam_mode[gspca_dev->curr_mode].priv & MODE_REDUCED_SIF*/) {
		regs[0x12] += 16;	/* hstart adjust */
		regs[0x13] += 24;	/* vstart adjust */
		regs[0x15]  = 320 / 16; /* hsize */
		regs[0x16]  = 240 / 16; /* vsize */
	}



	/// ***** VIR *****
    regs[0x17] = 0x68; // r17           aqui controla o frame rate  48MHz, 24MHz, 12MHz
    /// ***** VIR *****


	/* Set the mode */
	/* Disable compression when the raw bayer format has been selected */
	/// ***** VIR *****
	regs[0x18] = (0x8f | (mode << 4)) & ((cam->cam_mode[gspca_dev->curr_mode].priv & MODE_RAW)?0x7f:0xff);
	/// ***** VIR *****

	//KDBG(gspca_dev->v4l2_dev.dev, "regs[0x18] = ","0x%02x - %d",regs[0x18], elgcnt)

    /// ***** VIR *****
	regs[0x19] = 0x20; // r19      // aqui tem 2x;  aqui controla o frame rate quando utilizando a freq do sistema
    /// ***** VIR *****


}




	/* Setup pixel numbers and auto exposure window */
	if (1 /*sensor_data[sd->sensor].flags & F_SIF*/) {
		regs[0x1a] = 0x14; /* HO_SIZE 640, makes no sense */        // NÃO ALTEROU EM NADA
		regs[0x1b] = 0x0f; /* VO_SIZE 320, makes no sense */        // NÃO ALTEROU EM NADA
		regs[0x1c] = 0x02; /* AE H-start: 2 x 32 = 64 px */                      // ÁREA PARA CONTROLE AUTOMATICO DE GANHO OU BRILHO
		regs[0x1d] = 0x03; /* AE V-start: 3 x 32 = 96 linhas */                      // ÁREA PARA CONTROLE AUTOMATICO DE GANHO OU BRILHO
		regs[0x1e] = 0x0f; /* AE H-end: 15 x 32 = 480 px */                       // ÁREA PARA CONTROLE AUTOMATICO DE GANHO OU BRILHO
		regs[0x1f] = 0x0c; /* AE V-end: 12 x 32 = 384 linhas */                       // ÁREA PARA CONTROLE AUTOMATICO DE GANHO OU BRILHO
	} else {
		regs[0x1a] = 0x1d; /* HO_SIZE 960, makes no sense */
		regs[0x1b] = 0x10; /* VO_SIZE 512, makes no sense */
		regs[0x1c] = 0x02; /* AE H-start 160 */
		regs[0x1d] = 0x03; /* AE V-start 96 */
		regs[0x1e] = 0x0f; /* AE H-end 480 */
		regs[0x1f] = 0x0c; /* AE V-end 384 */
	}

	/* Modo de 60 fps (S910 160x120): a ponte recebe uma janela de 320x240 (o sensor vai a QVGA, ver
	 * abaixo) reduzida a 1/2; a janela de AE acompanha. Medido em testes/testa_60fps.py. */
	sd->ae_area = (regs[0x1e] - regs[0x1c]) * (regs[0x1f] - regs[0x1d]);
	if (sd->hi || (cam->cam_mode[gspca_dev->curr_mode].priv & MODE_QVGA)) {
		regs[0x15] = 320 / 16;
		regs[0x16] = 240 / 16;
		if (!(cam->cam_mode[gspca_dev->curr_mode].priv & MODE_QVGA))
			regs[0x18] = (regs[0x18] & ~0x70) | (1 << 4);          /* escala 1/2 (80x60 mantem a escala 1/4 do modo) */
		regs[0x1c] = 0x01;
		regs[0x1d] = 0x02;
		regs[0x1e] = 0x07;
		regs[0x1f] = 0x06;
		sd->ae_area = (regs[0x1e] - regs[0x1c]) * (regs[0x1f] - regs[0x1d]);
	}

	/* CIF/QCIF: janela 352x288 centrada no quadro VGA do sensor (offset 144 px, 96 linhas; a paridade de
	 * hstart/vstart mantem o mosaico BGGR). A escala vem do modo (CIF 1/1, QCIF 1/2). */
	if (cam->cam_mode[gspca_dev->curr_mode].priv & MODE_CIF) {
		regs[0x12] += 144;
		regs[0x13] += 96;
		regs[0x15] = 352 / 16;
		regs[0x16] = 288 / 16;
		regs[0x1c] = 0x02;
		regs[0x1d] = 0x02;
		regs[0x1e] = 0x09;
		regs[0x1f] = 0x07;
		sd->ae_area = (regs[0x1e] - regs[0x1c]) * (regs[0x1f] - regs[0x1d]);
	}

	/* fase do mosaico Bayer pedida pelo formato (BA81 = BGGR e a base) */
	switch (gspca_dev->pixfmt.pixelformat) {
	case V4L2_PIX_FMT_SGBRG8:
		regs[0x12]++;
		break;
	case V4L2_PIX_FMT_SGRBG8:
		regs[0x13]++;
		break;
	case V4L2_PIX_FMT_SRGGB8:
		regs[0x12]++;
		regs[0x13]++;
		break;
	}

	/* H_START base e correcao do espelho (a fase do mosaico Bayer muda com ele) */
	sd->hstart = regs[0x12];
	sd->vstart = regs[0x13];

	/* janela de AE: a padrao do modo (calculada acima) e a escolhida no controle "AE window" */
	memcpy(sd->ae_def, &regs[0x1c], 4);
	sd->ae_w = regs[0x15] / 2;
	sd->ae_h = regs[0x16] / 2;
	if (sd->ae_win) {
		ae_window(sd, &regs[0x1c]);
		ae_set_area(sd, &regs[0x1c]);
	}
	if (sd->vflip && sd->vflip->val)
		regs[0x13] += VFLIP_VSTART_ADJ;
	if (sd->hflip && sd->hflip->val)
		regs[0x12] += HFLIP_HSTART_ADJ;
	if (sd->black)
		regs[0x14] = sd->black->val;

	/* Setup the gamma table (only used with the sn9c103 bridge) */
	curva(sd, &regs[0x20]);		/* gama + contraste */









	/* reg 0x01 bit 2 video transfert on */
//	reg_w(gspca_dev, 0x01, &regs[0x01], 1);             // [0x01] = 0x44
	/* reg 0x17 SensorClk enable inv Clk 0x60 */
//	reg_w(gspca_dev, 0x17, &regs[0x17], 1);
	/* Set the registers from the template */
	/* Overrides de teste (parâmetros do módulo; -1 = não altera) */
	if (reg17 >= 0)
		regs[0x17] = reg17;
	if (reg19 >= 0)
		regs[0x19] = reg19;
	if (lq_sel >= 0)
		regs[0x17] = (regs[0x17] & 0xfe) | (lq_sel & 1);

	sd->reg17 = regs[0x17];
	if (sd->testpat && sd->testpat->val == 1)
		regs[0x17] |= 0x10;		/* TEST_IMG: imagem de teste da ponte */
	reg_w(gspca_dev, 0x01, &regs[0x01], 0x30); // josemar tamanho da tabela de registradores: 0x1F = SN9C102 e 0x30 = SN9C103





/// *********************************************************************************
/// **************************** SET REGISTERS OF SENSOR ****************************
/// *********************************************************************************




	/* Init the sensor */
	{
		sensor_init_t *sinit = sensor_data[sd->sensor].sensor_init;
		int ssize = sensor_data[sd->sensor].sensor_init_size;

		if (stock_sensor) {
			sinit = ov7630_sensor_init_stock;
			ssize = ARRAY_SIZE(ov7630_sensor_init_stock);
		}
		i2c_w_vector(gspca_dev, sinit, ssize);
	}

	/* Mode / bridge specific sensor setup */
	/* FIXME / TESTME We should be able to handle this identical
	   for the 101/102 and the 103 case */
	///const __u8 i2c[] = { 0xa0, 0x21, 0x13, 0x80, 0x00, 0x00, 0x00, 0x10 };
	i2c_w(gspca_dev, i2c);

	/* Modo de 60 fps: sensor em QVGA (COMC bit 5), saida de uma linha ENTRELACADA (COMH = 0x80; o
	 * Windows usa 0xA0, progressivo, a 30 fps) e so o campo impar (COMJ bit 6), ate 60 f/s */
	if (!sd->hi && (cam->cam_mode[gspca_dev->curr_mode].priv & MODE_QVGA)) {
		/* 80x60 a 30 fps ou menos: sensor em QVGA (COMC bit 5) e progressivo (COMH = 0xA0, como o Windows) */
		static const __u8 q_regs[][8] = {
			{ 0xa0, 0x21, 0x14, 0xa0, 0x00, 0x00, 0x00, 0x10 },
			{ 0xa0, 0x21, 0x28, 0xa0, 0x00, 0x00, 0x00, 0x10 },
		};

		i2c_w_vector(gspca_dev, q_regs, sizeof(q_regs));
	}
	if (sd->hi) {
		static const __u8 hi_regs[][8] = {
			{ 0xa0, 0x21, 0x14, 0xa0, 0x00, 0x00, 0x00, 0x10 },
			{ 0xa0, 0x21, 0x28, 0x80, 0x00, 0x00, 0x00, 0x10 },
			{ 0xa0, 0x21, 0x2d, 0xc1, 0x00, 0x00, 0x00, 0x10 },
		};

		i2c_w_vector(gspca_dev, hi_regs, sizeof(hi_regs));	/* i2c_w_vector conta BYTES, nao linhas */
	}







/// *********************************************************************************
/// ************************** SET REGISTERS OF CONTROLLER **************************
/// *********************************************************************************


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

	KDBG(gspca_dev->v4l2_dev.dev, "sd_start opts"," stock_regs=%d stock_sensor=%d sensor_reg11=%d r17=0x%02x r19=0x%02x - %d", stock_regs, stock_sensor, sensor_reg11, regs[0x17], regs[0x19], elgcnt)

	sd->reg11 = -1;

	/* banda do alt em uso, para a trava de banda; zera o estado dela */
	sd->size_peak = 0;
	sd->guard_hi = 0;
	sd->guard_lo = 0;
	atomic_set(&sd->frame_len, 0);
	sd->link_bw = 0;
	{
		struct usb_interface *intf = usb_ifnum_to_if(gspca_dev->dev, gspca_dev->iface);
		struct usb_host_endpoint *ep = intf ?
			alt_xfer(&intf->altsetting[gspca_dev->alt], USB_ENDPOINT_XFER_ISOC, gspca_dev->xfer_ep) : NULL;

		if (ep)
			sd->link_bw = (le16_to_cpu(ep->desc.wMaxPacketSize) & 0x7ff) * 1000;
	}


	setgain(gspca_dev);             // configura ganho pelo sensor          r00
	setbrightness(gspca_dev);       // configura brilho pelo sensor         r06
	setwb(gspca_dev);               // balanco de branco automatico do sensor (COMA bit 2)
	setgains(gspca_dev);            // ganhos R, G e B da ponte (r05, r06, r07)
	if (sd->hflip && (sd->hflip->val || sd->testpat->val == 2))
		setcoma(gspca_dev);         // espelho e/ou barras de cor: COMA do sensor (r12)
	setawbt(gspca_dev);             // alvos do AWB (r0c e r0d)
	setbandf(gspca_dev);            // filtro de banda (r2d)
	if (sd->vflip && sd->vflip->val)
		setvflip(gspca_dev);        // inversao vertical (r75)
	setexposure(gspca_dev);         // configura exposicao pelo sensor      r10 e r11
	setfreq(gspca_dev);             // configura freq pelo sensor           r2b
	sd->fps_logged = 0;             // cada captura mostra a taxa uma vez no log
	apply_fps(gspca_dev);           // reg 0x11 final: taxa da tabela de fps (ou sensor_reg11)

	if (regs_start[0]) {                // itens TIPO:REG=VALOR escolhidos pelo usuario
		int err0 = gspca_dev->usb_err;

		reg_list_apply(gspca_dev, regs_start);
		if (!err0 && gspca_dev->usb_err) {
			dev_err(gspca_dev->v4l2_dev.dev, "regs_start: erro %d (ignorado)\n", gspca_dev->usb_err);
			gspca_dev->usb_err = 0;
		}
	}

	sd->frames_to_drop = 0;
	sd->autogain_ignore_frames = 0;
	gspca_dev->exp_too_high_cnt = 0;
	gspca_dev->exp_too_low_cnt = 0;
	atomic_set(&sd->avg_lum, -1);
	return gspca_dev->usb_err;



	// ATENCAO: AS FUNÇÕES QUE FORAM IDENTIFICADAS COMO CAPAZES DE FAZEREM ALTERAÇÕES NOS REGISTRADOS
	// ESTÃO LOCALIZADAS EM PONTOS ESPECÍFICOS:
	// FUNÇÕES: setgain, setexposure, setbrightness, setfreq CHAMADAS DE DENTRO DAS FUNÇÕES sd_s_ctrl e sd_start
	// FUNÇÕES: gspca_expo_autogain e gspca_coarse_grained_expo_autogain CHAMDAS A PARTIR DE do_autogain

}






static void sd_stopN(struct gspca_dev *gspca_dev)
{
    KDBG(gspca_dev->v4l2_dev.dev, "sd_stopN"," - %d",elgcnt)

	sd_init(gspca_dev);
}

static u8* find_sof(struct gspca_dev *gspca_dev, u8 *data, int len)
{
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
		atomic_set(&sd->frame_len, gspca_dev->image_len);

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
	.get_streamparm = sd_get_streamparm,
	.set_streamparm = sd_set_streamparm,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.set_register = sd_dbg_s_register,
	.get_register = sd_dbg_g_register,
	.get_chip_info = sd_chip_info,
#endif
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
static int sd_probe(struct usb_interface *intf,	const struct usb_device_id *id)
{
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




//module_param_named(debug, gspca_debug, int, 0644);
//MODULE_PARM_DESC(debug, "1:probe 2:config 3:stream 4:frame 5:packet 6:usbi 7:usbo");



// ATENÇÃO:     ESTA OPÇÃO "module_usb_driver(sd_driver)" PERMITIU O LINK ENTRE OUTROS MÓDULOS DE DRIVER AUTOMATICAMENTE
/**************************/
module_usb_driver(sd_driver);
/**************************/




/*     OBSERVAÇOES   */
/*

/// *************************************************************************************************************
/// Quando a camera é conectada ao PC;

[ 6011.498336] gspca_elgin_sn9c103-2.14.0 probing 0c45:608f
[ 6011.498342] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_config - 0
[ 6011.498345] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 1
[ 6011.498464] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 2
[ 6011.498467] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 3
[ 6011.498582] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init_controls - 4
[ 6011.498592] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 5
[ 6011.498595] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 6
[ 6011.498596] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 7
[ 6011.498653] input: gspca_elgin_sn9c103 as /devices/pci0000:00/0000:00:1d.0/usb2/2-1/2-1.7/input/input28
[ 6011.498794] usbcore: registered new interface driver gspca_elgin_sn9c103

/// *************************************************************************************************************
/// Quando a o Software é iniciado e acessa a camera:

[ 7554.369643] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: gspca_start_streaming - 8
[ 7554.369650] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: gspca_init_transfer - 9
[ 7554.369656] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 128000 - 10
[ 7554.369660] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 256000 - 11
[ 7554.369664] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 384000 - 12
[ 7554.369668] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 512000 - 13
[ 7554.369672] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 680000 - 14
[ 7554.369675] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 800000 - 15
[ 7554.369679] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 900000 - 16
[ 7554.369683] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: bandwidth found: 1003000 - 17
[ 7554.369687] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: dev has usb audio, skipping highest alt - 18
[ 7554.372284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_start - 19
[ 7554.372292] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 20
[ 7554.372534] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 21
[ 7554.372654] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 22
[ 7554.372775] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w_vector - 23
[ 7554.372782] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 24
[ 7554.372785] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 25
[ 7554.382144] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 26
[ 7554.382282] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 27
[ 7554.382284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 28
[ 7554.394114] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 29
[ 7554.394283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 30
[ 7554.394285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 31
[ 7554.406115] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 32
[ 7554.406283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 33
[ 7554.406285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 34
[ 7554.418116] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 35
[ 7554.418284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 36
[ 7554.418286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 37
[ 7554.430116] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 38
[ 7554.430283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 39
[ 7554.430285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 40
[ 7554.442115] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 41
[ 7554.442283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 42
[ 7554.442285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 43
[ 7554.454117] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 44
[ 7554.454283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 45
[ 7554.454285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 46
[ 7554.466116] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 47
[ 7554.466284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 48
[ 7554.466286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 49
[ 7554.478118] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 50
[ 7554.478282] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 51
[ 7554.478285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 52
[ 7554.490118] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 53
[ 7554.490284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 54
[ 7554.490286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 55
[ 7554.502119] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 56
[ 7554.502283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 57
[ 7554.502285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 58
[ 7554.514122] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 59
[ 7554.514285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 60
[ 7554.514287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 61
[ 7554.526121] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 62
[ 7554.526283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 63
[ 7554.526285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 64
[ 7554.538121] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 65
[ 7554.538285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 66
[ 7554.538287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 67
[ 7554.550121] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 68
[ 7554.550284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 69
[ 7554.550286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 70
[ 7554.562118] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 71
[ 7554.562285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 72
[ 7554.562287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 73
[ 7554.574124] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 74
[ 7554.574283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 75
[ 7554.574285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 76
[ 7554.586121] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 77
[ 7554.586283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 78
[ 7554.586285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 79
[ 7554.598122] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 80
[ 7554.598286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 81
[ 7554.598290] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 82
[ 7554.610119] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 83
[ 7554.610283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 84
[ 7554.610286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 85
[ 7554.622122] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 86
[ 7554.622283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 87
[ 7554.622285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 88
[ 7554.634122] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 89
[ 7554.634285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 90
[ 7554.634287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 91
[ 7554.646123] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 92
[ 7554.646284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 93
[ 7554.646534] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 94
[ 7554.646658] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 95
[ 7554.646784] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 96
[ 7554.646908] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 97
[ 7554.647034] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 98
[ 7554.647158] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 99
[ 7554.647260] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 100
[ 7554.647382] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 101
[ 7554.674126] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setgain - 102
[ 7554.674128] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 103
[ 7554.674129] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 104
[ 7554.686123] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 105
[ 7554.686285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setbrightness - 106
[ 7554.686287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 107
[ 7554.686288] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 108
[ 7554.698121] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 109
[ 7554.698263] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setexposure - 110
[ 7554.698265] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 111
[ 7554.698267] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 112
[ 7554.710123] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 113
[ 7554.710285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setfreq - 114
[ 7554.710287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 115
[ 7554.710288] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 116
[ 7554.722124] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 117
[ 7554.722285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 118
[ 7554.722287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 119
[ 7554.722288] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 120
[ 7554.722296] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_stopN - 121
[ 7554.722297] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 122
[ 7554.722298] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 123
[ 7554.722719] gspca_elgin_sn9c103 2-1.7:1.0: alt 7 - bandwidth not wide enough, trying again
[ 7554.752462] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_start - 124
[ 7554.752464] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 125
[ 7554.752659] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 126
[ 7554.752784] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 127
[ 7554.752908] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w_vector - 128
[ 7554.752910] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 129
[ 7554.752911] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 130
[ 7554.762125] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 131
[ 7554.762283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 132
[ 7554.762285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 133
[ 7554.774126] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 134
[ 7554.774284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 135
[ 7554.774286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 136
[ 7554.786126] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 137
[ 7554.786284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 138
[ 7554.786286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 139
[ 7554.798128] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 140
[ 7554.798285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 141
[ 7554.798287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 142
[ 7554.810127] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 143
[ 7554.810283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 144
[ 7554.810284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 145
[ 7554.822134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 146
[ 7554.822284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 147
[ 7554.822286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 148
[ 7554.834126] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 149
[ 7554.834283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 150
[ 7554.834285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 151
[ 7554.846134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 152
[ 7554.846285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 153
[ 7554.846287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 154
[ 7554.858127] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 155
[ 7554.858283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 156
[ 7554.858285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 157
[ 7554.870129] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 158
[ 7554.870284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 159
[ 7554.870286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 160
[ 7554.882135] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 161
[ 7554.882283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 162
[ 7554.882285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 163
[ 7554.894129] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 164
[ 7554.894284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 165
[ 7554.894286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 166
[ 7554.906137] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 167
[ 7554.906283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 168
[ 7554.906285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 169
[ 7554.918130] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 170
[ 7554.918284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 171
[ 7554.918286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 172
[ 7554.930129] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 173
[ 7554.930283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 174
[ 7554.930285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 175
[ 7554.942131] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 176
[ 7554.942284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 177
[ 7554.942286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 178
[ 7554.954140] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 179
[ 7554.954283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 180
[ 7554.954285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 181
[ 7554.966129] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 182
[ 7554.966284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 183
[ 7554.966286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 184
[ 7554.978130] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 185
[ 7554.978283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 186
[ 7554.978285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 187
[ 7554.990130] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 188
[ 7554.990284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 189
[ 7554.990286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 190
[ 7555.002127] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 191
[ 7555.002283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 192
[ 7555.002285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 193
[ 7555.014130] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 194
[ 7555.014284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 195
[ 7555.014286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 196
[ 7555.026138] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 197
[ 7555.026283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 198
[ 7555.026533] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 199
[ 7555.026657] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 200
[ 7555.026783] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 201
[ 7555.026908] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 202
[ 7555.027032] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 203
[ 7555.027171] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 204
[ 7555.027258] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 205
[ 7555.027382] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 206
[ 7555.054132] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setgain - 207
[ 7555.054134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 208
[ 7555.054135] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 209
[ 7555.066131] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 210
[ 7555.066284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setbrightness - 211
[ 7555.066286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 212
[ 7555.066287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 213
[ 7555.078134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 214
[ 7555.078283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setexposure - 215
[ 7555.078286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 216
[ 7555.078287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 217
[ 7555.090134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 218
[ 7555.090285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setfreq - 219
[ 7555.090287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 220
[ 7555.090288] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 221
[ 7555.102137] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 222
[ 7555.102268] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 223
[ 7555.102272] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 224
[ 7555.102275] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 225
[ 7555.102287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_stopN - 226
[ 7555.102289] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 227
[ 7555.102291] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 228
[ 7555.102736] gspca_elgin_sn9c103 2-1.7:1.0: alt 6 - bandwidth not wide enough, trying again
[ 7555.131590] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_start - 229
[ 7555.131592] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 230
[ 7555.131784] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 231
[ 7555.131909] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 232
[ 7555.132035] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w_vector - 233
[ 7555.132037] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 234
[ 7555.132038] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 235
[ 7555.142134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 236
[ 7555.142284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 237
[ 7555.142286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 238
[ 7555.154133] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 239
[ 7555.154285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 240
[ 7555.154287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 241
[ 7555.166131] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 242
[ 7555.166285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 243
[ 7555.166287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 244
[ 7555.178134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 245
[ 7555.178285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 246
[ 7555.178287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 247
[ 7555.190134] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 248
[ 7555.190284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 249
[ 7555.190286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 250
[ 7555.202135] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 251
[ 7555.202285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 252
[ 7555.202287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 253
[ 7555.214135] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 254
[ 7555.214284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 255
[ 7555.214286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 256
[ 7555.226135] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 257
[ 7555.226284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 258
[ 7555.226286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 259
[ 7555.238135] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 260
[ 7555.238284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 261
[ 7555.238286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 262
[ 7555.250136] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 263
[ 7555.250284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 264
[ 7555.250286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 265
[ 7555.262136] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 266
[ 7555.262284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 267
[ 7555.262286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 268
[ 7555.274136] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 269
[ 7555.274285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 270
[ 7555.274287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 271
[ 7555.286136] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 272
[ 7555.286284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 273
[ 7555.286286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 274
[ 7555.298137] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 275
[ 7555.298284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 276
[ 7555.298286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 277
[ 7555.310137] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 278
[ 7555.310284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 279
[ 7555.310286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 280
[ 7555.322137] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 281
[ 7555.322284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 282
[ 7555.322286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 283
[ 7555.334140] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 284
[ 7555.334283] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 285
[ 7555.334285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 286
[ 7555.346137] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 287
[ 7555.346285] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 288
[ 7555.346287] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 289
[ 7555.358138] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 290
[ 7555.358284] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 291
[ 7555.358286] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 292
[ 7555.370172] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 293
[ 7555.370448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 294
[ 7555.370453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 295
[ 7555.382171] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 296
[ 7555.382342] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 297
[ 7555.382347] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 298
[ 7555.394181] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 299
[ 7555.394458] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 300
[ 7555.394462] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 301
[ 7555.406173] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 302
[ 7555.406323] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 303
[ 7555.406572] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 304
[ 7555.406822] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 305
[ 7555.407070] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 306
[ 7555.407322] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 307
[ 7555.407574] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 308
[ 7555.407821] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 309
[ 7555.408017] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 310
[ 7555.408139] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 311
[ 7555.434174] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setgain - 312
[ 7555.434179] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 313
[ 7555.434181] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 314
[ 7555.446173] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 315
[ 7555.446449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setbrightness - 316
[ 7555.446454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 317
[ 7555.446456] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 318
[ 7555.458175] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 319
[ 7555.458323] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setexposure - 320
[ 7555.458328] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 321
[ 7555.458330] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 322
[ 7555.470177] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 323
[ 7555.470449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setfreq - 324
[ 7555.470453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 325
[ 7555.470456] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 326
[ 7555.482174] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 327
[ 7555.482274] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 328
[ 7555.482278] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 329
[ 7555.482280] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 330
[ 7555.482294] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_stopN - 331
[ 7555.482297] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 332
[ 7555.482299] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 333
[ 7555.482839] gspca_elgin_sn9c103 2-1.7:1.0: alt 5 - bandwidth not wide enough, trying again
[ 7555.512088] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_start - 334
[ 7555.512093] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 335
[ 7555.512323] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 336
[ 7555.512573] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 337
[ 7555.512823] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w_vector - 338
[ 7555.512827] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 339
[ 7555.512829] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 340
[ 7555.522176] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 341
[ 7555.522320] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 342
[ 7555.522324] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 343
[ 7555.538206] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 344
[ 7555.538480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 345
[ 7555.538484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 346
[ 7555.550198] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 347
[ 7555.550479] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 348
[ 7555.550484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 349
[ 7555.562180] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 350
[ 7555.562480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 351
[ 7555.562484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 352
[ 7555.574197] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 353
[ 7555.574478] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 354
[ 7555.574482] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 355
[ 7555.586182] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 356
[ 7555.586480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 357
[ 7555.586484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 358
[ 7555.598180] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 359
[ 7555.598359] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 360
[ 7555.598364] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 361
[ 7555.610172] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 362
[ 7555.610480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 363
[ 7555.610484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 364
[ 7555.622185] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 365
[ 7555.622343] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 366
[ 7555.622347] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 367
[ 7555.634181] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 368
[ 7555.634482] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 369
[ 7555.634486] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 370
[ 7555.646186] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 371
[ 7555.646478] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 372
[ 7555.646482] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 373
[ 7555.658180] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 374
[ 7555.658480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 375
[ 7555.658485] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 376
[ 7555.670187] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 377
[ 7555.670478] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 378
[ 7555.670482] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 379
[ 7555.682183] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 380
[ 7555.682481] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 381
[ 7555.682485] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 382
[ 7555.694178] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 383
[ 7555.694342] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 384
[ 7555.694347] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 385
[ 7555.706164] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 386
[ 7555.706358] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 387
[ 7555.706363] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 388
[ 7555.718181] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 389
[ 7555.718359] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 390
[ 7555.718363] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 391
[ 7555.730207] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 392
[ 7555.730481] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 393
[ 7555.730485] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 394
[ 7555.742183] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 395
[ 7555.742356] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 396
[ 7555.742360] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 397
[ 7555.754185] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 398
[ 7555.754482] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 399
[ 7555.754487] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 400
[ 7555.766205] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 401
[ 7555.766479] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 402
[ 7555.766483] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 403
[ 7555.778164] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 404
[ 7555.778480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 405
[ 7555.778484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 406
[ 7555.790180] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 407
[ 7555.790354] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 408
[ 7555.790603] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 409
[ 7555.790853] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 410
[ 7555.791103] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 411
[ 7555.791355] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 412
[ 7555.791559] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 413
[ 7555.791681] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 414
[ 7555.791807] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 415
[ 7555.791931] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 416
[ 7555.818180] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setgain - 417
[ 7555.818189] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 418
[ 7555.818192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 419
[ 7555.830181] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 420
[ 7555.830480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setbrightness - 421
[ 7555.830484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 422
[ 7555.830487] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 423
[ 7555.842184] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 424
[ 7555.842461] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setexposure - 425
[ 7555.842464] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 426
[ 7555.842466] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 427
[ 7555.854186] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 428
[ 7555.854482] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setfreq - 429
[ 7555.854486] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 430
[ 7555.854489] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 431
[ 7555.866187] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 432
[ 7555.866481] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 433
[ 7555.866486] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 434
[ 7555.866488] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 435
[ 7555.866510] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_stopN - 436
[ 7555.866512] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 437
[ 7555.866514] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 438
[ 7555.866928] gspca_elgin_sn9c103 2-1.7:1.0: alt 4 - bandwidth not wide enough, trying again
[ 7555.896683] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_start - 439
[ 7555.896688] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 440
[ 7555.896946] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 441
[ 7555.897196] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 442
[ 7555.897446] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w_vector - 443
[ 7555.897450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 444
[ 7555.897453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 445
[ 7555.906177] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 446
[ 7555.906328] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 447
[ 7555.906332] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 448
[ 7555.918188] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 449
[ 7555.918450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 450
[ 7555.918454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 451
[ 7555.930187] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 452
[ 7555.930324] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 453
[ 7555.930329] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 454
[ 7555.942185] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 455
[ 7555.942450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 456
[ 7555.942454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 457
[ 7555.954189] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 458
[ 7555.954448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 459
[ 7555.954452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 460
[ 7555.966175] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 461
[ 7555.966399] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 462
[ 7555.966405] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 463
[ 7555.978188] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 464
[ 7555.978325] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 465
[ 7555.978329] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 466
[ 7555.990189] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 467
[ 7555.990465] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 468
[ 7555.990470] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 469
[ 7556.002188] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 470
[ 7556.002447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 471
[ 7556.002452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 472
[ 7556.014189] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 473
[ 7556.014448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 474
[ 7556.014452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 475
[ 7556.026187] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 476
[ 7556.026326] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 477
[ 7556.026330] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 478
[ 7556.038190] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 479
[ 7556.038448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 480
[ 7556.038453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 481
[ 7556.050190] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 482
[ 7556.050449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 483
[ 7556.050453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 484
[ 7556.062190] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 485
[ 7556.062448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 486
[ 7556.062452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 487
[ 7556.074188] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 488
[ 7556.074448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 489
[ 7556.074452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 490
[ 7556.086192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 491
[ 7556.086449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 492
[ 7556.086454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 493
[ 7556.098191] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 494
[ 7556.098448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 495
[ 7556.098452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 496
[ 7556.110201] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 497
[ 7556.110448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 498
[ 7556.110453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 499
[ 7556.122191] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 500
[ 7556.122447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 501
[ 7556.122451] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 502
[ 7556.134175] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 503
[ 7556.134401] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 504
[ 7556.134406] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 505
[ 7556.146208] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 506
[ 7556.146455] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 507
[ 7556.146460] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 508
[ 7556.158192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 509
[ 7556.158447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 510
[ 7556.158452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 511
[ 7556.170191] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 512
[ 7556.170446] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 513
[ 7556.170695] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 514
[ 7556.170947] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 515
[ 7556.171195] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 516
[ 7556.171393] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 517
[ 7556.171514] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 518
[ 7556.171696] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 519
[ 7556.171895] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 520
[ 7556.172013] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 521
[ 7556.198192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setgain - 522
[ 7556.198197] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 523
[ 7556.198199] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 524
[ 7556.210174] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 525
[ 7556.210401] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setbrightness - 526
[ 7556.210405] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 527
[ 7556.210408] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 528
[ 7556.222195] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 529
[ 7556.222447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setexposure - 530
[ 7556.222451] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 531
[ 7556.222454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 532
[ 7556.234181] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 533
[ 7556.234457] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setfreq - 534
[ 7556.234461] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 535
[ 7556.234472] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 536
[ 7556.246211] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 537
[ 7556.246451] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 538
[ 7556.246456] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 539
[ 7556.246458] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 540
[ 7556.246474] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_stopN - 541
[ 7556.246476] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 542
[ 7556.246478] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 543
[ 7556.246944] gspca_elgin_sn9c103 2-1.7:1.0: alt 3 - bandwidth not wide enough, trying again
[ 7556.276682] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_start - 544
[ 7556.276687] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 545
[ 7556.276828] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 546
[ 7556.277070] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 547
[ 7556.277321] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w_vector - 548
[ 7556.277326] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 549
[ 7556.277328] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 550
[ 7556.286192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 551
[ 7556.286446] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 552
[ 7556.286450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 553
[ 7556.298176] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 554
[ 7556.298450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 555
[ 7556.298454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 556
[ 7556.310171] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 557
[ 7556.310324] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 558
[ 7556.310328] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 559
[ 7556.322193] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 560
[ 7556.322449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 561
[ 7556.322453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 562
[ 7556.334194] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 563
[ 7556.334448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 564
[ 7556.334452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 565
[ 7556.346192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 566
[ 7556.346448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 567
[ 7556.346453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 568
[ 7556.358193] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 569
[ 7556.358448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 570
[ 7556.358453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 571
[ 7556.370192] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 572
[ 7556.370449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 573
[ 7556.370454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 574
[ 7556.382194] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 575
[ 7556.382447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 576
[ 7556.382451] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 577
[ 7556.394195] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 578
[ 7556.394449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 579
[ 7556.394453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 580
[ 7556.406188] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 581
[ 7556.406446] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 582
[ 7556.406451] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 583
[ 7556.418193] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 584
[ 7556.418449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 585
[ 7556.418453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 586
[ 7556.430194] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 587
[ 7556.430448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 588
[ 7556.430452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 589
[ 7556.442193] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 590
[ 7556.442449] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 591
[ 7556.442454] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 592
[ 7556.454198] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 593
[ 7556.454446] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 594
[ 7556.454450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 595
[ 7556.466198] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 596
[ 7556.466448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 597
[ 7556.466452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 598
[ 7556.478205] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 599
[ 7556.478447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 600
[ 7556.478452] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 601
[ 7556.490199] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 602
[ 7556.490448] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 603
[ 7556.490453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 604
[ 7556.502199] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 605
[ 7556.502447] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 606
[ 7556.502451] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 607
[ 7556.514199] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 608
[ 7556.514481] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 609
[ 7556.514485] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 610
[ 7556.526197] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 611
[ 7556.526480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 612
[ 7556.526484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 613
[ 7556.538198] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 614
[ 7556.538480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 615
[ 7556.538484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 616
[ 7556.550204] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 617
[ 7556.550478] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 618
[ 7556.550729] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 619
[ 7556.550978] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 620
[ 7556.551228] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 621
[ 7556.551480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 622
[ 7556.551730] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 623
[ 7556.551889] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 624
[ 7556.552013] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 625
[ 7556.552138] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 626
[ 7556.578219] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setgain - 627
[ 7556.578224] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 628
[ 7556.578226] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 629
[ 7556.590198] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 630
[ 7556.590480] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setbrightness - 631
[ 7556.590484] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 632
[ 7556.590487] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 633
[ 7556.602200] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 634
[ 7556.602478] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setexposure - 635
[ 7556.602483] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 636
[ 7556.602485] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 637
[ 7556.614204] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 638
[ 7556.614481] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: setfreq - 639
[ 7556.614486] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: i2c_w - 640
[ 7556.614488] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 641
[ 7556.626210] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_r - 642
[ 7556.626445] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 643
[ 7556.626450] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 644
[ 7556.626453] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_s_ctrl - 645
[ 7561.595064] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: gspca_stop_streaming - 646
[ 7561.595068] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_stopN - 647
[ 7561.595071] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: sd_init - 648
[ 7561.595073] gspca_elgin_sn9c103 2-1.7:1.0: Elgin_sn9c103: reg_w - 649


*/
