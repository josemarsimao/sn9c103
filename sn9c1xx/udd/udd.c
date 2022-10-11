/***************************************************************************
 * USB DEVICE DRIVER TEMPLATE                                              *
 *                                                                         *
 * Copyright (C) 2022-2022 by Josemar Simão <josemars@ifes.edu.br>         *
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




#include <linux/init.h>
#include <linux/module.h>
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Josemar Simao - josemars@ifes.edu.br");
MODULE_DESCRIPTION("A simple example Linux module.");
MODULE_VERSION("0.01");


enum sn9c102_bridge {
	BRIDGE_SN9C101 = 0x01,
	BRIDGE_SN9C102 = 0x02,
	BRIDGE_SN9C103 = 0x04,
	BRIDGE_SN9C105 = 0x08,
	BRIDGE_SN9C120 = 0x10,
};

#define UDD_USB_DEVICE(vend, prod, bridge)                                \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE |                           \
	               USB_DEVICE_ID_MATCH_INT_CLASS,                         \
	.idVendor = (vend),                                                   \
	.idProduct = (prod),                                                  \
	.bInterfaceClass = 0xff,                                              \
	.driver_info = (bridge)


static const struct usb_device_id udd_id_table[] = {
	{ UDD_USB_DEVICE(0x0c45, 0x608f, BRIDGE_SN9C103), },
	{ }
};

MODULE_DEVICE_TABLE(usb, udd_id_table);



int registers = 0;

/* It exists a single interface only. We do not need to validate anything. */
/* Called if device is plugged */
static int udd_usb_probe(struct usb_interface* intf, const struct usb_device_id* id) {

	registers++;
	pr_info("udd: " "Probing... %d\n", registers); 
	
	return 0;
}

/* Called when device is unplugged */
static void udd_usb_disconnect(struct usb_interface* intf)
{
	pr_info("udd: " "Disconnecting..." "\n"); 
}


static struct usb_driver udd_usb_driver = {
	.name = "udd",
	.probe = udd_usb_probe,
	.disconnect = udd_usb_disconnect,
	.id_table = udd_id_table,
};

/*****************************************************************************/



/* Called when module is loaded */
static int __init udd_module_init(void)
{
	int err = 0;
	
	pr_info("udd: " "registering... \n"); 
	err = usb_register(&udd_usb_driver);

	if (err < 0){
		
		return -1;

	}

	return 0;
}

/* Called when module is unloaded */
static void __exit udd_module_exit(void)
{
	pr_info("udd: " "deregistering...\n"); 
	usb_deregister(&udd_usb_driver);
}


module_init(udd_module_init);
module_exit(udd_module_exit);
