#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0xc6edef59, "module_layout" },
	{ 0x191c5c0e, "vb2_ioctl_reqbufs" },
	{ 0xa2cbe45, "kmalloc_caches" },
	{ 0x7de47e1, "v4l2_event_unsubscribe" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf9a482f9, "msleep" },
	{ 0xab968b5b, "video_device_release_empty" },
	{ 0xbe60066c, "v4l2_device_unregister" },
	{ 0xbb006837, "v4l2_ctrl_handler_free" },
	{ 0xba578fdc, "v4l2_ctrl_new_std" },
	{ 0x62b428b4, "vb2_fop_poll" },
	{ 0x59f1cf31, "vb2_ioctl_streamon" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x43420427, "usb_kill_urb" },
	{ 0xca3a6bc2, "vb2_ops_wait_prepare" },
	{ 0xf8727f41, "__video_register_device" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0xdd64e639, "strscpy" },
	{ 0xd9ec443c, "v4l2_device_register" },
	{ 0x555ee713, "vb2_fop_read" },
	{ 0x21cdffbb, "input_event" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x99e961d7, "v4l2_ctrl_new_std_menu" },
	{ 0xf9c0b663, "strlcat" },
	{ 0x387bf060, "v4l2_device_disconnect" },
	{ 0xf88f3508, "vb2_vmalloc_memops" },
	{ 0x3812050a, "_raw_spin_unlock_irqrestore" },
	{ 0xc3ff845b, "vb2_fop_mmap" },
	{ 0x54432f9b, "vb2_ioctl_qbuf" },
	{ 0x87931af6, "usb_deregister" },
	{ 0x977f511b, "__mutex_init" },
	{ 0xc5850110, "printk" },
	{ 0xdfba6911, "video_unregister_device" },
	{ 0x636d14b8, "usb_set_interface" },
	{ 0x84532625, "v4l2_ctrl_subscribe_event" },
	{ 0xac0ed99, "vb2_plane_vaddr" },
	{ 0x9013f92c, "vb2_buffer_done" },
	{ 0xe7b00dfb, "__x86_indirect_thunk_r13" },
	{ 0xcb6c514, "usb_control_msg" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x305ba5ea, "vb2_ioctl_create_bufs" },
	{ 0x56170306, "usb_free_coherent" },
	{ 0xb1abf808, "_dev_err" },
	{ 0x2d499fef, "vb2_ioctl_dqbuf" },
	{ 0xe2a472e1, "usb_submit_urb" },
	{ 0xb601be4c, "__x86_indirect_thunk_rdx" },
	{ 0x1a661ae2, "vb2_fop_release" },
	{ 0x48975461, "vb2_queue_error" },
	{ 0x3d2ad00f, "video_devdata" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0xb3f1bc13, "input_register_device" },
	{ 0x41a53776, "v4l2_ctrl_handler_setup" },
	{ 0xf464e6e7, "usb_clear_halt" },
	{ 0xc3f54027, "input_free_device" },
	{ 0x2ea2c95c, "__x86_indirect_thunk_rax" },
	{ 0xcc314e25, "v4l2_ctrl_auto_cluster" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xa2f50a17, "kmem_cache_alloc_trace" },
	{ 0x51760917, "_raw_spin_lock_irqsave" },
	{ 0xf15e61cc, "v4l2_fh_open" },
	{ 0xebbc50c0, "vb2_ioctl_querybuf" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0x74987ba0, "input_unregister_device" },
	{ 0x20fdae15, "v4l2_ctrl_handler_init_class" },
	{ 0x9d3b5fd7, "usb_register_driver" },
	{ 0x5c362313, "vb2_ops_wait_finish" },
	{ 0xb937e80f, "usb_ifnum_to_if" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x963b770d, "vb2_ioctl_expbuf" },
	{ 0x7044988b, "usb_alloc_coherent" },
	{ 0x4973febf, "vb2_ioctl_streamoff" },
	{ 0x51757574, "v4l2_device_put" },
	{ 0xc33fb9bb, "usb_free_urb" },
	{ 0xf80f69e9, "video_ioctl2" },
	{ 0x716950e2, "usb_alloc_urb" },
	{ 0xf68d3e84, "input_allocate_device" },
	{ 0xa75da217, "vb2_queue_init" },
};

MODULE_INFO(depends, "videobuf2-v4l2,videodev,videobuf2-vmalloc,videobuf2-common");

MODULE_ALIAS("usb:v0C45p608Fd*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "3C0DD96ECDBFB6662F718E3");
