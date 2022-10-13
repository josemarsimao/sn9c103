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
	{ 0x64491f21, "module_layout" },
	{ 0xeebae47a, "param_ops_short" },
	{ 0xc3ff0abd, "param_ops_bool" },
	{ 0xb78dec9d, "param_ops_uint" },
	{ 0x142c2787, "param_array_ops" },
	{ 0x999a9d51, "param_ops_ushort" },
	{ 0x13d86244, "usb_deregister" },
	{ 0x9d1de69d, "usb_register_driver" },
	{ 0x837b7b09, "__dynamic_pr_debug" },
	{ 0xc5850110, "printk" },
	{ 0xcd4f88d7, "usb_alloc_urb" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf84acc9b, "usb_altnum_to_altsetting" },
	{ 0x262e823a, "wait_for_completion_interruptible" },
	{ 0xf21017d9, "mutex_trylock" },
	{ 0x50eb1f19, "device_remove_file" },
	{ 0xfd94814e, "complete_all" },
	{ 0xec67dac3, "usb_get_dev" },
	{ 0xeac102bb, "device_create_file" },
	{ 0x4055b971, "__video_register_device" },
	{ 0xe9ff99b7, "video_device_release" },
	{ 0xe87d7621, "video_device_alloc" },
	{ 0xcf017746, "v4l2_device_register" },
	{ 0xc687b2c, "kmem_cache_alloc_trace" },
	{ 0x2f6390d, "kmalloc_caches" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x977f511b, "__mutex_init" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xbc5671dc, "v4l_printk_ioctl" },
	{ 0x6283ebdf, "usb_control_msg" },
	{ 0x3c93885d, "usb_match_id" },
	{ 0xa8c3d90c, "usb_ifnum_to_if" },
	{ 0x165b145c, "ex_handler_refcount" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xc2bf9c0e, "usb_submit_urb" },
	{ 0xdf3e0d02, "pv_ops" },
	{ 0xdbf17652, "_raw_spin_lock" },
	{ 0x69acdf38, "memcpy" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x20000329, "simple_strtoul" },
	{ 0x9166fada, "strncpy" },
	{ 0x5b93a99a, "vm_insert_page" },
	{ 0x173cb918, "vmalloc_to_page" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x2db3d320, "mutex_lock_interruptible" },
	{ 0xcff43d97, "usb_set_interface" },
	{ 0xe18ac8e0, "usb_free_urb" },
	{ 0x86e2f6ba, "usb_kill_urb" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0xdbddd45d, "usb_put_dev" },
	{ 0x5dc04bca, "video_unregister_device" },
	{ 0x37a0cba, "kfree" },
	{ 0x1b6da787, "v4l2_device_unregister" },
	{ 0xf3dba1dc, "v4l2_device_disconnect" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x754d539c, "strlen" },
	{ 0x999e8297, "vfree" },
	{ 0xb1beb31, "vmalloc_32_user" },
	{ 0x1000e51, "schedule" },
	{ 0x4bfecede, "_dev_err" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0xb44ad4b3, "_copy_to_user" },
	{ 0x2ea2c95c, "__x86_indirect_thunk_rax" },
	{ 0x6dbf546e, "video_devdata" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x362ef408, "_copy_from_user" },
	{ 0x8bec90d8, "_dev_info" },
	{ 0x3812050a, "_raw_spin_unlock_irqrestore" },
	{ 0x51760917, "_raw_spin_lock_irqsave" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "videodev");

MODULE_ALIAS("usb:v0C45p6001d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6005d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6007d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6009d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6011d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p600Dd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6019d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6024d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6025d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6028d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6029d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p602Ad*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p602Bd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p602Cd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p602Dd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p602Ed*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6030d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p603Fd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6080d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6082d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6083d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6088d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p608Ad*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p608Bd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p608Cd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p608Ed*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p608Fd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60A0d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60A2d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60A3d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60A8d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60AAd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60ABd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60ACd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60AEd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60AFd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60B0d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60B2d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60B3d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60B8d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60BAd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60BBd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60BCd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60BEd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v045Ep00F5d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v045Ep00F7d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0471p0327d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0471p0328d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v06F8p3004d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60C0d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60C2d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60C8d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60CCd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60EAd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60ECd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60EFd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60FAd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60FBd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60FCd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p60FEd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0458p7025d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6102d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6108d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p610Fd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p612Ad*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p612Cd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6130d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p6138d*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p613Ad*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p613Bd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p613Cd*dc*dsc*dp*icFFisc*ip*in*");
MODULE_ALIAS("usb:v0C45p613Ed*dc*dsc*dp*icFFisc*ip*in*");

MODULE_INFO(srcversion, "62CA5244043F367FC8633A3");
