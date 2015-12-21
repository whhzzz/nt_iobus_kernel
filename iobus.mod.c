#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x3972220f, "module_layout" },
	{ 0xcbc29716, "kmalloc_caches" },
	{ 0x2dfa9905, "__mxc_ioremap" },
	{ 0xf3703692, "device_create" },
	{ 0x8361ec5b, "__class_create" },
	{ 0x3ff6b7e9, "cdev_add" },
	{ 0x8ba3ac11, "cdev_init" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0x68a3b589, "kmem_cache_alloc" },
	{ 0x67c2fa54, "__copy_to_user" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0xfa2a45e, "__memzero" },
	{ 0xfbc74f64, "__copy_from_user" },
	{ 0x8893fa5d, "finish_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x75a17bed, "prepare_to_wait" },
	{ 0x5f754e5a, "memset" },
	{ 0xb9e52429, "__wake_up" },
	{ 0xf6288e02, "__init_waitqueue_head" },
	{ 0x9545af6d, "tasklet_init" },
	{ 0x859c6dc7, "request_threaded_irq" },
	{ 0xea147363, "printk" },
	{ 0x43b0c9c3, "preempt_schedule" },
	{ 0x82072614, "tasklet_kill" },
	{ 0xf20dabd8, "free_irq" },
	{ 0x37a0cba, "kfree" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0xa6c7eaa1, "cdev_del" },
	{ 0x327717a0, "class_destroy" },
	{ 0x498c887a, "device_destroy" },
	{ 0x45a55ec8, "__iounmap" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

