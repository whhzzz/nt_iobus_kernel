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
	{ 0xb76c596d, "module_layout" },
	{ 0xb48a77ef, "kmalloc_caches" },
	{ 0x40a6f522, "__arm_ioremap" },
	{ 0x4c57bdeb, "device_create" },
	{ 0x56303c2f, "__class_create" },
	{ 0x32e78a59, "cdev_add" },
	{ 0x88b2953c, "cdev_init" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xbcf83ac5, "kmem_cache_alloc" },
	{ 0x67c2fa54, "__copy_to_user" },
	{ 0x7d11c268, "jiffies" },
	{ 0xfa2a45e, "__memzero" },
	{ 0xfbc74f64, "__copy_from_user" },
	{ 0x8a7c6f1, "add_timer" },
	{ 0xf83178bd, "finish_wait" },
	{ 0x32f80ea9, "prepare_to_wait" },
	{ 0x1000e51, "schedule" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x72542c85, "__wake_up" },
	{ 0x27e1a049, "printk" },
	{ 0x7426f76c, "init_timer_key" },
	{ 0x41e92619, "__init_waitqueue_head" },
	{ 0x9545af6d, "tasklet_init" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0xa170bbdb, "outer_cache" },
	{ 0xc212a112, "_raw_spin_unlock_irq" },
	{ 0xd59daefe, "_raw_spin_lock_irq" },
	{ 0x1b945caf, "del_timer" },
	{ 0x82072614, "tasklet_kill" },
	{ 0xf20dabd8, "free_irq" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0x37a0cba, "kfree" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0xb3f8ec84, "cdev_del" },
	{ 0x86626159, "class_destroy" },
	{ 0xbb3ad7bb, "device_destroy" },
	{ 0x45a55ec8, "__iounmap" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

