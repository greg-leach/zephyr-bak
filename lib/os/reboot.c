/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 * Copyright (c) 2021 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <sys/printk.h>

extern void sys_arch_reboot(int type);
extern void sys_clock_disable(void);

__weak void sys_reboot_notification(int type)
{
        return;
}

void sys_reboot(int type)
{
	/* Notify application of pending reboot */
	sys_reboot_notification(type);

	(void)irq_lock();
#ifdef CONFIG_SYS_CLOCK_EXISTS
	sys_clock_disable();
#endif

	sys_arch_reboot(type);

	/* should never get here */
	printk("Failed to reboot: spinning endlessly...\n");
	for (;;) {
		k_cpu_idle();
	}
}
