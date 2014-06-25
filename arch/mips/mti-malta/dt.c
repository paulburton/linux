/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <asm/prom.h>
#include <linux/init.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>

extern u32 __dtb_malta_begin[];

void __init device_tree_init(void)
{
	__dt_setup_arch(__dtb_malta_begin);
	unflatten_and_copy_device_tree();
}

static struct of_device_id __initdata bus_ids[] = {
	{ .compatible = "simple-bus", },
	{ .compatible = "isa", },
	{},
};

int __init malta_publish_devices(void)
{
	if (!of_have_populated_dt())
		return 0;

	return of_platform_bus_probe(NULL, bus_ids, NULL);
}
device_initcall(malta_publish_devices);
