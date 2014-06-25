/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <asm/uaccess.h>
#include <generated/utsrelease.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

#define LCD_CHARS	8

#define ASCIIWORD	0x00
#define ASCIIPOS0	0x08
#define ASCIIPOS1	0x10
#define ASCIIPOSN(n)	(ASCIIPOS0 + ((n) * (ASCIIPOS1 - ASCIIPOS0)))

struct malta_lcd_ctx {
	void __iomem *base;
	char *message;
	unsigned message_len;
	unsigned scroll_pos;
	unsigned scroll_rate;
	struct timer_list timer;
	struct miscdevice miscdev;
	struct list_head node;
};

static LIST_HEAD(malta_lcd_list);
static DEFINE_MUTEX(malta_lcd_mutex);

static void malta_lcd_update(struct malta_lcd_ctx *ctx)
{
	unsigned i, ch = ctx->scroll_pos;

	for (i = 0; i < LCD_CHARS;) {
		/* copy as many characters from the string as possible */
		for (; i < LCD_CHARS && ch < ctx->message_len; i++, ch++)
			__raw_writel(ctx->message[ch],
				     ctx->base + ASCIIPOSN(i));

		/* wrap around to the start of the string */
		ch = 0;
	}
}

static void malta_lcd_scroll(unsigned long arg)
{
	struct malta_lcd_ctx *ctx = (struct malta_lcd_ctx *)arg;

	/* update the LCD */
	malta_lcd_update(ctx);

	/* move on to the next character */
	ctx->scroll_pos++;
	ctx->scroll_pos %= ctx->message_len;

	/* rearm the timer */
	mod_timer(&ctx->timer, jiffies + ctx->scroll_rate);
}

static int malta_lcd_display(struct malta_lcd_ctx *ctx, const char *msg)
{
	/* stop the scroll timer */
	del_timer(&ctx->timer);

	/* switch the message */
	if (msg) {
		kfree(ctx->message);
		ctx->message = kstrdup(msg, GFP_KERNEL);
		if (!ctx->message)
			return -ENOMEM;
	}

	ctx->message_len = strlen(ctx->message);
	ctx->scroll_pos = 0;

	/* update the LCD */
	malta_lcd_update(ctx);

	if (ctx->message_len > LCD_CHARS) {
		/* start the scroll timer */
		ctx->timer.expires = jiffies + ctx->scroll_rate;
		add_timer(&ctx->timer);
	}

	return 0;
}

static struct malta_lcd_ctx *malta_lcd_find(int minor)
{
	struct malta_lcd_ctx *ctx;

	list_for_each_entry(ctx, &malta_lcd_list, node) {
		if (ctx->miscdev.minor == minor)
			return ctx;
	}

	return NULL;
}

static int malta_lcd_open(struct inode *inode, struct file *filp)
{
	int err;

	err = mutex_lock_interruptible(&malta_lcd_mutex);
	if (err)
		return err;

	/* find the context for read/write */
	filp->private_data = malta_lcd_find(iminor(inode));
	if (!filp->private_data)
		err = -ENODEV;

	mutex_unlock(&malta_lcd_mutex);
	return err;
}

static ssize_t malta_lcd_read(struct file *filp, char __user *data,
			      size_t len, loff_t *ppos)
{
	struct malta_lcd_ctx *ctx = filp->private_data;
	size_t copy_bytes;

	if (*ppos >= ctx->message_len + 1)
		return 0;

	/* copy the message string */
	copy_bytes = min_t(size_t, ctx->message_len - *ppos, len);
	copy_to_user(data, &ctx->message[*ppos], copy_bytes);
	*ppos += copy_bytes;
	data += copy_bytes;
	len -= copy_bytes;

	/* add a newline at the end of the message */
	if (len && (*ppos == ctx->message_len)) {
		put_user('\n', data);
		(*ppos)++;
		copy_bytes++;
	}

	return copy_bytes;
}

static ssize_t malta_lcd_write(struct file *filp, const char __user *data,
			       size_t len, loff_t *ppos)
{
	struct malta_lcd_ctx *ctx = filp->private_data;
	char *new_msg;

	/* increase the message buffer size if necessary */
	if (*ppos + len + 1 > ctx->message_len) {
		new_msg = kmalloc(*ppos + len + 1, GFP_KERNEL);
		if (!new_msg)
			return -ENOMEM;

		memcpy(new_msg, ctx->message, ctx->message_len);
		kfree(ctx->message);
		ctx->message = new_msg;
	}

	/* copy the string & NULL terminate it */
	copy_from_user(&ctx->message[*ppos], data, len);
	ctx->message[*ppos + len] = 0;
	ctx->message_len = strlen(ctx->message);

	/* if the string ends with a newline, trim it */
	if (ctx->message_len && ctx->message[ctx->message_len - 1] == '\n') {
		ctx->message[ctx->message_len - 1] = 0;
		ctx->message_len--;
	}

	/* display the new message */
	malta_lcd_display(ctx, NULL);

	return len;
}

static const struct file_operations malta_lcd_char_ops = {
	.owner	= THIS_MODULE,
	.open	= malta_lcd_open,
	.write	= malta_lcd_write,
	.read	= malta_lcd_read,
};

static int malta_lcd_probe(struct platform_device *pdev)
{
	static unsigned lcd_count = 0;
	struct device_node *np = pdev->dev.of_node;
	struct malta_lcd_ctx *ctx;
	int err;

	ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		err = -ENOMEM;
		goto out;
	}

	ctx->base = of_iomap(np, 0);
	if (!ctx->base) {
		err = -EIO;
		goto out_free;
	}

	ctx->message = NULL;
	ctx->scroll_pos = 0;
	ctx->scroll_rate = HZ / 2;

	/* initialise a timer for scrolling the message */
	init_timer(&ctx->timer);
	ctx->timer.function = malta_lcd_scroll;
	ctx->timer.data = (unsigned long)ctx;

	platform_set_drvdata(pdev, ctx);

	/* display a default message */
	err = malta_lcd_display(ctx, "Linux " UTS_RELEASE "       ");
	if (err)
		goto out_free;

	mutex_lock(&malta_lcd_mutex);
	list_add_tail(&ctx->node, &malta_lcd_list);
	mutex_unlock(&malta_lcd_mutex);

	ctx->miscdev.minor = MISC_DYNAMIC_MINOR;
	ctx->miscdev.fops = &malta_lcd_char_ops;
	ctx->miscdev.parent = &pdev->dev;
	ctx->miscdev.name = kasprintf(GFP_KERNEL, "malta-lcd-%u", lcd_count++);
	if (!ctx->miscdev.name) {
		err = -ENOMEM;
		goto out_free;
	}

	err = misc_register(&ctx->miscdev);
	if (err)
		goto out_free_name;

	return 0;
out_free_name:
	kfree(ctx->miscdev.name);
out_free:
	kfree(ctx);
out:
	return err;
}

static int malta_lcd_remove(struct platform_device *pdev)
{
	struct malta_lcd_ctx *ctx = platform_get_drvdata(pdev);
	int err;

	err = misc_deregister(&ctx->miscdev);
	if (err)
		return err;

	kfree(ctx->miscdev.name);
	del_timer(&ctx->timer);
	kfree(ctx->message);
	iounmap(ctx->base);
	return 0;
}

static struct of_device_id malta_lcd_matches[] = {
	{ .compatible = "img,malta-lcd" },
};

static struct platform_driver malta_lcd_driver = {
	.driver = {
		.name		= "malta-lcd",
		.owner		= THIS_MODULE,
		.of_match_table	= malta_lcd_matches,
	},
	.probe	= malta_lcd_probe,
	.remove	= malta_lcd_remove,
};
module_platform_driver(malta_lcd_driver);
