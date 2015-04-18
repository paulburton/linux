/*
 * Ingenic JZ4775 SoC CGU driver
 *
 * Copyright (c) 2015 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <dt-bindings/clock/jz4775-cgu.h>
#include "cgu.h"

/* CGU register offsets */
#define CGU_REG_CLOCKCONTROL	0x00
#define CGU_REG_PLLCONTROL	0x0c
#define CGU_REG_APLL		0x10
#define CGU_REG_MPLL		0x14
#define CGU_REG_CLKGR		0x20
#define CGU_REG_OPCR		0x24
#define CGU_REG_CLKGR1		0x28
#define CGU_REG_DDRCDR		0x2c
#define CGU_REG_VPUCDR		0x30
#define CGU_REG_USBPCR		0x3c
#define CGU_REG_USBRDT		0x40
#define CGU_REG_USBVBFIL	0x44
#define CGU_REG_USBPCR1		0x48
#define CGU_REG_USBCDR		0x50
#define CGU_REG_I2SCDR		0x60
#define CGU_REG_LPCDR		0x64
#define CGU_REG_MSC0CDR		0x68
#define CGU_REG_UHCCDR		0x6c
#define CGU_REG_SSICDR		0x74
#define CGU_REG_CIMCDR		0x7c
#define CGU_REG_CIM1CDR		0x80
#define CGU_REG_PCMCDR		0x84
#define CGU_REG_MSC1CDR		0xa4
#define CGU_REG_MSC2CDR		0xa8
#define CGU_REG_BCHCDR		0xac
#define CGU_REG_CLOCKSTATUS	0xd4

static struct ingenic_cgu *cgu;

static const s8 pll_od_encoding[4] = {
	0x0, 0x1, 0x3, 0x7,
};

static const struct ingenic_cgu_clk_info jz4775_cgu_clocks[] = {

	/* External clocks */

	[JZ4775_CLK_EXCLK] = { "ext", CGU_CLK_EXT },
	[JZ4775_CLK_RTCLK] = { "rtc", CGU_CLK_EXT },

	[JZ4775_CLK_EXCLK_HALF] = {
		"ext_half", CGU_CLK_FIXDIV,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.fixdiv = { 2 },
	},

	/* PLLs */

#define DEF_PLL(name) { \
	.reg = CGU_REG_ ## name, \
	.m_shift = 24, \
	.m_bits = 7, \
	.m_offset = 1, \
	.n_shift = 18, \
	.n_bits = 5, \
	.n_offset = 1, \
	.od_shift = 16, \
	.od_bits = 2, \
	.od_max = 8, \
	.od_encoding = pll_od_encoding, \
	.stable_bit = 0, \
	.bypass_bit = 6, \
	.enable_bit = 7, \
}

	[JZ4775_CLK_APLL] = {
		"apll", CGU_CLK_PLL,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.pll = DEF_PLL(APLL),
	},

	[JZ4775_CLK_MPLL] = {
		"mpll", CGU_CLK_PLL,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.pll = DEF_PLL(MPLL),
	},

#undef DEF_PLL

	/* Muxes & dividers */

	[JZ4775_CLK_SCLKA] = {
		"sclk_a", CGU_CLK_MUX,
		.parents = { -1, JZ4775_CLK_APLL, JZ4775_CLK_EXCLK,
			     JZ4775_CLK_RTCLK },
		.mux = { CGU_REG_CLOCKCONTROL, 30, 2 },
	},

	[JZ4775_CLK_CPUMUX] = {
		"cpumux", CGU_CLK_MUX,
		.parents = { -1, JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 28, 2 },
	},

	[JZ4775_CLK_CPU] = {
		"cpu", CGU_CLK_DIV,
		.parents = { JZ4775_CLK_CPUMUX, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 0, 4, 22, -1, -1 },
	},

	[JZ4775_CLK_L2CACHE] = {
		"l2cache", CGU_CLK_DIV,
		.parents = { JZ4775_CLK_CPUMUX, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 4, 4, -1, -1, -1 },
	},

	[JZ4775_CLK_AHB0] = {
		"ahb0", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { -1, JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 26, 2 },
		.div = { CGU_REG_CLOCKCONTROL, 8, 4, 21, -1, -1 },
	},

	[JZ4775_CLK_AHB2PMUX] = {
		"ahb2_apb_mux", CGU_CLK_MUX,
		.parents = { -1, JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL,
			     JZ4775_CLK_RTCLK },
		.mux = { CGU_REG_CLOCKCONTROL, 24, 2 },
	},

	[JZ4775_CLK_AHB2] = {
		"ahb2", CGU_CLK_DIV,
		.parents = { JZ4775_CLK_AHB2PMUX, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 12, 4, 20, -1, -1 },
	},

	[JZ4775_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV,
		.parents = { JZ4775_CLK_AHB2PMUX, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 16, 4, 20, -1, -1 },
	},

	[JZ4775_CLK_DDR] = {
		"ddr", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { -1, JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_DDRCDR, 30, 2 },
		.div = { CGU_REG_DDRCDR, 0, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 31 },
	},

	[JZ4775_CLK_VPU] = {
		"vpu", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_VPUCDR, 31, 1 },
		.div = { CGU_REG_VPUCDR, 0, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 19 },
	},

	[JZ4775_CLK_OTGPHY_PLL] = {
		"otg_phy_pll", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_USBCDR, 30, 1},
		.div = { CGU_REG_USBCDR, 0, 8, 29, 28, 27 },
	},

	[JZ4775_CLK_OTGPHY] = {
		"otg_phy", CGU_CLK_MUX,
		.parents = { JZ4775_CLK_EXCLK, JZ4775_CLK_OTGPHY_PLL },
		.mux = { CGU_REG_USBCDR, 31, 1},
	},

	[JZ4775_CLK_I2S_PLL] = {
		"i2s_pll", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_I2SCDR, 30, 1 },
		.div = { CGU_REG_I2SCDR, 0, 8, 29, 28, 27 },
	},

	[JZ4775_CLK_I2S] = {
		"i2s", CGU_CLK_MUX,
		.parents = { JZ4775_CLK_EXCLK_HALF, JZ4775_CLK_I2S_PLL, -1 },
		.mux = { CGU_REG_I2SCDR, 31, 1 },
	},

	[JZ4775_CLK_LCDPIXCLK] = {
		"lcdpixclk", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_LPCDR, 31, 1 },
		.div = { CGU_REG_LPCDR, 0, 8, 28, 27, 26 },
	},

	[JZ4775_CLK_MSCMUX] = {
		"msc_mux", CGU_CLK_MUX,
		.parents = { -1, JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_MSC0CDR, 30, 2 },
	},

	[JZ4775_CLK_MSC0] = {
		"msc0", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4775_CLK_MSCMUX, -1 },
		.div = { CGU_REG_MSC0CDR, 0, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 3 },
	},

	[JZ4775_CLK_MSC1] = {
		"msc1", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4775_CLK_MSCMUX, -1 },
		.div = { CGU_REG_MSC1CDR, 0, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 11 },
	},

	[JZ4775_CLK_MSC2] = {
		"msc2", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4775_CLK_MSCMUX, -1 },
		.div = { CGU_REG_MSC2CDR, 0, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 12 },
	},

	[JZ4775_CLK_UHC] = {
		"uhc", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL,
			     JZ4775_CLK_OTGPHY, -1 },
		.mux = { CGU_REG_UHCCDR, 30, 2 },
		.div = { CGU_REG_UHCCDR, 0, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 22 },
	},

	[JZ4775_CLK_SSI_PLL] = {
		"ssi_pll", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_SSICDR, 30, 1 },
		.div = { CGU_REG_SSICDR, 0, 8, 29, 28, 27 },
	},

	[JZ4775_CLK_SSI] = {
		"ssi", CGU_CLK_MUX,
		.parents = { JZ4775_CLK_EXCLK, JZ4775_CLK_SSI_PLL, -1 },
		.mux = { CGU_REG_SSICDR, 31, 1 },
	},

	[JZ4775_CLK_CIM_MUX] = {
		"cim_mux", CGU_CLK_MUX,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_CIMCDR, 31, 1 },
	},

	[JZ4775_CLK_CIM0_MCLK] = {
		"cim_mclk", CGU_CLK_DIV,
		.parents = { JZ4775_CLK_CIM_MUX, -1 },
		.div = { CGU_REG_CIMCDR, 0, 8, 30, 29, 28 },
	},

	[JZ4775_CLK_CIM1_MCLK] = {
		"cim1_mclk", CGU_CLK_DIV,
		.parents = { JZ4775_CLK_CIM_MUX, -1 },
		.div = { CGU_REG_CIM1CDR, 0, 8, 30, 29, 28 },
	},

	[JZ4775_CLK_PCM_PLL] = {
		"pcm_pll", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_PCMCDR, 30, 1 },
		.div = { CGU_REG_PCMCDR, 0, 8, 28, 27, 26 },
	},

	[JZ4775_CLK_PCM] = {
		"pcm", CGU_CLK_MUX | CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK_HALF, JZ4775_CLK_PCM_PLL, -1 },
		.mux = { CGU_REG_PCMCDR, 31, 1 },
		.gate = { CGU_REG_CLKGR, 13 },
	},

	[JZ4775_CLK_BCH] = {
		"bch", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { -1, JZ4775_CLK_SCLKA, JZ4775_CLK_MPLL, -1 },
		.mux = { CGU_REG_BCHCDR, 30, 2 },
		.div = { CGU_REG_BCHCDR, 0, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 1 },
	},

	/* Gate-only clocks */

	[JZ4775_CLK_NEMC] = {
		"nemc", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_AHB2, -1 },
		.gate = { CGU_REG_CLKGR, 0 },
	},

	[JZ4775_CLK_OTG] = {
		"otg", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 2 },
	},

	[JZ4775_CLK_SSI0] = {
		"ssi0", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_SSI, -1 },
		.gate = { CGU_REG_CLKGR, 4 },
	},

	[JZ4775_CLK_I2C0] = {
		"i2c0", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_PCLK, -1 },
		.gate = { CGU_REG_CLKGR, 5 },
	},

	[JZ4775_CLK_I2C1] = {
		"i2c1", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_PCLK, -1 },
		.gate = { CGU_REG_CLKGR, 6 },
	},

	[JZ4775_CLK_I2C2] = {
		"i2c2", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 7 },
	},

	[JZ4775_CLK_AIC] = {
		"aic", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 8 },
	},

	[JZ4775_CLK_X2D] = {
		"x2d", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 9 },
	},

	[JZ4775_CLK_AHB_MON] = {
		"ahb_mon", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 10 },
	},

	[JZ4775_CLK_SADC] = {
		"sadc", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 14 },
	},

	[JZ4775_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 15 },
	},

	[JZ4775_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 16 },
	},

	[JZ4775_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 17 },
	},

	[JZ4775_CLK_UART3] = {
		"uart3", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 18 },
	},

	[JZ4775_CLK_PDMA] = {
		"pdma", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 20 },
	},

	[JZ4775_CLK_MAC] = {
		"mac", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 21 },
	},

	[JZ4775_CLK_CIM0] = {
		"cim0", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 23 },
	},

	[JZ4775_CLK_CIM1] = {
		"cim1", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 24 },
	},

	[JZ4775_CLK_LCD] = {
		"lcd", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 25 },
	},

	[JZ4775_CLK_EPDC] = {
		"epdc", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 26 },
	},

	[JZ4775_CLK_EPDE] = {
		"epde", CGU_CLK_GATE,
		.parents = { JZ4775_CLK_EXCLK, -1 },
		.gate = { CGU_REG_CLKGR, 27 },
	},

};

static void __init jz4775_cgu_init(struct device_node *np)
{
	int retval;

	cgu = ingenic_cgu_new(jz4775_cgu_clocks,
			      ARRAY_SIZE(jz4775_cgu_clocks), np);
	if (!cgu)
		pr_err("%s: failed to initialise CGU\n", __func__);

	retval = ingenic_cgu_register_clocks(cgu);
	if (retval)
		pr_err("%s: failed to register CGU Clocks\n", __func__);
}
CLK_OF_DECLARE(jz4775_cgu, "ingenic,jz4775-cgu", jz4775_cgu_init);
