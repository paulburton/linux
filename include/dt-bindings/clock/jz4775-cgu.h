/*
 * This header provides clock numbers for the ingenic,jz4775-cgu DT binding.
 *
 * They are roughly ordered as:
 *   - external clocks
 *   - PLLs
 *   - muxes/dividers in the order they appear in the JZ4775 programmers manual
 *   - gates in order of their bit in the CLKGR register
 */

#ifndef __DT_BINDINGS_CLOCK_JZ4775_CGU_H__
#define __DT_BINDINGS_CLOCK_JZ4775_CGU_H__

#define JZ4775_CLK_EXCLK	0
#define JZ4775_CLK_RTCLK	1
#define JZ4775_CLK_EXCLK_HALF	2
#define JZ4775_CLK_APLL		3
#define JZ4775_CLK_MPLL		4
#define JZ4775_CLK_SCLKA	5
#define JZ4775_CLK_CPUMUX	6
#define JZ4775_CLK_CPU		7
#define JZ4775_CLK_L2CACHE	8
#define JZ4775_CLK_AHB0		9
#define JZ4775_CLK_AHB2PMUX	10
#define JZ4775_CLK_AHB2		11
#define JZ4775_CLK_PCLK		12
#define JZ4775_CLK_DDR		13
#define JZ4775_CLK_VPU		14
#define JZ4775_CLK_OTGPHY_PLL	15
#define JZ4775_CLK_OTGPHY	16
#define JZ4775_CLK_I2S_PLL	17
#define JZ4775_CLK_I2S		18
#define JZ4775_CLK_LCDPIXCLK	19
#define JZ4775_CLK_MSCMUX	20
#define JZ4775_CLK_MSC0		21
#define JZ4775_CLK_MSC1		22
#define JZ4775_CLK_MSC2		23
#define JZ4775_CLK_UHC		24
#define JZ4775_CLK_SSI_PLL	25
#define JZ4775_CLK_SSI		26
#define JZ4775_CLK_CIM_MUX	27
#define JZ4775_CLK_CIM0_MCLK	28
#define JZ4775_CLK_CIM1_MCLK	29
#define JZ4775_CLK_PCM_PLL	30
#define JZ4775_CLK_PCM		31
#define JZ4775_CLK_BCH		32
#define JZ4775_CLK_NEMC		33
#define JZ4775_CLK_OTG		34
#define JZ4775_CLK_SSI0		35
#define JZ4775_CLK_I2C0		36
#define JZ4775_CLK_I2C1		37
#define JZ4775_CLK_I2C2		38
#define JZ4775_CLK_AIC		39
#define JZ4775_CLK_X2D		40
#define JZ4775_CLK_AHB_MON	41
#define JZ4775_CLK_SADC		42
#define JZ4775_CLK_UART0	43
#define JZ4775_CLK_UART1	44
#define JZ4775_CLK_UART2	45
#define JZ4775_CLK_UART3	46
#define JZ4775_CLK_PDMA		47
#define JZ4775_CLK_MAC		48
#define JZ4775_CLK_CIM0		49
#define JZ4775_CLK_CIM1		50
#define JZ4775_CLK_LCD		51
#define JZ4775_CLK_EPDC		52
#define JZ4775_CLK_EPDE		53

#endif /* __DT_BINDINGS_CLOCK_JZ4775_CGU_H__ */
