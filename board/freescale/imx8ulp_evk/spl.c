// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <common.h>
#include <init.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8ulp-pins.h>
#include <fsl_sec.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <asm/arch/ddr.h>
#include <asm/arch/upower.h>
#include <asm/arch/rdc.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/arch/s400_api.h>

DECLARE_GLOBAL_DATA_PTR;

void spl_dram_init(void)
{
	init_clk_ddr();
	ddr_init(&dram_timing);
}

u32 spl_boot_device(void)
{
#ifdef CONFIG_SPL_BOOTROM_SUPPORT
	return BOOT_DEVICE_BOOTROM;
#else
	enum boot_device boot_device_spl = get_boot_device();

	switch (boot_device_spl) {
	case SD1_BOOT:
	case MMC1_BOOT:
	case SD2_BOOT:
	case MMC2_BOOT:
		return BOOT_DEVICE_MMC1;
	case SD3_BOOT:
	case MMC3_BOOT:
		return BOOT_DEVICE_MMC2;
	case QSPI_BOOT:
		return BOOT_DEVICE_NOR;
	case NAND_BOOT:
		return BOOT_DEVICE_NAND;
	case USB_BOOT:
	case USB2_BOOT:
		return BOOT_DEVICE_BOARD;
	default:
		return BOOT_DEVICE_NONE;
	}
#endif
}

#define PMIC_I2C_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_SRE_SLOW | PAD_CTL_ODE)
#define PMIC_MODE_PAD_CTRL	(PAD_CTL_PUS_UP)

static iomux_cfg_t const pmic_pads[] = {
	IMX8ULP_PAD_PTB7__PMIC0_MODE2 | MUX_PAD_CTRL(PMIC_MODE_PAD_CTRL),
	IMX8ULP_PAD_PTB8__PMIC0_MODE1 | MUX_PAD_CTRL(PMIC_MODE_PAD_CTRL),
	IMX8ULP_PAD_PTB9__PMIC0_MODE0 | MUX_PAD_CTRL(PMIC_MODE_PAD_CTRL),
	IMX8ULP_PAD_PTB11__PMIC0_SCL | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
	IMX8ULP_PAD_PTB10__PMIC0_SDA | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
};

void setup_iomux_pmic(void)
{
	imx8ulp_iomux_setup_multiple_pads(pmic_pads, ARRAY_SIZE(pmic_pads));
}

int power_init_board(void)
{
	/* Set buck3 to 1.1v OD */
	upower_pmic_i2c_write(0x22, 0x28);
	return 0;
}

void spl_board_init(void)
{
	struct udevice *dev;
	u32 res;
	int node, ret;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, "fsl,imx8ulp-mu");
	ret = uclass_get_device_by_of_offset(UCLASS_MISC, node, &dev);
	if (ret) {
		return;
	}
	device_probe(dev);

	board_early_init_f();

	preloader_console_init();

	puts("Normal Boot\n");

	/* Set iomuxc0 for pmic when m33 is not booted */
	if (!m33_image_booted())
		setup_iomux_pmic();

	/* Load the lposc fuse for single boot to work around ROM issue,
	*  The fuse depends on S400 to read.
	*/
	if (is_soc_rev(CHIP_REV_1_0) && get_boot_mode() == SINGLE_BOOT)
		load_lposc_fuse();

	upower_init();

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	/* This must place after upower init, so access to MDA and MRC are valid */
	/* Init XRDC MDA  */
	xrdc_init_mda();

	/* Init XRDC MRC for VIDEO, DSP domains */
	xrdc_init_mrc();

	/* Call it after PS16 power up */
	set_lpav_qos();

	/* Asks S400 to release CAAM for A35 core */
	ret = ahab_release_caam(7, &res);
	if (!ret) {

		/* Only two UCLASS_MISC devicese are present on the platform. There
		 * are MU and CAAM. Here we initialize CAAM once it's released by
		 * S400 firmware..
		 */
		node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, "fsl,sec-v4.0");
		ret = uclass_get_device_by_of_offset(UCLASS_MISC, node, &dev);
		if (ret) {
			return;
		}
		device_probe(dev);
	}
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	timer_init();

	arch_cpu_init();

	board_init_r(NULL, 0);
}
