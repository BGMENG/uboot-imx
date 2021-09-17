// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <cpu_func.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <asm/global_data.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/snvs_security_sc.h>
#include <usb.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include "../common/tcpc.h"
#include "command.h"
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))


#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#ifdef CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY
static iomux_cfg_t uart2_pads[] = {
	SC_P_UART0_RTS_B | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART0_CTS_B | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#else
static iomux_cfg_t uart0_pads[] = {
	SC_P_UART0_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART0_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

static void setup_iomux_uart(void)
{
#ifdef CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY
	imx8_iomux_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
#else
	imx8_iomux_setup_multiple_pads(uart0_pads, ARRAY_SIZE(uart0_pads));
#endif
}

int board_early_init_f(void)
{
	sc_pm_clock_rate_t rate = SC_80MHZ;
	int ret;

	/* When start u-boot in XEN VM, directly return */
	if (IS_ENABLED(CONFIG_XEN)) {
		writel(0xF53535F5, (void __iomem *)0x80000000);
		return 0;
	}

#ifdef CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY
	/* Set UART2 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_2, rate);
	if (ret)
		return ret;
#else
	/* Set UART0 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_0, rate);
	if (ret)
		return ret;
#endif	/* CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY */

	setup_iomux_uart();

	return 0;
}


#define LED_0 IMX_GPIO_NR(2, 14)
#define LED_1 IMX_GPIO_NR(2, 15)

#define BB_GPIO_3V3_1 IMX_GPIO_NR(4, 20)
#define BB_GPIO_3V3_2 IMX_GPIO_NR(4, 24)
#define BB_GPIO_3V3_3 IMX_GPIO_NR(4, 23)

static void board_gpio_init(void)
{
#if defined(CONFIG_TARGET_IMX8QM_BES_BGW_0002_A) || defined(CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A53_ONLY)
	int ret;
	struct gpio_desc desc;
	/* LED 0 */
	ret = dm_gpio_lookup_name("GPIO2_14", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_14 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "led_0");
	if (ret) {
		printf("%s request led_0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* LED 1 */
	ret = dm_gpio_lookup_name("GPIO2_15", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_15 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "led_1");
	if (ret) {
		printf("%s request led_1 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* V3_8_PER */
	ret = dm_gpio_lookup_name("GPIO2_25", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_25 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "v3_8_per");
	if (ret) {
		printf("%s request v3_8_per failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* V3_3_PER */
	ret = dm_gpio_lookup_name("GPIO2_26", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_26 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "v3_3_per");
	if (ret) {
		printf("%s request 3v3_per failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* V5_0_PER */
	ret = dm_gpio_lookup_name("GPIO2_24", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_24 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "v5_0_per");
	if (ret) {
		printf("%s request v5_0_per failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
#if 1
	/* WIFI_VIO_EN */
	ret = dm_gpio_lookup_name("GPIO2_27", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_27 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "wifi_vio_en");
	if (ret) {
		printf("%s request wifi_vio_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	/* WIFI_VBAT_EN */
	ret = dm_gpio_lookup_name("GPIO2_29", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_29 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "wifi_vbat_en");
	if (ret) {
		printf("%s request wifi_vbat_en failed ret = %d\n", __func__, ret);
		return;
	}
udelay(1500);
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	/* WIFI_1V8_EN */
	ret = dm_gpio_lookup_name("GPIO2_28", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_28 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "wifi_1v8_en");
	if (ret) {
		printf("%s request wifi_1v8_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
udelay(500);
	/* WIFI_PDn */
	ret = dm_gpio_lookup_name("GPIO2_06", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_06 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "wifi_pdn");
	if (ret) {
		printf("%s request wifi_pdn failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
#endif
	/* enable LVDS SAS boards */
//	ret = dm_gpio_lookup_name("GPIO1_6", &desc);
//	if (ret) {
//		printf("%s lookup GPIO1_6 failed ret = %d\n", __func__, ret);
//		return;
//	}
//
//	ret = dm_gpio_request(&desc, "lvds_enable");
//	if (ret) {
//		printf("%s request lvds_enable failed ret = %d\n", __func__, ret);
//		return;
//	}
//
//	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
//
//	/* enable MIPI SAS boards */
//	ret = dm_gpio_lookup_name("GPIO1_7", &desc);
//	if (ret) {
//		printf("%s lookup GPIO1_7 failed ret = %d\n", __func__, ret);
//		return;
//	}
//
//	ret = dm_gpio_request(&desc, "mipi_enable");
//	if (ret) {
//		printf("%s request mipi_enable failed ret = %d\n", __func__, ret);
//		return;
//	}
//
//	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
#endif

}
int checkboard(void)
{
	puts("Board: iMX8QM BES_BGW_0002_A\n");

	print_bootinfo();

	return 0;
}

int board_init(void)
{
	if (IS_ENABLED(CONFIG_XEN))
		return 0;

	board_gpio_init();


#ifdef CONFIG_IMX_SNVS_SEC_SC_AUTO
	{
		int ret = snvs_security_sc_init();

		if (ret)
			return ret;
	}
#endif

	return 0;
}

void board_quiesce_devices(void)
{
	const char *power_on_devices[] = {
#ifdef CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY
		"dma_lpuart2",
		"PD_UART2_TX",
		"PD_UART2_RX",
#else
		"dma_lpuart0",
#endif
	};

	if (IS_ENABLED(CONFIG_XEN)) {
		/* Clear magic number to let xen know uboot is over */
		writel(0x0, (void __iomem *)0x80000000);
		return;
	}

	imx8_power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(ulong addr)
{
	sc_pm_reboot(-1, SC_PM_RESET_TYPE_COLD);
	while(1);
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}
#endif

int board_mmc_get_env_dev(int devno)
{
	/* Use EMMC */
	if (IS_ENABLED(CONFIG_XEN))
		return 0;

	return devno;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	/* Use EMMC */
	if (IS_ENABLED(CONFIG_XEN))
		return 0;

	return dev_no;
}

extern uint32_t _end_ofs;
int board_late_init(void)
{
	char *fdt_file;
//#if !defined(CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A53_ONLY) && !defined(CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY)
//	bool m4_booted;
//#endif

	build_info();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "BES_BGW_0002_A");
	env_set("board_rev", "iMX8QM");
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

	fdt_file = env_get("fdt_file");

	if (fdt_file && !strcmp(fdt_file, "undefined")) {
//#if defined(CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A53_ONLY)
//		env_set("fdt_file", "imx8qm-BES_BGW_0002_A-cockpit-ca53.dtb");
//#elif defined(CONFIG_TARGET_IMX8QM_BES_BGW_0002_A_A72_ONLY)
//		env_set("fdt_file", "imx8qm-BES_BGW_0002_A-cockpit-ca72.dtb");
//#else
//		m4_booted = m4_parts_booted();
//		if (m4_booted)
//			env_set("fdt_file", "imx8qm-BES_BGW_0002_A-rpmsg.dtb");
//		else
			env_set("fdt_file", "imx8qm-BES_BGW_0002_A.dtb");
//#endif
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#if defined(CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX) || defined(CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX)
	char *end_of_uboot;
	char command[256];
	end_of_uboot = (char *)(ulong)(CONFIG_SYS_TEXT_BASE + _end_ofs + fdt_totalsize(gd->fdt_blob));
	end_of_uboot += 9;

	/* load hdmitxfw.bin and hdmirxfw.bin*/
	memcpy((void *)IMX_HDMI_FIRMWARE_LOAD_ADDR, end_of_uboot,
			IMX_HDMITX_FIRMWARE_SIZE + IMX_HDMIRX_FIRMWARE_SIZE);

#ifdef CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX
	sprintf(command, "hdp load 0x%x", IMX_HDMI_FIRMWARE_LOAD_ADDR);
	run_command(command, 0);
#endif
#ifdef CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX
	sprintf(command, "hdprx load 0x%x",
			IMX_HDMI_FIRMWARE_LOAD_ADDR + IMX_HDMITX_FIRMWARE_SIZE);
	run_command(command, 0);
#endif
#endif /* CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX || CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX */

	return 0;
}

