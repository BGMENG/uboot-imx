/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2007 Nobuhiro Iwamatsu <iwamatsu@nigauri.org>
 *
 * SH7722 Internal I/O register
 */

#ifndef _ASM_CPU_SH7722_H_
#define _ASM_CPU_SH7722_H_

#define CACHE_OC_NUM_WAYS	4
#define CCR_CACHE_INIT	0x0000090d

/*	EXP	*/
#define TRA		0xFF000020
#define EXPEVT		0xFF000024
#define INTEVT		0xFF000028

/*	MMU	*/
#define PTEH		0xFF000000
#define PTEL		0xFF000004
#define TTB		0xFF000008
#define TEA		0xFF00000C
#define MMUCR		0xFF000010
#define PASCR		0xFF000070
#define IRMCR		0xFF000078

/*	CACHE	*/
#define CCR		0xFF00001C
#define RAMCR		0xFF000074

/*	XY MEMORY	*/
#define XSA		0xFF000050
#define YSA		0xFF000054
#define XDA		0xFF000058
#define YDA		0xFF00005C
#define XPR		0xFF000060
#define YPR		0xFF000064
#define XEA		0xFF000068
#define YEA		0xFF00006C

/*	INTC	*/
#define ICR0		0xA4140000
#define ICR1		0xA414001C
#define INTPRI0		0xA4140010
#define INTREQ0		0xA4140024
#define INTMSK0		0xA4140044
#define INTMSKCLR0	0xA4140064
#define NMIFCR		0xA41400C0
#define USERIMASK	0xA4700000
#define IPRA		0xA4080000
#define IPRB		0xA4080004
#define IPRC		0xA4080008
#define IPRD		0xA408000C
#define IPRE		0xA4080010
#define IPRF		0xA4080014
#define IPRG		0xA4080018
#define IPRH		0xA408001C
#define IPRI		0xA4080020
#define IPRJ		0xA4080024
#define IPRK		0xA4080028
#define IPRL		0xA408002C
#define IMR0		0xA4080080
#define IMR1		0xA4080084
#define IMR2		0xA4080088
#define IMR3		0xA408008C
#define IMR4		0xA4080090
#define IMR5		0xA4080094
#define IMR6		0xA4080098
#define IMR7		0xA408009C
#define IMR8		0xA40800A0
#define IMR9		0xA40800A4
#define IMR10		0xA40800A8
#define IMR11		0xA40800AC
#define IMCR0		0xA40800C0
#define IMCR1		0xA40800C4
#define IMCR2		0xA40800C8
#define IMCR3		0xA40800CC
#define IMCR4		0xA40800D0
#define IMCR5		0xA40800D4
#define IMCR6		0xA40800D8
#define IMCR7		0xA40800DC
#define IMCR8		0xA40800E0
#define IMCR9		0xA40800E4
#define IMCR10		0xA40800E8
#define IMCR11		0xA40800EC
#define MFI_IPRA	0xA40B0000
#define MFI_IPRB	0xA40B0004
#define MFI_IPRC	0xA40B0008
#define MFI_IPRD	0xA40B000C
#define MFI_IPRE	0xA40B0010
#define MFI_IPRF	0xA40B0014
#define MFI_IPRG	0xA40B0018
#define MFI_IPRH	0xA40B001C
#define MFI_IPRI	0xA40B0020
#define MFI_IPRJ	0xA40B0024
#define MFI_IPRK	0xA40B0028
#define MFI_IPRL	0xA40B002C
#define MFI_IMR0	0xA40B0080
#define MFI_IMR1	0xA40B0084
#define MFI_IMR2	0xA40B0088
#define MFI_IMR3	0xA40B008C
#define MFI_IMR4	0xA40B0090
#define MFI_IMR5	0xA40B0094
#define MFI_IMR6	0xA40B0098
#define MFI_IMR7	0xA40B009C
#define MFI_IMR8	0xA40B00A0
#define MFI_IMR9	0xA40B00A4
#define MFI_IMR10	0xA40B00A8
#define MFI_IMR11	0xA40B00AC
#define MFI_IMCR0	0xA40B00C0
#define MFI_IMCR1	0xA40B00C4
#define MFI_IMCR2	0xA40B00C8
#define MFI_IMCR3	0xA40B00CC
#define MFI_IMCR4	0xA40B00D0
#define MFI_IMCR5	0xA40B00D4
#define MFI_IMCR6	0xA40B00D8
#define MFI_IMCR7	0xA40B00DC
#define MFI_IMCR8	0xA40B00E0
#define MFI_IMCR9	0xA40B00E4
#define MFI_IMCR10	0xA40B00E8
#define MFI_IMCR11	0xA40B00EC

/*	BSC	*/
#define CMNCR	    0xFEC10000
#define	CS0BCR	    0xFEC10004
#define CS2BCR      0xFEC10008
#define CS4BCR      0xFEC10010
#define CS5ABCR     0xFEC10014
#define CS5BBCR     0xFEC10018
#define CS6ABCR     0xFEC1001C
#define CS6BBCR     0xFEC10020
#define CS0WCR      0xFEC10024
#define CS2WCR      0xFEC10028
#define CS4WCR      0xFEC10030
#define CS5AWCR     0xFEC10034
#define CS5BWCR     0xFEC10038
#define CS6AWCR     0xFEC1003C
#define CS6BWCR     0xFEC10040
#define RBWTCNT     0xFEC10054

/*	SBSC	*/
#define SBSC_SDCR   0xFE400008
#define SBSC_SDWCR  0xFE40000C
#define SBSC_SDPCR  0xFE400010
#define SBSC_RTCSR  0xFE400014
#define SBSC_RTCNT  0xFE400018
#define SBSC_RTCOR  0xFE40001C
#define SBSC_RFCR   0xFE400020

/*	DMAC	*/
#define SAR_0       0xFE008020
#define DAR_0       0xFE008024
#define TCR_0       0xFE008028
#define CHCR_0      0xFE00802C
#define SAR_1       0xFE008030
#define DAR_1       0xFE008034
#define TCR_1       0xFE008038
#define CHCR_1      0xFE00803C
#define SAR_2       0xFE008040
#define DAR_2       0xFE008044
#define TCR_2       0xFE008048
#define CHCR_2      0xFE00804C
#define SAR_3       0xFE008050
#define DAR_3       0xFE008054
#define TCR_3       0xFE008058
#define CHCR_3      0xFE00805C
#define SAR_4       0xFE008070
#define DAR_4       0xFE008074
#define TCR_4       0xFE008078
#define CHCR_4      0xFE00807C
#define SAR_5       0xFE008080
#define DAR_5       0xFE008084
#define TCR_5       0xFE008088
#define CHCR_5      0xFE00808C
#define SARB_0      0xFE008120
#define DARB_0      0xFE008124
#define TCRB_0      0xFE008128
#define SARB_1      0xFE008130
#define DARB_1      0xFE008134
#define TCRB_1      0xFE008138
#define SARB_2      0xFE008140
#define DARB_2      0xFE008144
#define TCRB_2      0xFE008148
#define SARB_3      0xFE008150
#define DARB_3      0xFE008154
#define TCRB_3      0xFE008158
#define DMAOR       0xFE008060
#define DMARS_0     0xFE009000
#define DMARS_1     0xFE009004
#define DMARS_2     0xFE009008

/*	CPG	*/
#define FRQCR       0xA4150000
#define VCLKCR      0xA4150004
#define SCLKACR     0xA4150008
#define SCLKBCR     0xA415000C
#define PLLCR       0xA4150024
#define DLLFRQ      0xA4150050

/*	LOW POWER MODE	*/
#define STBCR       0xA4150020
#define MSTPCR0     0xA4150030
#define MSTPCR1     0xA4150034
#define MSTPCR2     0xA4150038
#define BAR         0xA4150040

/*	RWDT	*/
#define RWTCNT      0xA4520000
#define RWTCSR      0xA4520004
#define WTCNT	RWTCNT


/*	TMU	*/
#define TMU_BASE	0xFFD80000

/*	TPU	*/
#define TPU_TSTR    0xA4C90000
#define TPU_TCR0    0xA4C90010
#define TPU_TMDR0   0xA4C90014
#define TPU_TIOR0   0xA4C90018
#define TPU_TIER0   0xA4C9001C
#define TPU_TSR0    0xA4C90020
#define TPU_TCNT0   0xA4C90024
#define TPU_TGR0A   0xA4C90028
#define TPU_TGR0B   0xA4C9002C
#define TPU_TGR0C   0xA4C90030
#define TPU_TGR0D   0xA4C90034
#define TPU_TCR1    0xA4C90050
#define TPU_TMDR1   0xA4C90054
#define TPU_TIER1   0xA4C9005C
#define TPU_TSR1    0xA4C90060
#define TPU_TCNT1   0xA4C90064
#define TPU_TGR1A   0xA4C90068
#define TPU_TGR1B   0xA4C9006C
#define TPU_TGR1C   0xA4C90070
#define TPU_TGR1D   0xA4C90074
#define TPU_TCR2    0xA4C90090
#define TPU_TMDR2   0xA4C90094
#define TPU_TIER2   0xA4C9009C
#define TPU_TSR2    0xA4C900A0
#define TPU_TCNT2   0xA4C900A4
#define TPU_TGR2A   0xA4C900A8
#define TPU_TGR2B   0xA4C900AC
#define TPU_TGR2C   0xA4C900B0
#define TPU_TGR2D   0xA4C900B4
#define TPU_TCR3    0xA4C900D0
#define TPU_TMDR3   0xA4C900D4
#define TPU_TIER3   0xA4C900DC
#define TPU_TSR3    0xA4C900E0
#define TPU_TCNT3   0xA4C900E4
#define TPU_TGR3A   0xA4C900E8
#define TPU_TGR3B   0xA4C900EC
#define TPU_TGR3C   0xA4C900F0
#define TPU_TGR3D   0xA4C900F4

/*	CMT	*/
#define CMSTR       0xA44A0000
#define CMCSR       0xA44A0060
#define CMCNT       0xA44A0064
#define CMCOR       0xA44A0068

/*	SIO	*/
#define SIOMDR      0xA4500000
#define SIOCTR      0xA4500004
#define SIOSTBCR0   0xA4500008
#define SIOSTBCR1   0xA450000C
#define SIOTDR      0xA4500014
#define SIORDR      0xA4500018
#define SIOSTR      0xA450001C
#define SIOIER      0xA4500020
#define SIOSCR      0xA4500024

/*	SIOF	*/
#define SIMDR0      0xA4410000
#define SISCR0      0xA4410002
#define SITDAR0     0xA4410004
#define SIRDAR0     0xA4410006
#define SICDAR0     0xA4410008
#define SICTR0      0xA441000C
#define SIFCTR0     0xA4410010
#define SISTR0      0xA4410014
#define SIIER0      0xA4410016
#define SITDR0      0xA4410020
#define SIRDR0      0xA4410024
#define SITCR0      0xA4410028
#define SIRCR0      0xA441002C
#define SPICR0      0xA4410030
#define SIMDR1      0xA4420000
#define SISCR1      0xA4420002
#define SITDAR1     0xA4420004
#define SIRDAR1     0xA4420006
#define SICDAR1     0xA4420008
#define SICTR1      0xA442000C
#define SIFCTR1     0xA4420010
#define SISTR1      0xA4420014
#define SIIER1      0xA4420016
#define SITDR1      0xA4420020
#define SIRDR1      0xA4420024
#define SITCR1      0xA4420028
#define SIRCR1      0xA442002C
#define SPICR1      0xA4420030

/*	SCIF	*/
#define SCIF0_BASE  0xFFE00000

/*	SIM	*/
#define SIM_SCSMR       0xA4490000
#define SIM_SCBRR       0xA4490002
#define SIM_SCSCR       0xA4490004
#define SIM_SCTDR       0xA4490006
#define SIM_SCSSR       0xA4490008
#define SIM_SCRDR       0xA449000A
#define SIM_SCSCMR      0xA449000C
#define SIM_SCSC2R      0xA449000E
#define SIM_SCWAIT      0xA4490010
#define SIM_SCGRD       0xA4490012
#define SIM_SCSMPL      0xA4490014
#define SIM_SCDMAEN     0xA4490016

/*	IrDA	*/
#define IRIF_INIT1      0xA45D0012
#define IRIF_INIT2      0xA45D0014
#define IRIF_RINTCLR    0xA45D0016
#define IRIF_TINTCLR    0xA45D0018
#define IRIF_SIR0       0xA45D0020
#define IRIF_SIR1       0xA45D0022
#define IRIF_SIR2       0xA45D0024
#define IRIF_SIR3       0xA45D0026
#define IRIF_SIR_FRM    0xA45D0028
#define IRIF_SIR_EOF    0xA45D002A
#define IRIF_SIR_FLG    0xA45D002C
#define IRIF_SIR_STS2   0xA45D002E
#define IRIF_UART0      0xA45D0030
#define IRIF_UART1      0xA45D0032
#define IRIF_UART2      0xA45D0034
#define IRIF_UART3      0xA45D0036
#define IRIF_UART4      0xA45D0038
#define IRIF_UART5      0xA45D003A
#define IRIF_UART6      0xA45D003C
#define IRIF_UART7      0xA45D003E
#define IRIF_CRC0       0xA45D0040
#define IRIF_CRC1       0xA45D0042
#define IRIF_CRC2       0xA45D0044
#define IRIF_CRC3       0xA45D0046
#define IRIF_CRC4       0xA45D0048

/*	IIC	*/
#define ICDR0       0xA4470000
#define ICCR0       0xA4470004
#define ICSR0       0xA4470008
#define ICIC0       0xA447000C
#define ICCL0       0xA4470010
#define ICCH0       0xA4470014
#define ICDR1       0xA4750000
#define ICCR1       0xA4750004
#define ICSR1       0xA4750008
#define ICIC1       0xA475000C
#define ICCL1       0xA4750010
#define ICCH1       0xA4750014

/*	FLCTL	*/
#define FLCMNCR     0xA4530000
#define FLCMDCR     0xA4530004
#define FLCMCDR     0xA4530008
#define FLADR       0xA453000C
#define FLDATAR     0xA4530010
#define FLDTCNTR    0xA4530014
#define FLINTDMACR  0xA4530018
#define FLBSYTMR    0xA453001C
#define FLBSYCNT    0xA4530020
#define FLDTFIFO    0xA4530024
#define FLECFIFO    0xA4530028
#define FLTRCR      0xA453002C
#define FLADR2      0xA453003C

/*	MFI	*/
#define MFIIDX      0xA4C10000
#define MFIGSR      0xA4C10004
#define MFISCR      0xA4C10008
#define MFIMCR      0xA4C1000C
#define MFIIICR     0xA4C10010
#define MFIEICR     0xA4C10014
#define MFIADR      0xA4C10018
#define MFIDATA     0xA4C1001C
#define MFIRCR      0xA4C10020
#define MFIINTEVT   0xA4C1002C
#define MFIIMASK    0xA4C10030
#define MFIBCR      0xA4C10040
#define MFIADRW     0xA4C10044
#define MFIADRR     0xA4C10048
#define MFIDATAW    0xA4C1004C
#define MFIDATAR    0xA4C10050
#define MFIMCRW     0xA4C10054
#define MFIMCRR     0xA4C10058
#define MFIDNRW     0xA4C1005C
#define MFIDNRR     0xA4C10060
#define MFISIZEW    0xA4C10064
#define MFISIZER    0xA4C10068
#define MFIDEVCR    0xA4C10038
#define MFISM4      0xA4C10080

/*	VPU	*/
#define VP4_CTRL        0xFE900000
#define VP4_VOL_CTRL    0xFE900004
#define VP4_IMAGE_SIZE  0xFE900008
#define VP4_MB_NUM      0xFE90000C
#define VP4_DWY_ADDR    0xFE900010
#define VP4_DWC_ADDR    0xFE900014
#define VP4_D2WY_ADDR   0xFE900018
#define VP4_D2WC_ADDR   0xFE90001C
#define VP4_DP1_ADDR    0xFE900020
#define VP4_DP2_ADDR    0xFE900024
#define VP4_STRS_ADDR   0xFE900028
#define VP4_STRE_ADDR   0xFE90002C
#define VP4_VOP_CTRL    0xFE900030
#define VP4_VOP_TIME    0xFE900034
#define VP4_263_CTRL    0xFE900038
#define VP4_264_CTRL    0xFE90003C
#define VP4_VLC_CTRL    0xFE900040
#define VP4_ENDIAN      0xFE900044
#define VP4_CMD         0xFE900048
#define VP4_ME_TH1      0xFE90004C
#define VP4_ME_TH2      0xFE900050
#define VP4_ME_COSTMB   0xFE900054
#define VP4_ME_SKIP     0xFE900058
#define VP4_ME_CTRL     0xFE90005C
#define VP4_MBRF_CTRL   0xFE900060
#define VP4_MC_CTRL     0xFE900064
#define VP4_PRED_CTRL   0xFE900068
#define VP4_SLC_SIZE    0xFE90006C
#define VP4_VOP_MINBIT  0xFE900070
#define VP4_MB_MAXBIT   0xFE900074
#define VP4_MB_TBIT     0xFE900078
#define VP4_RCQNT       0xFE90007C
#define VP4_RCRP        0xFE900080
#define VP4_RCDJ        0xFE900084
#define VP4_RCWQ        0xFE900088
#define VP4_FWD_TIME    0xFE900094
#define VP4_BWD_TIME    0xFE900098
#define VP4_PST_TIME    0xFE90009C
#define VP4_ILTFRAME    0xFE9000A0
#define VP4_EC_REF      0xFE9000A4
#define VP4_STATUS      0xFE900100
#define VP4_IRQ_ENB     0xFE900104
#define VP4_IRQ_STA     0xFE900108
#define VP4_VOP_BIT     0xFE90010C
#define VP4_PRV_BIT     0xFE900110
#define VP4_SLC_MB      0xFE900114
#define VP4_QSUM        0xFE900118
#define VP4_DEC_ERR     0xFE90011C
#define VP4_ERR_AREA    0xFE900120
#define VP4_NEXT_CODE   0xFE900124
#define VP4_MB_ATTR     0xFE900128
#define VP4_DBMON       0xFE90012C
#define VP4_DEBUG       0xFE900130
#define VP4_ERR_DET     0xFE900134
#define VP4_CLK_STOP    0xFE900138
#define VP4_MB_SADA     0xFE90013C
#define VP4_MB_SADR     0xFE900140
#define VP4_MAT_RAM     0xFE901000
#define VP4_NC_RAM      0xFE902000
#define WT              0xFE9020CC
#define VP4_CPY_ADDR    0xFE902264
#define VP4_CPC_ADDR    0xFE902268
#define VP4_R0Y_ADDR    0xFE90226C
#define VP4_R0C_ADDR    0xFE902270
#define VP4_R1Y_ADDR    0xFE902274
#define VP4_R1C_ADDR    0xFE902278
#define VP4_R2Y_ADDR    0xFE90227C
#define VP4_R2C_ADDR    0xFE902280
#define VP4_R3Y_ADDR    0xFE902284
#define VP4_R3C_ADDR    0xFE902288
#define VP4_R4Y_ADDR    0xFE90228C
#define VP4_R4C_ADDR    0xFE902290
#define VP4_R5Y_ADDR    0xFE902294
#define VP4_R5C_ADDR    0xFE902298
#define VP4_R6Y_ADDR    0xFE90229C
#define VP4_R6C_ADDR    0xFE9022A0
#define VP4_R7Y_ADDR    0xFE9022A4
#define VP4_R7C_ADDR    0xFE9022A8
#define VP4_R8Y_ADDR    0xFE9022AC
#define VP4_R8C_ADDR    0xFE9022B0
#define VP4_R9Y_ADDR    0xFE9022B4
#define VP4_R9C_ADDR    0xFE9022B8
#define VP4_RAY_ADDR    0xFE9022BC
#define VP4_RAC_ADDR    0xFE9022C0
#define VP4_RBY_ADDR    0xFE9022C4
#define VP4_RBC_ADDR    0xFE9022C8
#define VP4_RCY_ADDR    0xFE9022CC
#define VP4_RCC_ADDR    0xFE9022D0
#define VP4_RDY_ADDR    0xFE9022D4
#define VP4_RDC_ADDR    0xFE9022D8
#define VP4_REY_ADDR    0xFE9022DC
#define VP4_REC_ADDR    0xFE9022E0
#define VP4_RFY_ADDR    0xFE9022E4
#define VP4_RFC_ADDR    0xFE9022E8

/*	VIO(CEU)	*/
#define CAPSR       0xFE910000
#define CAPCR       0xFE910004
#define CAMCR       0xFE910008
#define CMCYR       0xFE91000C
#define CAMOR       0xFE910010
#define CAPWR       0xFE910014
#define CAIFR       0xFE910018
#define CSTCR       0xFE910020
#define CSECR       0xFE910024
#define CRCNTR      0xFE910028
#define CRCMPR      0xFE91002C
#define CFLCR       0xFE910030
#define CFSZR       0xFE910034
#define CDWDR       0xFE910038
#define CDAYR       0xFE91003C
#define CDACR       0xFE910040
#define CDBYR       0xFE910044
#define CDBCR       0xFE910048
#define CBDSR       0xFE91004C
#define CLFCR       0xFE910060
#define CDOCR       0xFE910064
#define CDDCR       0xFE910068
#define CDDAR       0xFE91006C
#define CEIER       0xFE910070
#define CETCR       0xFE910074
#define CSTSR       0xFE91007C
#define CSRTR       0xFE910080
#define CDAYR2      0xFE910090
#define CDACR2      0xFE910094
#define CDBYR2      0xFE910098
#define CDBCR2      0xFE91009C

/*	VIO(VEU)	*/
#define VESTR       0xFE920000
#define VESWR       0xFE920010
#define VESSR       0xFE920014
#define VSAYR       0xFE920018
#define VSACR       0xFE92001C
#define VBSSR       0xFE920020
#define VEDWR       0xFE920030
#define VDAYR       0xFE920034
#define VDACR       0xFE920038
#define VTRCR       0xFE920050
#define VRFCR       0xFE920054
#define VRFSR       0xFE920058
#define VENHR       0xFE92005C
#define VFMCR       0xFE920070
#define VVTCR       0xFE920074
#define VHTCR       0xFE920078
#define VAPCR       0xFE920080
#define VECCR       0xFE920084
#define VAFXR       0xFE920090
#define VSWPR       0xFE920094
#define VEIER       0xFE9200A0
#define VEVTR       0xFE9200A4
#define VSTAR       0xFE9200B0
#define VBSRR       0xFE9200B4

/*	VIO(BEU)	*/
#define BESTR       0xFE930000
#define BSMWR1      0xFE930010
#define BSSZR1      0xFE930014
#define BSAYR1      0xFE930018
#define BSACR1      0xFE93001C
#define BSAAR1      0xFE930020
#define BSIFR1      0xFE930024
#define BSMWR2      0xFE930028
#define BSSZR2      0xFE93002C
#define BSAYR2      0xFE930030
#define BSACR2      0xFE930034
#define BSAAR2      0xFE930038
#define BSIFR2      0xFE93003C
#define BSMWR3      0xFE930040
#define BSSZR3      0xFE930044
#define BSAYR3      0xFE930048
#define BSACR3      0xFE93004C
#define BSAAR3      0xFE930050
#define BSIFR3      0xFE930054
#define BTPSR       0xFE930058
#define BMSMWR1     0xFE930070
#define BMSSZR1     0xFE930074
#define BMSAYR1     0xFE930078
#define BMSACR1     0xFE93007C
#define BMSMWR2     0xFE930080
#define BMSSZR2     0xFE930084
#define BMSAYR2     0xFE930088
#define BMSACR2     0xFE93008C
#define BMSMWR3     0xFE930090
#define BMSSZR3     0xFE930094
#define BMSAYR3     0xFE930098
#define BMSACR3     0xFE93009C
#define BMSMWR4     0xFE9300A0
#define BMSSZR4     0xFE9300A4
#define BMSAYR4     0xFE9300A8
#define BMSACR4     0xFE9300AC
#define BMSIFR      0xFE9300F0
#define BBLCR0      0xFE930100
#define BBLCR1      0xFE930104
#define BPROCR      0xFE930108
#define BMWCR0      0xFE93010C
#define BLOCR1      0xFE930114
#define BLOCR2      0xFE930118
#define BLOCR3      0xFE93011C
#define BMLOCR1     0xFE930120
#define BMLOCR2     0xFE930124
#define BMLOCR3     0xFE930128
#define BMLOCR4     0xFE93012C
#define BMPCCR1     0xFE930130
#define BMPCCR2     0xFE930134
#define BPKFR       0xFE930140
#define BPCCR0      0xFE930144
#define BPCCR11     0xFE930148
#define BPCCR12     0xFE93014C
#define BPCCR21     0xFE930150
#define BPCCR22     0xFE930154
#define BPCCR31     0xFE930158
#define BPCCR32     0xFE93015C
#define BDMWR       0xFE930160
#define BDAYR       0xFE930164
#define BDACR       0xFE930168
#define BAFXR       0xFE930180
#define BSWPR       0xFE930184
#define BEIER       0xFE930188
#define BEVTR       0xFE93018C
#define BRCNTR      0xFE930194
#define BSTAR       0xFE930198
#define BBRSTR      0xFE93019C
#define BRCHR       0xFE9301A0
#define CLUT        0xFE933000

/*	JPU	*/
#define JCMOD       0xFEA00000
#define JCCMD       0xFEA00004
#define JCSTS       0xFEA00008
#define JCQTN       0xFEA0000C
#define JCHTN       0xFEA00010
#define JCDRIU      0xFEA00014
#define JCDRID      0xFEA00018
#define JCVSZU      0xFEA0001C
#define JCVSZD      0xFEA00020
#define JCHSZU      0xFEA00024
#define JCHSZD      0xFEA00028
#define JCDTCU      0xFEA0002C
#define JCDTCM      0xFEA00030
#define JCDTCD      0xFEA00034
#define JINTE       0xFEA00038
#define JINTS       0xFEA0003C
#define JCDERR      0xFEA00040
#define JCRST       0xFEA00044
#define JIFCNT      0xFEA00060
#define JIFECNT     0xFEA00070
#define JIFESYA1    0xFEA00074
#define JIFESCA1    0xFEA00078
#define JIFESYA2    0xFEA0007C
#define JIFESCA2    0xFEA00080
#define JIFESMW     0xFEA00084
#define JIFESVSZ    0xFEA00088
#define JIFESHSZ    0xFEA0008C
#define JIFEDA1     0xFEA00090
#define JIFEDA2     0xFEA00094
#define JIFEDRSZ    0xFEA00098
#define JIFDCNT     0xFEA000A0
#define JIFDSA1     0xFEA000A4
#define JIFDSA2     0xFEA000A8
#define JIFDDRSZ    0xFEA000AC
#define JIFDDMW     0xFEA000B0
#define JIFDDVSZ    0xFEA000B4
#define JIFDDHSZ    0xFEA000B8
#define JIFDDYA1    0xFEA000BC
#define JIFDDCA1    0xFEA000C0
#define JIFDDYA2    0xFEA000C4
#define JIFDDCA2    0xFEA000C8
#define JCQTBL0     0xFEA10000
#define JCQTBL1     0xFEA10040
#define JCQTBL2     0xFEA10080
#define JCQTBL3     0xFEA100C0
#define JCHTBD0     0xFEA10100
#define JCHTBA0     0xFEA10120
#define JCHTBD1     0xFEA10200
#define JCHTBA1     0xFEA10220

/*	LCDC	*/
#define MLDDCKPAT1R 0xFE940400
#define MLDDCKPAT2R 0xFE940404
#define SLDDCKPAT1R 0xFE940408
#define SLDDCKPAT2R 0xFE94040C
#define LDDCKR      0xFE940410
#define LDDCKSTPR   0xFE940414
#define MLDMT1R     0xFE940418
#define MLDMT2R     0xFE94041C
#define MLDMT3R     0xFE940420
#define MLDDFR      0xFE940424
#define MLDSM1R     0xFE940428
#define MLDSM2R     0xFE94042C
#define MLDSA1R     0xFE940430
#define MLDSA2R     0xFE940434
#define MLDMLSR     0xFE940438
#define MLDWBFR     0xFE94043C
#define MLDWBCNTR   0xFE940440
#define MLDWBAR     0xFE940444
#define MLDHCNR     0xFE940448
#define MLDHSYNR    0xFE94044C
#define MLDVLNR     0xFE940450
#define MLDVSYNR    0xFE940454
#define MLDHPDR     0xFE940458
#define MLDVPDR     0xFE94045C
#define MLDPMR      0xFE940460
#define LDPALCR     0xFE940464
#define LDINTR      0xFE940468
#define LDSR        0xFE94046C
#define LDCNT1R     0xFE940470
#define LDCNT2R     0xFE940474
#define LDRCNTR     0xFE940478
#define LDDDSR      0xFE94047C
#define LDRCR       0xFE940484
#define LDCMRKRGBR  0xFE9404C4
#define LDCMRKCMYR  0xFE9404C8
#define LDCMRK1R    0xFE9404CC
#define LDCMRK2R    0xFE9404D0
#define LDCMGKRGBR  0xFE9404D4
#define LDCMGKCMYR  0xFE9404D8
#define LDCMGK1R    0xFE9404DC
#define LDCMGK2R    0xFE9404E0
#define LDCMBKRGBR  0xFE9404E4
#define LDCMBKCMYR  0xFE9404E8
#define LDCMBK1R    0xFE9404EC
#define LDCMBK2R    0xFE9404F0
#define LDCMHKPR    0xFE9404F4
#define LDCMHKQR    0xFE9404F8
#define LDCMSELR    0xFE9404FC
#define LDCMTVR     0xFE940500
#define LDCMTVSELR  0xFE940504
#define LDCMDTHR    0xFE940508
#define LDCMCNTR    0xFE94050C
#define SLDMT1R     0xFE940600
#define SLDMT2R     0xFE940604
#define SLDMT3R     0xFE940608
#define SLDDFR      0xFE94060C
#define SLDSM1R     0xFE940610
#define SLDSM2R     0xFE940614
#define SLDSA1R     0xFE940618
#define SLDSA2R     0xFE94061C
#define SLDMLSR     0xFE940620
#define SLDHCNR     0xFE940624
#define SLDHSYNR    0xFE940628
#define SLDVLNR     0xFE94062C
#define SLDVSYNR    0xFE940630
#define SLDHPDR     0xFE940634
#define SLDVPDR     0xFE940638
#define SLDPMR      0xFE94063C
#define LDDWD0R     0xFE940800
#define LDDWD1R     0xFE940804
#define LDDWD2R     0xFE940808
#define LDDWD3R     0xFE94080C
#define LDDWD4R     0xFE940810
#define LDDWD5R     0xFE940814
#define LDDWD6R     0xFE940818
#define LDDWD7R     0xFE94081C
#define LDDWD8R     0xFE940820
#define LDDWD9R     0xFE940824
#define LDDWDAR     0xFE940828
#define LDDWDBR     0xFE94082C
#define LDDWDCR     0xFE940830
#define LDDWDDR     0xFE940834
#define LDDWDER     0xFE940838
#define LDDWDFR     0xFE94083C
#define LDDRDR      0xFE940840
#define LDDWAR      0xFE940900
#define LDDRAR      0xFE940904
#define LDPR00      0xFE940000

/*	VOU	*/
#define VOUER       0xFE960000
#define VOUCR       0xFE960004
#define VOUSTR      0xFE960008
#define VOUVCR      0xFE96000C
#define VOUISR      0xFE960010
#define VOUBCR      0xFE960014
#define VOUDPR      0xFE960018
#define VOUDSR      0xFE96001C
#define VOUVPR      0xFE960020
#define VOUIR       0xFE960024
#define VOUSRR      0xFE960028
#define VOUMSR      0xFE96002C
#define VOUHIR      0xFE960030
#define VOUDFR      0xFE960034
#define VOUAD1R     0xFE960038
#define VOUAD2R     0xFE96003C
#define VOUAIR      0xFE960040
#define VOUSWR      0xFE960044
#define VOURCR      0xFE960048
#define VOURPR      0xFE960050

/*	TSIF	*/
#define TSCTLR      0xA4C80000
#define TSPIDR      0xA4C80004
#define TSCMDR      0xA4C80008
#define TSSTR       0xA4C8000C
#define TSTSDR      0xA4C80010
#define TSBUFCLRR   0xA4C80014
#define TSINTER     0xA4C80018
#define TSPSCALER   0xA4C80020
#define TSPSCALERR  0xA4C80024
#define TSPCRADCMDR 0xA4C80028
#define TSPCRADCR   0xA4C8002C
#define TSTRPCRADCR 0xA4C80030
#define TSDPCRADCR  0xA4C80034

/*	SIU	*/
#define IFCTL       0xA454C000
#define SRCTL       0xA454C004
#define SFORM       0xA454C008
#define CKCTL       0xA454C00C
#define TRDAT       0xA454C010
#define STFIFO      0xA454C014
#define DPAK        0xA454C01C
#define CKREV       0xA454C020
#define EVNTC       0xA454C028
#define SBCTL       0xA454C040
#define SBPSET      0xA454C044
#define SBBUS       0xA454C048
#define SBWFLG      0xA454C058
#define SBRFLG      0xA454C05C
#define SBWDAT      0xA454C060
#define SBRDAT      0xA454C064
#define SBFSTS      0xA454C068
#define SBDVCA      0xA454C06C
#define SBDVCB      0xA454C070
#define SBACTIV     0xA454C074
#define DMAIA       0xA454C090
#define DMAIB       0xA454C094
#define DMAOA       0xA454C098
#define DMAOB       0xA454C09C
#define SPLRI       0xA454C0B8
#define SPRRI       0xA454C0BC
#define SPURI       0xA454C0C4
#define SPTIS       0xA454C0C8
#define SPSTS       0xA454C0CC
#define SPCTL       0xA454C0D0
#define SPIRI       0xA454C0D4
#define SPQCF       0xA454C0D8
#define SPQCS       0xA454C0DC
#define SPQCT       0xA454C0E0
#define DPEAK       0xA454C0F0
#define DSLPD       0xA454C0F4
#define DSLLV       0xA454C0F8
#define BRGASEL     0xA454C100
#define BRRA        0xA454C104
#define BRGBSEL     0xA454C108
#define BRRB        0xA454C10C

/*	USB	*/
#define IFR0        0xA4480000
#define ISR0        0xA4480010
#define IER0        0xA4480020
#define EPDR0I      0xA4480030
#define EPDR0O      0xA4480034
#define EPDR0S      0xA4480038
#define EPDR1       0xA448003C
#define EPDR2       0xA4480040
#define EPDR3       0xA4480044
#define EPDR4       0xA4480048
#define EPDR5       0xA448004C
#define EPDR6       0xA4480050
#define EPDR7       0xA4480054
#define EPDR8       0xA4480058
#define EPDR9       0xA448005C
#define EPSZ0O      0xA4480080
#define EPSZ3       0xA4480084
#define EPSZ6       0xA4480088
#define EPSZ9       0xA448008C
#define TRG         0xA44800A0
#define DASTS       0xA44800A4
#define FCLR        0xA44800AA
#define DMA         0xA44800AC
#define EPSTL       0xA44800B2
#define CVR         0xA44800B4
#define TSR         0xA44800B8
#define CTLR        0xA44800BC
#define EPIR        0xA44800C0
#define XVERCR      0xA44800D0
#define STLMR       0xA44800D4

/*	KEYSC	*/
#define KYCR1       0xA44B0000
#define KYCR2       0xA44B0004
#define KYINDR      0xA44B0008
#define KYOUTDR     0xA44B000C

/*	MMCIF	*/
#define CMDR0       0xA4448000
#define CMDR1       0xA4448001
#define CMDR2       0xA4448002
#define CMDR3       0xA4448003
#define CMDR4       0xA4448004
#define CMDR5       0xA4448005
#define CMDSTRT     0xA4448006
#define OPCR        0xA444800A
#define CSTR        0xA444800B
#define INTCR0      0xA444800C
#define INTCR1      0xA444800D
#define INTSTR0     0xA444800E
#define INTSTR1     0xA444800F
#define CLKON       0xA4448010
#define CTOCR       0xA4448011
#define VDCNT       0xA4448012
#define TBCR        0xA4448014
#define MODER       0xA4448016
#define CMDTYR      0xA4448018
#define RSPTYR      0xA4448019
#define TBNCR       0xA444801A
#define RSPR0       0xA4448020
#define RSPR1       0xA4448021
#define RSPR2       0xA4448022
#define RSPR3       0xA4448023
#define RSPR4       0xA4448024
#define RSPR5       0xA4448025
#define RSPR6       0xA4448026
#define RSPR7       0xA4448027
#define RSPR8       0xA4448028
#define RSPR9       0xA4448029
#define RSPR10      0xA444802A
#define RSPR11      0xA444802B
#define RSPR12      0xA444802C
#define RSPR13      0xA444802D
#define RSPR14      0xA444802E
#define RSPR15      0xA444802F
#define RSPR16      0xA4448030
#define RSPRD       0xA4448031
#define DTOUTR      0xA4448032
#define DR          0xA4448040
#define FIFOCLR     0xA4448042
#define DMACR       0xA4448044
#define INTCR2      0xA4448046
#define INTSTR2     0xA4448048

/*	Z3D3	*/
#define DLBI        0xFD980000
#define DLBD0       0xFD980080
#define DLBD1       0xFD980100
#define GEWM        0xFD984000
#define ICD0        0xFD988000
#define ICD1        0xFD989000
#define ICT         0xFD98A000
#define ILM         0xFD98C000
#define FLM0        0xFD98C800
#define FLM1        0xFD98D000
#define FLUT        0xFD98D800
#define Z3D_PC      0xFD98E400
#define Z3D_PCSP    0xFD98E404
#define Z3D_PAR     0xFD98E408
#define Z3D_IMADR   0xFD98E40C
#define Z3D_BTR0    0xFD98E410
#define Z3D_BTR1    0xFD98E414
#define Z3D_BTR2    0xFD98E418
#define Z3D_BTR3    0xFD98E41C
#define Z3D_LC0     0xFD98E420
#define Z3D_LC1     0xFD98E424
#define Z3D_LC2     0xFD98E428
#define Z3D_LC3     0xFD98E42C
#define Z3D_FR0     0xFD98E430
#define Z3D_FR1     0xFD98E434
#define Z3D_FR2     0xFD98E438
#define Z3D_SR      0xFD98E440
#define Z3D_SMDR    0xFD98E444
#define Z3D_PBIR    0xFD98E448
#define Z3D_DMDR    0xFD98E44C
#define Z3D_IREG    0xFD98E460
#define Z3D_AR00    0xFD98E480
#define Z3D_AR01    0xFD98E484
#define Z3D_AR02    0xFD98E488
#define Z3D_AR03    0xFD98E48C
#define Z3D_BR00    0xFD98E490
#define Z3D_BR01    0xFD98E494
#define Z3D_IXR00   0xFD98E4A0
#define Z3D_IXR01   0xFD98E4A4
#define Z3D_IXR02   0xFD98E4A8
#define Z3D_IXR03   0xFD98E4AC
#define Z3D_AR10    0xFD98E4C0
#define Z3D_AR11    0xFD98E4C4
#define Z3D_AR12    0xFD98E4C8
#define Z3D_AR13    0xFD98E4CC
#define Z3D_BR10    0xFD98E4D0
#define Z3D_BR11    0xFD98E4D4
#define Z3D_IXR10   0xFD98E4E0
#define Z3D_IXR11   0xFD98E4E4
#define Z3D_IXR12   0xFD98E4E8
#define Z3D_IXR13   0xFD98E4EC
#define Z3D_AR20    0xFD98E500
#define Z3D_AR21    0xFD98E504
#define Z3D_AR22    0xFD98E508
#define Z3D_AR23    0xFD98E50C
#define Z3D_BR20    0xFD98E510
#define Z3D_BR21    0xFD98E514
#define Z3D_IXR20   0xFD98E520
#define Z3D_IXR21   0xFD98E524
#define Z3D_IXR22   0xFD98E528
#define Z3D_IXR23   0xFD98E52C
#define Z3D_MR0     0xFD98E540
#define Z3D_MR1     0xFD98E544
#define Z3D_MR2     0xFD98E548
#define Z3D_MR3     0xFD98E54C
#define Z3D_WORKRST 0xFD98E558
#define Z3D_WORKWST 0xFD98E55C
#define Z3D_DBADR   0xFD98E560
#define Z3D_DLBPRST 0xFD98E564
#define Z3D_DLBRST  0xFD98E568
#define Z3D_DLBWST  0xFD98E56C
#define Z3D_UDR0    0xFD98E570
#define Z3D_UDR1    0xFD98E574
#define Z3D_UDR2    0xFD98E578
#define Z3D_UDR3    0xFD98E57C
#define Z3D_CCR0    0xFD98E580
#define Z3D_CCR1    0xFD98E584
#define Z3D_EXPR    0xFD98E588
#define Z3D_V0_X    0xFD9A0000
#define Z3D_V0_Y    0xFD9A0004
#define Z3D_V0_Z    0xFD9A0008
#define Z3D_V0_W    0xFD9A000C
#define Z3D_V0_A    0xFD9A0010
#define Z3D_V0_R    0xFD9A0014
#define Z3D_V0_G    0xFD9A0018
#define Z3D_V0_B    0xFD9A001C
#define Z3D_V0_F    0xFD9A0020
#define Z3D_V0_SR   0xFD9A0024
#define Z3D_V0_SG   0xFD9A0028
#define Z3D_V0_SB   0xFD9A002C
#define Z3D_V0_U0   0xFD9A0030
#define Z3D_V0_V0   0xFD9A0034
#define Z3D_V0_U1   0xFD9A0038
#define Z3D_V0_V1   0xFD9A003C
#define Z3D_V1_X    0xFD9A0080
#define Z3D_V1_Y    0xFD9A0084
#define Z3D_V1_Z    0xFD9A0088
#define Z3D_V1_W    0xFD9A008C
#define Z3D_V1_A    0xFD9A0090
#define Z3D_V1_R    0xFD9A0094
#define Z3D_V1_G    0xFD9A0098
#define Z3D_V1_B    0xFD9A009C
#define Z3D_V1_F    0xFD9A00A0
#define Z3D_V1_SR   0xFD9A00A4
#define Z3D_V1_SG   0xFD9A00A8
#define Z3D_V1_SB   0xFD9A00AC
#define Z3D_V1_U0   0xFD9A00B0
#define Z3D_V1_V0   0xFD9A00B4
#define Z3D_V1_U1   0xFD9A00B8
#define Z3D_V1_V1   0xFD9A00BC
#define Z3D_V2_X    0xFD9A0100
#define Z3D_V2_Y    0xFD9A0104
#define Z3D_V2_Z    0xFD9A0108
#define Z3D_V2_W    0xFD9A010C
#define Z3D_V2_A    0xFD9A0110
#define Z3D_V2_R    0xFD9A0114
#define Z3D_V2_G    0xFD9A0118
#define Z3D_V2_B    0xFD9A011C
#define Z3D_V2_F    0xFD9A0120
#define Z3D_V2_SR   0xFD9A0124
#define Z3D_V2_SG   0xFD9A0128
#define Z3D_V2_SB   0xFD9A012C
#define Z3D_V2_U0   0xFD9A0130
#define Z3D_V2_V0   0xFD9A0134
#define Z3D_V2_U1   0xFD9A0138
#define Z3D_V2_V1   0xFD9A013C
#define Z3D_RENDER              0xFD9A0180
#define Z3D_POLYGON_OFFSET      0xFD9A0184
#define Z3D_VERTEX_CONTROL      0xFD9A0200
#define Z3D_STATE_MODE          0xFD9A0204
#define Z3D_FPU_MODE            0xFD9A0318
#define Z3D_SCISSOR_MIN         0xFD9A0400
#define Z3D_SCISSOR_MAX         0xFD9A0404
#define Z3D_TEXTURE_MODE_A      0xFD9A0408
#define Z3D_TEXTURE_MODE_B      0xFD9A040C
#define Z3D_TEXTURE_BASE_HI_A   0xFD9A0418
#define Z3D_TEXTURE_BASE_LO_A   0xFD9A041C
#define Z3D_TEXTURE_BASE_HI_B   0xFD9A0420
#define Z3D_TEXTURE_BASE_LO_B   0xFD9A0424
#define Z3D_TEXTURE_ALPHA_A0    0xFD9A0438
#define Z3D_TEXTURE_ALPHA_A1    0xFD9A043C
#define Z3D_TEXTURE_ALPHA_A2    0xFD9A0440
#define Z3D_TEXTURE_ALPHA_A3    0xFD9A0444
#define Z3D_TEXTURE_ALPHA_A4    0xFD9A0448
#define Z3D_TEXTURE_ALPHA_A5    0xFD9A044C
#define Z3D_TEXTURE_ALPHA_B0    0xFD9A0450
#define Z3D_TEXTURE_ALPHA_B1    0xFD9A0454
#define Z3D_TEXTURE_ALPHA_B2    0xFD9A0458
#define Z3D_TEXTURE_ALPHA_B3    0xFD9A045C
#define Z3D_TEXTURE_ALPHA_B4    0xFD9A0460
#define Z3D_TEXTURE_ALPHA_B5    0xFD9A0464
#define Z3D_TEXTURE_FLUSH       0xFD9A0498
#define Z3D_GAMMA_TABLE0        0xFD9A049C
#define Z3D_GAMMA_TABLE1        0xFD9A04A0
#define Z3D_GAMMA_TABLE2        0xFD9A04A4
#define Z3D_ALPHA_TEST              0xFD9A0800
#define Z3D_STENCIL_TEST            0xFD9A0804
#define Z3D_DEPTH_ROP_BLEND_DITHER  0xFD9A0808
#define Z3D_MASK                    0xFD9A080C
#define Z3D_FBUS_MODE               0xFD9A0810
#define Z3D_GNT_SET                 0xFD9A0814
#define Z3D_BETWEEN_TEST            0xFD9A0818
#define Z3D_FB_BASE                 0xFD9A081C
#define Z3D_LCD_SIZE                0xFD9A0820
#define Z3D_FB_FLUSH                0xFD9A0824
#define Z3D_CACHE_INVALID           0xFD9A0828
#define Z3D_SC_MODE         0xFD9A0830
#define Z3D_SC0_MIN         0xFD9A0834
#define Z3D_SC0_MAX         0xFD9A0838
#define Z3D_SC1_MIN         0xFD9A083C
#define Z3D_SC1_MAX         0xFD9A0840
#define Z3D_SC2_MIN         0xFD9A0844
#define Z3D_SC2_MAX         0xFD9A0848
#define Z3D_SC3_MIN         0xFD9A084C
#define Z3D_SC3_MAX         0xFD9A0850
#define Z3D_READRESET       0xFD9A0854
#define Z3D_DET_MIN         0xFD9A0858
#define Z3D_DET_MAX         0xFD9A085C
#define Z3D_FB_BASE_SR      0xFD9A0860
#define Z3D_LCD_SIZE_SR     0xFD9A0864
#define Z3D_2D_CTRL_STATUS          0xFD9A0C00
#define Z3D_2D_SIZE                 0xFD9A0C04
#define Z3D_2D_SRCLOC               0xFD9A0C08
#define Z3D_2D_DSTLOC               0xFD9A0C0C
#define Z3D_2D_DMAPORT              0xFD9A0C10
#define Z3D_2D_CONSTANT_SOURCE0     0xFD9A0C14
#define Z3D_2D_CONSTANT_SOURCE1     0xFD9A0C18
#define Z3D_2D_STPCOLOR0            0xFD9A0C1C
#define Z3D_2D_STPCOLOR1            0xFD9A0C20
#define Z3D_2D_STPPARAMETER_SET0    0xFD9A0C24
#define Z3D_2D_STPPARAMETER_SET1    0xFD9A0C28
#define Z3D_2D_STPPAT_0     0xFD9A0C40
#define Z3D_2D_STPPAT_1     0xFD9A0C44
#define Z3D_2D_STPPAT_2     0xFD9A0C48
#define Z3D_2D_STPPAT_3     0xFD9A0C4C
#define Z3D_2D_STPPAT_4     0xFD9A0C50
#define Z3D_2D_STPPAT_5     0xFD9A0C54
#define Z3D_2D_STPPAT_6     0xFD9A0C58
#define Z3D_2D_STPPAT_7     0xFD9A0C5C
#define Z3D_2D_STPPAT_8     0xFD9A0C60
#define Z3D_2D_STPPAT_9     0xFD9A0C64
#define Z3D_2D_STPPAT_10    0xFD9A0C68
#define Z3D_2D_STPPAT_11    0xFD9A0C6C
#define Z3D_2D_STPPAT_12    0xFD9A0C70
#define Z3D_2D_STPPAT_13    0xFD9A0C74
#define Z3D_2D_STPPAT_14    0xFD9A0C78
#define Z3D_2D_STPPAT_15    0xFD9A0C7C
#define Z3D_2D_STPPAT_16    0xFD9A0C80
#define Z3D_2D_STPPAT_17    0xFD9A0C84
#define Z3D_2D_STPPAT_18    0xFD9A0C88
#define Z3D_2D_STPPAT_19    0xFD9A0C8C
#define Z3D_2D_STPPAT_20    0xFD9A0C90
#define Z3D_2D_STPPAT_21    0xFD9A0C94
#define Z3D_2D_STPPAT_22    0xFD9A0C98
#define Z3D_2D_STPPAT_23    0xFD9A0C9C
#define Z3D_2D_STPPAT_24    0xFD9A0CA0
#define Z3D_2D_STPPAT_25    0xFD9A0CA4
#define Z3D_2D_STPPAT_26    0xFD9A0CA8
#define Z3D_2D_STPPAT_27    0xFD9A0CAC
#define Z3D_2D_STPPAT_28    0xFD9A0CB0
#define Z3D_2D_STPPAT_29    0xFD9A0CB4
#define Z3D_2D_STPPAT_30    0xFD9A0CB8
#define Z3D_2D_STPPAT_31    0xFD9A0CBC
#define Z3D_WR_CTRL         0xFD9A1000
#define Z3D_WR_P0           0xFD9A1004
#define Z3D_WR_P1           0xFD9A1008
#define Z3D_WR_P2           0xFD9A100C
#define Z3D_WR_FGC          0xFD9A1010
#define Z3D_WR_BGC          0xFD9A1014
#define Z3D_WR_SZ           0xFD9A1018
#define Z3D_WR_PATPARAM     0xFD9A101C
#define Z3D_WR_PAT          0xFD9A1020
#define Z3D_SYS_STATUS      0xFD9A1400
#define Z3D_SYS_RESET       0xFD9A1404
#define Z3D_SYS_CLK         0xFD9A1408
#define Z3D_SYS_CONF        0xFD9A140C
#define Z3D_SYS_VERSION     0xFD9A1410
#define Z3D_SYS_DBINV       0xFD9A1418
#define Z3D_SYS_I2F_FMT     0xFD9A1420
#define Z3D_SYS_I2F_SRC     0xFD9A1424
#define Z3D_SYS_I2F_DST     0xFD9A1428
#define Z3D_SYS_GBCNT       0xFD9A1430
#define Z3D_SYS_BSYCNT      0xFD9A1434
#define Z3D_SYS_INT_STATUS  0xFD9A1450
#define Z3D_SYS_INT_MASK    0xFD9A1454
#define Z3D_SYS_INT_CLEAR   0xFD9A1458
#define TCD0        0xFD9C0000
#define TCD1        0xFD9C0400
#define TCD2        0xFD9C0800
#define TCD3        0xFD9C0C00
#define TCT0        0xFD9C1000
#define TCT1        0xFD9C1400
#define TCT2        0xFD9C1800
#define TCT3        0xFD9C1C00

/*	PFC	*/
#define PACR        0xA4050100
#define PBCR        0xA4050102
#define PCCR        0xA4050104
#define PDCR        0xA4050106
#define PECR        0xA4050108
#define PFCR        0xA405010A
#define PGCR        0xA405010C
#define PHCR        0xA405010E
#define PJCR        0xA4050110
#define PKCR        0xA4050112
#define PLCR        0xA4050114
#define PMCR        0xA4050116
#define PNCR        0xA4050118
#define PQCR        0xA405011A
#define PRCR        0xA405011C
#define PSCR        0xA405011E
#define PTCR        0xA4050140
#define PUCR        0xA4050142
#define PVCR        0xA4050144
#define PWCR        0xA4050146
#define PXCR        0xA4050148
#define PYCR        0xA405014A
#define PZCR        0xA405014C
#define PSELA       0xA405014E
#define PSELB       0xA4050150
#define PSELC       0xA4050152
#define PSELD       0xA4050154
#define PSELE       0xA4050156
#define HIZCRA      0xA4050158
#define HIZCRB      0xA405015A
#define HIZCRC      0xA405015C
#define HIZCRC		0xA405015C
#define MSELCRA		0xA4050180
#define MSELCRB		0xA4050182
#define PULCR		0xA4050184
#define SBSCR		0xA4050186
#define DRVCR		0xA405018A

/*	I/O Port	*/
#define PADR        0xA4050120
#define PBDR        0xA4050122
#define PCDR        0xA4050124
#define PDDR        0xA4050126
#define PEDR        0xA4050128
#define PFDR        0xA405012A
#define PGDR        0xA405012C
#define PHDR        0xA405012E
#define PJDR        0xA4050130
#define PKDR        0xA4050132
#define PLDR        0xA4050134
#define PMDR        0xA4050136
#define PNDR        0xA4050138
#define PQDR        0xA405013A
#define PRDR        0xA405013C
#define PSDR        0xA405013E
#define PTDR        0xA4050160
#define PUDR        0xA4050162
#define PVDR        0xA4050164
#define PWDR        0xA4050166
#define PXDR        0xA4050168
#define PYDR        0xA405016A
#define PZDR        0xA405016C

/*	UBC	*/
#define CBR0        0xFF200000
#define CRR0        0xFF200004
#define CAR0        0xFF200008
#define CAMR0       0xFF20000C
#define CBR1        0xFF200020
#define CRR1        0xFF200024
#define CAR1        0xFF200028
#define CAMR1       0xFF20002C
#define CDR1        0xFF200030
#define CDMR1       0xFF200034
#define CETR1       0xFF200038
#define CCMFR       0xFF200600
#define CBCR        0xFF200620

/*	H-UDI	*/
#define SDIR        0xFC110000
#define SDDRH       0xFC110008
#define SDDRL       0xFC11000A
#define SDINT       0xFC110018

#endif /* _ASM_CPU_SH7722_H_ */
