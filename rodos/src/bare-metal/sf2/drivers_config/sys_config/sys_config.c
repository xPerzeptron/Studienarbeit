/*******************************************************************************
 * (c) Copyright 2012 Microsemi SoC Products Group.  All rights reserved.
 *
 *  Smartfusion2 system configuration. This file is automatically generated
 *  by the Libero tools. It contains the Smartfusion2 system configuration that
 *  was selected during the hardware configuration flow. 
 *
 */

#include "../../CMSIS/m2sxxx.h"
#include "../../CMSIS/sys_init_cfg_types.h"
#include "sys_config.h"

/*==============================================================================
 *                             !!! WARNING !!!
 *==============================================================================
 * The project including this file must be linked so that the content of this
 * file is located in internal eNVM at run time. The content of this file is
 * used to configure the system prior to RAM content initialization. This means
 * that the content of the data structures below will be used before the copy
 * from LMA to VMA takes place. The LMA and VMA locations of the content of this
 * file must be identical for the system to be seamlessly configured as part of
 * the CMSIS boot process.
 */

/*==============================================================================
 * Clock configuration
 */
/* No configuration data structure required. */

/*==============================================================================
 * Memory remapping configuration
 */
/* TBD. */

/*==============================================================================
 * MDDR configuration
 */
#if MSS_SYS_MDDR_CONFIG_BY_CORTEX

#    include "sys_config_mddr_define.h"

MDDR_TypeDef* const g_m2s_mddr_addr = (MDDR_TypeDef*)0x40020800;

const ddr_subsys_cfg_t g_m2s_mddr_subsys_config = {
    /*---------------------------------------------------------------------
     * DDR Controller registers.
     * All registers are 16-bit wide unless mentioned beside the definition.
     */
    {
      MDDR_DDRC_DYN_SOFT_RESET_CR,
      MDDR_DDRC_RESERVED0,
      MDDR_DDRC_DYN_REFRESH_1_CR,
      MDDR_DDRC_DYN_REFRESH_2_CR,
      MDDR_DDRC_DYN_POWERDOWN_CR,
      MDDR_DDRC_DYN_DEBUG_CR,
      MDDR_DDRC_MODE_CR,
      MDDR_DDRC_ADDR_MAP_BANK_CR,
      MDDR_DDRC_ECC_DATA_MASK_CR,
      MDDR_DDRC_ADDR_MAP_COL_1_CR,
      MDDR_DDRC_ADDR_MAP_COL_2_CR,
      MDDR_DDRC_ADDR_MAP_ROW_1_CR,
      MDDR_DDRC_ADDR_MAP_ROW_2_CR,
      MDDR_DDRC_INIT_1_CR,
      MDDR_DDRC_CKE_RSTN_CYCLES_1_CR,
      MDDR_DDRC_CKE_RSTN_CYCLES_2_CR,
      MDDR_DDRC_INIT_MR_CR,
      MDDR_DDRC_INIT_EMR_CR,
      MDDR_DDRC_INIT_EMR2_CR,
      MDDR_DDRC_INIT_EMR3_CR,
      MDDR_DDRC_DRAM_BANK_TIMING_PARAM_CR,
      MDDR_DDRC_DRAM_RD_WR_LATENCY_CR,
      MDDR_DDRC_DRAM_RD_WR_PRE_CR,
      MDDR_DDRC_DRAM_MR_TIMING_PARAM_CR,
      MDDR_DDRC_DRAM_RAS_TIMING_CR,
      MDDR_DDRC_DRAM_RD_WR_TRNARND_TIME_CR,
      MDDR_DDRC_DRAM_T_PD_CR,
      MDDR_DDRC_DRAM_BANK_ACT_TIMING_CR,
      MDDR_DDRC_ODT_PARAM_1_CR,
      MDDR_DDRC_ODT_PARAM_2_CR,
      MDDR_DDRC_ADDR_MAP_COL_3_CR,
      MDDR_DDRC_MODE_REG_RD_WR_CR,
      MDDR_DDRC_MODE_REG_DATA_CR,
      MDDR_DDRC_PWR_SAVE_1_CR,
      MDDR_DDRC_PWR_SAVE_2_CR,
      MDDR_DDRC_ZQ_LONG_TIME_CR,
      MDDR_DDRC_ZQ_SHORT_TIME_CR,
      MDDR_DDRC_ZQ_SHORT_INT_REFRESH_MARGIN_1_CR,
      MDDR_DDRC_ZQ_SHORT_INT_REFRESH_MARGIN_2_CR,
      MDDR_DDRC_PERF_PARAM_1_CR,
      MDDR_DDRC_HPR_QUEUE_PARAM_1_CR,
      MDDR_DDRC_HPR_QUEUE_PARAM_2_CR,
      MDDR_DDRC_LPR_QUEUE_PARAM_1_CR,
      MDDR_DDRC_LPR_QUEUE_PARAM_2_CR,
      MDDR_DDRC_WR_QUEUE_PARAM_CR,
      MDDR_DDRC_PERF_PARAM_2_CR,
      MDDR_DDRC_PERF_PARAM_3_CR,
      MDDR_DDRC_DFI_RDDATA_EN_CR,
      MDDR_DDRC_DFI_MIN_CTRLUPD_TIMING_CR,
      MDDR_DDRC_DFI_MAX_CTRLUPD_TIMING_CR,
      MDDR_DDRC_DFI_WR_LVL_CONTROL_1_CR,
      MDDR_DDRC_DFI_WR_LVL_CONTROL_2_CR,
      MDDR_DDRC_DFI_RD_LVL_CONTROL_1_CR,
      MDDR_DDRC_DFI_RD_LVL_CONTROL_2_CR,
      MDDR_DDRC_DFI_CTRLUPD_TIME_INTERVAL_CR,
      MDDR_DDRC_DYN_SOFT_RESET_ALIAS_CR,
      MDDR_DDRC_AXI_FABRIC_PRI_ID_CR,
    },

    /*---------------------------------------------------------------------
     * DDR PHY configuration registers
     */
    {
      MDDR_PHY_LOOPBACK_TEST_CR,
      MDDR_PHY_BOARD_LOOPBACK_CR,
      MDDR_PHY_CTRL_SLAVE_RATIO_CR,
      MDDR_PHY_CTRL_SLAVE_FORCE_CR,
      MDDR_PHY_CTRL_SLAVE_DELAY_CR,
      MDDR_PHY_DATA_SLICE_IN_USE_CR,
      MDDR_PHY_LVL_NUM_OF_DQ0_CR,
      MDDR_PHY_DQ_OFFSET_1_CR,
      MDDR_PHY_DQ_OFFSET_2_CR,
      MDDR_PHY_DQ_OFFSET_3_CR,
      MDDR_PHY_DIS_CALIB_RST_CR,
      MDDR_PHY_DLL_LOCK_DIFF_CR,
      MDDR_PHY_FIFO_WE_IN_DELAY_1_CR,
      MDDR_PHY_FIFO_WE_IN_DELAY_2_CR,
      MDDR_PHY_FIFO_WE_IN_DELAY_3_CR,
      MDDR_PHY_FIFO_WE_IN_FORCE_CR,
      MDDR_PHY_FIFO_WE_SLAVE_RATIO_1_CR,
      MDDR_PHY_FIFO_WE_SLAVE_RATIO_2_CR,
      MDDR_PHY_FIFO_WE_SLAVE_RATIO_3_CR,
      MDDR_PHY_FIFO_WE_SLAVE_RATIO_4_CR,
      MDDR_PHY_GATELVL_INIT_MODE_CR,
      MDDR_PHY_GATELVL_INIT_RATIO_1_CR,
      MDDR_PHY_GATELVL_INIT_RATIO_2_CR,
      MDDR_PHY_GATELVL_INIT_RATIO_3_CR,
      MDDR_PHY_GATELVL_INIT_RATIO_4_CR,
      MDDR_PHY_LOCAL_ODT_CR,
      MDDR_PHY_INVERT_CLKOUT_CR,
      MDDR_PHY_RD_DQS_SLAVE_DELAY_1_CR,
      MDDR_PHY_RD_DQS_SLAVE_DELAY_2_CR,
      MDDR_PHY_RD_DQS_SLAVE_DELAY_3_CR,
      MDDR_PHY_RD_DQS_SLAVE_FORCE_CR,
      MDDR_PHY_RD_DQS_SLAVE_RATIO_1_CR,
      MDDR_PHY_RD_DQS_SLAVE_RATIO_2_CR,
      MDDR_PHY_RD_DQS_SLAVE_RATIO_3_CR,
      MDDR_PHY_RD_DQS_SLAVE_RATIO_4_CR,
      MDDR_PHY_WR_DQS_SLAVE_DELAY_1_CR,
      MDDR_PHY_WR_DQS_SLAVE_DELAY_2_CR,
      MDDR_PHY_WR_DQS_SLAVE_DELAY_3_CR,
      MDDR_PHY_WR_DQS_SLAVE_FORCE_CR,
      MDDR_PHY_WR_DQS_SLAVE_RATIO_1_CR,
      MDDR_PHY_WR_DQS_SLAVE_RATIO_2_CR,
      MDDR_PHY_WR_DQS_SLAVE_RATIO_3_CR,
      MDDR_PHY_WR_DQS_SLAVE_RATIO_4_CR,
      MDDR_PHY_WR_DATA_SLAVE_DELAY_1_CR,
      MDDR_PHY_WR_DATA_SLAVE_DELAY_2_CR,
      MDDR_PHY_WR_DATA_SLAVE_DELAY_3_CR,
      MDDR_PHY_WR_DATA_SLAVE_FORCE_CR,
      MDDR_PHY_WR_DATA_SLAVE_RATIO_1_CR,
      MDDR_PHY_WR_DATA_SLAVE_RATIO_2_CR,
      MDDR_PHY_WR_DATA_SLAVE_RATIO_3_CR,
      MDDR_PHY_WR_DATA_SLAVE_RATIO_4_CR,
      MDDR_PHY_WRLVL_INIT_MODE_CR,
      MDDR_PHY_WRLVL_INIT_RATIO_1_CR,
      MDDR_PHY_WRLVL_INIT_RATIO_2_CR,
      MDDR_PHY_WRLVL_INIT_RATIO_3_CR,
      MDDR_PHY_WRLVL_INIT_RATIO_4_CR,
      MDDR_PHY_WR_RD_RL_CR,
      MDDR_PHY_RDC_FIFO_RST_ERR_CNT_CLR_CR,
      MDDR_PHY_RDC_WE_TO_RE_DELAY_CR,
      MDDR_PHY_USE_FIXED_RE_CR,
      MDDR_PHY_USE_RANK0_DELAYS_CR,
      MDDR_PHY_USE_LVL_TRNG_LEVEL_CR,
      MDDR_PHY_DYN_CONFIG_CR,
      MDDR_PHY_RD_WR_GATE_LVL_CR,
      MDDR_PHY_DYN_RESET_CR },

    /*---------------------------------------------------------------------
     * FIC-64 registers
     * These registers are 16-bit wide and 32-bit aligned.
     */
    {
      MDDR_DDR_FIC_NB_ADDR_CR,
      MDDR_DDR_FIC_NBRWB_SIZE_CR,
      MDDR_DDR_FIC_WB_TIMEOUT_CR,
      MDDR_DDR_FIC_HPD_SW_RW_EN_CR,
      MDDR_DDR_FIC_HPD_SW_RW_INVAL_CR,
      MDDR_DDR_FIC_SW_WR_ERCLR_CR,
      MDDR_DDR_FIC_ERR_INT_ENABLE_CR,
      MDDR_DDR_FIC_NUM_AHB_MASTERS_CR,
      MDDR_DDR_FIC_LOCK_TIMEOUTVAL_1_CR,
      MDDR_DDR_FIC_LOCK_TIMEOUTVAL_2_CR,
      MDDR_DDR_FIC_LOCK_TIMEOUT_EN_CR }
};

#endif

/*==============================================================================
 * FDDR configuration
 */
#if MSS_SYS_FDDR_CONFIG_BY_CORTEX

#    include "sys_config_fddr_define.h"

FDDR_TypeDef* const g_m2s_fddr_addr = (FDDR_TypeDef*)0x40021000;

const fddr_sysreg_t g_m2s_fddr_sysreg_subsys_config = {
    0x0001u, /* PLL_CONFIG_LOW_1 */
    0x0002u, /* PLL_CONFIG_LOW_2 */
    0x0003u, /* PLL_CONFIG_HIGH */
    0x0004u, /* FACC_CLK_EN */
    0x0005u, /* FACC_MUX_CONFIG */
    0x0006u, /* FACC_DIVISOR_RATIO */
    0x0007u, /* PLL_DELAY_LINE_SEL */
    0x0008u, /* SOFT_RESET */
    0x0009u, /* IO_CALIB */
    0x000Au, /* INTERRUPT_ENABLE */
    0x000Bu, /* AXI_AHB_MODE_SEL */
    0x000Cu  /* PHY_SELF_REF_EN */
};

const ddr_subsys_cfg_t g_m2s_fddr_subsys_config = {
    /*---------------------------------------------------------------------
     * DDR Controller registers.
     * All registers are 16-bit wide unless mentioned beside the definition.
     */
    {
      FDDR_DDRC_DYN_SOFT_RESET_CR,
      FDDR_DDRC_RESERVED0,
      FDDR_DDRC_DYN_REFRESH_1_CR,
      FDDR_DDRC_DYN_REFRESH_2_CR,
      FDDR_DDRC_DYN_POWERDOWN_CR,
      FDDR_DDRC_DYN_DEBUG_CR,
      FDDR_DDRC_MODE_CR,
      FDDR_DDRC_ADDR_MAP_BANK_CR,
      FDDR_DDRC_ECC_DATA_MASK_CR,
      FDDR_DDRC_ADDR_MAP_COL_1_CR,
      FDDR_DDRC_ADDR_MAP_COL_2_CR,
      FDDR_DDRC_ADDR_MAP_ROW_1_CR,
      FDDR_DDRC_ADDR_MAP_ROW_2_CR,
      FDDR_DDRC_INIT_1_CR,
      FDDR_DDRC_CKE_RSTN_CYCLES_1_CR,
      FDDR_DDRC_CKE_RSTN_CYCLES_2_CR,
      FDDR_DDRC_INIT_MR_CR,
      FDDR_DDRC_INIT_EMR_CR,
      FDDR_DDRC_INIT_EMR2_CR,
      FDDR_DDRC_INIT_EMR3_CR,
      FDDR_DDRC_DRAM_BANK_TIMING_PARAM_CR,
      FDDR_DDRC_DRAM_RD_WR_LATENCY_CR,
      FDDR_DDRC_DRAM_RD_WR_PRE_CR,
      FDDR_DDRC_DRAM_MR_TIMING_PARAM_CR,
      FDDR_DDRC_DRAM_RAS_TIMING_CR,
      FDDR_DDRC_DRAM_RD_WR_TRNARND_TIME_CR,
      FDDR_DDRC_DRAM_T_PD_CR,
      FDDR_DDRC_DRAM_BANK_ACT_TIMING_CR,
      FDDR_DDRC_ODT_PARAM_1_CR,
      FDDR_DDRC_ODT_PARAM_2_CR,
      FDDR_DDRC_ADDR_MAP_COL_3_CR,
      FDDR_DDRC_MODE_REG_RD_WR_CR,
      FDDR_DDRC_MODE_REG_DATA_CR,
      FDDR_DDRC_PWR_SAVE_1_CR,
      FDDR_DDRC_PWR_SAVE_2_CR,
      FDDR_DDRC_ZQ_LONG_TIME_CR,
      FDDR_DDRC_ZQ_SHORT_TIME_CR,
      FDDR_DDRC_ZQ_SHORT_INT_REFRESH_MARGIN_1_CR,
      FDDR_DDRC_ZQ_SHORT_INT_REFRESH_MARGIN_2_CR,
      FDDR_DDRC_PERF_PARAM_1_CR,
      FDDR_DDRC_HPR_QUEUE_PARAM_1_CR,
      FDDR_DDRC_HPR_QUEUE_PARAM_2_CR,
      FDDR_DDRC_LPR_QUEUE_PARAM_1_CR,
      FDDR_DDRC_LPR_QUEUE_PARAM_2_CR,
      FDDR_DDRC_WR_QUEUE_PARAM_CR,
      FDDR_DDRC_PERF_PARAM_2_CR,
      FDDR_DDRC_PERF_PARAM_3_CR,
      FDDR_DDRC_DFI_RDDATA_EN_CR,
      FDDR_DDRC_DFI_MIN_CTRLUPD_TIMING_CR,
      FDDR_DDRC_DFI_MAX_CTRLUPD_TIMING_CR,
      FDDR_DDRC_DFI_WR_LVL_CONTROL_1_CR,
      FDDR_DDRC_DFI_WR_LVL_CONTROL_2_CR,
      FDDR_DDRC_DFI_RD_LVL_CONTROL_1_CR,
      FDDR_DDRC_DFI_RD_LVL_CONTROL_2_CR,
      FDDR_DDRC_DFI_CTRLUPD_TIME_INTERVAL_CR,
      FDDR_DDRC_DYN_SOFT_RESET_ALIAS_CR,
      FDDR_DDRC_AXI_FABRIC_PRI_ID_CR },

    /*---------------------------------------------------------------------
     * DDR PHY configuration registers
     */
    {
      FDDR_PHY_LOOPBACK_TEST_CR,
      FDDR_PHY_BOARD_LOOPBACK_CR,
      FDDR_PHY_CTRL_SLAVE_RATIO_CR,
      FDDR_PHY_CTRL_SLAVE_FORCE_CR,
      FDDR_PHY_CTRL_SLAVE_DELAY_CR,
      FDDR_PHY_DATA_SLICE_IN_USE_CR,
      FDDR_PHY_LVL_NUM_OF_DQ0_CR,
      FDDR_PHY_DQ_OFFSET_1_CR,
      FDDR_PHY_DQ_OFFSET_2_CR,
      FDDR_PHY_DQ_OFFSET_3_CR,
      FDDR_PHY_DIS_CALIB_RST_CR,
      FDDR_PHY_DLL_LOCK_DIFF_CR,
      FDDR_PHY_FIFO_WE_IN_DELAY_1_CR,
      FDDR_PHY_FIFO_WE_IN_DELAY_2_CR,
      FDDR_PHY_FIFO_WE_IN_DELAY_3_CR,
      FDDR_PHY_FIFO_WE_IN_FORCE_CR,
      FDDR_PHY_FIFO_WE_SLAVE_RATIO_1_CR,
      FDDR_PHY_FIFO_WE_SLAVE_RATIO_2_CR,
      FDDR_PHY_FIFO_WE_SLAVE_RATIO_3_CR,
      FDDR_PHY_FIFO_WE_SLAVE_RATIO_4_CR,
      FDDR_PHY_GATELVL_INIT_MODE_CR,
      FDDR_PHY_GATELVL_INIT_RATIO_1_CR,
      FDDR_PHY_GATELVL_INIT_RATIO_2_CR,
      FDDR_PHY_GATELVL_INIT_RATIO_3_CR,
      FDDR_PHY_GATELVL_INIT_RATIO_4_CR,
      FDDR_PHY_LOCAL_ODT_CR,
      FDDR_PHY_INVERT_CLKOUT_CR,
      FDDR_PHY_RD_DQS_SLAVE_DELAY_1_CR,
      FDDR_PHY_RD_DQS_SLAVE_DELAY_2_CR,
      FDDR_PHY_RD_DQS_SLAVE_DELAY_3_CR,
      FDDR_PHY_RD_DQS_SLAVE_FORCE_CR,
      FDDR_PHY_RD_DQS_SLAVE_RATIO_1_CR,
      FDDR_PHY_RD_DQS_SLAVE_RATIO_2_CR,
      FDDR_PHY_RD_DQS_SLAVE_RATIO_3_CR,
      FDDR_PHY_RD_DQS_SLAVE_RATIO_4_CR,
      FDDR_PHY_WR_DQS_SLAVE_DELAY_1_CR,
      FDDR_PHY_WR_DQS_SLAVE_DELAY_2_CR,
      FDDR_PHY_WR_DQS_SLAVE_DELAY_3_CR,
      FDDR_PHY_WR_DQS_SLAVE_FORCE_CR,
      FDDR_PHY_WR_DQS_SLAVE_RATIO_1_CR,
      FDDR_PHY_WR_DQS_SLAVE_RATIO_2_CR,
      FDDR_PHY_WR_DQS_SLAVE_RATIO_3_CR,
      FDDR_PHY_WR_DQS_SLAVE_RATIO_4_CR,
      FDDR_PHY_WR_DATA_SLAVE_DELAY_1_CR,
      FDDR_PHY_WR_DATA_SLAVE_DELAY_2_CR,
      FDDR_PHY_WR_DATA_SLAVE_DELAY_3_CR,
      FDDR_PHY_WR_DATA_SLAVE_FORCE_CR,
      FDDR_PHY_WR_DATA_SLAVE_RATIO_1_CR,
      FDDR_PHY_WR_DATA_SLAVE_RATIO_2_CR,
      FDDR_PHY_WR_DATA_SLAVE_RATIO_3_CR,
      FDDR_PHY_WR_DATA_SLAVE_RATIO_4_CR,
      FDDR_PHY_WRLVL_INIT_MODE_CR,
      FDDR_PHY_WRLVL_INIT_RATIO_1_CR,
      FDDR_PHY_WRLVL_INIT_RATIO_2_CR,
      FDDR_PHY_WRLVL_INIT_RATIO_3_CR,
      FDDR_PHY_WRLVL_INIT_RATIO_4_CR,
      FDDR_PHY_WR_RD_RL_CR,
      FDDR_PHY_RDC_FIFO_RST_ERR_CNT_CLR_CR,
      FDDR_PHY_RDC_WE_TO_RE_DELAY_CR,
      FDDR_PHY_USE_FIXED_RE_CR,
      FDDR_PHY_USE_RANK0_DELAYS_CR,
      FDDR_PHY_USE_LVL_TRNG_LEVEL_CR,
      FDDR_PHY_DYN_CONFIG_CR,
      FDDR_PHY_RD_WR_GATE_LVL_CR,
      FDDR_PHY_DYN_RESET_CR,
    },

    /*---------------------------------------------------------------------
     * FIC-64 registers
     * These registers are 16-bit wide and 32-bit aligned.
     */
    {
      FDDR_DDR_FIC_NB_ADDR_CR,
      FDDR_DDR_FIC_NBRWB_SIZE_CR,
      FDDR_DDR_FIC_WB_TIMEOUT_CR,
      FDDR_DDR_FIC_HPD_SW_RW_EN_CR,
      FDDR_DDR_FIC_HPD_SW_RW_INVAL_CR,
      FDDR_DDR_FIC_SW_WR_ERCLR_CR,
      FDDR_DDR_FIC_ERR_INT_ENABLE_CR,
      FDDR_DDR_FIC_NUM_AHB_MASTERS_CR,
      FDDR_DDR_FIC_LOCK_TIMEOUTVAL_1_CR,
      FDDR_DDR_FIC_LOCK_TIMEOUTVAL_2_CR,
      FDDR_DDR_FIC_LOCK_TIMEOUT_EN_CR }
};

#endif