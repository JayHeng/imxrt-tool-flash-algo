/*
 * Copyright 2019, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_PGMC_H_
#define _FSL_PGMC_H_

#include "fsl_common.h"

/*!
 * @addtogroup PGMC
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief PGMC driver version 2.0.0. */
#define FSL_PGMC_RIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief _pgmc_setpoint_map PGMC setpoint map. */
enum
{
    kPGMC_SetPoint0  = 1UL << 0UL,
    kPGMC_SetPoint1  = 1UL << 1UL,
    kPGMC_SetPoint2  = 1UL << 2UL,
    kPGMC_SetPoint3  = 1UL << 3UL,
    kPGMC_SetPoint4  = 1UL << 4UL,
    kPGMC_SetPoint5  = 1UL << 5UL,
    kPGMC_SetPoint6  = 1UL << 6UL,
    kPGMC_SetPoint7  = 1UL << 7UL,
    kPGMC_SetPoint8  = 1UL << 8UL,
    kPGMC_SetPoint9  = 1UL << 9UL,
    kPGMC_SetPoint10 = 1UL << 10UL,
    kPGMC_SetPoint11 = 1UL << 11UL,
    kPGMC_SetPoint12 = 1UL << 12UL,
    kPGMC_SetPoint13 = 1UL << 13UL,
    kPGMC_SetPoint14 = 1UL << 14UL,
    kPGMC_SetPoint15 = 1UL << 15UL,
};

/*!
 * @brief The enumeration of MIF signal behaviour.
 */
enum _pgmc_mif_signal_behaviour
{
    kPGMC_AssertSleepSignal               = 1U << 0U,  /*!< Assert Sleep signal. */
    kPGMC_AssertInputGateSignal           = 1U << 1U,  /*!< Assert InputGate signal. */
    kPGMC_AssetLowSpeedSignal             = 1U << 2U,  /*!< Assert LowSpeed signal. */
    kPGMC_AssertHighSpeedSignal           = 1U << 3U,  /*!< Assert HighSpeed signal. */
    kPGMC_AssertStandbySignal             = 1U << 4U,  /*!< Assert Standby signal. */
    kPGMC_AssertArrayPowerDownSignal      = 1U << 5U,  /*!< Assert ArrayPowerDown signal. */
    kPGMC_AssertPeripheralPowerDownSignal = 1U << 6U,  /*!< Assert PeripheralPowerDown signal. */
    kPGMC_AssertInitnSignal               = 1U << 7U,  /*!< Assert Initn signal. */
    kPGMC_AssertSwitch1OffSignal          = 1U << 8U,  /*!< Assert Switch1Off signal. */
    kPGMC_AssertSwitch2OffSignal          = 1U << 9U,  /*!< Assert Switch2Off signal. */
    kPGMC_AssertIsoSignal                 = 1U << 10U, /*!< Assert Iso_en signal. */
};

/*!
 * @brief PGMC BPC assign domain enumeration.
 */
typedef enum _pgmc_bpc_assign_domain
{
    kPGMC_CM7Core = 0U, /*!< CM7 Core domain. */
    kPGMC_CM4Core = 1U  /*!< CM4 Core domain. */
} pgmc_bpc_assign_domain_t;

/*! @brief PGMC BPC SSAR status. */
typedef enum _pgmc_bpc_ssar_status_flag
{
    kPGMC_BPC_DataSavedButNotRestored = PGMC_BPC_BPC_SSAR_STAT_SAVED_MASK, /*!< Indicate data in this power domain is
                                                          already saved but not restored yet. */
    kPGMC_BPC_SsarSaveBusy    = PGMC_BPC_BPC_SSAR_STAT_BUSY_SAVE_MASK,     /*!< Busy requesting SSAR save. */
    kPGMC_BPC_SsarRestoreBusy = PGMC_BPC_BPC_SSAR_STAT_BUSY_RESTORE_MASK,  /*!< Busy requesting SSAR restore. */
} pgmc_bpc_ssar_status_flag_t;

/*! @brief CPU mode. */
typedef enum _pgmc_cpu_mode
{
    kPGMC_RunMode     = 0x0UL, /*!< RUN mode. */
    kPGMC_WaitMode    = 0x1UL, /*!< WAIT mode. */
    kPGMC_StopMode    = 0x2UL, /*!< STOP mode. */
    kPGMC_SuspendMode = 0x3UL, /*!< SUSPEND mode. */
} pgmc_cpu_mode_t;

/*! @brief PGMC control modes. */
typedef enum _pgmc_control_mode
{
    kPGMC_DisableLowPowerControl   = 0UL,
    kPGMC_ControlledByCpuPowerMode = 1UL,
    kPGMC_ControlledBySetPoint     = 2UL,
} pgmc_control_mode_t;

/*!
 * @brief The enumeration of standby on/off count mode.
 */
typedef enum _pgmc_standby_on_off_count_mode
{
    kPGMC_FinishOnceSignalChange   = 0UL, /*!< Finish the process once pmic_standby signal changes. */
    kPGMC_FinishOnceGetAcknowledge = 1UL, /*!< Finish the process once getting acknowledge from PMIC. */
    kPGMC_IgnorePMICAcknowledge    = 2UL, /*!< Ignore PMIC acknowledge. */
    kPGMC_TimeoutMode              = 3UL, /*!< Time out mode. */
} pgmc_standby_on_off_count_mode_t;

/*!
 * @brief The enumeration of memory low power level.
 */
typedef enum _pgmc_memory_low_power_level
{
    kPGMC_MLPLHighSpeed    = 1U,
    kPGMC_MLPLNormal       = 3U,
    kPGMC_MLPLLowSpeed     = 4U,
    kPGMC_MLPLInputGating  = 5U,
    kPGMC_MLPLStandby      = 6U,
    kPGMC_MLPLSleep        = 8U,
    kPGMC_MLPLArrOnPerOff  = 9U,
    kPGMC_MLPLArrOffPerOn  = 10U,
    kPGMC_MLPLArrOffPerOff = 11U,
    kPGMC_MLPLSw2          = 13U,
    kPGMC_MLPLSw2PerOff    = 14U,
    kPGMC_MLPLSw1PerOff    = 15U,
} pgmc_memory_low_power_level_t;

/*!
 * @brief The enumeration of MIF signal.
 */
typedef enum _pgmc_mif_signal
{
    kPGMC_SleepSignal               = 0U,  /*!< MIF Sleep signal. */
    kPGMC_InputGateSignal           = 1U,  /*!< MIF InputGate signal. */
    kPGMC_LowSpeedSignal            = 2U,  /*!< MIF LowSpeed signal. */
    kPGMC_HighSpeedSignal           = 3U,  /*!< MIF HighSpeed signal. */
    kPGMC_StandbySignal             = 4U,  /*!< MIF Standby signal. */
    kPGMC_ArrayPowerDownSignal      = 5U,  /*!< MIF ArrayPowerDown signal. */
    kPGMC_PeripheralPowerDownSignal = 6U,  /*!< MIF PeripheralPowerDown signal. */
    kPGMC_InitnSignal               = 7U,  /*!< MIF Initn signal. */
    kPGMC_Switch1OffSignal          = 8U,  /*!< MIF Switch1Off signal. */
    kPGMC_Switch2OffSignal          = 9U,  /*!< MIF Switch2Off signal. */
    kPGMC_IsoSignal                 = 10U, /*!< MIF Iso_en signal. */
} pgmc_mif_signal_t;

/*!
 * @brief The control option of the power domain controlled by CPU power mode.
 */
typedef struct _pgmc_bpc_cpu_power_mode_option
{
    pgmc_bpc_assign_domain_t assignDomain; /*!< Domain assignment of the BPC. The power mode of the selected core domain
                                              will control the selected power domain. */
    pgmc_memory_low_power_level_t memoryLowPowerLevel; /*!< Memory low power level. */
    bool stateSave; /*!< Request save the state of power domain before entering target power mode.
                            true    -   Save data when domain enter the selected mode.
                            false   -   Do not save data when domain enter the selected mode. */
    bool powerOff;  /*!< Request power off the power domain.
                            true    -   Power off the power domain when enter the selected mode.
                            false   -   Do not power off the power domain when enter the selected mode. */
} pgmc_bpc_cpu_power_mode_option_t;

/*!
 * @brief The control option of the power domain controlled by setpoint mode.
 */
typedef struct _pgmc_bpc_setpoint_mode_option
{
    pgmc_memory_low_power_level_t memoryLowPowerLevel; /*!< Memory low power level. */
    bool stateSave; /*!< Request save the state of power domain before entering target setpoint.
                            true    -   Save data when domain enter the selected setpoint.
                            false   -   Do not save data when domain enter the selected setpoint. */
    bool powerOff;  /*!< Request power off the power domain.
                            true    -   Power off the power domain when enter the selected setpoint.
                            false   -   Do not power off the power domain when enter the selected setpoint. */
} pgmc_bpc_setpoint_mode_option_t;

/*!
 * @brief PGMC power switch delay structure.
 */
typedef struct _pgmc_power_switch_delay_option
{
    uint32_t powerOffDelay;     /*!< Delay from receiving power off request to high fanout power switch shut off. */
    uint32_t powerOnDelay;      /*!< Delay from receiving power on request to high fanout power switch shut off.  */
    uint32_t isolationOnDelay;  /*!< Delay from receiving iso on request to isolation enable. */
    uint32_t isolationOffDelay; /*!< Delay from receiving iso off requst to isolation disable. */
} pgmc_power_switch_delay_option_t;

/*!
 * @brief The delay of each signal.
 */
typedef struct _pgmc_mif_signal_delay
{
    uint8_t assertDelay;   /*!< Delay before asserting signal to high. */
    uint8_t deAssertDelay; /*!< Delay before de-asserting signal to low. */
} pgmc_mif_signal_delay_t;

/*!
 * @brief PGMC PMIC standby acknowledge configuration.
 */
typedef struct _pgmc_standby_acknowledge_config
{
    pgmc_standby_on_off_count_mode_t standbyOffCountMode; /*!< PMIC standby off acknowledge count mode. */
    uint32_t standbyOffCount;                             /*!< PMIC standby off acknowledge count configure. */
    pgmc_standby_on_off_count_mode_t standbyOnCountMode;  /*!< PMIC standby on acknowledge count mode. */
    uint32_t standbyOnCount;                              /*!< PMIC standby on acknowledge count configure. */
} pgmc_standby_acknowledge_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Basic power controller
 * @{
 */

/*!
 * @brief Make the BPC module controlled by the target CPU power mode.
 *
 * This function makes the module controlled by four typical CPU power modes, It also configs the resource domain and
 * set memory low power level.
 *
 * @param base PGMC basic power controller base address.
 * @param mode Target CPU power mode.
 * @param assignDomain Domain assignment of the BPC. The power mode of the selected core domain will contrl the selected
 * power domain.
 * @param memoryLowPowerLevel Memory low power level.
 * @param stateSave Request save the state of power domain before entering target power mode.
 */
void PGMC_BPC_ControlPowerDomainByCpuPowerMode(PGMC_BPC_Type *base,
                                               pgmc_cpu_mode_t mode,
                                               const pgmc_bpc_cpu_power_mode_option_t *option);

/*!
 * @brief Make the BPC module controlled by the target set points.
 *
 * This function makes the module controlled by specific set point, It also supports set memory lowe power level.
 *
 * @note When setting more than one set point, use "|" between the map values in "_pgmc_setpoint_map".
 *
 * @param base PGMC basic power controller base address.
 * @param setPointMap Select target set points, refer to "_pgmc_setpoint_map".
 * @param memoryLowPowerLevel Memory low power level.
 * @param stateSave Request save the state of power domain before entering target set point.
 */
void PGMC_BPC_ControlPowerDomainBySetPointMode(PGMC_BPC_Type *base,
                                               uint32_t setPointMap,
                                               const pgmc_bpc_setpoint_mode_option_t *option);

/*!
 * @brief Control the selected power domain by software mode.
 *
 * @note The function is used to control power domain when the CPU is in RUN mode.
 *
 * @param base PGMC basic power controller base address.
 * @param powerOff. Power On/Off power domain in software mode.
 *                  true    -   Power off the power domain in software mode.
 *                  false   -   Power on the power domain in software mode.
 * @param requestMLPLChange. Request memory low power level software change in software mode.
 *                  true    -   Request memory low power level software change in software mode.
 *                  false   -   Do not request memory low power level software change in software mode.
 */
void PGMC_BPC_ControlPowerDomainBySoftwareMode(PGMC_BPC_Type *base, bool powerOff, bool requestMLPLChange);

/*!
 * @brief Set delay of power on/off power domain.
 *
 * @param base PGMC basic power controller base address.
 * @param delayOption Pointer to the @ref pgmc_power_switch_delay_option_t structure.
 */
void PGMC_BPC_SetPowerDomainPowerSwitchDelay(PGMC_BPC_Type *base, const pgmc_power_switch_delay_option_t *delayOption);

/*!
 * @brief Disable low power mode control.
 *
 * @param base PGMC basic power controller base address.
 */
static inline void PGMC_BPC_DisableLowPower(PGMC_BPC_Type *base)
{
    base->BPC_MODE = PGMC_BPC_BPC_MODE_CTRL_MODE(kPGMC_DisableLowPowerControl);
}

/*!
 * @brief Request power domain state restore at run mode.
 *
 * @param base PGMC basic power controller base address.
 */
static inline void PGMC_BPC_RequestStateRestoreAtRunMode(PGMC_BPC_Type *base)
{
    base->BPC_SSAR_RESTORE_CTRL |= PGMC_BPC_BPC_SSAR_RESTORE_CTRL_RESTORE_AT_RUN_MASK;
}

/*!
 * @brief Request power domain state restore when enters a set point.
 *
 * @note When setting more than one set point, use "|" between the map values in "_pgmc_setpoint_map".
 * @param base PGMC basic power controller base address.
 * @param setPointMap Select target set points, refer to "_pgmc_setpoint_map".
 */
static inline void PGMC_BPC_RequestStateRestoreAtSetPoint(PGMC_BPC_Type *base, uint32_t setPointMap)
{
    base->BPC_SSAR_RESTORE_CTRL |= PGMC_BPC_BPC_SSAR_RESTORE_CTRL_RESTORE_AT_SP(setPointMap & 0xFFFFU);
}

/*!
 * @brief Get the status of SSAR.
 *
 * @param base PGMC basic power controller base address.
 * @status mask value for BPC ssar status, refer to "pgmc_bpc_ssar_status_flag_t".
 * @return Indicate the working status of the SSAR.
 */
static inline bool PGMC_BPC_GetSsarStatus(PGMC_BPC_Type *base, pgmc_bpc_ssar_status_flag_t status)
{
    return ((uint32_t)status == (base->BPC_SSAR_STAT & (uint32_t)status));
}

/*!
 * @}
 */

/*!
 * @name CPU power controller
 * @{
 */

/*!
 * @brief Power off the CPC CORE module by the target CPU power mode.
 *
 * @param base CPC CORE module base address.
 * @param mode Target CPU power mode.
 */
void PGMC_CPC_CORE_PowerOffByCpuPowerMode(PGMC_CPC_Type *base, pgmc_cpu_mode_t mode);

/*!
 * @brief Disable low power mode control.
 *
 * @param base CPC CORE module base address.
 */
static inline void PGMC_CPC_CORE_DisableLowPower(PGMC_CPC_Type *base)
{
    base->CPC_CORE_MODE = PGMC_CPC_CPC_CORE_MODE_CTRL_MODE(kPGMC_DisableLowPowerControl);
}

/*!
 * @brief Set delay of power on/off core's power.
 *
 * @param base CPC CORE module base address.
 * @param delayOption Pointer to the @ref pgmc_power_switch_delay_option_t structure.
 */
void PGMC_CPC_CORE_SetCorePowerSwitchDelay(PGMC_CPC_Type *base, const pgmc_power_switch_delay_option_t *delayOption);

/*!
 * @brief Make the CPC CACHE module controlled by the target CPU power mode.
 *
 * This function makes the module controlled by four typical CPU power modes, it also can set memory low power level.
 *
 * @param base CPC CACHE module base address.
 * @param mode Target CPU power mode.
 * @param memoryLowPowerLevel Memory low power level.
 */
void PGMC_CPC_CACHE_ControlByCpuPowerMode(PGMC_CPC_Type *base,
                                          pgmc_cpu_mode_t mode,
                                          pgmc_memory_low_power_level_t memoryLowPowerLevel);

/*!
 * @brief Make the CPC CACHE module controlled by the target set points.
 *
 * This function makes the module controlled by specific set point, It also supports set memory lowe power level.
 *
 * @note When setting more than one set point, use "|" between the map values in "_pgmc_setpoint_map".
 *
 * @param base CPC CACHE module base address.
 * @param setPointMap Select target set points, refer to "_pgmc_setpoint_map".
 * @param memoryLowPowerLevel Memory low power level.
 */
void PGMC_CPC_CACHE_ControlBySetPointMode(PGMC_CPC_Type *base,
                                          uint32_t setPointMap,
                                          pgmc_memory_low_power_level_t memoryLowPowerLevel);

/*!
 * @brief Disable low power mode control.
 *
 * @param base CPC CACHE module base address.
 */
static inline void PGMC_CPC_CACHE_DisableLowPower(PGMC_CPC_Type *base)
{
    base->CPC_CACHE_MODE = PGMC_CPC_CPC_CACHE_MODE_CTRL_MODE(kPGMC_DisableLowPowerControl);
}

/*!
 * @brief Request CPC cache module's memory low power level change by software mode.
 *
 * @note If request memory low power level change, must wait the MLPL transition complete.
 *
 * @param base CPC LMEM module base address.
 */
void PGMC_CPC_CACHE_TriggerMLPLSoftwareChange(PGMC_CPC_Type *base);

/*!
 * @brief Make the CPC LMEM module controlled by the target CPU power mode.
 *
 * This function makes the module controlled by four typical CPU power modes, it also can set memory low power level.
 *
 * @param base CPC LMEM module base address.
 * @param mode Target CPU power mode.
 * @param memoryLowPowerLevel Memory low power level.
 */
void PGMC_CPC_LMEM_ControlByCpuPowerMode(PGMC_CPC_Type *base,
                                         pgmc_cpu_mode_t mode,
                                         pgmc_memory_low_power_level_t memoryLowPowerLevel);

/*!
 * @brief Make the CPC LMEM module controlled by the target set points.
 *
 * This function makes the module controlled by specific set point, It also supports set memory lowe power level.
 *
 * @note When setting more than one set point, use "|" between the map values in "_pgmc_setpoint_map".
 *
 * @param base CPC LMEM module base address.
 * @param setPointMap Select target set points, refer to "_pgmc_setpoint_map".
 * @param memoryLowPowerLevel Memory low power level.
 */
void PGMC_CPC_LMEM_ControlBySetPointMode(PGMC_CPC_Type *base,
                                         uint32_t setPointMap,
                                         pgmc_memory_low_power_level_t memoryLowPowerLevel);

/*!
 * @brief Disable low power mode control.
 *
 * @param base CPC LMEM module base address.
 */
static inline void PGMC_CPC_LMEM_DisableLowPower(PGMC_CPC_Type *base)
{
    base->CPC_LMEM_MODE = PGMC_CPC_CPC_LMEM_MODE_CTRL_MODE(kPGMC_DisableLowPowerControl);
}

/*!
 * @brief Request CPC LMEM module's memory low power level change in software mode.
 *
 * @note If request memory low power level change, must wait the MLPL transition complete.
 *
 * @param base CPC LMEM module base address.
 */
void PGMC_CPC_LMEM_TriggerMLPLSoftwareChange(PGMC_CPC_Type *base);

/*!
 * @}
 */

/*!
 * @name MIF module related APIs
 * @{
 */

/*!
 * @brief Set the behaviour of each signal in MIF.
 *
 * @note To control the memory low power operation, this function must be invoked after selecting the
 *       memory low power level.
 *       Use case:
 *        @code
 *              PGMC_BPC_ControlPowerDomainByCpuPowerMode(PGMC_BPC0_BASE, kPGMC_WaitMode, kPGMC_CM7Core,
 *                  kPGMC_MLPLSleep, false);
 *              PGMC_MIF_SetSignalBehaviour(PGMC_BPC0_MIF_BASE, kPGMC_MLPLSleep, kPGMC_AssertSleepSignal);
 *        @endcode
 *
 * @param base PGMC MIF peripheral base address.
 * @param memoryLevel The selected memory low power level. For details please refer to @ref
 * pgmc_memory_low_power_level_t.
 * @param mask. The mask of MIF signal behaviour. Should be the OR'ed value of @ref _pgmc_mif_signal_behaviour
 */
void PGMC_MIF_SetSignalBehaviour(PGMC_MIF_Type *base, pgmc_memory_low_power_level_t memoryLevel, uint32_t mask);

/*!
 * @brief Set the assert delay and de-assert delay for the selected signal to adjust timing.
 *
 * @param base PGMC MIF peripheral base address.
 * @param signal The signal in MIF module.
 * @param delay The pointer to the structure @ref pgmc_mif_signal_delay_t.
 */
void PGMC_MIF_SetSignalDelay(PGMC_MIF_Type *base, pgmc_mif_signal_t signal, const pgmc_mif_signal_delay_t *delay);

/*! @} */

/*!
 * @name PMIC power controller
 * @{
 */

/*!
 * @brief Make the PMIC module controlled by the target CPU power mode.
 *
 * @param base PMIC module base address.
 * @param mode Target CPU power mode.
 */
void PGMC_PPC_ControlByCpuPowerMode(PGMC_PPC_Type *base, pgmc_cpu_mode_t mode);

/*!
 * @brief Make the PMIC module controlled by the target set points.
 *
 * This function makes the module controlled by specific set point, It also supports PMIC standby on.
 *
 * @note When setting more than one set point, use "|" between the map values in "_pgmc_setpoint_map".
 *
 * @param base PMIC module base address.
 * @param setPointMap Select target set points, refer to "_pgmc_setpoint_map".
 * @param enableStandby true: PMIC standby on when system enters set point number and system is in standby mode.
 *                      false: PMIC standby on when system enters set point number
 */
void PGMC_PPC_ControlBySetPointMode(PGMC_PPC_Type *base, uint32_t setPointMap, bool enableStandby);

/*!
 * @brief Disable low power mode control.
 *
 * @param base PMIC module bsase address.
 */
static inline void PGMC_PPC_DisableLowPower(PGMC_PPC_Type *base)
{
    base->PPC_MODE = PGMC_PPC_PPC_MODE_CTRL_MODE(kPGMC_DisableLowPowerControl);
}

/*!
 * @brief Set PGMC's PMIC standby module acknowledge config.
 *
 * @param base PMIC module base address.
 * @param pgmc_standby_acknowledge_config_t Pointer to the @ref pgmc_standby_acknowledge_config_t structure.
 */
void PGMC_PPC_SetAcknowledgeConfig(PGMC_PPC_Type *base, const pgmc_standby_acknowledge_config_t *config);

/*!
 * @}
 */

#if defined(__cplusplus)
}
#endif
/*!
 * @}
 */
#endif /* _FSL_PGMC_H_ */
