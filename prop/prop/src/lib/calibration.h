/**
 ****************************************************************************************
 *
 * @file calibration.h
 *
 * @brief Header file of calibration for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _CALIB_H_
#define _CALIB_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_CALIB==TRUE
#include "syscon.h"

/**
 ****************************************************************************************
 * @defgroup CALIB Calibration Driver
 * @ingroup DRIVERS
 * @brief Calibration driver
 *
 * @{
 *
 ****************************************************************************************
 */

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
#if CONFIG_ENABLE_ROM_DRIVER_CALIB==TRUE

typedef void     (*p_calibration_void)(void);
typedef void     (*p_calibration_init)(enum CLK_TYPE type);
typedef void     (*p_seq_calibration)(uint32_t pa_cal_en);
typedef void     (*p_freq_hop_calibration)(uint32_t lo_cal_skip, uint32_t lo_kcal_skip);
typedef void     (*p_calibration_cb_register)(void (*callback[8])(void));

#define calibration_init                ((p_calibration_init)           _calibration_init)
#define calibration_cb_register         ((p_calibration_cb_register)    _calibration_cb_register)
#define ref_pll_calibration             ((p_calibration_void)           _ref_pll_calibration)
#define rc_calibration                  ((p_calibration_void)           _rc_calibration)
#define lo_calibration                  ((p_calibration_void)           _lo_calibration)
#define lo_kcal_calibration             ((p_calibration_void)           _lo_kcal_calibration)
#define pa_calibration                  ((p_calibration_void)           _pa_calibration)
#define r_calibration                   ((p_calibration_void)           _r_calibration)
#define ros_calibration                 ((p_calibration_void)           _ros_calibration)
#define rco_calibration                 ((p_calibration_void)           _rco_calibration)
#define seq_calibration                 ((p_seq_calibration)            _seq_calibration)
#define freq_hop_calibration            ((p_freq_hop_calibration)       _freq_hop_calibration)
#define rco_calibration_cb              ((p_calibration_void)           _rco_calibration_cb)

#else

#if CONFIG_CALIB_DEFAULT_IRQHANDLER==TRUE
void CALIB_IRQHandler(void);
#endif
extern void calibration_init(enum CLK_TYPE type);
extern void calibration_cb_register(void (*callback[8])(void));
extern void seq_calibration(uint32_t pa_cal_en);
extern void freq_hop_calibration(uint32_t lo_cal_skip, uint32_t lo_kcal_skip);
extern void ref_pll_calibration(void);
extern void rc_calibration(void);
extern void lo_calibration(void);
extern void lo_kcal_calibration(void);
extern void pa_calibration(void);
extern void r_calibration(void);
extern void ros_calibration(void);
extern void rco_calibration(void);
extern void rco_calibration_cb(void);

#endif


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

/// @} CALIB
#endif /* CONFIG_ENABLE_DRIVER_CALIB==TRUE */
#endif /* _CALIB_H_ */
