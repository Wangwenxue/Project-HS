/**
 ****************************************************************************************
 *
 * @file nvds.h
 *
 * @brief Non Volatile Data Storage (NVDS) driver
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _NVDS_H_
#define _NVDS_H_

/**
 ****************************************************************************************
 * @addtogroup NVDS
 * @ingroup COMMON
 * @brief Non Volatile Data Storage (NVDS)
 *
  @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>           // boolean definition
#include <stdint.h>            // integer definition
#include "fw_func_addr.h"
#include "driver_config.h"

/*
 * DEFINES
 ****************************************************************************************
 */

typedef uint16_t nvds_tag_len_t;

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
/// Possible Returned Status
enum NVDS_STATUS
{
    /// NVDS status OK
    NVDS_OK,
    /// generic NVDS status KO
    NVDS_FAIL,
    /// NVDS TAG unrecognized
    NVDS_TAG_NOT_DEFINED,
    /// No space for NVDS
    NVDS_NO_SPACE_AVAILABLE,
    /// Length violation
    NVDS_LENGTH_OUT_OF_RANGE,
    /// NVDS parameter locked
    NVDS_PARAM_LOCKED,
    /// NVDS corrupted
    NVDS_CORRUPT,
    /// No purge buffer 
    NVDS_NO_TEMP_BUF_AVAILABLE
};

/// List of NVDS TAG identifiers
enum NVDS_TAG
{
    /// Definition of the tag associated to each parameters
    /// Local Bd Address
    NVDS_TAG_BD_ADDRESS                 = 0x01,
    /// Device Name
    NVDS_TAG_DEVICE_NAME                = 0x02,
    /// Radio Drift
    NVDS_TAG_LPCLK_DRIFT                = 0x03,
    /// factory setting
    NVDS_TAG_FACTORY_SETTING_0          = 0x04,
    /// Oscillator wake-up time
    NVDS_TAG_OSC_WAKEUP_TIME            = 0x05,
    /// Radio wake-up time
    NVDS_TAG_RM_WAKEUP_TIME             = 0x06,
    /// Enable sleep mode
    NVDS_TAG_SLEEP_ENABLE               = 0x07,
    /// factory setting
    NVDS_TAG_FACTORY_SETTING_1          = 0x08,
    NVDS_TAG_FACTORY_SETTING_2          = 0x09,
    NVDS_TAG_FACTORY_SETTING_3          = 0x0a,
    /// TK type
    NVDS_TAG_TK_TYPE                    = 0x0b,
    /// TK
    NVDS_TAG_TK_KEY                     = 0x0c,
    /// IRK
    NVDS_TAG_IRK_KEY                    = 0x0d,
    /// CSRK
    NVDS_TAG_CSRK_KEY                   = 0x0e,
    /// LTK
    NVDS_TAG_LTK_KEY                    = 0x0f,
    /// crystal oscillator cap loading selection
    NVDS_TAG_XCSEL                      = 0x10,
    /// temperature offset
    NVDS_TAG_TEMPERATURE_OFFSET         = 0x11,
    /// adc internal reference scale
    NVDS_TAG_ADC_INT_REF_SCALE          = 0x12,
    /// adc internal reference vcm
    NVDS_TAG_ADC_INT_REF_VCM            = 0x13,
    
    /// this is the TAG used to be the marker of the last TAG in NVDS (= 0xFF because when
    /// FLASH are erased, they are set to = 0xFF)
    NVDS_END_MARKER_TAG                 = 0xFF,
};

/// List of NVDS Tag lengths
enum NVDS_LEN
{
    // Definition of length associated to each parameters
    /// Local Bd Address
    NVDS_LEN_BD_ADDRESS                 = 6,
    /// Device Name
    NVDS_LEN_DEVICE_NAME                = 32,
    /// Low power clock drift
    NVDS_LEN_LPCLK_DRIFT                = 2,
    /// Factory setting
    NVDS_LEN_FACTORY_SETTING_0          = 2,
    /// Oscillator wake-up time
    NVDS_LEN_OSC_WAKEUP_TIME            = 2,
    /// Radio wake-up time
    NVDS_LEN_RM_WAKEUP_TIME             = 2,
    /// Enable sleep mode
    NVDS_LEN_SLEEP_ENABLE               = 1,
    /// Factory setting
    NVDS_LEN_FACTORY_SETTING_1          = 1,
    NVDS_LEN_FACTORY_SETTING_2          = 4,
    NVDS_LEN_FACTORY_SETTING_3          = 4,
    /// TK type
    NVDS_LEN_TK_TYPE                    = 1,
    /// TK
    NVDS_LEN_TK_KEY                     = 16,
    /// IRK
    NVDS_LEN_IRK_KEY                    = 16,
    /// CSRK
    NVDS_LEN_CSRK_KEY                   = 16,
    /// LTK
    NVDS_LEN_LTK_KEY                    = 16,
    /// crystal oscillator cap loading selection
    NVDS_LEN_XCSEL                      = 1,
    /// temperature offset
    NVDS_LEN_TEMPERATURE_OFFSET         = 4,
    /// adc internal reference scale
    NVDS_LEN_ADC_INT_REF_SCALE          = 4,
    /// adc internal reference vcm
    NVDS_LEN_ADC_INT_REF_VCM            = 4,
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Look for a specific tag and return, if found and matching (in length), the
 *        DATA part of the TAG.
 *
 * If the length does not match, the TAG header structure is still filled, in order for
 * the caller to be able to check the actual length of the TAG.
 *
 * @param[in]  tag     TAG to look for whose DATA is to be retrieved
 * @param[in]  length  Expected length of the TAG
 * @param[out] buf     A pointer to the buffer allocated by the caller to be filled with
 *                     the DATA part of the TAG
 *
 * @return  NVDS_OK                  The read operation was performed
 *          NVDS_LENGTH_OUT_OF_RANGE The length passed in parameter is different than the TAG's
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
uint8_t __nvds_get(uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf);
#define nvds_get __nvds_get
#elif defined(QN_9020_B4)
typedef uint8_t (*p_nvds_get)(uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf);
#define nvds_get ((p_nvds_get)(_nvds_get))
#endif


#if QN_NVDS_WRITE
/**
 ****************************************************************************************
 * @brief Look for a specific tag and delete it (Status set to invalid)
 *
 * Implementation notes
 * 1. The write function call return status is not handled
 *
 * @param[in]  tag    TAG to mark as deleted
 *
 * @return NVDS_OK                TAG found and deleted
 *         NVDS_PARAM_LOCKED    TAG found but can not be deleted because it is locked
 *         (others)        return values from function call @ref nvds_browse_tag
 ****************************************************************************************
 */
uint8_t __nvds_del(uint8_t tag);
#define nvds_del __nvds_del

/**
 ****************************************************************************************
 * @brief Look for a specific tag and lock it (Status lock bit set to LOCK).
 *
 * The write function call return status is not handled
 *
 * @param[in]  tag    TAG to mark as locked
 *
 * @return NVDS_OK    TAG found and locked
 *         (others)        return values from function call @ref nvds_browse_tag
 ****************************************************************************************
 */
uint8_t __nvds_lock(uint8_t tag);
#define nvds_lock __nvds_lock

/**
 ****************************************************************************************
 * @brief This function adds a specific TAG to the NVDS.
 *
 * Steps:
 * 1) parse all the TAGs to:
 * 1.1) calculate the total size of all the valid TAGs
 * 1.2) erase the existing TAGs that have the same ID
 * 1.3) check if we can use the same TAG area in case of an EEPROM
 * 1.4) check that the TAG is not locked
 * 2) if we have to add the new TAG at the end fo the NVDS (cant use same area):
 * 2.1) allocate the appropriate amount of memory
 * 2.2) purge the NVDS
 * 2.3) free the memory allocated
 * 2.4) check that there is now enough room for the new TAG or return
 *      NO_SPACE_AVAILABLE
 * 3) add the new TAG
 *
 * @param[in]  tag     TAG to look for whose DATA is to be retrieved
 * @param[in]  length  Expected length of the TAG
 * @param[in]  buf     Pointer to the buffer containing the DATA part of the TAG to add to
 *                     the NVDS
 *
 * @return NVDS_OK                  New TAG correctly written to the NVDS
 *         NVDS_PARAM_LOCKED        New TAG is trying to overwrite a TAG that is locked
 *         NO_SPACE_AVAILABLE       New TAG can not fit in the available space in the NVDS
 ****************************************************************************************
 */
uint8_t __nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf);
#define nvds_put __nvds_put

#endif

/// @} NVDS

#endif // _NVDS_H_
