/**
 ****************************************************************************************
 *
 * @file lib.c
 *
 * @brief Patch, BLE hardware initialization, BLE work mode.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


#include "lib.h"
#include "qnrf.h"
#include "calibration.h"
#include "nvds.h"
#include "stddef.h"
#include "serialflash.h"

/**
 ****************************************************************************************
 * @brief Set 32k xtal ppm.
 * @param[in]     ppm
 ***************************************************************************************
 */

extern uint32_t __rd_reg(uint32_t addr);
extern void __wr_reg(uint32_t addr, uint32_t val);
static volatile bool dc_dc_flag;

/**
 ****************************************************************************************
 * @brief DC-DC Enable
 * @param[in]   enable true - enable dc-dc; false - disable
 ****************************************************************************************
 */
void dc_dc_enable(bool enable)
{
    dc_dc_flag = enable;

    // DC-DC always bypass
    syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_BUCK_BYPASS, MASK_ENABLE);

    if(enable)
    {
        syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_PD_BUCK, MASK_DISABLE);
    }

    // Disable DC-DC
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_BUCK, MASK_ENABLE);
}



/*
 * NVDS patch
 ****************************************************************************************
 */

#if 1 // NVDS patch
/// Environment structure of the NVDS module
struct nvds_env_tag
{
    /// Function to read the device Address being in the NVDS memory space
    void  (*read)(uint32_t const                    address,
                  uint32_t const                    length,
                  uint8_t* const                    buf);
    /// Function to write the device Address being in the NVDS memory space
    void (*write)(uint32_t const                    address,
                  uint32_t const                    length,
                  uint8_t* const                    buf);
    /// Function to erase the entire NVDS memory space
    void (*erase)(uint32_t const                    address,
                  uint32_t const                    length);

    /// NVDS base pointer
    uint8_t *nvds_space;

    /// Total size of the NVDS area
    uint32_t   total_size;

    /// Flash ID
    uint8_t flash_id;

    // WJ : temp buffer
    /// temp buffer pointer
    uint8_t *nvds_temp_buf;

    /// temp buffer length
    uint32_t nvds_temp_buf_len;
};

#define FLASH_PAGE_SIZE     (256)
#define NVDS_ADDRESS        (0)
#define NVDS_SIZE           (4*1024)
#define NVDS_BACKUP_ADDRESS (124*1024)

/// NVDS environment
#define nvds_env ((struct nvds_env_tag *)(_nvds_env))
/// NVDS initialization
typedef uint8_t (*p_nvds_init)(uint8_t *base, uint32_t len, uint8_t *temp_buf, uint32_t temp_buf_len);
#define fw_nvds_init ((p_nvds_init)(_nvds_init))

#define _nvds_null_read                           0x01001f2d
#define _nvds_null_write                          0x01001f39
#define _nvds_null_erase                          0x01001f3b

typedef void (*p_nvds_null_read)(uint32_t address, uint32_t length, uint8_t *buf);
#define fw_nvds_null_read ((p_nvds_null_read)(_nvds_null_read))
typedef void (*p_nvds_null_write)(uint32_t address, uint32_t length, uint8_t *buf);
#define fw_nvds_null_write ((p_nvds_null_write)(_nvds_null_write))
typedef void (*p_nvds_null_erase)(uint32_t address, uint32_t length);
#define fw_nvds_null_erase ((p_nvds_null_erase)(_nvds_null_erase))

typedef bool (*p_is_flash_busy)(void);
#define is_flash_busy ((p_is_flash_busy)(_is_flash_busy))
typedef void (*p_flash_write_enable)(void);
#define flash_write_enable ((p_flash_write_enable)(_flash_write_enable))
typedef void (*p_read_flash)(uint32_t addr, uint32_t *pBuf, uint32_t nbyte);
#define read_flash ((p_read_flash) _read_flash)
typedef void (*p_write_flash)(uint32_t addr,const uint32_t * pBuf,uint32_t nbyte);
#define write_flash ((p_write_flash) _write_flash)

//__STATIC_INLINE bool is_flash_present(void)
//{
//    const uint32_t id = sf_ctrl_GetFlashID(QN_SF_CTRL) & 0xFFFFFF;
//
//    return ((id > 0 && id < 0xFFFFFF) ? true : false);
//}

//static uint32_t get_ahb_clk(void)
//{
//    uint32_t div_ctrl =  syscon_GetCMDCR(QN_SYSCON);
//    uint32_t clk_mux = (div_ctrl & SYSCON_MASK_CLK_MUX) >> SYSCON_POS_CLK_MUX;
//    uint32_t ahb_bypass = div_ctrl & SYSCON_MASK_AHB_DIV_BYPASS;
//    uint32_t sys_clk = 0, ahb_clk;
//
//    switch(clk_mux)
//    {
//        case CLK_XTAL:
//            if((syscon_GetLO1(QN_SYSCON) &  SYSCON_MASK_XDIV) == 0)
//            {
//                sys_clk = XTAL_16MHz;
//            }
//            else
//            {
//                sys_clk = XTAL_32MHz;
//            }
//            break;
//
//        case CLK_INT_20M:
//            sys_clk = SYS_INT_20M;
//            break;
//
//        case CLK_INT_32M:
//            sys_clk = SYS_PLL_32M;
//            break;
//
//        case CLK_LOW_32K:
//            sys_clk = SYS_LOW_32K;
//            break;
//
//        default:
//            break;
//    }
//
//    if (ahb_bypass != 0)
//    {
//        ahb_clk = sys_clk;
//    }
//    else
//    {
//        uint32_t ahb_div = (div_ctrl & SYSCON_MASK_AHB_DIVIDER) >> SYSCON_POS_AHB_DIVIDER;
//        ahb_clk = sys_clk / (2*(ahb_div+1));
//    }
//
//    return ahb_clk;
//}

__STATIC_INLINE bool is_flash_off(void)
{
    if((syscon_GetPGCR2(QN_SYSCON) & SYSCON_MASK_FLASH_VCC_EN) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//__STATIC_INLINE void flash_on(void)
//{
//    uint32_t time_out = 0;
//
//    // power on
//    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_FLASH_VCC_EN, MASK_ENABLE);
//
//    // wait 200us
//    delay(get_ahb_clk() / 45000 + 1);
//
//    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
//
//    //Wait flash is ready
//    while(++time_out < 256 && !is_flash_present());
//}

typedef void (*p_flash_off)(void);
#define fw_flash_off ((p_flash_off)(0x0100125d))

static void sector_erase_flash_patch(uint32_t addr, uint32_t n)
{
    while (n--)
    {
        flash_write_enable(); //set the Write Enable Latch bit
        sf_ctrl_SetCmdAddr(QN_SF_CTRL, __REV(addr | (0x20<<24)));
        addr += 4*1024;            //all flash sector erase command erasing size is 4K
    }
}

static void nvds_read(uint32_t address, uint32_t length, uint8_t *buf)
{
    uint32_t data, len;

    bool flash_off = is_flash_off();

    if(flash_off)
        power_on_flash(FLASH_RD);

    // Test the validity of address + length
    //ASSERT_ERR(((address + length) <= nvds_env->total_size));

    len = length & (~3);
    if(len)
    {
        if((uint32_t)buf & 0x3)
        {
            uint32_t offset = 0;
            while(offset<len)
            {
                read_flash(((uint32_t)nvds_env->nvds_space + address + offset), &data, 4);
                memcpy(buf+offset, &data, 4);
                offset += 4;
            }
        }
        else
        {
            read_flash(((uint32_t)nvds_env->nvds_space + address), (uint32_t*)buf, len);
        }
    }

    if(length != len)
    {
        read_flash(((uint32_t)nvds_env->nvds_space + address + len), &data, 4);
        memcpy(buf+len, &data, (length&0x3));
    }

    if(flash_off)
        //fw_flash_off();
        power_off_flash();
}

static void __nvds_wrtie(uint32_t address, uint32_t length, uint8_t *buf)
{
    uint32_t data, offset, len;

    // Unaligned start address
    if(address & 0x3)
    {
        offset = address - (((uint32_t)nvds_env->nvds_space + address) & (~3));
        len = 4 - offset;
        len = (len<length)?len:length;
        read_flash(((uint32_t)nvds_env->nvds_space + address) & (~3), &data, 4);
        memcpy(((uint8_t*)(&data) + offset), buf, len);
        write_flash(((uint32_t)nvds_env->nvds_space + address) & (~3), &data, 4);
        length -= len;
        address = (address & (~3)) + 4;
        buf += len;
    }

    // Aligned part
    len = length & (~3);
    if(len)
    {
        if((uint32_t)buf & 0x3)
        {
            uint32_t offset = 0;
            while(offset < len)
            {
                memcpy((uint8_t*)(&data), buf+offset, 4);
                write_flash(((uint32_t)nvds_env->nvds_space + address + offset), &data, 4);
                offset += 4;
            }
        }
        else
        {
            write_flash(((uint32_t)nvds_env->nvds_space + address), (uint32_t*)buf, len);
        }

        length -= len;
        address += len;
        buf += len;
    }

    // tail
    if(length)
    {
        read_flash(((uint32_t)nvds_env->nvds_space + address), &data, 4);
        memcpy((uint8_t*)(&data), buf, length);
        write_flash(((uint32_t)nvds_env->nvds_space + address), &data, 4);
    }
}

static void nvds_write(uint32_t address, uint32_t length, uint8_t *buf)
{
    const uint32_t end_address = address + length;
    const uint32_t up_align_address = (address + FLASH_PAGE_SIZE) & ~(FLASH_PAGE_SIZE - 1);

    //ASSERT_ERR(length != 0);

    // Cross Boundary
    if(end_address > up_align_address)
    {
        uint32_t written_len = up_align_address - address;
        __nvds_wrtie(address+0, written_len, &buf[0]);

        while(1)
        {
            if(written_len + FLASH_PAGE_SIZE >= length)
            {
                __nvds_wrtie(address+written_len, length-written_len, &buf[written_len]);
                break;
            }

            __nvds_wrtie(address+written_len, FLASH_PAGE_SIZE, &buf[written_len]);
            written_len += FLASH_PAGE_SIZE;
        }
    }
    // Not Cross Boundary
    else
    {
        __nvds_wrtie(address, length, buf);
    }
}

static void nvds_copy(uint32_t dst, uint32_t src)
{
    uint8_t nvds_temp_buff[FLASH_PAGE_SIZE];

    sector_erase_flash_patch(dst, 1);
    for(int i=0; i<NVDS_SIZE; i+=FLASH_PAGE_SIZE)
    {
        read_flash(src+i, (uint32_t *)nvds_temp_buff, FLASH_PAGE_SIZE);
        write_flash(dst+i, (uint32_t *)nvds_temp_buff, FLASH_PAGE_SIZE);
    }
}

static void nvds_erase(uint32_t address, uint32_t length)
{
    uint32_t data = 0;

    // backup
    nvds_copy(NVDS_BACKUP_ADDRESS, NVDS_ADDRESS);

    // clear magic number
    nvds_env->write(0, 4, (uint8_t*)&data);

    // erase nvds
    sector_erase_flash_patch(address, 1);
}

static void nvds_restore_backup(void)
{
    bool nvds_valid, nvds_backup_valid;
    uint8_t nvds_temp_buff[FLASH_PAGE_SIZE];

    read_flash(NVDS_ADDRESS, (uint32_t *)nvds_temp_buff, 4);

    nvds_valid = memcmp(nvds_temp_buff, "NVDS", 4) == 0 ? true : false;

    read_flash(NVDS_BACKUP_ADDRESS, (uint32_t *)nvds_temp_buff, FLASH_PAGE_SIZE);

    nvds_backup_valid = memcmp(nvds_temp_buff, "NVDS", 4) == 0 ? true : false;

    if(nvds_valid)
    {
        if(!nvds_backup_valid)
        {
            //backup
            nvds_copy(NVDS_BACKUP_ADDRESS, NVDS_ADDRESS);
        }
    }
    else
    {
        if(nvds_backup_valid)
        {
            //restore
            nvds_copy(NVDS_ADDRESS, NVDS_BACKUP_ADDRESS);
        }
    }
}

static void nvds_patch(uint8_t *temp_buf, uint32_t temp_buf_len)
{
    uint32_t nvds_length = 0x1000;

    // prevent unexpected software reset
    if(is_flash_off())
        power_on_flash(FLASH_WR);

    // User could configure nvds total length
    if(temp_buf_len != 0)
    {
        nvds_length = temp_buf_len;
        nvds_restore_backup();
    }

    // nvds initialization in the FW
    fw_nvds_init(0x0, nvds_length, temp_buf, temp_buf_len);

    // patch the access functions
    if(nvds_env->write != fw_nvds_null_write)
    {
        nvds_env->write = &nvds_write;
        nvds_env->erase = &nvds_erase;
    }

    if(nvds_env->read != fw_nvds_null_read)
    {
        nvds_env->read = &nvds_read;
    }
}

#define NVDS_PARAMETER_MAX_LENGTH   128

/// TAG STATUS bit assignment
#define NVDS_STATUS_VALID_MASK   0x01
#define NVDS_STATUS_VALID        0x00
#define NVDS_STATUS_NOT_VALID    0x01
#define NVDS_STATUS_LOCKED_MASK  0x02
#define NVDS_STATUS_LOCKED       0x00
#define NVDS_STATUS_NOT_LOCKED   0x02
#define NVDS_STATUS_ERASED_MASK  0x04
#define NVDS_STATUS_ERASED       0x00
#define NVDS_STATUS_NOT_ERASED   0x04

// NVDS Mapping

/// Magic number offset
#define NVDS_MAGIC_NUMBER_ADDRESS         0x0000
/// Size of magic number
#define NVDS_MAGIC_NUMBER_LENGTH               4

#define NVDS_START_STORAGE_AREA_ADDRESS        4

#define CO_ALIGN4_HI(val) (((val)+3)&~3)


/// Check is tag is the last one
#define NVDS_IS_TAG_LAST(h) \
    ((h).tag == NVDS_END_MARKER_TAG)
/// Check is tag is valid
#define NVDS_IS_TAG_OK(h) \
    ((((h).status) & (NVDS_STATUS_VALID_MASK|NVDS_STATUS_ERASED_MASK)) == \
     (NVDS_STATUS_VALID|NVDS_STATUS_NOT_ERASED))
/// Check is tag is locked
#define NVDS_IS_TAG_LOCKED(h) \
    ((((h).status) & NVDS_STATUS_LOCKED_MASK) == NVDS_STATUS_LOCKED)
/// Set tag as erased
#define NVDS_SET_TAG_ERASED(h) \
    ((((h).status) & (~NVDS_STATUS_ERASED_MASK)) | NVDS_STATUS_ERASED)
/// Set tag as locked
#define NVDS_SET_TAG_LOCKED(h) \
    ((((h).status) & (~NVDS_STATUS_LOCKED_MASK)) | NVDS_STATUS_LOCKED)
/// Set tag as valid
#define NVDS_SET_TAG_OK(h) \
    (NVDS_STATUS_VALID | NVDS_STATUS_NOT_LOCKED | NVDS_STATUS_NOT_ERASED)

#define NVDS_ALIGNMENT(p) CO_ALIGN4_HI(p)

/// Length of tag header
#define NVDS_TAG_HEADER_LENGTH \
    NVDS_ALIGNMENT(sizeof(struct nvds_tag_header))
/// Length of tag data
#define NVDS_TAG_CONTENT_LENGTH(h) \
    NVDS_ALIGNMENT((h).length)
/// Full length of tag (header+data)
#define NVDS_TAG_FULL_LENGTH(h) \
    NVDS_TAG_HEADER_LENGTH + NVDS_TAG_CONTENT_LENGTH(h)

/// Structure defining the header of a TAG.  It is very important that the TAG remains
/// the first element of the structure because it defines the LAST TAG of the NVDS when
/// set the oxFF.
struct nvds_tag_header
{
    /// current TAG identifier
    uint8_t  tag;
    /// status of the TAG (erased, locked ...)
    uint8_t  status;
    /// length of the TAG
    nvds_tag_len_t length;
};

#define _nvds_walk_tag  0x01001ebd
typedef uint8_t (*p_nvds_walk_tag)(uint32_t cur_tag_addr, struct nvds_tag_header *nvds_tag_header_ptr, uint32_t *nxt_tag_addr_ptr);
#define nvds_walk_tag ((p_nvds_walk_tag)(_nvds_walk_tag))

static void nvds_purge(uint32_t length, uint8_t* buf)
{
    uint8_t status;
    struct nvds_tag_header tag_hdr;
    uint32_t cur_tag_addr, nxt_tag_addr;
    uint32_t total_length;
    uint8_t *walk_ptr;
    uint8_t nvds_magic_number[NVDS_MAGIC_NUMBER_LENGTH] = {'N', 'V', 'D', 'S'};

    // store all the valid TAG elements in the locally allocated buffer
    total_length = 0;
    nxt_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS;
    walk_ptr = buf;
    do
    {
        // go to the next tag
        cur_tag_addr = nxt_tag_addr;

        status = nvds_walk_tag(cur_tag_addr, (struct nvds_tag_header*)&tag_hdr, &nxt_tag_addr);

        if ((status == NVDS_OK) && NVDS_IS_TAG_OK(tag_hdr))
        {
            // check that the current size is not overcoming the buffer
            total_length += NVDS_TAG_FULL_LENGTH(tag_hdr);
            //ASSERT_ERR(total_length <= length);

            // copy the header content
            *((struct nvds_tag_header*)walk_ptr) = tag_hdr;

            // increment the pointer to the data part
            walk_ptr += NVDS_TAG_HEADER_LENGTH;
            cur_tag_addr += NVDS_TAG_HEADER_LENGTH;

            // retrieve all the data part
            nvds_env->read((uint32_t)cur_tag_addr, (uint32_t)tag_hdr.length, walk_ptr);

            // increment the walking pointer
            walk_ptr += NVDS_TAG_CONTENT_LENGTH(tag_hdr);
        }

    } while (status == NVDS_OK);

    // clear the device
    nvds_env->erase((uint32_t)NVDS_MAGIC_NUMBER_ADDRESS, nvds_env->total_size);

    // rewrite the NVDS once cleaned
    nvds_env->write((uint32_t)NVDS_START_STORAGE_AREA_ADDRESS,
                   (uint32_t)total_length,
                   buf);

     // Write the magic number at address 0
    nvds_env->write((uint32_t)NVDS_MAGIC_NUMBER_ADDRESS,
                   (uint32_t)NVDS_MAGIC_NUMBER_LENGTH,
                   (uint8_t*)nvds_magic_number);
}

static uint8_t patch_nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
    uint8_t status;
    struct nvds_tag_header tag_hdr;
    uint8_t tag_buffer[NVDS_PARAMETER_MAX_LENGTH];
    uint32_t cur_tag_addr, nxt_tag_addr;
    uint8_t status_to_write;
    uint32_t total_length;
    int try = 0;

    /* parse once all the TAG elements of the NVDS to:
     *   1) find same tag
     *   2) erase and invalidate the former tag
     *   3) compute the total length needed by the all valid tags
     *   4) retrieve the first address where new data can be stored     */
    total_length = 0;
    nxt_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS;
    do
    {
        // Go to the next tag
        cur_tag_addr = nxt_tag_addr;

        // Read the next TAG header structure
        status = nvds_walk_tag(cur_tag_addr, &tag_hdr, &nxt_tag_addr);

        // check TAG is valid
        if ((status == NVDS_OK) && NVDS_IS_TAG_OK(tag_hdr))
        {
            // check TAG is identical to the new one
            if (tag_hdr.tag == tag)
            {
                // check TAG is not locked
                if (NVDS_IS_TAG_LOCKED(tag_hdr))
                {
                    return NVDS_PARAM_LOCKED;
                }

                // Read parameter data
                nvds_env->read((uint32_t)(cur_tag_addr + NVDS_TAG_HEADER_LENGTH),
                              (uint32_t)tag_hdr.length,
                              tag_buffer);

                // Compare data with new parameter
                if((tag_hdr.length == length) && !memcmp(buf, tag_buffer, tag_hdr.length))
                {
                    return NVDS_OK;
                }

                // then we set parameter to erased
                status_to_write = NVDS_SET_TAG_ERASED(tag_hdr);
                nvds_env->write((uint32_t)(cur_tag_addr+offsetof(struct nvds_tag_header, status)),
                               (uint32_t) sizeof(status_to_write),
                               (uint8_t*) &status_to_write);
            }
            else
            {
                // add the current tag length to the total length (used for purge)
                total_length += NVDS_TAG_FULL_LENGTH(tag_hdr);
            }
        }
    } while (status == NVDS_OK);

    // check that we've reached the last TAG of the NVDS
    if (status != NVDS_OK)
    {
        /* check if there is enough space to write next tag
           the limit is calculated including 2 TAG headers (the current and the next
           that is used to leave at least an end marker) */
        if ((cur_tag_addr + (NVDS_TAG_HEADER_LENGTH*2) + NVDS_ALIGNMENT(length))
             > (nvds_env->total_size))
        {
            // WJ : Check purge buffer
            //ASSERT_ERR(nvds_temp_buf != NULL);
            if((nvds_env->nvds_temp_buf==NULL) || (total_length > nvds_env->nvds_temp_buf_len))
            {
                return NVDS_NO_TEMP_BUF_AVAILABLE;
            }

            // purge the NVDS using the current buffer
            nvds_purge(total_length, nvds_env->nvds_temp_buf);

            // compute the next tag address in the NVDS memory space
            cur_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS + NVDS_ALIGNMENT(total_length);

            // if there is still not enough space, return an error
            if ((cur_tag_addr + NVDS_TAG_HEADER_LENGTH + NVDS_ALIGNMENT(length))
                 > (nvds_env->total_size - 1))
            {
                return NVDS_NO_SPACE_AVAILABLE;
            }
        }
    }

    // If try 2 times, return NVDS_CORRUPT directly.
    while(try++ < 2)
    {
        // First of all, write the data of the parameter
        nvds_env->write((uint32_t)(cur_tag_addr+NVDS_TAG_HEADER_LENGTH),
                   (uint32_t)length,
                   buf);

        nvds_env->read((uint32_t)(cur_tag_addr+NVDS_TAG_HEADER_LENGTH),
                       (uint32_t)length,
                       nvds_env->nvds_temp_buf);

        if(memcmp(buf, nvds_env->nvds_temp_buf, length) != 0)
        {
            // purge the NVDS, clean unavailable space
            nvds_purge(total_length, nvds_env->nvds_temp_buf);
            // compute the next tag address in the NVDS memory space
            cur_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS + NVDS_ALIGNMENT(total_length);
            continue;
        }

        // Second of all, configure the new value of the TAG HEADER
        tag_hdr.tag = tag;
        tag_hdr.status = NVDS_SET_TAG_OK(tag_hdr);
        tag_hdr.length = length;

        // Third of all, write the new TAG HEADER
        nvds_env->write((uint32_t)(cur_tag_addr),
                (uint32_t)sizeof(tag_hdr),
                (uint8_t*)&tag_hdr);

        nvds_env->read((uint32_t)(cur_tag_addr),
                       (uint32_t)sizeof(tag_hdr),
                       nvds_env->nvds_temp_buf);

        if(memcmp(&tag_hdr, nvds_env->nvds_temp_buf, sizeof(tag_hdr)) != 0)
        {
            tag_hdr.status = 0;
            nvds_env->write((uint32_t)(cur_tag_addr),
                (uint32_t)sizeof(tag_hdr),
                (uint8_t*)&tag_hdr);
            // purge the NVDS, clean unavailable space
            nvds_purge(total_length, nvds_env->nvds_temp_buf);
            // compute the next tag address in the NVDS memory space
            cur_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS + NVDS_ALIGNMENT(total_length);
            continue;
        }

        return(NVDS_OK);
    }

    return NVDS_CORRUPT;
}

uint8_t __nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
    uint8_t rt;
    bool flash_off = is_flash_off();

    if(flash_off)
        power_on_flash(FLASH_WR);

    rt = patch_nvds_put(tag, length, buf);

    if(flash_off)
        //fw_flash_off();
        power_off_flash();

    return rt;
}

typedef uint8_t (*p_nvds_get)(uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf);
#define fw_nvds_get ((p_nvds_get)(_nvds_get))
typedef uint8_t (*p_nvds_del)(uint8_t tag);
#define fw_nvds_del ((p_nvds_del)(_nvds_del))
typedef uint8_t (*p_nvds_lock)(uint8_t tag);
#define fw_nvds_lock ((p_nvds_lock)(_nvds_lock))

uint8_t __nvds_get(uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf)
{
    uint8_t rt;
    bool flash_off = is_flash_off();

    if(flash_off)
        power_on_flash(FLASH_RD);

    rt = fw_nvds_get(tag, lengthPtr, buf);

    if(flash_off)
        //fw_flash_off();
        power_off_flash();

    return rt;
}

uint8_t __nvds_del(uint8_t tag)
{
    uint8_t rt;
    bool flash_off = is_flash_off();

    if(flash_off)
        power_on_flash(FLASH_WR);

    rt = fw_nvds_del(tag);

    if(flash_off)
        //fw_flash_off();
        power_off_flash();

    return rt;
}

uint8_t __nvds_lock(uint8_t tag)
{
    uint8_t rt;
    bool flash_off = is_flash_off();

    if(flash_off)
        power_on_flash(FLASH_WR);

    rt = fw_nvds_lock(tag);

    if(flash_off)
        //fw_flash_off();
        power_off_flash();

    return rt;
}
#endif

typedef void (*p_save_cal_setting)(void);
#define save_cal_setting ((p_save_cal_setting)(_save_cal_setting))

/// Power up calibration
static void powerup_calibration(enum CLK_TYPE cry_type)
{
    int32_t ppm;

    /*
     **************************
     * RF calibartion
     **************************
     */

    calibration_init(cry_type);

    // Do not set dis, prevent software reset
    cal_cal_SetCAL0WithMask(QN_CALIB,
                            (CALIB_MASK_REF_CAL_DIS
                            | CALIB_MASK_RC_CAL_DIS),
                            MASK_DISABLE);

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS, MASK_DISABLE);

    cal_cal_SetCAL3WithMask(QN_CALIB,
                            (CALIB_MASK_ROS_CAL_Q_DIS
                            | CALIB_MASK_ROS_CAL_I_DIS
                            | CALIB_MASK_R_CAL_DIS),
                            MASK_DISABLE);

    dp_dp_SetRegWithMask(0x78, 0x40000000, 0x0);
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RX_PKDET, MASK_ENABLE);

    // REF PLL
    //ref_pll_calibration();
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_REF_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_REF_CAL_REQ, MASK_ENABLE);
    while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));    // Wait for REF PLL ready

    // RC->LO->GMR->PA(skip)
    seq_calibration(0);
    // write back LO code
    cal_cal_SetCAL1(QN_CALIB, cal_cal_GetCAL1(QN_CALIB));

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS|CALIB_MASK_LO_FCAL_DIS, MASK_ENABLE);
    delay(500);

    // LO2->KVCO->DC
    freq_hop_calibration(1, 0);
    // write back kvco code
    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_KCAL, cal_cal_GetCAL1(QN_CALIB));

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS|CALIB_MASK_LO_FCAL_DIS, MASK_DISABLE);

    // Set PA code
    cal_cal_SetCAL4WithMask(QN_CALIB,
                            (CALIB_MASK_PA_CODE_TX
                            | CALIB_MASK_PA_CODE_RX),
                            ((0xf<<CALIB_POS_PA_CODE_TX)
                            | (0x0<<CALIB_POS_PA_CODE_RX)));

    // PA auto mode.
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CAL_EN, MASK_DISABLE);

    // Dis REF PLL
    // Dis RC calibration
    // Do not Skip LO calibration
    // skip KVCO calibration when channel switch
    // skip ROS calibration when channel switch
    cal_cal_SetCAL0WithMask(QN_CALIB,
                            ( CALIB_MASK_REF_CAL_DIS
                            | CALIB_MASK_RC_CAL_DIS
                            | CALIB_MASK_LO_CAL_SKIP
                            | CALIB_MASK_LO_KCAL_SKIP
                            | CALIB_MASK_CAL_DONE_DIS),
                            (CALIB_MASK_REF_CAL_DIS
                            | CALIB_MASK_RC_CAL_DIS
                            //| CALIB_MASK_LO_CAL_SKIP
                            | CALIB_MASK_LO_KCAL_SKIP
                            | CALIB_MASK_CAL_DONE_DIS)
                            );

    // Disable LO amplitude cal
    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS, MASK_ENABLE);

    // Increase delta-f2
    cal_cal_SetCAL2WithMask(QN_CALIB,
                            (CALIB_MASK_LO_KDAC_E | CALIB_MASK_TX_DLY1),
                            ((0x1<<CALIB_POS_LO_KDAC_E) | (0x2)));

    // Disable Q & I channel ROS calibration
    // Disable R cal
    cal_cal_SetCAL3WithMask(QN_CALIB,
                            (CALIB_MASK_ROS_CAL_Q_DIS
                            | CALIB_MASK_ROS_CAL_I_DIS
                            | CALIB_MASK_R_CAL_DIS),
                            MASK_ENABLE);

    /*
    **************************
    * 32K clock calibartion
    * 32k xtal - set fixed ppm here
    * 32k rco  - shall do calibration periodically
    **************************
    */

    // 32.768k crystal or internal 32k rco
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, MASK_ENABLE);   // 32k RCO

    //RCO CAL - 32k
    rco_calibration();

    // 32.768k crystal or internal 32k rco
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, MASK_DISABLE);  // 32k XTAL

    // Enable Timer1 for 32k calibration
    //syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_TIMER1);

    // Calculate PPM of 32k clock
    //ppm = clock_32k_calibration(XTAL_16M, 16);
    ppm = -0x6000;

    // Set ppm for sleep compensation of BLE stack
    set_32k_ppm(ppm);

    // Disable Timer for 32k calibration
    //syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER1);

    /*
     **************************
     * Save calibration result
     **************************
     */
    save_cal_setting();

    // disable RF set by SW, enable set by HW
    rf_enable_sw_set_freq(MASK_DISABLE);
    rf_enable(RF_TX, MASK_DISABLE, MASK_DISABLE, 0x0);

    // Enable SD ADC RST again 2013 04 07 ljb
    dp_dp_SetRegWithMask(0x78, 0x40000000, ~0x0);
}

/**
 ****************************************************************************************
 * @brief Initilaize BLE hardware platform
 * @param[in]   pw_mode             NORMAL_MODE - low power; HIGH_PERFORMANCE - high power
 * @param[in]   xtal                which crystal is used - 16MHz or 32MHz
 * @param[in]   clk_32k             which 32k is used - 0:xtal, 1:rco
 * @param[in]   nvds_tmp_buf        pointer of nvds temp buffer
 * @param[in]   nvds_tmp_buf_len    length of nvds temp buffer
 ****************************************************************************************
 */
void plf_init(enum PW_MODE pw_mode, uint32_t xtal, uint8_t clk_32k, uint8_t* nvds_tmp_buf, uint32_t nvds_tmp_buf_len)
{
    uint32_t mask, val;
    uint8_t xcsel;
    nvds_tag_len_t length;


    /*
     **************************
     * Step 1. System Clock
     **************************
     */

    // external 16M/32M clk ready
    while(!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_RDY));
    // High frequency crystal
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_XTAL);

    // Switch on REF PLL power
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_REF_PLL, MASK_DISABLE);
    // PLL ready
    while(!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));

    if(clk_32k == 0)
    {
        // ensable schmitt trigger in 32.768KHz buffer
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32SMT_EN, MASK_ENABLE);
        // Set 32.768KHz xtal higher current, that can let the 32k xtal stable fastly (decrease stable time).
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32ICTRL, 32);

#if 0
        // wait for 32k xtal ready
        while(!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_XTAL32_RDY));

        // disable schmitt trigger in 32.768KHz buffer
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32SMT_EN, MASK_DISABLE);
        // Set 32.768KHz xtal to normal current
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32ICTRL, 16);
#endif
    }

    /*
     **************************
     * Step 2. QN9020 patch
     **************************
     */

    // Clean debug information register
    if(*(volatile uint32_t*)(0x10000000) != 0xf2f2f2f2)
    {
        *(volatile uint32_t*)(QN_DBG_INFO_REG) = 0x0;
    }

    // Register 0x400000a8
    // Analog XTAL setting
    // LO PLL CP current : 0.7x
    // LC VCO amplitude settting: VTH+50mV
    val = (xtal>XTAL_16MHz)?0x80000000:0;
    syscon_SetLO1WithMask(QN_SYSCON,
                            (SYSCON_MASK_XDIV | SYSCON_MASK_LO_TST_CP | SYSCON_MASK_LO_VCO_AMP),
                            (val | (0x2<<SYSCON_POS_LO_TST_CP) | (0x1<<SYSCON_POS_LO_VCO_AMP)));

    // Register 0x400000a0
    // BGSEL = 0000
    // 1.5v regulator for analog output voltage - 1.5v
    // Enable TR switch
    // VREG12_D = 00
    // Enable switch between dvdd12_core and dvdd12_pmu
    // BUCK_DPD
    // 32.768kHz crystal bias current control 100000 -> 001000
    // BUCK VBG = 0x1
    mask = SYSCON_MASK_BGSEL
         | SYSCON_MASK_VREG15
         | SYSCON_MASK_VREG12_A
         | SYSCON_MASK_TR_SWITCH
         | SYSCON_MASK_VREG12_D
         | SYSCON_MASK_DVDD12_SW_EN
         | SYSCON_MASK_BUCK_DPD
         | SYSCON_MASK_X32ICTRL
         | SYSCON_MASK_BUCK_VBG;
    if(dc_dc_flag)
    {
        // dc-dc
        val = (0X0<<SYSCON_POS_BGSEL)
            | (0x1<<SYSCON_POS_VREG15)
            | (0x1<<SYSCON_POS_VREG12_A)
            | (SYSCON_MASK_TR_SWITCH)
            | (0x0<<SYSCON_POS_VREG12_D)
            | (SYSCON_MASK_DVDD12_SW_EN)
            | (0x1<<SYSCON_POS_BUCK_VBG)
            | (0x1<<16)
            | (0x10);
    }
    else
    {
        val = (0X0<<SYSCON_POS_BGSEL)
            | (0x1<<SYSCON_POS_VREG15)
            | (0x1<<SYSCON_POS_VREG12_A)
            | (SYSCON_MASK_TR_SWITCH)
            | (0x0<<SYSCON_POS_VREG12_D)
            | (SYSCON_MASK_DVDD12_SW_EN)
            | (0x2<<SYSCON_POS_BUCK_VBG)
            | (0x1<<16)
            | (0x10);
    }
    syscon_SetIvrefX32WithMask(QN_SYSCON, mask, val);

    // Register 0x40000098
    // VREG12_A_BAK = 00
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A_BAK, (0x1<<SYSCON_POS_VREG12_A_BAK));

    // Register 0x4000009c
    // PA BM = 00
    // PA gain 2dbm
    mask = SYSCON_MASK_PA_GAIN_BOOST
         | SYSCON_MASK_BM_PA
         | SYSCON_MASK_PA_GAIN
         | SYSCON_MASK_VT_PKDET2
         | SYSCON_MASK_VT_PKDET3;

    if(dc_dc_flag)
    {
        // To achive a good performance of TX at 0dBm, Zhang Shun 2013-12-13
        val = (0x0<<SYSCON_POS_PA_GAIN_BOOST)
            | (0x0<<SYSCON_POS_BM_PA)
            | (0xe<<SYSCON_POS_PA_GAIN)
            | SYSCON_MASK_VT_PKDET2
            | SYSCON_MASK_VT_PKDET3;
    }
    else
    {
        val = (0x0<<SYSCON_POS_PA_GAIN_BOOST)   //diable gain boost in no dcdc mode from zhangshun
            | (0x2<<SYSCON_POS_BM_PA)           //change to '10' increase the PA power from zhangshun 20130821
            | (0xe<<SYSCON_POS_PA_GAIN)
            | SYSCON_MASK_VT_PKDET2
            | SYSCON_MASK_VT_PKDET3;
    }
    syscon_SetGCRWithMask(QN_SYSCON, mask, val);

    // Register 0x400000a4
    // XTAL speed up
    // buck tmos = 111
    // buck bm = 10
    // REFPLL CP current 1.3x
    mask = SYSCON_MASK_XSP_CSEL_B0
         | SYSCON_MASK_BUCK_TMOS
         | SYSCON_MASK_BUCK_BM
         | SYSCON_MASK_TST_CPREF;
    if(dc_dc_flag)
    {
        val = 0x80000000
            | (0X6<<SYSCON_POS_BUCK_TMOS) //change from 7 to 6 to reduce current and reduce sensitivity from yangyu
            | (0X2<<SYSCON_POS_BUCK_BM)
            | 0x8;

    }
    else
    {
        val = 0x80000000
            | (0X5<<SYSCON_POS_BUCK_TMOS) //change from 7 to 5 from test.
            | (0X2<<SYSCON_POS_BUCK_BM)
            | 0x8;
    }
    syscon_SetXtalBuckWithMask(QN_SYSCON, mask, val);

    // Register 0x400000b0
    // BM_PPF = 00
    // ADC_OPA12I = 00
    // ADC_OPA4I = 00
    mask = SYSCON_MASK_BM_LNA_PK
         | SYSCON_MASK_BM_PPF
         | SYSCON_MASK_EN_DPF_DIS
         | SYSCON_MASK_ADC_OPA12I
         | SYSCON_MASK_ADC_OPA4I;
    if(dc_dc_flag)
    {
        val = (0x1<<SYSCON_POS_BM_LNA_PK)
            | (0x0<<SYSCON_POS_BM_PPF)
            | SYSCON_MASK_EN_DPF_DIS
            | (0x0<<SYSCON_POS_ADC_OPA12I)
            | (0x0<<SYSCON_POS_ADC_OPA4I);
    }
    else
    {
        val = (0x1<<SYSCON_POS_BM_LNA_PK)
            | (0x2<<SYSCON_POS_BM_PPF) //change to '10' to increase the current and sensitivity from yangyu
            | SYSCON_MASK_EN_DPF_DIS
            |(0x0<<SYSCON_POS_ADC_OPA12I)
            |(0x0<<SYSCON_POS_ADC_OPA4I);
    }
    syscon_SetRXCRWithMask(QN_SYSCON, mask, val);

    // Register 0x400000b8
    // Speed up XTAL
    // B2 : dc-dc 400000B8[6]=1 otherwise 400000B8[6]=1
    mask = SYSCON_MASK_XSP_CSEL_B1
           | SYSCON_MASK_PA_GAIN_BIT4_B1;

    if(dc_dc_flag)
    {
        val = SYSCON_MASK_XSP_CSEL_B1
            | SYSCON_MASK_PA_GAIN_BIT4_B1;
    }
    else
    {
        val = SYSCON_MASK_XSP_CSEL_B1;
    }

    syscon_SetAnalogCRWithMask(QN_SYSCON, mask, val);

    // Register 0x400000bc
    // buck tmos back (tx) = 111
    // buck bm back (tx) = 10
    // Half LO CP OPAMP current
    // DIS XPD DLY = 1 to fix the slow power down time of vreg-a
    // PA_CK enable at the end of SPEED_UP
    syscon_SetAdditionCRWithMask(QN_SYSCON,
                                (SYSCON_MASK_BUCK_TMOS_BAK
                                | SYSCON_MASK_BUCK_BM_BAK
                                | SYSCON_MASK_HALF_LO_OPCUR
                                //| SYSCON_MASK_XADD_C
                                | SYSCON_MASK_DIS_XPD_DLY
                                | SYSCON_MASK_PA_CKEN_SEL
                                | SYSCON_MASK_DC_CAL_MODE),
                                ((0x7<<SYSCON_POS_BUCK_TMOS_BAK)
                                | (0x2<<SYSCON_POS_BUCK_BM_BAK)
                                | 0x200
                                //| 0x8
                                | 0x4
                                | 0x2
                                | 0x0)); // WJ ???

    // Register 0x4000c010
    // PLL settling time in KVCO calibration: 24us
    // Speed up time in KVCO calibration: 8us
    cal_cal_SetCAL4WithMask(QN_CALIB,
                            (CALIB_MASK_PLL_RDY_DLY |CALIB_MASK_LO_SU_DLY),
                            ((0x18<<CALIB_POS_PLL_RDY_DLY) | (0x8<<CALIB_POS_LO_SU_DLY)));

    // Datapath

    // Register 0x4000b000
    // increase delta-f2
    dp_dp_SetRegWithMask(0x00, 0x00050000, 0x00050000);

    // resampler_bp = 1, track length = 28
    dp_dp_SetRegWithMask(0x30, 0x808000, 0x808000);

    // Register 0x4000b04c
    // cordic min vin threshold
    dp_dp_SetRegWithMask(0x4c, 0x0f000800, 0x0);

    // Register 0x4000b078
    // bit[16] do not reset sdadc after fsinc
    // bit[0:1] rf agc adc reset delay
    dp_dp_SetRegWithMask(0x78, 0x00013000, 0x11000);

    dp_dp_SetRegWithMask(0x84,0x00000026,0x26);

    /*
     **************************
     * Step 3. Platform initialization ()
     *         nvds, rf, deep sleep, qn config
     **************************
     */
    // qn_plf_init()

    nvds_patch(nvds_tmp_buf, nvds_tmp_buf_len);
#if 0
    rf_init((struct rf_api*)_ble_rf);
    ((struct rf_api*)_ble_rf)->force_agc_enable = foo1;

    sleep_patch();

    qn_config_init(config_api);

    config_api->hci_enter_sleep = default_enter_sleep_cb;
    config_api->hci_exit_sleep = default_exit_sleep_cb;

    config_api->max_sleep_duration_periodic_wakeup = 0x320; // 0.5s

    config_api->prefetch_time = 145;

    config_api->plf_reset_cb = plf_reset_cb;
#endif
    // XTAL load capacitance
    length = sizeof(uint8_t);
    if (nvds_get(NVDS_TAG_XCSEL, &length, &xcsel) != NVDS_OK)
    {
        xcsel = 0x11;   // default value
    }
    syscon_SetXtalBuckWithMask(QN_SYSCON, SYSCON_MASK_XCSEL, xcsel<<SYSCON_POS_XCSEL);

    /*
     **************************
     * Step 4. Initialize BLE hardware
     **************************
     */

    // calibration
    //val &= 1;
    val = (xtal>XTAL_16MHz)?XTAL_32M:XTAL_16M;
    powerup_calibration((enum CLK_TYPE)val);

    mask = SYSCON_MASK_DIS_REF_PLL
         | SYSCON_MASK_DIS_LO_VCO
         | SYSCON_MASK_DIS_LO_PLL
         | SYSCON_MASK_DIS_PA
         | SYSCON_MASK_DIS_LNA
         | SYSCON_MASK_DIS_LNA_PKDET
         | SYSCON_MASK_DIS_MIXER
         | SYSCON_MASK_DIS_PPF_PKDET
         | SYSCON_MASK_DIS_PPF
         | SYSCON_MASK_DIS_RX_PKDET
         | SYSCON_MASK_DIS_RX_ADC
         | SYSCON_MASK_DIS_RCO
         | SYSCON_MASK_DIS_SAR_ADC
         | SYSCON_MASK_DIS_SAR_BUF;

    // Switch off REF PLL/LO_VCO/LO PLL/PA/LNA/LNA PKDET/MIXER... power
    syscon_SetPGCR1WithMask(QN_SYSCON, mask, MASK_ENABLE);

    //set random seed
    //srand((unsigned)QN_TIMER0->CNT);
}

/**
 ****************************************************************************************
 * @brief Check that BLE has already waked-up
 ****************************************************************************************
 */
uint32_t check_ble_wakeup(void)
{
    return (__rd_reg(0x2f000014) & 0x4);
}
