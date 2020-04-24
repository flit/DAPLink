/**
 * @file    target_flash.c
 * @brief   Implementation of target_flash.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifdef DRAG_N_DROP_SUPPORT
#include "string.h"

#include "target_config.h"
#include "target_reset.h"
#include "gpio.h"
#include "target_config.h"
#include "intelhex.h"
#include "swd_host.h"
#include "flash_intf.h"
#include "util.h"
#include "settings.h"
#include "target_family.h"
#include "target_board.h"

typedef enum {
    STATE_CLOSED,
    STATE_OPEN,
    STATE_ERROR
} state_t;

static error_t target_flash_init(void);
static error_t target_flash_uninit(void);
static error_t target_flash_program_page(uint32_t adr, const uint8_t *buf, uint32_t size);
static error_t target_flash_erase_sector(uint32_t addr);
static error_t target_flash_erase_chip(void);
static uint32_t target_flash_program_page_min_size(uint32_t addr);
static uint32_t target_flash_erase_sector_size(uint32_t addr);
static uint8_t target_flash_busy(void);
static error_t target_flash_set(uint32_t addr);

static const flash_intf_t flash_intf = {
    target_flash_init,
    target_flash_uninit,
    target_flash_program_page,
    target_flash_erase_sector,
    target_flash_erase_chip,
    target_flash_program_page_min_size,
    target_flash_erase_sector_size,
    target_flash_busy,
    target_flash_set,
};

#if defined(K3S_DEBUG)
extern uint32_t my_second_flash_blob[];  // nxa07750
extern uint32_t K32W0x2_P256_2KB_SEC_flash_prog_blob[];

static uint8_t write_k3s_flash(uint32_t addr, uint8_t * data);
#endif

static state_t state = STATE_CLOSED;

const flash_intf_t *const flash_intf_target = &flash_intf;

static flash_func_t last_flash_func = FLASH_FUNC_NOP;

//saved flash algo
static program_target_t * current_flash_algo = NULL;

//saved default region for default flash algo
static region_info_t * default_region = NULL;

//saved flash start from flash algo
static uint32_t flash_start = 0;

static program_target_t * get_flash_algo(uint32_t addr)
{
    region_info_t * flash_region = g_board_info.target_cfg->flash_regions;

    for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) { // Original
	  //for (; flash_region->start != 0 || flash_region->end != 0; flash_region++) {
        if (addr >= flash_region->start && addr <= flash_region->end) {
            flash_start = flash_region->start; //save the flash start
            if (flash_region->flash_algo) {
                return flash_region->flash_algo;
            }else{
                return NULL;
            }
        }
    }
    
    //could not find a flash algo for the region; use default
    if (default_region) {
        flash_start = default_region->start;
        return default_region->flash_algo;
    } else {
        return NULL;
    }
}

static error_t flash_func_start(flash_func_t func)
{
    program_target_t * flash = current_flash_algo;

    if (last_flash_func != func)
    {
        // Finish the currently active function.
        if (FLASH_FUNC_NOP != last_flash_func &&
            0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->uninit, last_flash_func, 0, 0, 0)) {
            return ERROR_UNINIT;
        }

        // Start a new function.
        if (FLASH_FUNC_NOP != func &&
            0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->init, flash_start, 0, func, 0)) {
            return ERROR_INIT;
        }

        last_flash_func = func;
    }

    return ERROR_SUCCESS;
}

static error_t target_flash_set(uint32_t addr)
{
    program_target_t * new_flash_algo = get_flash_algo(addr);
    if (new_flash_algo == NULL) {
        return ERROR_ALGO_MISSING;
    }
    if(current_flash_algo != new_flash_algo){
        //run uninit to last func
        error_t status = flash_func_start(FLASH_FUNC_NOP);
        if (status != ERROR_SUCCESS) {
            return status;
        }
        // Download flash programming algorithm to target
        if (0 == swd_write_memory(new_flash_algo->algo_start, (uint8_t *)new_flash_algo->algo_blob, new_flash_algo->algo_size)) {
            return ERROR_ALGO_DL;
        }
        
        current_flash_algo = new_flash_algo;
        
    }
    return ERROR_SUCCESS;
}

static error_t target_flash_init()
{
    if (g_board_info.target_cfg) {
        last_flash_func = FLASH_FUNC_NOP;
        
        current_flash_algo = NULL;
        
        if (0 == target_set_state(RESET_PROGRAM)) {
            return ERROR_RESET;
        }
        
        //get default region
        region_info_t * flash_region = g_board_info.target_cfg->flash_regions;
        for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
            if (flash_region->flags & kRegionIsDefault) {
                default_region = flash_region;
                break;
            }
        }

        state = STATE_OPEN;
        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
    
}

static error_t target_flash_uninit(void)
{
    if (g_board_info.target_cfg) {
        error_t status = flash_func_start(FLASH_FUNC_NOP);
        if (status != ERROR_SUCCESS) {
            return status;
        }
        if (config_get_auto_rst()) {
            // Resume the target if configured to do so
            target_set_state(RESET_RUN);
        } else {
            // Leave the target halted until a reset occurs
            target_set_state(RESET_PROGRAM);
        }
        // Check to see if anything needs to be done after programming.
        // This is usually a no-op for most targets.
        target_set_state(POST_FLASH_RESET);
      
        state = STATE_CLOSED;
        swd_off();
        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_program_page(uint32_t addr, const uint8_t *buf, uint32_t size)
{
#if defined(K3S_DEBUG)
	uint8_t targetAlgo = 0, flashAlgoBit = 0;	// nxa07750
	uint32_t i, j, tmp32; // nxa07750
    uint8_t writeBuf[8];
#endif
    if (g_board_info.target_cfg) {
        error_t status = ERROR_SUCCESS;
        program_target_t * flash = current_flash_algo;
        
        if (!flash) {
            return ERROR_INTERNAL;
        }
        
        // check if security bits were set
        if (g_target_family && g_target_family->security_bits_set){
            if (1 == g_target_family->security_bits_set(addr, (uint8_t *)buf, size)) {
                return ERROR_SECURITY_BITS;
            }
        }

        status = flash_func_start(FLASH_FUNC_PROGRAM);

        if (status != ERROR_SUCCESS) {
            return status;
        }
        
        while (size > 0) {
            uint32_t write_size = MIN(size, flash->program_buffer_size);
#if defined(K3S_DEBUG)
            if(addr < 0x01000000)
            {
#endif
                // Write page to buffer
                if (!swd_write_memory(flash->program_buffer, (uint8_t *)buf, write_size)) {
                    return ERROR_ALGO_DATA_SEQ;
                }
                
#if defined(K3S_DEBUG)
            }
#endif
						
#if defined(K3S_DEBUG)
						// nxa07750
						if(addr >= 0x01000000)
						{
                            j = 0;
                            
                            while(j < write_size){
                                // First populate the write buffer array
                                for(i=0; i<8; i++){
                                    if(j >= write_size){
                                        writeBuf[i] = 0;
                                    }
                                    else{
                                        writeBuf[i] = buf[j];
                                    }
                                    j++;
                                }
                                
                                // Now write to the K3S flash
                                // Address increment is j-8 because j is essentially pre-incremented to keep 
                                // up with the bytes that are being written.  
                                if(!write_k3s_flash((addr + j - 8), writeBuf)){
                                    return ERROR_WRITE;
                                }
                            }
                            // Uncomment this to verify flash algorithm was written correctly
							//for(j=0; j<200; j++)
//							for(j=0; j<(856*4); j++)
//							{
//								if(!swd_read_memory(0x20000000+j, &targetAlgo, 1))
//								{
//									return ERROR_ALGO_DATA_SEQ;
//								}
//							
////								flashAlgoBit = (my_second_flash_blob[j/4] & (0xFF << ((j%4)*8)));
//								//tmp32 = my_second_flash_blob[j/4];
//								//tmp32 = *((uint32_t *)my_second_flash_blob);
//								tmp32 = K32W0x2_P256_2KB_SEC_flash_prog_blob[j/4];
//								flashAlgoBit = (tmp32 & (0xFF << ((j%4)*8))) >> ((j%4)*8);
//								if(targetAlgo != flashAlgoBit)
//								{
//									//return ERROR_ALGO_DATA_SEQ;
//                                    if(!swd_write_byte(0x20000000+j, flashAlgoBit))
//                                    {
//                                        return ERROR_ALGO_DATA_SEQ;
//                                    }
//								}
//							
//							}
						} // nxa07750
                        else
                        {
                        #endif

            // Run flash programming
            if (!swd_flash_syscall_exec(&flash->sys_call_s,
                                        flash->program_page,
                                        addr,
                                        write_size,
                                        flash->program_buffer,
                                        0)) {
                return ERROR_WRITE;
            }
                                        
#if defined (K3S_DEBUG)
        }
#endif

            if (config_get_automation_allowed()) {
                // Verify data flashed if in automation mode
                if (flash->verify != 0) {
//                    status = flash_func_start(FLASH_FUNC_VERIFY);
//                    if (status != ERROR_SUCCESS) {
//                        return status;
//                    }
//                    if (!swd_flash_syscall_exec(&flash->sys_call_s,
//                                        flash->verify,
//                                        addr,
//                                        write_size,
//                                        flash->program_buffer,
//                                        0)) {
//                        return ERROR_WRITE_VERIFY;
//                    }
//                } else {
                    while (write_size > 0) {
                        uint8_t rb_buf[16];
												uint32_t rb_buf_size = sizeof(rb_buf);
                        //uint32_t verify_size = MIN(write_size, sizeof(rb_buf));
											  uint32_t verify_size = MIN(write_size, rb_buf_size);
                        if (!swd_read_memory(addr, rb_buf, verify_size)) {
                            return ERROR_ALGO_DATA_SEQ;
                        }
                        if (memcmp(buf, rb_buf, verify_size) != 0) {
                            return ERROR_WRITE_VERIFY;
                        }
                        addr += verify_size;
                        buf += verify_size;
                        size -= verify_size;
                        write_size -= verify_size;
                    }
                    continue;
                }
            }
            addr += write_size;
            buf += write_size;
            size -= write_size;
            
        }

        return ERROR_SUCCESS;
        
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_erase_sector(uint32_t addr)
{
    if (g_board_info.target_cfg) {
        error_t status = ERROR_SUCCESS;
        program_target_t * flash = current_flash_algo;

        if (!flash) {
            return ERROR_INTERNAL;
        }
        
        // Check to make sure the address is on a sector boundary
        if ((addr % target_flash_erase_sector_size(addr)) != 0) {
            return ERROR_ERASE_SECTOR;
        }

        status = flash_func_start(FLASH_FUNC_ERASE);

        if (status != ERROR_SUCCESS) {
            return status;
        }
        
        if (0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->erase_sector, addr, 0, 0, 0)) {
            return ERROR_ERASE_SECTOR;
        }

        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_erase_chip(void)
{
    if (g_board_info.target_cfg){
        error_t status = ERROR_SUCCESS;
        region_info_t * flash_region = g_board_info.target_cfg->flash_regions;

        for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
            status = target_flash_set(flash_region->start); 
            if (status != ERROR_SUCCESS) {
                return status;
            }
            status = flash_func_start(FLASH_FUNC_ERASE);
            if (status != ERROR_SUCCESS) {
                return status;
            }
            if (0 == swd_flash_syscall_exec(&current_flash_algo->sys_call_s, current_flash_algo->erase_chip, 0, 0, 0, 0)) {
                return ERROR_ERASE_ALL;
            }
        }

        // Reset and re-initialize the target after the erase if required
        if (g_board_info.target_cfg->erase_reset) {
            status = target_flash_init();
        }

        return status;
    } else {
        return ERROR_FAILURE;
    }
}

static uint32_t target_flash_program_page_min_size(uint32_t addr)
{
    if (g_board_info.target_cfg){
        uint32_t size = 256;
        if (size > target_flash_erase_sector_size(addr)) {
            size = target_flash_erase_sector_size(addr);
        }
        return size;
    } else {
        return 0;
    }
}

static uint32_t target_flash_erase_sector_size(uint32_t addr)
{
    if (g_board_info.target_cfg){
        if(g_board_info.target_cfg->sector_info_length > 0) { 
            int sector_index = g_board_info.target_cfg->sector_info_length - 1;
            for (; sector_index >= 0; sector_index--) {
                if (addr >= g_board_info.target_cfg->sectors_info[sector_index].start) {
                    return g_board_info.target_cfg->sectors_info[sector_index].size;
                }
            }
        }
        //sector information should be in sector_info
        util_assert(0);
        return 0;
    } else {
        return 0;
    }
}

static uint8_t target_flash_busy(void){
    return (state == STATE_OPEN);
}
#endif

#if defined(K3S_DEBUG)
static uint8_t write_k3s_flash(uint32_t addr, uint8_t * data)
{
    uint8_t statVal = 0;
    
    // put code here
    // First write STAT register
    if(!swd_write_byte(0x40023000, 0x70))
    {
        return 0;
    }
    
    // Next write CCOB registers
    swd_write_byte(0x40023007, 0x07);                               // FCCOB 0
    swd_write_byte(0x40023006, (((addr & 0xFF0000) >> 16) | 0x80)); // FCCOB 1:  ADD[23:16]
    swd_write_byte(0x40023005, ((addr & 0xFF00) >> 8));             // FCCOB 2   ADD[15:8]
    swd_write_byte(0x40023004, ((addr & 0xFF) >> 0));               // FCCOB 3   ADD[7:0]
    
    // Next write CCOB registers
    swd_write_byte(0x4002300B, data[3]); // FCCOB 4
    swd_write_byte(0x4002300A, data[2]); // FCCOB 5
    swd_write_byte(0x40023009, data[1]); // FCCOB 6  
    swd_write_byte(0x40023008, data[0]); // FCCOB 7   
    
    swd_write_byte(0x4002300F, data[7]); // FCCOB 8
    swd_write_byte(0x4002300E, data[6]); // FCCOB 9
    swd_write_byte(0x4002300D, data[5]); // FCCOB A
    swd_write_byte(0x4002300C, data[4]); // FCCOB B
    
    // Now launch the command
    swd_write_byte(0x40023000, 0x80);
    
    // Wait for the command to finish
    while(!statVal)
    {
        swd_read_byte(0x40023000, &statVal);
    }
    
    // Read STAT register again
    swd_read_byte(0x40023000, &statVal);
    
    // Check for errors
    if(statVal & 0x70)
    {
        // had an error
        return 0;
    }
    
    return 1;
}
#endif
