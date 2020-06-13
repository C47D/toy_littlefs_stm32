/*
 * Winbond_25Q.h
 *
 *  Created on: 16/05/2020
 *      Author: d3bug
 */

#ifndef WINBOND_25Q_H_
#define WINBOND_25Q_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLASH_MEMORY_ACCESS_SPI_SINGLE
#define FLASH_MEMORY_ACCESS_SPI_DUAL
#define FLASH_MEMORY_ACCESS_SPI_QUAD
#define FLASH_MEMORY_ACCESS_QPI

#define PAGE_SIZE	256

#define STATUS_REGISTER_1_BUSY_POS	(0)
#define STATUS_REGISTER_1_BUSY		(1 << STATUS_REGISTER_1_BUSY_POS)

uint32_t spi_write_enable(void);
uint32_t spi_write_enable_volatile_status_regiter(void);
uint32_t spi_write_disable(void);

/* IDs */
uint32_t spi_get_manu_device_id_single(void);
uint32_t spi_get_manu_device_id_dual(void);
uint32_t spi_get_manu_device_id_quad(void);
uint32_t spi_get_jedec_id(void);

/* Read/Write */
uint32_t spi_read_data(uint32_t addr, uint8_t *buffer, size_t buffer_sz);
uint32_t spi_fast_read_data(uint32_t addr, uint8_t *buffer, size_t buffer_sz);
uint32_t spi_fast_read_data_dual(uint32_t addr, uint8_t *buffer, size_t buffer_sz);
uint32_t spi_fast_read_data_dual_io(uint32_t addr, uint8_t *buffer, size_t buffer_sz);
uint32_t spi_fast_read_data_quad(uint32_t addr, uint8_t *buffer, size_t buffer_sz);
uint32_t spi_fast_read_data_quad_io(uint32_t addr, uint8_t *buffer, size_t buffer_sz);
uint32_t spi_page_program(uint32_t offset, const uint8_t *buffer, size_t buffer_sz);

/* Erase operations */
uint32_t spi_erase_sector(uint32_t addr);
uint32_t spi_erase_block_32kb(uint32_t addr);
uint32_t spi_erase_block_64kb(uint32_t addr);
uint32_t spi_erase_chip(void);

/* Suspend erase or program operations */
uint32_t spi_suspend();
uint32_t spi_resume();

/* Power */
uint32_t spi_power_down();
uint32_t spi_power_down_release();

/* Security registers */

uint32_t spi_lock_sector_block(uint32_t addr);
uint32_t spi_unlock_sector_block(uint32_t addr);
uint32_t spi_lock_global();
uint32_t spi_unlock_global();

uint32_t spi_enable_reset();
uint32_t spi_reset();

uint32_t spi_enter_qpi_mode(void);
uint32_t qpi_exit_qpi_mode(void);
uint32_t qpi_read_jedec_id(void);

/* Status registers */
uint8_t spi_read_status_1(void);
uint8_t spi_write_status_1(uint8_t status1_data);
uint8_t spi_read_status_2(void);
uint8_t spi_write_status_2(uint8_t status2_data);
uint8_t spi_read_status_3(void);
uint8_t spi_write_status_3(uint8_t status3_data);

/* allows from one byte to 256 bytes (a page, PAGE_SIZE)
 * of data to be programmed at prevously erased (FFh)
 * memory allocations. A write enable instruction must be
 * executed before the device will accept the page program
 * instruction (Status Regiter bit WEL = 1). */
uint32_t qpi_page_program();
uint32_t qpi_write_enable();
uint32_t qpi_write_disable();
uint32_t qpi_write_enable_volatile_status_regiter(void);
uint32_t qpi_read_jedec_id(void);

uint8_t qpi_read_status_1(void);
uint8_t qpi_write_status_1(uint8_t status1_data);
uint8_t qpi_read_status_2(void);
uint8_t qpi_write_status_2(uint8_t status2_data);
uint8_t qpi_read_status_3(void);
uint8_t qpi_write_status_3(uint8_t status3_data);

// read data instruction is not available in qpi mode
uint32_t qpi_fast_read_data(void);
// fast read quad i/o in qpi mode

#endif /* WINBOND_25Q_H_ */
