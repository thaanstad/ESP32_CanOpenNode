/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <Arduino.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#define CO_CONFIG_SDO_CLI (1)
#define CO_CONFIG_FIFO (1)


/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t *)(msg)))->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t *)(msg)))->DLC)
#define CO_CANrxMsg_readData(msg)  ((uint8_t *)(((CO_CANrxMsg_t *)(msg)))->data)

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
    SemaphoreHandle_t xOD_Mutex;
    SemaphoreHandle_t xCAN_SEND_Mutex;
    SemaphoreHandle_t xCAN_ERROR_Mutex;
} CO_CANmodule_t;

void CO_CANinterrupt(CO_CANmodule_t *CANmodule);
    /**
 * Data storage object for one entry.
 * For more information on Data storage see @ref CO_storage or **CO_storage.h**
 * file. Structure members documented here are always required or required with
 * @ref CO_storage_eeprom. Target system may add own additional, hardware
 * specific variables.
 */
typedef struct {
    /** Address of data to store, always required. */
    void *addr;
    /** Length of data to store, always required. */
    size_t len;
    /** Sub index in OD objects 1010 and 1011, from 2 to 127. Writing
     * 0x65766173 to 1010,subIndexOD will store data to non-volatile memory.
     * Writing 0x64616F6C to 1011,subIndexOD will restore default data, always
     * required. */
    uint8_t subIndexOD;
    /** Attribute from @ref CO_storage_attributes_t, always required. */
    uint8_t attr;
    /** Pointer to storage module, target system specific usage, required with
     * @ref CO_storage_eeprom. */
    void *storageModule;
    /** CRC checksum of the data stored in eeprom, set on store, required with
     * @ref CO_storage_eeprom. */
    uint16_t crc;
    /** Address of entry signature inside eeprom, set by init, required with
     * @ref CO_storage_eeprom. */
    size_t eepromAddrSignature;
    /** Address of data inside eeprom, set by init, required with
     * @ref CO_storage_eeprom. */
    size_t eepromAddr;
    /** Offset of next byte being updated by automatic storage, required with
     * @ref CO_storage_eeprom. */
    size_t offset;
    /** Additional target specific parameters, optional. */
    void *additionalParameters;
} CO_storage_entry_t;


/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)  xSemaphoreTake(CAN_MODULE->xCAN_SEND_Mutex, portMAX_DELAY); // take mutex
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) xSemaphoreGive(CAN_MODULE->xCAN_SEND_Mutex); // release mutex

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE) xSemaphoreTake(CAN_MODULE->xCAN_ERROR_Mutex, portMAX_DELAY); // take mutex
#define CO_UNLOCK_EMCY(CAN_MODULE) xSemaphoreGive(CAN_MODULE->xCAN_ERROR_Mutex); // release mutex

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE) xSemaphoreTake(CAN_MODULE->xOD_Mutex, portMAX_DELAY);   // take mutex
#define CO_UNLOCK_OD(CAN_MODULE) xSemaphoreGive(CAN_MODULE->xOD_Mutex); // release mutex

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew) {CO_MemoryBarrier(); rxNew = NULL;}


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
