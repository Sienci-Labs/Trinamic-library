/*
 * tmc2660.h - register and message (datagram) descriptors for Trinamic TMC2660 stepper driver
 *
 * v0.0.4 / 2022-12-22 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021-2022, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _TRINAMIC2660_H_
#define _TRINAMIC2660_H_

#include "common.h"
//#include "tmc26x.h"

//#define TMC2660_COMPLETE // comment out for minimum set of registers

#pragma pack(push, 1)

typedef enum {
    TMC2660_Microsteps_1 = 1,
    TMC2660_Microsteps_2 = 2,
    TMC2660_Microsteps_4 = 4,
    TMC2660_Microsteps_8 = 8,
    TMC2660_Microsteps_16 = 16,
    TMC2660_Microsteps_32 = 32,
    TMC2660_Microsteps_64 = 64,
    TMC2660_Microsteps_128 = 128,
    TMC2660_Microsteps_256 = 256
} tmc2660_microsteps_t;

// default values

// General
#define TMC2660_F_CLK               15000000UL  // factory tuned to 12MHz - see datasheet for calibration procedure if required
#define TMC2660_MODE                1           // 0 = TMCMode_StealthChop - not supported on 2660, 1 = TMCMode_CoolStep, 3 = TMCMode_StallGuard
#define TMC2660_MICROSTEPS          TMC2660_Microsteps_4
#define TMC2660_R_SENSE             100          // mOhm
#define TMC2660_CURRENT             500         // mA RMS
#define TMC2660_HOLD_CURRENT_PCT    50  //holding current percent

// CHOPCONF
#define TMC2660_CONSTANT_OFF_TIME   1   // toff: 1 - 15
#define TMC2660_BLANK_TIME          1   // tbl: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC2660_CHOPPER_MODE        0   // chm: 0 = spreadCycle, 1 = constant off time  Do not use constant off
#define TMC2660_HSTR                0   // hstr: 0 - 7
#define TMC2660_HEND                10   // hend: -3 - 12
#define TMC2660_HDEC                0   // Hysteresis decrement: 0 16 clocks
#define TMC2660_RNDTF               1  // Random off time

//SGCSCONF
#define TMC2660_CURRENT_SCALE       15   // current scale default (conservative)
#define TMC2660_SG_THRESH           64   // Stallguard threshold
#define TMC2660_SG_FILTER           1   // Enable Stallguard Filter

//DRVCONF
#define TMC2660_DRVCONF             0x0310   // 0x0310 DRVCONF Register defaults (likely don't need to change)

//DRVCTRL
#define TMC2660_MRES                TMC2660_MICROSTEPS
#define TMC2660_DEDGE               0           //double edge step pulses
#define TMC2660_INTPOL              1          //step interpolation

//SMARTEN
#define TMC2660_SEMIN               1   // 0 = Coolstep disabled
#define TMC2660_SEUP                3   // 0 - 3 (1 - 8)
#define TMC2660_SEMAX               0   // 0 - 15
#define TMC2660_SEDN                3   // 0 - 
#define TMC2660_SEIMIN              0   // 0 = 1/2 of CS, 1 = 1/4 of CS

// end of default values

typedef uint8_t tmc2660_regaddr_t;

#define TMC2660_WRITE_BIT 0x08

#if 1
//adresses are 3 bits.
enum tmc2660_regaddr_t {
    TMC2660Reg_DRVCTRL          = 0x0, //only ever going to use step/dir mode.
    TMC2660Reg_CHOPCONF         = 0x8,
    TMC2660Reg_SMARTEN          = 0xA, //Coolstep control register.
    TMC2660Reg_SGCSCONF         = 0xD, //Stallguard2 control register.    
    TMC2660Reg_DRVCONF          = 0xE, //Driver Control Register.
};
#else
//adresses are 3 bits.
enum tmc2660_regaddr_t {
    TMC2660Reg_DRVCTRL          = 0x0, //only ever going to use step/dir mode.
    TMC2660Reg_CHOPCONF         = 0x1,
    TMC2660Reg_SMARTEN          = 0x5, //Coolstep control register.
    TMC2660Reg_SGCSCONF         = 0x3, //Stallguard2 control register.    
    TMC2660Reg_DRVCONF          = 0x7, //Driver Control Register.
};
#endif

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag       :1,
        driver_error     :1, //SHORT, OPEN OR OT?
        sg               :1, 
        standstill       :1, //STST bit?
        //velocity_reached :1, //unsupported
        //position_reached :1, //unsupported
        status_stop_l    :1,  //??
        status_stop_r    :1;  //??
    };
} TMC2660_status_t;

// --- register definitions ---
//Datagram is 20 bits.
// DRVCTRL : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mres            :4,
        reserved1       :4,
        dedge           :1,
        intpol          :1,
        reserved2       :7;//only count 7 reserved bits for 17 in total.
    };
} TMC2660_drvctrl_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff            :4,
        hstrt           :3,
        hend            :4,  
        hdec            :2,  
        rndtf           :1,
        chm             :1,
        tbl             :2;                                   
    };
} TMC2660_chopconf_reg_t;  //17 bits is complete register.

// SMARTEN (Coolstep) : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        semin           :4, 
        reserved1       :1,  
        seup            :2,
        reserved2       :1,   
        semax           :4,                                                          
        reserved3       :1, 
        sedn            :2,    
        seimin          :1,  
        reserved4       :1;                                                                                                   
    };
} TMC2660_smarten_reg_t;  //17 bits is complete register.

// SGCSCONF (Stallguard) : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cs              :5,          
        reserved1       :3,  
        sgt             :7,
        reserved2       :1,    
        sfilt           :1;                                                                                                                                                              
    };
} TMC2660_sgcsconf_reg_t;  //17 bits is complete register.

// DRVCONF (Driver control) : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        en_s2vs         :1, 
        en_pfd          :1,
        shrtsens        :1,   
        otsens          :1,
        rdsel           :2, 
        vsense          :1, 
        sdoff           :1,   
        ts2g            :2,
        diss2g          :1,
        reserved1       :1, 
        slpl            :2,  
        slph            :2,  
        tst             :1;                                                                                                                                                                                                                             
    };
} TMC2660_drvconf_reg_t;  //17 bits is complete register.


// DRV_STATUS : R  //this is the status return
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg         :1, 
        ot         :1, //over temp shutdown  
        otpw       :1, //over temp warning    
        shorta     :1,                
        shortb     :1,    
        ola        :1,   
        olb        :1, 
        stst       :1,  //standstill indicator.
        chip_rev   :2,  
        sg_90      :10; //can also be mstep for phase info.                                    
    };
} TMC2660_drvstatus_reg_t;

// --- end of register definitions ---

/* TMC2660 has a different datagram*/

typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :3
    };
} TMC2660_addr_t;

typedef union {
    uint32_t value;
    uint8_t data[3];
} TMC2660_payload_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_payload_t payload;
} TMC2660_spi_datagram_t;
/*TMC2660*/


// --- datagrams ---

typedef struct {
    TMC_addr_t addr;
    TMC2660_drvctrl_reg_t reg;
} TMC2660_drvctrl_dgr_t;

typedef struct {
    TMC_addr_t addr;
    TMC2660_chopconf_reg_t reg;
} TMC2660_chopconf_dgt_t;

typedef struct {
    TMC_addr_t addr;
    TMC2660_smarten_reg_t reg;
} TMC2660_smarten_dgr_t;

typedef struct {
    TMC_addr_t addr;
    TMC2660_sgcsconf_reg_t reg;
} TMC2660_sgcsconf_dgr_t;

typedef struct {
    TMC_addr_t addr;
    TMC2660_drvconf_reg_t reg;
} TMC2660_drvconf_dgr_t;

typedef struct {
    //TMC_addr_t addr;//there is no address on this register.
    TMC2660_drvstatus_reg_t reg;
} TMC2660_drvstatus_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[3];
    TMC2660_drvctrl_reg_t drvctrl;
    TMC2660_chopconf_reg_t chopconf;
    TMC2660_smarten_reg_t smarten;
    TMC2660_sgcsconf_reg_t sgcsconf;
    TMC2660_drvconf_reg_t drvconf;
    TMC2660_drvstatus_reg_t drvstatus;
} TMC2660_payload;

typedef struct {
    TMC_addr_t addr;
    TMC2660_payload payload;
} TMC2660_datagram_t;

typedef struct {
    // driver registers
    TMC2660_drvctrl_dgr_t drvctrl;
    TMC2660_chopconf_dgt_t chopconf;
    TMC2660_smarten_dgr_t smarten;
    TMC2660_sgcsconf_dgr_t sgcsconf;
    TMC2660_drvconf_dgr_t drvconf;
    TMC2660_drvstatus_dgr_t drvstatus;
    //status reg
    TMC2660_status_t driver_status;
    //common config
    trinamic_config_t config;
} TMC2660_t;

#pragma pack(pop)

TMC_spi_status_t tmc2660_spi_write (trinamic_motor_t driver, TMC2660_spi_datagram_t *datagram);
TMC_spi_status_t tmc2660_spi_read (trinamic_motor_t driver, TMC2660_spi_datagram_t *datagram);

bool TMC2660_Init(TMC2660_t *driver);
void TMC2660_SetDefaults (TMC2660_t *driver);
void TMC2660_SetCurrent (TMC2660_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2660_GetCurrent (TMC2660_t *driver);
bool TMC2660_MicrostepsIsValid (uint16_t usteps);
void TMC2660_SetMicrosteps(TMC2660_t *driver, tmc2660_microsteps_t usteps);

//stallguard functions

void TMC2660_SetConstantOffTimeChopper(TMC2660_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2660_datagram_t *TMC2660_GetRegPtr (TMC2660_t *driver, tmc2660_regaddr_t reg);
TMC2660_status_t TMC2660_WriteRegister (TMC2660_t *driver, TMC2660_datagram_t *reg);
TMC2660_status_t TMC2660_ReadRegister (TMC2660_t *driver, TMC2660_datagram_t *reg);

#endif
