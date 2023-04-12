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
#define TMC2660_F_CLK               12000000UL  // factory tuned to 12MHz - see datasheet for calibration procedure if required
#define TMC2660_MODE                3           // 0 = TMCMode_StealthChop - not supported on 2660, 1 = TMCMode_CoolStep, 3 = TMCMode_StallGuard
#define TMC2660_MICROSTEPS          TMC2660_Microsteps_4
#define TMC2660_R_SENSE             100          // mOhm
#define TMC2660_CURRENT             1500         // mA RMS
#define TMC2660_HOLD_CURRENT_PCT    50

// CHOPCONF
#define TMC2660_INTERPOLATE         1   // intpol: 0 = off, 1 = on
#define TMC2660_CONSTANT_OFF_TIME   5   // toff: 1 - 15
#define TMC2660_BLANK_TIME          1   // tbl: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC2660_CHOPPER_MODE        0   // chm: 0 = spreadCycle, 1 = constant off time
// TMC2660_CHOPPER_MODE 0 defaults
#define TMC2660_HSTRT               3   // hstrt: 0 - 7
#define TMC2660_HEND                2   // hend: -3 - 12
// TMC2660_CHOPPER_MODE 1 defaults
#define TMC2130_TFD                 13  // fd3 & hstrt: 0 - 15

// IHOLD_IRUN
//#define TMC2660_IHOLDDELAY          6  // this is not supported.

// TPOWERDOWN
//#define TMC2660_TPOWERDOWN          128 // 0 - ((2^8)-1) * 2^18 tCLK  //don't think this is supported.

// TPWMTHRS
//#define TMC2660_TPWM_THRS           0   // tpwmthrs: 0 - 2^20 - 1 (20 bits) //don't think this is supported.

// PWMCONF - StealthChop defaults - not supported.
/*#define TMC2660_PWM_FREQ            1   // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC2660_PWM_AUTOGRAD        1   // boolean (0 or 1)
#define TMC2660_PWM_GRAD            14  // 0 - 255
#define TMC2660_PWM_LIM             12  // 0 - 15
#define TMC2660_PWM_REG             8   // 1 - 15
#define TMC2660_PWM_OFS             36  // 0 - 255

// TCOOLTHRS
#define TMC2660_COOLSTEP_THRS       TMC_THRESHOLD_MIN   // tpwmthrs: 0 - 2^20 - 1 (20 bits)
*/

// COOLCONF - CoolStep defaults
#define TMC2660_SEMIN               5   // 0 = coolStep off, 1 - 15 = coolStep on
#define TMC2660_SEUP                0   // 0 - 3 (1 - 8)
#define TMC2660_SEMAX               2   // 0 - 15
#define TMC2660_SEDN                1   // 0 - 3
#define TMC2660_SEIMIN              0   // boolean (0 or 1)

// end of default values

//#if TMC2660_MODE == 0   // StealthChop
//#define TMC2660_PWM_AUTOSCALE 1
//#define TMC2660_EN_PWM_MODE   1
#if TMC2660_MODE == 1 // CoolStep
#define TMC2660_PWM_AUTOSCALE 0
#define TMC2660_EN_PWM_MODE   0
#else                   //StallGuard
#define TMC2660_PWM_AUTOSCALE 0
#define TMC2660_EN_PWM_MODE   0
#endif

typedef uint8_t tmc2660_regaddr_t;


//adresses are 3 bits.
enum tmc2660_regaddr_t {
    TMC2660Reg_DRVCTR           = 0x0, //only ever going to use step/dir mode.
    TMC2660Reg_CHOPCONF         = 0x1,
    TMC2660Reg_SMARTEN          = 0x5, //Coolstep control register.
    TMC2660Reg_DRVCONF          = 0x7, //Driver Control Register.
    TMC2660Reg_SGCSCONF         = 0x9, //Stallguard2 control register.
};

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
        mres0           :1,
        mres1           :1,
        mres2           :1,
        mres3           :1,
        reserved1       :4,
        dedge           :1,
        intpol          :1,
        reserved2       :7,//only count 7 reserved bits for 17 in total.
    };
} TMC2660_drvctrl_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff0           :1,
        toff1           :1,
        toff2           :1,
        toff3           :1,
        hstrt0          :1,
        hstrt1          :1,
        hstrt2          :1,
        hend0           :1,  
        hend1           :1,
        hend2           :1,  
        hend3           :1,
        hdec0           :1,  
        hdec1           :1, 
        rndtf           :1,
        chm             :1,
        tbl0            :1,
        tbl1            :1,                                       
    };
} TMC2660_chopconf_reg_t;  //17 bits is complete register.

// SMARTEN (Coolstep) : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        semin0          :1, 
        semin1          :1,
        semin2          :1,   
        semin3          :1, 
        reserved1       :1,  
        seup0           :1,
        seup1           :1, 
        reserved2       :1,   
        semax0          :1,                                                          
        semax1          :1,      
        semax2          :1,      
        semax3          :1,  
        reserved3       :1, 
        sedn0           :1,  
        sedn1           :1,    
        seimin          :1,  
        reserved4       :1,                                                                                                   
    };
} TMC2660_smarten_reg_t;  //17 bits is complete register.

// SGCSCONF (Stallguard) : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cs0             :1, 
        cs1             :1,
        cs2             :1,   
        cs3             :1,
        cs4             :1,          
        reserved1       :3,  
        sgt0            :1,
        sgt1            :1,
        sgt2            :1,
        sgt3            :1,
        sgt4            :1,
        sgt5            :1, 
        sgt6            :1,  
        reserved2       :1,    
        sfilt           :1,                                                                                                                                                              
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
        rdsel0          :1, 
        rdsel1          :1,
        vsense          :1, 
        sdoff           :1,   
        ts2g0           :1,
        ts2g1           :1,
        diss2g          :1,
        reserved1       :1, 
        slpl0           :1,
        slpl1           :1,   
        slph0           :1,
        slph1           :1,    
        tst             :1,                                                                                                                                                                                                                             
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
        sg_90      :10,                                     
    };
} TMC2660_drvstatus_reg_t;

// --- end of register definitions ---

typedef union {
    tmc2660_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :3,
        write :1;
    };
} TMC2660_addr_t;

// --- datagrams ---

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_drvctrl_reg_t reg;
} TMC2660_drvctrl_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_chopconf_reg_t reg;
} TMC2660_chopconf_dgt_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_smarten_reg_t reg;
} TMC2660_smarten_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_sgcsconf_reg_t reg;
} TMC2660_sgcsconf_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_drvconf_reg_t reg;
} TMC2660_drvconf_dgr_t;

typedef struct {
    //TMC2660_addr_t addr;//there is no address on this register.
    TMC2660_drvstatus_reg_t reg;
} TMC2660_drvstatus_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    TMC2660_drvctrl_reg_t drvctrl;
    TMC2660_chopconf_reg_t chopconf;
    TMC2660_smarten_reg_t smarten;
    TMC2660_sgcsconf_reg_t sgcsconf;
    TMC2660_drvconf_reg_t drvconf;
    TMC2660_drvstatus_reg_t drvstatus;
} TMC2660_payload;

typedef struct {
    TMC2660_addr_t addr;
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

    TMC2660_status_t driver_status;

    trinamic_config_t config;
} TMC2660_t;

#pragma pack(pop)

bool TMC2660_Init(TMC2660_t *driver);
void TMC2660_SetDefaults (TMC2660_t *driver);
void TMC2660_SetCurrent (TMC2660_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2660_GetCurrent (TMC2660_t *driver);
bool TMC2660_MicrostepsIsValid (uint16_t usteps);
void TMC2660_SetMicrosteps(TMC2660_t *driver, tmc2660_microsteps_t usteps);
float TMC2660_GetTPWMTHRS (TMC2660_t *driver, float steps_mm);
void TMC2660_SetTPWMTHRS (TMC2660_t *driver, float mm_sec, float steps_mm);
void TMC2660_SetTHIGH (TMC2660_t *driver, float mm_sec, float steps_mm);
void TMC2660_SetTCOOLTHRS (TMC2660_t *driver, float mm_sec, float steps_mm);

void TMC2660_SetConstantOffTimeChopper(TMC2660_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2660_datagram_t *TMC2660_GetRegPtr (TMC2660_t *driver, tmc2660_regaddr_t reg);
TMC2660_status_t TMC2660_WriteRegister (TMC2660_t *driver, TMC2660_datagram_t *reg);
TMC2660_status_t TMC2660_ReadRegister (TMC2660_t *driver, TMC2660_datagram_t *reg);

#endif
