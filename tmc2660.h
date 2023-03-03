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
#define TMC2660_MODE                0           // 0 = TMCMode_StealthChop, 1 = TMCMode_CoolStep, 3 = TMCMode_StallGuard
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
#define TMC2660_IHOLDDELAY          6

// TPOWERDOWN
#define TMC2660_TPOWERDOWN          128 // 0 - ((2^8)-1) * 2^18 tCLK

// TPWMTHRS
#define TMC2660_TPWM_THRS           0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)

// PWMCONF - StealthChop defaults
#define TMC2660_PWM_FREQ            1   // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC2660_PWM_AUTOGRAD        1   // boolean (0 or 1)
#define TMC2660_PWM_GRAD            14  // 0 - 255
#define TMC2660_PWM_LIM             12  // 0 - 15
#define TMC2660_PWM_REG             8   // 1 - 15
#define TMC2660_PWM_OFS             36  // 0 - 255

// TCOOLTHRS
#define TMC2660_COOLSTEP_THRS       TMC_THRESHOLD_MIN   // tpwmthrs: 0 - 2^20 - 1 (20 bits)

// COOLCONF - CoolStep defaults
#define TMC2660_SEMIN               5   // 0 = coolStep off, 1 - 15 = coolStep on
#define TMC2660_SEUP                0   // 0 - 3 (1 - 8)
#define TMC2660_SEMAX               2   // 0 - 15
#define TMC2660_SEDN                1   // 0 - 3
#define TMC2660_SEIMIN              0   // boolean (0 or 1)

// end of default values

#if TMC2660_MODE == 0   // StealthChop
#define TMC2660_PWM_AUTOSCALE 1
#define TMC2660_EN_PWM_MODE   1
#elif TMC2660_MODE == 1 // CoolStep
#define TMC2660_PWM_AUTOSCALE 0
#define TMC2660_EN_PWM_MODE   0
#else                   //StallGuard
#define TMC2660_PWM_AUTOSCALE 0
#define TMC2660_EN_PWM_MODE   0
#endif

typedef uint8_t tmc2660_regaddr_t;

enum tmc2660_regaddr_t {
    TMC2660Reg_GCONF            = 0x00,
    TMC2660Reg_GSTAT            = 0x01,
    TMC2660Reg_IFCNT            = 0x02,
    TMC2660Reg_SLAVECONF        = 0x03,
    TMC2660Reg_IOIN             = 0x04,

    TMC2660Reg_OUTPUT           = 0x05,
    TMC2660Reg_X_COMPARE        = 0x06,
    TMC2660Reg_OTP_READ         = 0x07,
    TMC2660Reg_FACTORY_CONF     = 0x08,
    TMC2660Reg_SHORT_CONF       = 0x09,
    TMC2660Reg_DRV_CONF         = 0x0A,
    TMC2660Reg_GLOBAL_SCALER    = 0x0B,
    TMC2660Reg_OFFSET_READ      = 0x0C,

    TMC2660Reg_IHOLD_IRUN       = 0x10,
    TMC2660Reg_TPOWERDOWN       = 0x11,
    TMC2660Reg_TSTEP            = 0x12,
    TMC2660Reg_TPWMTHRS         = 0x13,
    TMC2660Reg_TCOOLTHRS        = 0x14,
    TMC2660Reg_THIGH            = 0x15,

    TMC2660Reg_XDIRECT          = 0x2D,

    TMC2660Reg_VDCMIN           = 0x33,
    TMC2660Reg_SW_MODE          = 0x34,
    TMC2660Reg_RAMP_STAT        = 0x35,
    TMC2660Reg_XLATCH           = 0x36,


    TMC2660Reg_MSLUT_BASE       = 0x60,
    TMC2660Reg_MSLUTSEL         = 0x68,
    TMC2660Reg_MSLUTSTART       = 0x69,
    TMC2660Reg_MSCNT            = 0x6A,
    TMC2660Reg_MSCURACT         = 0x6B,
    TMC2660Reg_CHOPCONF         = 0x6C,
    TMC2660Reg_COOLCONF         = 0x6D,
    TMC2660Reg_DCCTRL           = 0x6E,
    TMC2660Reg_DRV_STATUS       = 0x6F,
    TMC2660Reg_PWMCONF          = 0x70,
    TMC2660Reg_PWM_SCALE        = 0x71,
    TMC2660Reg_PWM_AUTO         = 0x72,
    TMC2660Reg_LOST_STEPS       = 0x73,
};

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag       :1,
        driver_error     :1,
        sg2              :1,
        standstill       :1,
        velocity_reached :1,
        position_reached :1,
        status_stop_l    :1,
        status_stop_r    :1;
    };
} TMC2660_status_t;

// --- register definitions ---

// GCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        recalibrate            :1,
        faststandstill         :1,
        en_pwm_mode            :1,
        multistep_filt         :1,
        shaft                  :1,
        diag0_error            :1,
        diag0_otpw             :1,
        diag0_stall            :1,
        diag1_stall            :1,
        diag1_index            :1,
        diag1_onstate          :1,
        diag1_steps_skipped    :1,
        diag0_int_pushpull     :1,
        diag1_poscomp_pushpull :1,
        small_hysteresis       :1,
        stop_enable            :1,
        direct_mode            :1,
        test_mode              :1,
        reserved               :14;
    };
} TMC2660_gconf_reg_t;

// GSTAT : R+C
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reset    :1,
        drv_err  :1,
        uv_cp    :1,
        reserved :29;
    };
} TMC2660_gstat_reg_t;

// IFCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        count    :8,
        reserved :24;
    };
} TMC2660_ifcnt_reg_t;

// SLAVECONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        slaveaddr :8,
        senddelay :4,
        reserved1 :20;
    };
} TMC2660_slaveconf_reg_t;

// IOIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        refl_step          :1,
        refl_dir           :1,
        encb_dcen_cfg4     :1,
        encb_dcen_cfg5     :1,
        drv_enn            :1,
        enc_n_dco_dco_cfg6 :1,
        sd_mode            :1,
        swcomp_in          :1,
        reserved           :16,
        version            :8;
    };
} TMC2660_ioin_reg_t;

// OUTPUT : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        io_polarity :1,
        version     :31;
    };
} TMC2660_output_reg_t;

// X_COMPARE : W
typedef union {
    uint32_t value;
    struct {
        uint32_t x_compare;
    };
} TMC2660_x_compare_reg_t;

// OTP_PROG : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpbit    :3,
        otpbyte   :2,
        reserved1 :3,
        otpmagic  :8,
        reserved2 :16;
    };
} TMC2660_otp_prog_reg_t;

// OTP_READ : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otp0_0_4 :5,
        otp0_5   :1,
        otp0_6   :1,
        otp0_7   :1,
        reserved :24;
    };
} TMC2660_otp_read_reg_t;

// FACTORY_CONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        fclktrim :5,
        reserved :27;
    };
} TMC2660_factory_conf_reg_t;

// SHORT_CONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        s2vs_level  :4,
        reserved1   :4,
        s2g_level   :4,
        reserved2   :4,
        shortfilter :2,
        shortdelay  :1,
        reserved3   :13;
    };
} TMC2660_short_conf_reg_t;

// DRV_CONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        bbmtime     :5,
        reserved1   :3,
        bbmclks     :4,
        reserved2   :4,
        otselect    :2,
        drvstrenght :2,
        filt_isense :2,
        reserved3   :10;
    };
} TMC2660_drv_conf_reg_t;

// GLOBAL_SCALER : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        scaler   :8,
        reserved :24;
    };
} TMC2660_global_scaler_reg_t;

// OFFSET_READ : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        phase_a  :8,
        phase_b  :8,
        reserved :20;
    };
} TMC2660_offset_read_reg_t;

// IHOLD_IRUN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,
        reserved1  :3,
        irun       :5,
        reserved2  :3,
        iholddelay :4,
        reserved3  :12;
    };
} TMC2660_ihold_irun_reg_t;

// TPOWERDOWN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,
        reserved   :24;
    };
} TMC2660_tpowerdown_reg_t;

// TSTEP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,
        reserved :12;
    };
} TMC2660_tstep_reg_t;

// TPWMTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,
        reserved :12;
    };
} TMC2660_tpwmthrs_reg_t;

// TCOOLTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tcoolthrs :20,
        reserved  :12;
    };
} TMC2660_tcoolthrs_reg_t;

// THIGH : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        thigh    :20,
        reserved :12;
    };
} TMC2660_thigh_reg_t;

// VDCMIN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        vdcmin   :23,
        reserved :9;
    };
} TMC2660_vdcmin_reg_t;

// MSLUTn : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mte :32;
    };
} TMC2660_mslut_n_reg_t;

// MSLUTSEL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        w0 :2,
        w1 :2,
        w2 :2,
        w3 :2,
        x1 :8,
        x2 :8,
        x3 :8;
    };
} TMC2660_mslutsel_reg_t;

// MSLUTSTART : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        start_sin   :8,
        reserved1   :8,
        start_sin90 :8,
        reserved2   :8;
    };
} TMC2660_mslutstart_reg_t;

//??MSLUTSEL

// MSCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,
        reserved :22;
    };
} TMC2660_mscnt_reg_t;

// MSCURACT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cur_a     :9,
        reserved1 :7,
        cur_b     :9,
        reserved2 :7;
    };
} TMC2660_mscuract_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff      :4,
        hstrt     :3,
        hend      :4,
        fd3       :1,
        disfdcc   :1,
        reserved1 :1,
        chm       :1,
        tbl       :2,
        reserved2 :1,
        vhighfs   :1,
        vhighchm  :1,
        tpfd      :4,
        mres      :4,
        intpol    :1,
        dedge     :1,
        diss2g    :1,
        diss2vs   :1;
    };
} TMC2660_chopconf_reg_t;

// COOLCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        semin     :4,
        reserved1 :1,
        seup      :2,
        reserved2 :1,
        semax     :4,
        reserved3 :1,
        sedn      :2,
        seimin    :1,
        sgt       :7,
        reserved4 :1,
        sfilt     :1,
        reserved5 :7;
    };
} TMC2660_coolconf_reg_t;

// DCCTRL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        dc_time   :9,
        reserved1 :7,
        dc_sg     :8,
        reserved2 :8;
    };
} TMC2660_dcctrl_reg_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg_result  :10,
        reserved1  :2,
        s2vsa      :1,
        s2vsb      :1,
        stealth    :1,
        fsactive   :1,
        cs_actual  :5,
        reserved2  :3,
        stallguard :1,
        ot         :1,
        otpw       :1,
        s2ga       :1,
        s2gb       :1,
        ola        :1,
        olb        :1,
        stst       :1;
    };
} TMC2660_drv_status_reg_t;

// PWMCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs       :8,
        pwm_grad      :8,
        pwm_freq      :2,
        pwm_autoscale :1,
        pwm_autograd  :1,
        freewheel     :2,
        reserved      :2,
        pwm_reg       :4,
        pwm_lim       :4;
    };
} TMC2660_pwmconf_reg_t;

// PWM_SCALE : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale_sum  :8,
        reserved1      :8,
        pwm_scale_auto :9,
        reserved2      :7;
    };
} TMC2660_pwm_scale_reg_t;

// PWM_AUTO : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs_auto   :8,
        reserved1      :8,
        pwm_grad_auto  :8,
        reserved2      :8;
    };
} TMC2660_pwm_auto_reg_t;

// LOST_STEPS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        lost_steps :20,
        reserved   :12;
    };
} TMC2660_lost_steps_reg_t;

// --- end of register definitions ---

typedef union {
    tmc2660_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC2660_addr_t;

// --- datagrams ---

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_gconf_reg_t reg;
} TMC2660_gconf_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_gstat_reg_t reg;
} TMC2660_stat_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_ioin_reg_t reg;
} TMC2660_ioin_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_global_scaler_reg_t reg;
} TMC2660_global_scaler_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_ihold_irun_reg_t reg;
} TMC2660_ihold_irun_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_tpowerdown_reg_t reg;
} TMC2660_tpowerdown_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_tcoolthrs_reg_t reg;
} TMC2660_tcoolthrs_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_mslutstart_reg_t reg;
} TMC2660_mslutstart_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_tstep_reg_t reg;
} TMC2660_tstep_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_tpwmthrs_reg_t reg;
} TMC2660_tpwmthrs_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_thigh_reg_t reg;
} TMC2660_thigh_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_vdcmin_reg_t reg;
} TMC2660_vdcmin_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_mslut_n_reg_t reg;
} TMC2660_mslut_n_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_mslutsel_reg_t reg;
} TMC2660_mslutsel_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_mscnt_reg_t reg;
} TMC2660_mscnt_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_mscuract_reg_t reg;
} TMC2660_mscuract_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_dcctrl_reg_t reg;
} TMC2660_dcctrl_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_chopconf_reg_t reg;
} TMC2660_chopconf_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_drv_status_reg_t reg;
} TMC2660_drv_status_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_coolconf_reg_t reg;
} TMC2660_coolconf_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_pwmconf_reg_t reg;
} TMC2660_pwmconf_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_pwm_scale_reg_t reg;
} TMC2660_pwm_scale_dgr_t;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_lost_steps_reg_t reg;
} TMC2660_lost_steps_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    /*TMC2660_gconf_reg_t gconf;
    TMC2660_gstat_reg_t gstat;
    TMC2660_ioin_reg_t ioin;
    TMC2660_global_scaler_dgr_t global_scaler;
    TMC2660_ihold_irun_reg_t ihold_irun;
    TMC2660_tpowerdown_reg_t tpowerdown;
    TMC2660_tstep_reg_t tstep;
    TMC2660_tpwmthrs_reg_t tpwmthrs;
    TMC2660_tcoolthrs_reg_t tcoolthrs;
    TMC2660_thigh_reg_t thigh;
    TMC2660_vdcmin_reg_t vdcmin;
    TMC2660_mslut_n_reg_t mslut;
    TMC2660_mslutsel_reg_t mslutsel;
    TMC2660_mslutstart_reg_t mslutstart;
    TMC2660_mscnt_reg_t mscnt;
    TMC2660_mscuract_reg_t mscuract;
    TMC2660_dcctrl_reg_t dcctrl;
    TMC2660_drv_status_reg_t drv_status;
    TMC2660_chopconf_reg_t chopconf;
    TMC2660_coolconf_reg_t coolconf;
    TMC2660_pwmconf_reg_t pwmconf;
    TMC2660_pwm_scale_reg_t pwm_scale;
    TMC2660_lost_steps_reg_t lost_steps;*/
} TMC2660_payload;

typedef struct {
    TMC2660_addr_t addr;
    TMC2660_payload payload;
} TMC2660_datagram_t;

typedef struct {
    // driver registers
    TMC2660_gconf_dgr_t gconf;
    TMC2660_stat_dgr_t gstat;
    TMC2660_ioin_dgr_t ioin;
    TMC2660_global_scaler_dgr_t global_scaler;
    TMC2660_ihold_irun_dgr_t ihold_irun;
    TMC2660_tpowerdown_dgr_t tpowerdown;
    TMC2660_tstep_dgr_t tstep;
    TMC2660_tpwmthrs_dgr_t tpwmthrs;
    TMC2660_tcoolthrs_dgr_t tcoolthrs;
    TMC2660_thigh_dgr_t thigh;
    TMC2660_vdcmin_dgr_t vdcmin;
#ifdef TMC2660_COMPLETE
    TMC2660_xdirect_dgr_t xdirect;
    TMC2660_mslut_n_dgr_t mslut[8];
    TMC2660_mslutsel_dgr_t mslutsel;
    TMC2660_mslutstart_dgr_t mslutstart;
    TMC2660_encm_ctrl_dgr_t encm_ctrl;
#endif
    TMC2660_mscnt_dgr_t mscnt;
    TMC2660_mscuract_dgr_t mscuract;
    TMC2660_dcctrl_dgr_t dcctrl;
    TMC2660_drv_status_dgr_t drv_status;
    TMC2660_chopconf_dgr_t chopconf;
    TMC2660_coolconf_dgr_t coolconf;
    TMC2660_pwmconf_dgr_t pwmconf;
    TMC2660_pwm_scale_dgr_t pwm_scale;
    TMC2660_lost_steps_dgr_t lost_steps;
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
