/*
 * tmc2660hal.c - interface for Trinamic TMC2660 stepper driver
 
   TMC2660 modifications Copyright (C) Sienci Labs Inc.
  
   This file is part of the SuperLongBoard family of products.
  
   This source describes Open Hardware and is licensed under the "CERN-OHL-S v2"

   You may redistribute and modify this source and make products using
   it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.t). 
   This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
   INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A 
   PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.
   
   As per CERN-OHL-S v2 section 4, should You produce hardware based on this 
   source, You must maintain the Source Location clearly visible on the external
   case of the CNC Controller or other product you make using this source.
  
   You should have received a copy of the CERN-OHL-S v2 license with this source.
   If not, see <https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-version-2>.
   
   Contact for information regarding this program and its license
   can be sent through gSender@sienci.com or mailed to the main office
   of Sienci Labs Inc. in Waterloo, Ontario, Canada.

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
specific prior written permission..

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

#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"

#include "tmc2660.h"
#include "tmchal.h"

static TMC2660_t *tmcdriver[6];

static trinamic_config_t *getConfig (uint8_t motor)
{
    return &tmcdriver[motor]->config;
}

static bool isValidMicrosteps (uint8_t motor, uint16_t msteps)
{
    return tmc_microsteps_validate(msteps);
}

static void setMicrosteps (uint8_t motor, uint16_t msteps)
{
   TMC2660_SetMicrosteps(tmcdriver[motor], (tmc2660_microsteps_t)msteps);
   tmcdriver[motor]->config.microsteps = (tmc2660_microsteps_t)msteps;
}

static void setCurrent (uint8_t motor, uint16_t mA, uint8_t hold_pct)
{
    TMC2660_SetCurrent(tmcdriver[motor], mA, hold_pct);
}

static uint16_t getCurrent (uint8_t motor)
{
    return TMC2660_GetCurrent(tmcdriver[motor]);
}

static TMC_chopconf_t getChopconf (uint8_t motor)
{
    TMC_chopconf_t chopconf;
    TMC2660_t *driver = tmcdriver[motor];

    //tmc2660 doesn't read registers directly.

    chopconf.mres = driver->drvctrl.reg.mres;
    chopconf.toff = driver->chopconf.reg.toff;
    chopconf.tbl = driver->chopconf.reg.tbl;
    chopconf.hend = driver->chopconf.reg.hend;
    chopconf.hstrt = driver->chopconf.reg.hstrt;

    return chopconf;
}

static uint32_t getStallGuardResult (uint8_t motor)
{
    
    tmc2660_spi_read(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvconf, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvstatus);

    return (uint32_t)tmcdriver[motor]->drvstatus.reg.sg_90;
}

static TMC_drv_status_t getDriverStatus (uint8_t motor)
{
    TMC_drv_status_t drv_status;
    TMC2660_status_t status;

    status.value = tmc2660_spi_read(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvconf, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvstatus);

    drv_status.driver_error = status.driver_error;
    drv_status.sg_result = tmcdriver[motor]->drvstatus.reg.sg_90;
    drv_status.ot = tmcdriver[motor]->drvstatus.reg.ot;
    drv_status.otpw = tmcdriver[motor]->drvstatus.reg.otpw;
    drv_status.cs_actual = tmcdriver[motor]->sgcsconf.reg.cs;
    drv_status.stst = tmcdriver[motor]->drvstatus.reg.stst;
    drv_status.fsactive = 0;
    drv_status.ola = tmcdriver[motor]->drvstatus.reg.ola;
    drv_status.olb = tmcdriver[motor]->drvstatus.reg.olb;
    drv_status.s2ga = tmcdriver[motor]->drvstatus.reg.shorta;
    drv_status.s2gb = tmcdriver[motor]->drvstatus.reg.shortb;
    drv_status.stallguard = tmcdriver[motor]->drvstatus.reg.sg;

    return drv_status;
}

/*TMC2660_drvstatus_dgr_t TMC2660_getAlarmStatus (uint8_t motor)
{
    //tmc2660_spi_read(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvstatus);

    //return tmcdriver[motor]->drvstatus.reg;
    //return tmcdriver[motor]->drvstatus.reg.stst<<7 | tmcdriver[motor]->drvstatus.reg.olb<<6 | tmcdriver[motor]->drvstatus.reg.ola<<5 | tmcdriver[motor]->drvstatus.reg.shortb<<4 | tmcdriver[motor]->drvstatus.reg.shorta<<3 | tmcdriver[motor]->drvstatus.reg.otpw<<2 | tmcdriver[motor]->drvstatus.reg.ot<<1 | tmcdriver[motor]->drvstatus.reg.sg;
}*/

static TMC_ihold_irun_t getIholdIrun (uint8_t motor)
{
    TMC_ihold_irun_t ihold_irun;

    ihold_irun.ihold = tmcdriver[motor]->config.hold_current_pct * tmcdriver[motor]->sgcsconf.reg.cs;
    ihold_irun.irun = tmcdriver[motor]->sgcsconf.reg.cs;
    ihold_irun.iholddelay = 2; //standstill delay is fixed with 2660.

    return ihold_irun;
}

static uint32_t getDriverStatusRaw (uint8_t motor)  //only used for reporting
{
    tmc2660_spi_read(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvconf, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->drvstatus);

    return tmcdriver[motor]->drvstatus.reg.value;
}

static void stallGuardEnable (uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity)
{
    TMC2660_t *driver = tmcdriver[motor];

    //sg_tst pin is always enabled.
    //driver->sgcsconf.reg.sgt = sensitivity & 0x7F; // 7-bits signed value
    tmc2660_spi_write(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&driver->sgcsconf);
}

// stallguard filter
static void sg_filter (uint8_t motor, bool val)
{
    //tmcdriver[motor]->sgcsconf.reg.sfilt = val;
    tmc2660_spi_write(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->sgcsconf);
}

static void sg_stall_value (uint8_t motor, int16_t val)
{
    tmcdriver[motor]->sgcsconf.reg.sgt = val & 0x7F; // 7-bits signed value
    tmc2660_spi_write(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&tmcdriver[motor]->sgcsconf);
}

static int16_t get_sg_stall_value (uint8_t motor)
{
    return (int16_t)tmcdriver[motor]->sgcsconf.reg.sgt;
}

static void coolconf (uint8_t motor, TMC_coolconf_t coolconf)
{
    TMC2660_t *driver = tmcdriver[motor];

     #if 0
    driver->smarten.reg.semin = coolconf.semin;
    driver->smarten.reg.semax = coolconf.semax;
    driver->smarten.reg.sedn = coolconf.sedn;
    #endif
    tmc2660_spi_write(tmcdriver[motor]->config.motor, (TMC2660_spi_datagram_t *)&driver->smarten);    
}

static void coolStepEnable (uint8_t motor)
{
    //coolstep is always enabled.
}
// chopconf

static void chopper_timing (uint8_t motor, TMC_chopper_timing_t timing)
{
    TMC2660_t *driver = tmcdriver[motor];

    #if 0
    driver->chopconf.reg.chm = TMC2660_CHOPPER_MODE;
    driver->chopconf.reg.hstrt = timing.hstrt + 1;
    driver->chopconf.reg.hend = timing.hend + 3;
    driver->chopconf.reg.tbl = timing.tbl;
    driver->chopconf.reg.toff = timing.toff;
    #endif

    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->chopconf);
    
}

static void updateSettings (uint8_t motor, TMC2660_settings_t* settings)
{
    TMC2660_t *driver = tmcdriver[motor];

    driver->chopconf.reg.toff = settings->toff;
    driver->chopconf.reg.tbl = settings->tbl;
    driver->chopconf.reg.chm = settings->chm;
    driver->chopconf.reg.hstrt = settings->hstr;
    driver->chopconf.reg.hend = settings->hend;
    driver->chopconf.reg.hdec = settings->hdec;
    driver->chopconf.reg.rndtf = settings->rndtf;

    driver->sgcsconf.reg.sgt = settings->thresh;

    driver->smarten.reg.semin = settings->semin;
    driver->smarten.reg.seup = settings->seup;
    driver->smarten.reg.semax = settings->semax;
    driver->smarten.reg.sedn = settings->sedn;
    driver->smarten.reg.seimin = settings->seimin;

    driver->drvconf.reg.value = settings->drvconf;             
    

    //DRVCONF
    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvconf);

    //DRVCTRL
    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvctrl);

    //CHOPCONF
    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->chopconf);

    //SMARTEN  
    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->smarten);

    //SGCSCONF
    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->sgcsconf);

    TMC2660_SetMicrosteps(driver, (tmc2660_microsteps_t)driver->config.microsteps);
    TMC2660_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);
    
}

static bool vsense (uint8_t motor)
{
    return tmcdriver[motor]->drvconf.reg.vsense;
}

static bool read_register (uint8_t motor, uint8_t addr, uint32_t *val)
{
    TMC2660_datagram_t reg, read;
    reg.addr = (TMC_addr_t)addr;
    reg.addr.write = Off;
    TMC2660_ReadRegister(tmcdriver[motor], &read);

    reg =  *TMC2660_GetRegPtr (tmcdriver[motor], addr);    

    *val = reg.payload.value;

    return true;
}

static bool write_register (uint8_t motor, uint8_t addr, uint32_t val)
{
    TMC2660_datagram_t reg;
    reg.addr = (TMC_addr_t)addr;
    reg.addr.write = On;
    reg.payload.value = val;

    TMC2660_WriteRegister(tmcdriver[motor], &reg);

    return true;
}

static void *get_register_addr (uint8_t motor, uint8_t addr)
{
    return TMC2660_GetRegPtr(tmcdriver[motor], (tmc2660_regaddr_t)addr);
}

static uint8_t getGlobalScaler (uint8_t motor)
{
    //this is here to avoid a null pointer.
    return 0;
}

static uint32_t getTStep (uint8_t motor)
{
    //this is here to avoid a null pointer.
    return 0;
}

static void stealthChopEnable (uint8_t motor)
{
    //this is here to avoid a null pointer.
    return;
}

static float getTPWMThrs (uint8_t motor, float steps_mm)
{
    //this is here to avoid a null pointer.
    return 0;
}

static uint32_t getTPWMThrsRaw (uint8_t motor)
{
    //this is here to avoid a null pointer.
    return 0;
}

static void setTPWMThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    //this is here to avoid a null pointer.
    return;
}

static bool stealthChopGet (uint8_t motor)
{
    //this is here to avoid a null pointer.
    return tmcdriver[motor]->config.mode;
}

static void stealthChop (uint8_t motor, bool on)
{
    tmcdriver[motor]->config.mode = TMCMode_CoolStep;
    //coolStepEnable(motor);
}

static uint8_t pwm_scale (uint8_t motor)
{
    //this is here to avoid a null pointer.
    return 0;
}

static const tmchal_t tmchal = {
    .driver = TMC2660,
    .name = "TMC2660",
    .get_config = getConfig,

    .update_settings = updateSettings,

    .microsteps_isvalid = isValidMicrosteps,
    .set_microsteps = setMicrosteps,
    .set_current = setCurrent,
    .get_current = getCurrent,
    .get_chopconf = getChopconf,
    .get_tstep = getTStep, //not supported
    .get_drv_status = getDriverStatus,
    .get_drv_status_raw = getDriverStatusRaw,
    //.set_tcoolthrs = setTCoolThrs,
    //.set_tcoolthrs_raw = setTCoolThrsRaw,
    .set_thigh = NULL, //not suported
    .set_thigh_raw = NULL, //not supported
    .stallguard_enable = stallGuardEnable,
    .stealthchop_enable = stealthChopEnable, //not supported
    .coolstep_enable = coolStepEnable,
    .get_sg_result = getStallGuardResult,
    .get_tpwmthrs = getTPWMThrs,  //not supported
    .get_tpwmthrs_raw = getTPWMThrsRaw,  //not supported
    .set_tpwmthrs = setTPWMThrs,  //not supported
    .get_global_scaler = getGlobalScaler, //not supported
    .get_en_pwm_mode = stealthChopGet,  //not supported
    .get_ihold_irun = getIholdIrun,

    .stealthChop = stealthChop,  //not supported
    .sg_filter = sg_filter,
    .sg_stall_value = sg_stall_value,
    .get_sg_stall_value = get_sg_stall_value,
    .coolconf = coolconf,
    .vsense = vsense,
    .pwm_scale = pwm_scale, //not supported
    .chopper_timing = chopper_timing,

    .get_register_addr = get_register_addr,
    .read_register = read_register,  //not really supported, just returns the status register
    .write_register = write_register
};

static const tmchal_t nullhal = {
    .driver = NULLDRIVER,
    .name = "null",
    .get_config = NULL,

    .update_settings = NULL,

    .microsteps_isvalid = NULL,
    .set_microsteps = NULL,
    .set_current = NULL,
    .get_current = NULL,
    .get_chopconf = NULL,
    .get_tstep = NULL, //not supported
    .get_drv_status = NULL,
    .get_drv_status_raw = NULL,
    //.set_tcoolthrs = setTCoolThrs,
    //.set_tcoolthrs_raw = setTCoolThrsRaw,
    .set_thigh = NULL, //not suported
    .set_thigh_raw = NULL, //not supported
    .stallguard_enable = NULL,
    .stealthchop_enable = NULL, //not supported
    .coolstep_enable = NULL,
    .get_sg_result = NULL,
    .get_tpwmthrs = NULL,  //not supported
    .get_tpwmthrs_raw = NULL,  //not supported
    .set_tpwmthrs = NULL,  //not supported
    .get_global_scaler = NULL, //not supported
    .get_en_pwm_mode = NULL,  //not supported
    .get_ihold_irun = NULL,

    .stealthChop = NULL,  //not supported
    .sg_filter = NULL,
    .sg_stall_value = NULL,
    .get_sg_stall_value = NULL,
    .coolconf = NULL,
    .vsense = NULL,
    .pwm_scale = NULL, //not supported
    .chopper_timing = NULL,

    .get_register_addr = NULL,
    .read_register = NULL,  //not really supported, just returns the status register
    .write_register = NULL
};

const tmchal_t *TMC2660_AddMotor (motor_map_t motor, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC2660_t))) != NULL)) {
        TMC2660_SetDefaults(tmcdriver[motor.id]);
        tmcdriver[motor.id]->config.motor.id = motor.id;
        tmcdriver[motor.id]->config.motor.axis = motor.axis;
        tmcdriver[motor.id]->config.current = current;
        tmcdriver[motor.id]->config.microsteps = microsteps;
        tmcdriver[motor.id]->config.r_sense = r_sense;
    }

    if(ok && !(ok = TMC2660_Init(tmcdriver[motor.id]))) {
        free(tmcdriver[motor.id]);
        tmcdriver[motor.id] = NULL;
    }

    return ok ? &tmchal : NULL;
}

const tmchal_t *TMC2660_AddNULLMotor (motor_map_t motor)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC2660_t))) != NULL)) {
    }

    return ok ? &nullhal : NULL;
}
