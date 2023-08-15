/*
 * tmc2660.c - interface for Trinamic TMC2660 stepper driver
 *
 * v0.0.3 / 2021-10-17 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021, Terje Io
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

/*
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2660_Calculations.xlsx
 *
 */

#include <string.h>
#include <math.h>
#include "tmc2660.h"
#include "driver.h"

static const TMC2660_t tmc2660_defaults = {
    .config.f_clk = TMC2660_F_CLK,
    .config.mode = TMC2660_MODE,
    .config.r_sense = TMC2660_R_SENSE,
    .config.current = TMC2660_CURRENT,
    .config.hold_current_pct = TMC2660_HOLD_CURRENT_PCT,
    .config.microsteps = TMC2660_MICROSTEPS,

    // register adresses
    .drvctrl.addr.value = TMC2660Reg_DRVCTRL,
    .drvctrl.reg.dedge = TMC2660_DEDGE,
    .drvctrl.reg.intpol = TMC2660_INTPOL,
    .drvctrl.reg.mres = TMC2660_MICROSTEPS,

    .chopconf.addr.value = TMC2660Reg_CHOPCONF,
    .chopconf.reg.toff = TMC2660_CONSTANT_OFF_TIME,
    .chopconf.reg.chm = TMC2660_CHOPPER_MODE,
    .chopconf.reg.tbl = TMC2660_BLANK_TIME,
    .chopconf.reg.hstrt = TMC2660_HSTR,
    .chopconf.reg.hend = TMC2660_HEND,
    .chopconf.reg.hdec = TMC2660_HDEC,
    .chopconf.reg.rndtf = TMC2660_RNDTF,

    .sgcsconf.addr.value = TMC2660Reg_SGCSCONF,
    .sgcsconf.reg.cs = TMC2660_CURRENT_SCALE,
    .sgcsconf.reg.sfilt = TMC2660_SG_FILTER,
    .sgcsconf.reg.sgt = TMC2660_SG_THRESH,

    .drvconf.addr.value = TMC2660Reg_DRVCONF,
    .drvconf.reg.value = TMC2660_DRVCONF,

    .smarten.addr.value = TMC2660Reg_SMARTEN,
    .smarten.reg.sedn = TMC2660_SEDN,
    .smarten.reg.seimin = TMC2660_SEIMIN,
    .smarten.reg.semax = TMC2660_SEMAX,
    .smarten.reg.seup = TMC2660_SEUP,
    .smarten.reg.semin = TMC2660_SEMIN,
};

bool TMC2660_Init (TMC2660_t *driver)
{
    #if 0
    
    TMC_spi_datagram_t gram = {0};

    // Read drv_status to check if driver is online
    //tmc2660_spi_read(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvstatus);
    //if(driver->drvstatus.reg.value == 0 || driver->drvstatus.reg.value == 0xFFFFFFFF)
    //    return false;

    //DRVCTRL
    //4 microsteps and disable interpolation
    gram.addr.value = 0x0003;//16 bit shift
    //gram.addr.value = TMC2660Reg_DRVCTRL;
    gram.payload.value = 0x00206;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&gram);    

    //CHOPCONF
    //098511 is good alternate
    gram.addr.value = 0x09;//16 bit shift
    //gram.addr.value = TMC2660Reg_CHOPCONF;
    //gram.payload.value = 0x0181;  //most efficient (0x09 addr)
    gram.payload.value = 0x2321;  //most efficient (0x09 addr) rndtf=1
    //gram.payload.value = 0x2181;  //most efficient (0x09 addr) rndtf=1
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&gram);

    //SMARTEN
    gram.addr.value = 0x0A;//16 bit shift
    //gram.addr.value = TMC2660Reg_SMARTEN;
    //gram.payload.value = 0x0000;
    // Enable CoolStep with minimum current 1/2 CS?
    //gram.payload.value = 0x8202;
    //gram.payload.value = 0x6062; //so far like this one.
    //gram.payload.value = 0x6061;
    //gram.payload.value = 0xAA62;
    gram.payload.value = 0xE560;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&gram);   

    //SGCSCONF
    gram.addr.value = 0x0D;//16 bit shift
    //gram.addr.value = TMC2660Reg_SGCSCONF;
    //gram.payload.value = 0x0219; //1.4A current 2 stallguard //don't use this with 0.05R sense
    //gram.payload.value = 0x020D; //3.1A current 2 stallguard
    gram.payload.value = 0x160D; //3.1A current 22 stallguard
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&gram);

    //DRVCONF
    //low driver strength, StallGuard2 read, SDOFF=0
    gram.addr.value = 0x0E;//16 bit shift
    //gram.addr.value = TMC2660Reg_DRVCONF;
    gram.payload.value = 0x0F10;
    //gram.payload.value = TMC2660_DRVCONF;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&gram);

    // Read drv_status to check if driver is online
    //tmc2660_spi_read(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvstatus);
    //if(driver->drvstatus.reg.value == 0 || driver->drvstatus.reg.value == 0xFFFFFFFF)
    //    return false;    

    stallguard thres of 17 seems to work ok?

    #endif         
    
    #if 1
    // Read drv_status to check if driver is online
    uint32_t ms = hal.get_elapsed_ticks() + 250;
    bool status = false;

    while(ms >= hal.get_elapsed_ticks() && status == false) {
        tmc2660_spi_read(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvstatus);
        if(driver->drvstatus.reg.value == 0 || driver->drvstatus.reg.value == 0xFFFFFF){
            status = false;
        } else{
            status = true;
        }
        HAL_Delay(5);
    }       
if (status == false)
    return status;

TMC_spi_datagram_t gram = {0};

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

    #endif

    return 1;
}

uint_fast16_t cs2rms_2660 (TMC2660_t *driver, uint8_t CS)
{
    int v_sense;
    
    if(driver->drvconf.reg.vsense)
        v_sense = 165;
    else   
        v_sense = 325;
    
    //float iRMS = (248.0/256.0) * ((CS + 1.0) / 32.0) * (v_sense / (float)driver->config.r_sense) * (1.0 / sqrt(2.0));
    float iRMS = (248.0/256.0) * ((CS + 1.0) / 32.0) * (v_sense / (float)TMC2660_R_SENSE) * (1.0 / sqrt(2.0));

    return (uint_fast16_t) (iRMS*1000);
    return 0;
}

uint16_t TMC2660_GetCurrent (TMC2660_t *driver)
{
    return cs2rms_2660(driver, driver->sgcsconf.reg.cs);
    return 0;
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC2660_SetCurrent (TMC2660_t *driver, uint16_t mA, uint8_t hold_pct)
{
    
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    float maxv = ((float)TMC2660_R_SENSE * (float)driver->config.current * 32.0f) / 1000000.0f;

    uint8_t current_scaling = (uint8_t)((maxv / 0.31f) - 0.5f);

    if ((driver->drvconf.reg.vsense = (current_scaling < 16)))
        current_scaling = (uint8_t)((maxv / 0.165f) - 0.5f);

    driver->sgcsconf.reg.cs = current_scaling > 31 ? 31 : current_scaling;

    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvconf);
    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->sgcsconf);
}

// 1 - 256 in steps of 2^value is valid for TMC2660
bool TMC2660_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC2660_SetMicrosteps (TMC2660_t *driver, tmc2660_microsteps_t usteps)
{
    driver->drvctrl.reg.mres = tmc_microsteps_to_mres(usteps);

    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvctrl);
}

void TMC2660_SetConstantOffTimeChopper (TMC2660_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
{
    
    #if 0
    //calculate the value acc to the clock cycles
    if (blank_time >= 54)
        blank_time = 3;
    else if (blank_time >= 36)
        blank_time = 2;
    else if (blank_time >= 24)
        blank_time = 1;
    else
        blank_time = 0;

    if (fast_decay_time > 15)
        fast_decay_time = 15;

    //if(driver->chopconf.reg.chm)
    //    driver->chopconf.reg.fd3 = (fast_decay_time & 0x8) >> 3;

    driver->chopconf.reg.tbl = blank_time;
    driver->chopconf.reg.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    driver->chopconf.reg.hstrt = fast_decay_time & 0x7;
    driver->chopconf.reg.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;

    tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->chopconf);
    #endif
}

void TMC2660_SetDefaults (TMC2660_t *driver)
{
    memcpy(driver, &tmc2660_defaults, sizeof(TMC2660_t));
}

TMC2660_status_t TMC2660_WriteRegister (TMC2660_t *driver, TMC2660_datagram_t *reg)
{
    TMC2660_status_t status;

    status.value = tmc2660_spi_write(driver->config.motor, (TMC2660_spi_datagram_t *)reg);

    return status;
}

TMC2660_status_t TMC2660_ReadRegister (TMC2660_t *driver, TMC2660_datagram_t *reg)
{
    TMC2660_status_t status;

    //TMC2660 does not support register reads.  Just return the shadow register value and driver status?
    status.value = tmc2660_spi_read(driver->config.motor, (TMC2660_spi_datagram_t *)&driver->drvstatus);

    return status;
}

// Returns pointer to shadow register or NULL if not found
TMC2660_datagram_t *TMC2660_GetRegPtr (TMC2660_t *driver, tmc2660_regaddr_t reg)
{
    TMC2660_datagram_t *ptr = (TMC2660_datagram_t *)driver;

    while(ptr && ptr->addr.value != reg) {
        ptr++;
    }

    return ptr;
}
