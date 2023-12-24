/*

Name:   Model4916-MultiGas-Sensor.h

Function:
    Global linkage for Model4916-MultiGas-Sensor.ino

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#ifndef _Model4916_LoRawan_h_
# define _Model4916_LoRawan_h_

#pragma once

#include <Catena.h>
#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Timer.h>
#include <Catena_BootloaderApi.h>
#include <Catena_Download.h>
#include <SD.h>
#include <SPI.h>
#include <Model4916_c4916Gpios.h>
#include <Model4916_cPCA9574.h>
#include "Model4916_cMeasurementLoop.h"

// the global clock object

extern  McciCatena::Catena                      gCatena;

extern  McciModel4916::c4916Gpios               gpiopower;
extern  McciModel4916::c4916Gpios               gpioenable;

extern  McciCatena::Catena::LoRaWAN             gLoRaWAN;
extern  McciCatena::StatusLed                   gLed;
extern  McciCatenaScd30::cSCD30                 gScd;

extern  SPIClass                                gSPI2;
extern  McciModel4916::cMeasurementLoop         gMeasurementLoop;

// the bootloader
extern  McciCatena::cBootloaderApi              gBootloaderApi;

// the downloaer
extern  McciCatena::cDownload                   gDownload;

// the SD card
extern  SDClass                                 gSD;

//   The flash
extern  McciCatena::Catena_Mx25v8035f           gFlash;

#endif // !defined(_Model4916_LoRawan_h_)
