/*

Module: Model4916-MultiGas-Sensor.ino

Function:
    Code for the Multiple Gas sensor based on Model 4916

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include <Arduino.h>
#include <Wire.h>
#include <Catena.h>
#include <arduino_lmic.h>
#include <Catena_Timer.h>
#include <Model4916.h>
#include <Model4916_cPCA9574.h>
#include <Catena_BootloaderApi.h>
#include <Catena_Download.h>
#include <Model4916_c4916Gpios.h>
#include "Model4916-MultiGas-Sensor.h"
#include "Model4916_cMeasurementLoop.h"
#include "Model4916_cmd.h"
#include <MCCI_Catena_SCD30.h>

using namespace McciModel4916;
using namespace McciCatena;
using namespace McciCatenaScd30;
using namespace McciCatenaAds131m04;
using namespace McciCatenaIps7100;
using namespace McciCatenaSht3x;
using namespace McciCatenaSamM8q;
//using namespace SDLib;

static_assert(
    CATENA_ARDUINO_PLATFORM_VERSION_COMPARE_GE(
        CATENA_ARDUINO_PLATFORM_VERSION,
        CATENA_ARDUINO_PLATFORM_VERSION_CALC(0, 21, 0, 5)
        ),
    "This sketch requires Catena-Arduino-Platform v0.21.0-5 or later"
    );

static const char sVersion[] = "1.5.2-pre2";

/****************************************************************************\
|
|   Variables.
|
\****************************************************************************/

// the global I2C GPIO object for enabling power
cPCA9574                i2cgpiopower    { &Wire, 0 };
c4916Gpios gpiopower    { &i2cgpiopower };

// the global I2C GPIO object for enabling communication
cPCA9574                i2cgpioenable    { &Wire, 1 };
c4916Gpios gpioenable   { &i2cgpioenable };

Catena gCatena;

Catena::LoRaWAN gLoRaWAN;
StatusLed gLed (Catena::PIN_STATUS_LED);

// the Temperature/Humidity Sensor
cSHT3x gSht { Wire };

// the CO2 Sensor
cSCD30 gScd { Wire };

// the Particle sensor
cIPS7100 gIps {Wire};

// the ADC for spec sensor
cADS131M04 gAds;

// GPS sensor
cSAM_M8Q gGps;

// the measurement loop instance
cMeasurementLoop gMeasurementLoop { gSht, gScd, gIps };

/* instantiate the bootloader API */
cBootloaderApi gBootloaderApi;

/* instantiate SPI */
SPIClass gSPI2(
		Catena::PIN_SPI2_MOSI,
		Catena::PIN_SPI2_MISO,
		Catena::PIN_SPI2_SCK
		);

/* instantiate the flash */
Catena_Mx25v8035f gFlash;

/* instantiate the downloader */
cDownload gDownload;

/****************************************************************************\
|
|   User commands
|
\****************************************************************************/

// the individual commmands are put in this table
static const cCommandStream::cEntry sMyExtraCommmands[] =
        {
        { "dir", cmdDir },
        { "log", cmdLog },
        { "tree", cmdDir },
        { "setsensitivity", cmdGasSensitivity },
        { "getsensitivity", cmdGasSensitivity },
        // other commands go here....
        };

/* a top-level structure wraps the above and connects to the system table */
/* it optionally includes a "first word" so you can for sure avoid name clashes */
static cCommandStream::cDispatch
sMyExtraCommands_top(
        sMyExtraCommmands,          /* this is the pointer to the table */
        sizeof(sMyExtraCommmands),  /* this is the size of the table */
        nullptr                     /* this is no "first word" for all the commands in this table */
        );


/****************************************************************************\
|
|   Setup
|
\****************************************************************************/

void setup()
    {
    pinMode(D12, OUTPUT);
    digitalWrite(D12, HIGH);
    pinMode(D10, OUTPUT);
    digitalWrite(D10, HIGH);
    pinMode(D13, OUTPUT);
    digitalWrite(D13, HIGH);
    Serial.begin(115200);
    Wire.begin();
    setup_gpio();
    setup_platform();
    setup_printSignOn();

    setup_flash();
    setup_download();
    setup_measurement();
    setup_radio();
    setup_commands();
    setup_start();
    }

void setup_gpio()
    {
    if (! gpiopower.begin())
        Serial.println("GPIO to power-up modules failed to initialize");

    delay(100);

    if (! gpioenable.begin())
        Serial.println("GPIO to enable I2C interface failed to initialize");
    }

void setup_platform()
    {
    // power-up and enable FRAM
    gpiopower.configVdd1(true);
    gpioenable.configVdd(true);
    gpiopower.setVdd1(true);
    delay(2);
    gpioenable.enableVdd1(true);
    delay(2);

    gCatena.begin();

    // power-up Flash
    gpiopower.configVSpi(true);
    delay(2);
    gpiopower.setVSpi(true);

    // if running unattended, don't wait for USB connect.
    if (! (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
        {
        while (!Serial)
            /* wait for USB attach */
            yield();
        }

    // set up the LED
    gLed.begin();
    gCatena.registerObject(&gLed);
    gLed.Set(LedPattern::FastFlash);
    }

static constexpr const char *filebasename(const char *s)
    {
    const char *pName = s;

    for (auto p = s; *p != '\0'; ++p)
        {
        if (*p == '/' || *p == '\\')
            pName = p + 1;
        }
    return pName;
    }

void setup_printSignOn()
    {
    static const char dashes[] = "------------------------------------";

    gCatena.SafePrintf("\n%s%s\n", dashes, dashes);

    gCatena.SafePrintf("This is %s (Mobile Mode) v%s.\n",
        filebasename(__FILE__),
        sVersion
        );

    do
        {
        char sRegion[16];
        gCatena.SafePrintf("Target network: %s / %s\n",
                        gLoRaWAN.GetNetworkName(),
                        gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                        );
        } while (0);

    gCatena.SafePrintf("System clock rate is %u.%03u MHz\n",
        ((unsigned)gCatena.GetSystemClockRate() / (1000*1000)),
        ((unsigned)gCatena.GetSystemClockRate() / 1000 % 1000)
        );
    gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
    gCatena.SafePrintf("%s%s\n" "\n", dashes, dashes);

    Catena::UniqueID_string_t CpuIDstring;

    gCatena.SafePrintf(
            "CPU Unique ID: %s\n",
            gCatena.GetUniqueIDstring(&CpuIDstring)
            );

    /* find the platform */
    const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

    uint32_t flags;
    const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

    if (pPlatform)
        {
        gCatena.SafePrintf("EUI64: ");
        for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
            {
            gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
            }
        gCatena.SafePrintf("\n");
        flags = gCatena.GetPlatformFlags();
        gCatena.SafePrintf("Platform Flags:  %#010x\n", flags);
        gCatena.SafePrintf("Operating Flags:  %#010x\n",
            gCatena.GetOperatingFlags()
            );
        }
    else
        {
        gCatena.SafePrintf("**** no platform, check provisioning ****\n");
        flags = 0;
        }

    /* is it modded? */
    uint32_t modnumber = gCatena.PlatformFlags_GetModNumber(flags);

    /* modnumber is 102 for WeRadiate app */
    if (modnumber != 0)
        {
        gCatena.SafePrintf("Model 4916-M%u\n", modnumber);
        }
    else
        {
        gCatena.SafePrintf("No mods detected\n");
        }
    }

void setup_flash(void)
    {
    gSPI2.begin();
    if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
        {
        gMeasurementLoop.registerSecondSpi(&gSPI2);
        gFlash.powerDown();
        gCatena.SafePrintf("FLASH found, put power down\n");
        }
    else
        {
        gFlash.end();
        gSPI2.end();
        gCatena.SafePrintf("No FLASH found: check hardware\n");
        }

    gpiopower.setVSpi(false);
    }

void setup_download()
    {
    gDownload.begin(gFlash, gBootloaderApi);
    }

void setup_radio()
    {
    gLoRaWAN.begin(&gCatena);
    gCatena.registerObject(&gLoRaWAN);
    LMIC_setClockError(5 * MAX_CLOCK_ERROR / 100);
    }

void setup_measurement()
    {
    gMeasurementLoop.begin();
    }

void setup_commands()
    {
    /* add our application-specific commands */
    gCatena.addCommands(
        /* name of app dispatch table, passed by reference */
        sMyExtraCommands_top,
        /*
        || optionally a context pointer using static_cast<void *>().
        || normally only libraries (needing to be reentrant) need
        || to use the context pointer.
        */
        nullptr
        );
    }

void setup_start()
    {
    gMeasurementLoop.requestActive(true);
    }

/****************************************************************************\
|
|   Loop
|
\****************************************************************************/

void loop()
    {
    gCatena.poll();
    }
