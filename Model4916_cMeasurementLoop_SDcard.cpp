/*

Module: Catena4430_cMeasurementLoop_SDcard.cpp

Function:
    Class for running the SD Card

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include "Model4916_cMeasurementLoop.h"

#include "Model4916-MultiGas-Sensor.h"

#include <Catena_Download.h>

#include <Catena_Fram.h>

#include <Arduino_LoRaWAN_lmic.h>

#include <SD.h>
#include <mcciadk_baselib.h>

using namespace McciModel4916;
using namespace McciCatena;

/****************************************************************************\
|
|   globals
|
\****************************************************************************/

SDClass gSD;

/****************************************************************************\
|
|   Some utilities
|
\****************************************************************************/

// turn on power to the SD card
void cMeasurementLoop::sdPowerUp(bool fOn)
    {
    gpiopower.setVSpi(fOn);
    }

void cMeasurementLoop::sdPrep()
    {
    digitalWrite(cMeasurementLoop::kSdCardCSpin, 1);
    pinMode(cMeasurementLoop::kSdCardCSpin, OUTPUT);
    if (! this->m_fSpi2Active)
        {
        this->m_pSPI2->begin();
        this->m_fSpi2Active = true;
        }

    digitalWrite(cMeasurementLoop::kSdCardCSpin, 1);
    gpiopower.configVSpi(true);
    this->sdPowerUp(true);
    delay(100);
    }

void cMeasurementLoop::sdFinish()
    {
    // gSD.end() calls card.forceIdle() which will
    // (try to) put the card in the idle state.
    if (! gSD.end())
        {
        gCatena.SafePrintf("gSD.end() timed out\n");
        }

    // turn off CS to avoid locking Vsdcard on.
    this->m_pSPI2->end();
    this->m_fSpi2Active = false;
    pinMode(Catena::PIN_SPI2_MOSI, OUTPUT);
    pinMode(Catena::PIN_SPI2_MISO, OUTPUT);
    pinMode(Catena::PIN_SPI2_SCK, OUTPUT);
    digitalWrite(Catena::PIN_SPI2_MOSI, 0);
    digitalWrite(Catena::PIN_SPI2_MISO, 0);
    digitalWrite(Catena::PIN_SPI2_SCK, 0);
    digitalWrite(cMeasurementLoop::kSdCardCSpin, 0);
    delay(1);
    this->sdPowerUp(false);
    }

/*

Name:   McciCatena4430::cMeasurementLoop::fillTxBuffer()

Function:
    Prepare a messages in a TxBuffer with data from current measurements.

Definition:
    void McciCatena4430::cMeasurementLoop::fillTxBuffer(
            cMeasurementLoop::TxBuffer_t& b
            );

Description:
    A format 0x22 message is prepared from the data in the cMeasurementLoop
    object.

*/

bool
cMeasurementLoop::initSdCard(
    )
    {
    bool fResult = this->checkSdCard();

    sdFinish();
    return fResult;
    }

bool
cMeasurementLoop::checkSdCard()
    {
    sdPrep();
    if (! gSD.begin(gSPI2, SPI_HALF_SPEED, kSdCardCSpin))
        return false;
    else
        return true;
    }

static const char kHeader[] =
    "SNo,Version,Timestamp,Mode,DevEUI,Raw,Uplink Port,Vbat,BootCount,Latitude,Longitude,"
    "T,RH,TVOC,IAQ,PC0.1,PC0.3,PC0.5,PC1.0,PC2.5,PC5.0,PC10,"
    "PM0.1,PM0.3,PM0.5,PM1.0,PM2.5,PM5.0,PM10,CO2(PPM),"
    "CO(PPM),NO2(PPM),O3(PPM),SO2(PPM),CO(uV),NO2(uV),O3(uV),SO2(uV)"
    "\n";

bool
cMeasurementLoop::writeSdCard(
    cMeasurementLoop::TxBuffer_t &b,
    cMeasurementLoop::Measurement const & mData
    )
    {
    bool fResult;
    File dataFile;

    fResult = this->checkSdCard();
    if (! fResult)
        gCatena.SafePrintf("** SD card not detected!\n");

    if (fResult)
        {
        // make a directory
        fResult = gSD.mkdir("Data");
        if (! fResult)
            gCatena.SafePrintf("mkdir failed\n");
        }

    if (fResult)
        {
        char fName[42];
        bool fNew;

        McciAdkLib_Snprintf(fName, sizeof(fName), 0, "Data/%02u.dat", mData.BootCount);

        fNew = !gSD.exists(fName);
        if (fNew)
            {
            //gCatena.SafePrintf("%s not found, will create & write header\n", fName);
            }

        File dataFile = gSD.open(fName, FILE_WRITE);
        if (dataFile)
            {
            if (fNew)
                {
                //gCatena.SafePrintf("write header\n");
                for (auto i : kHeader)
                    {
                    if (i == '\n')
                        {
                        dataFile.println();
                        }
                    else if (i == '\0')
                        {
                        break;
                        }
                    else
                        {
                        dataFile.print(i);
                        }
                    }
                }

            char buf[32];

            dataFile.print(this->m_sno);
            dataFile.print(',');

            dataFile.print(this->kMajor);
            dataFile.print('.');
            dataFile.print(this->kMinor);
            dataFile.print('.');
            dataFile.print(this->kPatch);
            dataFile.print('.');
            dataFile.print(this->kLocal);
            dataFile.print(',');

            if ((mData.flags & Flags::Time) != Flags(0))
                {
                dataFile.print(mData.position.UnixTime);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::Mode) != Flags(0))
                {
                dataFile.print("Mobile");
                }
            else
                {
                dataFile.print("Fixed");
                }
            dataFile.print(',');

            //gCatena.SafePrintf("write DevEUI");
            do  {
                CatenaBase::EUI64_buffer_t devEUI;

            	auto const pFram = gCatena.getFram();

                // use devEUI.
                if (pFram != nullptr &&
                    pFram->getField(cFramStorage::StandardKeys::kDevEUI, devEUI))
                    {
                    dataFile.print('"');

                    /* write the devEUI */
                    for (auto i = 0; i < sizeof(devEUI.b); ++i)
                        {
                        // the devEUI is stored in little-endian order.
                        McciAdkLib_Snprintf(
                            buf, sizeof(buf), 0,
                            "%02x", devEUI.b[sizeof(devEUI.b) - i - 1]
                            );
                        dataFile.print(buf);
                        }

                    dataFile.print('"');
                    }
                } while (0);

            dataFile.print(',');

            //gCatena.SafePrintf("write raw hex\n");
            dataFile.print('"');
            for (unsigned i = 0; i < b.getn(); ++i)
                {
                McciAdkLib_Snprintf(
                    buf, sizeof(buf), 0,
                    "%02x",
                    b.getbase()[i]
                    );
                dataFile.print(buf);
                }

            dataFile.print("\",");

            dataFile.print(kUplinkPort);
            dataFile.print(',');

            //gCatena.SafePrintf("write Vbat\n");
            if ((mData.flags & Flags::Vbat) != Flags(0))
               dataFile.print(mData.Vbat);

            dataFile.print(',');

            if ((mData.flags & Flags::Boot) != Flags(0))
                dataFile.print(mData.BootCount);

            dataFile.print(',');

            if ((mData.flags & Flags::GPS) != Flags(0))
                {
                dataFile.print(mData.position.Latitude, 7);
                // dataFile.print('.');
                // dataFile.print(this->getGpsDecimal(mData.position.Latitude));
                dataFile.print(',');

                dataFile.print(mData.position.Longitude, 7);
                // dataFile.print('.');
                // dataFile.print(this->getGpsDecimal(mData.position.Longitude));
                dataFile.print(',');
                }
            else
                {
                dataFile.print(",,");
                }

            if ((mData.flags & Flags::TH) != Flags(0))
                {
                dataFile.print(mData.env.TempC);
                dataFile.print(',');

                dataFile.print(mData.env.Humidity);
                dataFile.print(',');
                }
            else
                {
                dataFile.print(",,");
                }

            if ((mData.flags & Flags::TVOC) != Flags(0))
                {
                dataFile.print(mData.airQuality.BreathVoc);
                }
            dataFile.print(',');

            if ((mData.flags & Flags::IAQ) != Flags(0))
                {
                dataFile.print(mData.airQuality.Iaq);
                }
            dataFile.print(',');

            if ((mData.flags & Flags::PM) != Flags(0))
                {
                for (uint8_t i = 0; i < 7; i++) {
                    dataFile.print(mData.particle.Count[i]);
                    dataFile.print(',');
                    }
                for (uint8_t i = 0; i < 7; i++) {
                    dataFile.print(mData.particle.Mass[i]);
                    dataFile.print(',');
                    }
                }
            else
                {
                dataFile.print(",,,,,,,,,,,,,,");
                }

            if ((mData.flags2 & Flags::CO2) != Flags(0))
                {
                dataFile.print(this->co2int);
                dataFile.print('.');
                dataFile.print(this->co2frac);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::CO) != Flags(0))
                {
                dataFile.print(mData.gases.CO);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::NO2) != Flags(0))
                {
                dataFile.print(mData.gases.NO2);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::O3) != Flags(0))
                {
                dataFile.print(mData.gases.O3);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::SO2) != Flags(0))
                {
                dataFile.print(mData.gases.SO2);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::CO) != Flags(0))
                {
                dataFile.print(mData.gases.uVCO);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::NO2) != Flags(0))
                {
                dataFile.print(mData.gases.uVNO2);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::O3) != Flags(0))
                {
                dataFile.print(mData.gases.uVO3);
                }
            dataFile.print(',');

            if ((mData.flags2 & Flags::SO2) != Flags(0))
                {
                dataFile.print(mData.gases.uVSO2);
                }
            dataFile.print(',');

            dataFile.println();
            dataFile.close();
            }
        else
            {
            gCatena.SafePrintf("can't open: %s\n", fName);
            }
        }

    sdFinish();
    return fResult;
    }

/*

Name:	cMeasurementLoop::handleSdFirmwareUpdate()

Index:  Name:   cMeasurementLoop::handleSdFirmwareUpdateCardUp()

Function:
    Check for firmware update request via SD card and handle it

Definition:
    bool cMeasurementLoop::handleSdFirmwareUpdate(
        void
        );

    bool cMeasurementLoop::handleSdFirmwareUpdateCardUp(
        void
        );

Description:
    Check for a suitable file on the SD card. If found, copy to
    flash. If successful, set the update flag, and rename the
    update file so we won't consider it again.  handleSdFirmwareUpdateCardUp()
    is simply the inner method, to be called as a wrapper once power is
    up on the card.

Returns:
    True if an update was done and the system should be rebooted.

Notes:
    

*/

#define FUNCTION "cMeasurementLoop::handleSdFirmwareUpdate"

bool
cMeasurementLoop::handleSdFirmwareUpdate(
    void
    )
    {
    if (this->m_pSPI2 == nullptr)
        gLog.printf(gLog.kBug, "SPI2 not registered, can't program flash\n");

    bool fResult = this->checkSdCard();
    if (fResult)
        {
        fResult = this->handleSdFirmwareUpdateCardUp();
        }
    this->sdFinish();
    return fResult;
    }

#undef FUNCTION

#define FUNCTION "cMeasurementLoop::handleSdFirmwareUpdateCardUp"

bool
cMeasurementLoop::handleSdFirmwareUpdateCardUp(
    void
    )
    {
    static const char * const sUpdate[] = { "update.bin", "fallback.bin" };

    for (auto s : sUpdate)
        {
        if (! gSD.exists(s))
            {
            if (gLog.isEnabled(gLog.kTrace))
                gLog.printf(gLog.kAlways, "%s: not found: %s\n", FUNCTION, s);
            continue;
            }

        auto result = this->updateFromSd(
                            s,
                            s[0] == 'u' ? cDownload::DownloadRq_t::GetUpdate
                                        : cDownload::DownloadRq_t::GetFallback
                            );
        if (gLog.isEnabled(gLog.kTrace))
            gLog.printf(gLog.kTrace, "%s: applied update from %s: %s\n", FUNCTION, s, result ? "true": "false");
        return result;
        }

    return false;
    }

bool
cMeasurementLoop::updateFromSd(
    const char *sUpdate,
    cDownload::DownloadRq_t rq
    )
    {
    // launch a programming cycle. We'll stall the measurement FSM here while
    // doing the operation, but poll the other FSMs.
    struct context_t
            {
            cMeasurementLoop *pThis;
            bool fWorking;
            File firmwareFile;
            cDownload::Status_t status;
            cDownload::Request_t request;
            };
    context_t context { this, true };

    this->m_fFwUpdate = true;

    pinMode(D13, OUTPUT);
    digitalWrite(D13, HIGH);
    gLog.printf(gLog.kInfo, "Attempting to load firmware from %s\n", sUpdate);

    // power management: typically the SPI2 is powered down by a sleep,
    // and it's not powered back up when we wake up. The SPI flash is on
    // SPI2, and so we must power it up. We also have to deal with the
    // corner case where the SPI flash didn't probe correctly.
    if (this->m_pSPI2)
        {
        // SPI was found, bring it up.
        this->m_pSPI2->begin();
        // and bring up the flash.
        gFlash.begin(this->m_pSPI2, Catena::PIN_SPI2_FLASH_SS);
        }
    else
        {
        // something went wrong at boot time, we can't do anything
        // with firwmare update...
        gLog.printf(gLog.kError, "SPI2 pointer is null, give up\n");
        return false;
        }

    // try to open the file
    context.firmwareFile = gSD.open(sUpdate, FILE_READ);

    if (! context.firmwareFile)
        {
        // hmm. it exists but we could not open it.
        gLog.printf(gLog.kError, "%s: exists but can't open: %s\n", FUNCTION, sUpdate);
        return false;
        }

    // the downloader requires a "request block" that tells it what to do.
    // since we loop in this function, we can allocate it as a local variable,
    // and keep it in the context object. Save some typing by defining an
    // alias:
    auto & request = context.request;

    // The downloader is abstract; it doesn't know where data is coming from.
    // It calls various callbacks to get data and orchestrate the collection
    // of image data. So we (the client) must provide some callbacks. This
    // is done in a consistent way:
    //
    //  request.{callback}.init({pfn}, {pUserData});
    //
    // This initializes the specified callback with the specified function
    // and user data.
    //
    // In the code below, we use C++ lambdas to save useless static functions,
    // and also (nicely) to write code that is "inside" cMeasurementLoop and
    // can access protected and private items.
    //
    // The four callbacks are: QueryAvailableData, PromptForData, ReadBytes,
    // and Completion.
    //

    // initialize the "query available data" callback. We always say
    // kTransferChunkBytes are available, becuase we're reading from
    // a file. But this means we must fill buffer to max size in read
    // when we hit end of file.
    request.QueryAvailableData.init(
        [](void *pUserData) -> int
            {
            return cDownload::kTransferChunkBytes;
            },
        nullptr
        );
    
    // initalize the "prompt for more data" callback; we don't need one
    // when reading from a file.
    request.PromptForData.init(nullptr, nullptr);

    // initialize the read-byte callback
    request.ReadBytes.init(
        // this is called each time the downloader wants more data
        [](void *pUserData, std::uint8_t *pBuffer, size_t nBuffer) -> size_t
            {
            context_t * const pCtx = (context_t *)pUserData;

            gLog.printf(gLog.kInfo, ".");
            gCatena.poll();

            auto n = pCtx->firmwareFile.readBytes(pBuffer, nBuffer);
            if (n < nBuffer)
                {
                // at end of file we have spare bytes that are not
                // used. Initialize to 0xFF because that's nice for
                // SPI flash.
                memset(pBuffer + n, 0xFF, nBuffer - n);
                gLog.printf(gLog.kInfo, "\n");
                }

            return nBuffer;
            },
        (void *)&context
        );

    // initialize the "operation complete" callback. Just set
    // a flag to get us out of the wait loop.
    request.Completion.init(
        [](void *pUserData, cDownload::Status_t status) -> void
            {
            context_t * const pCtx = (context_t *)pUserData;

            pCtx->status = status;
            pCtx->fWorking = false;
            },
        (void *)&context
        );

    // set the request code in the request.
    request.rq = rq;

    // launch the request.
    if (! gDownload.evStart(request))
        {
        // It didn't launch. No callbacks will happen. Clean up.
        context.firmwareFile.close();
        // remove the file, so we don't get stuck in a loop.
        gSD.remove(sUpdate);
        // no need to reboot.
        return false;
        }

    // it launched: wait for transfer to complete
    while (context.fWorking)
        // give other clients a chance to look in.
        // and allow the download to be coded asynchronously
        // if necessary.
        gCatena.poll();

    // download operation is complete.
    // close and remove the file
    context.firmwareFile.close();
    gSD.remove(sUpdate);

    // if it failed, display the error code.
    if (context.status != cDownload::Status_t::kSuccessful)
        {
        gLog.printf(gLog.kError, "download failed, status %u\n", std::uint32_t(context.status));
        // no need to reboot.
        return false;
        }
    // if it succeeeded, say so, and tell caller to reboot.
    // don't reboot here, because the outer app may need to shut things down
    // in an orderly way.
    else
        {
        gLog.printf(gLog.kInfo, "download succeded.\n");
        return true;
        }
    }

#undef FUNCTION
