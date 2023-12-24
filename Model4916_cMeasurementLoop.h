/*

Module: Model4916_cMeasurementLoop.h

Function:
    cMeasurementLoop definitions.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#ifndef _Model4916_cMeasurementLoop_h_
# define _Model4916_cMeasurementLoop_h_

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Catena_FSM.h>
#include <Catena_Led.h>
#include <Catena_Log.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_PollableInterface.h>
#include <Catena_Timer.h>
#include <Catena_BootloaderApi.h>
#include <Catena_Download.h>
#include <Catena_TxBuffer.h>
#include <Catena.h>
#include <mcciadk_baselib.h>
#include <stdlib.h>
#include <bsec.h>
#include <Catena-SHT3x.h>
#include <MCCI_Catena_SCD30.h>
#include <MCCI_Catena_IPS-7100.h>
#include <MCCI_Catena_ADS131M04.h>
#include <MCCI_Catena_SAM-M8Q.h>

#include <cstdint>

const uint8_t bsec_config_iaq[] = {
#include "bsec_config.h"
};

extern McciCatena::Catena gCatena;
extern McciCatena::Catena::LoRaWAN gLoRaWAN;
extern McciCatena::StatusLed gLed;

using namespace McciCatenaSht3x;
using namespace McciCatenaAds131m04;
using namespace McciCatenaIps7100;
using namespace McciCatenaSamM8q;

constexpr uint32_t STATE_SAVE_PERIOD = 3 * 60 * 1000; // 3 minutes: basically any change.

namespace McciModel4916 {

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

class cMeasurementBase
    {

    };

class cMeasurementFormat : public cMeasurementBase
    {
public:
    // buffer size for uplink data
    static constexpr size_t kTxBufferSize = 64;// 58;

    // message format
    static constexpr uint8_t kMessageFormat = 0x27;

    enum class Flags : uint16_t
            {
            Vbat = 1 << 0,      // vBat
            Boot = 1 << 1,      // boot count
            TH = 1 << 2,        // temperature, humidity
            Time = 1 << 3,      // timestamp
            GPS = 1 << 4,       // latitude, longitude
            PM = 1 << 5,        // Particle
            TVOC = 1 << 6,     // TVOC
            IAQ = 1 << 7,      // Air Quality Index
            CO = 1 << 8,        // carbon-monoxide
            NO2 = 1 << 9,       // nitrogen-dioxide
            O3 = 1 << 10,        // ozone gas
            SO2 = 1 << 11,       // sulfur-dioxide
            CO2 = 1 << 12,       // carbondioxide
            Mode = 1 << 13,     // device mode
            };

    // the structure of a measurement
    struct Measurement
        {
        //----------------
        // the subtypes:
        //----------------

        // compost temperature with SHT
        struct AirQuality
            {
            // compost temperature (in degrees C)
            float                   Iaq;
            // compost humidity (in percentage)
            float                   BreathVoc;
            };

        // compost temperature with SHT
        struct Env
            {
            // compost temperature (in degrees C)
            float                   TempC;
            // compost humidity (in percentage)
            float                   Humidity;
            };

        // measure co2ppm
        struct CO2ppm
            {
            float                   CO2ppm;
            };

        // measure particle with IPS-7100
        struct Particle
            {
            float                   Mass[7];
            std::uint32_t           Count[7];
            };

        // measures spec sensor data
        struct Gases
            {
            float                   uVCO;
            float                   uVNO2;
            float                   uVO3;
            float                   uVSO2;
            float                   CO;
            float                   NO2;
            float                   O3;
            float                   SO2;
            };

        // get gps co-ordinates and satellite time
        struct Position
            {
            float                   Latitude;
            float                   Longitude;
            uint32_t                UnixTime;
            };

        //---------------------------
        // the actual members as POD
        //---------------------------

        // flags of entries that are valid.
        Flags                   	flags;
        // flags of entries that are valid.
        Flags                       flags2;
        // measured battery voltage, in volts
        float                       Vbat;
        // measured system Vdd voltage, in volts
        float                       Vsystem;
        // boot count
        uint32_t                    BootCount;
        // compost temperature at bottom
        Env                         env;
        // measure co2ppm
        CO2ppm                      co2ppm;
        // measure particle
        Particle                    particle;
        // measure different gases
        Gases                       gases;
        // get position and time
        Position                    position;
        // get air quality
        AirQuality                  airQuality;
        };
    };

class cMeasurementLoop : public McciCatena::cPollableObject
    {
public:
    // some parameters
    static constexpr std::uint8_t kUplinkPort = 1;
    using MeasurementFormat = McciModel4916::cMeasurementFormat;
    using Measurement = MeasurementFormat::Measurement;
    using Flags = MeasurementFormat::Flags;
    static constexpr std::uint8_t kMessageFormat = MeasurementFormat::kMessageFormat;
    static constexpr std::uint8_t kSdCardCSpin = D11;

    // version parameters
    static constexpr std::uint8_t kMajor = 1;
    static constexpr std::uint8_t kMinor = 5;
    static constexpr std::uint8_t kPatch = 2;
    static constexpr std::uint8_t kLocal = 1;

    enum OPERATING_FLAGS : uint32_t
        {
        fUnattended = 1 << 0,
        fManufacturingTest = 1 << 1,
        fConfirmedUplink = 1 << 16,
        fDisableDeepSleep = 1 << 17,
        fQuickLightSleep = 1 << 18,
        fDeepSleepTest = 1 << 19,
        };

    enum DebugFlags : std::uint32_t
        {
        kError      = 1 << 0,
        kWarning    = 1 << 1,
        kTrace      = 1 << 2,
        kInfo       = 1 << 3,
        };

    // constructor
    cMeasurementLoop(
            McciCatenaSht3x::cSHT3x& sht3x,
            McciCatenaScd30::cSCD30& scd30,
            McciCatenaIps7100::cIPS7100& ips7100
            )
        : m_Sht(sht3x)
        , m_Scd(scd30)
        , m_Ips(ips7100)
        , m_txCycleSec_Permanent(30 * 60) // default uplink interval
        , m_txCycleSec(30 * 60)                    // initial uplink interval
        , m_txCycleCount(2)                   // initial count of fast uplinks
        , m_DebugFlags(DebugFlags(kError | kTrace))
        {};

    // neither copyable nor movable
    cMeasurementLoop(const cMeasurementLoop&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&) = delete;
    cMeasurementLoop(const cMeasurementLoop&&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&&) = delete;

    enum class State : std::uint8_t
        {
        stNoChange = 0, // this name must be present: indicates "no change of state"
        stInitial,      // this name must be present: it's the starting state.
        stInactive,     // parked; not doing anything.
        stSleeping,     // active; sleeping between measurements
        stWarmup,       // transition from inactive to measure, get some data.
        stMeasure,      // take measurents
        stTransmit,     // transmit data
        stWriteFile,    // write file data
        stTryToUpdate,  // try to update firmware
        stAwaitCard,    // wait for a card to show up.
        stRebootForUpdate, // reboot system to complete firmware update
        stFinal,        // this name must be present, it's the terminal state.
        };

    static constexpr const char *getStateName(State s)
        {
        switch (s)
            {
            case State::stNoChange: return "stNoChange";
            case State::stInitial:  return "stInitial";
            case State::stInactive: return "stInactive";
            case State::stSleeping: return "stSleeping";
            case State::stWarmup:   return "stWarmup";
            case State::stMeasure:  return "stMeasure";
            case State::stTransmit: return "stTransmit";
            case State::stWriteFile: return "stWriteFile";
            case State::stTryToUpdate: return "stTryToUpdate";
            case State::stAwaitCard: return "stAwaitCard";
            case State::stRebootForUpdate: return "stRebootForUpdate";
            case State::stFinal:    return "stFinal";
            default:                return "<<unknown>>";
            }
        }

    // concrete type for uplink data buffer
    using TxBuffer_t = McciCatena::AbstractTxBuffer_t<MeasurementFormat::kTxBufferSize>;
    using TxBufferBase_t = McciCatena::AbstractTxBufferBase_t;

    // initialize measurement FSM.
    void begin();
    void end();
    void printBME680info();
    void configBME680();
    void checkIaqSensorStatus();
    void dumpCalibrationData();
    void loadCalibrationData();
    void possiblySaveCalibrationData();
    void saveCalibrationData();
    void configGps();
    float getCOConcentration(float voltage);
    float getNO2Concentration(float voltage);
    float getO3Concentration(float voltage);
    float getSO2Concentration(float voltage);

    void setTxCycleTime(
        std::uint32_t txCycleSec,
        std::uint32_t txCycleCount
        )
        {
        this->m_txCycleSec = txCycleSec;
        this->m_txCycleCount = txCycleCount;

        this->m_UplinkTimer.setInterval(txCycleSec * 1000);
        if (this->m_UplinkTimer.peekTicks() != 0)
            this->m_fsm.eval();
        }
    std::uint32_t getTxCycleTime()
        {
        return this->m_txCycleSec;
        }
    virtual void poll() override;

    void setBme680(bool fEnable)
        {
        this->m_fBme680 = fEnable;
        }

    void printSCDinfo()
        {
        auto const info = m_Scd.getInfo();
        gCatena.SafePrintf(
                    "Found sensor: firmware version %u.%u\n",
                    info.FirmwareVersion / 256u,
                    info.FirmwareVersion & 0xFFu
                    );
        gCatena.SafePrintf("  Automatic Sensor Calibration: %u\n", info.fASC_status);
        gCatena.SafePrintf("  Sample interval:      %6u secs\n", info.MeasurementInterval);
        gCatena.SafePrintf("  Forced Recalibration: %6u ppm\n", info.ForcedRecalibrationValue);
        gCatena.SafePrintf("  Temperature Offset:   %6d centi-C\n", info.TemperatureOffset);
        gCatena.SafePrintf("  Altitude:             %6d meters\n", info.AltitudeCompensation);
        }

    uint32_t getDecimal(float data)
        {
        uint32_t data100 = std::uint32_t (data * 100 + 0.5);
        uint32_t dataInt = data100/100;
        uint32_t dataFrac = data100 - (dataInt * 100);

        return dataFrac;
        }

    uint32_t getGpsDecimal(float data)
        {
        uint32_t data100 = std::uint32_t (data * 10000000 + 0.5);
        uint32_t dataInt = data100/10000000;
        uint32_t dataFrac = data100 - (dataInt * 10000000);

        return dataFrac;
        }

    float getFloat(uint32_t data)
        {
        float dataFloat;
        dataFloat = (float)data / 100;
        return dataFloat;
        }

    // request that the measurement loop be active/inactive
    void requestActive(bool fEnable);

    // return true if a given debug mask is enabled.
    bool isTraceEnabled(DebugFlags mask) const
        {
        return this->m_DebugFlags & mask;
        }

    // register an additional SPI for sleep/resume
    // can be called before begin().
    void registerSecondSpi(SPIClass *pSpi)
        {
        this->m_pSPI2 = pSpi;
        }

    /// bring up the SD card, if possible.
    bool checkSdCard();
    /// tear down the SD card.
    void sdFinish();
private:
    // sleep handling
    void sleep();
    bool checkDeepSleep();
    void doSleepAlert(bool fDeepSleep);
    void doDeepSleep();
    void savePower(uint32_t sleepInterval);
    void deepSleepPrepare();
    void deepSleepRecovery();

    // read data
    void updateScd30Measurements();
    void updateSynchronousMeasurements();
    void resetMeasurements();

    // telemetry handling.
    void fillTxBuffer(TxBuffer_t &b, Measurement const & mData);
    void startTransmission(TxBuffer_t &b);
    void sendBufferDone(bool fSuccess);
    static std::uint16_t uflt16(float v)
        {
        return McciCatena::TxBuffer_t::f2uflt16(v);
        }

    bool txComplete()
        {
        return this->m_txcomplete;
        }
    void updateTxCycleTime();

    // SD card handling
    bool initSdCard();

    bool writeSdCard(TxBuffer_t &b, Measurement const &mData);
    bool handleSdFirmwareUpdate();
    bool handleSdFirmwareUpdateCardUp();
    bool updateFromSd(const char *sFile, McciCatena::cDownload::DownloadRq_t rq);
    void sdPowerUp(bool fOn);
    void sdPrep();

    // timeout handling

    // set the timer
    void setTimer(std::uint32_t ms);
    // clear the timer
    void clearTimer();
    // test (and clear) the timed-out flag.
    bool timedOut();

    // instance data
private:
    McciCatena::cFSM<cMeasurementLoop, State> m_fsm;
    // evaluate the control FSM.
    State fsmDispatch(State currentState, bool fEntry);

    // second SPI class
    SPIClass                        *m_pSPI2;
    // Serial number for data count
    uint32_t                        m_sno;

    // BME680 Environmental sensor
    Bsec                            m_bme;
    uint8_t                         m_bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    uint16_t                        m_stateUpdateCounter = 0;
    String                          m_output;
    // time stamp of last time we saved the state.
    uint32_t                        m_lastCalibrationWriteMillis = 0;

    // SHT3x Environmental sensor
    McciCatenaSht3x::cSHT3x&        m_Sht;

    // SCD30 - CO2 sensor
    McciCatenaScd30::cSCD30&        m_Scd;
    char                            ts;
    int32_t                         t100;
    int32_t                         tint;
    int32_t                         tfrac;
    int32_t                         rh100;
    int32_t                         rhint;
    int32_t                         rhfrac;
    int32_t                         co2_100;
    int32_t                         co2int;
    int32_t                         co2frac;

    // IPS7100 - Particle sensor
    McciCatenaIps7100::cIPS7100&    m_Ips;

    // ADS131M04 - ADC for different spec sensor
    McciCatenaAds131m04::cADS131M04 m_Ads;

    float m_mVgasZeroCO = 602;
    float m_mVgasZeroO3 = 587;
    float m_mVgasZeroSO2 = 700;
    float m_mVgasZeroNO2 = 500;

    float m_IsenseCO;
    float m_IsenseO3;
    float m_IsenseSO2;
    float m_IsenseNO2;

    float m_RgainCO = 124;
    float m_RgainO3 = 249;
    float m_RgainSO2 = 100;
    float m_RgainNO2 = 150;

    // SAM-M8Q - GPS for position and time
    cSAM_M8Q                         m_Gps;
    float                            m_latitude;
    float                            m_longitude;

    // debug flags
    DebugFlags                      m_DebugFlags;

    // true if object is registered for polling.
    bool                            m_registered : 1;
    // true if object is running.
    bool                            m_running : 1;
    // true to request exit
    bool                            m_exit : 1;
    // true if in active uplink mode, false otehrwise.
    bool                            m_active : 1;

    // set true to request transition to active uplink mode; cleared by FSM
    bool                            m_rqActive : 1;
    // set true to request transition to inactive uplink mode; cleared by FSM
    bool                            m_rqInactive : 1;
    // set true if measurement is valid
    bool                            m_measurement_valid: 1;
    // set true if transmit is done
    bool                            m_fTransmit: 1;

    // set true if event timer times out
    bool                            m_fTimerEvent : 1;
    // set true while evenet timer is active.
    bool                            m_fTimerActive : 1;
    // set true if USB power is present.
    bool                            m_fUsbPower : 1;

    // set true while a transmit is pending.
    bool                            m_txpending : 1;
    // set true when a transmit completes.
    bool                            m_txcomplete : 1;
    // set true when a transmit complete with an error.
    bool                            m_txerr : 1;
    // set true when we've printed how we plan to sleep
    bool                            m_fPrintedSleeping : 1;
    // set true when SPI2 is active
    bool                            m_fSpi2Active: 1;
    // set true when we've BIN file in SD card to update
    bool                            m_fFwUpdate : 1;

    // set true if BME680 is present.
    bool                            m_fBme680 : 1;
    // set true if BME680 data is valid.
    bool                            m_fBme680Valid : 1;
    // set true if SHT3x is present
    bool                            m_fSht3x : 1;
    // set true if CO2 (SCD) is present
    bool                            m_fScd30 : 1;
    // set true if IPS-7100 is present
    bool                            m_fIps7100 : 1;
    // set true if ADS131M04 is present
    bool                            m_fAds131m04 : 1;
    // set true if SAM-M8q is present
    bool                            m_GpsSamM8q : 1;

    // uplink time control
    McciCatena::cTimer              m_UplinkTimer;
    std::uint32_t                   m_txCycleSec;
    std::uint32_t                   m_txCycleCount;
    std::uint32_t                   m_txCycleSec_Permanent;

    // simple timer for timing-out sensors.
    std::uint32_t                   m_timer_start;
    std::uint32_t                   m_timer_delay;

    // the current measurement
    Measurement                     m_data;

    // the data to write to the file
    Measurement                     m_FileData;
    TxBuffer_t                      m_FileTxBuffer;
    };

//
// operator overloads for ORing structured flags
//
static constexpr cMeasurementLoop::Flags operator| (const cMeasurementLoop::Flags lhs, const cMeasurementLoop::Flags rhs)
        {
        return cMeasurementLoop::Flags(uint16_t(lhs) | uint16_t(rhs));
        };

static constexpr cMeasurementLoop::Flags operator& (const cMeasurementLoop::Flags lhs, const cMeasurementLoop::Flags rhs)
        {
        return cMeasurementLoop::Flags(uint16_t(lhs) & uint16_t(rhs));
        };

static cMeasurementLoop::Flags operator|= (cMeasurementLoop::Flags &lhs, const cMeasurementLoop::Flags &rhs)
        {
        lhs = lhs | rhs;
        return lhs;
        };

} // namespace McciModel4916

#endif /* _Model4916_cMeasurementLoop_h_ */
