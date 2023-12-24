/*

Module: Model4916_cMeasurementLoop.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include "Model4916_cMeasurementLoop.h"

#include <arduino_lmic.h>
#include <Model4916-MultiGas-Sensor.h>
#include <stdint.h>
#include <Catena_Fram8k.h>

using namespace McciModel4916;
using namespace McciCatena;

extern cMeasurementLoop gMeasurementLoop;

McciCatena::cFram8k gFram;

auto const pFram = gCatena.getFram();

float voc;
float aqi;
uint8_t totalSamples = 32;

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

void cMeasurementLoop::begin()
    {
    // register for polling.
    if (! this->m_registered)
        {
        this->m_registered = true;

        gCatena.registerObject(this);

        this->m_UplinkTimer.begin(this->m_txCycleSec * 1000);
        }

    this->m_sno = 0;
    if (this->m_Sht.begin())
        {
        this->m_fSht3x = true;
        gCatena.SafePrintf("SHT3x found: Env sensor\n");
        }
    else
        {
        this->m_fSht3x = false;
        gCatena.SafePrintf("No SHT3x found: check wiring\n");
        }

    // powerup GPS
    gpiopower.configVgps(true);
    gpioenable.configGps(true);
    gpiopower.setVgps(true);
    delay(5);
    gpioenable.enableGps(true);
    delay(5);

    if (this->m_Gps.begin())
        {
        this->m_GpsSamM8q = true;
        gCatena.SafePrintf("SAM-M8Q found: GPS location and time\n");
        this->configGps();
        this->m_Gps.powerSaveMode(true);
        this->savePower(3);
        }
    else
        {
        this->m_GpsSamM8q = false;
        gCatena.SafePrintf("No SAM-M8Q GPS found: check wiring\n");
        }

    if (this->m_GpsSamM8q)
        {
        uint32_t startTime = millis();
        while (this->m_Gps.getLatitudeDegree() == 0.00f && this->m_Gps.getLongitudeDegree() == 0.00f)
            {
            // this->savePower(3);
            if ((millis() - startTime) > (1*60*1000))
                break;
            }
        // this->m_Gps.softwareResetGNSSOnly();
        }

    // start (or restart) the FSM.
    if (! this->m_running)
        {
        this->m_fTransmit = true;
        this->m_exit = false;
        this->m_fsm.init(*this, &cMeasurementLoop::fsmDispatch);
        }
    }

void cMeasurementLoop::end()
    {
    if (this->m_running)
        {
        this->m_exit = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::requestActive(bool fEnable)
    {
    if (fEnable)
        this->m_rqActive = true;
    else
        this->m_rqInactive = true;

    this->m_fsm.eval();
    }

//
// call this after waking up from a long (> 15 minute) sleep to correct for LMIC sleep defect
// This should be done after updating micros() and updating LMIC's idea of time based on
// the sleep time.
//
void fixLmicTimeCalculationAfterWakeup(void)
    {
    ostime_t const now = os_getTime();
    // just tell the LMIC that we're available *now*.
    LMIC.globalDutyAvail = now;
    // no need to randomize
    // for EU-like, we need to reset all the channel avail times to "now"
#if CFG_LMIC_EU_like
    for (unsigned i = 0; i < MAX_BANDS; ++i)
        {
        LMIC.bands[i].avail = now;
        }
#endif
    }

void cMeasurementLoop::printBME680info()
    {
    gCatena.SafePrintf("\nBSEC library version %d.%d.%d.%d\n",
            this->m_bme.version.major,
            this->m_bme.version.minor,
            this->m_bme.version.major_bugfix,
            this->m_bme.version.minor_bugfix
            );
    }

void cMeasurementLoop::configBME680()
    {
    this->m_bme.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();

    loadCalibrationData();
    bsec_virtual_sensor_t sensorList[2] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        };

    this->m_bme.updateSubscription(sensorList, sizeof(sensorList), BSEC_SAMPLE_RATE_ULP);
    checkIaqSensorStatus();
    }

// Helper function definitions
void cMeasurementLoop::checkIaqSensorStatus(void)
    {
    if (this->m_bme.status != BSEC_OK)
        {
        if (this->m_bme.status < BSEC_OK)
            {
            this->m_output = "BSEC error code : " + String(this->m_bme.status);
            // Serial.println(this->m_output);
            }
        else
            {
            this->m_output = "BSEC warning code : " + String(this->m_bme.status);
            // Serial.println(this->m_output);
            }
        }

    if (this->m_bme.bme680Status != BME680_OK)
        {
        if (this->m_bme.bme680Status < BME680_OK)
            {
            this->m_output = "BME680 error code : " + String(this->m_bme.bme680Status);
            // Serial.println(this->m_output);
            }
        else
            {
            this->m_output = "BME680 warning code : " + String(this->m_bme.bme680Status);
            // Serial.println(this->m_output);
            }
        }
    }

void cMeasurementLoop::dumpCalibrationData(void)
    {
    for (auto here = 0, nLeft = BSEC_MAX_STATE_BLOB_SIZE;
        nLeft != 0;)
        {
        char line[160];
        size_t n;

        n = 16;
        if (n >= nLeft)
            n = nLeft;

        McciAdkLib_FormatDumpLine(
            line, sizeof(line), 0,
            here,
            this->m_bsecState + here, n);

        gCatena.SafePrintf("%s\n", line);

        here += n, nLeft -= n;
        }
    }

void cMeasurementLoop::loadCalibrationData(void)
    {
    cFram * const pFram = gCatena.getFram();
    bool fDataOk;

    if (pFram->getField(cFramStorage::kBme680Cal, this->m_bsecState))
        {
        gCatena.SafePrintf("Got state from FRAM:\n");
        dumpCalibrationData();

        this->m_bme.setState(this->m_bsecState);
        if (this->m_bme.status != BSEC_OK)
            {
            gCatena.SafePrintf(
                "%s: BSEC error code: %d\n", __func__, this->m_bme.status
                );
            fDataOk = false;
            }
        else
            {
            fDataOk = true;
            }
        }
    else
        {
        fDataOk = false;
        }

    if (! fDataOk)
        {
        // initialize the saved state
        gCatena.SafePrintf("Initializing state\n");
        saveCalibrationData();
        }
    }

void cMeasurementLoop::possiblySaveCalibrationData(void)
    {
    bool update = false;
    if (this->m_stateUpdateCounter == 0)
        {
        /* First state update when IAQ accuracy is >= 1 (calibration complete) */
        if (this->m_bme.iaqAccuracy >= 1)
            {
            update = true;
            this->m_stateUpdateCounter++;
            }
        }
    else
        {
        /* Update every STATE_SAVE_PERIOD milliseconds */
        if (this->m_bme.iaqAccuracy >= 1 &&
            (uint32_t)(millis() - this->m_lastCalibrationWriteMillis) >= STATE_SAVE_PERIOD)
                {
                update = true;
                this->m_stateUpdateCounter++;
                }
        }

    if (update)
        {
        saveCalibrationData();
        }
    }

void cMeasurementLoop::saveCalibrationData(void)
    {
    this->m_lastCalibrationWriteMillis = millis();
    this->m_bme.getState(this->m_bsecState);
    checkIaqSensorStatus();

    gCatena.SafePrintf("Writing state to FRAM\n");
    gCatena.getFram()->saveField(cFramStorage::kBme680Cal, this->m_bsecState);
    dumpCalibrationData();
    }

void cMeasurementLoop::configGps()
    {
    uint8_t ret;

    if (! this->m_Gps.setmaxStartupStateDur(25))
      gCatena.SafePrintf("configGps: Set Max StartUp Duration failed\n");

    if (! this->m_Gps.setminAcqTime(5000))
      gCatena.SafePrintf("configGps: Set Min Acquisition Time failed\n");

    if (! this->m_Gps.setUpdatePeriod(5000))
      gCatena.SafePrintf("configGps: Set Min Acquisition Time failed\n");

    if (! this->m_Gps.setSearchPeriod(15000))
      gCatena.SafePrintf("configGps: Set Min Acquisition Time failed\n");

    gCatena.SafePrintf("configGps: GPS Max Startup time : %u seconds\n", this->m_Gps.getmaxStartupStateDur());
    gCatena.SafePrintf("configGps: GPS Min Acq time : %u seconds\n", this->m_Gps.getminAcqTime()/1000);
    gCatena.SafePrintf("configGps: GPS Update Period: %u ms\n", this->m_Gps.getUpdatePeriod());
    gCatena.SafePrintf("configGps: GPS Search Period: %u ms\n", this->m_Gps.getSearchPeriod());

    // Set the I2C port to output
    this->m_Gps.setI2COutput(COM_TYPE_UBX);
    delay (100);

    // Save Configuration
    this->m_Gps.saveConfiguration();
    }

float cMeasurementLoop::getCOConcentration(float uVoltage)
    {
    float gasConcentration;
    pFram->getField(
        cFramStorage::StandardKeys::kCOSensitivity,
        m_IsenseCO
        );

    float mVsenseCO = m_IsenseCO * m_RgainCO / 1000.00f;
    gasConcentration = ((uVoltage / 1000.00f) - m_mVgasZeroCO) / mVsenseCO;

    return gasConcentration;
    }

float cMeasurementLoop::getO3Concentration(float uVoltage)
    {
    float gasConcentration;
    pFram->getField(
        cFramStorage::StandardKeys::kO3Sensitivity,
        m_IsenseO3
        );

    m_IsenseO3 = m_IsenseO3 * (-1);
    float mVsenseO3 = m_IsenseO3 * m_RgainO3 / 1000.00f;
    gasConcentration = ((uVoltage / 1000.00f) - m_mVgasZeroO3) / mVsenseO3;

    return gasConcentration;
    }

float cMeasurementLoop::getSO2Concentration(float uVoltage)
    {
    float gasConcentration;
    pFram->getField(
        cFramStorage::StandardKeys::kSO2Sensitivity,
        m_IsenseSO2
        );

    float mVsenseSO2 = m_IsenseSO2 * m_RgainSO2 / 1000.00f;
    gasConcentration = ((uVoltage / 1000.00f) - m_mVgasZeroSO2) / mVsenseSO2;

    return gasConcentration;
    }

float cMeasurementLoop::getNO2Concentration(float uVoltage)
    {
    float gasConcentration;
    pFram->getField(
        cFramStorage::StandardKeys::kNO2Sensitivity,
        m_IsenseNO2
        );

    m_IsenseNO2 = m_IsenseNO2 * (-1);
    float mVsenseNO2 = m_IsenseNO2 * m_RgainNO2 / 1000.00f;
    gasConcentration = ((uVoltage / 1000.00f) - m_mVgasZeroNO2) / mVsenseNO2;

    return gasConcentration;
    }

cMeasurementLoop::State
cMeasurementLoop::fsmDispatch(
    cMeasurementLoop::State currentState,
    bool fEntry
    )
    {
    State newState = State::stNoChange;

    if (fEntry && this->isTraceEnabled(this->DebugFlags::kTrace))
        {
        gCatena.SafePrintf("cMeasurementLoop::fsmDispatch: enter %s\n",
                this->getStateName(currentState)
                );
        }

    switch (currentState)
        {
    case State::stInitial:
        newState = State::stInactive;
        this->resetMeasurements();
        break;

    case State::stInactive:
        if (fEntry)
            {
            // turn off anything that should be off while idling.
            }
        if (this->m_rqActive)
            {
            // when going active manually, start the measurement
            // cycle immediately.
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = true;
            this->m_UplinkTimer.retrigger();
            newState = State::stWarmup;
            }
        break;

    case State::stSleeping:
        if (fEntry)
            {
            // set the LEDs to flash accordingly.
            gLed.Set(McciCatena::LedPattern::Sleeping);
            }

        if (this->m_rqInactive)
            {
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = false;
            newState = State::stInactive;
            }
        else if (this->m_UplinkTimer.isready())
            newState = State::stMeasure;
        else if (this->m_UplinkTimer.getRemaining() > 1500)
            {
            this->sleep();
            }
        break;

      // get some data. This is only called while booting up.
	     case State::stWarmup:
	         if (fEntry)
	             {
	             //start the timer
	             gCatena.SafePrintf("15 seconds time window for configurations\n");
	             this->setTimer(15 * 1000);
	             gCatena.poll();
	             yield();
	             }
	         if (this->timedOut())
	             newState = State::stMeasure;
	         break;

    // fill in the measurement
    case State::stMeasure:
			if (fEntry)
            {
            this->updateSynchronousMeasurements();
            }

            if (this->m_fTransmit || (this->m_UplinkTimer.getRemaining() < (15 * 1000)))
                {
                fixLmicTimeCalculationAfterWakeup();
                newState = State::stTransmit;
                }
            else
                newState = State::stWriteFile;
        break;

    case State::stTransmit:
        if (fEntry)
            {
            TxBuffer_t b;

            this->fillTxBuffer(b, this->m_data);
            this->m_FileData = this->m_data;

            this->m_FileTxBuffer.begin();
            for (auto i = 0; i < b.getn(); ++i)
                this->m_FileTxBuffer.put(b.getbase()[i]);

            if (gLoRaWAN.IsProvisioned())
                this->startTransmission(b);

            while (true)
                {
                std::uint32_t lmicCheckTime;
                os_runloop_once();
                lmicCheckTime = this->m_UplinkTimer.getRemaining();

                // if we can sleep, break out of this loop
                // NOTE: if that the TX is not ready, LMIC is still waiting for interrupt
                if (! os_queryTimeCriticalJobs(ms2osticks(lmicCheckTime)) && LMIC_queryTxReady())
                    {
                    break;
                    }

                gCatena.poll();
                yield();
                }

            this->m_fTransmit = true;
            this->m_UplinkTimer.retrigger();
            }
        if (! gLoRaWAN.IsProvisioned())
            {
            newState = State::stWriteFile;
            }
        if (this->txComplete())
            {
            newState = State::stWriteFile;

            // calculate the new sleep interval.
            this->updateTxCycleTime();
            }
        break;

    // if there's an SD card, append to file
    case State::stWriteFile:
        if (fEntry)
            {
            if (! this->m_fTransmit)
                {
                TxBuffer_t b;

                this->fillTxBuffer(b, this->m_data);
                this->m_FileData = this->m_data;

                this->m_FileTxBuffer.begin();
                for (auto i = 0; i < b.getn(); ++i)
                    this->m_FileTxBuffer.put(b.getbase()[i]);

                this->resetMeasurements();
                this->m_fTransmit = false;
                }
            else
                {
                this->m_fTransmit = false;
                }
            }

        if (this->writeSdCard(this->m_FileTxBuffer, this->m_FileData))
            newState = State::stTryToUpdate;
        else if (gLoRaWAN.IsProvisioned())
            newState = State::stTryToUpdate;
        else
            newState = State::stAwaitCard;

        this->savePower(3);
        break;

    // try to update firmware
    case State::stTryToUpdate:
        if (this->handleSdFirmwareUpdate())
            newState = State::stRebootForUpdate;
        else {
            // gpiopower.setVSpi(false);
            // delay(5);
            newState = State::stMeasure;
        }
        this->savePower(18);
        this->m_fFwUpdate = false;
        break;

    // no SD card....
    case State::stAwaitCard:
        if (fEntry)
            {
            gCatena.SafePrintf("** no SD card and not provisioned!\n");
            }

        // gpiopower.setVSpi(false);
        // delay(50);

        newState = State::stMeasure;
        break;

    // reboot for update
    case State::stRebootForUpdate:
        if (fEntry)
            {
            gLog.printf(gLog.kInfo, "Rebooting to apply firmware\n");
            this->setTimer(1 * 1000);
            }
        if (this->timedOut())
            {
            NVIC_SystemReset();
            }
        break;

    case State::stFinal:
        break;

    default:
        break;
        }

    return newState;
    }

/****************************************************************************\
|
|   Take a measurement
|
\****************************************************************************/

void cMeasurementLoop::resetMeasurements()
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = Flags(0);
    this->m_data.flags2 = Flags(0);
    }

void cMeasurementLoop::updateScd30Measurements()
    {
    if (this->m_fScd30)
        {
        bool fError;
        if (this->m_Scd.queryReady(fError))
            {
            this->m_measurement_valid = this->m_Scd.readMeasurement();
            if ((! this->m_measurement_valid) && gLog.isEnabled(gLog.kError))
                {
                gLog.printf(gLog.kError, "SCD30 measurement failed: error %s(%u)\n",
                        this->m_Scd.getLastErrorName(),
                        unsigned(this->m_Scd.getLastError())
                        );
                }
            }
        else if (fError)
            {
            if (gLog.isEnabled(gLog.DebugFlags::kError))
                gLog.printf(
                    gLog.kAlways,
                    "SCD30 queryReady failed: status %s(%u)\n",
                    this->m_Scd.getLastErrorName(),
                    unsigned(this->m_Scd.getLastError())
                    );
            }
        }

    if (this->m_fScd30 && this->m_measurement_valid)
        {
        auto const m = this->m_Scd.getMeasurement();
        // temperature is 2 bytes from -163.840 to +163.835 degrees C
        // pressure is 4 bytes, first signed units, then scale.
        if (gLog.isEnabled(gLog.kInfo))
            {
            this->ts = ' ';
            this->t100 = std::int32_t(m.Temperature * 100.0f + 0.5f);
            if (m.Temperature < 0) {
                this->ts = '-';
                this->t100 = -this->t100;
                }
            this->tint = this->t100 / 100;
            this->tfrac = this->t100 - (tint * 100);

            this->rh100 = std::int32_t(m.RelativeHumidity * 100.0f + 0.5f);
            this->rhint = this->rh100 / 100;
            this->rhfrac = this->rh100 - (this->rhint * 100);

            this->co2_100 = std::int32_t(m.CO2ppm * 100.0f + 0.5f);
            this->co2int = this->co2_100 / 100;
            this->co2frac = this->co2_100 - (this->co2int * 100);
            }

        this->m_data.co2ppm.CO2ppm = m.CO2ppm;
        }
    }

void cMeasurementLoop::updateSynchronousMeasurements()
    {
    this->m_sno = this->m_sno + 1;

    this->m_data.Vbat = gCatena.ReadVbat();
    this->m_data.flags |= Flags::Vbat;

    if (gCatena.getBootCount(this->m_data.BootCount))
        {
        this->m_data.flags |= Flags::Boot;
        }

    if (this->m_fSht3x)
        {
        cSHT3x::Measurements m;
        this->m_Sht.getTemperatureHumidity(m);
        this->m_data.env.TempC = m.Temperature;
        this->m_data.env.Humidity = m.Humidity;
        this->m_data.flags |= Flags::TH;
        }

    if (this->m_GpsSamM8q)
        {
        this->m_latitude = this->m_Gps.getLatitudeDegree();
        this->m_longitude = this->m_Gps.getLongitudeDegree();
        if ((this->m_longitude >= -180.0f && this->m_longitude <= 180.0f) &&
            (this->m_latitude >= -90.0f && this->m_latitude <= 90.0f))
            {
            this->m_data.position.Latitude = this->m_latitude;
            this->m_data.position.Longitude = this->m_longitude;
            this->m_data.flags |= Flags::GPS;
            }
        this->savePower(3);
        if (this->m_Gps.getDateValid() != false && this->m_Gps.getTimeValid() != false)
            {
            this->m_data.position.UnixTime = this->m_Gps.getUnixEpoch();
            this->m_data.flags |= Flags::Time;
            }
        }
    this->savePower(3);

    // power-up and enable FRAM
    gpiopower.configVtvoc(true);
    gpioenable.configTvoc(true);
    gpiopower.setVtvoc(true);
    delay(20);
    gpioenable.enableTvoc(true);
    delay(2);

    this->m_bme.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    this->m_fBme680 = true;
    this->m_fBme680Valid = false;
    this->printBME680info();
    this->configBME680();
    checkIaqSensorStatus();
    this->savePower(1);

    if (this->m_fBme680)
        {
        if (this->m_bme.run())
            { // If new data is available
            this->m_fBme680Valid = true;

            aqi = this->m_bme.iaq;
            voc = this->m_bme.breathVocEquivalent;

            possiblySaveCalibrationData();
            }
        else
            {
            checkIaqSensorStatus();
            }
        }

    // power-up and enable FRAM
    gpiopower.configVqwiic(true);
    gpioenable.configQwiic(true);
    gpiopower.setVqwiic(true);
    delay(20);
    gpioenable.enableQwiic(true);
    delay(20);

    if (! m_Scd.begin())
        {
        this->m_fScd30 = false;

        gCatena.SafePrintf("No SCD30 found! Begin failed: %s(%u)\n",
                m_Scd.getLastErrorName(),
                unsigned(m_Scd.getLastError())
                );
        }
    else
        {
        gCatena.SafePrintf("SCD30 found! CO2 sensor\n");
        this->m_fScd30 = true;
        this->printSCDinfo();
        delay(5000);
        }

    if (this->m_fScd30) {
        updateScd30Measurements();
        // this->m_Scd.end();
        }
    if (this->m_data.co2ppm.CO2ppm != 0.0f)
        {
        this->m_data.flags2 |= Flags::CO2;
        }
    gpioenable.enableQwiic(false);
    gpiopower.setVqwiic(false);
    delay(2);
    this->savePower(5);

    if (this->m_fBme680)
        {
        if (this->m_bme.run())
            { // If new data is available
            this->m_fBme680Valid = true;

            aqi = this->m_bme.iaq;
            voc = this->m_bme.breathVocEquivalent;

            possiblySaveCalibrationData();
            }
        else
            {
            checkIaqSensorStatus();
            }
        }

    // power-up and enable FRAM
    pinMode(D12, OUTPUT);
    digitalWrite(D12, LOW);
    delay(100);

    gpiopower.configVPiera(true);
    gpioenable.configPiera(true);
    gpiopower.setVPiera(true);
    delay(2);
    gpioenable.enablePiera(true);
    delay(2);

    if (this->m_Ips.begin())
        {
        this->m_fIps7100 = true;
        gCatena.SafePrintf("IPS-7100 found: Particle sensor\n");
        // this->m_Ips.enablePowerSavingMode(true);
        }
    else
        {
        this->m_fIps7100 = false;
        gCatena.SafePrintf("No IPS-7100 found: check wiring\n");
        }

    if (this->m_fIps7100)
        {
        this->m_Ips.startMeasurement(1);
        delay(2500);
        }

    if (this->m_fIps7100)
        {
        this->m_Ips.updateData();

        this->m_data.particle.Count[0] = this->m_Ips.getPC01Data();
        this->m_data.particle.Count[1] = this->m_Ips.getPC03Data();
        this->m_data.particle.Count[2] = this->m_Ips.getPC05Data();
        this->m_data.particle.Count[3] = this->m_Ips.getPC10Data();
        this->m_data.particle.Count[4] = this->m_Ips.getPC25Data();
        this->m_data.particle.Count[5] = this->m_Ips.getPC50Data();
        this->m_data.particle.Count[6] = this->m_Ips.getPC100Data();

        this->m_data.particle.Mass[0] = this->m_Ips.getPM01Data();
        this->m_data.particle.Mass[1] = this->m_Ips.getPM03Data();
        this->m_data.particle.Mass[2] = this->m_Ips.getPM05Data();
        this->m_data.particle.Mass[3] = this->m_Ips.getPM10Data();
        this->m_data.particle.Mass[4] = this->m_Ips.getPM25Data();
        this->m_data.particle.Mass[5] = this->m_Ips.getPM50Data();
        this->m_data.particle.Mass[6] = this->m_Ips.getPM100Data();

        // this->m_Ips.enablePowerSavingMode(true);

        this->m_data.flags |= Flags::PM;
        }

    gpioenable.enablePiera(false);
    gpiopower.setVPiera(false);
    delay(2);
    pinMode(D12, OUTPUT);
    digitalWrite(D12, HIGH);
    this->savePower(5);

    if (this->m_fBme680)
        {
        if (this->m_bme.run())
            { // If new data is available
            this->m_fBme680Valid = true;

            aqi = this->m_bme.iaq;
            voc = this->m_bme.breathVocEquivalent;

            possiblySaveCalibrationData();
            }
        else
            {
            checkIaqSensorStatus();
            }
        }

    // power-up ADS131M04
    gpiopower.configVSpi(true);
    gpiopower.setVSpi(true);
    delay(20);

    if (this->m_Ads.begin(&gSPI2, D16, D17, D18))
        {
        this->m_fAds131m04 = true;
        this->m_Ads.globalChop(true);
        gCatena.SafePrintf("ADS131M04 found: Analog-to-digital (SPEC sensors)\n");
        }
    else
        {
        this->m_fAds131m04 = false;
        gCatena.SafePrintf("No ADS131M04 found: check wiring\n");
        }

    if (this->m_fAds131m04)
        {
        std::uint8_t channel0 = 0;
        std::uint8_t channel1 = 1;
        std::uint8_t channel2 = 2;
        std::uint8_t channel3 = 3;

        this->m_data.gases.uVCO = 0;
        this->m_data.gases.uVO3 = 0;
        this->m_data.gases.uVSO2 = 0;
        this->m_data.gases.uVNO2 = 0;

        uint32_t startTime = millis();
        while (! this->m_Ads.isDataReady())
            {
            if ((millis() - startTime) > 1000)
                break;
            }

        this->m_Ads.readVoltage(channel0);
        this->m_Ads.readVoltage(channel1);
        this->m_Ads.readVoltage(channel2);
        this->m_Ads.readVoltage(channel3);

        /* Read CO voltage */
        for (uint8_t nSample = 0; nSample < totalSamples; nSample++)
            this->m_data.gases.uVCO = this->m_data.gases.uVCO + this->m_Ads.readVoltage(channel0);

        this->m_data.gases.uVCO = this->m_data.gases.uVCO / totalSamples;
        this->m_data.gases.uVCO = this->m_data.gases.uVCO * 1000.00f * 1000.00f;
        auto concentration = getCOConcentration(this->m_data.gases.uVCO);
        this->m_data.gases.CO = concentration;
        this->m_data.flags2 |= Flags::CO;

        /* Read O3 voltage */
        for (uint8_t nSample = 0; nSample < totalSamples; nSample++)
            this->m_data.gases.uVO3 = this->m_data.gases.uVO3 + this->m_Ads.readVoltage(channel1);

        this->m_data.gases.uVO3 = this->m_data.gases.uVO3 / totalSamples;
        this->m_data.gases.uVO3 = this->m_data.gases.uVO3 * 1000.00f * 1000.00f;
        concentration = getO3Concentration(this->m_data.gases.uVO3);
        this->m_data.gases.O3 = concentration;
        this->m_data.flags2 |= Flags::O3;

        /* Read SO2 voltage */
        for (uint8_t nSample = 0; nSample < totalSamples; nSample++)
            this->m_data.gases.uVSO2 = this->m_data.gases.uVSO2 + this->m_Ads.readVoltage(channel2);

        this->m_data.gases.uVSO2 = this->m_data.gases.uVSO2 / totalSamples;
        this->m_data.gases.uVSO2 = this->m_data.gases.uVSO2 * 1000.00f * 1000.00f;
        concentration = getSO2Concentration(this->m_data.gases.uVSO2);
        this->m_data.gases.SO2 = concentration;
        this->m_data.flags2 |= Flags::SO2;

        /* Read NO2 voltage */
        for (uint8_t nSample = 0; nSample < totalSamples; nSample++)
            this->m_data.gases.uVNO2 = this->m_data.gases.uVNO2 + this->m_Ads.readVoltage(channel3);

        this->m_data.gases.uVNO2 = this->m_data.gases.uVNO2 / totalSamples;
        this->m_data.gases.uVNO2 = this->m_data.gases.uVNO2 * 1000.00f * 1000.00f;
        concentration = getNO2Concentration(this->m_data.gases.uVNO2);
        this->m_data.gases.NO2 = concentration;
        this->m_data.flags2 |= Flags::NO2;
        }

    gpiopower.configVSpi(false);
    gpiopower.setVSpi(false);
    delay(20);
    this->savePower(3);

    if (this->m_fBme680)
        {
        if (this->m_bme.run())
            { // If new data is available
            this->m_fBme680Valid = true;

            aqi = this->m_bme.iaq;
            voc = this->m_bme.breathVocEquivalent;

            possiblySaveCalibrationData();
            }
        else
            {
            checkIaqSensorStatus();
            }
        }
    if (this->m_fBme680Valid)
        {
        this->m_data.airQuality.BreathVoc = voc;
        this->m_data.airQuality.Iaq = aqi;
        this->m_data.flags |= Flags::IAQ;
        this->m_data.flags |= Flags::TVOC;
        }
    gpioenable.enableTvoc(false);
    gpiopower.setVtvoc(false);
    delay(2);

    this->m_data.flags2 |= Flags::Mode;
    }

/****************************************************************************\
|
|   Start uplink of data
|
\****************************************************************************/

void cMeasurementLoop::startTransmission(
    cMeasurementLoop::TxBuffer_t &b
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    gLed.Set(McciCatena::LedPattern::Sending);

    // by using a lambda, we can access the private contents
    auto sendBufferDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    if (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
        {
        gCatena.SafePrintf("requesting confirmed tx\n");
        fConfirmed = true;
        }

    this->m_txpending = true;
    this->m_txcomplete = this->m_txerr = false;

    if (! gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, (void *)this, fConfirmed, kUplinkPort))
        {
        // uplink wasn't launched.
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::sendBufferDone(bool fSuccess)
    {
    this->m_txpending = false;
    this->m_txcomplete = true;
    this->m_txerr = ! fSuccess;
    this->m_fsm.eval();
    }

/****************************************************************************\
|
|   The Polling function --
|
\****************************************************************************/

void cMeasurementLoop::poll()
    {
    bool fEvent;

    // no need to evaluate unless something happens.
    fEvent = false;

    // if we're not active, and no request, nothing to do.
    if (! this->m_active)
        {
        if (! this->m_rqActive)
            return;

        // we're asked to go active. We'll want to eval.
        fEvent = true;
        }

	   if (this->m_fTimerActive)
	        {
	        if ((millis() - this->m_timer_start) >= this->m_timer_delay)
	            {
	            this->m_fTimerActive = false;
	            this->m_fTimerEvent = true;
	            fEvent = true;
	            }
        }

    // check the transmit time.
    if (this->m_UplinkTimer.peekTicks() != 0)
        {
        fEvent = true;
        }

    if (fEvent)
        this->m_fsm.eval();
    }

/****************************************************************************\
|
|   Update the TxCycle count.
|
\****************************************************************************/

void cMeasurementLoop::updateTxCycleTime()
    {
    auto txCycleCount = this->m_txCycleCount;

    // update the sleep parameters
    if (txCycleCount > 1)
            {
            // values greater than one are decremented and ultimately reset to default.
            this->m_txCycleCount = txCycleCount - 1;
            }
    else if (txCycleCount == 1)
            {
            // it's now one (otherwise we couldn't be here.)
            gCatena.SafePrintf("resetting tx cycle to default: %u\n", this->m_txCycleSec_Permanent);

            this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
            }
    else
            {
            // it's zero. Leave it alone.
            }
    }

/****************************************************************************\
|
|   Handle sleep between measurements
|
\****************************************************************************/

void cMeasurementLoop::sleep()
    {
    const bool fDeepSleep = checkDeepSleep();

    if (! this->m_fPrintedSleeping)
            this->doSleepAlert(fDeepSleep);

    if (fDeepSleep)
            this->doDeepSleep();
    }

// for now, we simply don't allow deep sleep. In the future might want to
// use interrupts on activity to wake us up; then go back to sleep when we've
// seen nothing for a while.
bool cMeasurementLoop::checkDeepSleep()
    {
    bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    bool fDeepSleep;
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval < 2)
        fDeepSleep = false;
    else if (fDeepSleepTest)
        {
        fDeepSleep = true;
        }
    else if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
        {
        fDeepSleep = false;
        }
    else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
        {
        fDeepSleep = true;
        }
    else
        {
        fDeepSleep = false;
        }

    return fDeepSleep;
    }

void cMeasurementLoop::doSleepAlert(bool fDeepSleep)
    {
    this->m_fPrintedSleeping = true;

    if (fDeepSleep)
        {
        bool const fDeepSleepTest =
                gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

        gCatena.SafePrintf("using deep sleep in %u secs: ",
                            deepSleepDelay
                            );

        // sleep and print
        gLed.Set(McciCatena::LedPattern::TwoShort);

        for (auto n = deepSleepDelay; n > 0; --n)
            {
            uint32_t tNow = millis();

            while (uint32_t(millis() - tNow) < 1000)
                {
                gCatena.poll();
                yield();
                }
            gCatena.SafePrintf(".");
            }
        gCatena.SafePrintf("\nStarting deep sleep.\n");
        uint32_t tNow = millis();
        while (uint32_t(millis() - tNow) < 100)
            {
            gCatena.poll();
            yield();
            }
        }
    else
        gCatena.SafePrintf("using light sleep\n");
    }

void cMeasurementLoop::doDeepSleep()
    {
    // bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
    //                         static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();

    /* and now... we're awake again. trigger another measurement */
    this->m_fsm.eval();
    }

void cMeasurementLoop::savePower(uint32_t sleepInterval)
    {
    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();
    }

void cMeasurementLoop::deepSleepPrepare(void)
    {
    gpioenable.enableVdd1(false);
    gpiopower.setVdd1(false);

    Serial.end();
    Wire.end();
    SPI.end();
    if (this->m_pSPI2 && this->m_fSpi2Active)
        {
        this->m_pSPI2->end();
        this->m_fSpi2Active = false;
        }
    pinMode(D10, INPUT);
    }

void cMeasurementLoop::deepSleepRecovery(void)
    {
    pinMode(D10, OUTPUT);
    digitalWrite(D10, HIGH);
    Serial.begin();
    Wire.begin();
    SPI.begin();
    //if (this->m_pSPI2)
    //    this->m_pSPI2->begin();

    // power-up FRAM
    gpiopower.configVdd1(true);
    gpioenable.configVdd(true);
    gpiopower.setVdd1(true);
    delay(2);
    gpioenable.enableVdd1(true);
    delay(2);

    // start the FRAM
    if (! gFram.begin())
        {
        gLog.printf(
                gLog.kError,
                "CatenaStm32L0::begin: FRAM begin() failed\n"
                );
        }

    // check whether the FRAM is valid
    if (! gFram.isValid())
        {
        gLog.printf(
                gLog.kError,
                "CatenaStm32L0::begin: FRAM contents are not valid, resetting\n"
                );
        gFram.initialize();
        }
    }

/****************************************************************************\
|
|  Time-out asynchronous measurements.
|
\****************************************************************************/

// set the timer
void cMeasurementLoop::setTimer(std::uint32_t ms)
    {
    this->m_timer_start = millis();
    this->m_timer_delay = ms;
    this->m_fTimerActive = true;
    this->m_fTimerEvent = false;
    }

void cMeasurementLoop::clearTimer()
    {
    this->m_fTimerActive = false;
    this->m_fTimerEvent = false;
    }

bool cMeasurementLoop::timedOut()
    {
    bool result = this->m_fTimerEvent;
    this->m_fTimerEvent = false;
    return result;
    }
