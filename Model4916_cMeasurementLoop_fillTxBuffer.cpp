/*

Module: Model4916_cMeasurementLoop_fillBuffer.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include <Catena_TxBuffer.h>

#include "Model4916_cMeasurementLoop.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciModel4916;

/*

Name:   McciModel4916::cMeasurementLoop::fillTxBuffer()

Function:
    Prepare a messages in a TxBuffer with data from current measurements.

Definition:
    void McciModel4916::cMeasurementLoop::fillTxBuffer(
            cMeasurementLoop::TxBuffer_t& b
            );

Description:
    A format 0x28 message is prepared from the data in the cMeasurementLoop
    object.

*/

void
cMeasurementLoop::fillTxBuffer(
    cMeasurementLoop::TxBuffer_t& b, Measurement const &mData
    )
    {
    gLed.Set(McciCatena::LedPattern::Measuring);


    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(kMessageFormat);

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));

    // send Vbat
    if ((this->m_data.flags &  Flags::Vbat) !=  Flags(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // send boot count
    if ((this->m_data.flags &  Flags::Boot) !=  Flags(0))
        {
        b.putBootCountLsb(this->m_data.BootCount);
        }

    // put temperature and humidity data
    if ((mData.flags & Flags::TH) != Flags(0))
        {
        if (this->m_fSht3x)
            {
            gCatena.SafePrintf(
                    "SHT3x      :  T: %d RH: %d\n",
                    (int) mData.env.TempC,
                    (int) mData.env.Humidity
                    );
            b.putT(mData.env.TempC);
            // no method for 2-byte RH, directly encode it.
            b.put2uf((mData.env.Humidity / 100.0f) * 65535.0f);
            }
        }

    // put gps data
    if ((mData.flags & Flags::Time) != Flags(0))
        {
        gCatena.SafePrintf(
            "SAM-M8Q GPS:  UNIX EPOCH TIME: %d\n",
            (int) mData.position.UnixTime
            );

        // insert the timestamp from the gps module
        // stuff zero if time is not valid.
        b.put4u(std::uint32_t(mData.position.UnixTime));
        }

    if ((mData.flags & Flags::GPS) != Flags(0))
        {
        gCatena.SafePrintf(
            "SAM-M8Q GPS:  LATITUDE(deg): %d.%07d  LONGITUDE(deg): %d.%07d\n",
            (int) mData.position.Latitude, this->getGpsDecimal(mData.position.Latitude),
            (int) mData.position.Longitude, this->getGpsDecimal(mData.position.Longitude)
            );

        b.put2(std::uint32_t(LMIC_f2sflt16(mData.position.Latitude / 256.0f)));
        b.put2(std::uint32_t(LMIC_f2sflt16(mData.position.Longitude / 512.0f)));
        }

    // put pm and pc data
    if ((mData.flags & Flags::PM) != Flags(0))
        {
        gCatena.SafePrintf(
            "IPS7100    :  PM0.1: %d.%02d  PM0.3: %d.%02d  PM0.5: %d.%02d  PM1.0: %d.%02d  PM2.5: %d.%02d   PM5.0: %d.%02d   PM10: %d.%02d\n",
            (int) mData.particle.Mass[0], this->getDecimal(mData.particle.Mass[0]),
            (int) mData.particle.Mass[1], this->getDecimal(mData.particle.Mass[1]),
            (int) mData.particle.Mass[2], this->getDecimal(mData.particle.Mass[2]),
            (int) mData.particle.Mass[3], this->getDecimal(mData.particle.Mass[3]),
            (int) mData.particle.Mass[4], this->getDecimal(mData.particle.Mass[4]),
            (int) mData.particle.Mass[5], this->getDecimal(mData.particle.Mass[5]),
            (int) mData.particle.Mass[6], this->getDecimal(mData.particle.Mass[6])
            );

        gCatena.SafePrintf(
            "IPS7100    :  PC0.1: %d  PC0.3: %d  PC0.5: %d  PC1.0: %d  PC2.5: %d  PC5.0: %d  PC10: %d\n",
            mData.particle.Count[0],
            mData.particle.Count[1],
            mData.particle.Count[2],
            mData.particle.Count[3],
            mData.particle.Count[4],
            mData.particle.Count[5],
            mData.particle.Count[6]
            );

        for (uint8_t i = 0; i < 7; i++)
            b.put2uf(this->uflt16(mData.particle.Mass[i] / 512));

        // send Particle concentration only for PC1.0, PC2.5 and PC10
        b.put4u(mData.particle.Count[3]);
        b.put4u(mData.particle.Count[4]);
        b.put4u(mData.particle.Count[6]);
        }

    if ((mData.flags & Flags::TVOC) != Flags(0))
        {
        if (this->m_fBme680)
            {
            gCatena.SafePrintf(
                "BME680:   BreathVOC(ppm): %d.%02d\n",
                (int) (mData.airQuality.BreathVoc),
                this->getDecimal(mData.airQuality.BreathVoc)
                );
            }

        b.put2uf(this->uflt16(mData.airQuality.BreathVoc / 16));
        }

    if ((mData.flags & Flags::IAQ) != Flags(0))
        {
        if (this->m_fBme680)
            {
            gCatena.SafePrintf(
                "BME680:   IAQ: %d.%02d\n",
                (int) (mData.airQuality.Iaq + 0.5),
                this->getDecimal(mData.airQuality.Iaq + 0.5)
                );
            }

        b.put2uf(this->uflt16(mData.airQuality.Iaq / 512));
        }

    // the flags in Measurement correspond to the over-the-air flags.
    b.put((uint8_t)(uint16_t(mData.flags2) >> 8));

    gCatena.SafePrintf("VGas(mV):  CO:  %d  NO2:  %d  O3:  %d  SO2:  %d\n",
            (int) (mData.gases.uVCO / 1000.00f),
            (int) (mData.gases.uVNO2 / 1000.00f),
            (int) (mData.gases.uVO3 / 1000.00f),
            (int) (mData.gases.uVSO2 / 1000.00f)
            );

    // put CO
    if ((mData.flags2 & Flags::CO) != Flags(0))
        {
        gCatena.SafePrintf(
            "GasPPM:  CO: %d.%02d  ",
            (int) mData.gases.CO, this->getDecimal(mData.gases.CO)
            );

        b.putV(mData.gases.uVCO / 1000000);
        b.put2uf(this->uflt16(mData.gases.CO/1000));
        }

    // put NO2
    if ((mData.flags2 & Flags::NO2) != Flags(0))
        {
        gCatena.SafePrintf(
            " NO2: %d.%02d  ",
            (int) mData.gases.NO2, this->getDecimal(mData.gases.NO2)
            );

        b.putV(mData.gases.uVNO2 / 1000000);
        b.put2uf(this->uflt16(mData.gases.NO2/10));
        }

    // put O3
    if ((mData.flags2 & Flags::O3) != Flags(0))
        {
        gCatena.SafePrintf(
            " O3: %d.%02d  ",
            (int) mData.gases.O3, this->getDecimal(mData.gases.O3)
            );

        b.putV(mData.gases.uVO3 / 1000000);
        b.put2uf(this->uflt16(mData.gases.O3/30));
        }

    // put SO2
    if ((mData.flags2 & Flags::SO2) != Flags(0))
        {
        gCatena.SafePrintf(
            " SO2: %d.%02d\n",
            (int) mData.gases.SO2, this->getDecimal(mData.gases.SO2)
            );

        b.putV(mData.gases.uVSO2 / 1000000);
        b.put2uf(this->uflt16(mData.gases.SO2/30));
        }

    // put co2ppm
    if ((mData.flags2 & Flags::CO2) != Flags(0))
        {
        gCatena.SafePrintf(
            "SCD30      :  T(C): %c%d.%02d  RH(%%): %d.%02d  CO2(ppm): %d.%02d\n",
            this->ts, this->tint, this->tfrac,
            this->rhint, this->rhfrac,
            this->co2int, this->co2frac
            );

        b.put2uf(this->uflt16(mData.co2ppm.CO2ppm / 40000));
        }

    gLed.Set(McciCatena::LedPattern::Off);
    }