/*

Name:   catena-message-0x27-port-1-format-test.cpp

Function:
    Generate test vectors for port 0x02 format 0x22 messages.

Copyright and License:
    See accompanying LICENSE file at https://github.com/mcci-catena/Model4916-MultiGas-Sensor

Author:
    Pranau R, MCCI Corporation   December 2022

*/

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

enum class OutputFormat
    {
    Bytes, Yaml
    };

template <typename T>
struct val
    {
    bool fValid;
    T v;
    };

struct env
    {
    float t;
    float rh;
    };

struct position
    {
    float lat;
    float lng;
    uint32_t time;
    };

struct airQuality
    {
    float tvoc;
    float iaq;
    };

struct particle
    {
    float mass[7];
    std::uint32_t count[7];
    };

struct co2ppm
    {
    float co2;
    };

struct gases
    {
    float co;
    float no2;
    float o3;
    float so2;
    };

struct Measurements
    {
    val<float> Vbat;
    val<std::uint8_t> Boot;
    val<env> Env;
    val<position> Position;
    val<airQuality> AirQuality;
    val<particle> Particle;
    val<co2ppm> CO2ppm;
    val<gases> Gases;
    };

//--- globals
OutputFormat gOutputFormat = OutputFormat::Bytes;

//--- code
uint16_t
LMIC_f2uflt16(
    float f
    )
    {
    if (f < 0.0)
        return 0;
    else if (f >= 1.0)
        return 0xFFFF;
    else
        {
        int iExp;
        float normalValue;

        normalValue = std::frexp(f, &iExp);

        // f is supposed to be in [0..1), so useful exp
        // is [0..-15]
        iExp += 15;
        if (iExp < 0)
            // underflow.
            iExp = 0;

        // bits 15..12 are the exponent
        // bits 11..0 are the fraction
        // we conmpute the fraction and then decide if we need to round.
        uint16_t outputFraction = std::ldexp(normalValue, 12) + 0.5;
        if (outputFraction >= (1 << 12u))
            {
            // reduce output fraction
            outputFraction = 1 << 11;
            // increase exponent
            ++iExp;
            }

        // check for overflow and return max instead.
        if (iExp > 15)
            return 0xFFFF;

        return (uint16_t)((iExp << 12u) | outputFraction);
        }
    }

/*

Name:   LMIC_f2sflt16()

Function:
        Encode a floating point number into a uint16_t.

Definition:
        uint16_t LMIC_f2sflt16(
                float f
                );

Description:
        The float to be transmitted must be a number in the range (-1.0, 1.0).
        It is converted to 16-bit integer formatted as follows:

                bits 15: sign
                bits 14..11: biased exponent
                bits 10..0: mantissa

        The float is properly rounded, and saturates.

        Note that the encoded value is sign/magnitude format, rather than
        two's complement for negative values.

Returns:
        0xFFFF for negative values <= 1.0;
        0x7FFF for positive values >= 1.0;
        Otherwise an appropriate float.

*/

uint16_t
LMIC_f2sflt16(
    float f
    )
    {
    if (f <= -1.0)
        return 0xFFFF;
    else if (f >= 1.0)
        return 0x7FFF;
    else
        {
        int iExp;
        float normalValue;
        uint16_t sign;

        normalValue = frexpf(f, &iExp);

        sign = 0;
        if (normalValue < 0)
            {
            // set the "sign bit" of the result
            // and work with the absolute value of normalValue.
            sign = 0x8000;
            normalValue = -normalValue;
            }

        // abs(f) is supposed to be in [0..1), so useful exp
        // is [0..-15]
        iExp += 15;
        if (iExp < 0)
            iExp = 0;

        // bit 15 is the sign
        // bits 14..11 are the exponent
        // bits 10..0 are the fraction
        // we conmpute the fraction and then decide if we need to round.
        uint16_t outputFraction = ldexpf(normalValue, 11) + 0.5;
        if (outputFraction >= (1 << 11u))
            {
            // reduce output fraction
            outputFraction = 1 << 10;
            // increase exponent
            ++iExp;
            }

        // check for overflow and return max instead.
        if (iExp > 15)
            return 0x7FFF | sign;

        return (uint16_t)(sign | (iExp << 11u) | outputFraction);
        }
    }

std::uint16_t encode16s(float v)
    {
    float nv = std::floor(v + 0.5f);

    if (nv > 32767.0f)
        return 0x7FFFu;
    else if (nv < -32768.0f)
        return 0x8000u;
    else
        {
        return (std::uint16_t) std::int16_t(nv);
        }
    }

std::uint16_t encode16u(float v)
    {
    float nv = std::floor(v + 0.5f);
    if (nv > 65535.0f)
        return 0xFFFFu;
    else if (nv < 0.0f)
        return 0;
    else
        {
        return std::uint16_t(nv);
        }
    }

std::uint16_t encode32u(float v)
    {
    float nv = std::floor(v + 0.5f);
    if (nv > 4294967295.0f)
        return 0xFFFFFFFFu;
    else if (nv < 0.0f)
        return 0;
    else
        {
        return std::uint32_t(nv);
        }
    }

std::uint16_t encodeV(float v)
    {
    return encode16s(v * 4096.0f);
    }

std::uint16_t encodeT(float v)
    {
    return encode16s(v * 256.0f);
    }

std::uint16_t encodeRH(float v)
    {
    return encode16u(v * 65535.0f / 100.0f);
    }

std::uint16_t encodeTime(float v)
    {
    return encode32u(v);
    }

std::uint16_t encodeGPS(float v)
    {
    return encode16u(LMIC_f2sflt16(v));
    }

std::uint16_t encodeAQ(std::uint16_t v)
    {
    return encode16u(LMIC_f2uflt16(v));
    }

std::uint16_t encodeParticle(float v)
    {
    return encode16u(LMIC_f2uflt16(v / 65536.0f));
    }

std::uint16_t encodeCo2(float v)
    {
    return encode16u(LMIC_f2uflt16(v / 40000.0f));
    }

std::uint16_t encodeGases(float v)
    {
    return encode16u(LMIC_f2uflt16(v));
    }

class Buffer : public std::vector<std::uint8_t>
    {
public:
    Buffer() : std::vector<std::uint8_t>() {};

    void push_back_be(std::uint16_t v)
        {
        this->push_back(std::uint8_t(v >> 8));
        this->push_back(std::uint8_t(v & 0xFF));
        }

    void push_back_be4(std::uint32_t v)
        {
        this->push_back(std::uint8_t(v >> 24));
        this->push_back(std::uint8_t(v >> 16));
        this->push_back(std::uint8_t(v >> 8));
        this->push_back(std::uint8_t(v & 0xFF));
        }
    };

void encodeMeasurement(Buffer &buf, Measurements &m)
    {
    std::uint8_t flags = 0;

    // send the type byte
    buf.clear();
    buf.push_back(0x27);

    // send the flag byte
    buf.push_back(0u); // flag byte.
    auto const iFlags = buf.size() - 1;

    // put the fields
    if (m.Vbat.fValid)
        {
        flags |= 1 << 0;
        buf.push_back_be(encodeV(m.Vbat.v));
        }

    if (m.Boot.fValid)
        {
        flags |= 1 << 1;
        buf.push_back(m.Boot.v);
        }

    if (m.Env.fValid)
        {
        flags |= 1 << 2;

        buf.push_back_be(encodeT(m.Env.v.t));
        buf.push_back_be(encodeRH(m.Env.v.rh));
        }

    if (m.Position.fValid)
        {
        flags |= 1 << 3;

        buf.push_back_be4(std::uint32_t(m.Position.v.time));
        buf.push_back_be(encodeGPS(m.Position.v.lat));
        buf.push_back_be(encodeGPS(m.Position.v.lng));
        }

    if (m.AirQuality.fValid)
        {
        flags |= 1 << 4;

        buf.push_back_be(encodeAQ(m.AirQuality.v.tvoc));
        }

    if (m.AirQuality.fValid)
        {
        flags |= 1 << 5;

        buf.push_back_be(encodeAQ(m.AirQuality.v.iaq));
        }

    if (m.Particle.fValid)
        {
        flags |= 1 << 6;

        buf.push_back_be(encodeParticle(m.Particle.v.mass[0]));
        buf.push_back_be(encodeParticle(m.Particle.v.mass[1]));
        buf.push_back_be(encodeParticle(m.Particle.v.mass[2]));
        buf.push_back_be(encodeParticle(m.Particle.v.mass[3]));
        buf.push_back_be(encodeParticle(m.Particle.v.mass[4]));
        buf.push_back_be(encodeParticle(m.Particle.v.mass[5]));
        buf.push_back_be(encodeParticle(m.Particle.v.mass[6]));

        buf.push_back_be(encodeParticle(m.Particle.v.count[0]));
        buf.push_back_be(encodeParticle(m.Particle.v.count[1]));
        buf.push_back_be(encodeParticle(m.Particle.v.count[2]));
        buf.push_back_be(encodeParticle(m.Particle.v.count[3]));
        buf.push_back_be(encodeParticle(m.Particle.v.count[4]));
        buf.push_back_be(encodeParticle(m.Particle.v.count[5]));
        buf.push_back_be(encodeParticle(m.Particle.v.count[6]));
        }

    if (m.CO2ppm.fValid)
        {
        flags |= 1 << 7;

        buf.push_back_be(encodeCo2(m.CO2ppm.v.co2));
        }

    if (m.Gases.fValid)
        {
        flags |= 1 << 8;

        buf.push_back_be(encodeCo2(m.Gases.v.co));
        }

    if (m.Gases.fValid)
        {
        flags |= 1 << 9;

        buf.push_back_be(encodeCo2(m.Gases.v.no2));
        }

    if (m.Gases.fValid)
        {
        flags |= 1 << 10;

        buf.push_back_be(encodeCo2(m.Gases.v.o3));
        }

    if (m.Gases.fValid)
        {
        flags |= 1 << 11;

        buf.push_back_be(encodeCo2(m.Gases.v.so2));
        }
    
    // update the flags
    buf.data()[iFlags] = flags;
    }

void logMeasurement(Measurements &m)
    {
    class Padder {
    public:
        Padder() : m_first(true) {}
        const char *get() {
            if (this->m_first)
                {
                this->m_first = false;
                return "";
                }
            else
                return " ";
            }
        const char *nl() {
            return this->m_first ? "" : "\n";
            }
    private:
        bool m_first;
    } pad;

    if (m.Vbat.fValid)
        {
        std::cout << pad.get() << "Vbat " << m.Vbat.v;
        }

    if (m.Boot.fValid)
        {
        std::cout << pad.get() << "Boot " << unsigned(m.Boot.v);
        }

    if (m.Env.fValid)
        {
        std::cout << pad.get() << "Env " << m.Env.v.t << " "
                                         << m.Env.v.rh;
        }

    if (m.Position.fValid)
        {
        std::cout << pad.get() << "Position "   << m.Position.v.time << " "
                                                << m.Position.v.lat << " "
                                                << m.Position.v.lng;
        }

    if (m.AirQuality.fValid)
        {
        std::cout << pad.get() << "AirQuality " << m.AirQuality.v.tvoc << " "
                                                << m.AirQuality.v.iaq;
        }

    if (m.Particle.fValid)
        {
        std::cout << pad.get() << "Particle " << m.Particle.v.mass[0] << " "
                                              << m.Particle.v.mass[1] << " "
                                              << m.Particle.v.mass[2] << " "
                                              << m.Particle.v.mass[3] << " "
                                              << m.Particle.v.mass[4] << " "
                                              << m.Particle.v.mass[5] << " "
                                              << m.Particle.v.mass[6] << " "
                                              << m.Particle.v.count[0] << " "
                                              << m.Particle.v.count[1] << " "
                                              << m.Particle.v.count[2] << " "
                                              << m.Particle.v.count[3] << " "
                                              << m.Particle.v.count[4] << " "
                                              << m.Particle.v.count[5] << " "
                                              << m.Particle.v.count[6];
        }

    if (m.CO2ppm.fValid)
        {
        std::cout << pad.get() << "CO2ppm " << m.CO2ppm.v.co2;
        }

    if (m.Gases.fValid)
        {
        std::cout << pad.get() << "Gases " << m.Gases.v.co << " "
                                           << m.Gases.v.no2 << " "
                                           << m.Gases.v.o3 << " "
                                           << m.Gases.v.so2;
        }

    // make the syntax cut/pastable.
    std::cout << pad.get() << ".\n";
    }

void putTestVector(Measurements &m)
    {
    Buffer buf {};
    logMeasurement(m);
    encodeMeasurement(buf, m);
    bool fFirst;

    fFirst = true;
    if (gOutputFormat == OutputFormat::Bytes)
        {
        for (auto v : buf)
            {
            if (! fFirst)
                std::cout << ' ';
            fFirst = false;
            std::cout.width(2);
            std::cout.fill('0');
            std::cout << std::hex << unsigned(v);
            }
        std::cout << '\n';
        std::cout << "length: " << std::dec << buf.end() - buf.begin() << '\n';
        }
    else if (gOutputFormat == OutputFormat::Yaml)
        {
        auto const sLeft = "  ";
        std::cout << "  examples:" << '\n'
                  << "    - description: XXX\n"
                  << "      input:\n"
                  << "        fPort: XXX\n"
                  << "        bytes: [";

        for (auto v : buf)
            {
            if (! fFirst)
                std::cout << ", ";
            fFirst = false;
            std::cout << std::dec << unsigned(v);
            }

        std::cout << "]\n"
                  << "      output:\n"
                  << "        data:\n"
                  << "          JSON-HERE\n"
                  ;
        }
    }

int main(int argc, char **argv)
    {
    Measurements m {0};
    Measurements m0 {0};
    bool fAny;
    std::string key;

    if (argc > 1)
        {
        std::string opt;
        opt = argv[1];
        if (opt == "--yaml")
            {
            std::cout << "(output in yaml format)\n";
            gOutputFormat = OutputFormat::Yaml;
            }
        else
            {
            std::cout << "invalid option ignored: " << opt << '\n';
            }
        }

    std::cout << "Input one or more lines of name/value tuples, ended by '.'\n";

    fAny = false;
    while (std::cin.good())
        {
        bool fUpdate = true;
        key.clear();

        std::cin >> key;
        
        if (key == "Vbat")
            {
            std::cin >> m.Vbat.v;
            m.Vbat.fValid = true;
            }
        else if (key == "Boot")
            {
            std::uint32_t nonce;
            std::cin >> nonce;
            m.Boot.v = (std::uint8_t) nonce;
            m.Boot.fValid = true;
            }
        else if (key == "Env")
            {
            std::cin >> m.Env.v.t >> m.Env.v.rh;
            m.Env.fValid = true;
            }
        else if (key == "Position")
            {
            std::cin >> m.Position.v.time >> m.Position.v.lat >> m.Position.v.lng;
            m.Position.fValid = true;
            }
        else if (key == "AirQuality")
            {
            std::cin >> m.AirQuality.v.tvoc >> m.AirQuality.v.iaq;
            m.AirQuality.fValid = true;
            }
        else if (key == "Particle")
            {
            std::cin >> m.Particle.v.mass[0]
                     >> m.Particle.v.mass[1]
                     >> m.Particle.v.mass[2]
                     >> m.Particle.v.mass[3]
                     >> m.Particle.v.mass[4]
                     >> m.Particle.v.mass[5]
                     >> m.Particle.v.mass[6]
                     >> m.Particle.v.count[0]
                     >> m.Particle.v.count[1]
                     >> m.Particle.v.count[2]
                     >> m.Particle.v.count[3]
                     >> m.Particle.v.count[4]
                     >> m.Particle.v.count[5]
                     >> m.Particle.v.count[6];

            m.Particle.fValid = true;
            }
        else if (key == "CO2ppm")
            {
            std::cin >> m.CO2ppm.v.co2;
            m.CO2ppm.fValid = true;
            }
        else if (key == "Gases")
            {
            std::cin >> m.Gases.v.co
                     >> m.Gases.v.no2
                     >> m.Gases.v.o3
                     >> m.Gases.v.so2;

            m.Gases.fValid = true;
            }
        else if (key == ".")
            {
            putTestVector(m);
            m = m0;
            fAny = false;
            fUpdate = false;
            }
        else if (key == "")
            /* ignore empty keys */
            fUpdate = false;
        else
            {
            std::cerr << "unknown key: " << key << "\n";
            fUpdate = false;
            }

        fAny |= fUpdate;
        }

    if (!std::cin.eof() && std::cin.fail())
        {
        std::string nextword;

        std::cin.clear(std::cin.goodbit);
        std::cin >> nextword;
        std::cerr << "parse error: " << nextword << "\n";
        return 1;
        }

    if (fAny)
        putTestVector(m);

    return 0;
    }