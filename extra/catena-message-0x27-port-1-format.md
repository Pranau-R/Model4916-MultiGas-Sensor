# Understanding MCCI Catena data sent on port 1 format 0x27

<!-- markdownlint-disable MD033 -->
<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
<!-- TOC depthFrom:2 updateOnSave:true -->

- [Overall Message Format](#overall-message-format)
- [Field format definitions](#field-format-definitions)
	- [Battery Voltage (field 0)](#battery-voltage-field-0)
	- [Boot counter (field 1)](#boot-counter-field-1)
	- [Environmental Readings (field 2)](#environmental-readings-field-2)
	- [GPS Readings (field 3)](#gps-readings-field-3)
	- [Particle Concentrations (field 4)](#particle-concentrations-field-4)
	- [Carbon-dioxide (field 5)](#carbon-dioxide-field-5)
	- [Carbon-monoxide (field 6)](#carbon-monoxide-field-6)
	- [Nitrogen-dioxide (field 7)](#nitrogen-dioxide-field-7)
	- [Ozone gas (field 8)](#ozone-gas-field-8)
	- [Sulfur-dioxide (field 9)](#sulfur-dioxide-field-9)
	- [Breath VOC (field 10)](#breath-voc-field-10)
	- [Air Quality Index (field 11)](#air-quality-index-field-11)
- [Data Formats](#data-formats)
	- [`uint16`](#uint16)
	- [`int16`](#int16)
	- [`uint8`](#uint8)
	- [`uint32`](#uint32)
	- [`int32`](#int32)
	- [`float32`](#float32)
	- [`float24`](#float24)
  - [`sflt16`](#sflt16)
  - [`uflt16`](#uflt16)
- [Test Vectors](#test-vectors)
- [Node-RED Decoding Script](#node-red-decoding-script)
- [The Things Network Console decoding script](#the-things-network-console-decoding-script)

<!-- /TOC -->
<!-- markdownlint-restore -->
<!-- Due to a bug in Markdown TOC, the table is formatted incorrectly if tab indentation is set other than 4. Due to another bug, this comment must be *after* the TOC entry. -->
<!-- due to another bug in Markdown TOC, you need to have Editor>Tab Size set to 4 (even though it will be auto-overridden); it uses that setting rather than the current setting for the file. -->

## Overall Message Format

MCCI Model 4916 - Multi Gas Sensor data with message format 0x27 are always sent on LoRaWAN port 1. Each message has the following layout.

byte | description
:---:|:---
0 | Format code (always 0x27, decimal 39).
1 | bitmap encoding the fields that follow
2..n | data bytes; use bitmap to decode.

Each bit in byte 1 represent whether a corresponding field in bytes 2..n is present. If all bits are clear, then no data bytes are present. If bit 0 is set, then field 0 is present; if bit 1 is set, then field 1 is present, and so forth.

Fields are appended sequentially in ascending order.  A bitmap of 0000101 indicates that field 0 is present, followed by field 2; the other fields are missing.  A bitmap of 00011010 indicates that fields 1, 3, and 4 are present, in that order, but that fields 0, 2, 5 and 6 are missing.

## Field format definitions

Each field has its own format, as defined in the following table. `int16`, `uint16`, etc. are defined after the table.

Field number (Bitmap bit) | Length of corresponding field (bytes) | Data format |Description
:---:|:---:|:---:|:----
0 | 2 | [int16](#int16) | [Battery voltage](#battery-voltage-field-0)
1 | 1 | [uint8](#uint8) | [Boot counter](#boot-counter-field-1)
2 | 4 | [int16](#int16), [uint16](#uint16) | [Temperature, humidity](environmental-readings-field-2)
3 | 8 | [uint32](#uint32), [sflt16](#sflt16), [sflt16](#sflt16) | [Timestamp, Latitude, Longitude](#gps-readings-field-3)
4 | 14 |  7 times [uflt16](#uflt16) | [Particle Concentrations](#particle-concentrations-field-4)
5 | 2 | [uflt16](#uflt16) | [Carbon-dioxide](#carbon-dioxide-field-5)
6 | 2 | [uflt16](#uflt16) | [Carbon-monoxide (field 6)](#carbon-monoxide-field-6)
7 | 2 | [uflt16](#uflt16) | [Nitrogen-dioxide (field 7)](#nitrogen-dioxide-field-7)
8 | 2 | [uflt16](#uflt16) | [Ozone gas (field 8)](#ozone-gas-field-8)
9 | 2 | [uflt16](#uflt16) | [Sulfur-dioxide (field 9)](#sulfur-dioxide-field-9)
10 | 2 | [uflt16](#uflt16) | [Breath VOC (field 10)](#breath-voc-field-10)
11 | 2 | [uflt16](#uflt16) | [Air Quality Index (field 11)](#air-quality-index-field-11)
12 | n/a | n/a | reserved, must always be zero.
13 | n/a | n/a | reserved, must always be zero.
14 | n/a | n/a | reserved, must always be zero.
14 | n/a | n/a | reserved, must always be zero.

### Battery Voltage (field 0)

Field 0, if present, carries the current battery voltage. To get the voltage, extract the int16 value, and divide by 4096.0. (Thus, this field can represent values from -8.0 volts to 7.998 volts.)

### Boot counter (field 1)

Field 2, if present, is a counter of number of recorded system reboots, modulo 256.

### Environmental Readings (field 2)

Field 3, if present, has three environmental readings.

- The first two bytes are a [`int16`](#int16) representing the temperature (divide by 256 to get degrees C).

- The next two bytes are a [`uint16`](#uint16) representing the barometric pressure (divide by 25 to get millibars). This is the station pressure, not the sea-level pressure.

- The last byte is a [`uint8`](#uint8) representing the relative humidity (divide by 2.56 to get percent).  (This field can represent humidity from 0% to 99.6%.)

### GPS Readings (field 3)

Field 3, if present, has 4-byte of timestamp as [`uint32`](#uint32). The field also has GPS co-ordinates, latitude and longitude data as [`sflt16`](#sflt16).

### Particle Concentrations (field 4)

Field 4, if present, has 14 particle concentrations as 28 bytes of data, each as a [`uflt16`](#uflt16).  `uflt16` values respresent values in [0, 1).

The fields in order are:

- PM0.1, PM0.3, PM0.5, PM1.0, PM2.5, PM5.0 and PM10 concentrations. Multiply by 65536 to get concentrations in &mu;g per cubic meter.
- Dust concentrations for particles of size 0.1, 0.3, 0.5, 1.0, 2.5, 5.0 and 10 microns. Multiply by 65536 to get particle counts per 0.1L of air.

### Carbon-dioxide (field 5)

Field 5, if present, is a two-byte [`uflt16`](#uflt16) representing the carbon dioxide concentration in parts per million (ppm). `uflt16` values represent numbers in the range [0.0..1.0). Multiply by 40000.0f to convert to ppm.

### Carbon-monoxide (field 6)

Field 6, if present, is a two-byte [`uflt16`](#uflt16) representing the carbon monoxide concentration in parts per million (ppm). `uflt16` values represent numbers in the range [0.0..1.0). Multiply by 40000.0f to convert to ppm.

### Nitrogen-dioxide (field 7)

Field 7, if present, is a two-byte [`uflt16`](#uflt16) representing the nitrogen dioxide concentration in parts per million (ppm). `uflt16` values represent numbers in the range [0.0..1.0). Multiply by 4.0f to convert to ppm.

### Ozone gas (field 8)

Field 8, if present, is a two-byte [`uflt16`](#uflt16) representing the ozone gas concentration in parts per million (ppm). `uflt16` values represent numbers in the range [0.0..1.0). Multiply by 4.0f to convert to ppm.

### Sulfur-dioxide (field 9)

Field 9, if present, is a two-byte [`uflt16`](#uflt16) representing the sulfur dioxide concentration in parts per million (ppm). `uflt16` values represent numbers in the range [0.0..1.0). Multiply by 4.0f to convert to ppm.

### Breath VOC (field 10)

Field 9, if present, is a two-byte [`uflt16`](#uflt16) representing the breath voc equivalent.

### Air Quality Index (field 11)

Field 11, if present, represents the BSEC Index of Air Quality (IAQ).

This is a number in the range 0 to 500, and is transmitted as a [`uflt16`](#uflt16) after dividing by 512.

   Quality   |   IAQ
:-----------:|:--------:
"good"       | 0 - 50
"average"    | 51 - 100
"little bad" | 101 - 150
"bad"        | 151 - 200
"worse"      | 201 - 300
"very bad"   | 301 - 500

## Data Formats

All multi-byte data is transmitted with the most significant byte first (big-endian format).  Comments on the individual formats follow.

### `uint16`

an integer from 0 to 65536.

### `int16`

a signed integer from -32,768 to 32,767, in two's complement form. (Thus 0..0x7FFF represent 0 to 32,767; 0x8000 to 0xFFFF represent -32,768 to -1.)

### `uint8`

an integer from 0 to 255.

### `uint32`

An integer from 0 to 4,294,967,295. The first byte is the most-significant 8 bits; the last byte is the least-significant 8 bits.

### `int32`

a signed integer from -2,147,483,648 to 2,147,483,647, in two's complement form. (0x00000000 through 0x7FFFFFFF represent 0 to 2,147,483,647; 0x80000000 to 0xFFFFFFFF represent -2,147,483,648 to -1.)

### `float32`

An IEEE-754 single-precision floating point number, in big-endian byte order. The layout is:

|  31  |  30..23  |   22..0
|:----:|:--------:|:---------:
| sign | exponent | mantissa

The sign bit is set if the number is negative, reset otherwise. Note that negative zero is possible.

The exponent is the binary exponent to be applied to the mantissa, less 127. (Thus an exponent of zero is represented by 0x7F in this field). The values 0xFF and 0x00 are special.  0xFF is used to represent "not-a-number" values; 0x00 is used to represent zero and "denormal" values.

The mantissa field represents the most-significant 23 bits of the binary fraction, starting at bit -1 (the "two-to-the-1/2" bit). The effective mantissa is the represented mantissa, plus 1 if the exponent field is not 0.

The following JavaScript code extracts a `float32` starting at position `i` from buffer `bytes`. It handles NANs and +/- infinity, which are all represented with `uExp` == 0xFF. JavaScript doesn't distinguish quiet and signaling NaNs, nor does it have signed NaNs. So for all NaNs other than infinite values, the result is just `Number.NaN`.

```javascript
function u4toFloat32(bytes, i) {
    // pick up four bytes at index i into variable u32
    var u32 = (bytes[i] << 24) + (bytes[i+1] << 16) + (bytes[i+2] << 8) + bytes[i+3];

    // extract sign, exponent, mantissa
    var bSign =     (u32 & 0x80000000) ? true : false;
    var uExp =      (u32 & 0x7F800000) >> 23;
    var uMantissa = (u32 & 0x007FFFFF);

    // if non-numeric, return appropriate result.
    if (uExp == 0xFF) {
      if (uMantissa == 0)
        return bSign ? Number.NEGATIVE_INFINITY
                     : Number.POSITIVE_INFINITY;
        else
            return Number.NaN;
    // else unless denormal, set the 1.0 bit
    } else if (uExp != 0) {
      uMantissa +=   0x00800000;
    } else { // denormal: exponent is the minimum
      uExp = 1;
    }

    // make a floating mantissa in [0,2); usually [1,2), but
    // sometimes (0,1) for denormals, and exactly zero for zero.
    var mantissa = uMantissa / 0x00800000;

    // apply the exponent.
    mantissa = Math.pow(2, uExp - 127) * mantissa;

    // apply sign and return result.
    return bSign ? -mantissa : mantissa;
}
```

### `float24`

An compressed single-precision floating point number, in big-endian byte order. The layout is:

|  23  |  22..16  |   15..0
|:----:|:--------:|:---------:
| sign | exponent | mantissa

This format (which is not standardized) can represent magnitudes in the approximate range $2.168 \cdot 10^{-19} < |x| < 1.8445 \cdot 10^{19}$, with a little more than 5 decimal digits of precision. It can also represent $\pm0$, $\pm\infty$, and "Not a Number".

The sign bit is set if the number is negative, reset otherwise. Note that negative zero is possible.

The 7-bit exponent is the binary exponent to be applied to the mantissa, less 63. (Thus an exponent of zero is represented by 0x3F in this field.)

The mantissa is the most significant 16 bits of the binary fraction, starting at bit -1 (the "two-to-the-1/2" bit). The effective mantissa is the represented mantissa, plus 1.0.

An exponent value of 0x7F is special; it means that the number is a "not-a-number" value. If the mantissa is zero, then such values represent positive or negative infinity (as determined by the sign). If the mantissa is non-zero, then the meaning is currently unspecified.

An exponent value of 0x00 is also special. If the mantissa is zero, then the number represents positive or negative zero (as determined by the sign) If the mantissa is non-zero, then the effective mantissa is the represented mantissa (as a fraction in $(0, 1.0)$, without the added 1.0 of a regular number, scaled by $2^{-62}$. Numbers of this kind are called _denormals_.

The following JavaScript code extracts a `float24` starting at position `i` from buffer `bytes`. It handles NANs and +/- infinity, which are all represented with `uExp` == 0x7F. JavaScript doesn't distinguish quiet and signaling NaNs, nor does it have signed NaNs. So for all NaNs other than infinite values, the result is just `Number.NaN`. It also handles denormals properly.

```javascript
function u3toFloat24(bytes, i) {
  // pick up three bytes at index i into variable u32
  var u24 = (bytes[i] << 16) + (bytes[i + 1] << 8) + bytes[i + 2];

  // extract sign, exponent, mantissa
  var bSign     = (u24 & 0x800000) ? true : false;
  var uExp      = (u24 & 0x7F0000) >> 16;
  var uMantissa = (u24 & 0x00FFFF);

  // if non-numeric, return appropriate result.
  if (uExp === 0x7F) {
    if (uMantissa === 0)
      return bSign ? Number.NEGATIVE_INFINITY
                   : Number.POSITIVE_INFINITY;
    else
      return Number.NaN;
  // else unless denormal, set the 1.0 bit
  } else if (uExp !== 0) {
    uMantissa += 0x010000;
  } else { // denormal: exponent is the minimum
    uExp = 1;
  }

  // make a floating mantissa in [0,2); usually [1,2), but
  // sometimes (0,1) for denormals, and exactly zero for zero.
  var mantissa = uMantissa / 0x010000;

  // apply the exponent.
  mantissa = Math.pow(2, uExp - 63) * mantissa;

  // apply sign and return result.
  return bSign ? -mantissa : mantissa;
}
```

### `sflt16`

A signed floating point number in the half-open range [0, 1), transmitted as a 16-bit number with the following interpretation:

bits | description
:---:|:---
15   | Sign (negative if set)
14..11 | binary exponent `b`
10..0 | fraction `f`

The floating point number is computed by computing `f`/2048 * 2^(`b`-15). Note that this format is deliberately not IEEE-compliant; it's intended to be easy to decode by hand and not overwhelmingly sophisticated.

Floating point mavens will immediately recognize:

1. This is a sign/magnitude format, not a two's complement format.
2. Numbers do not need to be normalized (although in practice they always are).
3. The format is somewhat wasteful, because it explicitly transmits the most-significant bit of the fraction. (Most binary floating-point formats assume that `f` is is normalized, which means by definition that the exponent `b` is adjusted and `f` is shifted left until the most-significant bit of `f` is one. Most formats then choose to delete the most-significant bit from the encoding. If we were to do that, we would insist that the actual value of `f` be in the range 2048..4095, and then transmit only `f - 2048`, saving a bit. However, this complicates the handling of gradual underflow; see next point.)
4. Gradual underflow at the bottom of the range is automatic and simple with this encoding; the more sophisticated schemes need extra logic (and extra testing) in order to provide the same feature.

### `uflt16`

A unsigned floating point number in the half-open range [0, 1), transmitted as a 16-bit number with the following interpretation:

bits | description
:---:|:---
15..12 | binary exponent `b`
11..0 | fraction `f`

The floating point number is computed by computing `f`/4096 * 2^(`b`-15). Note that this format is deliberately not IEEE-compliant; it's intended to be easy to decode by hand and not overwhelmingly sophisticated.

For example, if the transmitted message contains 0x1A, 0xAB, the equivalent floating point number is found as follows.

1. The full 16-bit number is 0x1AAB.
2. `b`  is therefore 0x1, and `b`-15 is -14.  2^-14 is 1/32768
3. `f` is 0xAAB. 0xAAB/4096 is 0.667
4. `f * 2^(b-15)` is therefore 0.6667/32768 or 0.0000204

Floating point mavens will immediately recognize:

- There is no sign bit; all numbers are positive.
- Numbers do not need to be normalized (although in practice they always are).
- The format is somewhat wasteful, because it explicitly transmits the most-significant bit of the fraction. (Most binary floating-point formats assume that `f` is is normalized, which means by definition that the exponent `b` is adjusted and `f` is shifted left until the most-significant bit of `f` is one. Most formats then choose to delete the most-significant bit from the encoding. If we were to do that, we would insist that the actual value of `f` be in the range 2048.. 4095, and then transmit only `f - 2048`, saving a bit. However, this complicated the handling of gradual underflow; see next point.)
- Gradual underflow at the bottom of the range is automatic and simple with this encoding; the more sophisticated schemes need extra logic (and extra testing) in order to provide the same feature.

## Test Vectors

<To be added>

## Node-RED Decoding Script

A Node-RED script to decode this data is part of this repository. You can download the latest version from gitlab:

- in raw form: <https://raw.githubusercontent.com/dhineshkumarmcci/Model4916-MultiGas-Sensor/main/extra/catena-message-0x27-port-1-decoder-nodered.js>
- or view it: <https://github.com/dhineshkumarmcci/Model4916-MultiGas-Sensor/blob/main/extra/catena-message-0x27-port-1-decoder-nodered.js>

The MCCI decoders add dewpoint where needed. For historical reasons, the temperature probe data is labled "tWater".

## The Things Network Console decoding script

The repository contains the script that decodes format 0x15, for [The Things Network console](https://console.thethingsnetwork.org).

You can get the latest version on gitlab:

- in raw form: <https://raw.githubusercontent.com/dhineshkumarmcci/Model4916-MultiGas-Sensor/main/extra/catena-message-0x27-port-1-decoder-ttn.js>
- or view it: <https://github.com/dhineshkumarmcci/Model4916-MultiGas-Sensor/blob/main/extra/catena-message-0x27-port-1-decoder-ttn.js>

The MCCI decoders add dewpoint where needed. For historical reasons, the temperature probe data is labled "tWater".
