/*
Name:   model4916-decoder-ttn.js

Function:
    This function decodes the record (port 1, format 0x27) sent by the
    MCCI Model 4916 multigas and environment sensor application.

Copyright and License:
    See accompanying LICENSE file

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022
*/

// calculate dewpoint (degrees C) given temperature (C) and relative humidity (0..100)
// from http://andrew.rsmas.miami.edu/bmcnoldy/Humidity.html
// rearranged for efficiency and to deal sanely with very low (< 1%) RH
function dewpoint(t, rh) {
    var c1 = 243.04;
    var c2 = 17.625;
    var h = rh / 100;
    if (h <= 0.01)
        h = 0.01;
    else if (h > 1.0)
        h = 1.0;

    var lnh = Math.log(h);
    var tpc1 = t + c1;
    var txc2 = t * c2;
    var txc2_tpc1 = txc2 / tpc1;

    var tdew = c1 * (lnh + txc2_tpc1) / (c2 - lnh - txc2_tpc1);
    return tdew;
}

/*

Name:   CalculateHeatIndex()

Description:
        Calculate the NWS heat index given dry-bulb T and RH

Definition:
        function CalculateHeatIndex(t, rh) -> value or null

Description:
        T is a Farentheit temperature in [76,120]; rh is a
        relative humidity in [0,100]. The heat index is computed
        and returned; or an error is returned.

Returns:
        number => heat index in Farenheit.
        null => error.

References:
        https://github.com/mcci-catena/heat-index/
        https://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

        Results was checked against the full chart at iweathernet.com:
        https://www.iweathernet.com/wxnetcms/wp-content/uploads/2015/07/heat-index-chart-relative-humidity-2.png

        The MCCI-Catena heat-index site has a test js script to generate CSV to
        match the chart, a spreadsheet that recreates the chart, and a
        spreadsheet that compares results.

*/

// calculate dewpoint (degrees C) given temperature (C) and relative humidity (0..100)
// from http://andrew.rsmas.miami.edu/bmcnoldy/Humidity.html
// rearranged for efficiency and to deal sanely with very low (< 1%) RH
function dewpoint(t, rh) {
    var c1 = 243.04;
    var c2 = 17.625;
    var h = rh / 100;
    if (h <= 0.01)
        h = 0.01;
    else if (h > 1.0)
        h = 1.0;

    var lnh = Math.log(h);
    var tpc1 = t + c1;
    var txc2 = t * c2;
    var txc2_tpc1 = txc2 / tpc1;

    var tdew = c1 * (lnh + txc2_tpc1) / (c2 - lnh - txc2_tpc1);
    return tdew;
}

/*

Name:   CalculateHeatIndex()

Description:
        Calculate the NWS heat index given dry-bulb T and RH

Definition:
        function CalculateHeatIndex(t, rh) -> value or null

Description:
        T is a Farentheit temperature in [76,120]; rh is a
        relative humidity in [0,100]. The heat index is computed
        and returned; or an error is returned.

Returns:
        number => heat index in Farenheit.
        null => error.

References:
        https://github.com/mcci-catena/heat-index/
        https://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

        Results was checked against the full chart at iweathernet.com:
        https://www.iweathernet.com/wxnetcms/wp-content/uploads/2015/07/heat-index-chart-relative-humidity-2.png

        The MCCI-Catena heat-index site has a test js script to generate CSV to
        match the chart, a spreadsheet that recreates the chart, and a
        spreadsheet that compares results.

*/

function CalculateHeatIndex(t, rh) {
    var tRounded = Math.floor(t + 0.5);

    // return null outside the specified range of input parameters
    if (tRounded < 76 || tRounded > 126)
        return null;
    if (rh < 0 || rh > 100)
        return null;

    // according to the NWS, we try this first, and use it if we can
    var tHeatEasy = 0.5 * (t + 61.0 + ((t - 68.0) * 1.2) + (rh * 0.094));

    // The NWS says we use tHeatEasy if (tHeatHeasy + t)/2 < 80.0
    // This is the same computation:
    if ((tHeatEasy + t) < 160.0)
            return tHeatEasy;

    // need to use the hard form, and possibly adjust.
    var t2 = t * t;         // t squared
    var rh2 = rh * rh;      // rh squared
    var tResult =
        -42.379 +
        (2.04901523 * t) +
        (10.14333127 * rh) +
        (-0.22475541 * t * rh) +
        (-0.00683783 * t2) +
        (-0.05481717 * rh2) +
        (0.00122874 * t2 * rh) +
        (0.00085282 * t * rh2) +
        (-0.00000199 * t2 * rh2);

    // these adjustments come from the NWA page, and are needed to
    // match the reference table.
    var tAdjust;
    if (rh < 13.0 && 80.0 <= t && t <= 112.0)
        tAdjust = -((13.0 - rh) / 4.0) * Math.sqrt((17.0 - Math.abs(t - 95.0)) / 17.0);
    else if (rh > 85.0 && 80.0 <= t && t <= 87.0)
        tAdjust = ((rh - 85.0) / 10.0) * ((87.0 - t) / 5.0);
    else
        tAdjust = 0;

    // apply the adjustment
    tResult += tAdjust;

    // finally, the reference tables have no data above 183 (rounded),
    // so filter out answers that we have no way to vouch for.
    if (tResult >= 183.5)
        return null;
    else
        return tResult;
}

function CalculateHeatIndexCelsius(t, rh) {
    var result = CalculateHeatIndex(t, rh);
    if (result !== null) {
        // convert to celsius.
        result = (result - 32) * 5 / 9;
    }
    return result;
}

function DecodeU16(Parse) {
    var i = Parse.i;
    var bytes = Parse.bytes;
    var raw = (bytes[i] << 8) + bytes[i + 1];
    Parse.i = i + 2;
    return raw;
}

function DecodeU16(Parse) {
    var i = Parse.i;
    var bytes = Parse.bytes;
    var raw = (bytes[i] << 8) + bytes[i + 1];
    Parse.i = i + 2;
    return raw;
}

function DecodeUflt16(Parse) {
    var rawUflt16 = DecodeU16(Parse);
    var exp1 = rawUflt16 >> 12;
    var mant1 = (rawUflt16 & 0xFFF) / 4096.0;
    var f_unscaled = mant1 * Math.pow(2, exp1 - 15);
    return f_unscaled;
}

function DecodeU32(Parse) {
    var i = Parse.i;
    var bytes = Parse.bytes;

    var result = (bytes[i + 0] << 24)+ (bytes[i + 1] << 16) + (bytes[i + 2] << 8) + bytes[i + 3];
    Parse.i = i + 4;

    return result;
}

function DecodePM(Parse) {
    return (DecodeUflt16(Parse) * 512);
}

function DecodePC(Parse) {
    return DecodeU32(Parse);
}

function DecodeSflt16(Parse) {
    var rawSflt16 = DecodeU16(Parse);

    // special case minus zero:
    if (rawSflt16 == 0x8000)
        return -0.0;

    // extract the sign.
    var sSign = ((rawSflt16 & 0x8000) != 0) ? -1 : 1;

    // extract the exponent
    var exp1 = (rawSflt16 >> 11) & 0xF;

    // extract the "mantissa" (the fractional part)
    var mant1 = (rawSflt16 & 0x7FF) / 2048.0;

    // convert back to a floating point number. We hope
    // that Math.pow(2, k) is handled efficiently by
    // the JS interpreter! If this is time critical code,
    // you can replace by a suitable shift and divide.
    var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);

    return f_unscaled;
}

function DecodeGpsCoordinates(Parse) {
    return DecodeSflt16(Parse);
}

function DecodeI16(Parse) {
    var Vraw = DecodeU16(Parse);

    // interpret uint16 as an int16 instead.
    if (Vraw & 0x8000)
        Vraw += -0x10000;

    return Vraw;
}

function DecodeI16(Parse) {
    var i = Parse.i;
    var bytes = Parse.bytes;
    var Vraw = (bytes[i] << 8) + bytes[i + 1];
    Parse.i = i + 2;

    // interpret uint16 as an int16 instead.
    if (Vraw & 0x8000)
        Vraw += -0x10000;

    return Vraw;
}

function DecodeV(Parse) {
    return DecodeI16(Parse) / 4096.0;
}

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    if (! (port === 1))
        return null;

    var uFormat = bytes[0];
    if (! (uFormat === 0x27))
        return null;

    // an object to help us parse.
    var Parse = {};
    Parse.bytes = bytes;
    // i is used as the index into the message. Start with the flag byte.
    Parse.i = 1;

    // fetch the bitmap.
    var flags = bytes[Parse.i++];

    if (flags & 0x1) {
        // scale and save in decoded.
        decoded.vBat = DecodeV(Parse);
    }

    if (flags & 0x2) {
        var iBoot = bytes[Parse.i++];
        decoded.boot = iBoot;
    }

    if (flags & 0x4) {
        // we have temp, RH
        decoded.t = DecodeI16(Parse) / 256;
        decoded.rh = DecodeU16(Parse) * 100 / 65535.0;
        decoded.tDew =  dewpoint(decoded.t, decoded.rh);
        decoded.tHeatIndexC = CalculateHeatIndexCelsius(decoded.t, decoded.rh);
    }

    if (flags & 0x8) {
        // we have gps time
        decoded.time = new Date((DecodeU32(Parse) + /* gps epoch to posix */ 315964800 - /* leap seconds */ 17) * 1000);
    }

    if (flags & 0x10) {
        // we have gps co-ordinates
        decoded.latitude = DecodeGpsCoordinates(Parse) * 9.0;
        decoded.longitude = DecodeGpsCoordinates(Parse) * 18.0;
    }

    if (flags & 0x20) {
        // we have micro particles and their count
        decoded.pm = {};
        decoded.pm["0.1"] = DecodePM(Parse);
        decoded.pm["0.3"] = DecodePM(Parse);
        decoded.pm["0.5"] = DecodePM(Parse);
        decoded.pm["1.0"] = DecodePM(Parse);
        decoded.pm["2.5"] = DecodePM(Parse);
        decoded.pm["5.0"] = DecodePM(Parse);
        decoded.pm["10"] = DecodePM(Parse);

        decoded.pc = {};
        decoded.pc["1.0"] = DecodePC(Parse);
        decoded.pc["2.5"] = DecodePC(Parse);
        decoded.pc["10"] = DecodePC(Parse);
    }

    if (flags & 0x40) {
        // we have tvoc
        decoded.tvoc =  DecodeUflt16(Parse) * 16;
    }

    if (flags & 0x80) {
        // we have iaq
        decoded.iaq = DecodeUflt16(Parse) * 512;
    }

    // fetch the bitmap.
    var flags2 = bytes[Parse.i++];    

    if (flags2 & 0x1) {
        // we have carbon monoxide
        decoded.vCO = DecodeV(Parse);
        decoded.COppm = DecodeUflt16(Parse) * 1000;
    }

    if (flags2 & 0x2) {
        // we have nitrogen-dioxide
        decoded.vNO2 = DecodeV(Parse);
        decoded.NO2ppm = DecodeUflt16(Parse) * 10;
    }

    if (flags2 & 0x4) {
        // we have ozone gas
        decoded.vO3 = DecodeV(Parse);
        decoded.O3ppm = DecodeUflt16(Parse) * 30;
    }

    if (flags2 & 0x8) {
        // we have sulphur-dioxide
        decoded.vSO2 = DecodeV(Parse);
        decoded.SO2ppm = DecodeUflt16(Parse) * 30;
    }

    if (flags & 0x10) {
        // we have co2
        decoded.CO2 = DecodeUflt16(Parse) * 40000;
    }
    return decoded;
}

var bytes;

if ("payload_raw" in msg) {
    // the console already decoded this
    bytes = msg.payload_raw;  // pick up data for convenience
    // msg.payload_fields still has the decoded data from ttn
} else {
    // no console decode
    bytes = msg.payload;  // pick up data for conveneince
}

// try to decode.
var result = Decoder(bytes, msg.port);

if (result === null) {
    node.error("not port 1/fmt 0x27! port=" + msg.port.toString());
}

// now update msg with the new payload and new .local field
// the old msg.payload is overwritten.
msg.payload = result;
msg.local =
    {
        nodeType: "Model 4916",
        platformType: "Model 4916",
        radioType: "Murata",
        applicationName: "MultiGas sensor"
    };

return msg;