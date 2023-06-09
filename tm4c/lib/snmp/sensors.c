//
// Created by robert on 5/3/23.
//

#include <math.h>
#include <string.h>
#include "../clk/mono.h"
#include "../ptp/pll.h"
#include "../ptp/tcmp.h"
#include "sensors.h"
#include "util.h"


static const uint8_t OID_SENSOR_PREFIX[] = { 0x06, 0x0A, 0x2B, 6, 1, 2, 1, 99, 1, 1, 1 };


// CPU temperature getter
static int getCpuTemp()         { return lroundf(TCMP_temp() * 1e4f); }
// PLL offset getters
static int getPllOffsetLast()   { return lroundf(PLL_offsetLast() * 1e10f); }
static int getPllOffsetMean()   { return lroundf(PLL_offsetMean() * 1e10f); }
static int getPllOffsetRms()    { return lroundf(PLL_offsetRms() * 1e10f); }
static int getPllOffsetStdDev() { return lroundf(PLL_offsetStdDev() * 1e10f); }
static int getPllOffsetProp()   { return lroundf(PLL_offsetProp() * 1e10f); }
static int getPllOffsetInt()    { return lroundf(PLL_offsetInt() * 1e10f); }
static int getPllOffsetCorr()   { return lroundf(PLL_offsetCorr() * 1e10f); }
// PLL drift getters
static int getPllDriftLast()    { return lroundf(PLL_driftLast() * 1e10f); }
static int getPllDriftMean()    { return lroundf(PLL_driftMean() * 1e10f); }
static int getPllDriftRms()     { return lroundf(PLL_driftRms() * 1e10f); }
static int getPllDriftStdDev()  { return lroundf(PLL_driftStdDev() * 1e10f); }
static int getPllDriftCorr()    { return lroundf(PLL_driftCorr() * 1e10f); }
static int getPllDriftFreq()    { return lroundf(PLL_driftFreq() * 1e10f); }


// SNMP Sensor Registry
static const struct SnmpSensor {
    const char *name;
    const char *units;
    uint8_t typeId;
    uint8_t scale;
    uint8_t precision;
    int (*getter)();
} snmpSensors[] = {
        // CPU temperature
        { "cpu.temp",           "C",    OID_SENSOR_TYPE_CELSIUS,  OID_SENSOR_SCALE_1,     4, getCpuTemp         },
        // PLL offset stats
        { "pll.offset.last",    "s",    OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetLast   },
        { "pll.offset.mean",    "s",    OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetMean   },
        { "pll.offset.rms",     "s",    OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetRms    },
        { "pll.offset.stddev",  "s",    OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetStdDev },
        { "pll.offset.prop",    "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetProp   },
        { "pll.offset.int",     "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetInt    },
        { "pll.offset.corr",    "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllOffsetCorr   },
        // PLL drift stats
        { "pll.drift.last",     "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllDriftLast    },
        { "pll.drift.mean",     "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllDriftMean    },
        { "pll.drift.rms",      "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllDriftRms     },
        { "pll.drift.stddev",   "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllDriftStdDev  },
        { "pll.drift.corr",     "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllDriftCorr    },
        { "pll.drift.freq",     "s/s",  OID_SENSOR_TYPE_OTHER,    OID_SENSOR_SCALE_1E_6,  4, getPllDriftFreq    }
};
#define SNMP_SENS_CNT (sizeof(snmpSensors) / sizeof(struct SnmpSensor))


int SNMP_writeSensorTypes(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // report sensor type
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt8(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_TYPE, snmpSensors[i].typeId
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorScales(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // report sensor scale
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt8(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_SCALE, snmpSensors[i].scale
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorPrecs(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // report sensor precision
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt8(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_PREC, snmpSensors[i].precision
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorValues(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // report sensor values
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt32(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_VALUE, (*(snmpSensors[i].getter))()
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorStatuses(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // sensors are virtual and  always present
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt8(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_STATUS, OID_SENSOR_STATUS_OK
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorUnits(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // report human-readable units
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        const char *units = snmpSensors[i].units;
        ptr += SNMP_writeValueBytes(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_UNITS, units, (int) strlen(units)
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorUpdateTimes(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // sensors are virtual and always current
    uint32_t timeTicks = ((CLK_MONO() * 100) >> 32);
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt32(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_UPDATE_TIME, timeTicks
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorUpdateRates(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // sensors are virtual and always current
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        ptr += SNMP_writeValueInt8(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_UPDATE_RATE, 0
        );
    }

    return ptr - dst;
}

int SNMP_writeSensorNames(uint8_t * const dst) {
    uint8_t *ptr = dst;

    // report human-readable names
    for(int i = 0; i < SNMP_SENS_CNT; i++) {
        const char *name = snmpSensors[i].name;
        ptr += SNMP_writeValueBytes(
                ptr, OID_SENSOR_PREFIX, sizeof(OID_SENSOR_PREFIX), OID_SENSOR_NAME, name, (int) strlen(name)
        );
    }

    return ptr - dst;
}
