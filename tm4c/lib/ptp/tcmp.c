//
// Created by robert on 4/30/23.
//

#include <math.h>
#include "../../hw/adc.h"
#include "../../hw/eeprom.h"
#include "../clk/comp.h"
#include "../clk/mono.h"
#include "../delay.h"
#include "../format.h"
#include "../run.h"
#include "tcmp.h"

#define ADC_RATE_MEAN (0x1p-11f)
#define ADC_RATE_VAR  (0x1p-12f)

#define INTV_TEMP (1u << (32 - 10)) // 1024 Hz
#define INTV_TCMP (1u << (32 - 4))  // 16 Hz

#define TCMP_SAVE_INTV (3600) // save state every hour

#define SOM_EEPROM_BASE (0x0020)
#define SOM_NODE_CNT (16)
#define SOM_FILL_OFF (2.0f)
#define SOM_FILL_REG (8.0f)
#define SOM_RATE_MAX (0.25f)

#define REG_MIN_RMSE (250e-9f)

static volatile float adcMean;
static volatile float adcVar;
static volatile float tempValue;

static volatile uint32_t tcmpSaved;
static volatile float tcmpValue;

static volatile float tcmpMean[3];
static volatile float tcmpCoeff[3];
static volatile float tcmpRmse;

// SOM for filtering compensation samples
static volatile float somNode[SOM_NODE_CNT][3];
static volatile float somNW[SOM_NODE_CNT];

static void loadSom();
static void saveSom();
static void seedSom(float temp, float comp);
static void updateSom(float temp, float comp);
static void updateRegression();
static void fitLinear(const float *data, int cnt, float *coef, float *mean);

/**
 * Estimate temperature correction using 3rd order taylor series
 * @param temp current temperature in Celsius
 * @return estimated correction value
 */
static float tcmpEstimate(float temp);

static void runAdc(void *ref) {
    uint32_t acc;
    // ADC FIFO will always contain eight samples
    acc  = ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    // trigger next sample
    ADC0.PSSI.SS0 = 1;

    // store result
    float adcValue = 0x1p-3f * (float) acc;
    float diff = adcValue - adcMean;
    float var = diff * diff;
    if(var <= 4 * adcVar)
        adcMean += ADC_RATE_MEAN * diff;
    adcVar += ADC_RATE_VAR * (var - adcVar);
}

static void runComp(void *ref) {
    // update temperature compensation
    tempValue = 147.5f - (0.0604248047f * adcMean);
    tcmpValue = tcmpEstimate(tempValue);
    CLK_COMP_setComp((int32_t) (0x1p32f * tcmpValue));
}

void TCMP_init() {
    // Enable ADC0
    RCGCADC.EN_ADC0 = 1;
    delay_cycles_4();

    // configure ADC0 for temperature measurement
    ADC0.CC.CLKDIV = 0;
    ADC0.CC.CS = ADC_CLK_MOSC;
    ADC0.SAC.AVG = 0;
    ADC0.EMUX.EM0 = ADC_SS_TRIG_SOFT;
    ADC0.SS0.CTL.TS0 = 1;
    ADC0.SS0.TSH.TSH0 = ADC_TSH_256;
    ADC0.SS0.CTL.TS1 = 1;
    ADC0.SS0.TSH.TSH1 = ADC_TSH_256;
    ADC0.SS0.CTL.TS2 = 1;
    ADC0.SS0.TSH.TSH2 = ADC_TSH_256;
    ADC0.SS0.CTL.TS3 = 1;
    ADC0.SS0.TSH.TSH3 = ADC_TSH_256;
    ADC0.SS0.CTL.TS4 = 1;
    ADC0.SS0.TSH.TSH4 = ADC_TSH_256;
    ADC0.SS0.CTL.TS5 = 1;
    ADC0.SS0.TSH.TSH5 = ADC_TSH_256;
    ADC0.SS0.CTL.TS6 = 1;
    ADC0.SS0.TSH.TSH6 = ADC_TSH_256;
    ADC0.SS0.CTL.TS7 = 1;
    ADC0.SS0.TSH.TSH7 = ADC_TSH_256;
    ADC0.SS0.CTL.IE7 = 1;
    ADC0.SS0.CTL.END7 = 1;
    ADC0.ACTSS.ASEN0 = 1;
    // take and discard some initial measurements
    ADC0.PSSI.SS0 = 1;      // trigger temperature measurement
    while(!ADC0.RIS.INR0);  // wait for data
    ADC0.ISC.IN0 = 1;       // clear flag
    uint32_t acc;
    // ADC FIFO will always contain eight samples
    acc  = ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    acc += ADC0.SS0.FIFO.DATA;
    // trigger next sample
    ADC0.PSSI.SS0 = 1;
    // store result
    adcMean = 0x1p-3f * (float) acc;

    loadSom();
    if(isfinite(somNode[0][0])) {
        updateRegression();
        runComp(NULL);
    }

    // schedule thread
    runPeriodic(INTV_TEMP, runAdc, NULL);
    runPeriodic(INTV_TCMP, runComp, NULL);
}

float TCMP_temp() {
    return tempValue;
}

float TCMP_get() {
    return tcmpValue;
}

void TCMP_update(const float target) {
    updateSom(tempValue, target);
    updateRegression();

    const uint32_t now = CLK_MONO_INT();
    if(now - tcmpSaved > TCMP_SAVE_INTV) {
        tcmpSaved = now;
        saveSom();
    }
}

unsigned TCMP_status(char *buffer) {
    char tmp[32];
    char *end = buffer;

    end = append(end, "tcomp status:\n");

    tmp[fmtFloat(tempValue, 12, 4, tmp)] = 0;
    end = append(end, "  - temp:    ");
    end = append(end, tmp);
    end = append(end, " C\n");

    tmp[fmtFloat(0.0604248047f * sqrtf(adcVar), 12, 4, tmp)] = 0;
    end = append(end, "  - noise:   ");
    end = append(end, tmp);
    end = append(end, " C\n");

    tmp[fmtFloat(tcmpMean[2], 12, 4, tmp)] = 0;
    end = append(end, "  - fill:    ");
    end = append(end, tmp);
    end = append(end, "\n");

    tmp[fmtFloat(tcmpRmse * 1e6f, 12, 4, tmp)] = 0;
    end = append(end, "  - rmse:    ");
    end = append(end, tmp);
    end = append(end, " ppm\n");

    tmp[fmtFloat(tcmpMean[0], 12, 4, tmp)] = 0;
    end = append(end, "  - mean[0]: ");
    end = append(end, tmp);
    end = append(end, " C\n");

    tmp[fmtFloat(tcmpMean[1] * 1e6f, 12, 4, tmp)] = 0;
    end = append(end, "  - mean[1]: ");
    end = append(end, tmp);
    end = append(end, " ppm\n");

    tmp[fmtFloat(tcmpCoeff[0] * 1e6f, 12, 4, tmp)] = 0;
    end = append(end, "  - coef[0]: ");
    end = append(end, tmp);
    end = append(end, " ppm/C\n");

    tmp[fmtFloat(tcmpCoeff[1] * 1e6f, 12, 4, tmp)] = 0;
    end = append(end, "  - coef[1]: ");
    end = append(end, tmp);
    end = append(end, " ppm/C^2\n");

    tmp[fmtFloat(tcmpCoeff[2] * 1e6f, 12, 4, tmp)] = 0;
    end = append(end, "  - coef[2]: ");
    end = append(end, tmp);
    end = append(end, " ppm/C^3\n");

    tmp[fmtFloat(tcmpValue * 1e6f, 12, 4, tmp)] = 0;
    end = append(end, "  - curr:    ");
    end = append(end, tmp);
    end = append(end, " ppm\n\n");

    return end - buffer;
}

static void loadSom() {
    // initialize neighbor weights
    for(int i = 0; i < SOM_NODE_CNT; i++)
        somNW[i] = expf(-2.0f * (float) (i * i));

    uint32_t *ptr = (uint32_t *) somNode;
    uint32_t *end = ptr + (sizeof(somNode) / sizeof(uint32_t));

    // load SOM data
    EEPROM_seek(SOM_EEPROM_BASE);
    while(ptr < end)
        *(ptr++) = EEPROM_read();
}

static void saveSom() {
    uint32_t *ptr = (uint32_t *) somNode;
    uint32_t *end = ptr + (sizeof(somNode) / sizeof(uint32_t));

    // load SOM data
    EEPROM_seek(SOM_EEPROM_BASE);
    while(ptr < end)
        EEPROM_write(*(ptr++));
}

static void seedSom(float temp, float comp) {
    float norm = 0.01f / (float) SOM_NODE_CNT;
    int mid = SOM_NODE_CNT / 2;
    for(int i = 0; i < SOM_NODE_CNT; i++) {
        somNode[i][0] = temp + (norm * (float)(i - mid));
        somNode[i][1] = comp;
        somNode[i][2] = 0;
    }
}

static void updateSom(float temp, float comp) {
    // initialize som nodes if necessary
    if(!isfinite(somNode[0][0])) {
        seedSom(temp, comp);
    }

    // locate nearest node and measure learning progress
    float alpha = 0;
    int best = 0;
    float dist = fabsf(temp - somNode[0][0]);
    for(int i = 1; i < SOM_NODE_CNT; i++) {
        alpha += somNode[i][2];
        float diff = fabsf(temp - somNode[i][0]);
        if(diff < dist) {
            dist = diff;
            best = i;
        }
    }

    // update nodes using dynamic learning rate
    alpha = SOM_RATE_MAX / (1.0f + (alpha * alpha));
    for(int i = 0; i < SOM_NODE_CNT; i++) {
        // compute neighbor distance
        int ndist = i - best;
        if(ndist < 0) ndist = -ndist;
        // compute node alpha
        const float w = alpha * somNW[ndist];
        // update node weights
        somNode[i][0] += (temp - somNode[i][0]) * w;
        somNode[i][1] += (comp - somNode[i][1]) * w;
        somNode[i][2] += (1.0f - somNode[i][2]) * w;
    }
}

static void updateRegression() {
    float mean[3];
    float coef[4];

    // compute initial linear fit
    fitLinear((float *) somNode, SOM_NODE_CNT, coef + 1, mean);
    // wait for sufficient data
    if(mean[2] < SOM_FILL_OFF)
        return;
    // update means
    tcmpMean[0] = mean[0];
    tcmpMean[1] = mean[1];
    tcmpMean[2] = mean[2];

    // wait for sufficient data
    if(mean[2] < SOM_FILL_REG)
        return;

    // compute residual error
    float rmse = 0;
    for(int i = 0; i < SOM_NODE_CNT; i++) {
        float x = somNode[i][0] - mean[0];
        float y = somNode[i][1] - mean[1];
        y -= x * coef[1];
        rmse += y * y * somNode[i][2];
    }
    rmse /= mean[2];
    rmse *= 4;

    // prune outliers
    float scratch[SOM_NODE_CNT][3];
    for(int i = 0; i < SOM_NODE_CNT; i++) {
        // copy X and Y values
        scratch[i][0] = somNode[i][0];
        scratch[i][1] = somNode[i][1];

        // determine if sample should be excluded
        float x = somNode[i][0] - mean[0];
        float y = somNode[i][1] - mean[1];
        y -= x * coef[1];
        y *= y;
        scratch[i][2] = (y <= rmse) ? somNode[i][2] : 0;
    }

    // compute final linear fit
    fitLinear((float *) scratch, SOM_NODE_CNT, coef + 1, mean);

    // compute residual error
    rmse = 0;
    for(int i = 0; i < SOM_NODE_CNT; i++) {
        float x = scratch[i][0] - mean[0];
        float y = scratch[i][1] - mean[1];
        y -= x * coef[1];
        rmse += y * y * scratch[i][2];
    }
    tcmpRmse = sqrtf(rmse / mean[2]);

    // quality check fit
    if(tcmpRmse <= REG_MIN_RMSE) {
        // update means
        tcmpMean[0] = mean[0];
        tcmpMean[1] = mean[1];
        tcmpMean[2] = mean[2];
        // update coefficients
        tcmpCoeff[0] = coef[1];
        tcmpCoeff[1] = 0;
        tcmpCoeff[2] = 0;
    }
}

static float tcmpEstimate(const float temp) {
    float x = temp - tcmpMean[0];
    float y = tcmpMean[1];
    y += x * tcmpCoeff[0];
    y += x * x * tcmpCoeff[1];
    y += x * x * x * tcmpCoeff[2];
    return y;
}

unsigned statusSom(char *buffer) {
    char *end = buffer;

    // export in C-style array syntax
    end = append(end, "float som[16][3] = {\n");
    for(int i = 0; i < SOM_NODE_CNT; i++) {
        end += fmtFloat(somNode[i][0], 0, -1, end);
        end = append(end, ", ");
        end += fmtFloat(somNode[i][1] * 1e6f, 0, -1, end);
        end = append(end, ", ");
        end += fmtFloat(somNode[i][2], 0, -1, end);
        if(i < SOM_NODE_CNT - 1)
            *(end++) = ',';
        *(end++) = '\n';
    }
    end = append(end, "};\n");

    return end - buffer;
}

__attribute__((optimize(3)))
static void fitLinear(const float * const data, const int cnt, float *coef, float *mean) {
    // compute means
    mean[0] = 0;
    mean[1] = 0;
    mean[2] = 0;
    for(int i = 0; i < cnt; i++) {
        const float *row = data + (i * 3);
        mean[0] += row[0] * row[2];
        mean[1] += row[1] * row[2];
        mean[2] += row[2];
    }
    mean[0] /= mean[2];
    mean[1] /= mean[2];

    float xx = 0, xy = 0;
    for(int i = 0; i < cnt; i++) {
        const float *row = data + (i * 3);
        float x = row[0] - mean[0];
        float y = row[1] - mean[1];

        xx += x * x * row[2];
        xy += x * y * row[2];
    }
    coef[0] = xy / xx;
}
