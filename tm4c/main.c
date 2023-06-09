/**
 * GPSDO with NTP and IEEE PTP
 * @author Robert J. Rouquette
 * @date 2022-04-13
 */

#include "hw/eeprom.h"
#include "hw/sys.h"
#include "lib/clk/clk.h"
#include "lib/delay.h"
#include "lib/led.h"
#include "lib/net.h"
#include "lib/ptp/ptp.h"
#include "lib/rand.h"
#include "lib/run.h"

#include "gitversion.h"
#include "snmp.h"
#include "status.h"

#define EEPROM_FORMAT (0x00000003)

static void EEPROM_init();

int main(void) {
    // enable FPU
    CPAC.CP10 = 3;
    CPAC.CP11 = 3;

    // initialize system clock
    CLK_initSys();
    // initialize task scheduler
    initScheduler();
    // initialize status LEDs
    LED_init();
    // initialize clock
    CLK_init();
    // initialize RNG
    RAND_init();
    // initialize EEPROM
    EEPROM_init();
    // initialize networking
    NET_init();
    PTP_init();
    SNMP_init();
    STATUS_init();

    // run task scheduler
    runScheduler();
}

void Fault_Hard() {
    faultBlink(2, 1);
}

void Fault_Memory() {
    faultBlink(2, 2);
}

void Fault_Bus() {
    faultBlink(2, 3);
}

void Fault_Usage() {
    faultBlink(2, 4);
}

static void EEPROM_init() {
    // enable EEPROM
    RCGCEEPROM.EN_EEPROM = 1;
    delay_us(1);
    EEPROM_wait();

    // verify eeprom format
    EEPROM_seek(0);
    uint32_t format = EEPROM_read();
    if(format != EEPROM_FORMAT) {
        // reformat EEPROM
        EEPROM_mass_erase();
        EEPROM_seek(0);
        EEPROM_write(EEPROM_FORMAT);
        EEPROM_write(VERSION_FW);
    }
}
