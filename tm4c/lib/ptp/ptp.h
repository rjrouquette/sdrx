//
// Created by robert on 3/31/23.
//

#ifndef GPSDO_PTP_H
#define GPSDO_PTP_H

void PTP_init();

/**
 * Write current status of the PTP engine to a buffer
 * @param buffer destination for status information
 * @return number of bytes written to buffer
 */
unsigned PTP_status(char *buffer);

#endif //GPSDO_PTP_H
