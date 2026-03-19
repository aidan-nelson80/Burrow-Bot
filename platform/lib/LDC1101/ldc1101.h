#ifndef LDC1101_H
#define LDC1101_H

#include <stdint.h>
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Rp_ohms;
    float L_uH;
    uint32_t timestamp_ms;
} ldc1101_measurement_t;


/* Initialize SPI and verify chip connection */
void ldc1101_init(void);


/* Configure LDC1101 registers based on sensor parameters */
void ldc1101_configure(
    float L_h,
    float C_sensor,
    float Q);


/* Read measurement and convert to physical units */
ldc1101_measurement_t ldc1101_read(
    float C_sensor);

#ifdef __cplusplus
}
#endif

#endif