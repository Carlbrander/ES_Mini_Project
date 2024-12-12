/**
 * @file application.h
 * @brief Mini Project
 * @author Philipp Schilk, 2023, changed from Carl 
 */
#ifndef DSP_H_
#define DSP_H_

#include <stdint.h>
#include <stdio.h>

void task(void);

void dump_waveform(int32_t *buf, size_t len);
void dump_fft_mag(float *buf, size_t len, uint32_t max_idx, uint32_t fs);

#endif /* DSP_H_ */
