/**
 * @file application.c
 * @brief Mini Project from Carl
 * @author Philipp Schilk, 2023, changed from Carl
 */
#include "application.h"
#include "arm_math.h"
#include "kiss_fftr.h"
#include "main.h"
#include <inttypes.h>
#include <stdio.h>

#define INPUT_SIZE 2048
#define FS         16300

int32_t mic_buffer[INPUT_SIZE] = {0};


//void Get_microphone_data(DFSDM_Filter_HandleTypeDef hdfsdm1_filter0);





// === Main task ===
void task(void) {

  //Create a new arm_rfft_fast_instance_f32 and initialise it.
  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, INPUT_SIZE);
  

  kiss_fftr_cfg cfg = kiss_fftr_alloc(INPUT_SIZE, 0, 0, 0);

  // Enable the DWT cycle counter:
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  while (1) {

    printf("\r\n\r\n\r\n\r\n\r\n\r\n");
    
    // === Read data from the microphone and store it in the 'mic_buffer' array: ===
    Get_microphone_data(&hdfsdm1_filter0);
    // === Find dominant frequency using CMSIS DSP: =============================
    Find_maximum_frequency(S);

    
    // === Find dominant frequency using KISS DSP: =============================

    // Re-copy buffer since CMSIS modifies the input buffer during computation:
    kiss_fft_cpx FFT_KISS_Buffer[(INPUT_SIZE / 2) + 1] = {0};
    for (int i = 0; i < INPUT_SIZE; i++) {
      IN_Buffer[i] = (float)mic_buffer[i];
    }

    // Hold on to DWT timer value when the KISS RFFT was started:
    uint32_t kiss_fft_start = DWT->CYCCNT; 

    // Perform DFT:
    kiss_fftr(cfg, IN_Buffer, FFT_KISS_Buffer);

    // Grab DWT timer value after the KISS RFFT finished:
    uint32_t kiss_fft_stop = DWT->CYCCNT; 

    // Calculate the number of DWT cycles the KISS RFFT took:
    uint32_t duration_kiss = kiss_fft_stop - kiss_fft_start; 

    // Calculate the magnitude of each value:
    for (int i = 0; i < (INPUT_SIZE / 2) + 1; i++) {
      kiss_fft_cpx val = FFT_KISS_Buffer[i];
      FFT_KISS_Buffer[i].r = sqrt(val.i * val.i + val.r * val.r);
      FFT_KISS_Buffer[i].i = 0;
    }

    // Find the largest, skipping DC offset
    uint32_t maxindex_kiss = 1;
    float maxval_kiss = FFT_KISS_Buffer[1].r;
    for (int i = 2; i < (INPUT_SIZE / 2) + 1; i++) {
      if (FFT_KISS_Buffer[i].r > maxval_kiss) {
        maxval_kiss = FFT_KISS_Buffer[i].r;
        maxindex_kiss = i;
      }
    }

    // Convert index to a frequency:
    int f_max_kiss = (int)(maxindex_kiss)*res;

    printf("Frequency [KISS]:  %10i     Hz\r\n", f_max_kiss);
    printf("Duration  [KISS]:  %10" PRIu32 " Cycles\r\n", duration_kiss);

    HAL_Delay(2000);
  }
}



//Read data from the microphone and store it in the 'mic_buffer' array.
void Get_microphone_data(DFSDM_Filter_HandleTypeDef *hdfsdm1_filter0) {
  // ==== Acquire microphone data: ===========================================

  mic_dma_finished_flag = 0;
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, mic_buffer, INPUT_SIZE) != HAL_OK) {
    printf("Failed to start DFSDM!\r\n");
    Error_Handler();
  }
  while (!mic_dma_finished_flag) {
  }
  if (HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0) != HAL_OK) {
    printf("Failed to start DFSDM!\r\n");
    Error_Handler();
  }

}


void Find_maximum_frequency(S) {
    // === Find dominant frequency using CMSIS DSP: ============================

    float IN_Buffer[INPUT_SIZE] = {0};

    //Convert the microphone data from the 'mic_buffer' array to floats and store it in the 'IN_Buffer' array.
    for (int i = 0; i < INPUT_SIZE; i++) {
      IN_Buffer[i] = (float)mic_buffer[i];
    }

    // Set the correct size for the FFT buffer, which will hold the
    // INPUT_SIZE/2 complex values that the RFFT of the INPUT_SIZE real
    // samples will produce.
    

    // The real-fft of INPUT_SIZE real samples will produce INPUT_SIZE/2 complex
    // values, which require INPUT_SIZE floats to store:
    float FFT_Buffer[INPUT_SIZE] = {0};

    // Save time when RFFT started:
    uint32_t cmsis_fft_start = DWT->CYCCNT;

    //Compute the RFFT of 'IN_buffer' , and store the result in 'FFT_Buffer'
    arm_rfft_fast_f32(&S, IN_Buffer, FFT_Buffer, 0); //$ SOL

    // Save time when RFFT finished:
    uint32_t cmsis_fft_stop = DWT->CYCCNT;

    // Calculate the number of DWT cycles the KISS RFFT took:
    uint32_t duration_cmsis = cmsis_fft_stop - cmsis_fft_start;

    //Convert complex samples to real samples by taking the magnitude:
    
    // Note that the FFT_Buffer only contains INPUT_SIZE/2 complex values, which
    // requires INPUT_SIZE floats.
    arm_cmplx_mag_f32(FFT_Buffer, FFT_Buffer, INPUT_SIZE / 2);
    

    //Find the value and index of the largest value, ignoring the first (DC offset):
    uint32_t maxindex_cmsis;
    float maxval_cmsis;
    
    FFT_Buffer[0] = 0; // Ignore DC offset.
    // After computing the complex magnitude, the FFT_Buffer contains INPUT_SIZE/2
    // real values (which occupy the first INPUT_SIZE/2 floats).
    arm_max_f32(FFT_Buffer, (INPUT_SIZE / 2), &maxval_cmsis, &maxindex_cmsis);
    

    // Convert index to a frequency:
    float res = ((1.0 * FS) / INPUT_SIZE);
    int f_max_cmsis = (int)maxindex_cmsis * res;

    // Transmit result via UART:
    printf("Frequency [CMSIS]: %10i     Hz\r\n", f_max_cmsis);
    printf("Duration  [CMSIS]: %10" PRIu32 " Cycles\r\n", duration_cmsis);


}



//void dump_waveform(int32_t *buf, size_t len) {
//  printf("\r\nWAVEFORM:");
//  fflush(stdout);
//  for (size_t i = 0; i < len; i++) {
//    printf("%s%" PRIi32, i == 0 ? "" : ",", buf[i]);
//    fflush(stdout);
//  }
//  printf("\r\n");
//}
//
//void dump_fft_mag(float *buf, size_t len, uint32_t max_idx, uint32_t fs) {
//  printf("\r\nFFTMAG:%" PRIu32 ",%" PRIu32, max_idx, fs);
//  fflush(stdout);
//  for (size_t i = 0; i < len; i++) {
//    printf(",%f", buf[i]);
//    fflush(stdout);
//  }
//  printf("\r\n");
//}
