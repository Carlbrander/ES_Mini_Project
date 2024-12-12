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

#define INPUT_SIZE 2048 //8192 //16384
#define FS         16447

int32_t mic_buffer[INPUT_SIZE] = {0};


//This global variable is used to store the detected sound classification.

//0: No intrusion detected
//1: Intrusion detected: Glass break
//2: Intrusion detected: foot steps
//3: Intrusion detected: Voices
//4: Intrusion detected: Other

int16_t classification = 0;




// === Function prototypes ===
void print_sampling_frequency();

void Get_microphone_data(DFSDM_Filter_HandleTypeDef *hdfsdm1_filter0);

float* DSP_FFT(arm_rfft_fast_instance_f32 *S);

int16_t sound_classification(float *fft_results);






// === Main task ===
void task(void) {

  //Create a new arm_rfft_fast_instance_f32 and initialise it.
  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, INPUT_SIZE);

  // Print the sampling frequency
  print_sampling_frequency();
  
  // Enable the DWT cycle counter:
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;




  while (1) {

    printf("\r\n\r\n\r\n\r\n\r\n\r\n");

    // Read 1 second of audio data from the microphone and store it in the 'mic_buffer' array.
    Get_microphone_data(&hdfsdm1_filter0);

    // Do DSP FFT of the microphone data from the 'mic_buffer' array.
    float *fft_results = DSP_FFT(&S);

    // Do sound classification based on the FFT results.
    classification = sound_classification(fft_results);

    // Print the classification result.
    switch (classification) {
      case 0:
        printf("No intrusion detected.\r\n");
        break;
      case 1:
        printf("Intrusion detected: Glass break.\r\n");
        break;
      case 2:
        printf("Intrusion detected: Foot steps.\r\n");
        break;
      case 3:
        printf("Intrusion detected: Voices.\r\n");
        break;
      case 4:
        printf("Intrusion detected: Other.\r\n");
        break;
      default:
        printf("Unknown classification.\r\n");
        break;
    }



    HAL_Delay(2000);
  }
}





//Read data from the microphone and store it in the 'mic_buffer' array.
void Get_microphone_data(DFSDM_Filter_HandleTypeDef *hdfsdm1_filter0) {
  // ==== Acquire microphone data: ===========================================



  //The DFSDM frequency is set in CubeMX to 80Mhz, so the clock frequency is 80Mhz.
  //There is a output clock divider of 32, so the actual clock frequency is 2.5Mhz.
  //The microphone is used with an oversampling rate of 38, so the actual sampling frequency is 2.5Mhz/38 = 65.789kHz.
  //Then there is an int_oversampling ratio of 4 which means that 4 samples are averaged to get one sample.
  //Therefore the final sampling frequency is 65.789kHz/4 = 16.447kHz.

  //Therefore to record for 1 second, we need to record 16.447kHz * 1s = 16'447 samples.
  //For ease of use I rounded this to the next power of 2, which is 2^14 = 16'384 samples.


  //count clock cycles used to record the data
  uint32_t start = DWT->CYCCNT;

  mic_dma_finished_flag = 0;
  if (HAL_DFSDM_FilterRegularStart_DMA(hdfsdm1_filter0, mic_buffer, INPUT_SIZE) != HAL_OK) {
    printf("Failed to start DFSDM!\r\n");
    Error_Handler();
  }
  while (!mic_dma_finished_flag) {
  }
  if (HAL_DFSDM_FilterRegularStop_DMA(hdfsdm1_filter0) != HAL_OK) {
    printf("Failed to start DFSDM!\r\n");
    Error_Handler();
  }

  uint32_t stop = DWT->CYCCNT;

  // Calculate the number of DWT cycles the DFSDM took:
  uint32_t duration = stop - start;

  printf("Microphone recording Duration in seconds: %f\r\n", (float)duration / 80e6);

}






void print_sampling_frequency() {
    float system_clock = 80e6; // 80 MHz
    float output_clock_divider = 32;
    float oversampling_ratio = 38;
    float int_oversampling_ratio = 4;

    float base_clock_frequency = system_clock / output_clock_divider;
    float sampling_frequency = base_clock_frequency / (oversampling_ratio * int_oversampling_ratio);


    printf("System Clock: %.2f MHz\n", system_clock / 1e6);
    printf("Output Clock Divider: %.2f\n", output_clock_divider);
    printf("Oversampling Ratio: %.2f\n", oversampling_ratio);
    printf("Base Clock Frequency: %.2f MHz\n", base_clock_frequency / 1e6);
    printf("Calculated Sampling Frequency: %.2f kHz\n", sampling_frequency / 1e3);
}






float* DSP_FFT(arm_rfft_fast_instance_f32 *S) {
    // === Find dominant frequency using CMSIS DSP: ============================
    printf("Starting CMSIS FFT...\r\n");
    static float IN_Buffer[INPUT_SIZE] = {0};

    //Convert the microphone data from the 'mic_buffer' array to floats and store it in the 'IN_Buffer' array.
    for (int i = 0; i < INPUT_SIZE; i++) {
    IN_Buffer[i] = (float)mic_buffer[i];
    }


    // The real-fft of INPUT_SIZE real samples will produce INPUT_SIZE/2 complex
    // values, which require INPUT_SIZE floats to store:
    static float FFT_Buffer[INPUT_SIZE] = {0};

    // Save time when RFFT started:
    uint32_t cmsis_fft_start = DWT->CYCCNT;

    //Compute the RFFT of 'IN_buffer' , and store the result in 'FFT_Buffer'
    arm_rfft_fast_f32(S, IN_Buffer, FFT_Buffer, 0); //$ SOL

    // Save time when RFFT finished:
    uint32_t cmsis_fft_stop = DWT->CYCCNT;

    // Calculate the number of DWT cycles the KISS RFFT took:
    uint32_t duration_cmsis = cmsis_fft_stop - cmsis_fft_start;

    printf("CMSIS FFT Duration in seconds: %f\r\n", (float) duration_cmsis / 80e6);

    //Convert complex samples to real samples by taking the magnitude:
    
    // Note that the FFT_Buffer only contains INPUT_SIZE/2 complex values, which
    // requires INPUT_SIZE floats.
    arm_cmplx_mag_f32(FFT_Buffer, FFT_Buffer, INPUT_SIZE / 2);

    return FFT_Buffer;

}




int16_t sound_classification(float *fft_results) {
    // Process the FFT results and classify the sound into different categories.
    // The classification is based on the dominant frequency of the sound.
    
    // Calculate the RMS value of the FFT results:
    float rms = 0;
    for (int i = 0; i < INPUT_SIZE / 2; i++) {
      rms += fft_results[i] * fft_results[i];
    }
    rms = sqrt(rms / (INPUT_SIZE / 2));

    // If the RMS value is below a certain threshold, classify the sound as "No intrusion detected".
    if (rms < 100) {
      classification = 0;
      return classification;
    }

    // Find the maximum value in the FFT results:
    float max_value = 0;
    int max_index = 0;

    //remove the DC offset like in the exercise
    fft_results[0] = 0;
    for (int i = 0; i < INPUT_SIZE / 2; i++) {
      if (fft_results[i] > max_value) {
        max_value = fft_results[i];
        max_index = i;
      }
    }

    // Classify the sound based on the dominant frequency:

    // Glass break: 2kHz - 5kHz
    if (max_index >= 2 * INPUT_SIZE / FS && max_index <= 5 * INPUT_SIZE / FS) {
      classification = 1;
      return classification;
    }

    // Foot steps: 100Hz - 500Hz
    if (max_index >= 100 * INPUT_SIZE / FS && max_index <= 500 * INPUT_SIZE / FS) {
      classification = 2;
      return classification;
    }

    // Voices: 100Hz - 2kHz
    if (max_index >= 100 * INPUT_SIZE / FS && max_index <= 2 * INPUT_SIZE / FS) {
      classification = 3;
      return classification;
    }

    // Other: Anything else
    classification = 4;
    return classification;
    

}