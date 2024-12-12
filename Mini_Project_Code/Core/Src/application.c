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

//this one is a great trade-off between number of samples and expressivity(?) of the camptured sound. And it fits into RAM
#define INPUT_SIZE 1024//4096//2048//8192//16384
#define FS 16447
//This is needed to normalize the microphone data
#define MIC_SCALE_FACTOR (1.0f / INT32_MAX)

//INT32 buffer for the microphone output
int32_t mic_buffer_raw[INPUT_SIZE] = {0};
//Float buffer for the FFT
float mic_buffer[INPUT_SIZE] = {0};



//This global variable is used to store the detected sound classification.

//0: No intrusion detected
//1: Intrusion detected: Glass break
//2: Intrusion detected: foot steps
//3: Intrusion detected: Voices
//4: Intrusion detected: Other
int16_t classification = 0;

//Counters for majority voting system
int16_t glass_break_counter = 0;
int16_t foot_steps_counter = 0;
int16_t voices_counter = 0;
int16_t mosquito_counter = 0;
int16_t no_intrusion_detected_counter = 0;

//variables for the time per iteration of the task
float total_time_for_recording_data = 0;
float total_time_for_fft = 0;
float total_time_for_rms = 0;
float total_time_for_classification = 0;
float total_time_voting = 0;
float total_time_active_task = 0;
float total_time_for_sleep = 0;


//TIM handle for the timer used for the sleep function
extern TIM_HandleTypeDef htim2;

//Create a new arm_rfft_fast_instance_f32 and initialise it.
arm_rfft_fast_instance_f32 S;



// === Function prototypes ===
void print_sampling_frequency();
void Get_microphone_data(DFSDM_Filter_HandleTypeDef *hdfsdm1_filter0);
float* DSP_FFT(arm_rfft_fast_instance_f32 *S);
int16_t sound_classification(float *fft_results, float rms);
void Sleep_For_2_Seconds(void);
float calculate_rms(float *fft_results, size_t len);



// === Main task ===
void task(void) {

  // Initialize the CMSIS DSP FFT:
  if (arm_rfft_fast_init_f32(&S, INPUT_SIZE) != ARM_MATH_SUCCESS) {
        printf("FFT initialization failed.\r\n");
        Error_Handler();
    }
  
  // Enable the DWT cycle counter:
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Start the main loop:
  while (1) {
    uint32_t start_active_task = DWT->CYCCNT;
    printf("\r\n\r\n");
    // Read a fraction of the complete 1 second each iteration and do classification on it:
    for (int i= 0; i < (FS / INPUT_SIZE); i++) {


    //##### RECORDING DATA #####

    //measure start time
    uint32_t start = DWT->CYCCNT;
    Get_microphone_data(&hdfsdm1_filter0);
    uint32_t stop = DWT->CYCCNT;
    // Calculate the number of DWT cycles the DFSDM took:
    uint32_t duration = stop - start;
    //add the time to the total time
    total_time_for_recording_data += (float)duration;



    //##### FFT #####

    // Do DSP FFT of the microphone data from the 'mic_buffer' array.
    uint32_t start_fft = DWT->CYCCNT;
    float *fft_results = DSP_FFT(&S);
    uint32_t stop_fft = DWT->CYCCNT;
    // Calculate the number of DWT cycles the FFT took:
    uint32_t duration_fft = stop_fft - start_fft;
    //add the time to the total time
    total_time_for_fft += (float)duration_fft;



    //##### RMS #####

    uint32_t start_rms = DWT->CYCCNT;
    float rms = calculate_rms(fft_results, INPUT_SIZE / 2);
    uint32_t stop_rms = DWT->CYCCNT;
    // Calculate the number of DWT cycles the RMS took:
    uint32_t duration_rms = stop_rms - start_rms;
    //add the time to the total time
    total_time_for_rms += (float)duration_rms;




    //##### CLASSIFICATION #####

    // Do sound classification based on the FFT results.
    uint32_t start_classification = DWT->CYCCNT;
    classification = sound_classification(fft_results, rms);
    uint32_t stop_classification = DWT->CYCCNT;
    // Calculate the number of DWT cycles the classification took:
    uint32_t duration_classification = stop_classification - start_classification;
    //add the time to the total time
    total_time_for_classification += (float)duration_classification;



    //##### VOTING #####

      uint32_t start_voting = DWT->CYCCNT;
      switch (classification) {
        case 0:
          no_intrusion_detected_counter++;
          break;
        case 1:
          glass_break_counter++;
          break;
        case 2:
          foot_steps_counter++;
          break;
        case 3:
          voices_counter++;
          break;
        case 4:
          mosquito_counter++;
          break;
        default:
          break;
      }
      uint32_t stop_voting = DWT->CYCCNT;

      // Calculate the number of DWT cycles the voting took:
      uint32_t duration_voting = stop_voting - start_voting;
      //add the time to the total time
      total_time_voting += (float)duration_voting;
    }

    uint32_t start_voting = DWT->CYCCNT;
    //weight no intrusion counter with less weight so quick changes are also detected
    no_intrusion_detected_counter = no_intrusion_detected_counter / 5;


    //print all counters for debugging
    //printf("No intrusion detected counter: %d\r\n", no_intrusion_detected_counter);
    //printf("Glass break counter: %d\r\n", glass_break_counter);
    //printf("Foot steps counter: %d\r\n", foot_steps_counter);
    //printf("Voices counter: %d\r\n", voices_counter);
    //printf("Mosquito counter: %d\r\n", mosquito_counter);

    
    
    // Majority voting system (check which counter has the highest value)
    if (no_intrusion_detected_counter > glass_break_counter && no_intrusion_detected_counter > foot_steps_counter && no_intrusion_detected_counter > voices_counter && no_intrusion_detected_counter > mosquito_counter) {
      printf("No intrusion detected\r\n");
    } 
    else if (glass_break_counter > no_intrusion_detected_counter && glass_break_counter > foot_steps_counter && glass_break_counter > voices_counter && glass_break_counter > mosquito_counter) {
      printf("Intrusion detected: Glass break\r\n");
    } 
    else if (foot_steps_counter > no_intrusion_detected_counter && foot_steps_counter > glass_break_counter && foot_steps_counter > voices_counter && foot_steps_counter > mosquito_counter) {
      printf("Intrusion detected: Foot steps\r\n");
    } 
    else if (voices_counter > no_intrusion_detected_counter && voices_counter > glass_break_counter && voices_counter > foot_steps_counter && voices_counter > mosquito_counter) {
      printf("Intrusion detected: Voices\r\n");
    }
    else if (mosquito_counter > no_intrusion_detected_counter && mosquito_counter > glass_break_counter && mosquito_counter > foot_steps_counter && mosquito_counter > voices_counter) {
      printf("IT's A MOSQUITO!!! KILL IT BEFORE IT LAYS EGGS!!!\r\n");

    } else {
      //This case happens if there is a draw between the counters in this case we establish a hierarchy
      //1. Glass break
      //2. Foot steps
      //3. Voices
      //4. No Intrusion detected
      //5. Mosquito
      if (glass_break_counter >= foot_steps_counter && glass_break_counter >= voices_counter && glass_break_counter >= no_intrusion_detected_counter) {
        printf("Intrusion detected: Glass break\r\n");
      } 
      else if (foot_steps_counter >= voices_counter && foot_steps_counter >= no_intrusion_detected_counter) {
        printf("Intrusion detected: Foot steps\r\n");
      } 
      else if (voices_counter >= no_intrusion_detected_counter) {
        printf("Intrusion detected: Voices\r\n");
      } 
      else if (mosquito_counter >= no_intrusion_detected_counter) {
        printf("IT's A MOSQUITO!!! KILL IT BEFORE IT LAYS EGGS!!!\r\n");
      }
      else {
        printf("No intrusion detected\r\n");
      }
      
    }

    uint32_t stop_voting = DWT->CYCCNT;
    // Calculate the number of DWT cycles the voting took:
    uint32_t duration_voting = stop_voting - start_voting;
    //add the time to the total time
    total_time_voting += (float)duration_voting;
      

    // Reset counters
    glass_break_counter = 0;
    foot_steps_counter = 0;
    voices_counter = 0;
    mosquito_counter = 0;
    no_intrusion_detected_counter = 0;


    uint32_t stop_active_task = DWT->CYCCNT;
    //Calculate the total time for the active task
    total_time_active_task = (float)(stop_active_task - start_active_task);


    //Print statistics

    //Print the clock frequency
    printf("Clock frequency: %.2f MHz\r\n", HAL_RCC_GetHCLKFreq() / 1e6);
    // Print the sampling frequency of the microphone
    print_sampling_frequency();
    //Print the total time for each task
    printf("Total time for recording data: %f cycles\r\n", total_time_for_recording_data);
    printf("Or in seconds: %f\r\n", total_time_for_recording_data / 80e6);

    printf("Total time for FFT: %f cycles\r\n", total_time_for_fft);
    printf("Or in seconds: %f\r\n", total_time_for_fft / 80e6);

    printf("Total time for RMS: %f cycles\r\n", total_time_for_rms);
    printf("Or in seconds: %f\r\n", total_time_for_rms / 80e6);

    printf("Total time for classification: %f cycles\r\n", total_time_for_classification);
    printf("Or in seconds: %f\r\n", total_time_for_classification / 80e6);

    printf("Total time for voting: %f cycles\r\n", total_time_voting);
    printf("Or in seconds: %f\r\n", total_time_voting / 80e6);

    //Print the total time for the active task
    printf("Total time for active task: %f cycles\r\n", total_time_active_task);
    printf("Or in seconds: %f\r\n", total_time_active_task / 80e6);

    
    //reset total time for recording data
    total_time_for_recording_data = 0;
    total_time_for_fft = 0;
    total_time_for_rms = 0;
    total_time_for_classification = 0;
    total_time_voting = 0;


    uint32_t start_sleep = DWT->CYCCNT;
    // Sleep for 2 seconds (puts Microcontroller into sleep mode with a 2-second timer interrupt)
    Sleep_For_2_Seconds();
    uint32_t stop_sleep = DWT->CYCCNT;

    // Calculate the number of DWT cycles the sleep took:
    uint32_t duration_sleep = stop_sleep - start_sleep;

    //add the time to the total time
    total_time_for_sleep += (float)duration_sleep;
    printf("Total time for sleep task: %f cycles\r\n", total_time_for_sleep);
    printf("Or in seconds: %f\r\n", total_time_for_sleep / 80e6);
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
  if (HAL_DFSDM_FilterRegularStart_DMA(hdfsdm1_filter0, mic_buffer_raw, INPUT_SIZE) != HAL_OK) {
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
  // Scale and convert data from raw to float
  for (int i = 0; i < INPUT_SIZE; i++) {
    mic_buffer[i] = (float)mic_buffer_raw[i] * MIC_SCALE_FACTOR;
  }
}


void print_sampling_frequency() {
    float system_clock = 80e6; // 80 MHz
    float output_clock_divider = 32;
    float oversampling_ratio = 38;
    float int_oversampling_ratio = 4;

    float base_clock_frequency = system_clock / output_clock_divider;
    float sampling_frequency = base_clock_frequency / (oversampling_ratio * int_oversampling_ratio);


    //printf("System Clock: %.2f MHz\n", system_clock / 1e6);
    //printf("Output Clock Divider: %.2f\n", output_clock_divider);
    //printf("Oversampling Ratio: %.2f\n", oversampling_ratio);
    //printf("Base Clock Frequency: %.2f MHz\n", base_clock_frequency / 1e6);
    printf("Calculated Sampling Frequency of Microphone: %.2f kHz\n", sampling_frequency / 1e3);
}


float* DSP_FFT(arm_rfft_fast_instance_f32 *S) {
    // === Find dominant frequency using CMSIS DSP: ============================
    
    // The real-fft of INPUT_SIZE real samples will produce INPUT_SIZE/2 complex
    // values, which require INPUT_SIZE floats to store:
    //static float FFT_Buffer[INPUT_SIZE] = {0};

    // Save time when RFFT started:
    uint32_t cmsis_fft_start = DWT->CYCCNT;

    //Compute the RFFT of 'IN_buffer' , and store the result in 'FFT_Buffer'
    arm_rfft_fast_f32(S, mic_buffer, mic_buffer, 0); //$ SOL

    // Save time when RFFT finished:
    uint32_t cmsis_fft_stop = DWT->CYCCNT;

    // Calculate the number of DWT cycles the KISS RFFT took:
    uint32_t duration_cmsis = cmsis_fft_stop - cmsis_fft_start;

    //Convert complex samples to real samples by taking the magnitude:
    
    // Note that the FFT_Buffer only contains INPUT_SIZE/2 complex values, which
    // requires INPUT_SIZE floats.
    arm_cmplx_mag_f32(mic_buffer, mic_buffer, INPUT_SIZE / 2);

    //remove the lowest 20Hz
    for (int i = 0; i < 20 * INPUT_SIZE / FS; i++) {
      mic_buffer[i] = 0;
    }

    return mic_buffer;

}




int16_t sound_classification(float *fft_results, float rms) {
    // Process the FFT results and classify the sound into different categories.
    // The classification is based on the dominant frequency of the sound.

    // Find the maximum value in the FFT results:
    float max_value = 0;
    int max_index = 0;

    //remove  (all frequencies under 20Hz)
    for (int i = 0; i < 20 * INPUT_SIZE / FS; i++) {
      fft_results[i] = 0;
    }

    for (int i = 0; i < INPUT_SIZE / 2; i++) {
      if (fft_results[i] > max_value) {
        max_value = fft_results[i];
        max_index = i;
      }
    }

    int32_t dominant_frequency = max_index * FS / INPUT_SIZE;

    //Print to dominant frequency
    //printf("Dominant frequency: %d Hz\r\n", dominant_frequency);
    // Classify the sound based on the dominant frequency:

    // If the RMS value is below a certain threshold, classify the sound as "No intrusion detected".
    if (rms < 0.03) {
      classification = 0;
      return classification;
    }
    // Background: <20Hz
    if (dominant_frequency < 20) {
      classification = 0;
      return classification;
    }

    // Glass break: 1700Hz - 7kHz
    if (dominant_frequency >= 1700 && dominant_frequency <= 20000) {
      classification = 1;
      return classification;
    }

    // Just a Mosquito: 1700Hz - 2800Hz
    if (dominant_frequency >= 1700 && dominant_frequency <= 2800) {
      classification = 4;
      return classification;
    }

    // Foot steps: 800Hz - 1700Hz
    if (dominant_frequency >= 800 && dominant_frequency < 1700) {
      classification = 2;
      return classification;
    }

    // Voices: 20Hz - 800Hz
    if (dominant_frequency >= 20 && dominant_frequency < 800) {
      classification = 3;
      return classification;
    }

}



float calculate_rms(float *fft_results, size_t len) {
    // Calculate the root mean square (RMS) value of the FFT results.
    // The RMS value is used to determine if there is a sound present.
    float sum = 0;
    for (int i = 0; i < len; i++) {
      sum += fft_results[i] * fft_results[i];
    }
    float rms = sqrt(sum / len);
    return rms;
}


void Sleep_For_2_Seconds(void) {
 
    // Start TIM2 in interrupt mode
    HAL_TIM_Base_Start_IT(&htim2);

    // Enter sleep mode; the Microcontroller will wake up on any interrupt
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();

    // Execution resumes here after waking up
    // Stop TIM2 to prevent further interrupts
    HAL_TIM_Base_Stop_IT(&htim2);
}


// Callback function called when TIM2 update interrupt occurs
// This needs to be here for  the timer to function correctly
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // Timer overflowed
    }
}