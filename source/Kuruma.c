/*
 * Copyright 2016-2026 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Kuruma.c
 * @brief   Main control logic for the autonomous line-tracking vehicle "Kuruma".
 * Implements sensor reading, PID/Bang-Bang control, and actuation.
 *
 * @project Mikrocontroller und Sensorik (WS 25/26)
 * @Uni     Technische Hochschule Deggendorf
 * @author  Hamid Hekmatnezhad
 * @date    January 2026
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_i2c.h"

// ----------------------- define -----------------------
#define LOGIC_RATE 50000 //us
#define LOOP_DELAY_MS 5

// LENKUNG
#define LEFT  100.0f
#define RIGHT 50.0f
#define CENTER_L 60.0f //50.0f
#define CENTER_R 90.0f//100.0f
#define CENTER 75.0f
#define KICK_DURATION_MS 150 // ms
#define DELAY_LENKUNG_TICKS 5000 // 50ms
#define LENKUNG_IDX_0_6 'r'
#define LENKUNG_IDX_9_15 'l'
#define STEERING_DELAY_STEPS  10  // find best setting
#define MAX_BUFFER_SIZE 100

// BLDC
#define ADC_BUFSIZE 1024
#define TIME_FOR_STARTUP_BIG_BLDC 100 // 100
#define TIME_FOR_STARTUP_SMALL_BLDC 20 // 20
#define DELAY_FOR_START_MOTOR 2000

#define START_DUTY_CYCLE_BLDC 50 // 28
#define RUN_DUTY_CYCLE_BLDC 30 //22 // power
#define SPEED_FOR_GEAR_1 5000 // speed
#define SPEED_FOR_GEAR_2 4000
#define SPEED_FOR_GEAR_3 3000
#define SPEED_FOR_GEAR_4 2000
#define GEAR_FOR_LENKUNG 1
#define GEAR_FOR_STRAIGHT 2
// BLDC TUNING PARAMETERS
#define BLDC_START_DELAY_TICKS   100000 // 1 sec delay
#define BLDC_INITIAL_PERIOD      35000 //20000  // Start slow/strong
#define BLDC_RAMP_THRESHOLD_LOW  10000 // 10000  // Speed up phase 1 limit
#define BLDC_RAMP_THRESHOLD_HIGH 2000 // 2000   // Speed up phase 2 limit // 3000
#define BLDC_RAMP_END_TARGET     2000   // Target to switch to RUNNING
#define BLDC_RUNNING_ACCEL_STEP  2      // How fast speed changes in RUNNING
#define BLDC_STOP_WAIT_TICKS     400000 // 4 sec safety wait
#define BLDC_SPEED_HYSTERESIS    10     // Sensitivity for speed change
#define BLDC_TIMER_PRESCALER     10     // Timer logic divider

// CAMERA
#define CAMERA_PIXEL_COUNT 128
#define WHITE 2
#define GRAY 1
#define BLACK 0

#define BIN_COUNT_CAMERA  16
#define BLOCK_SIZE_CAMERA 8
#define GRAY_THRESHOLD_ADD  300
#define BLACK_THRESHOLD 900

#define IDEAL_CENTER_DIFF           5
#define CAMERA_RIGHT_SIDE_START_IDX 7
#define CAMERA_RIGHT_SIDE_END_IDX   0
#define CAMERA_LEFT_SIDE_START_IDX  8
#define CAMERA_LEFT_SIDE_END_IDX    15

// LCD
#define I2C_MASTER_BASEaddr I2C0
#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_BAUDRATE 100000U
#define LCD_ADDR 0x27
#define PIN_RS    (1 << 0)
#define PIN_RW    (1 << 1)
#define PIN_EN    (1 << 2)
#define PIN_BKLT  (1 << 3)
#define LCD_REFRESH_RATE 500 // ms

// LED
#define BUFF_LENGTH (3*3*8*8) // (hhl hll)*farbe*matrix led
#define SIREN_SPEED 150 // ms


// ----------------------- state machine -----------------------
typedef enum { // servo
    STEER_IDLE,
    STEER_WAITING_STEP2
} SteeringState_t;

typedef enum { //brushless mc
    MOTOR_STATE_STOP,
    MOTOR_STATE_START_DELAY,
    MOTOR_STATE_RAMP_UP,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_STOPPING
} MotorState_t;

// ----------------------- variables -----------------------
volatile uint32_t ticks=0;
volatile uint32_t logic_timer = 0;
volatile uint32_t millis = 0;
volatile uint32_t last_lcd_update = 0;
volatile int last_valid_center = 7;

// lenkung
volatile float dutycycle_lenkung = 50.0f;
volatile int state_lenkung = 0;
SteeringState_t steer_state = STEER_IDLE;
uint32_t steer_timer_start = 0;
float pending_final_pwm = 0;
volatile int left_wall_idx = -1;
volatile int right_wall_idx = -1;
volatile int road_center = -1;
// buffer for steering with delay
volatile char steer_buffer[MAX_BUFFER_SIZE];
volatile int buffer_head = 0;
volatile int buffer_tail = 0;
volatile int current_buffer_items = 0;
volatile int steering_delay_steps_current = STEERING_DELAY_STEPS;  // find best setting
volatile int steering_delay_steps_new = STEERING_DELAY_STEPS;
volatile bool adc_lock = true;
volatile bool prev_adc_lock = true;
volatile bool first_time_delay_steering = true;
volatile char last_command = 'x';

// motor
volatile uint32_t ticks_bldc = 0;
volatile uint8_t current_phase_bldc = 0;
volatile uint8_t current_duty_cycle_bldc = START_DUTY_CYCLE_BLDC;
MotorState_t motor_state = MOTOR_STATE_STOP;
MotorState_t motor_state_prev = MOTOR_STATE_STOP;
uint32_t last_commutation_tick_bldc = 0;
uint32_t state_timer_tick_bldc = 0;
uint32_t commutation_period_bldc = 0;
uint32_t target_period_bldc = 2000;
uint8_t current_gear = 1;
uint8_t prev_gear = 1;
volatile bool start_motor = false;
volatile bool initial_start = false;


// ADC buffer
volatile int16_t  AT_NONCACHEABLE_SECTION_INIT(adc_value_buffer[ADC_BUFSIZE]);
volatile uint32_t AT_NONCACHEABLE_SECTION_INIT(adc_tick_buffer[ADC_BUFSIZE]);
// volatile uint16_t adc_bufpos = 0;
volatile bool adc_write_buf = true;

// Camera
AT_NONCACHEABLE_SECTION(uint16_t camera_adc_output[128]);
AT_NONCACHEABLE_SECTION(uint16_t vision_output[BIN_COUNT_CAMERA]);
volatile uint16_t current_pixel_nr=0;
volatile bool camera_frame_ready = false;
volatile int black_threshold_current = BLACK_THRESHOLD;
volatile int black_threshold_new = BLACK_THRESHOLD;
volatile int gray_threshold = BLACK_THRESHOLD + GRAY_THRESHOLD_ADD;

// Liquid LCD
static int lcd_update_timer = 0;
char st_str[16];

// LED
edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;
volatile bool isTransferCompleted  = false;
AT_NONCACHEABLE_SECTION(uint8_t led_red_blue[BUFF_LENGTH]);
AT_NONCACHEABLE_SECTION(uint8_t led_blue_red[BUFF_LENGTH]);
AT_NONCACHEABLE_SECTION(uint8_t led_off_1[BUFF_LENGTH]);
AT_NONCACHEABLE_SECTION(uint8_t led_off_2[BUFF_LENGTH]);
volatile uint32_t last_led_update = 0;
volatile bool toggle_led = false;
const uint8_t BIT_0[3] = {0x92, 0x49, 0x24}; // off - 0
const uint8_t BIT_1[3] = {0xDB, 0x6D, 0xB6}; // on - 255
// const uint8_t BIT_1[3] = {0xDA, 0x4D, 0x24}; // on - 200
volatile bool change_color = true; // true:red and blue,  false: cyan and blue

// Hardware Tables BLDC
uint32_t motor_enable_gpioa_mask[] = {
    BOARD_MOTOR_EN_A_PTA6_GPIO_PIN_MASK | BOARD_MOTOR_EN_C_PTA1_GPIO_PIN_MASK,
    BOARD_MOTOR_EN_A_PTA6_GPIO_PIN_MASK | BOARD_MOTOR_EN_B_PTA7_GPIO_PIN_MASK,
    BOARD_MOTOR_EN_C_PTA1_GPIO_PIN_MASK | BOARD_MOTOR_EN_B_PTA7_GPIO_PIN_MASK,
    BOARD_MOTOR_EN_C_PTA1_GPIO_PIN_MASK | BOARD_MOTOR_EN_A_PTA6_GPIO_PIN_MASK,
    BOARD_MOTOR_EN_B_PTA7_GPIO_PIN_MASK | BOARD_MOTOR_EN_A_PTA6_GPIO_PIN_MASK,
    BOARD_MOTOR_EN_B_PTA7_GPIO_PIN_MASK | BOARD_MOTOR_EN_C_PTA1_GPIO_PIN_MASK
};

uint32_t motor_direction_ftm_channel[] = {
    FTM0_MOTOR_A_FTM0_CH0_CHANNEL,
    FTM0_MOTOR_A_FTM0_CH0_CHANNEL,
    FTM0_MOTOR_C_FTM0_CH2_CHANNEL,
    FTM0_MOTOR_C_FTM0_CH2_CHANNEL,
    FTM0_MOTOR_B_FTM0_CH1_CHANNEL,
    FTM0_MOTOR_B_FTM0_CH1_CHANNEL
};

uint8_t back_emf_adc_channel_number[] = {
    ADC0_MOTOR_BEMF10_B_IRQ_CHANNEL, ADC0_MOTOR_BEMF10_C_IRQ_CHANNEL,
    ADC0_MOTOR_BEMF10_A_IRQ_CHANNEL, ADC0_MOTOR_BEMF10_B_IRQ_CHANNEL,
    ADC0_MOTOR_BEMF10_C_IRQ_CHANNEL, ADC0_MOTOR_BEMF10_A_IRQ_CHANNEL
};


void bldc_update_isr(void);
void lcd_send_nibble(uint8_t val, uint8_t mode);
uint8_t configure_off(void);


/* ----------------------------------------------------------------------------
   INTERRUPT HANDLERS
   ---------------------------------------------------------------------------- */

/**
 * @brief  System Tick Handler.
 * Increments global tick counters and handles millisecond tracking.
 * Calls the BLDC motor update routine if the motor is started.
 * @note   This ISR (Interrupt Service Runtime) runs every 10us (100kHz).
 */
void SysTick_Handler(void) {
    ticks++;
    if (ticks % 100 == 0) millis++; // for 1ms
    if (start_motor) bldc_update_isr();
}

/**
 * @brief  FTM1 Interrupt Handler (Camera Clock/SI).
 * Manages the timing signals (SI, CLK) for the linear camera sensor.
 * Triggers ADC reading for specific pixel windows.
 */
void FTM1_IRQHANDLER(void) {
  uint32_t intStatus;
  static int16_t parallax1_si_count=0;

  intStatus = FTM_GetStatusFlags(FTM1_PERIPHERAL);
  FTM_ClearStatusFlags(FTM1_PERIPHERAL, intStatus);

  parallax1_si_count++;

  if(parallax1_si_count==1){
      GPIO_PortClear(BOARD_PARALLAX1_SI_PTA9_GPIO, BOARD_PARALLAX1_SI_PTA9_PIN_MASK);
      current_pixel_nr=0;
  }

  if((parallax1_si_count >= 1)&&(parallax1_si_count <= 128)){
        ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP, &ADC1_channelsConfig[0]);
  }

  if(parallax1_si_count > 200){ // end of reading
      parallax1_si_count=0;
      GPIO_PortSet(BOARD_PARALLAX1_SI_PTA9_GPIO, BOARD_PARALLAX1_SI_PTA9_PIN_MASK);

      camera_frame_ready = true; // flag end
  }
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/**
 * @brief  ADC1 Interrupt Handler.
 * Reads the converted analog values from the camera sensor and stores
 * them in the camera_adc_output buffer.
 */
void ADC1_IRQHANDLER(void) {
  uint32_t result_values[2] = {0};
  for ( int i=0; i<2; i++){
      uint32_t status = ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, i);
      if ( status == kADC16_ChannelConversionDoneFlag){
          result_values[i] = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, i);
      }
  }

  if (current_pixel_nr < CAMERA_PIXEL_COUNT) {
      camera_adc_output[current_pixel_nr] = result_values[0];
      current_pixel_nr++;
  }
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/**
 * @brief  FTM3 Interrupt Handler.
 * Clears status flags for the FlexTimer module used for PWM generation.
 */
void FTM3_IRQHANDLER(void) {
      uint32_t intStatus;
      intStatus = FTM_GetStatusFlags(FTM3);
      FTM_ClearStatusFlags(FTM3, intStatus);
    #if defined __CORTEX_M && (__CORTEX_M == 4U)
      __DSB();
    #endif
}

/**
 * @brief  Port C Interrupt Handler.
 * Handles GPIO interrupts on Port C (e.g., button press).
 * Toggles the LED siren color mode.
 */
void GPIOC_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOC);

  /* Place your interrupt code here */
  change_color = !change_color; // change siren color

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOC, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/**
 * @brief  Port D Interrupt Handler.
 * Handles GPIO interrupts on Port D.
 * - Pin 13: Toggles Motor Start/Stop.
 * - Pin 12: Toggles Potentiometer ADC Lock/Unlock.
 */
void GPIOD_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOD);

  /* Place your interrupt code here */
  if (pin_flags & (1U << 13)) { //pin 13
      start_motor = !start_motor;
      if (start_motor) { // szenario for start motor
          // reset all state motor
          motor_state = MOTOR_STATE_STOP;
          state_timer_tick_bldc = 0;
          initial_start = false;
      }
      else { // szenario for stop motor
          motor_state = MOTOR_STATE_STOP;
          configure_off(); // pwm off
      }
  }

  else if (pin_flags & (1U << 12)) { // pin 12
      adc_lock = !adc_lock;
  }

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOD, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/**
 * @brief  BLDC Back-EMF ADC Interrupt Handler.
 * Reads the Back Electromotive Force (BEMF) voltage to detect
 * zero-crossing points for sensorless BLDC commutation.
 */
void motor_bemf_irq_handler(void) {
    uint32_t result_values[2] = { 0 };
    for (int i = 0; i < 2; i++) {
        uint32_t status = ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, i);
        if (status == kADC16_ChannelConversionDoneFlag) {
            result_values[i] = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, i);
            // if(adc_write_buf){
            //     adc_value_buffer[adc_bufpos] = result_values[0];
            //     adc_tick_buffer[adc_bufpos]= ticks;
            //     adc_bufpos = (adc_bufpos + 1) % ADC_BUFSIZE;
            // }
        }
    }
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP,
            &ADC0_channelsConfig[back_emf_adc_channel_number[current_phase_bldc]]);
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/* ----------------------------------------------------------------------------
   HELPER FUNCTIONS
   ---------------------------------------------------------------------------- */

/**
 * @brief  Maps a raw ADC value to a specific target range based on the goal.
 * @param  raw_val: The raw 12-bit ADC value (0-4095).
 * @param  goal: 0 for Steering Delay mapping, 1 for Black Threshold mapping.
 * @note   Updates global variables 'steering_delay_steps_new' or 'black_threshold_new'.
 */
void mapping_pototiometer(uint32_t raw_val, int goal){
    if (goal == 0)
        steering_delay_steps_new = (raw_val * 100) / 4096;
    else if (goal == 1)
        black_threshold_new = ((raw_val * 1300) / 4096) + 300;
}

/**
 * @brief  Safely reads the potentiometer for Steering Delay.
 * Disables interrupts during the read process to ensure atomic access
 * and prevents ADC conflicts.
 */
void adc_read_pot_safe_for_delay_steering(void) {
    uint32_t result = 0;

    // 1. define pototiometer channel
    adc16_channel_config_t potChannelConfigStruct;
    potChannelConfigStruct.channelNumber = 19; // channel number
    potChannelConfigStruct.enableInterruptOnConversionCompleted = false; // turn off interrupt
    potChannelConfigStruct.enableDifferentialConversion = false;

    // 2. Critical Section
    __disable_irq();

    // 3. start to read ADC
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, 0, &potChannelConfigStruct);

    // 4. Polling
    // wait for continuous flag == 1
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, 0)))
    {
    }

    // 5. read result
    result = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, 0);

    // 6. turn on interrupt
    __enable_irq();

    mapping_pototiometer(result, 0);
}

/**
 * @brief  Safely reads the potentiometer for Black Threshold.
 * Disables interrupts during the read process to ensure atomic access.
 */
void adc_read_pot_safe_for_black_threshold(void) {
    uint32_t result = 0;

    // 1. define pototiometer channel
    adc16_channel_config_t potChannelConfigStruct;
    potChannelConfigStruct.channelNumber = 0; // channel number
    potChannelConfigStruct.enableInterruptOnConversionCompleted = false; // turn off interrupt
    potChannelConfigStruct.enableDifferentialConversion = false;

    // 2. Critical Section
    __disable_irq();

    // 3. start to read ADC
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, 0, &potChannelConfigStruct);

    // 4. Polling
    // wait for continuous flag == 1
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, 0)))
    {
    }

    // 5. read result
    result = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, 0);

    // 6. turn on interrupt
    __enable_irq();

    mapping_pototiometer(result, 1);
}

/**
 * @brief  Blocking delay in microseconds.
 * @param  us: Time to wait in microseconds.
 */
void wait_us(uint32_t us) {
    uint32_t start_tick = ticks;
    uint32_t wait_ticks = us / 10;
    if (wait_ticks == 0) wait_ticks = 1;
    while (ticks - start_tick < wait_ticks);
}

/**
 * @brief  Blocking delay in milliseconds.
 * @param  ms: Time to wait in milliseconds.
 */
void sleep_ms(uint32_t ms) {
    uint32_t start = millis;
    while (millis - start < ms);
}

/**
 * @brief  Blocking delay (alternate implementation).
 * @param  microseconds: Time to wait in microseconds.
 */
void wait_microseconds(long microseconds) {
    uint32_t start_tick = ticks;
    while (ticks - start_tick < microseconds / 10);
}

/* ----------------------------------------------------------------------------
   LED DRIVER FUNCTIONS
   ---------------------------------------------------------------------------- */

/**
 * @brief  DSPI EDMA Callback.
 * Called when the DMA transfer for LED data is complete.
 */
void kuruma_spi0_edma_handler(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        ;
    }

    isTransferCompleted = true;
}

/**
 * @brief  Fills the LED buffer with RGB values for a specific range of LEDs.
 * @param  buffer: Pointer to the LED data buffer.
 * @param  start: Start index of the LED strip.
 * @param  end: End index of the LED strip.
 * @param  r: Red component (0 or 1 for simple on/off logic).
 * @param  g: Green component.
 * @param  b: Blue component.
 */
void set_led_range(uint8_t* buffer, int start, int end, int r, int g, int b) {
    for (int i = start; i <= end; i++) {
        int offset = i * 9;

        // Green
        const uint8_t* pG = g ? BIT_1 : BIT_0;
        buffer[offset + 0] = pG[0];
        buffer[offset + 1] = pG[1];
        buffer[offset + 2] = pG[2];

        // Red
        const uint8_t* pR = r ? BIT_1 : BIT_0;
        buffer[offset + 3] = pR[0];
        buffer[offset + 4] = pR[1];
        buffer[offset + 5] = pR[2];

        // Blue
        const uint8_t* pB = b ? BIT_1 : BIT_0;
        buffer[offset + 6] = pB[0];
        buffer[offset + 7] = pB[1];
        buffer[offset + 8] = pB[2];
    }
}

/**
 * @brief  Sends the constructed LED buffer to the WS2812 strip via SPI/DMA.
 * @param  data_buffer: Pointer to the buffer containing formatted bit patterns.
 */
void send_led_data(uint8_t* data_buffer) {
    dspi_transfer_t masterXfer;

    isTransferCompleted = false;
    masterXfer.txData      = data_buffer;
    masterXfer.rxData      = NULL;
    masterXfer.dataSize    = BUFF_LENGTH;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    if (kStatus_Success != DSPI_MasterTransferEDMA(SPI0_PERIPHERAL, &kuruma_spi0_edma_handle, &masterXfer))
    {
        ;
    }

    while (!isTransferCompleted)
    {
    }
}

/**
 * @brief  Periodic task to handle LED Siren effects.
 * Toggles between colors based on 'SIREN_SPEED' without blocking.
 */
void led_siren_task(void) {
    // Non-Blocking method
    if (millis - last_led_update > SIREN_SPEED) {
        last_led_update = millis;
        toggle_led = !toggle_led;

        if (change_color){
            if (toggle_led) send_led_data(led_red_blue);
            else send_led_data(led_blue_red);
        }
        else {
            if (toggle_led) send_led_data(led_off_1);
            else send_led_data(led_off_2);
        }
    }
}

/* ----------------------------------------------------------------------------
   LCD DRIVER FUNCTIONS
   ---------------------------------------------------------------------------- */

/**
 * @brief  Writes a single byte to the I2C bus.
 * @param  addr: I2C slave address.
 * @param  data: Data byte to send.
 * @return status_t: kStatus_Success if successful.
 */
status_t i2c_write_byte(uint8_t addr, uint8_t data) {
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &data;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    return I2C_MasterTransferBlocking(I2C_MASTER_BASEaddr, &masterXfer);
}

/**
 * @brief  Scans the I2C bus for active devices and prints addresses to debug console.
 * Useful for verifying LCD connection.
 */
void i2c_scan_bus(void) {
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_write_byte(addr, 0) == kStatus_Success) {
            found = true;
        }
    }
    if (!found) {
        ;
    }
}

/**
 * @brief  Sends a byte to the LCD in 4-bit mode (splits into two nibbles).
 * @param  val: The byte to send.
 * @param  mode: PIN_RS status (0 for Command, 1 for Data).
 */
void lcd_send_byte(uint8_t val, uint8_t mode) {
    uint8_t high = val & 0xF0;
    uint8_t low = (val << 4) & 0xF0;

    lcd_send_nibble(high, mode);
    lcd_send_nibble(low, mode);
}

/**
 * @brief  Sends a command instruction to the LCD.
 * @param  cmd: Command byte.
 */
void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, 0); // RS = 0
}

/**
 * @brief  Sends data (character) to the LCD to be displayed.
 * @param  data: Character ASCII value.
 */
void lcd_data(uint8_t data) {
    lcd_send_byte(data, PIN_RS); // RS = 1
}

/**
 * @brief  Initializes the LCD module.
 * Sets up 4-bit mode, display settings, and clears the screen.
 */
void lcd_init(void) {
    sleep_ms(50);

    // reset
    lcd_send_nibble(0x30, 0);
    sleep_ms(5);
    lcd_send_nibble(0x30, 0);
    sleep_ms(1);
    lcd_send_nibble(0x30, 0);
    sleep_ms(1);

    // set for 4 bits mode
    lcd_send_nibble(0x20, 0);
    sleep_ms(1);

    // main settings
    lcd_command(0x28); // Function set: 4-bit, 2 line, 5x8 dots
    lcd_command(0x0C); // Display on, Cursor off, Blink off
    lcd_command(0x06); // Entry mode: Increment cursor
    lcd_command(0x01); // Clear display
    sleep_ms(2);
}

/**
 * @brief  Prints a string to the LCD at the current cursor position.
 * @param  str: Null-terminated string.
 */
void lcd_print(char *str) {
    while (*str) {
        lcd_data((uint8_t)(*str));
        str++;
    }
}

/**
 * @brief  Moves the LCD cursor to a specific position.
 * @param  col: Column index (0-19).
 * @param  row: Row index (0-3).
 */
void lcd_set_cursor(uint8_t col, uint8_t row) {
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    lcd_command(0x80 | (col + row_offsets[row]));
}

/**
 * @brief  Sends 4 bits (nibble) to the LCD with the Enable pulse.
 * @param  val: Byte containing the 4 bits in the upper nibble.
 * @param  mode: RS pin status.
 */
void lcd_send_nibble(uint8_t val, uint8_t mode) {
    uint8_t data = val & 0xF0; // hold upper 4 bits
    uint8_t backlight = PIN_BKLT; // background light always on

    // Mode: RS=0 for Command, RS=1 for Data
    // RW bit is always 0 (since we don't include PIN_RW in the bitwise OR)

    // 1. data + light + mode
    i2c_write_byte(LCD_ADDR, data | mode | backlight);
    sleep_ms(1);

    // 2. pulse Enable HIGH
    i2c_write_byte(LCD_ADDR, data | mode | backlight | PIN_EN);
    sleep_ms(1);

    // 3. pulse Enable LOW
    i2c_write_byte(LCD_ADDR, data | mode | backlight);
    sleep_ms(1); // wait for LCD processing
}

/**
 * @brief  Updates the LCD content.
 * Refreshes Camera status, Motor status, Gear, and Settings.
 * Uses a non-blocking timer to limit refresh rate.
 */
void refresh_lcd(void) {
    if (millis - last_lcd_update > LCD_REFRESH_RATE) {
        last_lcd_update = millis;

        // line 1: camera
        // char cam_str[21] = "cam:                ";
        char cam_str[21] = "   :                ";
        if (state_lenkung == -1) {
            // cam_str[1] = 'L';     // <--
            // cam_str[0] = '<'; cam_str[2] = '-';
            cam_str[1] = '<';
            }
        else if (state_lenkung == 0) {
            // cam_str[1] = 'C'; // ^^^
            // cam_str[0] = '^'; cam_str[2] = '^';
            cam_str[1] = '^';
        }
        else if (state_lenkung == 1) {
            // cam_str[1] = 'R'; // -->
            // cam_str[0] = '-'; cam_str[2] = '>';
            cam_str[1] = '>';
        }
        if (left_wall_idx == -1) cam_str[0] = '-';
        else cam_str[0] = left_wall_idx + '0';
        if (right_wall_idx == -1) cam_str[2] = '-';
        else cam_str[2] = right_wall_idx + '0';

        int j = 0;
        for (int i=BIN_COUNT_CAMERA; i>0; i--) {
            if (vision_output[i] == BLACK) cam_str[j+4] = '#';
            else if (vision_output[i] == GRAY) cam_str[j+4] = 'x';
            else cam_str[j+4] = '.';
            j++;
        }
        cam_str[20] = '\0';
        lcd_set_cursor(0, 0);
        lcd_print(cam_str);

        // line 2: status Motor
        if (motor_state_prev != motor_state) {
            motor_state_prev = motor_state;
            char st_str[21] = "St. motor:          ";
            if (motor_state == MOTOR_STATE_STOP) {
                st_str[11] = 'S'; st_str[12] = 'T'; st_str[13] = 'O'; st_str[14] = 'P';
                st_str[15] = ' '; st_str[16] = ' '; st_str[17] = ' ';
            }
            else if (motor_state == MOTOR_STATE_START_DELAY){
                st_str[11] = 'S'; st_str[12] = 'T'; st_str[13] = 'A'; st_str[14] = 'R';
                st_str[15] = 'T'; st_str[16] = ' '; st_str[17] = ' ';
            }

            else if (motor_state == MOTOR_STATE_RAMP_UP){
                st_str[11] = 'R'; st_str[12] = 'A'; st_str[13] = 'M'; st_str[14] = 'P';
                st_str[15] = ' '; st_str[16] = 'U'; st_str[17] = 'P';
            }
            else if (motor_state == MOTOR_STATE_RUNNING){
                st_str[11] = 'R'; st_str[12] = 'U'; st_str[13] = 'N'; st_str[14] = 'N';
                st_str[15] = 'I'; st_str[16] = 'N'; st_str[17] = 'G';
            }
            st_str[20] = '\0';
            lcd_set_cursor(0, 1);
            lcd_print(st_str);
        }

        // line 3: gear and adc lck nlc
        if ((prev_gear != current_gear) || (prev_adc_lock != adc_lock)) {
            prev_gear = current_gear;
            prev_adc_lock = adc_lock;
            char ge_str[21] = "gear: 0, ADC: UNLOCK";
            ge_str[6] = current_gear + '0';
            if(adc_lock) {
                ge_str[14] = ' ';
                ge_str[15] = ' ';
            }
            ge_str[20] = '\0';
            lcd_set_cursor(0, 2);
            lcd_print(ge_str);
        }

        // line 4: set delay steering
        if (!adc_lock || first_time_delay_steering) {
            first_time_delay_steering = false;
            char dly_str[21] = "dly-st:00 blk-th:000";
            dly_str[7] = (steering_delay_steps_new / 10) + '0';
            dly_str[8] = (steering_delay_steps_new % 10) + '0';
            if (black_threshold_current >= 1000) // if it was a four-digit number
                dly_str[16] = (black_threshold_new / 1000) + '0';
            dly_str[17] = ((black_threshold_new / 100) % 10) + '0';
            dly_str[18] = ((black_threshold_new / 10) % 10) + '0';
            dly_str[19] = (black_threshold_new % 10) + '0';
            dly_str[20] = '\0';
            lcd_set_cursor(0, 3);
            lcd_print(dly_str);
            }
    }
}

/* ----------------------------------------------------------------------------
   BLDC MOTOR CONTROL FUNCTIONS
   ---------------------------------------------------------------------------- */

/**
 * @brief  Turns off all motor phases (Coasting).
 * @return uint8_t: 1 if successful.
 */
uint8_t configure_off(void) {
    uint8_t success = 0;
    GPIO_PortClear(BOARD_MOTOR_EN_A_PTA6_GPIO, BOARD_MOTOR_EN_A_PTA6_GPIO_PIN_MASK);
    GPIO_PortClear(BOARD_MOTOR_EN_B_PTA7_GPIO, BOARD_MOTOR_EN_B_PTA7_GPIO_PIN_MASK);
    GPIO_PortClear(BOARD_MOTOR_EN_C_PTA1_GPIO, BOARD_MOTOR_EN_C_PTA1_GPIO_PIN_MASK);

    if (FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, FTM0_MOTOR_A_FTM0_CH0_CHANNEL, kFTM_EdgeAlignedPwm, 0) == kStatus_Success) {
        if (FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, FTM0_MOTOR_B_FTM0_CH1_CHANNEL, kFTM_EdgeAlignedPwm, 0) == kStatus_Success) {
            if (FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, FTM0_MOTOR_C_FTM0_CH2_CHANNEL, kFTM_EdgeAlignedPwm, 0) == kStatus_Success) {
                success = 1;
            }
        }
    }
    return (success);
}

/**
 * @brief  Configures the FTM/PWM signals for a specific commutation step.
 * @param  phase: Current commutation step (0-5).
 * @param  duty_cycle: PWM duty cycle (speed).
 * @return uint8_t: 1 if successful.
 */
uint8_t reconfigure_for_phase(uint8_t phase, uint8_t duty_cycle) {
    uint8_t success = 0;
    if (configure_off()) {
        FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, motor_direction_ftm_channel[phase], kFTM_EdgeAlignedPwm, duty_cycle);
        FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);
        GPIO_PortSet(GPIOA, motor_enable_gpioa_mask[phase]);
    }
    success = 1;
    return success;
}

/**
 * @brief  Performs the commutation to the next motor phase.
 * @param  dutycycle_bldc: The target duty cycle for the next phase.
 */
void commute(int dutycycle_bldc) {
    current_phase_bldc = (current_phase_bldc + 1) % 6;
    current_duty_cycle_bldc = (uint8_t)(dutycycle_bldc);
    reconfigure_for_phase(current_phase_bldc, current_duty_cycle_bldc);
}

/**
 * @brief  Main BLDC Control Loop (called from SysTick).
 * Manages the State Machine:
 * - STOP: Motor off.
 * - START_DELAY: Initial wait.
 * - RAMP_UP: Open-loop acceleration sequence.
 * - RUNNING: Closed-loop control with dynamic speed adjustment.
 * - STOPPING: Safe shutdown sequence.
 * @note ISR: Interrupt Service Runtime
 */
void bldc_update_isr(void) {
    uint32_t current_tick = ticks;

    switch (motor_state) {

        case MOTOR_STATE_STOP:
            if (state_timer_tick_bldc == 0) {
                 configure_off();
                 state_timer_tick_bldc = current_tick;
            }
            break;

        case MOTOR_STATE_START_DELAY:
            if (current_tick - state_timer_tick_bldc > BLDC_START_DELAY_TICKS) {
                commutation_period_bldc = BLDC_INITIAL_PERIOD;
                target_period_bldc = BLDC_RAMP_END_TARGET;
                last_commutation_tick_bldc = current_tick;
                adc_write_buf = true;
                motor_state = MOTOR_STATE_RAMP_UP;
            }
            break;

        case MOTOR_STATE_RAMP_UP:
            if (current_tick - last_commutation_tick_bldc >= (commutation_period_bldc / BLDC_TIMER_PRESCALER)) {

                commute(START_DUTY_CYCLE_BLDC);
                last_commutation_tick_bldc = current_tick;

                // Log - nicht nutzlich
                // adc_value_buffer[adc_bufpos] = 6000;
                // adc_tick_buffer[adc_bufpos] = ticks;
                // adc_bufpos = (adc_bufpos + 1) % ADC_BUFSIZE;

                if (commutation_period_bldc > BLDC_RAMP_THRESHOLD_HIGH) {
                    if (commutation_period_bldc > BLDC_RAMP_THRESHOLD_LOW) commutation_period_bldc -= TIME_FOR_STARTUP_BIG_BLDC;
                    else commutation_period_bldc -= TIME_FOR_STARTUP_SMALL_BLDC;
                } else {
                    commutation_period_bldc = BLDC_RAMP_END_TARGET;
                    state_timer_tick_bldc = current_tick;
                    motor_state = MOTOR_STATE_RUNNING;
                }
            }
            break;

        case MOTOR_STATE_RUNNING:
            if (current_tick - last_commutation_tick_bldc >= (commutation_period_bldc / BLDC_TIMER_PRESCALER)) {

                // Dynamic Speed
                if (commutation_period_bldc > target_period_bldc) {
                    if (commutation_period_bldc - target_period_bldc > BLDC_SPEED_HYSTERESIS) commutation_period_bldc -= BLDC_RUNNING_ACCEL_STEP;
                    else commutation_period_bldc = target_period_bldc;

                } else if (commutation_period_bldc < target_period_bldc) {
                    if (target_period_bldc - commutation_period_bldc > BLDC_SPEED_HYSTERESIS) commutation_period_bldc += BLDC_RUNNING_ACCEL_STEP;
                    else commutation_period_bldc = target_period_bldc;
                }

                commute(RUN_DUTY_CYCLE_BLDC);
                last_commutation_tick_bldc = current_tick;

                // adc_value_buffer[adc_bufpos] = 6000;
                // adc_tick_buffer[adc_bufpos] = ticks;
                // adc_bufpos = (adc_bufpos + 1) % ADC_BUFSIZE;
            }
            break;

        case MOTOR_STATE_STOPPING:
            configure_off();
            adc_write_buf = false;
            if (current_tick - state_timer_tick_bldc > BLDC_STOP_WAIT_TICKS) {
                 state_timer_tick_bldc = 0;
                 motor_state = MOTOR_STATE_STOP;
            }
            break;
    }
}

/* ----------------------------------------------------------------------------
   STEERING & LOGIC FUNCTIONS
   ---------------------------------------------------------------------------- */

/**
 * @brief  Applies the calculated PWM to the steering servo.
 * Converts duty cycle (ms) to FTM register value.
 * @param  dc: Duty cycle in milliseconds * 10 (e.g., 70 for center).
 */
void apply_steering_pwm(float dc)
{
    if (dc != dutycycle_lenkung){
        dutycycle_lenkung = dc;
        uint32_t mod = FTM3->MOD;
        uint32_t cnv = (uint32_t)((dutycycle_lenkung / 1000.0f) * mod);

        FTM3->CONTROLS[6].CnV = cnv;
        FTM_SetSoftwareTrigger(FTM3, true);
    }
}

/**
 * @brief  Core Steering Logic.
 * Decides the steering PWM based on command ('l', 'r', 'c').
 * Implements "Kick-Start" logic to overcome static friction
 * by momentarily over-steering.
 * @param  lkg: Steering command char ('l'=Left, 'r'=Right, 'c'=Center).
 */
void lenkung(char lkg) {
    // kick-start protection
    if (steer_state == STEER_WAITING_STEP2 && lkg == last_command) {
        return;
    }

    if (lkg != last_command) {
        steer_state = STEER_IDLE;
        last_command = lkg;
    }

    if (lkg == 'l') { // Left
        if (steer_state == STEER_IDLE){
            if ((false) && (dutycycle_lenkung == CENTER_R || dutycycle_lenkung == CENTER)) { // need kick-start
                apply_steering_pwm(CENTER_L);
                pending_final_pwm = LEFT;
                steer_timer_start = ticks;
                steer_state = STEER_WAITING_STEP2;
            } else {
                apply_steering_pwm(LEFT);
            }
            state_lenkung = -1;
        }
    }
    else if (lkg == 'r') { // Right
        if (steer_state == STEER_IDLE) {
            if ((dutycycle_lenkung == CENTER_L || dutycycle_lenkung == CENTER) && (false)) { // need kick-start
                apply_steering_pwm(CENTER_R);
                pending_final_pwm = RIGHT;
                steer_timer_start = ticks;
                steer_state = STEER_WAITING_STEP2;
            } else {
                apply_steering_pwm(RIGHT);
            }
            state_lenkung = 1;
        }
    }
    else if (lkg == 'c') { // Center
        steer_state = STEER_IDLE;
        apply_steering_pwm(CENTER);
        // if (dutycycle_lenkung == LEFT) apply_steering_pwm(CENTER_L);
        // else if (dutycycle_lenkung == RIGHT) apply_steering_pwm(CENTER_R);
        // else apply_steering_pwm(CENTER);
        state_lenkung = 0;
    }
}

/**
 * @brief  Initializes/Resets the steering circular buffer.
 * Sets the buffer head and tail based on the current delay setting.
 */
void init_steering_buffer(void) {
    for (int i = 0; i < MAX_BUFFER_SIZE; i++) {
        steer_buffer[i] = CENTER; // init data
    }
    buffer_head = 0;
    // init tail
    buffer_tail = (MAX_BUFFER_SIZE - steering_delay_steps_current) % MAX_BUFFER_SIZE;
}

/**
 * @brief  Handles the Look-Ahead Delay for steering.
 * Stores the current decision in a circular buffer and retrieves
 * the delayed command to apply to the servo.
 * @param  direction: The calculated steering direction for the *current* frame.
 */
void delayed_lenkung(char direction) {

    // 1. save new steer in buffer (head pos.)
    steer_buffer[buffer_head] = direction;

    // 2. forward Head
    buffer_head = (buffer_head + 1) % MAX_BUFFER_SIZE;

    // 3. read prev. commond from buffer (tail pos.)
    char delayed_direction = steer_buffer[buffer_tail];

    // 3. forward tail
    buffer_tail = (buffer_tail + 1) % MAX_BUFFER_SIZE;

    // 5. send perv. commond to lenkung and then servo
    lenkung(delayed_direction);
}

/**
 * @brief  Steering Task Manager.
 * Manages the timing for the 2-step "Kick-Start" mechanism.
 * resets steering to the final position after the kick duration.
 */
void steering_task_manager(void) {
    if (steer_state == STEER_WAITING_STEP2) {
        if (ticks - steer_timer_start > DELAY_LENKUNG_TICKS) {
            apply_steering_pwm(pending_final_pwm);
            steer_state = STEER_IDLE; // free state
        }
    }
}

/**
 * @brief  Image Processing.
 * Analyzes the ADC array from the camera.
 * Classifies each bin as BLACK, GRAY, or WHITE based on thresholds.
 */
void find_line(void) {
    uint16_t min_val = 4096;
    int sum = 0;
    int idx = 0;

    for (int i = 0; i < 128; i+=8) {
        sum = 0;
        for (int j = i; j < BLOCK_SIZE_CAMERA+i; j++) {
            sum += camera_adc_output[j];
        }
        sum /= BLOCK_SIZE_CAMERA;

        if (sum <= black_threshold_current)
            vision_output[idx] = BLACK;
        else if (sum <= gray_threshold)
            vision_output[idx] = GRAY;
        else
            vision_output[idx] = WHITE;

        idx++;
    }
}


/**
 * @brief  Main Application Logic.
 * 1. Determines motor start timing.
 * 2. Finds left/right track walls from vision data.
 * 3. Calculates the road center.
 * 4. Decides steering direction and gear/speed based on curvature.
 */
void logic_task(void) {

    // start motor
    if (!initial_start && millis > DELAY_FOR_START_MOTOR && start_motor) {
        motor_state = MOTOR_STATE_START_DELAY;
        state_timer_tick_bldc = ticks;
        initial_start = true;
    }

    left_wall_idx = -1;
    right_wall_idx = -1;

    // 1. find right side black line (index: 0 - 7)
    for(int i=CAMERA_RIGHT_SIDE_START_IDX; i >= CAMERA_RIGHT_SIDE_END_IDX; i--) {
        if(vision_output[i] == BLACK) {
            right_wall_idx = i;
            break;
        }
    }

    // 2. find left side black line (index: 8 - 15)
    for(int i=CAMERA_LEFT_SIDE_START_IDX; i <= CAMERA_LEFT_SIDE_END_IDX; i++) {
        if(vision_output[i] == BLACK) {
            left_wall_idx = i;
            break;
        }
    }

    road_center = 7; // center

    if (0){
    road_center = -1;
    last_valid_center = 7;

    // calculate center
    if (left_wall_idx != -1 && right_wall_idx != -1) {
        road_center = (left_wall_idx + right_wall_idx) / 2;
        // last_valid_center = road_center; // no see
    }
    else if (left_wall_idx != -1) {
        // Only left blackline detected -> estimated center
        road_center = left_wall_idx - IDEAL_CENTER_DIFF;
    }
    else if (right_wall_idx != -1) {
        // Only right blackline detected -> estimated center
        road_center = right_wall_idx + IDEAL_CENTER_DIFF;
    }
    else {
        // No blacklines detected -> use memory (last valid)
        road_center = last_valid_center;
    }
    }

    // steering
    // road_center index: 7, 8
    char target_steering = 'c';

    if (road_center > 2 && road_center <= 6) { // find best range - hh
        target_steering = LENKUNG_IDX_0_6;
        current_gear = GEAR_FOR_LENKUNG;
        change_color = true;
    }
    else if (road_center < 13 && road_center >= 9) { // find best range - hh
        target_steering = LENKUNG_IDX_9_15;
        current_gear = GEAR_FOR_LENKUNG;
        change_color = true;
    }
    else {
        target_steering = 'c';
        current_gear = GEAR_FOR_STRAIGHT;
        change_color = false;
    }
    delayed_lenkung(target_steering);

    // set speed
    if (current_gear == 1) target_period_bldc = SPEED_FOR_GEAR_1;
    else if (current_gear == 2) target_period_bldc = SPEED_FOR_GEAR_2;
    else if (current_gear == 3) target_period_bldc = SPEED_FOR_GEAR_3;
    else if (current_gear == 4) target_period_bldc = SPEED_FOR_GEAR_4;
}

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    SysTick_Config(SystemCoreClock/100000U);
    __enable_irq();

    // init LCD
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(I2C_MASTER_BASEaddr, &masterConfig, I2C_MASTER_CLK_FREQ);
    i2c_scan_bus();
    lcd_init();

    // init steering - bufer for delay
    init_steering_buffer();

    // setup LED
    memset(led_red_blue, 0, BUFF_LENGTH);
    memset(led_blue_red, 0, BUFF_LENGTH);
    memset(led_off_1, 0, BUFF_LENGTH);
    memset(led_off_2, 0, BUFF_LENGTH);

    set_led_range(led_red_blue, 0, 31, 1, 0, 0); //red
    set_led_range(led_red_blue, 32, 63, 0, 0, 1); // blue
    set_led_range(led_blue_red, 0, 31, 0, 0, 1);  // blue
    set_led_range(led_blue_red, 32, 63, 1, 0, 0); // red

    // set_led_range(led_cyan_yellow, 0,31, 0, 1, 1); // cyan
    // set_led_range(led_cyan_yellow, 32, 63, 1, 1, 0); // yellow
    // set_led_range(led_yellow_cyan, 0, 31, 1, 1, 0); // yellow
    // set_led_range(led_yellow_cyan, 32, 63, 0, 1, 1); // cyan

    set_led_range(led_off_1, 0,31, 0, 0, 0);   // off
    set_led_range(led_off_1, 32, 63, 0, 0, 0); // off
    set_led_range(led_off_2, 0, 31, 0, 0, 0);  // off
    set_led_range(led_off_2, 32, 63, 0, 0, 0); // off

    // clean up camera pixel
    for(int i=0; i<CAMERA_PIXEL_COUNT; i++){
        camera_adc_output[i]=0;
    }

    bool START_MAIN = true;
    while(START_MAIN)
    {
        // step 1:
        find_line();

        // step 2:
        logic_task();
        steering_task_manager();

        // set new delay for steering and black threshold for camera
        if (!adc_lock){
            adc_read_pot_safe_for_delay_steering();
            adc_read_pot_safe_for_black_threshold();
        }
        if (adc_lock){
            if (steering_delay_steps_current != steering_delay_steps_new) {
                steering_delay_steps_current = steering_delay_steps_new;
                init_steering_buffer();
            }
            if (black_threshold_current != black_threshold_new) {
                black_threshold_current = black_threshold_new;
                gray_threshold = black_threshold_current + GRAY_THRESHOLD_ADD;
            }
        }

        // step 3
        refresh_lcd();
        led_siren_task();

        sleep_ms(LOOP_DELAY_MS);
    }

    int test_number = 3;
    while(!START_MAIN){

        if (test_number == 1) {
            apply_steering_pwm(30);
            sleep_ms(2000);

            apply_steering_pwm(80);
            sleep_ms(2000);

            apply_steering_pwm(120);
            sleep_ms(2000);

            apply_steering_pwm(80);
            sleep_ms(2000);

            apply_steering_pwm(30);
            sleep_ms(2000);
        }

        else if (test_number == 2){
            current_gear = SPEED_FOR_GEAR_1;
            sleep_ms(3000);

            current_gear = SPEED_FOR_GEAR_2;
            sleep_ms(3000);

            current_gear = SPEED_FOR_GEAR_3;
            sleep_ms(3000);

            current_gear = SPEED_FOR_GEAR_4;
            sleep_ms(3000);
        }

        else if (test_number == 3){
            apply_steering_pwm(50);
            sleep_ms(3000);
            apply_steering_pwm(100);
            sleep_ms(3000);
            apply_steering_pwm(75);
            sleep_ms(3000);
        }
    }

    return 0;
}
