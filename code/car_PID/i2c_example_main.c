#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/pcnt_struct.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"


#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)
#define UART_TX_GPIO_NUM 26 // A0

#define UART_RX_GPIO_NUM 16 // A2
#define BUF_SIZE (1024)

#define GPIO_INPUT_IO_1       33

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1


#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      100
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    3.0F
#define PCNT_THRESH0_VAL    0.20F
#define PCNT_INPUT_SIG_IO   34  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=dista up, LOW=count down
#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

uint32_t count_drive;

// For wheels
#define DRIVE_MIN_PULSEWIDTH 900 //Minimum pulse width in microsecond
#define DRIVE_MAX_PULSEWIDTH 1700 //Maximum pulse width in microsecond
#define DRIVE_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define STEERING_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 100 //Maximum angle in degree upto which servo can rotate

static const adc_channel_t channel_right = ADC_CHANNEL_3;  //pin A3
static const adc_channel_t channel_left = ADC_CHANNEL_5;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_10;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static esp_adc_cal_characteristics_t *adc_chars_left;
static esp_adc_cal_characteristics_t *adc_chars_right;

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisamp

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDARLite_v4LED slave address
#define SLAVE_ADDR1                         0x62 // slave address

double dist_right;
double dist_left;
float speed;
int stop = 0;
int16_t distance;


int16_t count;

char myStart;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

int blackCount = 0;



#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_2_SEC  (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

// Default ID/color
#define ID 3
#define COLOR 'R'

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char) ID;
char myColor = (char) COLOR;
int len_out = 4;
bool button_flag = false;
// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
// static xQueueHandle gpio_evt_queue = NULL;
// static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "system";       // For debug logs

// Button interrupt handler -- add to queue
// static void IRAM_ATTR gpio_isr_handler(void* arg){
//   uint32_t gpio_num = (uint32_t) arg;
//   xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }

// ISR handler
// void IRAM_ATTR timer_group0_isr(void *para) {
//
//     // Prepare basic event data, aka set flag
//     timer_event_t evt;
//     evt.flag = 1;
//
//     // Yellow is shorter
//     if (myColor == 'G') {
//       timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
//     }
//     else {
//       timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     }
//
//     // Clear the interrupt, Timer 0 in group 0
//     TIMERG0.int_clr_timers.t0 = 1;
//
//     // After the alarm triggers, we need to re-enable it to trigger it next time
//     TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
//
//     // Send the event data back to the main program task
//     xQueueSendFromISR(timer_queue, &evt, NULL);
// }

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum
char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}
bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

// Init Functions //////////////////////////////////////////////////////////////
// RMT tx init
static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}


// Configure timer
// static void alarm_init() {
//     // Select and initialize basic parameters of the timer
//     timer_config_t config;
//     config.divider = TIMER_DIVIDER;
//     config.counter_dir = TIMER_COUNT_UP;
//     config.counter_en = TIMER_PAUSE;
//     config.alarm_en = TIMER_ALARM_EN;
//     config.intr_type = TIMER_INTR_LEVEL;
//     config.auto_reload = TEST_WITH_RELOAD;
//     timer_init(TIMER_GROUP_0, TIMER_0, &config);
//
//     // Timer's counter will initially start from value below
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
//
//     // Configure the alarm value and the interrupt on alarm
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
//         (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
//
//     // Start timer
//     timer_start(TIMER_GROUP_0, TIMER_0);
// }



////////////////////////////////////////////////////////////////////////////////

// Tasks //////////////////////////////////////////////////////////////////////

// Receives task -- looks for Start byte then stores received values
void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in >0) {
      printf("received\n");
      //if (data_in[0] == start) {
        if (checkCheckSum(data_in,len_out)) {
          ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
          myStart = data_in[2];
          printf("%c\n", myStart);
        }
      //}
    }
    else{
      //printf("Nothing received.\n");
    }
    vTaskDelay(5/ portTICK_PERIOD_MS);
  }
  free(data_in);
}




void distance_task(void){
    while(1){
        speed = ((float)(count / 6.0) * (.62)); //0.005; //(count / 6) * 0.62;
        printf("Speed is %.1fm/s\n", speed);

        vTaskDelay(1000/portTICK_RATE_MS);
        pcnt_counter_clear(PCNT_TEST_UNIT);
    }
}


static void led_init() {
    gpio_pad_select_gpio(13);

    gpio_set_direction(13, GPIO_MODE_OUTPUT);
}


// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}


int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}


// For wheels
static void mcpwm_example_gpio_initialize(void)
{
    gpio_pad_select_gpio(25);
    gpio_set_level(25,1);
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    //18
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 26);    //26
}

static uint32_t drive_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (DRIVE_MIN_PULSEWIDTH + (((DRIVE_MAX_PULSEWIDTH - DRIVE_MIN_PULSEWIDTH) * (degree_of_rotation)) / (DRIVE_MAX_DEGREE)));
    return cal_pulsewidth;
}

static uint32_t steering_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (degree_of_rotation)) / (STEERING_MAX_DEGREE)));
    return cal_pulsewidth;
}

void pwm_init() {
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}


void calibrateESC() {
    printf("Turn on in 3 seconds\n");
    gpio_set_level(13,1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler (3s)
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(3100 / portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
}

void drive_control(void *arg)
{
     uint32_t angle, count;
     printf("Driving!\n");

     printf("Forward...\n");
     int Kp = 1;
      int Ki = 1;
      int Kd = 1;
      float previous_error = 0.00;
      float integral = 0.00;
      float derivative;
      int dt  = 100;
      float setpoint = 0.5;
      // if (myStart == '1'){
      //   for (count = 1400; count < 1500; count += 1) {
      //   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
      //       vTaskDelay(100/portTICK_RATE_MS);
      //   }

      while(1){
        if (myStart == '1'){
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
        }

        if (stop == 1){
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1300);
        }
        else if (distance < 50 && distance > 35) {
          for (count = 1500; count > 1400 ; count -= 1) {
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
              vTaskDelay(25/portTICK_RATE_MS);
          }

        }

        // else if (distance < 100 && distance > 50){
        //   //printf("Stopping...\n");
        //   //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1300);
        //   for (count = 1500; count > 1400 ; count -= 1) {
        //       mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        //       vTaskDelay(100/portTICK_RATE_MS);
        //   }
        //
        //
        // }
        else if (myStart == '0'){
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1300);
        }

        else {// if (stop == 0){
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
            // printf("Speed is: %f, setpoint is: %f\n",speed, setpoint);
            // float error = setpoint - speed;
            // if (error > 0) {
            //     //printf("Speeding up to setpoint\n");
            //   while (speed < setpoint){
            //     //printf("count_drive: %d\n",count_drive);
            //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count_drive);
            //     count_drive = count_drive + 5;
            //     vTaskDelay(100/portTICK_RATE_MS);
            //   }
            // } else if (error < 0) {
            //     printf("Slowing down to setpoint\n");
            //   while (speed > setpoint){
            //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count_drive);
            //     count_drive = count_drive - 5;
            //     vTaskDelay(100/portTICK_RATE_MS);
            //   }
            // } else if (error == 0) {
            //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count_drive);
            //     vTaskDelay(100/portTICK_RATE_MS);
            // }
            // integral = integral + error * dt;
            // derivative = (error - previous_error) / dt;
            // float output = Kp * error + Ki * integral + Kd * derivative;
            // previous_error = error;
            // //printf("PID found the following: \n\tError:\t\t%.0f \n\tIntegral:\t%.1f \n\tDerivative:\t%.1f \n\tOutput is: \t%.1f\n\n", error, integral, derivative, output);
            // vTaskDelay(dt);


        }
        vTaskDelay(100/portTICK_RATE_MS);

      // //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);
      // vTaskDelay(1000/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
      // //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
      // vTaskDelay(100/portTICK_RATE_MS);
    }


    vTaskDelete(NULL);
}



void steering_control(void *arg)
{
    uint32_t angle, count;
    //vTaskDelay(25000/portTICK_RATE_MS);
      printf("Steering!\n");
      while (1) {

          if (dist_right < 28){
              angle = steering_per_degree_init(60);
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          }

          else if (dist_right > 50){
              angle = steering_per_degree_init(40);
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          }

          else if (dist_right > 28 && dist_right < 50){
            angle = steering_per_degree_init(50);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          }
          // angle = steering_per_degree_init(50);
          // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);


          // else if (dist_left < 30){
          //     angle = steering_per_degree_init(25);
          //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          // }
          //
          // else if (dist_left > 35){
          //     angle = steering_per_degree_init(75);
          //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          // }

          else {
              angle = steering_per_degree_init(50);
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          }
          //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50);
          // for (count = 50; count < STEERING_MAX_DEGREE; count++) {
          //     //count = 90;
          //     //printf("Angle of rotation: %d\n", count);
          //     angle = steering_per_degree_init(count);
          //     //printf("pulse width: %dus\n", angle);
          //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          //     vTaskDelay(25/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
          // }
          //
          // for (count = STEERING_MAX_DEGREE; count > 0; count--) {
          //     //count = 90;
          //     //printf("Angle of rotation: %d\n", count);
          //     angle = steering_per_degree_init(count);
          //     //printf("pulse width: %dus\n", angle);
          //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          //     vTaskDelay(25/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
          // }
          //
          // for (count = 0; count <50; count++) {
          //     //count = 90;
          //     //printf("Angle of rotation: %d\n", count);
          //     angle = steering_per_degree_init(count);
          //     //printf("pulse width: %dus\n", angle);
          //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          //     vTaskDelay(25/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
          // }
          vTaskDelay(500/portTICK_RATE_MS);

      }
}







// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
    // YOUR CODE HERE
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
    // YOUR CODE HERE
    int ret;
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    // YOUR CODE HERE
    int ret;
    uint8_t data, data2;
    uint16_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_VAL);
    i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    result = (data2 << 8) | data;
    // printf("\nread16 Data: %d\n", result);
    return result;
}


static void test_LIDARLite_v4LED(){
    printf("\n>> Polling LIDARLite_v4LED!\n");
    // variables

    uint8_t busyflag = 1;

    while (1) {
        writeRegister(0x00, 0x04);  // start the read/write - modification? do I need to writeRegister everytime?

        // check busy flag
        do {
            busyflag = 0x01 & readRegister(0x01);
        } while(busyflag == 1);

        // read LIDAR data
        distance = read16(0x10);
        printf("Distance: %d cm\n", distance);
        if (distance < 35){
          stop = 1;
        }
        else if (distance > 35){
          stop = 0;
        }

        // reset busy flag
        busyflag = 1;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}



static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void test_ir_left(){
      //Configure ADC
    if (unit == ADC_UNIT_1) {
      adc1_config_width(width);
      adc1_config_channel_atten(channel_left, atten);
    } else {
      adc2_config_channel_atten((adc2_channel_t)channel_left, atten);
    }

    //Characterize ADC
    adc_chars_left = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars_left);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
      uint32_t adc_reading_left = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          if (unit == ADC_UNIT_1) {
              adc_reading_left += adc1_get_raw((adc1_channel_t)channel_left);
          } else {
              int raw;
              adc2_get_raw((adc2_channel_t)channel_left, width, &raw);
              adc_reading_left += raw;
          }
      }
      adc_reading_left /= NO_OF_SAMPLES;
      //Convert adc_reading to voltage in mV
      uint32_t voltage_left = esp_adc_cal_raw_to_voltage(adc_reading_left, adc_chars_left);
      //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
      double volts_left =  (double) voltage_left / 1000.0;

      dist_left = (.0198)*volts_left;
      dist_left = (1.0/dist_left);
      if (dist_left > 150){
         printf("Object too far (left).\n");
      }
      else{
        printf("Distance (left): %.0f cm\n", dist_left);
      }
          //printf("Volts = %f\n", volts);
      vTaskDelay(pdMS_TO_TICKS(500));
    }


}


static void test_ir_right(){
      //Configure ADC
    if (unit == ADC_UNIT_1) {
      adc1_config_width(width);
      adc1_config_channel_atten(channel_right, atten);
    } else {
      adc2_config_channel_atten((adc2_channel_t)channel_right, atten);
    }

    //Characterize ADC
    adc_chars_right = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars_right);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
      uint32_t adc_reading_right = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          if (unit == ADC_UNIT_1) {
              adc_reading_right += adc1_get_raw((adc1_channel_t)channel_right);
          } else {
              int raw;
              adc2_get_raw((adc2_channel_t)channel_right, width, &raw);
              adc_reading_right += raw;
          }
      }
      adc_reading_right /= NO_OF_SAMPLES;
      //Convert adc_reading to voltage in mV
      uint32_t voltage_right = esp_adc_cal_raw_to_voltage(adc_reading_right, adc_chars_right);
      //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
      double volts_right =  (double) voltage_right / 1000.0;

      dist_right = (.0198)*volts_right;
      dist_right = (1.0/dist_right);
      if (dist_right > 150){
         printf("Object too far (right).\n");
      }
      else{
        printf("Distance (right): %.0f cm\n", dist_right);
      }
          //printf("Volts = %f\n", volts);
      vTaskDelay(pdMS_TO_TICKS(500));
    }


}

void app_main() {
    gpio_set_level(13,0);
    check_efuse();
    led_init();

    rmt_tx_init();
    uart_init();
    //alarm_init();
    // Routine
    i2c_master_init();
    i2c_scanner();
    pwm_init();
    calibrateESC();
    printf("Testing servo motor.......\n");
    xTaskCreate(recv_task, "uart_rx_task", 4096, NULL, 5, NULL);
    //xTaskCreate(test_ir_left, "ir", 4096, NULL, 5, NULL);
    xTaskCreate(test_ir_right, "ir", 4096, NULL, 5, NULL);
    xTaskCreate(steering_control, "steering_control", 4096, NULL, 5, NULL);
    xTaskCreate(drive_control, "drive_control", 4096, NULL, 5, NULL);
    xTaskCreate(test_LIDARLite_v4LED,"test_LIDARLite_v4LED", 4096, NULL, 5, NULL);
}
