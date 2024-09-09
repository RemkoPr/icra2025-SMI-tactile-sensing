/*
 * SOURCE:
 * https://forum.arduino.cc/t/i2s-wav-playback-on-nrf52840/1058134
 * https://jimmywongiot.com/2021/06/26/i2c-mems-microphone-with-nordic-nrf52-series/
 * https://github.com/gregtomasch/nRF52_24-bit-_I2S_Microphone_Audio_Recording_Utility/blob/729c85e05485d0dac77a24e3e650015ecd7c4914/i2s_Slave_External_PWM_CK_Mono/main.c
 * 
 * 
 * Implementation with own I2S clocks using PWM, better readout attempt:
 * sck and sdin are logged while WS low (left channel), "dead time" while WS high is used to 
 * extract a value from the logs
 * 
 */

#include <nrfx_i2s.h> // or #include <hal/nrf_i2s.h>
#include <nrfx_pwm.h>


#define PIN_SCK     D10                 // BCLK/SCK
#define PIN_LRCK    D9                 // WS/LRCK
#define PIN_SDIN    D8                 // DIN
#define BUFFER_LEN  49*32  // 64 MHz clock frequency, SCK of ~1.3MHz. 32 SCK cycles must be stored,
                         // at the very most 49 MCU clock cycles per SCK cycle, so 49*32. Measurements 
                         // show nothing above 139, most of the time its 73


uint8_t data[3];
uint32_t buffer_ctr = 0;
uint8_t sck_buffer[BUFFER_LEN] = {0};
uint8_t sdin_buffer[BUFFER_LEN] = {0};
//uint32_t buffer_ctr_ctr = 0;
//uint32_t buffer_ctr_buffer[100] = {0};
int32_t currentValue = 0;
int bit_ctr = 31;
int sample_ctr = 0;
int t_start = micros();

static nrfx_pwm_t pwm1 = NRFX_PWM_INSTANCE(0);
static nrfx_pwm_t pwm2 = NRFX_PWM_INSTANCE(1);

static nrf_pwm_values_individual_t seq1_values[] = {3};

static nrf_pwm_sequence_t seq1 = {
    .values = {
        .p_individual = seq1_values
    },
    .length          = NRF_PWM_VALUES_LENGTH(seq1_values),
    .repeats         = 1,
    .end_delay       = 0
};

nrfx_pwm_config_t config1 = {
    .output_pins  = {
        32 + 8, // Arduino Nano 33 BLE pin D12, SCK source
        NRFX_PWM_PIN_NOT_USED,
        NRFX_PWM_PIN_NOT_USED,
        NRFX_PWM_PIN_NOT_USED,
    },
    .irq_priority = 7,
    .base_clock   = (nrf_pwm_clk_t)(1UL),  //NRF_PWM_CLK_8MHz,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = 6,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_AUTO,
};
nrfx_pwm_config_t config2 = {
    .output_pins  = {
        32 + 1, // Arduino Nano 33 BLE pin D11, WS source
        NRFX_PWM_PIN_NOT_USED,
        NRFX_PWM_PIN_NOT_USED,
        NRFX_PWM_PIN_NOT_USED,
    },
    .irq_priority = 7,
    .base_clock   = (nrf_pwm_clk_t)(7UL),  //NRF_PWM_CLK_125kHz,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = 6,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_AUTO,
};


void setup() {

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // PWM setup for spoofed BCLK/SCK and WS/LRCK
  nrfx_pwm_init(&pwm1, &config1, NULL);
  nrfx_pwm_init(&pwm2, &config2, NULL);

  nrfx_pwm_simple_playback(&pwm1, &seq1, 1, NRFX_PWM_FLAG_LOOP);
  nrfx_pwm_simple_playback(&pwm2, &seq1, 1, NRFX_PWM_FLAG_LOOP);
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_LRCK, INPUT);
  pinMode(PIN_SDIN, INPUT);

  for(int i=0 ; i < 9; i++) {
    Serial.write((byte)0);
  }
}

void loop() {
  if (!((NRF_P0->IN >> 27))) { // WS already low, wait out this cycle
    while(!((NRF_P0->IN >> 27)));
    // WS turns high, nothing interesting happens here, we have some time to get out of this "if"
  }
  
  while((NRF_P0->IN >> 27)); // WS known to be high, wait for it to turn low
  
  // WS just turned low, immediately log
  sck_buffer[buffer_ctr] = (NRF_P1->IN >> 2);
  sdin_buffer[buffer_ctr] = (NRF_P0->IN >> 21);
  buffer_ctr++;
  while(!((NRF_P0->IN >> 27))) { // keep logging while WS remains low
    sck_buffer[buffer_ctr] = (NRF_P1->IN >> 2);
    sdin_buffer[buffer_ctr] = (NRF_P0->IN >> 21);
    buffer_ctr++;
  }
  
  // WS turned high again, some time for processing
  bool sck_high = false;
  for(int i = 0; i < buffer_ctr; i++) {
    if(sck_buffer[i] & !sck_high) { // rising sck edge
      sck_high = true;
    } else if(!sck_buffer[i] & sck_high) { // falling sck edge
      sck_high = false;
      currentValue |= sdin_buffer[i] << bit_ctr;
      bit_ctr--;
    }
  }
  //Serial.println("value: " + String(currentValue >> 8) + "\t bit ctr: " + String(bit_ctr));
  if(bit_ctr == 0) {
    //Serial.println(currentValue >> 8);
    /*Serial.write(currentValue >> 24);
    Serial.write(currentValue >> 16);
    Serial.write(currentValue >> 8);*/
    sample_ctr++;
  }
  currentValue = 0;
  bit_ctr = 31;
  buffer_ctr = 0;
  if(sample_ctr > 5000) {
    Serial.println( sample_ctr/((micros() - t_start)/1000000.0) );
    t_start = micros();
    sample_ctr = 0;
  }
}
