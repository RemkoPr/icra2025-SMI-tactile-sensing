/*
 This example reads audio data from an Invensense's ICS43432 I2S microphone
 breakout board, and prints out the samples to the Serial console. The
 Serial Plotter built into the Arduino IDE can be used to plot the audio
 data (Tools -> Serial Plotter)

 Circuit:
 * Arduino/Genuino Zero, MKR family and Nano 33 IoT
 * ICS43432:
   * GND connected GND
   * 3.3V connected to 3.3V (Zero, Nano) or VCC (MKR)
   * WS connected to pin 0 (Zero) or 3 (MKR) or A2 (Nano)
   * CLK connected to pin 1 (Zero) or 2 (MKR) or A3 (Nano)
   * SD connected to pin 9 (Zero) or A6 (MKR) or 4 (Nano)

 created 17 November 2016
 by Sandeep Mistry adapted by Remko Proesmans
 */

#include <I2S.h>

#define FS  20000 
#define SAMPLE_BUFFER_LEN   50
#define SAMPLE_BYTE_LEN     3

int start_time_ = micros();
int time_passed = 0;
uint8_t sample_buffer[SAMPLE_BUFFER_LEN * SAMPLE_BYTE_LEN] = {0};
int sample_ctr = 0;
uint8_t start_seq[10] = {1};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // start I2S at 8 kHz with 32-bits per sample
  if (!I2S.begin(I2S_PHILIPS_MODE, FS, 32)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  
  delay(5000);
}

void loop() {
  int32_t sample = I2S.read();
  
  /*time_passed = micros() - start_time_;
  if (time_passed > 1000000) {
    Serial.println(sample_ctr/(time_passed/1000000.0));
    start_time_ = micros();
    sample_ctr = 0;
  }*/
  if (sample) {
    //Serial.println("x: " + String(sample) + " -> " + String(sample >> 8));
    sample_buffer[sample_ctr*SAMPLE_BYTE_LEN] = sample >> 24;
    sample_buffer[sample_ctr*SAMPLE_BYTE_LEN + 1] = (sample >> 16) & 0xFF;
    sample_buffer[sample_ctr*SAMPLE_BYTE_LEN + 2] = (sample >> 8) & 0xFF;
    /*Serial.println(String(sample_buffer[sample_ctr]) + " " 
                    + String(sample_buffer[sample_ctr + 1]) + " " 
                    + String(sample_buffer[sample_ctr + 2]));*/

    sample_ctr++;
  }
  if(sample_ctr == SAMPLE_BUFFER_LEN) {
    sample_ctr = 0;
    Serial.write(sample_buffer, SAMPLE_BUFFER_LEN*SAMPLE_BYTE_LEN);

    /*Serial.println("------------------------------------------");
    for(int i=0 ; i < SAMPLE_BUFFER_LEN ; i++) {
      Serial.println(String(sample_buffer[i*SAMPLE_BYTE_LEN]) + " " 
                    + String(sample_buffer[i*SAMPLE_BYTE_LEN + 1]) + " " 
                    + String(sample_buffer[i*SAMPLE_BYTE_LEN + 2]));
    }
    Serial.println("------------------------------------------");*/
  }
}
