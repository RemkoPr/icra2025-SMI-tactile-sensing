#include "mcp3201.h"

#define SAMPLE_LEN_IN_BYTES     2
#define BUFFER_LEN_IN_SAMPLES   100
#define BUFFER_LEN_IN_BYTES     BUFFER_LEN_IN_SAMPLES*SAMPLE_LEN_IN_BYTES

const byte csPin = 10;
const float Vref = 5;
int sample_ctr = 0;
uint8_t byte_buffer[BUFFER_LEN_IN_BYTES]= {0};
  uint8_t start_seq[10] = {1};

MCP3201 myMCP3201(csPin);

void setup() {
  Serial.begin(115200);
  while(!Serial);  // wait
  delay(5000);
}

void loop() {
  
  uint16_t sample = myMCP3201.readAdc();
  byte_buffer[2*sample_ctr] = sample >> 8;  // MSB first
  byte_buffer[2*sample_ctr + 1] = sample & 0xFF;  // LSB
  //Serial.println(String(sample) + " " + String(byte_buffer[2*sample_ctr]) + " " + String(byte_buffer[2*sample_ctr + 1]));
  sample_ctr++;
  if (sample_ctr >= BUFFER_LEN_IN_SAMPLES) {
    sample_ctr = 0;
    Serial.write(byte_buffer, BUFFER_LEN_IN_BYTES);
  }
}
