#include <stdint.h>
#include <math.h>
#include <Arduino.h>
#include "LTC2664.h"
#include <SPI.h>

union LT_union_int16_2bytes
{
    int16_t LT_int16;    //!< 16-bit signed integer to be converted to two bytes
    uint16_t LT_uint16;  //!< 16-bit unsigned integer to be converted to two bytes
    uint8_t LT_byte[2];  //!< 2 bytes (unsigned 8-bit integers) to be converted to a 16-bit signed or unsigned integer
};

int8_t LTC2664_write(uint8_t cs, uint8_t dac_command, uint8_t dac_address, uint16_t dac_code)
// Write the 16-bit dac_code to the LTC2664
{
  static uint8_t last_data_array[4];
  uint8_t data_array[4], rx_array[4];
  int8_t ret;
  LT_union_int16_2bytes data;
  int8_t i;
  data.LT_int16 = dac_code;                                // Copy DAC code to union
  // transmit 8-bit at a time
  data_array[3] = 0;                                             // Only required for 32 byte readback transaction
  data_array[2] = dac_command | dac_address;             // Build command / address byte
  data_array[1] = data.LT_byte[1];                       // MS Byte
  data_array[0] = data.LT_byte[0];                       // LS Byte

    digitalWrite(SS, LOW);
    for (i = 3; i >= 0; i--)
    {
     rx_array[i] = SPI.transfer(data_array[i]);
    }
    digitalWrite(SS, HIGH);
  
   // Compare data read back to data that was sent the previous time this function was called
  if ((rx_array[2] == last_data_array[2]) && (rx_array[1] == last_data_array[1]) && (rx_array[0] == last_data_array[0]))
  {
    ret = 0;
  }
  else
  {
    ret = 1;
  }

  last_data_array[0] = data_array[0]; // Copy data array to a static array to compare
  last_data_array[1] = data_array[1]; // the next time the function is called
  last_data_array[2] = data_array[2];

  return(ret);
}

uint16_t LTC2664_voltage_to_code(float dac_voltage, float min_output, float max_output)
// Calculate a LTC2664 DAC code given the desired output voltage and the minimum / maximum
// outputs for a given softspan range.
{
  uint16_t dac_code;
  float float_code;
  float_code = 65535.0 * (dac_voltage - min_output) / (max_output - min_output);                    // Calculate the DAC code
  float_code = (float_code > (floor(float_code) + 0.5)) ? ceil(float_code) : floor(float_code);     // Round 
  if (float_code < 0.0) float_code = 0.0;
  if (float_code > 65535.0) float_code = 65535.0;
  dac_code = (uint16_t) (float_code);                                                               // Convert to unsigned integer
  return (dac_code);
}

float LTC2664_code_to_voltage(uint16_t dac_code, float min_output, float max_output)
// Calculate the LTC2664 DAC output voltage given the DAC code and and the minimum / maximum
// outputs for a given softspan range.
{
  float dac_voltage;
  dac_voltage = (((float) dac_code / 65535.0) * (max_output - min_output)) + min_output;            // Calculate the dac_voltage
  return (dac_voltage);
}
