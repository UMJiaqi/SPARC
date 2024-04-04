/*!
LTC2664: 4-Channel SPI 16-/12-Bit +/-10V Vout SoftSpan DACs with 10ppm/C Max Reference.

The SPI/Microwire-compatible 3-wire serial interface operates on logic levels as low as 1.71V, at clock
rates up to 50MHz.

 SPI DATA FORMAT (MSB First):

 24-Bit Load Sequence:

              Byte #1                       Byte #2                       Byte #3
              Command                   MSB                             LSB
 LTC2664-16 : C3 C2 C1 C0 A3 A2 A1 A0   D15 D14 D13 D12 D11 D10 D9 D8   D7 D6 D5 D4 D3 D2 D1 D0
 LTC2664-12 : C3 C2 C1 C0 A3 A2 A1 A0   D11 D10 D9  D8  D7  D6  D5 D4   D3 D2 D1 D0 X  X  X  X

 32-Bit Load Sequence:
              Byte #1                       Byte #2                       Byte #3                 Byte #4
              Command                   MSB                             LSB
 LTC2664-16 : X  X  X  X  X  X  X  X    C3 C2 C1 C0 A3 A2 A1 A0   D15 D14 D13 D12 D11 D10 D9 D8   D7 D6 D5 D4 D3 D2 D1 D0
 LTC2664-12 : X  X  X  X  X  X  X  X    C3 C2 C1 C0 A3 A2 A1 A0   D11 D10 D9  D8  D7  D6  D5 D4   D3 D2 D1 D0 X  X  X  X

 Cx   : DAC Command Code
 Ax   : DAC Address (0 to F, corresponding to DAC 0 to DAC 15)
 Dx   : DAC Data Bits
 X    : Don't care
*/

#ifndef LTC2664_H
#define LTC2664_H
#include <stdint.h>
//! @name LTC2664 Command Codes
//! OR'd together with the DAC address to form the command byte
#define  LTC2664_CMD_WRITE_N              0x00  //!< Write to input register n
#define  LTC2664_CMD_UPDATE_N             0x10  //!< Update (power up) DAC register n
#define  LTC2664_CMD_WRITE_N_UPDATE_ALL   0x20  //!< Write to input register n, update (power-up) all
#define  LTC2664_CMD_WRITE_N_UPDATE_N     0x30  //!< Write to input register n, update (power-up) 
#define  LTC2664_CMD_POWER_DOWN_N         0x40  //!< Power down n
#define  LTC2664_CMD_POWER_DOWN_ALL       0x50  //!< Power down chip (all DAC's, MUX and reference)

#define  LTC2664_CMD_SPAN                 0x60  //!< Write span to dac n
#define  LTC2664_CMD_CONFIG               0x70  //!< Configure reference / toggle
#define  LTC2664_CMD_WRITE_ALL            0x80  //!< Write to all input registers
#define  LTC2664_CMD_UPDATE_ALL           0x90  //!< Update all DACs
#define  LTC2664_CMD_WRITE_ALL_UPDATE_ALL 0xA0  //!< Write to all input reg, update all DACs
#define  LTC2664_CMD_MUX                  0xB0  //!< Select MUX channel (controlled by 5 LSbs in data word)
#define  LTC2664_CMD_TOGGLE_SEL           0xC0  //!< Select which DACs can be toggled (via toggle pin or global toggle bit)
#define  LTC2664_CMD_GLOBAL_TOGGLE        0xD0  //!< Software toggle control via global toggle bit
#define  LTC2664_CMD_SPAN_ALL             0xE0  //!< Set span for all DACs
#define  LTC2664_CMD_NO_OPERATION         0xF0  //!< No operation
//! @}

//! @name LTC2664 Span Codes
//! @{
//! Descriptions are valid for a 2.5V reference.
//! These can also be interpreted as 0 to 2*Vref, 0 to 4*Vref, etc.
//! when an external reference other than 2.5V is used.
#define  LTC2664_SPAN_0_TO_5V             0x0000
#define  LTC2664_SPAN_0_TO_10V            0x0001
#define  LTC2664_SPAN_PLUS_MINUS_5V       0x0002
#define  LTC2664_SPAN_PLUS_MINUS_10V      0x0003
#define  LTC2664_SPAN_PLUS_MINUS_2V5      0x0004
//! @}

//! @name LTC2664 Minimums and Maximums for each Span
//! @{
//! Lookup tables for minimum and maximum outputs for a given span
const float LTC2664_MIN_OUTPUT[5] = {0.0, 0.0, -5.0, -10.0, -2.5};
const float LTC2664_MAX_OUTPUT[5] = {5.0, 10.0, 5.0, 10.0, 2.5};
//! @}

//! @name LTC2664 Configuration options
//! @{
//! Used in conjunction with LTC2664_CMD_CONFIG command
#define  LTC2664_REF_DISABLE              0x0001  //! Disable internal reference to save power when using an ext. ref.
#define  LTC2664_THERMAL_SHUTDOWN         0x0002  //! Disable thermal shutdown (NOT recommended)
//! @}

//! @name LTC2664 MUX enable
//! @{
//! Used in conjunction with LTC2664_CMD_MUX command
#define  LTC2664_MUX_DISABLE              0x0000  //! Disable MUX
#define  LTC2664_MUX_ENABLE               0x0010  //! Enable MUX, OR with MUX channel to be monitored
//! @}

//! @name LTC2664 Global Toggle
//! @{
//! Used in conjunction with LTC2664_CMD_GLOBAL_TOGGLE command, affects DACs whose
//! Toggle Select bits have been set to 1
#define  LTC2664_TOGGLE_REG_A              0x0000  //! Update DAC with register A
#define  LTC2664_TOGGLE_REG_B              0x0010  //! Update DAC with register B
//! @}

//! Write the 16-bit dac_code to the LTC2664
//! @return Zero if readback (SDO) data matches the value previously sent, One if mismatched (indicating data error somewhere)
int8_t LTC2664_write(uint8_t cs, uint8_t dac_command, uint8_t dac_address, uint16_t dac_code);

//! Calculate a LTC2664 DAC code given the desired output voltage and DAC address (0-3)
//! @return The 16 bit code to send to the DAC
uint16_t LTC2664_voltage_to_code(float dac_voltage, float min_output, float max_output);

//! Calculate the LTC2664 DAC output voltage given the DAC code, offset, and LSB
//! @return the output voltage
float LTC2664_code_to_voltage(uint16_t dac_code, float min_output, float max_output);

#endif  // LTC2664_H
