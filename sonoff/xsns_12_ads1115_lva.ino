/*
  xsns_12_ads1115_lva.ino - ADS1115 A/D Converter support for Sonoff-Tasmota

  Copyright (C) 2018  Theo Arends

UPDATING LVA

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
//#undef USE_ADS1115_LVA
#ifdef USE_ADS1115_LVA
/*********************************************************************************************\
 * ADS1115 - 4 channel 16BIT A/D converter
 *
 * Required library: none but based on Adafruit Industries ADS1015 library
 *
 * I2C Address: 0x48, 0x49, 0x4A or 0x4B
 *
 * The ADC input range (or gain) can be changed via the following
 * defines, but be careful never to exceed VDD +0.3V max, or to
 * exceed the upper and lower limits if you adjust the input range!
 * Setting these values incorrectly may destroy your ADC!
 *                                                                 ADS1115
 *                                                                 -------
 * ADS1115_REG_CONFIG_PGA_6_144V  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
 * ADS1115_REG_CONFIG_PGA_4_096V  // 1x gain   +/- 4.096V  1 bit = 0.125mV
 * ADS1115_REG_CONFIG_PGA_2_048V  // 2x gain   +/- 2.048V  1 bit = 0.0625mV
 * ADS1115_REG_CONFIG_PGA_1_024V  // 4x gain   +/- 1.024V  1 bit = 0.03125mV
 * ADS1115_REG_CONFIG_PGA_0_512V  // 8x gain   +/- 0.512V  1 bit = 0.015625mV
 * ADS1115_REG_CONFIG_PGA_0_256V  // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
\*********************************************************************************************/

#define ADS1115_ADDRESS_ADDR_GND        0x48      // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD        0x49      // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA        0x4A      // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL        0x4B      // address pin tied to SCL pin

#define ADS1115_CONVERSIONDELAY         (8)       // CONVERSION DELAY (in mS)

/*======================================================================
POINTER REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_POINTER_MASK        (0x03)
#define ADS1115_REG_POINTER_CONVERT     (0x00)
#define ADS1115_REG_POINTER_CONFIG      (0x01)
#define ADS1115_REG_POINTER_LOWTHRESH   (0x02)
#define ADS1115_REG_POINTER_HITHRESH    (0x03)

/*======================================================================
CONFIG REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_CONFIG_OS_MASK      (0x8000)
#define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
#define ADS1115_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
#define ADS1115_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

#define ADS1115_REG_CONFIG_MUX_MASK     (0x7000)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
#define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

#define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00)
#define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3 (default)
#define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2
#define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)
#define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_DR_MASK      (0x00E0)
#define ADS1115_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
#define ADS1115_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
#define ADS1115_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
#define ADS1115_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
#define ADS1115_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
#define ADS1115_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
#define ADS1115_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second
#define ADS1115_REG_CONFIG_DR_6000SPS   (0x00E0)  // 6000 samples per second

#define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010)
#define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008)
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK    (0x0003)
#define ADS1115_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

// #define ADS1115_DEVICES_MAX 4 // 0 don't found devices

const char HTTP1_ADS1115[] PROGMEM = "%s{s}ADS1115_%d: A%d{m}%d{e}"; // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
const char HTTP2_ADS1115[] PROGMEM = "%s{s}A%d{m}%d{e}"; // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>

//const char HTTP_SNS_ANALOG[] PROGMEM = "%s{s}%s " D_ANALOG_INPUT "%d{m}%d{e}";

//uint8_t ads1115_type = 0; // непонял для чего возможно надо убрать

//uint8_t ads1115_addres[ADS1115_DEVICES_MAX];
uint8_t ads1115_devices;
//uint8_t ads1115_address;
uint8_t ads1115_device[ADS1115_DEVICES_MAX];// LVA = { ADS1115_ADDRESS_ADDR_GND, ADS1115_ADDRESS_ADDR_VDD, ADS1115_ADDRESS_ADDR_SDA, ADS1115_ADDRESS_ADDR_SCL };

//Ads1115StartComparator(channel, ADS1115_REG_CONFIG_MODE_SINGLE);
//Ads1115StartComparator(channel, ADS1115_REG_CONFIG_MODE_CONTIN);
void Ads1115StartComparator(uint8_t ads1115, uint8_t channel, uint16_t mode)
{
    //Serial.print("Ads1115StartComparator addr:");  Serial.println(ads1115,HEX);
  // Start with default values
  uint16_t config = mode |
                    ADS1115_REG_CONFIG_CQUE_NONE    | // Comparator enabled and asserts on 1 match
                    ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non Latching mode
                    ADS1115_REG_CONFIG_PGA_6_144V   | // ADC Input voltage range (Gain)
                    ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1115_REG_CONFIG_DR_6000SPS;    // 6000 samples per second

  // Set single-ended input channel
  config |= (ADS1115_REG_CONFIG_MUX_SINGLE_0 + (0x1000 * channel));

  // Write config register to the ADC
  I2cWrite16(ads1115, ADS1115_REG_POINTER_CONFIG, config);
}

int16_t Ads1115GetConversion(uint8_t ads1115, uint8_t channel)
{
  Ads1115StartComparator(ads1115, channel, ADS1115_REG_CONFIG_MODE_SINGLE);
  // Wait for the conversion to complete
  delay(ADS1115_CONVERSIONDELAY);
  // Read the conversion results
  I2cRead16(ads1115, ADS1115_REG_POINTER_CONVERT);

  Ads1115StartComparator(ads1115, channel, ADS1115_REG_CONFIG_MODE_CONTIN);
  delay(ADS1115_CONVERSIONDELAY);
  // Read the conversion results
  uint16_t res = I2cRead16(ads1115, ADS1115_REG_POINTER_CONVERT);
  return (int16_t)res;
}

/********************************************************************************************/

void Ads1115Detect()
{
  uint16_t buffer;
  if (!ads1115_devices) {
    //Serial.println("Ads1115Detect start");
    snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "Start detect ads1115"));
    AddLog(LOG_LEVEL_INFO);
    //   return;
    // }
    //for (byte i = 0; i < sizeof(ads1115_addresses); i++) {
    //LVA
    for (byte i = 0; i < 4; i++) {
      if (I2cValidRead16(&buffer, ADS1115_ADDRESS_ADDR_GND+i, ADS1115_REG_POINTER_CONVERT)) {
        ads1115_devices++;
        ads1115_device[ads1115_devices]=ADS1115_ADDRESS_ADDR_GND+i;
        for (byte ii=0; ii<4; ii++){
          Ads1115StartComparator(ads1115_device[ads1115_devices], ii, ADS1115_REG_CONFIG_MODE_CONTIN);
        }
        //ads1115_type = 1;
        //snprintf_P(log_data, sizeof(log_data), S_LOG_I2C_FOUND_AT, "ADS1115", ads1115_device[ads1115_devices]);
        //Serial.println(log_data);
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "Detected ads1115_%d Adr:%d"), ads1115_devices, ads1115_device[ads1115_devices]);
        AddLog(LOG_LEVEL_INFO);
      }
    }
  }
}
void Ads1115Show(boolean json) {
  if (ads1115_devices > 0) {

    //Serial.println("Ads1115Show Empy :( )");

    char stemp[10];

    byte dsxflg = 0;
    for (byte a = 1; a <= ads1115_devices; a++){
      for (byte i = 0; i < 4; i++) {
        uint16_t adc_value = Ads1115GetConversion(ads1115_device[a], i);
                if (json) {
          if (!dsxflg  ) {
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s,\"ADS1115\":{"), mqtt_data);
            stemp[0] = '\0';
          }
          if (i==0){
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s\"ADS_%d\":{"), mqtt_data, a);
            stemp[0] = '\0';
          }
          snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s%s\"A%d\":%d"), mqtt_data, stemp, i, adc_value);
          if (i==3 && a == ads1115_devices){
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s}"), mqtt_data);
          } else if (i==3) {
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s}"), mqtt_data);
            strcpy(stemp, ",");
          } else {
            strcpy(stemp, ",");
          }
    #ifdef USE_WEBSERVER
        dsxflg++;
        } else {  //const char HTTP_ADS1115[] PROGMEM = "%s{s}%s%d A%d{m}%d{e}"; // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
          //snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_ADS1115, mqtt_data, a, i, adc_value);
          if (i==0) {
            snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP1_ADS1115, mqtt_data, a, i, adc_value);
          }
          else {
            snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP2_ADS1115, mqtt_data, i, adc_value);
          }
#endif  // USE_WEBSERVER
          }
        }
    }
    // похоже тепрь лишнее
    if (json) {
      if (dsxflg) {
        snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s}"), mqtt_data);
      }
    }
  }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XSNS_12

boolean Xsns12(byte function)
{
  boolean result = false;

  if (i2c_flg) {
    switch (function) {
      case FUNC_PREP_BEFORE_TELEPERIOD:
        Ads1115Detect();
        break;
      case FUNC_JSON_APPEND:
        Ads1115Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_APPEND:
        Ads1115Show(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_ADS1115
#endif  // USE_I2C
