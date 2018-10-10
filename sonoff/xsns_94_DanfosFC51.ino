/*
  xsns_34_DanfosFC51.ino - Danfos VLT MICRO DRIVE FC-51 Modbus frequency converter support for Sonoff-Tasmota

  Copyright (C) 2018  LVA  based by code Gennaro Tortone

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
//#undef USE_MODBUS
#ifdef USE_MODBUS
#ifdef USE_FC51


//#warning "USE_FC51"
#define D_LOG_FC51 "FC51: " // Wifi
//#define
/*********************************************************************************************\
 * Danfos FC51-Modbus frequency converter
 *
 * Based on: https://github.com/reaper7/SDM_Energy_Meter
\*********************************************************************************************/

#include <TasmotaSerial.h>

/* 
adress 1 bit
function 1 bit
data N bit
CRC 2 bit 
*/
/*----------------------------------START MODBUS BLOCK -------------------------------- */
  //0x01 - Read coils, 
  #define READ_MODBUS_REGISTR 0x03 // Read holding registers,
    //0x05 - Write single coil, 
  #define WRITE_MODBUS_REGISTR 0x06 //- Write single register,
    //0x0f - Write multiple coils,
    //0x10 - Write multiple registers,
    //0x0b - Get comm. event counter,
    //0x11v Report slave ID

TasmotaSerial *MODBUS_Serial;
bool MODBUS_initialized = false; 
bool ModbusReceiveReady()
{
  return (MODBUS_Serial->available() > 1);
}
bool use_modbus_tx_eneble_pin = false;

void MODBUS_Init() {
  //Serial.println("MODBUS START Init");
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "MODBUS START init"));
  AddLog(LOG_LEVEL_INFO);
  if ((pin[GPIO_MODBUS_RX] < 99) && (pin[GPIO_MODBUS_TX] < 99))
  {
    MODBUS_Serial = new TasmotaSerial(pin[GPIO_MODBUS_RX], pin[GPIO_MODBUS_TX], 0);
#ifdef MODBUS_SPEED
    if (MODBUS_Serial->begin(MODBUS_SPEED))
    {
#else
    if (MODBUS_Serial->begin(19200))
    {
#endif
      if (MODBUS_Serial->hardwareSerial())
      {
        ClaimSerial();
      }
      snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "MODBUS inited"));
      AddLog(LOG_LEVEL_INFO);
      MODBUS_initialized = true;
    }
    if ((pin[GPIO_MODBUS_TX_ENABLE] < 99)) 
    {
      //use_modbus_tx_eneble_pin = false;
      digitalWrite(pin[GPIO_MODBUS_TX_ENABLE], 0);
      #ifdef _LVA_DEBUG
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "GPIO_MODBUS_TX_ENABLE pin:%d"), pin[GPIO_MODBUS_TX_ENABLE]);
        AddLog(LOG_LEVEL_INFO);
        //Serial.println(log_data);
      #endif
    }
    MODBUS_initialized=true;
  }
  else {
    snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "MODBUS ERROR: TX and RX pins don't set"));
    AddLog(LOG_LEVEL_INFO);
  }
}

void ModbusSend16(uint8_t ModBusAdress, uint8_t function_code, uint16_t start_address, uint16_t register_count) 
{
  uint8_t frame[8];
  frame[0] = ModBusAdress; // FC51 Modbus address
  frame[1] = function_code;
  frame[2] = (uint8_t)(start_address >> 8);
  frame[3] = (uint8_t)(start_address);
  frame[4] = (uint8_t)(register_count >> 8);
  frame[5] = (uint8_t)(register_count);

  uint16_t crc = MODBUS_CRC(frame, 6);  // calculate out crc only from first 6 bytes
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);

  while (MODBUS_Serial->available() > 0)
  { // read serial if any old data is available
    MODBUS_Serial->read();
  }

  MODBUS_Serial->flush();

  if ((pin[GPIO_MODBUS_TX_ENABLE] < 99)) digitalWrite(pin[GPIO_MODBUS_TX_ENABLE], 1);
  MODBUS_Serial->write(frame, sizeof(frame));
  if ((pin[GPIO_MODBUS_TX_ENABLE] < 99)) digitalWrite(pin[GPIO_MODBUS_TX_ENABLE], 0);
}

uint8_t ModbusReceive(uint32_t *value, uint8_t ModBusAdress, uint8_t function, uint8_t answerBytes) {
  uint8_t buffer[answerBytes+5];
  *value = NAN;
  uint8_t len = 0;
  while (MODBUS_Serial->available() > 0)
  {
    buffer[len++] = (uint8_t)MODBUS_Serial->read();
  }
  if (len != answerBytes+5)
    return 3;   // FC51_ERR_NOT_ENOUGHT_BYTES
  else
  {
    if (buffer[0] == ModBusAdress && buffer[1] == function && buffer[2] == answerBytes)
    { // check node number, op code and reply bytes count
      if ((MODBUS_CRC(buffer, len - 2)) == ((buffer[len - 1] << 8) | buffer[len - 2]))
      { //calculate crc from first len - 2 bytes and compare with received crc (bytes 7 & 8)
        for (uint8_t i = 0; i < answerBytes;)
        {
          ((uint8_t *)value)[i] = buffer[i];
        }
      } else
          return 1;   // FC51_ERR_CRC_ERROR
    } else
        return 2; // FC51_ERR_WRONG_BYTES
  }
  return 0; // FC51_ERR_NO_ERROR
}

uint16_t MODBUS_CRC(uint8_t *frame, uint8_t num) {
  uint16_t crc, flag;
  crc = 0xFFFF;
  for (uint8_t i = 0; i < num; i++) {
    crc ^= frame[i];
    for (uint8_t j = 8; j; j--) {
      if ((crc & 0x0001) != 0) {        // If the LSB is set
        crc >>= 1;                      // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                          // Else LSB is not set
        crc >>= 1;                      // Just shift right
      }
    }
  }
  return crc;
}

/*********************************************************************************************/
/*
const uint16_t FC51_StatusAdress[] {
    // address  | request bytes | 
    //4999,  // Command Word
    50199, // Status Word
    3029,// Max Output Frequency param. 3-03
    50209, // Actual Output Frequency
    16099 // Motor Power kW int32, converter inex 1 
};
*/
#define FC51_CTW_ADDR 49999
#define FC51_CTW_BYTES 2
#define FC51_STW_ADDR 50100
#define FC51_STW_BYTES 2
#define FC51_MAX_FRQ_ADDR 3029     // Max Output Frequency param.3 - 03
#define FC51_MAX_FRQ_BYTES 4
#define FC51_ACT_FRQ_ADDR 50209
#define FC51_ACT_FRQ_BYTES 4
#define FC51_ACT_OUT_POWER_ADDR 16099 // Motor Power kW int32, converter inex 1
#define FC51_ACT_OUT_POWER_BYTES 4
/*
struct ModbusDevices
{
  uint8_t Addr = 0;
  uint8_t FC51_Runung = 0;
  uint16_t CTW = 0;           // Command Word
  uint16_t STW = 0;           // Status Word
  uint32_t MAX_FRQ = 0;       // Max Output Frequency param. 3-03
  uint16_t MAV = 0;           // Actual Output Frequency
  uint16_t AOP = 0;           //Actual Output POWER W
};
struct ModbusDevices FC51[5];
FC51[0].Addr = FC51_1_ADDR;
FC51[1].Addr = FC51_2_ADDR;
FC51[2].Addr = FC51_3_ADDR;
*/
/************************************************************************************************/
uint16_t CTW[FC51_DEVICES];// = 0;
uint16_t STW[FC51_DEVICES];// = 0;
uint32_t MAX_FRQ[FC51_DEVICES];// = 0;
uint16_t MAV[FC51_DEVICES];// = 0;     // Actual Output Frequency
uint16_t AOP[FC51_DEVICES];// = 0;     //Actual Output POWER W
uint8_t FC51_Runung = 0; // включен, зачение пишем и читаем по маске.
uint8_t FC51_Addr[FC51_DEVICES] = {FC51_1_ADDR, FC51_2_ADDR, FC51_3_ADDR};
uint8_t modbus_repeat = 0; // номер  попытки чтения параметра
uint8_t FC51_Point=0; // номер команды, четный - запрос,  нечетный -чтение
#define FC51_READ_PARAMETERS 5
#define FC51_MAX_POINT 2*FC51_DEVICES*FC51_READ_PARAMETERS
#define MODBUS_MAX_REPEAT 3

void FC51_50ms()
{ // Every 100 mSec
  uint32_t value = 0;
  uint8_t error = 0;
  if (FC51_MAX_POINT == FC51_Point)   {
    FC51_Point = 0;
  }
  uint8_t n_dev = FC51_Point / (FC51_READ_PARAMETERS * 2); // number of devises
  uint8_t n_com = (FC51_Point/2) % FC51_READ_PARAMETERS; // number of command
  #ifdef DEBUG_MODBUS
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "FC51_Point:%d\t n_dev:%d\t n_com:%d\t devices addr:%d\t modbus_repeat:%d\n"), FC51_Point, n_dev, n_com, FC51_Addr[n_dev], modbus_repeat);
  Serial.print(log_data);
  #endif //DEBUG_MODBUS

  switch (n_com) {
    case 0:       // ---------------------------- C T W ---------------------------------------------
      error = ModBusRead(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_CTW_ADDR, FC51_CTW_BYTES); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      }
      else if (99 == error) { // закончились попытки чтения, обнуляем параметр
        CTW[n_dev] = 0;
      }
      else if (0 == error) { // все хорошо 
        CTW[n_dev] = (uint16_t)value;
       // все хорошо можно читать следущий параметр
        snprintf_P(log_data, sizeof(log_data), PSTR(" Control Word %x"), CTW[n_dev]);
        Serial.println(log_data);
      }
      break;
    case 1:  // ---------------------------- S T W ---------------------------------------------
      error = ModBusRead(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_STW_ADDR, FC51_STW_BYTES); //  Status Word
      if (10 == error)
      { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      }
      else if (0 == error || 99 == error) { // все хорошо
        STW[n_dev] = (uint16_t)value;
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "Status Word %x"), STW[n_dev]);
        Serial.println(log_data);
        //AddLog(LOG_LEVEL_INFO);
        //FC51_Decode_STW();
      }
    break;
    case 2:
      // ------------------ Max Output Frequency param. 3-03 --------------------------------------------
      error = ModBusRead(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_MAX_FRQ_ADDR, FC51_MAX_FRQ_BYTES); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      }
      else if (0 == error || 99 == error)     { // все хорошо
        MAX_FRQ[n_dev] = value;
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "MAX_FRQ: %d/1000"), MAX_FRQ[n_dev]);
        Serial.println(log_data);
        //AddLog(LOG_LEVEL_INFO);
      }
      break;
    case 3:
      // ------------------ Actual Output Frequency (MAV) --------------------------------------------
      error = ModBusRead(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_ACT_FRQ_ADDR, FC51_ACT_FRQ_BYTES); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      }
      else if (0 == error || 99 == error) { // все хорошо
        MAV[n_dev] = value;
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "Actual Output Frequency: %d/10"), MAV[n_dev]);
        Serial.println(log_data);
        //AddLog(LOG_LEVEL_INFO);
      }
      break;
    case 4:
      // ------------------ Actual Output POWER W (AOP) --------------------------------------------
      error = ModBusRead(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_ACT_OUT_POWER_ADDR, FC51_ACT_OUT_POWER_BYTES); //  Status Word
      if (10 == error)
      { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      }
      else if (0 == error || 99 == error)
      { // все хорошо
        AOP[n_dev] = value;
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "Actual Output POWER W: %d/1000"), AOP[n_dev]);
        Serial.println(log_data);
        //AddLog(LOG_LEVEL_INFO);
      }
      break;
  }
}
// =======================================

uint8_t ModBusRead(uint8_t *point, uint32_t *p_value, uint8_t m_address, uint8_t m_function, uint16_t m_registr, uint16_t m_bytes)
{
  uint8_t error = 0;  // то что будем возвращать
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "ModBusRead:{Point:%d\t, Repeat:%d\tADR:%d\t REG:%d"), *point, modbus_repeat, m_address, m_registr);
  if (!(*point & 1)) {  // если четный  то пытаемся сделать запрос PointRead evene - request
    //Serial.print("Point:");
    //Serial.println(*point);
    if (modbus_repeat < MODBUS_MAX_REPEAT)
    { // если не закончились попытки
      // запрашиваем
      MODBUS_Serial->flush();
      ModbusSend16(m_address, m_function, m_registr, m_bytes);

      snprintf_P(log_data, sizeof(log_data), PSTR("%s\t -> ModbusSend16, error: 10"), log_data);
      Serial.println(log_data);

      ++modbus_repeat; // инкрементируем запрос
      ++*point; // переходим к чтению
      return 10; // послали запрос надо читать следующий раз
    }
    else {
      modbus_repeat = 0; // обнуляем попытки
      ++*point;
      ++*point;   // надо перескочить чтение, сразу делаемзапрос следующего
      *p_value=0; // обнуляем буфер со значением
      snprintf_P(log_data, sizeof(log_data), PSTR("%s\t MODBUS_MAX_REPEAT, error: 99"), log_data);
      Serial.println(log_data);
      return 99;  // закончилось число попыток чтения
    }
  }
  else  {// не четный, читаем что получилось PointRead odd - revieve
    if (MODBUS_Serial->available() == 0)  {// проверяем, что что-то есть приемном буфере
      snprintf_P(log_data, sizeof(log_data), PSTR("%s\tNot data in buffer"), log_data);
      //--*point; // все плохо повторяем чтение
      error = 4;
    }
    else  {
      error = ModbusReceive(p_value, m_address, m_registr, m_bytes);
    }
    if (error) {
      --*point; // все плохо повторяем чтение
      snprintf_P(log_data, sizeof(log_data), PSTR("%s\tError read"), log_data);
    }
    else {
      ++*point; // все хорошо переходим к следующему параметру
      snprintf_P(log_data, sizeof(log_data), PSTR("%s\tead OK, value:%d"), log_data, *p_value);
      //Serial.println(log_data);
      modbus_repeat = 0;
    }
    snprintf_P(log_data, sizeof(log_data), PSTR("%s\t, error:%d}"), log_data, error);
    Serial.println(log_data);
    //AddLog(LOG_LEVEL_DEBUG);
    return error;
  }
}

  /*
    bit,     0               |  1                   |  Normal
    0 - Control not ready    | Control ready        |    1
    1 - Unit not ready       | Unit ready           |    1
    2 - Coasted              | Not coasted          |    0
    3 -                      | Error, tripped       |    0
    4 -                      | Error, no trip       |    0
    5 - Not used             | Not used             |    x
    6 -                      | Error, trip locked   |    0
    7 - No warning           | Warning              |    0
    8 - Not on reference     | On reference         |    1
    9  - Hand mode           | Auto mode            |    1
    10 - Out of freq. range  | In frequency range   |    1
    11 - Not running         | Running              |    1
    12 - No res. brake fault | Resistor brake fault |    0
    13 - No voltage warning  | Voltage warning      |    0
    14 - Not in current limit| Current limit        |    0
    15 - No thermal warning  |Thermal warning       |    0
    --------------------------------------------------------
                                                      0xF03
    */

char*  FC51_Decode_STW (uint8_t n)
{
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "%d:{"), n);
  if (!STW[n])
  {
    snprintf_P(log_data, sizeof(log_data), PSTR("%s NOT DATA"), log_data);
  }
  else
  {
    if (STW[n] != 0xF03)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s STW:%d"), log_data, STW[n]);
    }
    if (!(STW[n] & 1 < 0))
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Control not ready"), log_data);
    }
    if (!(STW[n] & 1 < 1))
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Unit not ready"), log_data);
    }
    if (STW[n] & 1 < 2)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Not Coasted"), log_data);
    }
    if (STW[n] & 1 < 3)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Error, tripped"), log_data);
    }
    if (STW[n] & 1 < 4)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Error, no trip"), log_data);
    }
    if (STW[n] & 1 < 6)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Error, trip locked"), log_data);
    }
    if (STW[n] & 1 < 7)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Warning"), log_data);
    }
    if (!(STW[n] & 1 < 8))
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Not on reference"), log_data);
    }
    if (!(STW[n] & 1 < 9))
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Hand mode"), log_data);
    }
    if (!(STW[n] & 1 < 10))
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Out of freq. range"), log_data);
    }
    if (!(STW[n] & 1 < 11))
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Not running"), log_data);
    }
    if (STW[n] & 1 < 12)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Resistor brake fault"), log_data);
    }
    if (STW[n] & 1 < 13)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Voltage warning"), log_data);
    }
    if (STW[n] & 1 < 14)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Current limit"), log_data);
    }
    if (STW[n] & 1 < 15)
    {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s, Thermal warning"), log_data);
    }
  }
  snprintf_P(log_data, sizeof(log_data), PSTR("%s}"), log_data);
  Serial.println(log_data);
  return log_data;
  //AddLog(LOG_LEVEL_INFO);
}
/*
*/


  /*  Command Word byte:
    0 - Предустановленное задание/ Preset reference LSB
    1 - Предустановленное задание/Preset reference MSB
    2 - торможение DC (0 включено/ 1 выключено) / NO DC brake 
    3 - Остановка выбегом (0 включено/ 1 выключено) /No coast stop
    4 - Быстрая остановка (0 включено/ 1 выключено) / No quick stop
    5 - фиксация частоты (0 включено/ 1 выключено) /No freeze outp
    6 - Остановка(0) - запуск (1)/ Start
    7 - сброс (1) /Reset
    8 - нет фиксации часоты (0)- есть (1) /Jog ???
    9 - изменение скорости 1/2 / Ramp 1 -Ramp 2
    10 - недействительные данные/ действительные / Data valid
    11 - Relay 1 off/on
    12 - Not used
    13 - Setup 1 Setup 2
    14 - Not used
    15  - Реверс(1) /Reversing

    1148 (0x047C/  0000 0100 0111 1100) -включить
                              ^ пуск        
    */
/*
void FC51_Write_CTW(bool start_stop) // не дописана еще
{
  FC51_ModbusSend16(WRITE_REGISTR, FC51_StatusAdress[0], 2);
  error = FC51_ModbusReceive(&value, 0x03, 2);
  if (error)
  {
    snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "FC51 CTW response error %d"), error);
    AddLog(LOG_LEVEL_INFO);
    break;
  }
  CTW = (uint16_t)value;
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "FC51 Command Word %x"), value);
  AddLog(LOG_LEVEL_INFO);
  FC51_Runung = CTW & 1 < 6; // определили статус
  //для включния надо будет использовать 'CTW|=1<6' для выключения 'CTW&=~(1<<6)' , где 6 - номер бита
  // можно использовать bitRead(value, bit), или bitSet или bitClear
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 " Runnund %x"), FC51_Runung);
  AddLog(LOG_LEVEL_INFO);
}
*/




/*
#ifdef USE_WEBSERVER
const char HTTP_FC51_DATA[] PROGMEM = "%s"
  "{s}SDM120 " D_VOLTAGE "{m}%s " D_UNIT_VOLT "{e}"
  "{s}SDM120 " D_CURRENT "{m}%s " D_UNIT_AMPERE "{e}"
  "{s}SDM120 " D_POWERUSAGE_ACTIVE "{m}%s " D_UNIT_WATT "{e}"
  "{s}SDM120 " D_POWERUSAGE_APPARENT "{m}%s " D_UNIT_VA "{e}"
  "{s}SDM120 " D_POWERUSAGE_REACTIVE "{m}%s " D_UNIT_VAR "{e}"
  "{s}SDM120 " D_POWER_FACTOR "{m}%s{e}"
  "{s}SDM120 " D_FREQUENCY "{m}%s " D_UNIT_HERTZ "{e}"
  "{s}SDM120 " D_ENERGY_TOTAL "{m}%s " D_UNIT_KILOWATTHOUR "{e}";
#endif  // USE_WEBSERVER
*/

#ifdef USE_WEBSERVER
const char HTTP_FC51_STR[] PROGMEM = "%s" "{s}FC51_%d %s:" "{m}%s" "{e}"; // 4 переменные
const char HTTP_FC51_INT[] PROGMEM = "%s" "{s}FC51_%d %s:" "{m}%d %s" "{e}"; // 5 переменных "FC51_1 STW:111 Hz"
const char HTTP_FC51_MODBUS_ERR[] PROGMEM = "%s" "{s}MODBUS" "{m}%s" "{e}"; // 5 переменных "FC51_1 STW:111 Hz"
#endif  // USE_WEBSERVER

// for i18.h
#define D_FC51 "FC51_"
#define D_CTW "CTW"
#define D_STW "STW"
#define D_STW_D "STW decode"
#define D_MAX_FRQ "MAX FRQ"
#define D_MAV "OUT FRQ"
#define D_AOP "OUT POWER"



void FC51_Show(boolean json)
{
/*
  char voltage[10];
  char current[10];
  char active_power[10];
  char apparent_power[10];
  char reactive_power[10];
  char power_factor[10];
  char frequency[10];
  char energy_total[10];

  //dtostrfd((double)ESP.getVcc()/1000, 3, stemp1); "значение инт", "количество знаков", "стороковая переменная куда сохраняем результат"

  dtostrfd(sdm120_voltage, Settings.flag2.voltage_resolution, voltage);
  dtostrfd(sdm120_current, Settings.flag2.current_resolution, current);
  dtostrfd(sdm120_active_power, Settings.flag2.wattage_resolution, active_power);
  dtostrfd(sdm120_apparent_power, Settings.flag2.wattage_resolution, apparent_power);
  dtostrfd(sdm120_reactive_power, Settings.flag2.wattage_resolution, reactive_power);
  dtostrfd(sdm120_power_factor, 2, power_factor);
  dtostrfd(sdm120_frequency, Settings.flag2.frequency_resolution, frequency);
  dtostrfd(sdm120_energy_total, Settings.flag2.energy_resolution, energy_total);
*/
  if (json)    {
    for (uint8_t n_dev = 0; n_dev < FC51_DEVICES; ++n_dev)     {
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s,\"" D_FC51 "%d\":{\"" D_CTW "\":%d,\"" D_STW "\":%d,\"" D_MAX_FRQ "\":%d,\"" D_MAV "\":%d,\"" D_AOP "\":%d,\"" D_STW_D "\":%s}"),
                 mqtt_data, n_dev, CTW[n_dev], STW[n_dev], MAX_FRQ[n_dev], MAV[n_dev], AOP[n_dev], FC51_Decode_STW(n_dev));
      if (n_dev < (FC51_DEVICES - 1)) {
        snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s%s"),mqtt_data,",");
      }
    }
/*
    snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s,\"" D_RSLT_ENERGY "\":{\"" D_JSON_TOTAL "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s,\"" D_JSON_APPARENT_POWERUSAGE "\":%s,\"" D_JSON_REACTIVE_POWERUSAGE "\":%s,\"" D_JSON_FREQUENCY "\":%s,\"" D_JSON_POWERFACTOR "\":%s,\"" D_JSON_VOLTAGE "\":%s,\"" D_JSON_CURRENT "\":%s}"),
      mqtt_data, energy_total, active_power, apparent_power, reactive_power, frequency, power_factor, voltage, current);
#ifdef USE_DOMOTICZ
    if (0 == tele_period) {
      DomoticzSensor(DZ_VOLTAGE, voltage);
      DomoticzSensor(DZ_CURRENT, current);
      DomoticzSensorPowerEnergy((int)sdm120_active_power, energy_total);
    }
#endif  // USE_DOMOTICZ
*/
#ifdef USE_WEBSERVER
  } else {
    if (MODBUS_initialized){
      for (uint8_t n_dev = 0; n_dev < FC51_DEVICES; ++n_dev)
      {
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, n_dev, D_CTW, CTW[n_dev], "");
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, n_dev, D_STW, STW[n_dev], "");
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_STR, mqtt_data, n_dev, D_STW_D, FC51_Decode_STW(n_dev));
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, n_dev, D_MAX_FRQ, MAX_FRQ[n_dev], "Hz");
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, n_dev, D_MAV, MAV[n_dev], "Hz");
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, n_dev, D_AOP, AOP[n_dev], "W");
      }
    }
    else {
      snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_MODBUS_ERR, mqtt_data,"don't set TX and RX Pins");
    }
#endif  // USE_WEBSERVER
  }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XSNS_94

boolean Xsns94(byte function)
{
  boolean result = false;

  //if (MODBUS_initialized) 
  //{
    switch (function) 
    {
      case FUNC_INIT:
        MODBUS_Init();
        break;
      case FUNC_EVERY_50_MSECOND:
        //FC51_50ms();
          break;
      case FUNC_EVERY_SECOND:
        if (MODBUS_initialized)
          FC51_50ms();
        break;
      case FUNC_JSON_APPEND:
        //FC51_Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_APPEND:
        //if (MODBUS_initialized)
          FC51_Show(0);
        break;
#endif  // USE_WEBSERVER
    }
  //}
  return result;
}

#endif   // USE_FC51
#endif   //USE_MODBUS
