/*
  xsns_34_DanfosFC51.ino - Danfos VLT MICRO DRIVE FC-51 Modbus frequency converter support for Sonoff-Tasmota
  version 0.1.1.beta

  Copyright (C) 2018  LVA  

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

#define XSNS_94   94

#define MODBUS_DEBUG
#define MODBUS_SIM // modbus summulation

#undef MODBUS_SPEED
#ifndef MODBUS_SPEED
#define MODBUS_SPEED 19200
#endif

//#warning "USE_FC51"
#define D_LOG_FC51 "FC51: " // FC51
#define D_LOG_MODBUS "MODBUS: "  // MQTT

/*
#define D_ONLINE "Online"
#define D_OFFLINE "Offline"
#define D_PORT "Port"
#define D_START "Start"

*/

/*********************************************************************************************\
 * 
 * Danfos FC51-Modbus frequency converter setting
 *
 * adress 1 bit
 * function 1 bit
 * data N bit
 * CRC 2 bit 
 * 
\*********************************************************************************************/

#include <TasmotaSerial.h>


/*----------------------------------START MODBUS BLOCK -------------------------------- */
  //0x01 - Read coils, 
  #define READ_MODBUS_REGISTR 0x03 // Read holding registers,
    //0x05 - Write single coil, 
  #define WRITE_MODBUS_REGISTR 0x06 //- Write single register,
    //0x0f - Write multiple coils,
    //0x10 - Write multiple registers,
    //0x0b - Get comm. event counter,
    //0x11 - Report slave ID

TasmotaSerial *MODBUS_Serial;
bool MODBUS_initialized = false; 
bool ModbusReceiveReady() {
  return (MODBUS_Serial->available() > 1);
}
bool use_modbus_tx_eneble_pin = false;

void MODBUS_Init() {
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG D_LOG_MODBUS D_START " init"));
  AddLog(LOG_LEVEL_INFO);
  if ((pin[GPIO_MODBUS_RX] < 99) && (pin[GPIO_MODBUS_TX] < 99))  {
    MODBUS_Serial = new TasmotaSerial(pin[GPIO_MODBUS_RX], pin[GPIO_MODBUS_TX], 0);
    if (MODBUS_Serial->begin(MODBUS_SPEED))   {
      if (MODBUS_Serial->hardwareSerial())   {
      ClaimSerial();
      }
      snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG D_LOG_MODBUS "inited TX/RX pin:%i/%i"), pin[GPIO_MODBUS_TX], pin[GPIO_MODBUS_RX]);
      AddLog(LOG_LEVEL_INFO);
      MODBUS_initialized = true;
    }
    if ((pin[GPIO_MODBUS_TX_ENABLE] < 99))  {
      pinMode(pin[GPIO_MODBUS_TX_ENABLE], OUTPUT);
      digitalWrite(pin[GPIO_MODBUS_TX_ENABLE], 0);
      #ifdef _LVA_DEBUG
        snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "SET MODBUS_TX_ENABLE pin:%i"), pin[GPIO_MODBUS_TX_ENABLE]);
        AddLog(LOG_LEVEL_INFO);
      #endif
    }
    MODBUS_initialized=true;
  }
  else {
    snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG D_LOG_MODBUS D_ERROR "TX and RX pins don't set"));
    AddLog(LOG_LEVEL_INFO);
  }
}

void ModbusSend16(uint8_t ModBusAdress, uint8_t function_code, uint16_t start_address, uint16_t register_count) {
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

  while (MODBUS_Serial->available() > 0)  { // read serial if any old data is available
    MODBUS_Serial->read();
  }

  MODBUS_Serial->flush();

  if ((pin[GPIO_MODBUS_TX_ENABLE] < 99)) digitalWrite(pin[GPIO_MODBUS_TX_ENABLE], 1);
  MODBUS_Serial->write(frame, sizeof(frame));
  if ((pin[GPIO_MODBUS_TX_ENABLE] < 99)) digitalWrite(pin[GPIO_MODBUS_TX_ENABLE], 0);

#ifdef MODBUS_DEBUG
  snprintf_P(log_data, sizeof(log_data), PSTR("%s\nModbusSend16 --> "), log_data);
  for (uint8_t i=0;i<8;++i){
    snprintf_P(log_data, sizeof(log_data), PSTR("%s%02X"), log_data, frame[i]);
    if (i < 7) {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s:"), log_data);
    }
  }
#endif //MOBUS_DEBUG
}
//sensor94 0 stop
uint8_t ModbusReceive(uint32_t *value, uint8_t mb_addr, uint8_t mb_function, uint16_t mb_reg, uint16_t answer_bytes) {
  uint8_t total_bytes;
  uint8_t th_bites;
  if (READ_MODBUS_REGISTR == mb_function)   { //READ_MODBUS_REGISTR
     total_bytes = answer_bytes + 5;
     th_bites = answer_bytes;
  } else if (WRITE_MODBUS_REGISTR==mb_function) { //WRITE_MODBUS_REGISTR
    total_bytes = 8;
    th_bites = (mb_reg >> 8) & 0xFF;
  }
  uint8_t buffer[total_bytes];
  *value = NAN;
  uint8_t len = 0;
  uint8_t ret = 0; // return code
  while (MODBUS_Serial->available() > 0)   {
    buffer[len++] = (uint8_t)MODBUS_Serial->read();
  }
  if (len != total_bytes) { 
    ret=3;        // FC51_ERR_NOT_ENOUGHT_BYTES
  } else {
    if (buffer[0] == mb_addr && buffer[1] == mb_function && buffer[2] == th_bites) { // check node number, op code and reply bytes count
      if ((MODBUS_CRC(buffer, len - 2)) == ((buffer[len - 1] << 8) | buffer[len - 2])) { //calculate crc from first len - 2 bytes and compare with received crc (bytes 7 & 8)
        for (uint8_t i = 0; i < answer_bytes; ++i) {
          ((uint8_t *)value)[i] = buffer[(total_bytes - 3) - i];
          //snprintf_P(log_data, sizeof(log_data), PSTR("%s i:%i -> %x "), log_data, i, buffer[(total_bytes - 2 - AnswerBytes) + i]);
        }
        ret=0; // FC51_ERR_NO_ERROR
      } else {
        ret=1; // FC51_ERR_CRC_ERROR
      }
    } else { // FC51_ERR_WRONG_BYTES
        ret=2;
    }
  }
  #ifdef MODBUS_DEBUG
    snprintf_P(log_data, sizeof(log_data), PSTR("%s Read bytes: %i <-- "), log_data, len);
    for (uint8_t i = 0; i < len; ++i)  {
      snprintf_P(log_data, sizeof(log_data), PSTR("%s%02X"), log_data, buffer[i]);
      if (i < (len-1))  {
        snprintf_P(log_data, sizeof(log_data), PSTR("%s:"), log_data);
      }
    }
    snprintf_P(log_data, sizeof(log_data), PSTR("%s ret:%i"), log_data, ret);
  #endif //MOBUS_DEBUG
  return ret;
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
#define FC51_CTW_WORDS 1 // Work ;)
#define FC51_STW_ADDR 50199
#define FC51_STW_WORDS 1 // work
#define FC51_MAX_FRQ_ADDR 3029 // Max Output Frequency param.3 - 03   int32 converter inex -3
#define FC51_MAX_FRQ_WORDS 2 // work не понял порядок
#define FC51_REF_ADDR 50009  // задание частоты через Bus reference register (REF)
#define FC51_REF_WORDS 1
#define FC51_MAV_ADDR 50209 // actual frequency 
#define FC51_MAV_WORDS 1
#define FC51_AOP_ADDR 16099 // actual Motor Power kW int16, converter inex -3
#define FC51_AOP_WORDS 1
#define FC51_100PERCENT 16384 // значние 100% заданного параметра
#define FC51_1PERCENT 163.84 // значние 1% заданного параметра
/*
struct ModbusDevices {
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
uint16_t REF[FC51_DEVICES]; // = 0;
uint16_t MAV[FC51_DEVICES];// = 0;     // Actual Output Frequency
uint16_t AOP[FC51_DEVICES];// = 0;     //Actual Output POWER W
uint8_t FC51_status = 0; // отвечает, зачение пишем и читаем по маске.
uint8_t FC51_Addr[FC51_DEVICES] = {FC51_1_ADDR, FC51_2_ADDR, FC51_3_ADDR};
uint8_t Modbus_repeat = 0; // номер  попытки чтения параметра
uint8_t FC51_Point=0; // номер команды, четный - запрос,  нечетный -чтение

#define FC51_READ_PARAMETERS 6
#define FC51_MAX_POINT 2*FC51_DEVICES*FC51_READ_PARAMETERS
#define MODBUS_MAX_REPEAT 3
#define MODBUS_COMMAND_REPEAT MODBUS_MAX_REPEAT *5 // для повышения вероятности достучатся. 1 команда бывает не проходит т.к. не всегда уудается отследить момент приходя команды и состояние буфера

void FC51_pooling()
{
  uint32_t value = 0;
  uint8_t error = 0;
  if (FC51_MAX_POINT == FC51_Point)   {
  //if (12 == FC51_Point)   {
  //if (2 == FC51_Point)   { // only CTW dev 0
    FC51_Point = 0;
  }
  uint8_t n_dev = FC51_Point / (FC51_READ_PARAMETERS * 2); // number of devises
  uint8_t n_com = (FC51_Point/2) % FC51_READ_PARAMETERS; // number of command
  // #ifdef DEBUG_MODBUS
  // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "FC51_Point:%i\t DEV:%i\t n_com:%i\t devices addr:%i\t repeat:%i\n"), FC51_Point, n_dev, n_com, FC51_Addr[n_dev], modbus_repeat);
  // Serial.print(log_data);
  // #endif //DEBUG_MODBUS

  switch (n_com) {
    case 0:       // ---------------------------- C T W ---------------------------------------------
      error = ModBusPool(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_CTW_ADDR, FC51_CTW_WORDS); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      } else if (99 == error) { // закончились попытки чтения, обнуляем параметр
        CTW[n_dev] = 0;
      } else if (0 == error) { // все хорошо можно читать следущий параметр
        CTW[n_dev] = (uint16_t)value;
        // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "DEV:%i, CTW:%x"), n_dev, CTW[n_dev]);
        // Serial.println(log_data);
      }
      break;
    case 1:  // ---------------------------- S T W ---------------------------------------------
      error = ModBusPool(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_STW_ADDR, FC51_STW_WORDS); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      } else if (0 == error || 99 == error) { // все хорошо
        STW[n_dev] = (uint16_t)value;
        // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "DEV:%i, STW:%x"), n_dev, STW[n_dev]);
        // Serial.println(log_data);    //AddLog(LOG_LEVEL_INFO);
      }
    break;
    case 2:   // ------------------ Max Output Frequency param. (MAX_FRQ) 3-03 --------------------------------------------
      error = ModBusPool(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_MAX_FRQ_ADDR, FC51_MAX_FRQ_WORDS); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      } else if (0 == error || 99 == error) { // все хорошо
        MAX_FRQ[n_dev] = value; // -3 степень
        // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "DEV:%i, MAX_FRQ: %i Hz"), n_dev, MAX_FRQ[n_dev]/1000);
        // Serial.println(log_data);        //AddLog(LOG_LEVEL_INFO);
      }
      break;
    case 3:       // ------------------ Actual Output Frequency (MAV) --------------------------------------------
      error = ModBusPool(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_MAV_ADDR, FC51_MAV_WORDS); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      } else if (0 == error || 99 == error) { // все хорошо
        MAV[n_dev] = value;
        // float mav_Hz = MAV[n_dev] * MAX_FRQ[n_dev] / 1000 / FC51_100PERCENT;
        // char mav_Hzs[7];
        // dtostrfd(mav_Hz, 2, mav_Hzs);
        // float mav_Pc = MAV[n_dev] / FC51_1PERCENT;
        // char mav_Pcs[7];
        // dtostrfd(mav_Pc, 2, mav_Pcs);
        // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "DEV:%i, Actual Out Freq: %s%% / %s Hz(%d)"), n_dev, mav_Pcs, mav_Hzs, REF[n_dev]);
        // Serial.println(log_data);
      }
      break;
    case 4:      // ------------------ Actual Output POWER W (AOP) --------------------------------------------
      error = ModBusPool(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_AOP_ADDR, FC51_AOP_WORDS); //  Status Word
      if (10 == error) { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      } else if (0 == error || 99 == error)   { // все хорошо
        AOP[n_dev] = value; //  -3 степень
        // uint16_t aop = AOP[n_dev]/1000;
        // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "DEV:%i, Actual Out POWER W: %i(%i)"), n_dev, aop, AOP[n_dev]);
        // Serial.println(log_data);
      }
      break;
    case 5: // ------------------ REFERANCY FRQ (REF) --------------------------------------------
      error = ModBusPool(&FC51_Point, &value, FC51_Addr[n_dev], READ_MODBUS_REGISTR, FC51_REF_ADDR, FC51_REF_WORDS); //  Status Word
      if (10 == error)  { // послали запрос на чтение, ничего делать не надо просто выходим
        break;
      } else if (0 == error || 99 == error) { // все хорошо
        REF[n_dev] = value;
        // float ref_Hz = REF[n_dev] * MAX_FRQ[n_dev] / 1000 / FC51_100PERCENT;
        // char ref_Hzs[7];
        // dtostrfd(ref_Hz, 2, ref_Hzs);
        // float ref_Pc = REF[n_dev] / FC51_1PERCENT;
        // char ref_Pcs[7];
        // dtostrfd(ref_Pc, 2, ref_Pcs);
        // snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "DEV:%i, REF: %s%% / %s Hz(%d)"), n_dev, ref_Pcs, ref_Hzs, REF[n_dev]);
        // Serial.println(log_data);
      }
      break;
  }
//   // определяем статус устройств
//   for (uint8_t dev=0; dev<FC51_DEVICES; ++dev){
//     if (STW[dev] == 0 && CTW[dev]==0){
//       FC51_status |= (1 << dev);// устанавливаем бит
//     } else {
//       FC51_status &= ~(1 << dev);  // сбрасываем  бит
//     }
//   }
} // FC51_pooling()
// =======================================

uint8_t ModBusPool(uint8_t *point, uint32_t *p_value, uint8_t m_address, uint8_t m_function, uint16_t m_registr, uint16_t m_bytes) {
  uint8_t error = 0;  // то что будем возвращать
  snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_FC51 "ModBusPool:{Point:%i, Repeat:%i\tADR:%i\t REG:%i\t"), *point, Modbus_repeat, m_address, m_registr);
  if (!(*point & 1)) {  // если четный  то пытаемся сделать запрос PointRead evene - request
    if (Modbus_repeat < MODBUS_MAX_REPEAT)    { // если не закончились попытки запрашиваем
      ModbusSend16(m_address, m_function, m_registr, m_bytes);
      // snprintf_P(log_data, sizeof(log_data), PSTR("%s, Wait answer}"), log_data);
      // Serial.println(log_data);
      ++*point; // переходим к чтению
      return 10; // послали запрос надо читать следующий раз
    }
    else { // если закончились попытки
      Modbus_repeat = 0; // обнуляем попытки
      ++*point;
      ++*point;   // надо перескочить чтение, сразу делаемзапрос следующего
      *p_value=0; // обнуляем буфер со значением
      // snprintf_P(log_data, sizeof(log_data), PSTR("%s MAX_REPEAT"), log_data);
      // Serial.println(log_data);
      return 99;  // закончилось число попыток чтения
    }
  } else {// не четный, читаем что получилось PointRead odd - revieve
    if (MODBUS_Serial->available() == 0)  {// проверяем, что что-то есть приемном буфере
      // snprintf_P(log_data, sizeof(log_data), PSTR("%s Not data in buffer\t"), log_data);
      error = 4;
    } else { // если что-то есть в буфере читаем
      error = ModbusReceive(p_value, m_address, m_function, m_registr, m_bytes * 2);
    }
    if (error) {
      --*point; // все плохо повторяем чтение
      ++Modbus_repeat; // инкрементируем номер попытки
      // snprintf_P(log_data, sizeof(log_data), PSTR("%s Error read:%i"), log_data, error);
    } else {
      ++*point; // все хорошо переходим к следующему параметру
      // snprintf_P(log_data, sizeof(log_data), PSTR("%s Read OK, Value:%i"), log_data, *p_value);
      Modbus_repeat = 0;
    }
    // snprintf_P(log_data, sizeof(log_data), PSTR("%s}"), log_data);
    // Serial.println(log_data); //AddLog(LOG_LEVEL_DEBUG);
    return error;
  }
} // ModBusPool

void log_comma(bool comma) {
  if (comma)
    snprintf_P(log_data, sizeof(log_data), PSTR("%s, "), log_data);
} //log_comma

char *FC51_Decode_STW(uint8_t n) {
  snprintf_P(log_data, sizeof(log_data), PSTR("")); // clear !!!
  //snprintf_P(log_data, sizeof(log_data), PSTR("{"));
  if (!STW[n]) {
    snprintf_P(log_data, sizeof(log_data), PSTR("%s NOT DATA"), log_data);
  } else  {
    bool comma = false;
    if (!(STW[n] & (1 << 0))) {
      snprintf_P(log_data, sizeof(log_data), PSTR("%sControl not ready"), log_data);
      comma = true;
    }
    if (!(STW[n] & (1 << 1))) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sUnit not ready"), log_data);
      comma = true;
    }
    // if (STW[n] &(1 << 2)) {
    //   snprintf_P(log_data, sizeof(log_data), PSTR("%s, Coasted"), log_data);
    // }
    if (STW[n] & (1 << 3)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%s" D_ERROR " tripped"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 4)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%s" D_ERROR " no trip"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 6))    {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%s" D_ERROR " trip locked"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 7))    {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sWARNING"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 8) && !(STW[n] & (1 << 11))) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sOn reference"), log_data);
      comma = true;
    }
    else if (!(STW[n] & (1 << 8)) && STW[n] & (1 << 11)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sNot at reference"), log_data);
      comma = true;
    }
    if (!(STW[n] & (1 << 9))) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sHand mode"), log_data);
      comma = true;
    }
    if (!(STW[n] & (1 << 10)) && STW[n] & (1 << 11)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sOut of freq. range"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 11)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%s" D_ON), log_data);
      comma = true;
    }
    else {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%s" D_OFF), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 12)) {
      log_comma(comma); 
      snprintf_P(log_data, sizeof(log_data), PSTR("%sResistor brake fault"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 13)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sVoltage warning"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 14)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sCurrent limit"), log_data);
      comma = true;
    }
    if (STW[n] & (1 << 15)) {
      log_comma(comma);
      snprintf_P(log_data, sizeof(log_data), PSTR("%sThermal warning"), log_data);
    }
  }
  //snprintf_P(log_data, sizeof(log_data), PSTR("%s}"), log_data);
//  Serial.println(log_data);//AddLog(LOG_LEVEL_INFO);
  return log_data;
}

// for i18.h
#define D_FC51 "FC"
#define D_CTW "CTW"
#define D_STW "STW"
#define D_STW_D "STWdecode"
#define D_MAX_FRQ "MaxFRQ"
#define D_MAV "FRQ"
#define D_AOP "POWER"
#define D_REF "ReferFRQ"
#define D_STATUS "STATUS"

#ifdef USE_WEBSERVER
const char HTTP_FC51_STR_4[] PROGMEM =  "%s" "{s}" D_FC51 ":%i %s:" "{m}%s"        "{e}"; // 4 переменных
const char HTTP_FC51_STR_4R[] PROGMEM = "%s" "{s}" D_FC51 ":%i %s:" "{mr}%s"       "{e}"; // 4 переменных
const char HTTP_FC51_STR_4G[] PROGMEM = "%s" "{s}" D_FC51 ":%i %s:" "{mg}%s"       "{e}"; // 4 переменных
const char HTTP_FC51_STR_7[] PROGMEM =  "%s" "{s}" D_FC51 ":%i %s:" "{m}%s%s/%s%s" "{e}"; // 7 переменных
const char HTTP_FC51_INT[] PROGMEM =    "%s" "{s}" D_FC51 ":%i %s:" "{m}%i %s"     "{e}"; // 5 переменных "FC51_1 STW:111 Hz"
const char HTTP_FC51_MODBUS_ERR[] PROGMEM = "%s" "{s}MODBUS" "{m}%s" "{e}"; // 2 переменных 
#endif  // USE_WEBSERVER

void FC51_Show(boolean json) {
  if (json) {
    snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s,"), mqtt_data); // onli first comma 
    for (uint8_t n_dev = 0; n_dev < FC51_DEVICES; ++n_dev) {
// надо переделать на 1 переменную
      float mav_Pc = MAV[n_dev] / FC51_1PERCENT; //MAV
      char mav_Pcs[7];
      dtostrfd(mav_Pc, 2, mav_Pcs);
      float ref_Pc = REF[n_dev] / FC51_1PERCENT; // REF
      char ref_Pcs[7];
      dtostrfd(ref_Pc, 2, ref_Pcs);
      char status[4];
      if (STW[n_dev] & (1 << 11))    {  // STATUS
        snprintf_P(status, sizeof(status), PSTR(D_ON));
      } else {
        snprintf_P(status, sizeof(status), PSTR(D_OFF));
      }
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s\"" D_FC51 "%i\":{\"" D_STATUS "\":\"%s\",\"" D_CTW "\":%i,\"" D_STW "\":%i,\"" D_MAX_FRQ "\":%i,\"" D_REF "\":%s,\"" D_MAV "\":%s,\"" D_AOP "\":%i,\"" D_STW_D "\":\"%s\"}"),
                 mqtt_data, (n_dev + 1), status, CTW[n_dev], STW[n_dev], MAX_FRQ[n_dev] / 1000, ref_Pcs, mav_Pcs, AOP[n_dev], FC51_Decode_STW(n_dev));
      if (n_dev < (FC51_DEVICES - 1)) {
        snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s%s"),mqtt_data,",");
      }
    }

#ifdef USE_WEBSERVER
  } else {
    if (MODBUS_initialized){
      for (uint8_t n_dev = 0; n_dev < FC51_DEVICES; ++n_dev) {
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, (n_dev + 1), D_CTW, CTW[n_dev], "");
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, (n_dev + 1), D_STW, STW[n_dev], "");
        if (STW[n_dev] & (1 << 11)) { //STATUS
          snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_STR_4R, mqtt_data, (n_dev + 1), D_STW_D, FC51_Decode_STW(n_dev), "");
        } else {
          snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_STR_4G, mqtt_data, (n_dev + 1), D_STW_D, FC51_Decode_STW(n_dev), "");
        }
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, (n_dev + 1), D_MAX_FRQ, MAX_FRQ[n_dev] / 1000, D_UNIT_HERTZ);
        float f_float1 = MAV[n_dev] * MAX_FRQ[n_dev] / 1000 / FC51_100PERCENT;        //MAV to Hz
        char s_float1[7];
        dtostrfd(f_float1, 2, s_float1); //mav_Hzs
        float f_float2 = MAV[n_dev] / FC51_1PERCENT;
        char s_float2[7];
        dtostrfd(f_float2, 2, s_float2); //mav_pcs
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_STR_7, mqtt_data, (n_dev + 1), D_MAV, s_float2, "\%", s_float1, D_UNIT_HERTZ);
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_INT, mqtt_data, n_dev + 1, D_AOP, AOP[n_dev], D_UNIT_WATT);
        f_float1 = REF[n_dev] * MAX_FRQ[n_dev] / 1000 / FC51_100PERCENT; // REF to Hz
        dtostrfd(f_float1, 2, s_float1); // ref_Hzs
        f_float2 = REF[n_dev] / FC51_1PERCENT;
        dtostrfd(f_float2, 2, s_float2); // ref_Pcs
        snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_STR_7, mqtt_data, (n_dev + 1), D_REF, s_float2, "\%", s_float1, D_UNIT_HERTZ);
      }
    } else {
      snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_FC51_MODBUS_ERR, mqtt_data,"don't set TX and RX Pins");
    }
#endif  // USE_WEBSERVER
  }
}

/*********************************************************************************************\
 * START/STOP
 * "SENSOR94,N,command" or "SENSOR94 N command"
 *    - command: STOP/START or OFF/ON or 1/2  sensor94 0,stop
 *    - N -  1...3  (ноль не может быть иначе неработае функция atoi)
 *    example: sensor94,1,stop   sensor94 1 OFF   sensor94,1,1
 * "SENSOR94,N,SPEED FRQ"
      FRQ - speed  from 0...100  persent
\*********************************************************************************************/
#define D_SPEED "SPEED"
const char FC_STATUS[] PROGMEM = "FC%i_STATUS"; // 1 переменая
const char FC_SPEED[] PROGMEM = "FC%i_SPEED";   // 1 переменая    <--- "FC%i_" D_SPEED;
//#define D_ERROR "ERROR"

bool FC51Command() {
  boolean serviced = false; // переменная для return
  uint8_t paramcount = 0; // количестdо полученных в команде параметров
  uint8_t dev_id = 99; // на 1 больше n_dev
  if (XdrvMailbox.data_len > 0) { // проверяем что есть парамаметры
    paramcount = 1;
  } else {
    return serviced;
  }
  char sub_string[XdrvMailbox.data_len];
  for (uint8_t ca=0;ca<XdrvMailbox.data_len;ca++) { // определяем количество полученных параметров
    if ((' ' == XdrvMailbox.data[ca]) || ('=' == XdrvMailbox.data[ca])) { XdrvMailbox.data[ca] = ','; }
    if (',' == XdrvMailbox.data[ca]) { paramcount++; }
  }
  //
  // Serial.print("FC51 Command.paramcount:"); //-
  // Serial.println(paramcount, DEC); // -
  UpperCase(XdrvMailbox.data, XdrvMailbox.data); // все сделали строковыми
   
  if (FC51_Point & 1) { //ПРОВЕРКА ТЕКУШИЙ СТАТУС ОПРОСА СОСТОЯНИЯ FC51_pooling, сделали запрос но не прочитали  
    FC51_pooling();
  }

  char suffics[11];
  if (paramcount > 1) { // если команд больше 1, то вторым (от 0) номер устройства, в индексе 0 идентификатор сервиса
    Settings.flag.mqtt_response=1; // разрешаем перезапись префикса
    //FC1_STATUS
    dev_id = atoi(subStr(sub_string, XdrvMailbox.data, ",", 1)); // Function to return a substring defined by a delimiter at an index char *subStr(char *dest, char *str, const char *delim, int index)
    if (dev_id > FC51_DEVICES || dev_id == 0) { // проверка на правильный  номер устройства
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "\t" D_ERROR ": Number devices not detected\""));
      return serviced;
    }
    if (CTW[dev_id-1]==0) { // проверка что устройство отзывается
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_ERROR ": devices " D_OFFLINE "\""), dev_id);
      return serviced;
    }
    if (paramcount == 2) { //  sensor94 1 start 
      snprintf_P(suffics, sizeof(suffics), FC_STATUS, dev_id); //меняем MQTT суфикс на FC1_STATUS
      uint16_t start_stop = CTW[dev_id-1]; 
      char c_str[7]; //6
      snprintf_P(c_str, sizeof(c_str), subStr(sub_string, XdrvMailbox.data, ",", 2));
      // Serial.print("c_str:[");                     //-
      // Serial.print(c_str);                         //-
      // Serial.println("]");                         //-
      if (0==strcmp(c_str, D_START) || 2 == atoi(c_str) || 0==strcmp(c_str, "ON")) { // D_ON-------- здесь включаем   sensor94 1,START    sensor94 1,2 sensor94 1,ON sensor94 1,on
        // Serial.println("-start-"); //
        start_stop |= (1 << 6);// пример Var |= (1 << 3) | (1 << 5);
      } else if (0==strcmp(c_str, D_STOP) || 1 == atoi(c_str) || 0==strcmp(c_str, "OFF")) {// D_OFF//  sensor94 1,STOP  sensor94 1,stop, sensor94 0,OFF sensor94 1 1 sensor94 0,qqqqq
        // Serial.println("-stop-"); //
        start_stop &= ~(1<<6);  // сбрасываем 6 бит, пример Var &= ~((1 << 2) | (1 << 6));
      } else if (0==strcmp(c_str, "TOGGLE") || 3 == atoi(c_str)) {// //  sensor94 1,TOGGLE  sensor94 1,TOGGLE, sensor94 0,TOGGLE sensor94 1 1 sensor94 0,qqqqq
        start_stop ^= (1 << 6);      //TOGGLE  мнверсия бита
      } else {
        // Serial.println("-error-"); //
        snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_ERROR ":" D_LOG_COMMAND "<%s>\""), dev_id, c_str);
        return serviced;
      }
      // Serial.print("to ModbusWrReg start_stop:\t");  // debug
      // Serial.println(start_stop); // debug
      if (start_stop != CTW[dev_id-1]) {
        serviced = ModbusWrReg(FC51_Addr[dev_id-1], WRITE_MODBUS_REGISTR, FC51_CTW_ADDR, start_stop, FC51_CTW_WORDS);
      } else {
        serviced=true;
      }
      if (serviced) {
        snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s"), c_str);
        //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"" D_FC51 "%i_STATUS\":\"%s\"}}"), dev_id, c_str);
      } else {
        //+snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("ERROR"));
      }
      MqttPublishPrefixTopic_P(RESULT_OR_TELE, suffics, 3);
      mqtt_data[0] = '\0';
      return serviced;
    }
  }
  if (paramcount > 2) {// sensor94 1 SPEED 20    sensor94 1 SPEED 29 sensor94 1 SPEED 100
    if (0==strcmp(subStr(sub_string, XdrvMailbox.data, ",", 2), D_SPEED)) { // если задается скорость в %
      uint16_t new_REF = atoi(subStr(sub_string, XdrvMailbox.data, ",", 3));
      if (new_REF <= 100) {// праверяем правильность скорости в %
        if (new_REF == 0) { //если выключили
          if ((CTW[dev_id-1] & (1 << 6))) { // если  включен 
            uint16_t start_stop = CTW[dev_id-1];
            start_stop &= ~(1 << 6); // сбросили бит
            serviced = ModbusWrReg(FC51_Addr[dev_id-1], WRITE_MODBUS_REGISTR, FC51_CTW_ADDR, start_stop, FC51_CTW_WORDS);
            if (serviced) {
              //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"" D_FC51 "%i\":{\"STATUS\":\"%s\",\"SPEED\":0}}"), dev_id, "OFF");
              snprintf_P(suffics, sizeof(suffics), FC_STATUS, dev_id); //меняем MQTT суфикс на FC1_STATUS
              snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR(D_OFF));   // FC1_STATUS OFF
              MqttPublishPrefixTopic_P(RESULT_OR_TELE, suffics, 3);
              mqtt_data[0] = '\0';
              snprintf_P(suffics, sizeof(suffics), FC_SPEED, dev_id);     //меняем MQTT суфикс на FC1_SPEED
              snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("0"));       // FC1_SPEED 0
              MqttPublishPrefixTopic_P(RESULT_OR_TELE, suffics, 3);
              mqtt_data[0] = '\0';
            } else {
              snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_SPEED ": " D_ERROR "\""), dev_id);
            }
          } else {
            snprintf_P(suffics, sizeof(suffics), FC_STATUS, dev_id); //меняем MQTT суфикс на FC1_STATUS
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR(D_OFF));   // FC1_STATUS OFF
            MqttPublishPrefixTopic_P(RESULT_OR_TELE, suffics, 3);
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_SPEED ": Was STOPPED!\""), dev_id);
            //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"" D_FC51 "%i\":{\"SPEED\":\"Was STOPPED!\",\"STATUS\":\"OFF\"}}"), dev_id); //
            serviced = true;
          }
        } else { // -------- здесь устанавливаем скорость   sensor94 1 SPEED 1 sensor94 1 SPEED 0    sensor94 1 SPEED 31     sensor94 1 SPEED 100
          uint16_t fc_ref = new_REF * FC51_1PERCENT;
          // snprintf_P(log_data, sizeof(log_data), PSTR("new_REF: %i  <-->  REF:%i "), new_REF, REF[dev_id]);
          // Serial.println(log_data);
          if (fc_ref != REF[dev_id-1] || !(CTW[dev_id-1] & (1 << 6))) { // проверяем что она была изменена или не включен
            serviced = ModbusWrReg(FC51_Addr[dev_id - 1], WRITE_MODBUS_REGISTR, FC51_REF_ADDR, fc_ref, FC51_REF_WORDS);
          } else {
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_SPEED ": Not changed!\""), dev_id);
            serviced=true;
          }
          // snprintf_P(log_data, sizeof(log_data), PSTR("serviced: %i"), serviced);
          // Serial.println(log_data);
          if (serviced) {
            //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"" D_FC51 "%i\":{\"SPEED\":%i,\"STATUS\":"), dev_id, new_REF);
            snprintf_P(suffics, sizeof(suffics), FC_SPEED, dev_id); //меняем MQTT суфикс на FC1_SPEED
            snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%i"), new_REF); // FC1_SPEED 34 (new_REF)
            MqttPublishPrefixTopic_P(RESULT_OR_TELE, suffics, 3);
            snprintf_P(suffics, sizeof(suffics), FC_STATUS, dev_id); //меняем MQTT суфикс на FC1_STATUS
            if (!(CTW[dev_id-1] & (1 << 6)))   { // если прошла  предварительная команда и выключен 
              uint16_t start_stop = CTW[dev_id-1];
              start_stop |= (1 << 6);
              //Serial.println("Ready to On");       //-
              serviced = ModbusWrReg(FC51_Addr[dev_id-1], WRITE_MODBUS_REGISTR, FC51_CTW_ADDR, start_stop, FC51_CTW_WORDS);  //  -- отключаем на время отладки
              if (serviced) {
                //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s\"%s\"}}"), mqtt_data, "ON");
                snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR(D_ON)); // FC1_STATUS OFF
              } else {
                //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s\"%s\"}}"), mqtt_data, "OFF");
                snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR(D_OFF));   // FC1_STATUS OFF
              }
            } else {
              //snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s\"%s\"}}"), mqtt_data, "ON");
              snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR(D_ON)); // FC1_STATUS OFF
            }
            MqttPublishPrefixTopic_P(RESULT_OR_TELE, suffics, 3);
            mqtt_data[0] = '\0';
          }
        }
        if (!serviced) { // проверяем что все закончиось хорошо
          // snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"FC51\":{\"ID_%i\":{\"SPEED\":\"%d\"}}}"), dev_id, new_REF);
          snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_SPEED ":" D_ERROR "\""), dev_id);
        }
      } else {
        snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_SPEED ": OUT RANGE %i\""), dev_id, new_REF);
      } // <100
      return serviced;
    } else { // команада не распознана
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("\"" D_FC51 "%i \t" D_LOG_COMMAND ": " D_ERROR "\""), dev_id);
    }// SPEED
  }   // paramcount > 2
  return serviced;
} // FC51Command()
// mqtt_data[0] = '\0';


boolean ModbusWrReg(uint8_t s_address, uint8_t s_function, uint16_t s_reg, uint16_t s_data, uint8_t s_words) { //  sensor94 0 stop  sensor94 1 SPEED 1  sensor94 1 SPEED 0
  uint8_t repeat = 1;
  bool status = false;
  // char temp_[150]; // ВНИМАНИЕ Глючит, нехватает памяти для стека
  // snprintf_P(temp_, sizeof(temp_), PSTR("\nModbusWrReg FC51_Point:%i"), FC51_Point);
  if (FC51_Point & 1) { //ПРОВЕРКА ТЕКУШИЙ СТАТУС ОПРОСА СОСТОЯНИЯ FC51_pooling, сделали запрос но не прочитали
    // snprintf_P(temp_, sizeof(temp_), PSTR("%s\t\t RUN FC51_pooling!!!!!!!!!"), temp_);
    FC51_pooling();
  } else {
    if (MODBUS_Serial->available() > 0) { // read serial if any old data is available
      // snprintf_P(temp_, sizeof(temp_), PSTR("%s\t\t HAVE Not READ data!!!!!!!!!"), temp_);
    }
  }
    // Serial.println(temp_);
  log_data[0] = '\0'; // clear !!! /
  while (repeat <= MODBUS_COMMAND_REPEAT && status == false ) {
    while (MODBUS_Serial->available() > 0) { // read serial if any old data is available
      MODBUS_Serial->read();
    }
    ModbusSend16(s_address, s_function, s_reg, s_data);
    // snprintf_P(log_data, sizeof(log_data), PSTR("%s\nRepeat %i set command to %i"), log_data, repeat, s_address);
    // Serial.println(log_data);
    #define RDELAY 800000 // 700000
    uint16_t delay_read = RDELAY / MODBUS_SPEED + RDELAY / (MODBUS_SPEED*7) * repeat;
    delay(delay_read); 
    uint32_t r_data;
    uint8_t r_bytes=s_words * 2;
    uint8_t error = ModbusReceive(&r_data, s_address, s_function, s_reg, r_bytes);
    //Serial.println(log_data);
    if (s_data == (uint16_t)r_data) {
      // snprintf_P(log_data, sizeof(log_data), PSTR("\nSENT and RECIEVED COMMAND to ADDR:%i Valid"), s_address);
      // Serial.println(log_data);
      status = true;
    }
    else {
      Serial.println(log_data);
      snprintf_P(log_data, sizeof(log_data), PSTR("\n" D_COMMAND "NOT Valid: ADDR:%i\tFUNC:%i\tREG:%i\tRepeat:%i\tdelay_read:%i\t TX/RX DATA:%i/%i"), s_address, s_function, s_reg, repeat, delay_read, s_data, r_data);
      Serial.println(log_data);
    }
    ++repeat;
  }
  return status;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

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
      case FUNC_EVERY_250_MSECOND: //50 100 250
        if (MODBUS_initialized) {
          FC51_pooling();
        }
        break;
      case FUNC_EVERY_SECOND:
        if (MODBUS_initialized) {
          //FC51_pooling();
        }
        break;
      case FUNC_JSON_APPEND:
        FC51_Show(1);
        break;
      case FUNC_COMMAND:
        if (MODBUS_initialized) {
          if (XSNS_94 == XdrvMailbox.index)  {
            result = FC51Command();
          }
        }
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_APPEND:
        if (MODBUS_initialized) {
          FC51_Show(0);
        }
        break;
#endif  // USE_WEBSERVER
    }
  //} // MODBUS_initialized
  return result;
}
#endif   // USE_FC51
#endif   //USE_MODBUS
