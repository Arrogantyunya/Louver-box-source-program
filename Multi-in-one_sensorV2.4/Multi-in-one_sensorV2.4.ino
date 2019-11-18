
#include "AsyncDelay.h"
#include"Adafruit_TSL2561_U.h"
#include <SoftwareSerial.h>
#include "SoftwareI2C.h"
#include "HTU21D.h"
#include "bmp180.h"
#include "SparkFunCCS811.h"
#include <EEPROM.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "i2c.h"
#include "i2c_MAX44009.h"

#define BUFFERSIZE          255  //The maximum number of bytes of a frame data

#define EN485_DE            4

//CO2 and TVOC
#define CCS811_ADDR         0x5A
CCS811 CCS811_Sensor(CCS811_ADDR);

//485 Serial port
SoftwareSerial ModBus_Serial(2, 3);

//Lux and UV
SoftwareI2C CJMCU6750;
MAX44009 max44009;
//Integration Time
#define IT_1_2    0x0 //1/2T
#define IT_1      0x1 //1T
#define IT_2      0x2 //2T
#define IT_4      0x3 //4T
#define I2C_ADDR  0x38

//Temperature and humidity
HTU21D myHumidity; 

//Function declaration
void Enable_ModBus(void);
void Disable_ModBus();
void Debug_Sensor(void);
void Receive_ModBus_Cmd_and_Parse(void);
void Send_ModBus(void);
void Change_SlaveID(void);
void Change_BaudRate(void);
void Set_RS485_BaudRate_List(void);
void TSL_displaySensorDetails(void);
void TSL_configureSensor(void);
void Get_illumination_and_UV(void);
void Get_Temperature_and_humidity(void);
void BMP_calculate(void);
void Get_Pressure(void);
void Get_Altitude(void);
void Get_CO2_and_TVOC(void);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
unsigned int Calculate_CRC(unsigned char* _regs, unsigned char arraySize);

//Some global variables of bus protocol communication about ModBus
unsigned char g_Receive_Frame[BUFFERSIZE] = {0}; //Used to store received data
bool g_Receive_Frame_OK_Flag = false;
unsigned char g_Sensor_Data_Buffer[28]  = {0}; //Save sensor value
unsigned int  g_RS485_BaudRate;
unsigned char g_BaudRate_Value;

// Some global variables of BMP180 sensor 
float g_temperature = 0.0;  
double g_pressure   = 0.0;   
double g_pressure2  = 0.0;     
double g_altitude   = 0.0;

void setup() {

  Serial.begin(9600); 

  Set_RS485_BaudRate_List();

  pinMode(EN485_DE, OUTPUT);
  Disable_ModBus(); //Receive mode.

  //CJMCU6750 sensor initialization.
  CJMCU6750.begin(11, 12);
  CJMCU6750.beginTransmission(I2C_ADDR);
  CJMCU6750.write((IT_1<<2) | 0x02);
  CJMCU6750.endTransmission();
  delay(500);

  if (max44009.initialize()) 
    Serial.println("Sensor MAX44009 found");
  else
    Serial.println("Light Sensor missing");   

  //HTU21 sensor initialization.  
  myHumidity.begin();

  //BMP180 sensor initialization.
  Bmp_begin();
  OSS = 2;
  BMP180start();

  //CCS811 sensor initialization.
  CCS811_Sensor.begin();
  CCS811_Sensor.setDriveMode(1);

  //Verify ModBus address.
  if (EEPROM.read(0) == 0x05 || EEPROM.read(0) == 0x08)
    ;
  else
    EEPROM.write(0, 5);

  Serial.println("Input 'S', print sensor data...");
}

void loop() 
{
  Receive_ModBus_Cmd_and_Parse(); //Receive ModBus commomd.
  Debug_Sensor();
}

/*
 *brief   : R/W ModBus enable function
 *para    : None
 *return  : None
 */
void Enable_ModBus(void)
{
  digitalWrite(EN485_DE, HIGH);
  delay(25);
}

void Disable_ModBus(void)
{
  delay(100);
  digitalWrite(EN485_DE, LOW);
}

void Debug_Sensor(void)
{
  char c;
  while (Serial.available() > 0){
    c = Serial.read();

    if (Serial.available() == 0){
      if (c == 'S' || c == 's'){
        /*Totol collect 7 meteorological data*/
        Get_illumination_and_UV();  //Collect Lux and UV
        Get_Temperature_and_humidity(); //Collect temperature and humidity
        Get_Pressure(); //Collect air pressure.
        Get_CO2_and_TVOC(); //Collect CO2 and TVOC
      }else{
        Serial.println("Input ERROR, please input 'S'");
      }
    }
  }
}

/*
 *brief   : Receive ModBus instruction and parse
 *para    : None
 *return  : None
 */
void Receive_ModBus_Cmd_and_Parse(void)
{
  unsigned int CharacterTime; //字节时间检测
  unsigned int CRC16, CRC16_Temp;
  unsigned char Receive_Length = 0;

  if (g_RS485_BaudRate > 19200) 
    CharacterTime = 750; 
  else
    CharacterTime = 15000000 / g_RS485_BaudRate;  //1.5字节时间

  while(ModBus_Serial.available() > 0){

    if(Receive_Length < BUFFERSIZE)  
      g_Receive_Frame[Receive_Length++] = ModBus_Serial.read();
    else  
       ModBus_Serial.read();  //不满足条件，清空

    delayMicroseconds(CharacterTime);  //等待1.5个字节

    if(ModBus_Serial.available() == 0){  

      CRC16_Temp = Calculate_CRC(g_Receive_Frame, 6);
      CRC16 = ((g_Receive_Frame[6] << 8) | g_Receive_Frame[7]);

      unsigned char Slave_id = EEPROM.read(0);

      if (Slave_id == g_Receive_Frame[0]){
        if (CRC16_Temp == CRC16){
          g_Receive_Frame_OK_Flag = true;

        }else{
          Serial.println("CRC16 ERROR !!! <Receive_ModBus_Cmd_and_Parse>");
          Serial.println(CRC16_Temp, HEX);
          Serial.println(CRC16, HEX);
          g_Receive_Frame_OK_Flag = false;
        }
      }else{
        Serial.println("Slave ID ERROR !!! <Send_ModBus>");
        Serial.print("Received slave ID is : ");
        Serial.println(g_Receive_Frame[0]);
      }
    }
  }

  if (g_Receive_Frame_OK_Flag == true){
    switch (g_Receive_Frame[1]){
      case 0x03 : Send_ModBus();    g_Receive_Frame_OK_Flag = false; break;
      case 0x06 : 
                  if (g_Receive_Frame[2] == 1){
                    switch (g_Receive_Frame[3]){
                      case 0x01 : Change_SlaveID();   g_Receive_Frame_OK_Flag = false; break;
                      case 0x02 : Change_BaudRate();  g_Receive_Frame_OK_Flag = false; break;  
                    }
                  }
                  break;
    }
    memset(g_Receive_Frame, 0x00, sizeof(g_Receive_Frame));
  }
}

/*
 *brief   : Packaging and sending packets in ModBus format according to the received ModBus instructions
 *para    : None
 *return  : None
 */
void Send_ModBus(void)
{
  unsigned char Send_Frame_Buffer[BUFFERSIZE] = {0};
  unsigned char Send_Length = 0;
  unsigned char Start_addr = 0, End_addr = 0, Valid_addr = 0;
  unsigned char Data_Length = 0;
  unsigned int  CRC16 = 0;
  
  unsigned char Slave_id = EEPROM.read(0);
  
  if (g_Receive_Frame[5] != 0){
    /*Totol collect 7 meteorological data*/
    Get_illumination_and_UV();  //Collect Lux and UV
    Get_Temperature_and_humidity(); //Collect temperature and humidity
    Get_Pressure(); //Collect air pressure.
    Get_CO2_and_TVOC(); //Collect CO2 and TVOC

    Data_Length = g_Receive_Frame[5] * 2;

    Start_addr = g_Receive_Frame[3] * 2;
    End_addr   = Start_addr + Data_Length;
    Valid_addr = End_addr - Start_addr;

    Send_Frame_Buffer[Send_Length++] = g_Receive_Frame[0];
    Send_Frame_Buffer[Send_Length++] = g_Receive_Frame[1];
    Send_Frame_Buffer[Send_Length++] = Valid_addr;

    for (unsigned char i = Start_addr; i < End_addr; i++)
      Send_Frame_Buffer[Send_Length++] = g_Sensor_Data_Buffer[i];

    CRC16 = Calculate_CRC(&Send_Frame_Buffer[0], Send_Length);

    Send_Frame_Buffer[Send_Length++] = CRC16 >> 8;
    Send_Frame_Buffer[Send_Length++]   = CRC16 & 0x00FF;

    Enable_ModBus();
    ModBus_Serial.write(&Send_Frame_Buffer[0], Send_Length);
    Disable_ModBus();
  }
}

/*
 *brief   : Modify slave ID according to ModBus instruction
 *para    : None
 *return  : None
 */
void Change_SlaveID(void)
{
  unsigned char Frame_ID_Buffer[8] = {0};

  if (g_Receive_Frame[5] <= 255){
    unsigned char Slave_id = EEPROM.read(0);
    unsigned char New_Flave_id = g_Receive_Frame[5];
    Frame_ID_Buffer[0] = Slave_id;

    for (unsigned char i = 1; i < 6; i++)
      Frame_ID_Buffer[i] = g_Receive_Frame[i];

    unsigned int CRC16 = Calculate_CRC(&Frame_ID_Buffer[0], 6);
    Frame_ID_Buffer[6] = CRC16 >> 8;
    Frame_ID_Buffer[7] = CRC16 & 0x00FF;

    Enable_ModBus();
    ModBus_Serial.write(&Frame_ID_Buffer[0], 8);
    Disable_ModBus();

    EEPROM.write(0, New_Flave_id);
    Serial.println("Change slave ID SUCCESS...");
    Serial.print("New slave ID is : ");
    Serial.println(New_Flave_id, HEX);

  }else{
    Serial.println("Slave ID nubmer ERROR !!! <Modify_SlaveID>");
  }
}

/*
 *brief   : Modify baud rate of ModBus according to ModBus instruction
 *para    : None
 *return  : None
*/
void Change_BaudRate(void)
{
  unsigned char BaudRate_Frame_Buffer[8] = {0};
  
  if (g_Receive_Frame[5] <= 7){
                   
    for (unsigned char i = 0; i < 6; i++)
      BaudRate_Frame_Buffer[i] = g_Receive_Frame[i];

    unsigned int CRC16 = Calculate_CRC(BaudRate_Frame_Buffer, 6);

    BaudRate_Frame_Buffer[6] = CRC16 >> 8;
    BaudRate_Frame_Buffer[7] = CRC16 & 0x00FF;
    
    Enable_ModBus();
    ModBus_Serial.write(&BaudRate_Frame_Buffer[0], 8);
    Disable_ModBus();

    EEPROM.write(1, g_Receive_Frame[5]);
    Set_RS485_BaudRate_List();

  }else{
    Serial.println("Received baud rate ERROR !!! <Modify_BaudRate>");
  }
}

void Set_RS485_BaudRate_List(void)
{
  unsigned char BaudRate_Mark = EEPROM.read(1);
  unsigned long RS485_BaudRate_Value;

  switch (BaudRate_Mark){
    case  0: RS485_BaudRate_Value = 2400;   break;   
    case  1: RS485_BaudRate_Value = 4800;   break;
    case  2: RS485_BaudRate_Value = 9600;   break;
    case  3: RS485_BaudRate_Value = 14400;  break;
    case  4: RS485_BaudRate_Value = 19200;  break;
    case  5: RS485_BaudRate_Value = 38400;  break;
    case  6: RS485_BaudRate_Value = 56000;  break;
    case  7: RS485_BaudRate_Value = 57600;  break;
    default: RS485_BaudRate_Value = 9600;   break;
  }

  Serial.print("Now the RS485 baud rate is: ");
  Serial.println(RS485_BaudRate_Value);

  ModBus_Serial.begin(RS485_BaudRate_Value);
  ModBus_Serial.listen(); 
}

/*
 *brief   : Get light intensity value and uv value.
 *para    : None
 *return  : None
 */
void Get_illumination_and_UV(void)
{
  byte msb = 0, lsb = 0;
  uint16_t uv = 0xFFFF;
  static unsigned long mLux_value = 0xFFFFFFFF;
  unsigned long Tsl_lux = 0xFFFFFFFF;
  unsigned char Collect_Num = 0;
  bool Collect_Flag = false;

  do{
      CJMCU6750.requestFrom(I2C_ADDR+1, 1); //MSB
      delay(5);
      if(CJMCU6750.available())
      msb = CJMCU6750.read();

      CJMCU6750.requestFrom(I2C_ADDR+0, 1); //LSB
      delay(5);
      if(CJMCU6750.available())
        lsb = CJMCU6750.read();

      uv = (msb << 8) | lsb;

      max44009.getMeasurement(mLux_value);
      Tsl_lux = mLux_value / 1000L;

      if (uv >= 65535 || uv == 0)
        Collect_Flag = true;

      if (Tsl_lux == 0)
        Collect_Flag = true;
      else if (Tsl_lux >= 3941072){
        Tsl_lux = 0xFFFFFFFF;
        Collect_Flag = true;
      }

      if (Collect_Flag == false){
        uv *= 0.005625;
        break;
      }
      
      Collect_Num++;

  }while (Collect_Num < 3);

  Serial.print("UV : ");
  Serial.println(uv); //output in steps (16bit)

  Serial.print("Lux: ");
  Serial.print(Tsl_lux);
  Serial.println(" lux");

  g_Sensor_Data_Buffer[26] = uv >> 8;
  g_Sensor_Data_Buffer[27] = uv & 0xFF;

  unsigned char Tsl_lux_high1 =  Tsl_lux >> 24;
  unsigned char Tsl_lux_low1  = (Tsl_lux & 0x00FF0000) >> 16;
  unsigned char Tsl_lux_high2 = (Tsl_lux & 0x0000FF00) >> 8;
  unsigned char Tsl_lux_low2  = Tsl_lux & 0x000000FF; 

   g_Sensor_Data_Buffer[14] = Tsl_lux_high1;
   g_Sensor_Data_Buffer[15] = Tsl_lux_low1;
   g_Sensor_Data_Buffer[16] = Tsl_lux_high2;
   g_Sensor_Data_Buffer[17] = Tsl_lux_low2;
}

/*
 *brief   : Get temperature and humidity values
 *para    : None
 *return  : None
 */
void Get_Temperature_and_humidity(void)
{
  float temp, humi;                    
  int temp_value = 0; 
  int humi_value = 0;
  unsigned char Collect_Num = 0;
  
  do{
      temp = myHumidity.readTemperature();
      delay(5);
      humi = myHumidity.readHumidity();
      if ((int)temp == 0 || (int)humi == 0 || temp >= 998 || humi >= 998)
        ;
      else break;

      Collect_Num++;

  }while (Collect_Num < 3);

  Serial.print("temp:");
  Serial.println(temp);

  Serial.print("humi:");
  Serial.println(humi);

  temp_value = temp * 10;
  humi_value = humi * 10;
  
  g_Sensor_Data_Buffer[2]  = temp_value >> 8;
  g_Sensor_Data_Buffer[3]  = temp_value & 0xFF;
  g_Sensor_Data_Buffer[0]  = humi_value >> 8;
  g_Sensor_Data_Buffer[1]  = humi_value & 0xFF;
}

/*
 *brief   : Calculate temperature,air pressure and altitude
 *para    : None
 *return  : None
 */
void BMP_calculate(void)
{
  g_temperature = bmp180GetTemperature(bmp180ReadUT());
  g_temperature = g_temperature * 0.1;
  g_pressure    = bmp180GetPressure(bmp180ReadUP());
  g_pressure2   = g_pressure / 101325;
  g_pressure2   = pow(g_pressure2, 0.19029496);
  g_altitude    = 44330 * (1 - g_pressure2); //altitude = 44330*(1-(pressure/101325)^0.19029496);
}

/*
 *brief   : Get atmospheric pressure value
 *para    : None
 *return  : None
 */
void Get_Pressure(void)
{
  unsigned long int Pressure_Value = 0;
  unsigned char Collect_Num = 0;

  do{
    BMP_calculate();  
    Pressure_Value = (unsigned long int)g_pressure;

    if (Pressure_Value == 0)
      Pressure_Value = 0xFFFFFFFF;
    else 
      break;

    Collect_Num++;

  }while (Collect_Num < 3);

  unsigned char Pressure_High1  = Pressure_Value >> 24;
  unsigned char Pressure_Low1   = (Pressure_Value & 0x00FF0000) >> 16;
  unsigned char Pressure_High2  = (Pressure_Value & 0x0000FF00) >> 8;
  unsigned char Pressure_Low2   = Pressure_Value & 0x000000FF;

  g_Sensor_Data_Buffer[20] = Pressure_High1;
  g_Sensor_Data_Buffer[21] = Pressure_Low1;
  g_Sensor_Data_Buffer[22] = Pressure_High2;
  g_Sensor_Data_Buffer[23] = Pressure_Low2;

  Serial.print("Pressure : ");
  Serial.print(Pressure_Value * 10);
  Serial.println("Pa");
}

/*
 *brief   : Get altitude value(There is no requirement for altitude at present, so reservation here)
 *para    : None
 *return  : None
 */
void Get_Altitude(void)
{
  BMP_calculate();
}

/*
 *brief   : Get the values of CO2 and TVOC
 *para    : None
 *return  : None
 */
void Get_CO2_and_TVOC(void)
{
  unsigned int CO2_Value = 0;
  unsigned int TVOC_Value = 0;
  unsigned char Collect_Num = 0;

  do{
    if (CCS811_Sensor.dataAvailable()){
      CCS811_Sensor.readAlgorithmResults();
      CO2_Value = CCS811_Sensor.getCO2(); 
      delay(5);
      TVOC_Value = CCS811_Sensor.getTVOC();
    }

    if (CO2_Value == 0 || CO2_Value == 65535 || TVOC_Value == 0 || TVOC_Value == 65535)
      ;
      else break;

    Collect_Num++;

  }while (Collect_Num < 3);

  g_Sensor_Data_Buffer[10] = CO2_Value >> 8;  
  g_Sensor_Data_Buffer[11] = CO2_Value & 0xFF;
  g_Sensor_Data_Buffer[12] = TVOC_Value >> 8;
  g_Sensor_Data_Buffer[13] = TVOC_Value & 0xFF;

  Serial.print("CO2 : ");
  Serial.print(CO2_Value);
  Serial.println("ppm");

  Serial.print("TVOC : ");
  Serial.print(TVOC_Value);
  Serial.println("ppm");
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 *CRC校验函数
 *参数1:待校验数组的起�?地址
 *参数2:待校验数组的长度
 *返回值CRC校验结果,16�??,低字节在�??
 */
unsigned int Calculate_CRC(unsigned char* _regs, unsigned char arraySize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < arraySize; i++){
    temp = temp ^ *(_regs + i);
    for (unsigned char j = 1; j <= 8; j++){
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF; 
  return temp; 
}
