/***************************************************************************
* This sketch is written and provided by EnviTronics Lab free of charge. 
* Please feel free to use in your project and redistribute. 
* 
* This sketch shows how you can read up to four EnviTronics Lab soil & rockwool HITA E0 EC/TDS sensor:
* https://www.envitronicslab.com/hita-e0
*
* You will also be able to read arduino serial number and sensor 64-bit ID.
*  
* Further information can be found on:
* https://www.envitronicslab.com/
* 
***************************************************************************/
#include <DallasTemperature.h>   // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <ArduinoUniqueID.h>     // https://github.com/ricaun/ArduinoUniqueID
#include <ADS1115_WE.h>          // https://github.com/wollewald/ADS1115_WE
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog
#include <Wire.h>

// Device information
static constexpr int FIRMWARE_VERSION{ 1001 };  // V1.0.0.1 = 1001

// Temperature Sensors
static constexpr int ONE_WIRE_BUS[4]{ 2, 3, 4, 5 };  // Assign Arduino pin to temp sensor
OneWire oneWire[4]{                                  // Setup a oneWire instance to communicate with any OneWire devices
                    // initializers for each instance in the array go here
                    OneWire(ONE_WIRE_BUS[0]),
                    OneWire(ONE_WIRE_BUS[1]),
                    OneWire(ONE_WIRE_BUS[2]),
                    OneWire(ONE_WIRE_BUS[3])
};
DallasTemperature tempSensors[4]{ // Pass our oneWire reference to Dallas Temperature sensor
                                  // initializers for each instance in the array go here
                                  DallasTemperature(&oneWire[0]),
                                  DallasTemperature(&oneWire[1]),
                                  DallasTemperature(&oneWire[2]),
                                  DallasTemperature(&oneWire[3])
};
DeviceAddress deviceAddress[4];  // Arrays to hold device address
bool getAddResult[4]{ false, false, false, false };
float fltSensorCode[4]{ 0.0, 0.0, 0.0, 0.0 };
float fltSensorOffset[4]{ 0.0, 0.0, 0.0, 0.0 };

// ADS1115 settings; 16-bit version
ADS1115_WE ads1115a(0x48);  // ADC board 1: ADDR pin connected to GND
ADS1115_WE ads1115b(0x49);  // ADC board 2: ADDR pin connected to VCC

// General constants and variables to store values
bool blnTempMeas[]{ false, false, false, false };
unsigned int avgReading[]{ 0, 0, 0, 0 };
unsigned int Reading[]{ 0, 0, 0, 0 };
unsigned int RAW[4]{ 0, 0, 0, 0 };
float TpC[4]{ 999, 999, 999, 999 };
bool blnNewSensor[]{ true, true, true, true };

// HITA E0 EC Sensor
float EC25[4]{ 0.0, 0.0, 0.0, 0.0 };
unsigned int intTDS[4]{ 0.0, 0.0, 0.0, 0.0 };

static constexpr int SENSOR_ENABLE_PIN[4]{ 6, 7, 8, 9 };  // Sensor Voltage pin



void setup(void) {
  Serial.begin(115200);  // Start serial port

  getSerialNumber();  // Get Arduino serial number

  // ADS1115 A to D
  Wire.begin();
  /// First ADC
  ads1115a.init();
  ads1115a.setVoltageRange_mV(ADS1115_RANGE_2048);  // 2x gain   +/- 2.048V  1 bit = 0.0625 mV (16 bit) / 1.0mV (12 bit)
  ads1115a.setConvRate(ADS1115_860_SPS);            // Other options: ADS1115_8_SPS, ADS1115_16_SPS, ADS1115_32_SPS, ADS1115_64_SPS, ADS1115_128_SPS, ADS1115_250_SPS, ADS1115_475_SPS, ADS1115_860_SPS)
  /// Second ADC
  ads1115b.init();
  ads1115b.setVoltageRange_mV(ADS1115_RANGE_2048);  // 2x gain   +/- 2.048V  1 bit = 0.0625 mV (16 bit) / 1.0mV (12 bit)
  ads1115b.setConvRate(ADS1115_860_SPS);            // Other options: ADS1115_8_SPS, ADS1115_16_SPS, ADS1115_32_SPS, ADS1115_64_SPS, ADS1115_128_SPS, ADS1115_250_SPS, ADS1115_475_SPS, ADS1115_860_SPS

  // Initialize the Sensor Enable pins as output:
  pinMode(SENSOR_ENABLE_PIN[0], OUTPUT);
  pinMode(SENSOR_ENABLE_PIN[1], OUTPUT);
  pinMode(SENSOR_ENABLE_PIN[2], OUTPUT);
  pinMode(SENSOR_ENABLE_PIN[3], OUTPUT);
  // Make sure sensor is off
  SensorOnFun(false, SENSOR_ENABLE_PIN[0]);
  SensorOnFun(false, SENSOR_ENABLE_PIN[1]);
  SensorOnFun(false, SENSOR_ENABLE_PIN[2]);
  SensorOnFun(false, SENSOR_ENABLE_PIN[3]);

  // Configure Watchdog Timer
  Watchdog.enable(30000);
}

void loop(void) {
  // Sensor 1
  doSensorReadings(0);  // Read thesensor
  sendAllDataToPC(0);   // Send all measurements to PC
  /*
  // Sensor 2
  doSensorReadings(1); // Read thesensor
  sendAllDataToPC(1); // Send all measurements to PC
  // Sensor 3
  doSensorReadings(2); // Read thesensor
  sendAllDataToPC(2); // Send all measurements to PC
  // Sensor 4
  doSensorReadings(3); // Read thesensor
  sendAllDataToPC(3); // Send all measurements to PC
  */

  // A little bit of delay between measurements
  delay(5000);

  Watchdog.reset();  // Reset watchdog.
}

void SensorOnFun(bool blnOn, const int Pin) {
  if (blnOn) {
    digitalWrite(Pin, HIGH);
  } else {
    digitalWrite(Pin, LOW);
  }
}

void doSensorReadings(uint8_t bytSensor) {
  // Turn sensor on
  SensorOnFun(true, SENSOR_ENABLE_PIN[bytSensor]);
  // Measure sensor analoge output voltage
  RAW[bytSensor] = readSensor(bytSensor);
  // Obtain sensor ID and sensor code
  if (blnNewSensor[bytSensor]) {
    blnNewSensor[bytSensor] = false;
    getSensorID(bytSensor);  // Get sensor ID
  }
  // Turn sensor off
  SensorOnFun(false, SENSOR_ENABLE_PIN[bytSensor]);

  if (!getAddResult[bytSensor]) {
    blnNewSensor[bytSensor] = true;
    return;  // Sensor was not connected
  }

  // Calculate EC
  EC25[bytSensor] = calculateEC(RAW[bytSensor], TpC[bytSensor], bytSensor);

  // Calibrate EC
  EC25[bytSensor] = CalibrateEC(EC25[bytSensor]);

  // Temperature-compenscate EC
  EC25[bytSensor] = ECTempCompensate(TpC[bytSensor], EC25[bytSensor]);

  // Calculate TDS
  ConvertECtoTDSCurve(bytSensor);
}


//############################################################################################
// Common sensor routines
//############################################################################################
unsigned int readSensor(uint8_t bytSensor) {
  /// If it's HITA E0 sensor.
  delay(5);                             // Start with a delay to make sure sensor is ready
  if (!IsSensorConnected(bytSensor)) {  // Takes 14 m
    return 0;                           // If sensor is not connected just return.
  }
  measSensorTemp(bytSensor);
  return { avgADC(bytSensor) };  // Takes -- ms
}

unsigned int avgADC(uint8_t bytSensor) {
  float avg{ measureADC(bytSensor) };  // Takes 3 ms
  avg /= 2048.0;
  return { avg * 32768 };
}

float measureADC(uint8_t bytSensor) {
  float rawADCReading{ 0.0 };
  switch (bytSensor) {
    case 0:
      rawADCReading = ads1115a_readChannel(ADS1115_COMP_0_1);  // Takes 10 ms
      break;
    case 1:
      rawADCReading = ads1115a_readChannel(ADS1115_COMP_2_3);  // Takes 10 ms
      break;
    case 2:
      rawADCReading = ads1115b_readChannel(ADS1115_COMP_0_1);  // Takes 10 ms
      break;
    case 3:
      rawADCReading = ads1115b_readChannel(ADS1115_COMP_2_3);  // Takes 10 ms
      break;
    default:
      // if nothing else matches, do the default
      break;
  }
  return rawADCReading;
}

float ads1115a_readChannel(ADS1115_MUX channel) {
  ads1115a.setCompareChannels(channel);
  ads1115a.startSingleMeasurement();
  while (ads1115a.isBusy()) {}
  float volt{ ads1115a.getResult_mV() };  // alternative: getResult_mV for Millivolt
  return volt;
}

float ads1115b_readChannel(ADS1115_MUX channel) {
  ads1115b.setCompareChannels(channel);
  ads1115b.startSingleMeasurement();
  while (ads1115b.isBusy()) {}
  float volt{ ads1115b.getResult_mV() };  // alternative: getResult_mV for Millivolt
  return volt;
}

void measSensorTemp(uint8_t bytSensor) {
  // Number of milliseconds to wait till conversion is complete based on resolution: 94, 188, 375, 750 ms for 9, 10, 11, 12, respectively.
  // Thisequates  to  a  temperature  resolution  of  0.5°C,  0.25°C,  0.125°C,  or  0.0625°C.
  const uint8_t resolution{ 11 };  // Set the resolution to 11 bit
  tempSensors[bytSensor].setResolution(deviceAddress[bytSensor], resolution);
  tempSensors[bytSensor].setWaitForConversion(false);  // Request temperature conversion - non-blocking / async
  tempSensors[bytSensor].requestTemperatures();        // Takes 14 ms
  delay(375);
  // Get Sensor temp
  TpC[bytSensor] = tempSensors[bytSensor].getTempC(deviceAddress[bytSensor]);
}

bool IsSensorConnected(uint8_t bytSensor) {
  getAddResult[bytSensor] = tempSensors[bytSensor].getAddress(deviceAddress[bytSensor], 0);  // Takes 14 ms // Check see if a sensor is connected
  return getAddResult[bytSensor];
}

//############################################################################################
// HITA E0 Electrical Conductivity Sensor Measurements and Compensation
//############################################################################################
float calculateEC(unsigned int EC_R, float Tp, uint8_t bytSensor) {
  float fltEC{ 0.0 };
  const float Resolution{ 32768.0 };  // (65535/2); 16-bit resolution
  const float Vin{ 2492.0 };          // mV; Amplitude of the squarewave generated by the oscillator multiplied by 2
  const float Vref{ 2048.0 };         // mV; ADC Vref = 2.048V (Gain Two)
  float Vout{ 0 };                    // Vout default value

  Vout = EC_R * Vref;
  Vout = Vout / Resolution;
  if (Vout < 0) Vout = 0;

  const float Rref{ 2200.0 };  // Theoretical value
  const float Rg{ 43.0 };      // To account for the -- Ohm resitor connected to the Ground.
  float R{ 0.0 };
  float Rn{ 0.0 };
  float Rd{ 0.0 };
  //R = Rref * (1 / (((Vin / Vout) - 1))) - Rg;  // Formula to calculate tested resistor's value //R = (Rn / Rd) - Rg;
  Rn = Rref * Vout;
  Rd = Vin - Vout;
  R = (Rn / Rd) - Rg;

  if (R <= 0.0) {
    fltEC = 0.0;
    return fltEC;
  }

  // Calibrate R
  R = +1.4535 * R + -2908;  // R2 = 0.9999

  const float C{ 1.0 };             // Theoretical value: C = 1/5.3 cm = 0.188679;    // EC = C/R and C = L/A
  fltEC = (C * 1000.0) / float(R);  // Calculate EC; Units: 1 mho/m = 1 mmho/cm = 1 mS/cm = 1 dS/m = 1000 µS/cm

  return fltEC;
}

float CalibrateEC(float fltEC) {
  // Internal Calibration
  float slope{ 0.8989 };  //
  float intercept{ -0.13 };
  float fltCalibEC{ slope * fltEC + intercept };

  if (fltCalibEC <= intercept + 0.01) fltCalibEC = 0.0;
  if (fltCalibEC >= 10.00) fltCalibEC = 10.0;

  return fltCalibEC;
}

float ECTempCompensate(float Tp, float fltEC) {
  // Simplified method
  //const float ft {0.0185}; // considered the standard for plant nutrient solutions
  //float ECTemp{ fltEC/( 1 + ft * (Tp - 25.0))};

  // Advanced method: US  Salinity  Laboratory  Staff,  1954 (http://www.fao.org/3/x2002e/x2002e.pdf)
  float dT{ (Tp - 25.0) / 10.0 };
  float ft{ 1 - 0.20346 * (dT) + 0.03822 * pow(dT, 2) - 0.00555 * pow(dT, 3) };
  float ECTemp{ ft * fltEC };

  if (ECTemp <= 0) ECTemp = 0.0;
  if (ECTemp >= 10.00) ECTemp = 10.0;

  return ECTemp;
}

void ConvertECtoTDSCurve(uint8_t bytSensor) {
  if (EC25[bytSensor] > 0.1) {
    //intTDS[bytSensor]  = 8.2876 * EC25[bytSensor]^2 + 779.46 * EC25[bytSensor] - 74.853
    intTDS[bytSensor] = 8.2876 * pow(EC25[bytSensor], 2);
    intTDS[bytSensor] += 779.46 * EC25[bytSensor];
    intTDS[bytSensor] += 74.853;

  } else {
    intTDS[bytSensor] = 0;
  }
}

//############################################################################################
// -- Serial Port Communication --
//############################################################################################
void sendAllDataToPC(uint8_t bytSensor) {
  // Send all measurements to PC
  // Sensors 0, 1, 2, 3
  if (getAddResult[bytSensor]) {
    /// If it's HITA E0 EC sensor
    serialPrintFunc(1000 + bytSensor, EC25[bytSensor], TpC[bytSensor], intTDS[bytSensor]);  // SI units

  } else {                                      // Sensor is disconnected, so reset all related variables.
    serialPrintFunc(100 + bytSensor, 0, 0, 0);  // Let computer know Sensor is disconnected
  }
}

void serialPrintFunc(int intCommandCode, float fltB, float fltC, float fltD) {
  Serial.print(F(">"));
  Serial.print(intCommandCode);
  serialPrintFuncSum2(',', fltB);
  serialPrintFuncSum2(',', fltC);
  serialPrintFuncSum2(',', fltD);
  Serial.println(F(","));

  Serial.flush();  // Wait untill all data are sent to computer before putting Arduino into sleep
}
void serialPrintFuncSum2(char Letter, float fltValue) {
  Serial.print(Letter);  // Send coefficients to GUI!
  Serial.print(fltValue);

  Serial.flush();  // Wait untill all data are sent to computer
}

//############################################################################################
// -- Calibration Coefficients --
//############################################################################################
void getCalibrationCoefficients(uint8_t bytSensor) {
  if (!getAddResult[bytSensor]) return;

  int intSensorType{ tempSensors[bytSensor].getHighAlarmTemp(deviceAddress[bytSensor]) };
  int intcHighLow{ tempSensors[bytSensor].getLowAlarmTemp(deviceAddress[bytSensor]) };

  // Put new values into variables
  calculateHighLowCalCoeffs(intSensorType, intcHighLow, bytSensor);
}

void calculateHighLowCalCoeffs(int intSensorType, int intcHighLow, uint8_t bytSensor) {
  fltSensorCode[bytSensor] = intSensorType;
  fltSensorOffset[bytSensor] = intcHighLow;
}

//############################################################################################
// -- Get Device SN and 64-bit Sensor ID --
//############################################################################################
void getSerialNumber() {
  String strSN = "";
  for (size_t i = 0; i < UniqueIDsize; i++) {
    if (UniqueID[i] < 100)
      strSN += '0';
    if (UniqueID[i] < 10)
      strSN += '0';
    strSN += UniqueID[i];
    if (i < 15) strSN += " ";
  }

  Serial.print(F("Device ID: "));
  Serial.println(strSN);
  Serial.flush();  // Wait untill all data are sent to computer
}

void getSensorID(uint8_t bytSensor) {
  if (!getAddResult[bytSensor]) return;

  uint8_t i{ 0 };
  uint8_t ID[8];
  String strID{ "" };

  // Turn Sensor on
  SensorOnFun(true, SENSOR_ENABLE_PIN[bytSensor]);
  delay(20);  // Wait for the Sensor to wake up

  // Start up the library / locate devices on the bus
  tempSensors[bytSensor].begin();

  // Initiate a search for the OneWire object created and read its value into addr array we declared above
  while (oneWire[bytSensor].search(ID)) {
    // Read each uint8_t in the address array
    for (i = 0; i < 8; i++) {
      // Put each uint8_t in the ID array
      strID += ID[i];
      if (ID[i] < 10) {
        strID += "00";
      }
      if (ID[i] > 10 && ID[i] < 100) {
        strID += '0';
      }
      if (i == 1) {
        strID += ' ';
      }
      if (i == 3) {
        strID += ' ';
      }
      if (i == 5) {
        strID += ' ';
      }
    }
    // A check to make sure that what we read is correct.
    if (OneWire::crc8(ID, 7) != ID[7]) {
      // CRC is not valid!
      return;
    }
  }
  oneWire[bytSensor].reset_search();

  // Turn Sensor off
  SensorOnFun(false, SENSOR_ENABLE_PIN[bytSensor]);

  // Send sensor ID to GUI
  Serial.print(F("Sensor "));
  Serial.print(bytSensor);
  Serial.print(F(" ID: "));
  Serial.println(strID);
  Serial.flush();  // Wait untill all data are sent to computer before putting Arduino into sleep

  return;
}

/***********************************************************************************************/
float ConvertCtoF(float Temp) {
  return { Temp * (9 / 5) + 32 };
}
