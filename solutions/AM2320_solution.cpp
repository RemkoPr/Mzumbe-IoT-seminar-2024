/* Most of this code was adapted from the following GitHub repository:
*  https://github.com/RobTillaart/AM232X
*  If you'd like to try and understand the read/write functions, compare with the
*  AM2320 sensor datasheet:
*  https://cdn-shop.adafruit.com/product-files/3721/AM2320.pdf#page=5.30
*/
#include <Wire.h>
#include <Dwenguino.h>

// device I2C address
#define AM232X_ADDRESS                  0x5C
// communication error codes
#define AM232X_OK                        0
#define AM232X_ERROR_UNKNOWN            -10
#define AM232X_ERROR_CONNECT            -11
#define AM232X_ERROR_FUNCTION           -12
#define AM232X_ERROR_ADDRESS            -13
#define AM232X_ERROR_REGISTER           -14
#define AM232X_ERROR_CRC_1              -15
#define AM232X_ERROR_CRC_2              -16
#define AM232X_ERROR_WRITE_DISABLED     -17
#define AM232X_ERROR_WRITE_COUNT        -18
#define AM232X_MISSING_BYTES            -19
#define AM232X_READ_TOO_FAST            -20
#define AM232X_INVALID_VALUE            -999

// global variables
uint8_t  _bits[8];    //  buffer to hold raw data  
float _humidity      = 0.0;
float _temperature   = 0.0;
float _humOffset     = 0.0;
float _tempOffset    = 0.0;
int _lastRead      = 0;
int _readDelay     = 2000;
bool _suppressError = false;

// function declarations
bool isConnected(uint16_t timeout=3000);
uint32_t getDeviceID();
int _readRegister(uint8_t reg, uint8_t count);
int _getData(uint8_t length);
uint16_t _crc16(uint8_t *ptr, uint8_t length);
int read();
float readTemperature();

void setup() {
  initDwenguino(); // Initialize all Dwenguino functionality.
  Serial.begin(9600);
  while(!Serial);
  Wire.setClock(100000);  // Initialize I2C bus
}

void loop() {
  Serial.println("--------------");
  Serial.println("[DEBUG] Communication code (0=SUCCESS): "+ String(read()));
  Serial.println("Temp: " + String(_temperature));
  Serial.println("Hum: " + String(_humidity));
  delay(1000);
}


// function definitions
bool isConnected(uint16_t timeout) {
  uint32_t start = micros();
  while (micros() - start < timeout)
  {
    Wire.beginTransmission(AM232X_ADDRESS);
    int endTransmissionReturnCode = Wire.endTransmission();
    if ( endTransmissionReturnCode == 0) {
      //Serial.println("Connected!");
      return true;
    } /*else {
      Serial.println("Not connected. Return code: " + String(endTransmissionReturnCode));
    }*/
    yield();
    delayMicroseconds(100);
  }
  return false;
}

int _readRegister(uint8_t reg, uint8_t count) {
  //  HANDLE PENDING IRQ
  yield();

  //  WAKE UP the sensor
  if (! isConnected() ) return AM232X_ERROR_CONNECT;

  //  request the data
  Wire.beginTransmission(AM232X_ADDRESS);
  Wire.write(0x03);
  Wire.write(reg);
  Wire.write(count);
  int rv = Wire.endTransmission();
  if (rv < 0) return rv;

  //  request 4 extra, 2 for command + 2 for CRC
  rv = _getData(count + 4);
  return rv;
}

int _getData(uint8_t length) {
  int bytes = Wire.requestFrom(AM232X_ADDRESS, length);
  if (bytes == 0) return AM232X_ERROR_CONNECT;

  for (int i = 0; i < bytes; i++)
  {
    _bits[i] = Wire.read();
  }

  // ANALYZE ERRORS
  //   will not detect if we requested 1 byte as that will
  //   return 5 bytes as requested. E.g. getStatus()
  //   design a fix if it becomes a problem.
  if (bytes != length)
  {
    switch (_bits[3])
    {
      case 0x80: return AM232X_ERROR_FUNCTION;
      case 0x81: return AM232X_ERROR_ADDRESS;
      case 0x82: return AM232X_ERROR_REGISTER;
      case 0x83: return AM232X_ERROR_CRC_1;  // previous write had a wrong CRC
      case 0x84: return AM232X_ERROR_WRITE_DISABLED;
      default:   return AM232X_ERROR_UNKNOWN;
    }
  }

  // CRC is LOW Byte first
  uint16_t crc = _bits[bytes - 1] * 256 + _bits[bytes - 2];
  if (_crc16(&_bits[0], bytes - 2) != crc)
  {
    return AM232X_ERROR_CRC_2;  // read itself has wrong CRC
  }

  return AM232X_OK;
}

uint16_t _crc16(uint8_t *ptr, uint8_t length) {
  uint16_t crc = 0xFFFF;
  while(length--)
  {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++)
    {
      if (crc & 0x01)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

int read() {
  if (millis() - _lastRead < _readDelay)
  {
    return AM232X_READ_TOO_FAST;
  }
  _lastRead = millis();

  //  READ HUMIDITY AND TEMPERATURE REGISTERS
  int rv = _readRegister(0x00, 4);
  if (rv < 0) return rv;

  if (rv != AM232X_OK)
  {
    if (_suppressError == false)
    {
      _humidity    = AM232X_INVALID_VALUE;
      _temperature = AM232X_INVALID_VALUE;
    }
    return rv;  //  propagate error value
  }

  //  EXTRACT HUMIDITY AND TEMPERATURE
  _humidity = (_bits[2] * 256 + _bits[3]) * 0.1;
  int16_t t = ((_bits[4] & 0x7F) * 256 + _bits[5]);
  if (t == 0)
  {
    _temperature = 0.0;     //  prevent -0.0;
  }
  else
  {
    _temperature = t * 0.1;
    if ((_bits[4] & 0x80) == 0x80 )
    {
      _temperature = -_temperature;
    }
  }

#ifdef AM232X_VALUE_OUT_OF_RANGE
  //  TEST OUT OF RANGE
  if (_humidity > 100)
  {
    return AM232X_HUMIDITY_OUT_OF_RANGE;
  }
  if ((_temperature < -40) || (_temperature > 80))
  {
    return AM232X_TEMPERATURE_OUT_OF_RANGE;
  }
#endif

  return AM232X_OK;
}

float readTemperature() {
  //  READ TEMPERATURE REGISTERS
  int rv = _readRegister(0x02, 2);
  if (rv < 0) return rv;

  if (rv != AM232X_OK)
  {
    if (_suppressError == false) {
      _temperature = AM232X_INVALID_VALUE;
    }
    return rv;  //  propagate error value
  }

  //  EXTRACT TEMPERATURE
  int16_t t = ((_bits[2] & 0x7F) * 256 + _bits[3]);
  float __temperature;
  if (t == 0) {
    __temperature = 0.0;     //  prevent -0.0;
  } else {
    __temperature = t * 0.1;
    if ((_bits[2] & 0x80) == 0x80 ) {
      __temperature = -__temperature;
    }
  }
  return __temperature;
}

uint32_t getDeviceID() {
  int rv = _readRegister(0x0B, 4);
  if (rv < 0) return rv;

  uint32_t _deviceID = (_bits[2] * 256) + _bits[3];
  _deviceID = _deviceID * 256 + _bits[4];
  _deviceID = _deviceID * 256 + _bits[5];
  return _deviceID;
}
