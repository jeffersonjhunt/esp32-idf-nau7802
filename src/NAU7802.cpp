/*
  This is an ESP32 IDF library written for the NAU7802 24-bit wheatstone
  bridge and load cell amplifier. 
  
  It is a port by Jefferson J Hunt @ OOE, August 23rd, 2024 from the 
  Arduino driver by Nathan Seidle @ SparkFun Electronics, March 3nd, 2019

  The NAU7802 is an I2C device that converts analog signals to a 24-bit
  digital signal. This makes it possible to create your own digital scale
  either by hacking an off-the-shelf bathroom scale or by creating your
  own scale using a load cell.

  The NAU7802 is a better version of the popular HX711 load cell amplifier.
  It uses a true I2C interface so that it can share the bus with other
  I2C devices while still taking very accurate 24-bit load cell measurements
  up to 320Hz.

  Original Nathan Seidle driver
  https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

  ESP32 IDF port by Jefferson J Hunt
  https://github.com/jeffersonjhunt/esp32-idf-nau7802
*/

#include "NAU7802.h"

static const char *TAG = "driver-nau7802"; // tag for logging

//Constructor
NAU7802::NAU7802(  i2c_master_bus_config_t *bus_config, i2c_device_config_t *dev_config )
{
    // initialize device
    ESP_ERROR_CHECK(i2c_new_master_bus(bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_reset(bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, dev_config, &dev_handle));
}

//Sets up the NAU7802 for basic function
//If initialize is true (or not specified), default init and calibration is performed
//If initialize is false, then it's up to the caller to initalize and calibrate
//Returns true upon completion
bool NAU7802::begin(bool initialize)
{
  //Check if the device ack's over I2C
  if (isConnected() == false)
  {
    //There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
    if (isConnected() == false)
      return (false);
  }

  bool result = true; //Accumulate a result as we do the setup

  if (initialize)
  {
    result &= reset(); //Reset all registers

    result &= powerUp(); //Power on analog and digital sections of the scale

    result &= setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3V

    result &= setGain(NAU7802_GAIN_128); //Set gain to 128

    result &= setSampleRate(NAU7802_SPS_80); //Set samples per second to 10

    //Turn off CLK_CHP. From 9.1 power on sequencing.
    uint8_t adc = getRegister(NAU7802_ADC);
    adc |= 0x30;
    result &= setRegister(NAU7802_ADC, adc);

    result &= setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

    result &= clearBit(NAU7802_PGA_LDOMODE, NAU7802_PGA); //Ensure LDOMODE bit is clear - improved accuracy and higher DC gain, with ESR < 1 ohm

    vTaskDelay(_ldoRampDelay / portTICK_PERIOD_MS); //Wait for LDO to stabilize - takes about 200ms

    getWeight(true, 10); //Flush

    result &= calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel
  }

  return (result);
}

//Returns true if device is present
//Tests for device ack to I2C address
bool NAU7802::isConnected()
{
  ESP_ERROR_CHECK(i2c_master_probe(bus_handle, NAU7802_I2CADDR_DEFAULT, 10)); // probe device
  return (true);    //All good  
}

//Returns true if Cycle Ready bit is set (conversion is complete)
bool NAU7802::available()
{
  bool ready = getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL);
  if(ready)
    ESP_LOGI(TAG, "Cycle Ready bit is set");
  else
    ESP_LOGI(TAG, "Cycle Ready bit is not set");
  return ready;
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool NAU7802::calibrateAFE(NAU7802_Cal_Mode mode)
{
  beginCalibrateAFE(mode);
  return waitForCalibrateAFE(1000);
}

//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
void NAU7802::beginCalibrateAFE(NAU7802_Cal_Mode mode)
{
  uint8_t value = getRegister(NAU7802_CTRL2);
  value &= 0xFC; // Clear CALMOD bits
  uint8_t calMode = (uint8_t)mode;
  calMode &= 0x03; // Limit mode to 2 bits
  value |= calMode; // Set the mode
  setRegister(NAU7802_CTRL2, value);

  setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

//Check calibration status.
NAU7802_Cal_Status NAU7802::calAFEStatus()
{
  if (getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2))
  {
    return NAU7802_CAL_IN_PROGRESS;
  }

  if (getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2))
  {
    return NAU7802_CAL_FAILURE;
  }

  // Calibration passed
  return NAU7802_CAL_SUCCESS;
}

//Wait for asynchronous AFE calibration to complete with optional timeout.
//If timeout is not specified (or set to 0), then wait indefinitely.
//Returns true if calibration completes succsfully, otherwise returns false.
bool NAU7802::waitForCalibrateAFE(int64_t timeout_ms)
{
  // TODO: Redo with ESP-IDF framework
  int64_t startTime = CURRENT_TIME_MS;
  NAU7802_Cal_Status cal_ready;

  while ((cal_ready = calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
  {
    if ((timeout_ms > 0) && ((CURRENT_TIME_MS - startTime) > timeout_ms))
    {
      break;
    }
    DELAY_1MS;
  }

  if (cal_ready == NAU7802_CAL_SUCCESS)
  {
    return (true);
  }
  return (false);
}

//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
bool NAU7802::setSampleRate(uint8_t rate)
{
  if (rate > 0b111)
    rate = 0b111; //Error check

  uint8_t value = getRegister(NAU7802_CTRL2);
  value &= 0b10001111; //Clear CRS bits
  value |= rate << 4;  //Mask in new CRS bits

  return (setRegister(NAU7802_CTRL2, value));
}

//Select between 1 and 2
bool NAU7802::setChannel(uint8_t channelNumber)
{
  if (channelNumber == NAU7802_CHANNEL_1)
    return (clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 1 (default)
  else
    return (setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 2
}

//Power up digital and analog sections of scale
bool NAU7802::powerUp()
{
  setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

  //Wait for Power Up bit to be set - takes approximately 200us
  uint8_t counter = 0;
  while (1)
  {
    if (getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
      break; //Good to go
    DELAY_1MS;
    if (counter++ > 100)
      return (false); //Error
  }
  return (setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL)); // Set Cycle Start bit. See 9.1 point 5
}

//Puts scale into low-power mode
bool NAU7802::powerDown()
{
  clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  return (clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}

//Resets all registers to Power Of Defaults
bool NAU7802::reset()
{
  setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); //Set RR
  DELAY_1MS;
  return (clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)); //Clear RR to leave reset state
}

//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
bool NAU7802::setLDO(uint8_t ldoValue)
{
  if (ldoValue > 0b111)
    ldoValue = 0b111; //Error check

  //Set the value of the LDO
  uint8_t value = getRegister(NAU7802_CTRL1);
  value &= 0b11000111;    //Clear LDO bits
  value |= ldoValue << 3; //Mask in new LDO bits
  setRegister(NAU7802_CTRL1, value);

  return (setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Enable the internal LDO
}

void NAU7802::setLDORampDelay(unsigned long delay)
{
  _ldoRampDelay = delay;
}
unsigned long NAU7802::getLDORampDelay()
{
  return _ldoRampDelay;
}

//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
bool NAU7802::setGain(uint8_t gainValue)
{
  if (gainValue > 0b111)
    gainValue = 0b111; //Error check

  uint8_t value = getRegister(NAU7802_CTRL1);
  value &= 0b11111000; //Clear gain bits
  value |= gainValue;  //Mask in new bits

  return (setRegister(NAU7802_CTRL1, value));
}

//Get the revision code of this IC
uint8_t NAU7802::getRevisionCode()
{
  uint8_t revisionCode = getRegister(NAU7802_DEVICE_REV);
  return (revisionCode & 0x0F);
}

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
int32_t NAU7802::getReading()
{
  return get24BitRegister(NAU7802_ADCO_B2);
}

//Return the average of a given number of readings
//Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
int32_t NAU7802::getAverage(uint8_t averageAmount, unsigned long timeout_ms)
{
  int32_t total = 0; // Readings are 24-bit. We're good to average 255 if needed
  uint8_t samplesAquired = 0;

  // TODO: Redo with ESP-IDF framework
  unsigned long startTime = CURRENT_TIME_MS;
  while (1)
  {
    if (available() == true)
    {
      total += getReading();
      if (++samplesAquired == averageAmount)
        break; //All done
    }
    if (CURRENT_TIME_MS - startTime > timeout_ms)
      return (0); //Timeout - Bail with error
  }
  total /= averageAmount;

  return (total);
}

//Call when scale is setup, level, at running temperature, with nothing on it
void NAU7802::calculateZeroOffset(uint8_t averageAmount, unsigned long timeout_ms)
{
  setZeroOffset(getAverage(averageAmount, timeout_ms));
}

//Sets the internal variable. Useful for users who are loading values from NVM.
void NAU7802::setZeroOffset(int32_t newZeroOffset)
{
  _zeroOffset = newZeroOffset;
}

int32_t NAU7802::getZeroOffset()
{
  return (_zeroOffset);
}

//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
void NAU7802::calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount, unsigned long timeout_ms)
{
  int32_t onScale = getAverage(averageAmount, timeout_ms);
  float newCalFactor = ((float)(onScale - _zeroOffset)) / weightOnScale;
  setCalibrationFactor(newCalFactor);
}

//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
//If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
void NAU7802::setCalibrationFactor(float newCalFactor)
{
  _calibrationFactor = newCalFactor;
}

float NAU7802::getCalibrationFactor()
{
  return (_calibrationFactor);
}

//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
float NAU7802::getWeight(bool allowNegativeWeights, uint8_t samplesToTake, unsigned long timeout_ms)
{
  int32_t onScale = getAverage(samplesToTake, timeout_ms);

  //Prevent the current reading from being less than zero offset
  //This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
  //causing the weight to be negative or jump to millions of pounds
  if (allowNegativeWeights == false)
  {
    if (onScale < _zeroOffset)
      onScale = _zeroOffset; //Force reading to zero
  }

  float weight = ((float)(onScale - _zeroOffset)) / _calibrationFactor;
  return (weight);
}

//Set Int pin to be high when data is ready (default)
bool NAU7802::setIntPolarityHigh()
{
  return (clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}

//Set Int pin to be low when data is ready
bool NAU7802::setIntPolarityLow()
{
  return (setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}

//Mask & set a given bit within a register
bool NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = getRegister(registerAddress);
  value |= (1 << bitNumber); //Set this bit
  return (setRegister(registerAddress, value));
}

//Mask & clear a given bit within a register
bool NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = getRegister(registerAddress);
  value &= ~(1 << bitNumber); //Set this bit
  return (setRegister(registerAddress, value));
}

//Return a given bit within a register
bool NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = getRegister(registerAddress);
  value &= (1 << bitNumber); //Clear all but this bit
  return (value);
}

//Get contents of a register
uint8_t NAU7802::getRegister(uint8_t registerAddress)
{
    ESP_LOGI(TAG, "Getting 0x%02X", registerAddress);
    uint8_t reg[1] = { registerAddress };
    uint8_t buffer[1];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, reg, sizeof(reg), buffer, sizeof(buffer), 50));

    return buffer[0];
}

//Send a given value to be written to given address
//Return true if successful
bool NAU7802::setRegister(uint8_t registerAddress, uint8_t value)
{
    ESP_LOGI(TAG, "Setting 0x%02X to 0x%02X", registerAddress, value);
    uint8_t reg[2] = { registerAddress, value };
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, reg, sizeof(reg), 50));
    return true;
}
// Get contents of a 24-bit signed register (conversion result and offsets)
int32_t NAU7802::get24BitRegister(uint8_t registerAddress)
{
  uint8_t _buffer[3];
  union
  {
    uint32_t unsigned32;
    int32_t signed32;
  } signedUnsigned32; // Avoid ambiguity

  ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &registerAddress, sizeof(registerAddress), _buffer, sizeof(_buffer), 50));

  signedUnsigned32.unsigned32 = (uint32_t)_buffer[0] << 16; // MSB
  signedUnsigned32.unsigned32 |= (uint32_t)_buffer[1] << 8; // MidSB
  signedUnsigned32.unsigned32 |= (uint32_t)_buffer[3];      // LSB

  if ((signedUnsigned32.unsigned32 & 0x00800000) == 0x00800000)
    signedUnsigned32.unsigned32 |= 0xFF000000; // Preserve 2's complement

  return (signedUnsigned32.signed32);
}

//Send 24 LSBs of value to given register address. Return true if successful
//Note: assumes bit 23 is already correct for 24-bit signed
bool NAU7802::set24BitRegister(uint8_t registerAddress, int32_t value)
{
  uint8_t _buffer[4];
  union
  {
    uint32_t unsigned32;
    int32_t signed32;
  } signedUnsigned32; // Avoid ambiguity

  signedUnsigned32.signed32 = value;

  _buffer[0] = registerAddress;
  _buffer[1] = (uint8_t)((signedUnsigned32.unsigned32 >> 16) & 0xFF); 
  _buffer[2] = (uint8_t)((signedUnsigned32.unsigned32 >> 8) & 0xFF);


  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, _buffer, sizeof(_buffer),  50));

  return (true);
}

// Get contents of a 32-bit register (gains)
uint32_t NAU7802::get32BitRegister(uint8_t registerAddress)
{
  uint8_t _buffer[4];
  uint32_t unsigned32;

  ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &registerAddress, sizeof(registerAddress), _buffer, sizeof(_buffer), 50));

  unsigned32 = (uint32_t)_buffer[0] << 24; // MSB
  unsigned32 |= (uint32_t)_buffer[1] << 16;
  unsigned32 |= (uint32_t)_buffer[2] << 8;
  unsigned32 |= (uint32_t)_buffer[3]; // LSB

  return (unsigned32);
}

// Send 32-bit value to given register address. Return true if successful
bool NAU7802::set32BitRegister(uint8_t registerAddress, uint32_t value)
{
  uint8_t _buffer[5];

  _buffer[0] = registerAddress;
  _buffer[1] = (uint8_t)((value >> 24) & 0xFF);
  _buffer[2] = (uint8_t)((value >> 16) & 0xFF);
  _buffer[3] = (uint8_t)((value >> 8) & 0xFF);
  _buffer[4] = (uint8_t)(value & 0xFF);

  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, _buffer, sizeof(_buffer), 50));

  return (true);
}

//Helper methods
int32_t NAU7802::getChannel1Offset() { return get24BitRegister(NAU7802_OCAL1_B2); }
bool NAU7802::setChannel1Offset(int32_t value) { return set24BitRegister(NAU7802_OCAL1_B2, value); }
uint32_t NAU7802::getChannel1Gain() { return get32BitRegister(NAU7802_GCAL1_B3); }
bool NAU7802::setChannel1Gain(uint32_t value) { return set32BitRegister(NAU7802_GCAL1_B3, value); }
