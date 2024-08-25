// ADS115-Driver Library Ver 1.0.2 by Wh11eRabbitHU


// initialising the ADS1115
#include "ADS1115-Driver.h"

ADS1115 ads1115 = ADS1115(0x48);
  int MinV=0;
  int MaxV = 3800;
uint16_t readValue(uint8_t input) 
  {
	  ads1115.setMultiplexer(input);
	  ads1115.startSingleConvertion();
	  delayMicroseconds(25); // The ADS1115 needs to wake up from sleep mode and usually it takes 25 uS to do that
	  while (ads1115.getOperationalStatus() == 0);
	  return ads1115.readConvertedValue();
  }



void setup() {
	Serial.begin(9600);

// Settingup the ADS1115
	ads1115.reset();
	ads1115.setDeviceMode(ADS1115_MODE_SINGLE);
	ads1115.setDataRate(ADS1115_DR_250_SPS);
	ads1115.setPga(ADS1115_PGA_4_096);

  
}

void loop() {

Serial.print("Channel 0: "); Serial.print(getMilliVolt(0)); Serial.print("mV , Angle: "); Serial.println(getAngle(0));
Serial.print("Channel 1: "); Serial.print(getMilliVolt(1)); Serial.print("mV , Angle: "); Serial.println(getAngle(1));
Serial.print("Channel 2: "); Serial.print(getMilliVolt(2)); Serial.print("mV , Angle: "); Serial.println(getAngle(2));
Serial.print("Channel 3: "); Serial.print(getMilliVolt(3)); Serial.print("mV , Angle: "); Serial.println(getAngle(3));
Serial.println(" ");

delay(2000);

}

int getAngle(int Channel)
{
  //return the angle in degrees
  return map (readValue(Channel + 4), MinV,MaxV, 0,360);
}

int getMilliVolt(int Channel)
{
  //return the voltage in mV
  return readValue(Channel + 4);
}