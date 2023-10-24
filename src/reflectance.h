#ifndef __REFLECTANCE_H
#define __REFLECTANCE_H

#define REF_INTERVAL 16

//struct ReflectanceData
//{
//  uint8_t dummy;
//  uint16_t values[2];
//};

class ReflectanceSensor
{
protected:
  uint8_t pin;

  //calibrated min and max
  int16_t minVal;
  int16_t maxVal;

public:
  ReflectanceSensor(uint8_t p) : pin(p)
  {}

  virtual void Calibrate(uint32_t timeMS)
  {
    minVal = 65535;
    maxVal = 0;

    uint32_t startTime = millis();
    while(millis() - startTime < timeMS)
    {
      uint16_t val = ReadRaw();
      if(val < minVal) minVal = val;
      if(val > maxVal) maxVal = val;
      delay(2); //let's not overdo it...
    }
  }

protected:
  virtual uint16_t ReadRaw(void) = 0; //raw value

public:  
  int16_t ReadScaled(void)     //returns a 'gray' value from [0, 1024]
  {
    uint16_t raw = ReadRaw();

    if(raw < minVal) raw = minVal;
    if(raw > maxVal) raw = maxVal;
    return ((raw - minVal) * 1024) / (maxVal - minVal);
  }
};

class ReflectanceSensorRC : public ReflectanceSensor
{
public:
  ReflectanceSensorRC(uint8_t p) : ReflectanceSensor(p)
  {}

protected:
  uint16_t ReadRaw(void)
  {
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
    delayMicroseconds(100);
    uint32_t startTime = micros();
    pinMode(pin, INPUT);
    while(digitalRead(pin)) {}
    uint32_t duration = micros() - startTime;
    if(duration > 65535) duration = 65535;
    return duration;
  }
};

class ReflectanceSensorAnalog : public ReflectanceSensor
{
public:
  ReflectanceSensorAnalog(uint8_t p) : ReflectanceSensor(p)
  {}
  
protected:
  uint16_t ReadRaw(void)
  {
    return analogRead(pin);
  }
};

#endif
