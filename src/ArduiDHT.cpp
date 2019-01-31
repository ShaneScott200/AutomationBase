/* DHT library


*/
#include <iostream>
using namespace std;
#include "../include/ArduiDHT.h"

#define MIN_INTERVAL 2000
#define TIMEOUT -1

DHT::DHT() {

}

DHT::DHT(int pin, int type, int count) {
  _pin = pin;
  _type = type;
  _maxcycles = 100000;//microsecondsToClockCycles(1000);  // 1 millisecond timeout for reading pulses from DHT sensor.
}

void DHT::Configure(int pin, int type)
{
    _pin = pin;
    _type = type;
    _maxcycles = 1000;//microsecondsToClockCycles(1000);  // 1 millisecond timeout for reading pulses from DHT sensor.

}

void DHT::begin(void) {
  // set up the pins!
  pinMode(_pin, INPUT);
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = millis() - MIN_INTERVAL;
//  DEBUG_PRINT("DHT max clock cycles: "); DEBUG_PRINTLN(_maxcycles, DEC);
}

//boolean S == Scale.  True == Fahrenheit; False == Celcius
float DHT::readTemperature(bool S, bool force) {
  float f = -88888.0;

  if (read(force)) {
    switch (_type) {
    case DHT11:
    case DHT12:
      f = data[2];
      f += (data[3] & 0x0f) * 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if(S) {
        f = convertCtoF(f);
      }
      break;
    case DHT22:
    case DHT21:
        //f = ((word)(data[2] & 0x7F)) << 8 | data[3];
        /*f = ((data[2] & 0x7F)) << 8 | data[3];
        f *= 0.1;
        if (data[2] & 0x80) {
        f *= -1;
        }*/
        f = (float)(data[2] & 0x7F)* 256 + (float)data[3];
        f /= 10.0;
        if ((data[2] & 0x80) != 0)  f *= -1;
        if(S)
        {
            f = convertCtoF(f);
        }
        break;
    }
  }
  return f;
}

float DHT::convertCtoF(float c) {
  return c * 1.8 + 32;
}

float DHT::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float DHT::readHumidity(bool force) {
  float f = -88888.0;
  if (read(force)) {
    switch (_type) {
    case DHT11:
    case DHT12:
      f = data[0] + data[1] * 0.1;
      break;
    case DHT22:
    case DHT21:
      //f = ((word)data[0]) << 8 | data[1];
      //f = (data[0]) << 8 | data[1];
      //f *= 0.1;
        f = (float)data[0] * 256 + (float)data[1];
        f /= 10;
      break;
    }
  }
  return f;
}
/*
bool DHT::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  long currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < MIN_INTERVAL)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and start the reading process.
  pinMode(_pin, INPUT);
  delay(1);

  // First set data line low for a period according to sensor type
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  switch(_type) {
    case DHT22:
    case DHT21:
      delayMicroseconds(1100); // data sheet says "at least 1ms"
      break;
    case DHT11:
    default:
      delay(20); //data sheet says at least 18ms, 20ms just to be safe
      break;
  }

  long cycles[80];
  // End the start signal by setting data line high for 40 microseconds.
    pinMode(_pin, INPUT);

    // Now start reading the data line to get the value from the DHT sensor.
    delayMicroseconds(60);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == TIMEOUT) {
      //DEBUG_PRINTLN(F("DHT timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == TIMEOUT) {
      //DEBUG_PRINTLN(F("DHT timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i)
  {
    long lowCycles  = cycles[2*i];
    long highCycles = cycles[2*i+1];
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT))
    {
      //DEBUG_PRINTLN(F("DHT timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles)
    {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  //DEBUG_PRINTLN(F("Received from DHT:"));
  //DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
  //DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  }
  else {
    //DEBUG_PRINTLN(F("DHT checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}
*/
/*
// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
long DHT::expectPulse(bool level) {

  long count = 0;

  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return TIMEOUT; // Exceeded timeout, fail.
      }
    }

  return count;
}
*/


/*static int sizecvt(const int read)
{
  // digitalRead() and friends from wiringpi are defined as returning a value
  //< 256. However, they are returned as int() types. This is a safety function

  if (read > 255 || read < 0)
  {
    printf("Invalid data from wiringPi library\n");
    exit(EXIT_FAILURE);
  }
  return (int)read;
}*/

bool DHT::read(bool force)
{
    #define MAXTIMINGS 85
    int DHTPIN = 2;
    //int dht22_dat[5] = {0,0,0,0,0};

    int laststate = HIGH;
    int counter = 0;
    int j = 0, i;

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // pull pin down for 18 milliseconds
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, HIGH);
    delay(500);
    digitalWrite(DHTPIN, LOW);
    delay(20);
    // prepare to read the pin
    pinMode(DHTPIN, INPUT);

    // detect change and read data
    for ( i=0; i< MAXTIMINGS; i++)
    {
        counter = 0;
        //while (sizecvt(digitalRead(DHTPIN)) == laststate) {
        while (digitalRead(DHTPIN) == laststate)
        {
            counter++;
            delayMicroseconds(2);
            if (counter == 255) {
            break;
            }
        }
        //laststate = sizecvt(digitalRead(DHTPIN));
        laststate = digitalRead(DHTPIN);

        if (counter == 255) break;

        // ignore first 3 transitions
        if ((i >= 4) && (i%2 == 0))
        {
            // shove each bit into the storage bytes
            data[j/8] <<= 1;
            if (counter > 16)
            data[j/8] |= 1;
            j++;
        }
    }
}



/// ********** ORIGINAL CODE **********
/*
int DHT::read_dht22_dat()
{
    #define MAXTIMINGS 85
    int DHTPIN = 2;
    int dht22_dat[5] = {0,0,0,0,0};

  int laststate = HIGH;
  int counter = 0;
  int j = 0, i;

  dht22_dat[0] = dht22_dat[1] = dht22_dat[2] = dht22_dat[3] = dht22_dat[4] = 0;

  // pull pin down for 18 milliseconds
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, HIGH);
  delay(500);
  digitalWrite(DHTPIN, LOW);
  delay(20);
  // prepare to read the pin
  pinMode(DHTPIN, INPUT);

  // detect change and read data
  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    //while (sizecvt(digitalRead(DHTPIN)) == laststate) {
    while (digitalRead(DHTPIN) == laststate) {
      counter++;
      delayMicroseconds(2);
      if (counter == 255) {
        break;
      }
    }
    //laststate = sizecvt(digitalRead(DHTPIN));
    laststate = digitalRead(DHTPIN);

    if (counter == 255) break;

    // ignore first 3 transitions
    if ((i >= 4) && (i%2 == 0)) {
      // shove each bit into the storage bytes
      dht22_dat[j/8] <<= 1;
      if (counter > 16)
        dht22_dat[j/8] |= 1;
      j++;
    }
  }

  // check we read 40 bits (8bit x 5 ) + verify checksum in the last byte
  // print it out if data is good
  cout << "Raw Data: " << dht22_dat[0] << " " << dht22_dat[1] << " " << dht22_dat[2] << " " << dht22_dat[3] << " " << dht22_dat[4] << "\t";
  //if ((j >= 40) && (dht22_dat[4] == ((dht22_dat[0] + dht22_dat[1] + dht22_dat[2] + dht22_dat[3]) & 0xFF)) )
  if (true) // skip check altogether
  {
        float t, h;
        h = (float)dht22_dat[0] * 256 + (float)dht22_dat[1];
        h /= 10;
        t = (float)(dht22_dat[2] & 0x7F)* 256 + (float)dht22_dat[3];
        t /= 10.0;
        if ((dht22_dat[2] & 0x80) != 0)  t *= -1;


    cout << "Humidity = " << h << " %\tTemperature = " << t << "*C \n" << endl;
    return 1;
  }
  else
  {
    cout << "Data not good, skip\n" << endl;
    cout << "\tFound: " << j << " points!" << endl;
    cout << "CRC Check: " << dht22_dat[4] << " should equal: " << (dht22_dat[0] + dht22_dat[1] + dht22_dat[2] + dht22_dat[3]) << endl;
    return 0;
  }
}*/
