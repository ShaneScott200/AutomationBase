/* DHT library

*/
#ifndef DHT_H
#define DHT_H

#include <wiringPi.h>

// Define types of sensors.
#define DHT11 11
#define DHT12 12
#define DHT22 22
#define DHT21 21
#define AM2301 21


class DHT {
  public:
   DHT();
   DHT(int pin, int type, int count=6);
   void Configure(int pin, int type);
   void begin(void);
   float readTemperature(bool S=false, bool force=false);
   float convertCtoF(float);
   float convertFtoC(float);
   float readHumidity(bool force=false);
   bool read(bool force=false);

   int read_dht22_dat();

 private:
  int data[5];
  int _pin, _type;
  long _lastreadtime, _maxcycles;
  bool _lastresult;

  long expectPulse(bool level);

};

/*class InterruptLock {
  public:
   InterruptLock();
   ~InterruptLock();

};
*/
#endif
