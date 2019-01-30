#ifndef BASE_H_
#define BASE_H_

#include <string>
//#include "ArduiPi_OLED_lib.h"
//#include "ArduiPi_OLED.h"
//#include "Adafruit_GFX.h"
#include "DS18B20.h"
#include </usr/include/mosquittopp.h>
#include "ArduiDHT.h"
#include </usr/include/pcf8574.h>
#include </usr/include/wiringPi.h>

using namespace std;

class Base
{
    public:
        // Default Constructor
        Base();
        // Overload Constructor
        //Base(..arguments…)

        // Configuration Function
        void ConfigureOLED(int address, bool type, bool size, int rotation);
        void ConfigureDS18B20(const char* address);
        void ConfigureMQTT(const char* host, int port, const char* id);
        void ConfigureDHT (int pin, int DHTType);
        void ConfigurePCF8574(int address);
        bool isConfigured();

        // OLED Functions
        void WriteOLED(string message);

        // DS18B20 Functions
        double ReadBaseTemperature(); // (from DS18B20)

        // DHT Functions
        double ReadTemperature(); // (from DHT22 if installed)  --> need to differentiate between DHT & DS18B20
        double ReadHumidity(); // (from DHT22 if installed)

        // MQTT Functions
        void PublishMessage(const char* topic, const int length, const char* message);
        //void SubscribeMessage(string message, callback) ;// (callback 	provided in main automation program)

        // PCF8574 Functions
        // define DIO pins (unless pin layout fixed?)
        void WritePin(int pin, bool value);
        void ToggleOutput(int pin, bool startValue, int delayTime);
        bool ReadPin(int pin);
        int ReadAllPins();

    private:
        bool _configured;
        DS18B20 _ds18b20Device;
        mosqpp::mosquittopp* m;
        DHT dht;
        int pcf8574PinOffset = 100;
};

#endif /* BASE_H_ */
