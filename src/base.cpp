#include "../include/base.h"

#include <iostream>
using namespace std;

Base::Base()
{
    wiringPiSetup();
    cout << "wiringPi Setup successfully..." << endl;
    _configured = false;
}

bool Base::isConfigured()
{
    return _configured;
}

void Base::ConfigureOLED(int address, bool type, bool size, int rotation)
{

}

void Base::ConfigureDS18B20(const char* address)
{
    try
    {
        _ds18b20Device.SetAddress(address);
        _configured = true;
    }
    catch (int error_code)
    {
        _configured = false;
    }
}

void Base::ConfigureMQTT(const char* host, int port, const char* id)
{
    m = new mosqpp::mosquittopp(id, false);
    m->connect(host, port, 60);
    char resultstring[32] = "MQTT configured...";
    PublishMessage("test/topic", 32, resultstring);
}

void Base::ConfigureDHT (int pin, int DHTType)
{
    dht.Configure(pin, DHTType);
    dht.begin();
}

void Base::ConfigurePCF8574(const int address)
{
    pcf8574Setup(pcf8574PinOffset,address);
    cout << "Configured PCF8574 with address: " << address << endl;
    for (int i = 0; i < 2; i++)
    {
        pinMode(pcf8574PinOffset + i, OUTPUT);
    }
    for (int i = 2; i < 8; i++)
    {
        pinMode(pcf8574PinOffset + i, INPUT);
    }
}


// OLED Functions
void Base::WriteOLED(string message)
{

}


// DS18B20 Functions
double Base::ReadBaseTemperature() // (from DS18B20)
{
    double value = -99999.0;

    try
    {
        value = _ds18b20Device.getTemp();
    }
    catch (int error_code)
    {
        value = -99999.0;
    }

    return value;
}


// DHT Functions
double Base::ReadTemperature() // (from DHT22 if installed)  --> need to differentiate between DHT & DS18B20
{
    //return dht.read_dht22_dat();
    return dht.readTemperature();
    //return -99999;
}

double Base::ReadHumidity() // (from DHT22 if installed)
{
    return dht.readHumidity();
    //return -99999.0;
}


// MQTT Functions
void Base::PublishMessage(const char* topic, const int length, const char* message)
{
    m->publish(NULL, topic, length, message, 0, false);
}

/*void Base::SubscribeMessage(string message, callback) ;// (callback 	provided in main automation program)
{

}*/


// PCF8574 Functions
// define DIO pins (unless pin layout fixed?)
void Base::WritePin(int pin, bool value)
{
    digitalWrite(pcf8574PinOffset + pin, value);
}

void Base::ToggleOutput(int pin, bool startValue, int delayTime)
{
    digitalWrite(pin, HIGH);
    delay(delayTime);
    digitalWrite(pin, LOW);
    delay(delayTime);
}

bool Base::ReadPin(int pin)
{
    return digitalRead(pcf8574PinOffset + pin);
}

int Base::ReadAllPins()
{
    return 0;
}
