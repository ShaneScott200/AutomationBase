#include <iostream>
#include "./include/base.h"
#include <wiringPi.h>

using namespace std;

int main()
{
    cout << "Starting..." << endl;
    int counter = 0;
    Base b;

    // OLED
    int oledAddress = 0x3c;
    bool oledType = true;
    bool oledSize = true;
    int oledRotation= 0;
    b.ConfigureOLED(oledAddress, oledType, oledSize, oledRotation);

    // base temperature
    char w1_address[16] = "28-041752e4cdff";
    b.ConfigureDS18B20(w1_address);
    cout << "Temp: " << b.ReadBaseTemperature() << endl;

    // temperature/humidity
    b.ConfigureDHT(2, 11);
    b.ReadTemperature();
    //cout << "Temp: " << b.ReadTemperature() << endl;
    //cout << "Humidity: " << b.ReadHumidity() << endl;

    // DIO with pcf8574
    cout << "Toggle pin 0 & 1HIGH and LOW (5 times): " <<  endl;
    int pcf8574Address = 0x20;
    b.ConfigurePCF8574(pcf8574Address);
    counter = 0;
    while (counter < 5)
    {
        //b.ToggleOutput(0, HIGH, 500);
        cout << "Iteration: " <<
            counter << endl;

        b.WritePin(0, 1);
        b.WritePin(1, 0);
        b.ReadBaseTemperature();    // used as a delay

        b.WritePin(0, 0);
        b.WritePin(1, 1);
        b.ReadBaseTemperature();

        counter++;
    }

    // MQTT
    char* hostAddress = "192.168.0.200";
    int port = 1883;
    char* id = "test";
    cout << "Configure MQTT" << endl;
    b.ConfigureMQTT(hostAddress, port, id);
    cout << "MQTT Configured" << endl;
    b.PublishMessage("test/anothertopic", 52, "this is a very long message from automation program");

    counter = 0;
    while (counter < 100)
    {
        float bt = b.ReadBaseTemperature();
        float t = b.ReadTemperature();
        float h = b.ReadHumidity();
        cout << "Iteration: " << counter << "\t" /*<< bt << "\t" << t << "\t" << h << endl*/;
        counter++;
    }

    return 0;
}
