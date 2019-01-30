#ifndef DHT_H_INCLUDED
#define DHT_H_INCLUDED

#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define MAXTIMINGS	85
#define DHTPIN		7


class DHT
{
    public:
        DHT();
        void ReadDHT();

    private:
        int dht11_dat[5];


};

#endif // DHT_H_INCLUDED
