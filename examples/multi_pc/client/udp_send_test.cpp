/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// Notice: This exemple should running on another PC, and make sure the Ethernet is stable.

#include "laikago_sdk/laikago_sdk.hpp"
#include <math.h>

using namespace laikago;

struct AAA{
    int direction;
    float deepth;
    uint32_t crc;
};

struct BBB{
    float yaw;
    float pitch;
    float roll;
    uint32_t crc;
};

class CustomData
{
public:
    CustomData(): udp(8017, "192.168.2.100", 8018, sizeof(AAA), sizeof(BBB)){}
    
    UDP udp;
    AAA a;
    BBB b;
};

void UDPRecv(void *param)
{
    UDP *data = (UDP *)param;
    data->Recv();
}

void UDPSend(void *param) 
{
    CustomData *data = (CustomData *)param;
    data->udp.GetState((char*)&data->b);
    printf("%f\n", data->b.yaw);

    data->a.direction += 1000;
    data->udp.Send((char*)&data->a);
}

int main(void) 
{
    LOOP loop;
    CustomData custom;
    loop.SetUDPPeriod(10000);
    loop.RegistFunc("UDP/Send", UDPSend, &custom);
    loop.RegistFunc("UDP/Recv", UDPRecv, &custom.udp);
    loop.Start();

    return 0; 
}