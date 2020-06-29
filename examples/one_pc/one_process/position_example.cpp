/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "laikago_sdk/laikago_sdk.hpp"
#include <math.h>
#include <iostream>

#include <sys/time.h>
struct timeval tbegin;
struct timeval tend;

using namespace laikago;

class CustomData
{
public:
    CustomData(): udp(LOW_CMD_LENGTH, LOW_STATE_LENGTH){
        this->motiontime = 0;
        this->Tpi = 0;
        this->cmd = {0};
        this->state = {0};
    }
    
    UDP udp;
    LowCmd cmd;
    LowState state;
    float movejoint1;
    float movejoint2;
    float Kv[3];  
    float Kp[3];
    double time_consume;
    int Tpi;
    int motiontime;
};

void Print(void *param)
{
    CustomData *data = (CustomData *)param;  
    printf("%f    %f    %f\n", data->cmd.motorCmd[FR_1].position, data->state.motorState[FR_1].position, data->time_consume);
}

void UDPRecv(void *param)
{
    UDP *data = (UDP *)param;  
    data->Recv();
}

void RobotControl(void *param) 
{
    struct CustomData *data;
    data = (struct CustomData *)param;

    data->motiontime++;
    data->udp.GetState(data->state);

    // gravity compensation
    data->cmd.motorCmd[FR_0].torque = -0.65f;
    data->cmd.motorCmd[FL_0].torque = +0.65f;
    data->cmd.motorCmd[RR_0].torque = -0.65f;
    data->cmd.motorCmd[RL_0].torque = +0.65f;

    if( data->motiontime >= 100){
	    if( data->motiontime == 100){
            data->Kp[0] = 0.132; data->Kp[1] = 0.132; data->Kp[2] = 0.132; 
            data->Kv[0] = 0.02; data->Kv[1] = 0.02; data->Kv[2] = 0.02; 
	    }
	    if( data->motiontime == 500){
            data->Kp[0] = 0.5; data->Kp[1] = 0.15; data->Kp[2] = 0.15; 
            data->Kv[0] = 0.02; data->Kv[1] = 0.02; data->Kv[2] = 0.02; 
	    }
	    if( data->motiontime == 900){
            data->Kp[0] = 0.8; data->Kp[1] = 0.18; data->Kp[2] = 0.18; 
            data->Kv[0] = 0.02; data->Kv[1] = 0.02; data->Kv[2] = 0.02; 
	    }
	    if( data->motiontime == 1300){
            data->Kp[0] = 1.2; data->Kp[1] = 0.22; data->Kp[2] = 0.22; 
            data->Kv[0] = 0.02; data->Kv[1] = 0.02; data->Kv[2] = 0.02; 
	    }
	    if( data->motiontime == 1700){
            data->Kp[0] = 1.84; data->Kp[1] = 0.263; data->Kp[2] = 0.263; 
            data->Kv[0] = 0.02; data->Kv[1] = 0.02; data->Kv[2] = 0.02; 
	    }
        // Move leg
        if(data->motiontime >2000){
            data->Tpi++;
            data->movejoint1 = 0.25 * sin(4*M_PI*data->Tpi/1000.0);
            data->movejoint2 = 0.4 * sin(4*M_PI*data->Tpi/1000.0);
        }
        
        data->cmd.motorCmd[FR_0].position = 0.0;
        data->cmd.motorCmd[FR_0].velocity = 0;
        data->cmd.motorCmd[FR_0].positionStiffness = data->Kp[0];
        data->cmd.motorCmd[FR_0].velocityStiffness = data->Kv[0];
        data->cmd.motorCmd[FR_0].torque = 0.65f;

        data->cmd.motorCmd[FR_1].position = 0.5 + data->movejoint2;
        data->cmd.motorCmd[FR_1].velocity = 0;
        data->cmd.motorCmd[FR_1].positionStiffness = data->Kp[1];
        data->cmd.motorCmd[FR_1].velocityStiffness = data->Kv[1];
        data->cmd.motorCmd[FR_1].torque = 0.0f;

        data->cmd.motorCmd[FR_2].position =  -1.1 + data->movejoint1;
        data->cmd.motorCmd[FR_2].velocity = 0;
        data->cmd.motorCmd[FR_2].positionStiffness = data->Kp[2];
        data->cmd.motorCmd[FR_2].velocityStiffness = data->Kv[2];
        data->cmd.motorCmd[FR_2].torque = 0.0f;
    }

    // Control::PositionLimit(data->cmd);
    // Control::PowerProtect(data->cmd, data->state, 1);
    // Control::PositionProtect(data->cmd, data->state, 0.087);

    data->udp.Send(data->cmd);

    gettimeofday(&tend, NULL);
    data->time_consume = ((tend.tv_sec-tbegin.tv_sec)*1000000+(tend.tv_usec-tbegin.tv_usec))/1000.0;
    tbegin = tend;
}

int main(void)
{
    std::cout << "Control level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Control control(LOWLEVEL);
    CustomData custom;
    control.loop.SetPrint(true);
    control.loop.SetPrintPeriod(2000); //4ms
    control.InitCmdData(custom.cmd);
    control.loop.RegistFunc("UDP/Send", RobotControl, &custom);
    control.loop.RegistFunc("UDP/Recv", UDPRecv, &custom.udp);
    control.loop.RegistFunc("PRINT", Print, &custom);
    control.loop.Start();
    return 0; 
}
