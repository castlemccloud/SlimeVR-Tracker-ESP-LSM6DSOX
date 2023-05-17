/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2023 SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "sensors/LSM6DSOXsensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "GlobalVars.h"


void LSM6DSOXsensor::motionSetup()
{

#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    if( !imu.begin() ) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        return;
    }

    
    if( !magneto.begin() ) {
        m_Logger.fatal("Can't connect to Magneto at address 0x%02x", _ADDRESS_MAG);
        return;
    }
    

    m_Logger.info("Connected to %s on 0x%02x. ",
                  getIMUNameByType(sensorType), 
                  addr
                );

    working = true;
};

void LSM6DSOXsensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        imu.readGyroscope(&rX, &rY, &rZ);
        imu.readAcceleration(&aX, &aY, &aZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#endif

    now = micros();
    deltat = now - last; //seconds since last update
    last = now;

    float Gxyz[3] = {0};
    float Axyz[3] = {0};
    float Mxyz[3] = {0};

    imu.readGyroscope( Gxyz );
    imu.readAcceleration( Axyz );
    
    magneto.getData( Mxyz );

    //mahonyQuaternionUpdate(q, -Axyz[0], Axyz[1], -Axyz[2], -Gxyz[0], Gxyz[1], -Gxyz[2], -Mxyz[0], Mxyz[1], -Mxyz[2], deltat * 1.0e-6f);
    mahonyQuaternionUpdate(q, -Axyz[0], Axyz[1], -Axyz[2], -Gxyz[0], Gxyz[1], -Gxyz[2], deltat * 1.0e-6f);
    quaternion.set(-q[2], q[1], q[3], q[0]);

#if SEND_ACCELERATION
    {
        // Use the same mapping as in quaternion.set(-q[2], q[1], q[3], q[0]);
        this->acceleration[0] = -Axyz[1];
        this->acceleration[1] = Axyz[0];
        this->acceleration[2] = Axyz[2];

        // get the component of the acceleration that is gravity
        VectorFloat gravity;
        gravity.x = 2 * (this->quaternion.x * this->quaternion.z - this->quaternion.w * this->quaternion.y);
        gravity.y = 2 * (this->quaternion.w * this->quaternion.x + this->quaternion.y * this->quaternion.z);
        gravity.z = this->quaternion.w * this->quaternion.w - this->quaternion.x * this->quaternion.x - this->quaternion.y * this->quaternion.y + this->quaternion.z * this->quaternion.z;
        
        // subtract gravity from the acceleration vector
        this->acceleration[0] -= gravity.x;
        this->acceleration[1] -= gravity.y;
        this->acceleration[2] -= gravity.z;
    }
#endif

    quaternion *= sensorOffset;

#if ENABLE_INSPECTION
    {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
    }
#endif

    if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }

};


