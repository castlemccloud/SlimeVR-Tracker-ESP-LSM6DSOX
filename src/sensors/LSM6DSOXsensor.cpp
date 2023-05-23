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
#include "calibration.h"
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

    
    if( !mag.begin() ) {
        m_Logger.fatal("Can't connect to Magneto at address 0x%02x", _ADDRESS_MAG);
        return;
    }
    

    m_Logger.info("Connected to %s on 0x%02x. ",
                  getIMUNameByType(sensorType), 
                  addr
                );
    

    // TODO: Conditional to trigger re-calibration
    if( 0 ) {
        startCalibration( 0 );
    }



{   // Initialize the configuration
    SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type) {
    case SlimeVR::Configuration::CalibrationConfigType::LSM6DSOX:
        m_Calibration = sensorCalibration.data.lsm6dsox;
        break;

    case SlimeVR::Configuration::CalibrationConfigType::NONE:
        m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
        break;

    default:
        m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
    }
}

    working = true;
};

void LSM6DSOXsensor::motionLoop()
{

    // Update Time
    now = micros();
    deltat = now - last;
    last = now;

    // Read Sensors
    imu.readGyroscope( Gxyz );
    imu.readAcceleration( Axyz );
    
    mag.getData( Mxyz );


{    // Apply calibation data to Mag readings
    float temp[3];
    for( int i = 0; i < 3; i++ ) {
        temp[i] = Mxyz[i] - m_Calibration.M_B[i];
    }
    for( int i = 0; i < 3; i++ ) {
        #if useFullCalibrationMatrix == true
            Mxyz[i] = m_Calibration.M_Ainv[i][0] * temp[0] + m_Calibration.M_Ainv[i][1] * temp[1] + m_Calibration.M_Ainv[i][1] * temp[1];
        #else
            Mxyz[i] = temp[i];
        #endif
    }
}


#if ENABLE_INSPECTION
{
    Network::sendInspectionRawIMUData(sensorId, Gxyz[0], Gxyz[1], Gxyz[2], 255, Axyz[0], Axyz[1], Axyz[2], 255, Mxyz[0], Mxyz[1], Mxyz[2], 255);
}
#endif


    madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], -Mxyz[2], deltat * 1.0e-6f);

    quaternion.set(-q[2], q[1], q[3], q[0]);

#if SEND_ACCELERATION
    {
        // Use the same mapping as in quaternion.set(-q[2], q[1], q[3], q[0]);
        this->linearAcceleration[0] = -Axyz[1];
        this->linearAcceleration[1] = Axyz[0];
        this->linearAcceleration[2] = Axyz[2];

        // get the component of the acceleration that is gravity
        VectorFloat gravity;
        gravity.x = 2 * (this->quaternion.x * this->quaternion.z - this->quaternion.w * this->quaternion.y);
        gravity.y = 2 * (this->quaternion.w * this->quaternion.x + this->quaternion.y * this->quaternion.z);
        gravity.z = this->quaternion.w * this->quaternion.w - this->quaternion.x * this->quaternion.x - this->quaternion.y * this->quaternion.y + this->quaternion.z * this->quaternion.z;
        
        // subtract gravity from the acceleration vector
        this->linearAcceleration[0] -= gravity.x;
        this->linearAcceleration[1] -= gravity.y;
        this->linearAcceleration[2] -= gravity.z;
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


void LSM6DSOXsensor::startCalibration(int calibrationType) {
    ledManager.on();
    // with DMP, we just need mag data
    constexpr int calibrationSamples = 500;

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering magnetometer data");
    ledManager.pattern(15, 300, 3000/310);
    MagnetoCalibration *magneto = new MagnetoCalibration();
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t m[3];
        mag.getRaw(m);
        magneto->sample(m[0], m[1], m[2]);

        float rawMagFloat[3] = { (float)m[0], (float)m[1], (float)m[2]};
        Network::sendRawCalibrationData(rawMagFloat, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");

    float M_BAinv[4][3];
    magneto->current_calibration(M_BAinv);
    delete magneto;

    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::LSM6DSOX;
    calibration.data.lsm6dsox = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
}

