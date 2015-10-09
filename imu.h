/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#ifndef _imu_h
#define _imu_h

#include "aq.h"
#include "adc.h"
#include "d_imu.h"

#define IMU_ROOM_TEMP		20.0f
#define IMU_STATIC_STD		0.05f
#define IMU_STATIC_TIMEOUT	5	// seconds

// these define where to get certain data
#ifndef BYPASS_UKF
#define AQ_YAW			navUkfData.yaw
#define AQ_PITCH		navUkfData.pitch
#define AQ_ROLL			navUkfData.roll
#define AQ_PRES_ADJ		UKF_PRES_ALT
#else 
#define AQ_YAW                  simu.yaw 
#define AQ_PITCH                simu.pitch 
#define AQ_ROLL                 simu.roll 
#define AQ_PRES_ADJ             simu.alt 
#endif 


#ifndef USE_SIMU
#ifdef USE_DIGITAL_IMU
// using the Digital IMU as IMU
#ifdef DIMU_HAVE_MAX21100
#define IMU_DRATEX		max21100Data.dRateGyo[0]
#define IMU_DRATEY		max21100Data.dRateGyo[1]
#define IMU_DRATEZ		max21100Data.dRateGyo[2]
#define IMU_RATEX		max21100Data.gyo[0]
#define IMU_RATEY		max21100Data.gyo[1]
#define IMU_RATEZ		max21100Data.gyo[2]
#define IMU_RAW_RATEX   max21100Data.rawGyo[0]
#define IMU_RAW_RATEY   max21100Data.rawGyo[1]
#define IMU_RAW_RATEZ   max21100Data.rawGyo[2]
#define IMU_ACCX		max21100Data.acc[0]
#define IMU_ACCY		max21100Data.acc[1]
#define IMU_ACCZ		max21100Data.acc[2]
#define IMU_RAW_ACCX    max21100Data.rawAcc[0]
#define IMU_RAW_ACCY    max21100Data.rawAcc[1]
#define IMU_RAW_ACCZ    max21100Data.rawAcc[2]
#else
#define IMU_DRATEX		mpu6000Data.dRateGyo[0]
#define IMU_DRATEY		mpu6000Data.dRateGyo[1]
#define IMU_DRATEZ		mpu6000Data.dRateGyo[2]
#define IMU_RATEX		mpu6000Data.gyo[0]
#define IMU_RATEY		mpu6000Data.gyo[1]
#define IMU_RATEZ		mpu6000Data.gyo[2]
#define IMU_RAW_RATEX   mpu6000Data.rawGyo[0]
#define IMU_RAW_RATEY   mpu6000Data.rawGyo[1]
#define IMU_RAW_RATEZ   mpu6000Data.rawGyo[2]
#define IMU_ACCX		mpu6000Data.acc[0]
#define IMU_ACCY		mpu6000Data.acc[1]
#define IMU_ACCZ		mpu6000Data.acc[2]
#define IMU_RAW_ACCX    mpu6000Data.rawAcc[0]
#define IMU_RAW_ACCY    mpu6000Data.rawAcc[1]
#define IMU_RAW_ACCZ    mpu6000Data.rawAcc[2]
#endif
#define IMU_MAGX		hmc5983Data.mag[0]
#define IMU_MAGY		hmc5983Data.mag[1]
#define IMU_MAGZ		hmc5983Data.mag[2]
#define IMU_RAW_MAGX    hmc5983Data.rawMag[0]
#define IMU_RAW_MAGY    hmc5983Data.rawMag[1]
#define IMU_RAW_MAGZ    hmc5983Data.rawMag[2]
#define IMU_TEMP		dImuData.temp
#define IMU_LASTUPD		dImuData.lastUpdate
#define AQ_OUTER_TIMESTEP	DIMU_OUTER_DT
#define AQ_INNER_TIMESTEP	DIMU_INNER_DT
#define AQ_PRESSURE		ms5611Data.pres
#define AQ_MAG_ENABLED          hmc5983Data.enabled
#endif	// USE_DIGITAL_IMU

#ifndef USE_DIGITAL_IMU
// using ADC as IMU
#define IMU_DRATEX		adcData.dRateX
#define IMU_DRATEY		adcData.dRateY
#define IMU_DRATEZ		adcData.dRateZ
#define IMU_RATEX		adcData.rateX
#define IMU_RATEY		adcData.rateY
#define IMU_RATEZ		adcData.rateZ
#define IMU_RAW_RATEX   adcData.voltages[ADC_VOLTS_RATEX]
#define IMU_RAW_RATEY   adcData.voltages[ADC_VOLTS_RATEY]
#define IMU_RAW_RATEZ   adcData.voltages[ADC_VOLTS_RATEZ]
#define IMU_ACCX		adcData.accX
#define IMU_ACCY		adcData.accY
#define IMU_ACCZ		adcData.accZ
#define IMU_RAW_ACCX    adcData.voltages[ADC_VOLTS_ACCX]
#define IMU_RAW_ACCY    adcData.voltages[ADC_VOLTS_ACCY]
#define IMU_RAW_ACCZ    adcData.voltages[ADC_VOLTS_ACCZ]
#define IMU_MAGX		adcData.magX
#define IMU_MAGY		adcData.magY
#define IMU_MAGZ		adcData.magZ
#define IMU_RAW_MAGX    adcData.voltages[ADC_VOLTS_MAGX]
#define IMU_RAW_MAGY    adcData.voltages[ADC_VOLTS_MAGY]
#define IMU_RAW_MAGZ    adcData.voltages[ADC_VOLTS_MAGZ]
#define IMU_TEMP		adcData.temperature
#define IMU_LASTUPD		adcData.lastUpdate
#define AQ_OUTER_TIMESTEP	adcData.dt
#define AQ_INNER_TIMESTEP	(adcData.dt * 0.5f)
#define AQ_PRESSURE		adcData.pressure
#define AQ_MAG_ENABLED          1
#endif

#else 
#include "../hils/hils.h"
#define IMU_DRATEX              simu.gyro[0] 
#define IMU_DRATEY              simu.gyro[1] 
#define IMU_DRATEZ              simu.gyro[2] 
#define IMU_RATEX               simu.gyro[0] 
#define IMU_RATEY               simu.gyro[1] 
#define IMU_RATEZ               simu.gyro[2] 
#define IMU_ACCX                simu.acc[0] 
#define IMU_ACCY                simu.acc[1] 
#define IMU_ACCZ                simu.acc[2] 
#define IMU_MAGX                simu.mag[0] 
#define IMU_MAGY                simu.mag[1] 
#define IMU_MAGZ                simu.mag[2] 
#define IMU_TEMP                simu.temperature 
#define IMU_LASTUPD             sImuData.lastUpdate 
#define AQ_OUTER_TIMESTEP       SIMU_OUTER_DT 
#define AQ_INNER_TIMESTEP       SIMU_INNER_DT 
#define AQ_PRESSURE             simu.pSensor 
#define AQ_MAG_ENABLED          1 
#endif
typedef struct {
    OS_FlagID dRateFlag;
    OS_FlagID sensorFlag;
    float sinRot, cosRot;
    uint32_t fullUpdates;
    uint32_t halfUpdates;
} imuStruct_t;

extern imuStruct_t imuData;

extern void imuInit(void);
extern void imuQuasiStatic(int n);
extern void imuAdcDRateReady(void);
extern void imuAdcSensorReady(void);
extern void imuDImuDRateReady(void);
extern void imuDImuSensorReady(void);

#endif
