/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file ms5611.c
 * Driver for the ms5611 pressure sensor from measurement specialties.
 * Datasheet at http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */
#include "ms5611.h"

#define EXTRA_PRECISION      5 // trick to add more precision to the pressure and temp readings
#define CONVERSION_TIME_MS   10 // conversion time in milliseconds. 10 is minimum
#define PRESSURE_PER_TEMP 5 // Length of reading cycle: 1x temp, rest pressure. Good values: 1-10
#define FIX_TEMP 25         // Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
                            // TLDR: Adjusting for temp changes does more harm than good.

typedef struct
{
  uint16_t psens;//Pressure sensitivity
  uint16_t off;  //Pressure offset
  uint16_t tcs;	 //Temperature coefficient of pressure sensitivity
  uint16_t tco;  //Temperature coefficient of pressure offset
  uint16_t tref; //Reference temperature
  uint16_t tsens;//Temperature coefficient of the temperature
} CalReg;

static bool isInit;

static CalReg   calReg;
static uint32_t lastPresConv;
static uint32_t lastTempConv;
static int32_t  tempCache;

static uint8_t readState=0;
static uint32_t lastConv=0;
static int32_t tempDeltaT;

//LOW level Driver
/**************************************************************/
/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
 */
void MS5611_Reset(void)
{
  MS5611_CS_LOW;
  
  SPI1_RW(MS5611_RESET);
  
  MS5611_CS_HIGH;
}

// see page 11 of the datasheet
void MS5611_StartConversion(uint8_t command)
{
  MS5611_CS_LOW;
  
  SPI1_RW(command);
  
  MS5611_CS_HIGH;
}

int32_t MS5611_GetConversion(void)
{
	uint8_t i;
	int32_t conversion = 0;
	uint8_t buffer[MS5611_D1D2_SIZE];
	// start the read sequence
	MS5611_CS_LOW;
	// start read sequence
	SPI1_RW(0);
	// Read conversion
	for(i=0;i<MS5611_D1D2_SIZE;i++)
	{
	  buffer[i] = SPI1_RW(0);
	}
	//end of the read sequence
	MS5611_CS_HIGH;
	
	conversion = ((int32_t)buffer[0] << 16) |((int32_t)buffer[1] << 8) | buffer[2];

	return conversion;
}

int32_t MS5611_RawPressure(uint8_t osr)
{
  uint32_t now = xTaskGetTickCount();
  if (lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME_MS)
  {
    lastPresConv = 0;
    return MS5611_GetConversion();
  }
  else
  {
    if (lastPresConv == 0 && lastTempConv == 0)
    {
      MS5611_StartConversion(MS5611_D1 + osr);
      lastPresConv = now;
    }
    return 0;
  }
}

int32_t MS5611_RawTemperature(uint8_t osr)
{
  uint32_t now = xTaskGetTickCount();
  if (lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME_MS)
  {
    lastTempConv = 0;
    tempCache = MS5611_GetConversion();
    return tempCache;
  }
  else
  {
    if (lastTempConv == 0 && lastPresConv == 0)
    {
      MS5611_StartConversion(MS5611_D2 + osr);
      lastTempConv = now;
    }
    return tempCache;
  }
}
/**
 * Reads factory calibration and store it into object variables.
 */
void MS5611_ReadPROM(void)
{
	uint8_t buffer[MS5611_PROM_REG_SIZE];
	uint16_t* pCalRegU16 = (uint16_t*)&calReg;
	int32_t i = 0,j = 0;
	
	// start the read sequence
	MS5611_CS_LOW;

	for (i = 0; i < MS5611_PROM_REG_COUNT; i++)
	{
		// start read sequence
		SPI1_RW(MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
		// Read conversion

		for(j=0;j < MS5611_PROM_REG_SIZE;j++)
		{
			buffer[j] = SPI1_RW(0);
		}		  
		pCalRegU16[i] = ((uint16_t)buffer[0] << 8) | buffer[1];
	}
	
	//end of the read sequence
	MS5611_CS_HIGH;
}

//at the datasheet page 7 ,describe the in detial
//dT = D2 - TREF = D2 - C5 * 2^8
int32_t MS5611_CalcDeltaTemp(int32_t rawTemp)
{
	if (rawTemp == 0)
	{
		return 0;
	}
	else
	{
		return rawTemp - (((int32_t)calReg.tref) << 8);
	}
}

//TEMP = 20°C+ dT *TEMPSENS= 2000 + dT *C6 / 2^23
float MS5611_CalcTemp(int32_t deltaT) 
{
	if (deltaT == 0)
	{
		return 0;
	}
		else
	{
		return (float)(((1 << EXTRA_PRECISION) * 2000)+ (((int64_t)deltaT * calReg.tsens) >> (23 - EXTRA_PRECISION)))/
					  ((1 << EXTRA_PRECISION) * 100.0);
	}
}

//see the page 7 
//Calculate temperature compensated pressure
float MS5611_CalcPressure(int32_t rawPress, int32_t dT)
{
	int64_t off;
	int64_t sens;

	if (rawPress == 0 || dT == 0)
	{
		return 0;
	}
	// Offset at actual temperature
	// OFF =OFFT1 +TCO* dT =C2 * 2^16 + (C4* dT ) / 2^7
	off  = (((int64_t)calReg.off) << 16) + ((calReg.tco * (int64_t)dT) >> 7);
	
	// Sensitivity at actual temperature
	// SENS =SENST1 +TCS* dT =C1 * 2^15 + (C3 * dT ) / 2^8
	sens = (((int64_t)calReg.psens) << 15) + ((calReg.tcs * (int64_t)dT) >> 8);
	
	// Temperature compensated pressure (10…1200mbar with0.01mbar resolution)
	// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
	return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))/
			((1 << EXTRA_PRECISION) * 100.0);
}

bool ms5611EvaluateSelfTest(float min, float max, float value, char* string)
{
  if (value < min || value > max)
  {
    printf("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, min, max, value);
    return false;
  }
  return true;
}



int32_t MS5611_GetDeltaTemp(uint8_t osr)
{
  int32_t rawTemp = MS5611_RawTemperature(osr);
  if (rawTemp != 0)
  {
    return MS5611_CalcDeltaTemp(rawTemp);
  }
  else
  {
    return 0;
  }
}

float MS5611_GetTemperature(uint8_t osr)
{
  // see datasheet page 7 for formulas
  int32_t dT;

  dT = MS5611_GetDeltaTemp(osr);
  if (dT != 0)
  {
    return MS5611_CalcTemp(dT);
  }
  else
  {
    return 0;
  }
}

float MS5611_GetPressure(uint8_t osr)
{
  // see datasheet page 7 for formulas
  int32_t rawPress = MS5611_RawPressure(osr);
  int64_t dT = (int64_t)MS5611_GetDeltaTemp(osr);
  if (dT == 0)
  {
    return 0;
  }
  int64_t off = (((int64_t)calReg.off) << 16) + ((calReg.tco * dT) >> 7);
  int64_t sens = (((int64_t)calReg.psens) << 15) + ((calReg.tcs * dT) >> 8);
  if (rawPress != 0)
  {
    return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))/
		   ((1 << EXTRA_PRECISION) * 100.0);
  }
  else
  {
    return 0;
  }
}
/***************************Public Function***********************************/
bool MS5611_Init(void)
{
  if (isInit)
    return true;

  MS5611_Reset(); // reset the device to populate its internal PROM registers
  vTaskDelay(M2T(5)); //the datasheet describe the min time is 2.8ms
  MS5611_ReadPROM(); // reads the PROM into object variables for later use

  isInit = true;
  return true;
}

bool MS5611_SelfTest(void)
{
  bool 	testStatus = true;
  int32_t rawPress;
  int32_t rawTemp;
  int32_t deltaT;
  float pressure;
  float temperature;

  if (!isInit)
    return false;

  MS5611_StartConversion(MS5611_D1 + MS5611_OSR_4096);
  vTaskDelay(M2T(CONVERSION_TIME_MS));
  rawPress = MS5611_GetConversion();

  MS5611_StartConversion(MS5611_D2 + MS5611_OSR_4096);
  vTaskDelay(M2T(CONVERSION_TIME_MS));
  rawTemp = MS5611_GetConversion();

  deltaT      = MS5611_CalcDeltaTemp(rawTemp);
  temperature = MS5611_CalcTemp(deltaT);
  pressure    = MS5611_CalcPressure(rawPress, deltaT);

  if (ms5611EvaluateSelfTest(MS5611_ST_PRESS_MIN, MS5611_ST_PRESS_MAX, pressure, "pressure") &&
      ms5611EvaluateSelfTest(MS5611_ST_TEMP_MIN, MS5611_ST_TEMP_MAX, temperature, "temperature"))
  {
    printf("Self test [OK].\n");
  }
  else
  {
   testStatus = false;
  }

  return testStatus;
}

//TODO: pretty expensive function. Rather smooth the pressure estimates and only call this when needed

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float MS5611_PressureToAltitude(float* pressure/*, float* ground_pressure, float* ground_temp*/)
{
    if (*pressure > 0)
    {
        //return (1.f - pow(*pressure / CONST_SEA_PRESSURE, CONST_PF)) * CONST_PF2;
        //return ((pow((1015.7 / *pressure), CONST_PF) - 1.0) * (25. + 273.15)) / 0.0065;
        return ((pow((1015.7 / *pressure), CONST_PF) - 1.0) * (FIX_TEMP + 273.15)) / 0.0065;
    }
    else
    {
        return 0;
    }
}


/**
 * Gets pressure, temperature and above sea level altitude estimate (asl).
 * Best called at 100hz. For every PRESSURE_PER_TEMP-1 pressure readings temp is read once.
 * Effective 50-90hz baro update and 50-10hz temperature update if called at 100hz.
 */
void MS5611_GetData(float* pressure, float* temperature, float* asl)
{
    int32_t tempPressureRaw, tempTemperatureRaw;
    static float savedPress, savedTemp;

    // Dont reader faster than we can
    uint32_t now = xTaskGetTickCount();
    if ((now - lastConv) < CONVERSION_TIME_MS)
    {
      *pressure = savedPress;
      *temperature = savedTemp;
      return;
    }
    lastConv = now;

    if (readState == 0)
    {
        // read temp
        ++readState;
        tempTemperatureRaw = MS5611_GetConversion();
        tempDeltaT   = MS5611_CalcDeltaTemp(tempTemperatureRaw);
        *temperature = MS5611_CalcTemp(tempDeltaT);
        savedTemp = *temperature;
        *pressure = savedPress;
        // cmd to read pressure
        MS5611_StartConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
    }
    else
    {
        // read pressure
        ++readState;
        tempPressureRaw = MS5611_GetConversion();
        *pressure = MS5611_CalcPressure(tempPressureRaw, tempDeltaT);
        savedPress = *pressure;
        *asl = MS5611_PressureToAltitude(pressure);
        *temperature = savedTemp;
        if (readState == PRESSURE_PER_TEMP){
            // cmd to read temp
            MS5611_StartConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
            readState = 0;
        }
        else
        {
            // cmd to read pressure
            MS5611_StartConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
        }
    }
}

