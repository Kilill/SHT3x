#include "SHT3x.h"
#include "I2Cdev.h"

//#define W_DEBUG
#ifdef W_DEBUG
#define DBUG(x) Serial.print(x)
#define DBUGLN(x) Serial.println(x)
#define DBUGA(x,n) Serial.print(x,n)
#define DBUGLNA(x,n) Serial.println(x,n)
#else
#define DBUG(x)
#define DBUGA(x)
#define DBUGLN(x)
#define DBUGLNA(x,n)

#endif

SHT3x::SHT3x(uint8_t addrArg, SHT3xSensor sensTArg,uint8_t resetPinArg, uint8_t intrPinArg, Commands modeArg, ValueIfError valArg)
{
	error = NoError;
	address = addrArg;
	errorValue = valArg;
	measureCommand = modeArg;
	sensorType = sensTArg;
	resetPin = resetPinArg;
	alertPin = intrPinArg;
}

void SHT3x::begin()
{
	/* not needed for I2Cdev
	 Wire.begin();
	 */
	operationEnabled = true;
}

void SHT3x::updateData()
{
	float tempRaw;
	float rhRaw;
	uint8_t data[10];

	error = NoError;
	/* TODO: hmm what about continous ....
	 *
	 */
	for(int i=0;i<5;i++) data[i]=0;
	if ((lastUpdate == 0) || ((millis() - lastUpdate) >= updateInterval)) { // are we back to soon ?
		I2Cdev::readCommand16(address, static_cast<uint16_t>(measureCommand), 6, data, timeout);

		if ((CRC8(data[0], data[1], data[2])) && (CRC8(data[3], data[4], data[5]))){
			tempRaw = (((uint16_t)data[0] << 8) | (data[1]));
			rhRaw = (((uint16_t) data[3] << 8) | (data[4]));

			// C=175*tempRaw/65535-45 F=315*tempRaw/65535=49 Doc: 4.12

			// Compute celcius first and convert to other scales;

			temperatureCel = 175.0 * tempRaw / 65355.0 - 45.0;
			temperatureCel = temperatureCel * temperatureCalibration.Factor + temperatureCalibration.Shift;
			temperatureFar = temperatureCel*(9.0/5.0)+32.0;
			temperatureKel = temperatureKel +273.15;

			relHumidity = 100 * rhRaw  / 65535;
			relHumidity = relHumidity * relHumidityCalibration.Factor + relHumidityCalibration.Shift;

			error = NoError;

		} else {
			Serial.println("SHT3x: CRC FAILED");
			switch (errorValue) {
				case NanValue:
						temperatureCel=temperatureFar=temperatureKel=relHumidity=NAN;
				break;
				case ZeroValue:
						temperatureCel=temperatureFar=temperatureKel=relHumidity=NAN;
				break;
				default:
					/* do nothing previous value stays */
				break;
			}
			error = DataCorrupted;
		}
	}
}

// taken from http://irtfweb.ifa.hawaii.edu/~tcs3/tcs3/Misc/Dewpoint_Calculation_Humidity_Sensor_E.pdf
float SHT3x::getDewPoint(){
	float tmp     = (log10(relHumidity)-2)/0.4343 + (17.62*temperatureCel)/(243.12+temperatureCel);
	return 243.12*tmp/(17.62-tmp);
}

#define MOLAR_MASS_OF_WATER     18.01534
#define UNIVERSAL_GAS_CONSTANT  8.21447215

float SHT3x::getAbsHumidity()
{
  //taken from https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
  //precision is about 0.1°C in range -30 to 35°C
  //August-Roche-Magnus   6.1094 exp(17.625 x T)/(T + 243.04)
  //Buck (1981)     6.1121 exp(17.502 x T)/(T + 240.97)
  //reference https://www.eas.ualberta.ca/jdwilson/EAS372_13/Vomel_CIRES_satvpformulae.html    // Use Buck (1981)
  return (6.1121 * pow(2.718281828, (17.67 * temperatureCel) / (temperatureCel + 243.5)) * relHumidity * MOLAR_MASS_OF_WATER) / ((273.15 + temperatureCel) * UNIVERSAL_GAS_CONSTANT);
}

float SHT3x::getHumidityPressure(HumidityPressureScale Scale)
{
	float millikelvins = temperatureKel / 1000.;
	float Pressure = 0.;

	if (error != NoError && errorValue != PrevValue) return (errorValue == NanValue) ? NAN : 0.0;

	for (uint8_t i = 0; i < 6; i++) {
		float term = 1.;
		for (uint8_t j = 0; j < i; j++) {
			term *= millikelvins;
		}
		Pressure += term * absHumPoly[i];
	}
	Pressure *= relHumidity;
	switch (Scale) {
		case mmHg:
		case Torr:
			break; //Already in Torr
		case Pa:   Pressure *= 133.322; break;
		case Bar:  Pressure *= 0.0013332; break;
		case At:   Pressure *= 0.0013595; break;
		case Atm:  Pressure *= 0.0013158; break;
		case mH2O: Pressure *= 0.013595; break;
		case psi:  Pressure *= 0.019337; break;
		default: break;
	}
	return Pressure;
}

float SHT3x::getTempTolerance(TemperatureScale degree, SHT3xSensor sensorType)
{
	//Temperature tolerance is similar for both SHT30 and SHT31
	//At first, calculate at Celsius (similar to Kelvins), than, if need, recalculate to Farenheit
	float tolerance;
	if(error && (errorValue != PrevValue)) return (errorValue == NanValue) ? NAN : 0.0;
	switch (sensorType) {
		case SHT30:
			if (temperatureCel > 65.) 		tolerance =  0.0067 * temperatureCel - 0.2333;	//Linear from 0.2 at 65 C to 0.6 at 125 C.
			else if (temperatureCel >= 0.0) tolerance =  0.2;
			else 							tolerance = -0.01  * temperatureCel + 0.2;		//Linear from 0.6 at -40 C to 0.2 at 0 C.
			break;

		case SHT31:
			if (temperatureCel > 65.)	tolerance =  0.0086  * temperatureCel - 0.5714;	//Linear from 0.2 at 90 C to 0.5 at 125 C.
			if (temperatureCel >= 0.0)	tolerance =  0.2;
			else 						tolerance = -0.0025 * temperatureCel + 0.2;		//Linear from 0.3 at -40 C to 0.2 at 0 C.
			break;

		case SHT35:
			if (temperatureCel <= 0.0) 		tolerance = 0.2;
			else if (temperatureCel <= 20.)	tolerance = -0.005  * temperatureCel + 0.2; //Linear from 0.2 at 0 C to 0.1 at 20 C.
			else if (temperatureCel <= 60.)	tolerance =  0.1;
			else if (temperatureCel <= 90.)	tolerance = -0.0033 * temperatureCel - 0.1; //Linear from 0.1 at 60 C to 0.2 at 90 C.
			else /* Temperature > 90.0 */ 	tolerance =  0.0057 * temperatureCel - 0.3143; //Linear from 0.2 at 90 C to 0.4 at 125 C.
			break;
	}
	if (degree == Far) tolerance *= 1.8;
	return tolerance;
}

float 	SHT3x::getRelHumTolerance(SHT3xSensor SensorType)
{
	float RelHumidity = relHumidity;
	float Tolerance;
	if(error && (errorValue != PrevValue)) return (errorValue == NanValue) ? NAN : 0.0;
	switch (SensorType)
	{
		case SHT30:
			if (relHumidity>90.0)	Tolerance =  0.2 * RelHumidity - 16.; //Linear from 2 at 90% to 4 at 100%
			if (RelHumidity>10.0)	Tolerance =  2.;
			else 					Tolerance = -0.2 * RelHumidity + 4.; //Linear from 4 at 0% to 2 at 10%
			break;
		case SHT31:
			Tolerance = 2.;
			break;
		case SHT35:
			if (RelHumidity <= 80.) Tolerance = 1.5;
			else					Tolerance = 0.025 * RelHumidity - 0.5; //Linear from 0.5 at 80% to 2 at 100%
			break;	
	}
	return Tolerance;

}

float 	SHT3x::getAbsHumTolerance(HumidityPressureScale Scale, SHT3xSensor SensorType)
{
	/*	Dependence of absolute humidity is similar (from 0 to 80C) to P = H*a*exp(b*T),
	*	where P is absolute humidity, H is relative humidity, T is temperature (Celsius),
	*	a ~= 0.0396, b~=0.0575. 
	*	So its relative tolerance dP/P =  square root  [ (dH/H)^2 + (b*dT)^2 ].
	*/
	
	float humTol;
	float tempTol;

	if(error && (errorValue != PrevValue)) return (errorValue == NanValue) ? NAN : 0.0;

	humTol = getRelHumTolerance(SensorType)/getRelHumidity();
	tempTol = 0.0575*getTempTolerance(Cel, SensorType);

	return	 getHumidityPressure(Scale) * sqrt((humTol * humTol) + (tempTol * tempTol));
}

void	SHT3x::setTemperatureCalibrationPoints(CalibrationPoints SensorValues, CalibrationPoints Reference)
{
	temperatureCalibration.Factor = (Reference.Second - Reference.First) / (SensorValues.Second - SensorValues.First);
	temperatureCalibration.Shift = Reference.First - temperatureCalibration.Factor * SensorValues.First;
}

void	SHT3x::setRelHumidityCalibrationPoints(CalibrationPoints SensorValues, CalibrationPoints Reference)
{
	relHumidityCalibration.Factor = (Reference.Second - Reference.First) / (SensorValues.Second - SensorValues.First);
	relHumidityCalibration.Shift = Reference.First - relHumidityCalibration.Factor * SensorValues.First;
}



void	SHT3x::hardReset()
{
	if (resetPin<100) //MEGA2560 have only 68 pins
	{
		pinMode(resetPin,OUTPUT);
		digitalWrite(resetPin,LOW);
		delayMicroseconds(1); //Reset pin have to be LOW at least 350ns, so 1 usec is enough. I think, there is no need for micros() for 1 usec.
		digitalWrite(resetPin,HIGH);
	}
}



void	SHT3x::setAddress(uint8_t Address)
{
	if ((Address == 0x44) || (Address == 0x45))
	{
		address = Address;
	}
	else
	{
		error = WrongAddress;
	}
}

void	SHT3x::setUpdateInterval(uint32_t updateIntervalArg)
{
	if (updateIntervalArg > 0)
	{
		updateInterval = updateIntervalArg;
	}
}

void	SHT3x::setTimeout(uint32_t timeoutArg)
{
	if (timeoutArg > 0)
	{
		timeout= timeoutArg;
	}
}

void 	SHT3x::I2CError(uint8_t I2Canswer)
{
	switch (I2Canswer) {
		case 0: error = NoError; 		break;
		case 1: error = TooMuchData;	break;
		case 2: error = AddressNACK;	break;
		case 3: error = DataNACK; 		break;
		case 4: error = OtherI2CError;	break;
		default: error = UnexpectedError; break;
	}
}
void 	SHT3x::sendCommand(Commands command)
{
#ifdef HAS_I2Cdev
	I2Cdev::writeWordNR(adress,command);
#else
	if (! operationEnabled) {
		Wire.begin();
		operationEnabled = true;
	}

	Wire.beginTransmission(address);
	// Send Soft Reset command
	Wire.write(command >>8);
	Wire.write(command &0x0f);
	// Stop I2C transmission
	uint8_t success = Wire.endTransmission();
#endif
}
// Generator polynomial for CRC
// P(x) = x^8 + x^5 + x^4 + 1 = 100110001
#define POLYNOMIAL  0x131

 uint8_t SHT3x::calcCrc(uint8_t data[], uint8_t nbrOfBytes)
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  }
  return crc;
}

//-----------------------------------------------------------------------------
bool SHT3x::checkCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  uint8_t crc;     // calculated checksum
  char buffer[30];

  // calculates 8-Bit checksum
  crc = calcCrc(data, nbrOfBytes);


  // verify checksum
  if(crc != checksum) return false;
  else                return true;
}

bool SHT3x::CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC)
{
	/*
	 *	Name  : CRC-8
	 *	Poly  : 0x31	x^8 + x^5 + x^4 + 1
	 *	Init  : 0xFF
	 *	Revert: false
	 *	XorOut: 0x00
	 *	Check : for 0xBE,0xEF CRC is 0x92
	 */
	uint8_t crc = 0xFF;
	uint8_t i;
	char buffer[30];

	crc ^= MSB;
	for (i = 0; i < 8; i++)
		crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

	crc ^= LSB;
	for (i = 0; i < 8; i++)
		crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

	sprintf(buffer,"crc8: %0x - %0x\n",crc,CRC);
	if (crc == CRC)
		return true;
	 else
		return false;

}
