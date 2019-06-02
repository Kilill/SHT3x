/*
	Arduino library for Sensirion temperature and humidity sensors SHT30, SHT31 & SHT35.
	the heavy version.
	Check for /examples for examples of different use cases.
	
	The datasheet I followed is:
	https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital.pdf
	Sensirion_Humidity_Sensors_SHT3x_Application_Note_Alert_Mode_DIS.pdf
	For more simple version check the SimpleSHT3x library.
	
	Supports:
		Temperature data at Celsius, Kelvin and Fahrenheit scales.
		Relative humidity data.
		Absolute humidity data at Torr, mm Hg, Pa, bar, technical and standard atmosphere, psi scales.
		Data integrity (by CRC8 algorithm) (datasheet/section 4.12).
		Temperature, relative and absolute humidity tolerances (in dependence from measured values)
		Calibration (linear) of temperature and humidity data by factors or by reverse sensor values (2 points)
		Heater On/Off (integrated to SHT3x sensor) (datasheet/section 4.10)
		Different sensor actions modes (datasheet/section 4.3)
		Reset: soft (I2C) and hard (by corresponding pin) (datasheet/section 4.9)
		
	Do not supports:
		Action in periodic mode (datasheet/section 4.5)
		Interrupts (datasheet/section 3.5)
		
		
	Note 1: by default, the data from sensor updates not faster, than 2 times a second.
	For faster update use SetUpdateInterval(uint32_t UpdateIntervalMillisec); but do not exceed the datasheet values (10 measurments per second (100 ms)) because of sensor self-heating (datasheet/section 4.5, at the end of Table 9)
	
	Note 2: The sensor type affects the tolerance values only. 
	
	Created by Risele for everyone's use (profit and non-profit).
	Modifed by Kim Lilliestierna (2019)

	ALL THESE WOR_DS
	ARE YOURS EXCEPT
	RISELE
	ATTEMPT NO
	namechangING THERE
	USE THEM TOGETHER
	USE THEM IN PEACE
	
*/


#ifndef SHT3x_h
#define SHT3x_h
	

	//Arduino standart libraries
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
	#else
	#include "WProgram.h"
#endif
/*
	//Arduino I2C/TWI library
	#include <Wire.h>
*/
#include "I2Cdev.h"
	//For calculating the tolerance of absolute humidity
#include <math.h>
	
	
class SHT3x
{
	public:

	enum ValueIfError {  ///< Error return value, Define, what to return in case of error: Zero or previous value
		ZeroValue,
		PrevValue,
		NanValue
	};

	enum Commands:uint16_t {
		///<  Single shot Meassuring mode (Doc 4.3)
		///< Single Shot , Clock stretch commands
		Single_HighRep_ClockStretch		= 0x2C06, ///< Single Shot High Repeatability with Clock stretch
		Single_MediumRep_ClockStretch	= 0x2C0D, ///< Single Shot Medium Repeatability with Clock stretch
		Single_LowRep_ClockStretch		= 0x2C10, ///< Single Shot Low Repeatability with Clock stretch

		///< Single Shot , No Clock stretch commands
		Single_HighRep_NoClockStretch	= 0x2406,
		Single_MediumRep_NoClockStretch	= 0x240D,
		Single_LowRep_NoClockStretch	= 0x2410,

		///< Periodic  Meassuring mode (Doc 4.5)

		///< Periodic 1/2 measurement per sec
		Repeat_HighRep_05				= 0x2032,
		Repeat_MedRep_05				= 0x2024,
		Repeat_LowRep_05				= 0x202F,

		///< Periodic 1 measurement per sec
		Repeat_HighRep_1				= 0x2130,
		Repeat_MedRep_1					= 0x2126,
		Repeat_LowRep_1					= 0x212D,

		///< Periodic 2 measurement per sec
		Repeat_HighRep_2				= 0x2236,
		Repeat_MedRep_2					= 0x2220,
		Repeat_LowRep_2					= 0x222B,

		///< Periodic 4 measurement per sec
		Repeat_HighRep_4				= 0x2334,
		Repeat_MedRep_4					= 0x2322,
		Repeat_LowRep_4					= 0x2329,

		///< Periodic 10 measurement per sec
		Repeat_HighRep_10				= 0x2737,
		Repeat_MedRep_10				= 0x2721,
		Repeat_LowRep_14				= 0x272A,

		///< Read periodic data (Doc 4.6)
		FetchData						= 0xE000,

		 ///< Peridic ART Doc 4.7
		PeriodicArt						= 0x2B32,

		///< Break Doc 4.8
		Break							= 0x3093,

		///< Reset Doc 4.9
		SoftReset						= 0x30A2,
		GeneralCallReset				= 0x0006,

		///< Heater Doc 4.10
		HeaterEnable					= 0x306D,
		HeaterDisable					= 0x3066,

		// Status  Doc 4.11
		ReadStatus						= 0XF32D,
		ClrStatus						= 0X3041,

		// Utility
		ReadSerialNumber				= 0X3780,
		NoSleep							= 0x303E,

		// Alert
		// see : Sensirion_Humidity_Sensors_SHT3x_Application_Note_Alert_Mode_DIS.pdf
		ReadAlamrLimitsLowSet			= 0xE102,
		ReadAlamrLimitsLowClear			= 0xE109,
		ReadAlamrLimitsHighSet			= 0xE11F,
		ReadAlamrLimitsHighClear		= 0xE114,

		WriteAlarmLimitsHigSet			= 0x611D,
		WriteAlarmLimitsHigClear		= 0x6116,
		WriteAlarmLimitsLowClear		= 0x610B,
		WriteAlarmLimitsLowSet			= 0x6100,
	};

	/** Status register (Doc 4.11)
	 *
	 */
	enum StatusBits {		  //111111
							  //5432109876543210
			AlertPending	= 0b1000000000000000, ///< 0: No alert pending 1: Alert Pending
			Heater			= 0b0010000000000000, ///< 0:Heater off 1:Heater On
			HAlert			= 0b0000100000000000, ///< RelHumidity Tracking Alert 0: No alert 1:Alert
			TAlert			= 0b0000010000000000, ///< Temp Tracking Alert 0: No alert 1: Alert
			SystemReset		= 0b0000000000010000, ///< 0: No reset since last clr status command
			ComandStatus	= 0b0000000000000010, ///< 0: Last command successful 1: Not proccessed
			CheckSumValid	= 0b0000000000000001, ///< 0: last write crc correct 1: crc failed
	};
	///< Sensor type
	enum SHT3xSensor {
		SHT30,
		SHT31,
		SHT35
	};

	///< Temperature scale
	enum TemperatureScale {
		Cel,
		Far,
		Kel
	};

	///< pressure scales
	enum HumidityPressureScale {
		mmHg,
		Torr, 	//same as mm Hg
		Pa,
		Bar,
		At,	 	//Techical atmosphere
		Atm,	//Standart atmosphere
		mH2O,
		psi,
	};

	enum Errors {
			NoError 		= 0,
			Timeout 		= 1,
			DataCorrupted 	= 2,
			WrongAddress	= 3,

			//I2C errors
			TooMuchData 	= 4,
			AddressNACK 	= 5,
			DataNACK 		= 6,
			OtherI2CError 	= 7,
			UnexpectedError = 8
		};

	struct CalibrationPoints {
		float First;
		float Second;
	};

	struct CalibrationFactors {
		CalibrationFactors():Factor(1.), Shift(0.){}
		float Factor;
		float Shift;
	};

	private:

		static constexpr float absHumPoly[6] ={-157.004, 3158.0474, -25482.532, 103180.197, -209805.497, 171539.883};
		uint8_t 	address;
		uint8_t 	resetPin;
		uint8_t		alertPin;
		SHT3xSensor sensorType;
		ValueIfError errorValue;

		float 		temperatureCel;
		float		temperatureFar;
		float		temperatureKel;
		float 		relHumidity;
		bool 		operationEnabled 	= false;

		uint32_t 	updateInterval = 500; ///< Minimum update intervall in millis
		uint32_t 	lastUpdate = 0;			///< Last update time stamp in millis
		uint32_t 	timeout = 100;

		Commands	measureCommand;

		CalibrationFactors relHumidityCalibration;
		CalibrationFactors temperatureCalibration;

		/* alarm interupt routine
		void *isr(void);
		*/

		void sendCommand(Commands command);

		uint8_t calcCrc(uint8_t data[], uint8_t nbrOfBytes);
		bool 	checkCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);

		bool CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC);

		void toReturnIfError(ValueIfError Value);

		/*
		* 	Factors for poly for calculating absolute humidity (in Torr):
		*	P = (RelativeHumidity /100%) * sum(_AbsHumPoly[i]*T^i)
		*	where:
		*	P is absolute humidity (Torr/mm Hg),
		*	T is Temperature(Kelvins degree) / 1000,
		* 	^ means power.
		*	For more data, check the NIST chemistry webbook:
		*	http://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Units=SI&Mask=4&Type=ANTOINE&Plot=on#ANTOINE
		*/


		Errors error;

		void I2CError(uint8_t I2Canswer);

public:
		
	// --- Constructor
	SHT3x(	uint8_t Address 		= 0x45,
		SHT3xSensor SensorType 		= SHT30,
		uint8_t 	resetPin	 	= 255,
		uint8_t 	alarmPin	 	= 255,
		Commands	MeasureCommand	= Single_MediumRep_ClockStretch,
		ValueIfError errorValue 	= NanValue);

	void begin();
	void updateData();

		
	inline float getTempCel() { return temperatureCel;}
	inline float getTempKel(){ return temperatureKel;}
	inline float getTempFar(){ return temperatureFar;}
	inline float getRelHumidity(){return relHumidity;}

	float getDewPoint();

	float getAbsHumidity();
	float getHumidityPressure(HumidityPressureScale Scale = Torr);

	float getTempTolerance(TemperatureScale Degree = Cel,SHT3xSensor sensor=SHT30);
	float getRelHumTolerance(SHT3xSensor sensor=SHT30);
	float getAbsHumTolerance(HumidityPressureScale Scale = Torr,SHT3xSensor sensor=SHT30);
		
	inline void setMode(Commands Mode = Single_HighRep_ClockStretch) { measureCommand=Mode; }
	inline void	setTemperatureCalibrationFactors(CalibrationFactors factor) { temperatureCalibration = factor; }
	inline void setRelHumidityCalibrationFactors(CalibrationFactors factor) { relHumidityCalibration = factor; }

	void setTemperatureCalibrationPoints(CalibrationPoints sensorValues, CalibrationPoints reference);
	void setRelHumidityCalibrationPoints(CalibrationPoints sensorValues, CalibrationPoints reference);

	inline uint8_t getError(){return error;}


	inline void softReset() { I2Cdev::writeWordNR(address,SoftReset); }
	void hardReset();
	inline void	heaterOn() { I2Cdev::writeWordNR(address,HeaterEnable); }
	inline void	heaterOff() { I2Cdev::writeWordNR(address,HeaterDisable); }

	void setAddress(uint8_t NewAddress);
	void setUpdateInterval(uint32_t UpdateIntervalMillisec);
	void setTimeout(uint32_t TimeoutMillisec);
	/* TODO:
	void enableIsr(void *isr(void));
	 * Alert commands
	 * getSingleAlert(bool,higOrLow,bool setOrClr,float *temp,float *humidity);
	 * 	getAlertHighSet(float *temp, float *humidity);
	 * 	getAlertHighClr(float *temp, float *humidity);
	 * 	getAlertLowSet(float *temp, float *humidity);
	 * 	getAlertLowClr(float *temp, float *humidity);
	 * 	getAllAlert(float *highTem,float *lowTemp, float *highHum, float *lowHum);
	 *
	 * setSingle(bool,higOrLow,bool setOrClr,float temp, float humidity);
	 * 	setAlertHighSet(float temp, float humidity);
	 * 	setAlertHighClr(float temp, float humidity);
	 * 	setAlertLowSet(float temp, float humidity);
	 * 	setAlertLowClr(float temp, float humidity);
	 * 	setAllAlert(float *highTem,float lowtemp, float highHum, float Lowhum);
	 *
	 */
		


		
};
#endif //SHT3x_h
