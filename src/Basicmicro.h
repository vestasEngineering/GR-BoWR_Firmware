/**
 * @file Basicmicro.h
 * @brief Interface library for Basicmicro motor controllers
 *
 * This library provides communication and control functions for Basicmicro
 * motor controllers. It supports various control modes including duty cycle,
 * speed, and position control, along with encoder feedback and PID parameter settings.
 */

#ifndef Basicmicro_h
#define Basicmicro_h

#include <stdarg.h>

#include <inttypes.h>
#include <Stream.h>
#include <HardwareSerial.h>
#ifdef __AVR__
	#include <SoftwareSerial.h>
#endif

/******************************************************************************
* Definitions
******************************************************************************/

#define _BM_VERSION 10 // software version of this library
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#define _SS_VERSION 17 // Note: This seems to be a mismatch with _BM_VERSION, potentially intended for SoftwareSerial version? Keeping as is for now.

/**
 * @brief Main class for interfacing with Basicmicro motor controllers
 *
 * This class provides methods to control motors, read encoders, set PID parameters,
 * monitor battery voltage, and configure various settings on Basicmicro motor controllers.
 */
class Basicmicro : public Stream
{
	uint16_t crc;
	uint32_t timeout;

	HardwareSerial *hserial;
#ifdef __AVR__
	SoftwareSerial *sserial;
#endif

	enum {
			//Commands deprecated. Use stub functions in arduino library
			M1FORWARD = 0,
			M1BACKWARD = 1,
			M2FORWARD = 4,
			M2BACKWARD = 5,
			M17BIT = 6,
			M27BIT = 7,
			MIXEDFORWARD = 8,
			MIXEDBACKWARD = 9,
			MIXEDRIGHT = 10,
			MIXEDLEFT = 11,
			MIXEDFB = 12,
			MIXEDLR = 13,
			//End of deprecated commands

			SETTIMEOUT = 14,
			GETTIMEOUT = 15,
			GETM1ENC = 16,
			GETM2ENC = 17,
			GETM1SPEED = 18,
			GETM2SPEED = 19,
			RESETENC = 20,
			GETVERSION = 21,
			SETM1ENCCOUNT = 22,
			SETM2ENCCOUNT = 23,
			GETMBATT = 24,
			GETLBATT = 25,

			SETM1PID = 28,
			SETM2PID = 29,
			GETM1ISPEED = 30,
			GETM2ISPEED = 31,
			M1DUTY = 32,
			M2DUTY = 33,
			MIXEDDUTY = 34,
			M1SPEED = 35,
			M2SPEED = 36,
			MIXEDSPEED = 37,
			M1SPEEDACCEL = 38,
			M2SPEEDACCEL = 39,
			MIXEDSPEEDACCEL = 40,
			M1SPEEDDIST = 41,
			M2SPEEDDIST = 42,
			MIXEDSPEEDDIST = 43,
			M1SPEEDACCELDIST = 44,
			M2SPEEDACCELDIST = 45,
			MIXEDSPEEDACCELDIST = 46,
			GETBUFFERS = 47,
			GETPWMS = 48,
			GETCURRENTS = 49,
			MIXEDSPEED2ACCEL = 50,
			MIXEDSPEED2ACCELDIST = 51,
			M1DUTYACCEL = 52,
			M2DUTYACCEL = 53,
			MIXEDDUTYACCEL = 54,
			READM1PID = 55,
			READM2PID = 56,
			SETMAINVOLTAGES = 57,
			SETLOGICVOLTAGES = 58,
			GETMINMAXMAINVOLTAGES = 59,
			GETMINMAXLOGICVOLTAGES = 60,
			SETM1POSPID = 61,
			SETM2POSPID = 62,
			READM1POSPID = 63,
			READM2POSPID = 64,
			M1SPEEDACCELDECCELPOS = 65,
			M2SPEEDACCELDECCELPOS = 66,
			MIXEDSPEEDACCELDECCELPOS = 67,
			SETM1DEFAULTACCEL = 68,
			SETM2DEFAULTACCEL = 69,
			SETM1DEFAULTSPEED = 70,
			SETM2DEFAULTSPEED = 71,
			GETDEFAULTSPEEDS = 72,
			GETSTATUS = 73,
			SETPINFUNCTIONS = 74,
			GETPINFUNCTIONS = 75,
			SETCTRLSETTINGS	= 76,
			GETCTRLSETTINGS	= 77,
			GETENCODERS = 78,
			GETISPEEDS = 79,
			RESTOREDEFAULTS = 80,
			GETDEFAULTACCELS = 81,
			GETTEMP = 82,
			GETTEMP2 = 83,	//Only valid on some models

			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94,
			READNVM = 95,	//Reloads values from Flash into Ram
			SETSERIALNUMBER = 96,
			GETSERIALNUMBER = 97,
			SETCONFIG = 98,
			GETCONFIG = 99,
			GETVOLTS = 100,
			GETTEMPS = 101,
			SETAUXDUTYS = 102,
			GETENCSTATUS = 103,
			GETAUXDUTYS = 104,
			SETAUTO1 = 105,
			SETAUTO2 = 106,
			GETAUTOS = 107,
			GETSPEEDS = 108,
			SETSPEEDERRORLIMIT = 109,
			GETSPEEDERRORLIMIT = 110,
			GETSPEEDERRORS = 111,
			SETPOSERRORLIMIT = 112,
			GETPOSERRORLIMIT = 113,
			GETPOSERRORS = 114,
			SETOFFSETS = 115,	//MCP only
			GETOFFSETS = 116,	//MCP only

			M1POS = 119,
			M2POS = 120,
			MIXEDPOS = 121,
			M1SPEEDPOS = 122,
			M2SPEEDPOS = 123,
			MIXEDSPEEDPOS = 124,
			M1PPOS = 125,
			M2PPOS = 126,
			MIXEDPPOS = 127,

			SETM1LR = 128, //MCP only
			SETM2LR = 129, //MCP only
			GETM1LR = 130, //MCP only
			GETM2LR = 131, //MCP only

			SETM1MAXCURRENT = 133,
			SETM2MAXCURRENT = 134,
			GETM1MAXCURRENT = 135,
			GETM2MAXCURRENT = 136,

			SETDOUT = 137, 				//MCP only
			GETDOUTS = 138, 			//MCP only
			SETPRIORITY = 139, 			//MCP only
			GETPRIORITY = 140, 			//MCP only
			SETADDRESSMIXED = 141, 		//MCP only
			GETADDRESSMIXED = 142,		//MCP only
			SETSIGNAL = 143, 			//MCP only
			GETSIGNALS = 144, 			//MCP only
			SETSTREAM = 145, 			//MCP only
			GETSTREAMS = 146, 			//MCP only
			GETSIGNALSDATA = 147,		//MCP only

			SETPWMMODE = 148,
			GETPWMMODE = 149,

			SETNODEID = 150,	//MCP only
			GETNODEID = 151,	//MCP only

			SETPWMIDLE = 160,
			GETPWMIDLE = 161,

			CANBUFFERSTATE = 180,	//MCP only
			CANPUTPACKET = 181,		//MCP only
			CANGETPACKET = 182,		//MCP only

			CANOPENWRITEDICT = 190,	//MCP only
			CANOPENREADDICT = 191,		//MCP only

			RESETESTOP = 200,	//MCP only
			SETESTOPLOCK = 201,	//MCP only
			GETESTOPLOCK = 202,	//MCP only

			SETSCRIPTAUTORUN = 246,	//MCP only
			GETSCRIPTAUTORUN = 247,	//MCP only
			STARTSCRIPT = 248,		//MCP only
			STOPSCRIPT = 249,		//MCP only

			READEEPROM = 252,
			WRITEEEPROM = 253,
	};	//Only available via USB communications

public:
	// Error and warning code definitions with explanations
	enum {
		ERROR_NONE			= 0x00000000,
		ERROR_ESTOP			= 0x00000001, //Error: E-Stop active
		ERROR_TEMP			= 0x00000002, //Error: Temperature Sensor 1 >=100c
		ERROR_TEMP2			= 0x00000004, //Error: Temperature Sensor 2 >=100C (available only on some models)

		ERROR_LBATHIGH		= 0x00000010, //Error: Logic Battery High Voltage
		ERROR_LBATLOW		= 0x00000020, //Error: Logic Battery Low Voltage

		ERROR_SPEED1		= 0x00000100, //Error: Motor 1 Speed Error Limit
		ERROR_SPEED2		= 0x00000200, //Error: Motor 2 Speed Error Limit
		ERROR_POS1			= 0x00000400, //Error: Motor 1 Position Error Limit
		ERROR_POS2			= 0x00000800, //Error: MOtor2 Position Error Limit
		ERROR_CURRENTM1		= 0x00001000, //Error: Motor 1 Current Limited
		ERROR_CURRENTM2		= 0x00002000, //Error: Motor 2 Current Limited

		WARN_OVERCURRENTM1	= 0x00010000, //Warning: Motor 1 Current Limited
		WARN_OVERCURRENTM2	= 0x00020000, //Warning: Motor 2 CUrrent Limited
		WARN_MBATHIGH		= 0x00040000, //Warning: Main Battery Voltage High
		WARN_MBATLOW		= 0x00080000, //Warning: Main Battery Low Voltage
		WARN_TEMP			= 0x00100000, //Warning: Temperaure Sensor 1 >=85C
		WARN_TEMP2			= 0x00200000, //Warning: Temperature Sensor 2 >=85C (available only on some models)
		WARN_S4				= 0x00400000, //Warning: Motor 1 Home/Limit Signal
		WARN_S5				= 0x00800000, //Warning: Motor 2 Home/Limit Signal

		WARN_BOOT			= 0x20000000, //Warning: Set on power up. Cleared once unit is completely booted
		WARN_OVERREGENM1	= 0x40000000, //Warning: Motor 1 Regen Current Limited
		WARN_OVERREGENM2	= 0x80000000, //Warning: Motor 2 Regen Current Limited
	};

	struct SignalConfig {
		uint8_t type;
		uint8_t mode;
		uint8_t target;
		uint16_t minAction;
		uint16_t maxAction;
		uint8_t lowpass;
		uint32_t timeout;
		int32_t loadhome;
		int32_t minVal;
		int32_t maxVal;
		int32_t center;
		uint32_t deadband;
		uint32_t powerexp;
		uint32_t minout;
		uint32_t maxout;
		uint32_t powermin;
		uint32_t potentiometer;
	};

	struct StreamConfig {
		uint8_t type;
		uint32_t baudrate;
		uint32_t timeout;
	};

	struct SignalData {
		uint32_t command;
		uint32_t position;
		uint32_t percent;
		uint32_t speed;
		uint32_t speeds;
	};

    /**
     * @brief Constructor using hardware serial
     *
     * @param hserial Pointer to a HardwareSerial instance
     * @param tout Communication timeout in microseconds
     */
	Basicmicro(HardwareSerial *hserial,uint32_t tout);

    /**
     * @brief Constructor using software serial (AVR platforms only)
     *
     * @param sserial Pointer to a SoftwareSerial instance
     * @param tout Communication timeout in microseconds
     */
#ifdef __AVR__
	Basicmicro(SoftwareSerial *sserial,uint32_t tout);
#endif

    /**
     * @brief Destructor
     */
	~Basicmicro();

	bool ForwardM1(uint8_t address, uint8_t speed);
	bool BackwardM1(uint8_t address, uint8_t speed);
	bool ForwardM2(uint8_t address, uint8_t speed);
	bool BackwardM2(uint8_t address, uint8_t speed);
	bool ForwardBackwardM1(uint8_t address, uint8_t speed);
	bool ForwardBackwardM2(uint8_t address, uint8_t speed);
	bool ForwardMixed(uint8_t address, uint8_t speed);
	bool BackwardMixed(uint8_t address, uint8_t speed);
	bool TurnRightMixed(uint8_t address, uint8_t speed);
	bool TurnLeftMixed(uint8_t address, uint8_t speed);
	bool ForwardBackwardMixed(uint8_t address, uint8_t speed);
	bool LeftRightMixed(uint8_t address, uint8_t speed);
	uint32_t ReadEncM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
	uint32_t ReadEncM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
	bool SetEncM1(uint8_t address, int32_t val);
	bool SetEncM2(uint8_t address, int32_t val);
	uint32_t ReadSpeedM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
	uint32_t ReadSpeedM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
	bool ResetEncoders(uint8_t address);
	bool ReadVersion(uint8_t address,char *version);
	uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL);
	uint16_t ReadLogicBatteryVoltage(uint8_t address,bool *valid=NULL);
	bool SetM1VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
	bool SetM2VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
	uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
	uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
	bool DutyM1(uint8_t address, uint16_t duty);
	bool DutyM2(uint8_t address, uint16_t duty);
	bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
	bool SpeedM1(uint8_t address, uint32_t speed);
	bool SpeedM2(uint8_t address, uint32_t speed);
	bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);
	bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
	bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
	bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);
	bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
	bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
	bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);
	bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2);
	bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &urrent2);
	bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);
	bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
	bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);
	bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);
	bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
	bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);
	bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);
	bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max,uint8_t autoMax);
	bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max);
	bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max,uint8_t& autoMax);
	bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max);
	bool SetM1PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
	bool SetM2PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
	bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
	bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
	bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
	bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
	bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);
	bool SetM1DefaultAccel(uint8_t address, uint32_t accel, uint32_t decel);
	bool SetM2DefaultAccel(uint8_t address, uint32_t accel, uint32_t decel);
	bool GetDefaultAccels(uint8_t address, uint32_t &accelM1, uint32_t &decelM1, uint32_t &accelM2, uint32_t &decelM2);
	bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode, uint8_t D1mode, uint8_t D2mode);
	bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode, uint8_t &D1mode, uint8_t &D2mode);
	bool SetCtrlSettings(uint8_t address, uint8_t minDBM1, uint8_t maxDBM1, uint16_t minLimitsM1, uint16_t maxLimitsM1, uint16_t centerM1, uint16_t minM1, uint16_t maxM1,
										  uint8_t minDBM2, uint8_t maxDBM2, uint16_t minLimitsM2, uint16_t maxLimitsM2, uint16_t centerM2, uint16_t minM2, uint16_t maxM2);
	bool GetCtrlSettings(uint8_t address, uint8_t &minDBM1, uint8_t &maxDBM1, uint16_t &minLimitsM1, uint16_t &maxLimitsM1, uint16_t &centerM1, uint16_t &minM1, uint16_t &maxM1,
										  uint8_t &minDBM2, uint8_t &maxDBM2,uint16_t &minLimitsM2, uint16_t &maxLimitsM2, uint16_t &centerM2,  uint16_t &minM2, uint16_t &maxM2);
	bool ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2);
	bool ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2);
	bool RestoreDefaults(uint8_t address);
	bool ReadTemp(uint8_t address, uint16_t &temp);
	bool ReadTemp2(uint8_t address, uint16_t &temp);
	uint32_t ReadError(uint8_t address,bool *valid=NULL);
	bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);
	bool SetM1EncoderMode(uint8_t address,uint8_t mode);
	bool SetM2EncoderMode(uint8_t address,uint8_t mode);
	bool WriteNVM(uint8_t address);
	bool ReadNVM(uint8_t address);
	bool SetConfig(uint8_t address, uint16_t config);
	bool GetConfig(uint8_t address, uint16_t &config);
	bool SetM1MaxCurrent(uint8_t address,uint32_t max,uint32_t min);
	bool SetM2MaxCurrent(uint8_t address,uint32_t max,uint32_t min);
	bool ReadM1MaxCurrent(uint8_t address,uint32_t &max,uint32_t &min);
	bool ReadM2MaxCurrent(uint8_t address,uint32_t &max,uint32_t &min);
	bool SetPWMMode(uint8_t address, uint8_t modeM1 ,uint8_t modeM2);
	bool GetPWMMode(uint8_t address, uint8_t &modeM1,uint8_t &modeM2);
	bool SetAUXDutys(uint8_t address, uint16_t S3duty, uint16_t S4duty, uint16_t S5duty, uint16_t CTRL1duty, uint16_t CTRL2duty);
	bool GetAUXDutys(uint8_t address, uint16_t &S3duty, uint16_t &S4duty, uint16_t &S5duty, uint16_t &CTRL1duty, uint16_t &CTRL2duty);

	bool SetTimeout(uint8_t address, float timeout);
	bool GetTimeout(uint8_t address, float &timeout);

	bool SetM1DefaultSpeed(uint8_t address, uint16_t speed);
	bool SetM2DefaultSpeed(uint8_t address, uint16_t speed);
	bool GetDefaultSpeeds(uint8_t address, uint16_t &speed1, uint16_t &speed2);

	bool GetStatus(uint8_t address, uint32_t &tick, uint32_t &state, uint16_t &temp1, uint16_t &temp2,
				  uint16_t &mainBattVoltage, uint16_t &logicBattVoltage,
				  int16_t &pwm1, int16_t &pwm2, int16_t &current1, int16_t &current2,
				  uint32_t &enc1, uint32_t &enc2, uint32_t &speed1, uint32_t &speed2,
				  uint32_t &ispeed1, uint32_t &ispeed2, uint16_t &speedError1, uint16_t &speedError2,
				  uint16_t &posError1, uint16_t &posError2);

	bool SetSerialNumber(uint8_t address, const char* serialNumber);
	bool GetSerialNumber(uint8_t address, char* serialNumber);

	bool GetVolts(uint8_t address, uint16_t &mainBattVoltage, uint16_t &logicBattVoltage);
	bool GetTemps(uint8_t address, uint16_t &temp1, uint16_t &temp2);
	bool GetEncStatus(uint8_t address, uint8_t &enc1Status, uint8_t &enc2Status);

	bool SetAuto1(uint8_t address, uint32_t value);
	bool SetAuto2(uint8_t address, uint32_t value);
	bool GetAutos(uint8_t address, uint32_t &auto1, uint32_t &auto2);

	bool GetSpeeds(uint8_t address, uint32_t &speed1, uint32_t &speed2);
	bool SetSpeedErrorLimit(uint8_t address, uint16_t limit1, uint16_t limit2);
	bool GetSpeedErrorLimit(uint8_t address, uint16_t &limit1, uint16_t &limit2);
	bool GetSpeedErrors(uint8_t address, uint16_t &error1, uint16_t &error2);
	bool SetPosErrorLimit(uint8_t address, uint16_t limit1, uint16_t limit2);
	bool GetPosErrorLimit(uint8_t address, uint16_t &limit1, uint16_t &limit2);
	bool GetPosErrors(uint8_t address, uint16_t &error1, uint16_t &error2);
	bool SetOffsets(uint8_t address, uint8_t offset1, uint8_t offset2);
	bool GetOffsets(uint8_t address, uint8_t &offset1, uint8_t &offset2);

	bool M1Position(uint8_t address, uint32_t position, uint8_t buffer);
	bool M2Position(uint8_t address, uint32_t position, uint8_t buffer);
	bool MixedPosition(uint8_t address, uint32_t position1, uint32_t position2, uint8_t buffer);
	bool M1SpeedPosition(uint8_t address, uint32_t speed, uint32_t position, uint8_t buffer);
	bool M2SpeedPosition(uint8_t address, uint32_t speed, uint32_t position, uint8_t buffer);
	bool MixedSpeedPosition(uint8_t address, uint32_t speed1, uint32_t position1, uint32_t speed2, uint32_t position2, uint8_t buffer);
	bool M1PercentPosition(uint8_t address, int16_t position, uint8_t buffer);
	bool M2PercentPosition(uint8_t address, int16_t position, uint8_t buffer);
	bool MixedPercentPosition(uint8_t address, int16_t position1, int16_t position2, uint8_t buffer);

	bool SetM1LR(uint8_t address, float L, float R);
	bool SetM2LR(uint8_t address, float L, float R);
	bool GetM1LR(uint8_t address, float &L, float &R);
	bool GetM2LR(uint8_t address, float &L, float &R);

	bool SetDOUT(uint8_t address, uint8_t index, uint8_t action);
	bool GetDOUTS(uint8_t address, uint8_t &count, uint8_t *actions, uint8_t maxActions);
	bool SetPriority(uint8_t address, uint8_t priority1, uint8_t priority2, uint8_t priority3);
	bool GetPriority(uint8_t address, uint8_t &priority1, uint8_t &priority2, uint8_t &priority3);
	bool SetAddressMixed(uint8_t address, uint8_t newAddress, uint8_t enableMixing);
	bool GetAddressMixed(uint8_t address, uint8_t &newAddress, uint8_t &mixingEnabled);
	bool SetSignal(uint8_t address, uint8_t index, uint8_t signalType, uint8_t mode,
				  uint8_t target, uint16_t minAction, uint16_t maxAction, uint8_t lowpass,
				  uint32_t timeout, int32_t loadhome, int32_t minVal, int32_t maxVal,
				  int32_t center, uint32_t deadband, uint32_t powerexp, uint32_t minout,
				  uint32_t maxout, uint32_t powermin, uint32_t potentiometer);
	bool GetSignals(uint8_t address, uint8_t &count, SignalConfig *signals, uint8_t maxSignals);
	bool SetStream(uint8_t address, uint8_t index, uint8_t streamType, uint32_t baudrate, uint32_t timeout);
	bool GetStreams(uint8_t address, uint8_t &count, StreamConfig *streams, uint8_t maxStreams);
	bool GetSignalsData(uint8_t address, uint8_t &count, SignalData *signalsData, uint8_t maxSignals);

	bool SetNodeID(uint8_t address, uint8_t nodeID);
	bool GetNodeID(uint8_t address, uint8_t &nodeID);
	bool SetPWMIdle(uint8_t address, float idleDelay1, bool idleMode1, float idleDelay2, bool idleMode2);
	bool GetPWMIdle(uint8_t address, float &idleDelay1, bool &idleMode1, float &idleDelay2, bool &idleMode2);

	bool CANBufferState(uint8_t address, uint8_t &count);
	bool CANPutPacket(uint8_t address, uint16_t cobID, uint8_t rtr, uint8_t length, const uint8_t *data);
	bool CANGetPacket(uint8_t address, uint16_t &cobID, uint8_t &rtr, uint8_t &length, uint8_t *data);

	bool CANOpenWriteDict(uint8_t address, uint8_t nodeID, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size, uint32_t &result);
	bool CANOpenReadDict(uint8_t address, uint8_t nodeID, uint16_t index, uint8_t subindex, uint32_t &value, uint8_t &size, uint8_t &type, uint32_t &result);

	bool ResetEStop(uint8_t address);
	bool SetEStopLock(uint8_t address, uint8_t lockState);
	bool GetEStopLock(uint8_t address, uint8_t &lockState);
	bool SetScriptAutorun(uint8_t address, uint32_t scriptAutorunTime);
	bool GetScriptAutorun(uint8_t address, uint32_t &scriptAutorunTime);
	bool StartScript(uint8_t address);
	bool StopScript(uint8_t address);

	bool ReadEEPROM(uint8_t address, uint8_t eeAddress, uint16_t &value);
	bool WriteEEPROM(uint8_t address, uint8_t eeAddress, uint16_t value);

	static int16_t library_version() { return _SS_VERSION; }

	virtual int available();
	void begin(long speed);
	bool isListening();
	bool overflow();
	int peek();
	virtual int read();
	int read(uint32_t timeout); // Original read with timeout
	bool listen();
	virtual size_t write(uint8_t byte);
	virtual void flush();
	void clear();

private:
	int16_t ST_Power,ST_Turn;

	void crc_clear();
	void crc_update (uint8_t data);
	uint16_t crc_get();

	// New helper function to read a byte with timeout and update CRC
	int16_t _read(void);

	// New helper function to read the expected CRC and verify it
	bool _checkcrc(void);

	bool write_n(uint8_t byte,...);

    bool read_n(uint8_t cnt, uint8_t address, uint8_t cmd, ...);
    bool read_n_words(uint8_t cnt, uint8_t address, uint8_t cmd, ...);
    bool read_n_bytes(uint8_t cnt, uint8_t address, uint8_t cmd, ...);

	uint32_t Read4_1(uint8_t address,uint8_t cmd,uint8_t *status,bool *valid);
	uint32_t Read4(uint8_t address,uint8_t cmd,bool *valid);
	uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid);
	uint8_t Read1(uint8_t address,uint8_t cmd,bool *valid);

	void write_address_cmd(uint8_t address,uint8_t cmd);

	void _write(uint8_t val);
    void _writeword(uint16_t val);
    void _writelong(uint32_t val);
    bool _writechecksum();

    // Modified read functions using the new helper
    bool ReadByte(uint8_t &value);
    bool ReadWord(uint16_t &value);
    bool ReadLong(uint32_t &value);

	bool SetPWM(uint8_t address, uint8_t motor, int16_t value, int32_t accel, uint16_t range);
	bool ST_Single(uint8_t cmd,uint8_t address, uint8_t speed);
	bool SetPWM2(uint8_t address, int16_t value1, int32_t accel1, int16_t value2, int32_t accel2, uint16_t range);
	bool ST_Mixed(uint8_t cmd, uint8_t address, uint8_t speed);
};

#endif