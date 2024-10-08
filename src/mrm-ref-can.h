#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-ref-can interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_REF_CAN0_IN 0x0160
#define CAN_ID_REF_CAN0_OUT 0x0161
#define CAN_ID_REF_CAN1_IN 0x0162
#define CAN_ID_REF_CAN1_OUT 0x0163
#define CAN_ID_REF_CAN2_IN 0x0164
#define CAN_ID_REF_CAN2_OUT 0x0165
#define CAN_ID_REF_CAN3_IN 0x0166
#define CAN_ID_REF_CAN3_OUT 0x0167
#define CAN_ID_REF_CAN4_IN 0x0168
#define CAN_ID_REF_CAN4_OUT 0x0169
#define CAN_ID_REF_CAN5_IN 0x016A
#define CAN_ID_REF_CAN5_OUT 0x016B
#define CAN_ID_REF_CAN6_IN 0x016C
#define CAN_ID_REF_CAN6_OUT 0x016D
#define CAN_ID_REF_CAN7_IN 0x016E
#define CAN_ID_REF_CAN7_OUT 0x016F

#define MRM_REF_CAN_SENSOR_COUNT 9 // Number of IR transistors in each device.

//CANBus commands
#define COMMAND_REF_CAN_MEASURE_ONCE_CENTER 0x04
#define COMMAND_REF_CAN_MEASURE_CONTINUOUS_CENTER 0x05
#define COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3 0x06
#define COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6 0x07
#define COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9 0x08
#define COMMAND_REF_CAN_CALIBRATE 0x09
#define COMMAND_REF_CAN_CALIBRATION_DATA_DARK_1_TO_3 0x0A
#define COMMAND_REF_CAN_CALIBRATION_DATA_DARK_4_TO_6 0x0B
#define COMMAND_REF_CAN_CALIBRATION_DATA_DARK_7_TO_9 0x0C
#define COMMAND_REF_CAN_CALIBRATION_DATA_REQUEST 0x0D
#define COMMAND_REF_CAN_SENDING_SENSORS_CENTER 0x0E
#define COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_1_TO_3 0x0F
#define COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_4_TO_6 0x50
#define COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_7_TO_9 0x51
#define COMMAND_REF_CAN_REPORT_ALIVE_QUEUELESS 0x53
#define COMMAND_REF_CAN_RECORD_PEAK 0x54
#define COMMAND_REF_CAN_REFRESH_MS 0x55

#define MRM_REF_CAN_INACTIVITY_ALLOWED_MS 10000

class Mrm_ref_can : public SensorBoard
{
	enum mode { ANALOG_VALUES, DIGITAL_AND_BRIGHT_CENTER, DIGITAL_AND_DARK_CENTER };

	std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>* calibrationDataDark; // 
	std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>* calibrationDataBright;
	std::vector<uint16_t>* centerOfMeasurements; // Center of the dark sensors.
	std::vector<uint8_t>* dataFresh; // All the data refreshed, bitwise stored. 
									// Most significant bit 0: readings for transistors 1 - 3, 
									// bit 1: 4 - 6, 
									// bit 2: 7 - 9, 
									// bit 3: calibration data for transistors 1 - 3, 
									// bit 4: 4 - 6, 
									// bit 5: 7 - 9
	std::vector<uint8_t>* _mode;
	bool readingDigitalAndCenter = true; // Reading only center and transistors as bits. Otherwise reading all transistors as analog values.
	std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>* _reading; // Analog or digital readings of all sensors, depending on measuring mode.
																// When digital, 0 is bright and 1 is dark
	std::vector<uint8_t>* _transistorCount;

	/** If analog mode not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool analogStarted(uint8_t deviceNumber);

	/** Calibration data fresh?
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - yes or no
	*/
	bool dataCalibrationFreshAsk(uint8_t deviceNumber) { return ((*dataFresh)[deviceNumber] & 0b00011100) == 0b00011100; }

	/** All data fresh?
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - yes or no
	*/
	bool dataFreshAsk(uint8_t deviceNumber) { return (*dataFresh)[deviceNumber] == 0xFF; }

	/** Set calibration data freshness
	@param setToFresh - set value to be fresh. Otherwise set to not to be.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	*/
	void dataFreshCalibrationSet(bool setToFresh, uint8_t deviceNumber = 0);

	/** Set readings data freshness
	@param setToFresh - set value
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	*/
	void dataFreshReadingsSet(bool setToFresh, uint8_t deviceNumber = 0);

	/** If digital mode with dark center not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param darkCenter - Center of dark. If not, center of bright.
	@param startIfNot - If not already started, start now.
	@return - started or not
	*/
	bool digitalStarted(uint8_t deviceNumber, bool darkCenter, bool startIfNot = true);
	
public:
	enum RecordPeakType {NO_PEAK, MAX_PEAK, MIN_PEAK} recordPeak = NO_PEAK;

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_ref_can(Robot* robot = NULL, uint8_t maxNumberOfBoards = 5);

	~Mrm_ref_can();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");

	/** Any dark or bright
	@param dark - any dark? Otherwise, any bright?
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param firstTransistor - start checking from this transistor
	@param lastTransistor - do not check after this one
	*/
	bool any(bool dark = true, uint8_t deviceNumber = 0, uint8_t fistTransistor = 0, uint8_t lastTransistor = 0xFF);

	/** Calibrate the array
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	*/
	void calibrate(uint8_t deviceNumber = 0xFF);

	/** Get local calibration data
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param isDark - if true calibration for dark, otherwise for bright
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t calibrationDataGet(uint8_t receiverNumberInSensor, bool isDark, uint8_t deviceNumber = 0);

	/** Print all calibration data in a line
	*/
	void calibrationPrint();

	/** Request sensor to send calibration data
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@param waitForResult - Blocks program flow till results return.
	*/
	void calibrationDataRequest(uint8_t deviceNumber = 0, bool waitForResult = false);

	/** Center of measurements, like center of the line
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@param ofDark - center of dark. Otherwise center of bright.
	@return - 1000 - 9000. 1000 means center exactly under first phototransistor (denoted with "1" on the printed circuit board), 5000 is center transistor.
	*/
	uint16_t center(uint8_t deviceNumber = 0, bool ofDark = true);

	/** Dark?
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param fromAnalog - from analog local values. If not, sensor-supplied center data.
	@return - yes or no.
	*/
	bool dark(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0, bool fromAnalog = false);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t dlc = 8);

	/** Sets recording of peaks between refreshes
	 * 
	*/
	void peakRecordingSet(RecordPeakType type, uint8_t deviceNumber = 0xFF);

	/** Enable plug and play
	@param enable - enable or disable
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void pnpSet(bool enable, uint8_t deviceNumber = 0xFF);
	
	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Sets refresh rate for sensor 
	 * 
	*/
	void refreshSet(uint16_t ms, uint8_t deviceNumber = 0xFF);

	/**Test
	@param analog - if true, analog values - if not, digital values.
	*/
	void test(bool analog);

	/**Transistor count
	@param count - transistor count
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void transistorCountSet(uint8_t count, uint8_t deviceNumber = 0){
		if (count <= 9)
			(*_transistorCount)[deviceNumber] = count;
	}

};



