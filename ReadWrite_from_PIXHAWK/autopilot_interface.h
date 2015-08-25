
#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_


#include "my_serial.h"
#include <time.h>
#include <iostream>
#include "mavlink\include\mavlink\v1.0\common\mavlink.h"



DWORD WINAPI start_autopilot_interface_read_thread(void *args);
DWORD WINAPI start_autopilot_interface_write_thread(void *args);



struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t attitude;
	//uint64_t global_position_int;
	//uint64_t nav_controller_output;
	//uint64_t servo_output_raw;
	uint64_t imu_raw;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		attitude = 0;
		//global_position_int = 0;
		//nav_controller_output = 0;
		//servo_output_raw = 0;
		imu_raw = 0;
	}

};


struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// Attitude
	mavlink_attitude_t attitude;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Nav_Controller_Output
    mavlink_nav_controller_output_t nav_controller_output;

	// Servo_Outpuy_Raw
	mavlink_servo_output_raw_t servo_output_raw; 

	// IMU_RAW
	mavlink_raw_imu_t imu_raw;
	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_);
	~Autopilot_Interface();

	BOOL reading_status;
	BOOL writing_status;
	BOOL control_status;
    uint64_t write_count;

    int system_id;
	int autopilot_id;
	int companion_id;

	int num_of_message;

	Mavlink_Messages current_messages;  

	void read_messages(); // Read the messages
	int  write_message(mavlink_message_t message); // White the messages you want to the PIXHAWK


	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void RequestAllDataStreams();  // Send the request of data_streams to PIXHAWK

	void enableExtendedSystemStatusTransmission(int rate);

    void enablePositionTransmission(int rate);

    void enableExtra1Transmission(int rate);

    void enableExtra3Transmission(int rate);

    void enableRawSensorDataTransmission(int rate);

    void enableRCChannelDataTransmission(int rate);
	

private:

	Serial_Port *Pmy_serial;
	BOOL time_to_exit;


	LPDWORD read_tid;
	LPDWORD write_tid;
	FLOAT a_of_c;
	HANDLE hThread1,hThread2;

	void read_thread();
	void write_thread(void);
};



#endif // AUTOPILOT_INTERFACE_H_


