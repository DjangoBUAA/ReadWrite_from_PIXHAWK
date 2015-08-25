#include "autopilot_interface.h"



// Get the system time(ms)
uint64_t get_time_msec()
{
	SYSTEMTIME sys_time;
	GetSystemTime(&sys_time); // system time 
	return sys_time.wMilliseconds;
}




Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	 write_count = 0;

	reading_status = false;      // whether the read thread is running
	writing_status = false;      // whether the write thread is running
	control_status = false;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	num_of_message = 0;

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	Pmy_serial = serial_port_; // serial port management object
}

Autopilot_Interface::
~Autopilot_Interface()
{}



void
Autopilot_Interface::
read_messages()
{
	BOOL success;               
	BOOL received_all = false; 
	Time_Stamps this_timestamps;

	// read the data in buffer
	while ( !received_all )
	{
		// define a message struct
		mavlink_message_t message;
		success = Pmy_serial->read_byte_of_message(message);

		if( success )
		{

			// save the system id
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;
			//printf("The message ID is %d\n",message.msgid);
			

			// process different messages 
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT: // heartbeat message
				{
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_msec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:  // attitude message
				{
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					printf("%lu,%f\n",current_messages.attitude.time_boot_ms,current_messages.attitude.roll);
					current_messages.time_stamps.attitude = get_time_msec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}
				case MAVLINK_MSG_ID_RAW_IMU:  // raw imu kessage
				{
					mavlink_msg_raw_imu_decode(&message, &(current_messages.imu_raw));
					
					printf("%llu,%d\n",current_messages.imu_raw.time_usec,current_messages.imu_raw.xacc);
					current_messages.time_stamps.imu_raw = get_time_msec();
					this_timestamps.imu_raw = current_messages.time_stamps.imu_raw;
					break;
				}


				default:
				{
					//printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} 

		} 
 

	} 

	return;
}



int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = Pmy_serial->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}



void
Autopilot_Interface::
start()
{

	// check the status of serial
	if ( ! Pmy_serial->status == 1 ) 
	{
		fprintf(stderr,"ERROR: serial port not open\n");
	}

	// start the read data thread
	printf("START READ THREAD \n");

	hThread1 = CreateThread(NULL, 0, start_autopilot_interface_read_thread, this, 0, read_tid);  // read data thread
	if (! hThread1 ) printf("START READ THREAD FAILURE\n");


	// check that whether we have got the system id and component id
	printf("CHECK FOR MESSAGES\n");

	while ( ! current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		Sleep(500); 
	}

	printf("Found\n");

    // Print the system id and component id
	if ( ! system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	if ( ! autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}
	
	
	// Request the data we want from PIXHAWK
	printf("Request all data to autopilot\n");
    RequestAllDataStreams();
	printf("Receive data.....\n");


	
	/*
	while ( 1 )
	{
		if ( time_to_exit )
			return;
	}
	*/
	return;

}


void Autopilot_Interface::RequestAllDataStreams()
{   
	
	//enableExtendedSystemStatusTransmission(10);
    //enablePositionTransmission(10);  // 33
    enableExtra1Transmission(100);// 30

    //enableExtra3Transmission(10); // 29
    enableRawSensorDataTransmission(100);
    //enableRCChannelDataTransmission(10);//36 35
	
}


void Autopilot_Interface::enableExtendedSystemStatusTransmission(int rate)
{
    // Buffers to write data to
    mavlink_message_t msg;
    mavlink_request_data_stream_t stream;
    // Select the message to request from now on
    stream.req_stream_id = MAV_DATA_STREAM_EXTENDED_STATUS;
    // Select the update rate in Hz the message should be send
    stream.req_message_rate = rate;
    // Start / stop the message
    stream.start_stop = (rate) ? 1 : 0;
    // The system which should take this command
    stream.target_system = system_id;
    // The component / subsystem which should take this command
    stream.target_component = 0;
    // Encode and send the message
    mavlink_msg_request_data_stream_encode(system_id, autopilot_id, &msg, &stream);
    // Send message twice to increase chance of reception
    write_message(msg);
}


void Autopilot_Interface::enablePositionTransmission(int rate)
{
	// Buffers to write data to
    mavlink_message_t msg;
    mavlink_request_data_stream_t stream;
    // Select the message to request from now on
    stream.req_stream_id = MAV_DATA_STREAM_POSITION;
    // Select the update rate in Hz the message should be send
    stream.req_message_rate = rate;
    // Start / stop the message
    stream.start_stop = (rate) ? 1 : 0;
    // The system which should take this command
    stream.target_system = system_id;
    // The component / subsystem which should take this command
    stream.target_component = 0;
    // Encode and send the message
    mavlink_msg_request_data_stream_encode(system_id, autopilot_id, &msg, &stream);
    // Send message twice to increase chance of reception
    write_message(msg);
}

void Autopilot_Interface::enableExtra1Transmission(int rate)
{
	 // Buffers to write data to
    mavlink_message_t msg;
    mavlink_request_data_stream_t stream;
    // Select the message to request from now on
    stream.req_stream_id = MAV_DATA_STREAM_EXTRA1;
    // Select the update rate in Hz the message should be send
    stream.req_message_rate = rate;
    // Start / stop the message
    stream.start_stop = (rate) ? 1 : 0;
    // The system which should take this command
    stream.target_system = system_id;
    // The component / subsystem which should take this command
    stream.target_component = 0;
    // Encode and send the message
    mavlink_msg_request_data_stream_encode(system_id, autopilot_id, &msg, &stream);
    // Send message twice to increase chance of reception
    write_message(msg);
    write_message(msg);
}



void Autopilot_Interface::enableExtra3Transmission(int rate)
{
	// Buffers to write data to
    mavlink_message_t msg;
    mavlink_request_data_stream_t stream;
    // Select the message to request from now on
    stream.req_stream_id = MAV_DATA_STREAM_EXTRA3;
    // Select the update rate in Hz the message should be send
    stream.req_message_rate = rate;
    // Start / stop the message
    stream.start_stop = (rate) ? 1 : 0;
    // The system which should take this command
    stream.target_system = system_id;
    // The component / subsystem which should take this command
    stream.target_component = 0;
    // Encode and send the message
    mavlink_msg_request_data_stream_encode(system_id, autopilot_id, &msg, &stream);
    // Send message twice to increase chance of reception
    write_message(msg);
    write_message(msg);
}


void Autopilot_Interface::enableRawSensorDataTransmission(int rate)
{
    // Buffers to write data to
    mavlink_message_t msg;
    mavlink_request_data_stream_t stream;
    // Select the message to request from now on
    stream.req_stream_id = MAV_DATA_STREAM_RAW_SENSORS;
    // Select the update rate in Hz the message should be send
    stream.req_message_rate = rate;
    // Start / stop the message
    stream.start_stop = (rate) ? 1 : 0;
    // The system which should take this command
    stream.target_system = system_id;
    // The component / subsystem which should take this command
    stream.target_component = 0;
    // Encode and send the message
    mavlink_msg_request_data_stream_encode(system_id, autopilot_id, &msg, &stream);
    // Send message twice to increase chance of reception
    write_message(msg);
}



void Autopilot_Interface::enableRCChannelDataTransmission(int rate)
{
#if defined(MAVLINK_ENABLED_UALBERTA_MESSAGES)
    mavlink_message_t msg;
    mavlink_msg_request_rc_channels_pack(systemId, componentId, &msg, enabled);
    sendMessage(msg);
#else
    mavlink_message_t msg;
    mavlink_request_data_stream_t stream;
    // Select the message to request from now on
    stream.req_stream_id = MAV_DATA_STREAM_RC_CHANNELS;
    // Select the update rate in Hz the message should be send
    stream.req_message_rate = rate;
    // Start / stop the message
    stream.start_stop = (rate) ? 1 : 0;
    // The system which should take this command
    stream.target_system = system_id;
    // The component / subsystem which should take this command
    stream.target_component = 0;
    // Encode and send the message
    mavlink_msg_request_data_stream_encode(system_id, autopilot_id, &msg, &stream);
    // Send message twice to increase chance of reception
    write_message(msg);
#endif
}


void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	WaitForSingleObject(hThread1,100);
	WaitForSingleObject(hThread2,100);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}



void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


void
Autopilot_Interface::
start_write_thread(void)
{
	if ( writing_status )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		Sleep(100); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = true;

	

	// write a message and signal writing

	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( ! time_to_exit )
	{
		Sleep(250);   // Stream at 4Hz

	}

	// signal end
	writing_status = false;

	return;

}


DWORD WINAPI start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return 0;
}

DWORD WINAPI start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return 0;
}