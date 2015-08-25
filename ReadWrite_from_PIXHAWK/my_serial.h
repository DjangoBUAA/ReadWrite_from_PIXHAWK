#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------


#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <Windows.h>
#include <stdlib.h>


#include "mavlink\include\mavlink\v1.0\common\mavlink.h"


// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

#define SERIAL_PORT_OPEN   1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;

// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */
class Serial_Port
{

public:
	Serial_Port(char *&uart_name_, int &baudrate_);
	~Serial_Port();

	BOOL debug;
	int  status;
	const char *LPportname;
	int  baudrate;

	void open_serial();
	void close_serial();
	
	int read_byte_of_message(mavlink_message_t &message);  // Read the byte data
	int	write_message(mavlink_message_t &message);  // White the byte data
	
	void start();
	void stop();
	
	BYTE          Inbuffer;
	uint8_t          msgReceived;
	BYTE m_szWriteBuffer[300];
private:

	HANDLE  hComm;
	OVERLAPPED m_ov;
    COMSTAT comstat;
    DWORD	m_dwCommEvents;
	mavlink_status_t lastStatus;
	HANDLE hMutex;
    RTL_SRWLOCK lock;


	HANDLE  _open_port(const char* port);
	BOOL _setupdcb(int rate_arg);
	BOOL _setuptimeout(DWORD ReadInterval,DWORD ReadTotalMultiplier,DWORD ReadTotalconstant,DWORD WriteTotalMultiplier,DWORD WriteTotalconstant);
	BOOL _read_serial();
	
	
	void _write_serial(DWORD m_nToSend);
};



#endif // SERIAL_PORT_H_


