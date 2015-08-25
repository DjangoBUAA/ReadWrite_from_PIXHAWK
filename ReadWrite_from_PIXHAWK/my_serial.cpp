// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "my_serial.h"



Serial_Port::Serial_Port(char *&uart_name_, int &baudrate_)
{
	// Initialize attributes
	debug  = false;
	hComm     = NULL;
	status = SERIAL_PORT_CLOSED;

	LPportname = uart_name_;
	baudrate  = baudrate_;

	InitializeSRWLock(&lock);  // Init SRWLock
}

Serial_Port::~Serial_Port()
{
}

int Serial_Port::read_byte_of_message(mavlink_message_t &message)
{

	mavlink_status_t status;

	AcquireSRWLockShared(&lock);  // when read data we need to lock the serial

	BOOL result = _read_serial();  

	ReleaseSRWLockShared(&lock);   // unlock
	
	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
	// --------------------------------------------------------------------------
	if (result)
	{
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, Inbuffer, &message, &status);

		// check for dropped packets
		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v=Inbuffer;
			fprintf(stderr,"%02x ", v);
		}
		lastStatus = status;
	}

	// Couldn't read from port
	else
	{
		fprintf(stderr, "ERROR: Could not read from %s\n");
	}
// --------------------------------------------------------------------------
	//   DEBUGGING REPORTS
	// --------------------------------------------------------------------------
	if(msgReceived && debug)
	{
		// Report info
		printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		fprintf(stderr,"Received serial data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		if (messageLength > MAVLINK_MAX_PACKET_LEN)
		{
			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		}
// print out the buffer
		else
		{
			for (i=0; i<messageLength; i++)
			{
				unsigned char v=buffer[i];
				fprintf(stderr,"%02x ", v);
			}
			fprintf(stderr,"\n");
		}
	}

	// Done!
	return msgReceived;
}

// ------------------------------------------------------------------------------
//     read a byte from serial and save it in cp
// ------------------------------------------------------------------------------
BOOL Serial_Port::_read_serial()
{
    BOOL  bResult = TRUE;
    DWORD dwError = 0;
    DWORD BytesRead = 0;

	
for (;;)
    {  // before use ReadFile to read data,we should use ClearCommError to clear error£¬then return the serial status
 	   bResult = ClearCommError(hComm, &dwError, &comstat);

       if (comstat.cbInQue == 0)  // serial status is saved in COMSTAT
          continue;
	   else
	   {
           bResult = ReadFile(hComm,     // portname:the handle of serial
                              &Inbuffer,          // data are saved in Inbuffer   		      
	                          1,	       // read 1 byte
	                          &BytesRead,  // return the real number of byte 
		                      &m_ov);      // whne overlapped operation, m_ov points to a OVERLAPPED struct

              if (!bResult)   // when ReadFile returns FALSE£¬the read thread should use GetLastError to analyze the result
                 {
                    switch (dwError = GetLastError())
                       {
                          case ERROR_IO_PENDING:
                            {
                               bResult = GetOverlappedResult(hComm,	   // Handle to COMM port
                                           &m_ov,	   // Overlapped structure
                                           &BytesRead, // Stores number of bytes read
                                           TRUE); 	   // Wait flag
                               break;
                             }
                          default:
                                 break;  
                        }
                  }
        }
	   break;
   }
 

return bResult;
}

int	Serial_Port::write_message(mavlink_message_t &message)
{
	m_szWriteBuffer[300] = 0;

	// send the message packet to the serial output buffer
	DWORD m_nToSend = mavlink_msg_to_send_buffer((uint8_t*)m_szWriteBuffer, &message);

	
	AcquireSRWLockExclusive(&lock); // write data to serial and lock it
   
	_write_serial(m_nToSend);     // write data in buffer to serial, locks port while writing
	
	ReleaseSRWLockExclusive(&lock); // unlock

	return m_nToSend;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------

void Serial_Port::_write_serial(DWORD m_nToSend)
{
    BOOL bWrite = TRUE;
    BOOL bResult = TRUE;
    DWORD BytesSent = 0;
    HANDLE	m_hWriteEvent = NULL;
    ResetEvent(m_hWriteEvent);

if (bWrite)
   {
       m_ov.Offset = 0;
       m_ov.OffsetHigh = 0;
       bResult = WriteFile(hComm,	
	                       m_szWriteBuffer,	 
	                       m_nToSend,	
	                       &BytesSent,	 
	                       &m_ov );	
       
	   if (!bResult)       
          {
             DWORD dwError = GetLastError();
             switch (dwError)
                {
                   case ERROR_IO_PENDING: //if return ERROR_IO_PENDING,overlapped operation has not finished               
                      {
                          bResult = GetOverlappedResult(hComm,&m_ov,&BytesSent,TRUE); 
                          break;
                       }
                   default:
                          break;
                  } 
            }

   } 

if (BytesSent != m_nToSend) 
   printf("WARNING: WriteFile() error.. Bytes Sent: %d; Message Length: %d\n", BytesSent, strlen((char*)m_szWriteBuffer));

return;
}


void Serial_Port::open_serial()
{
    // --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
	printf("OPEN SERIAL PORT\n");

	hComm = _open_port(LPportname);

	if (hComm == INVALID_HANDLE_VALUE)
	{
		CloseHandle(hComm);
	    printf("Can not open serial port %s.\n",LPportname);
	}
	else
	{
		SetupComm(hComm,10000,10000);
		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); // before read or write serial£¬we should use PurgeComm() to clean the buffer
                  //PURGE_TXABORT	  break off all write operation and return immediately,even if the write operation has not finished
                  //PURGE_RXABORT	  break off all read operation and return immediately,even if the read operation has not finished
                  //PURGE_TXCLEAR	  clean the output buffer
                  //PURGE_RXCLEAR	  clean the input buffer
		memset(&m_ov,0,sizeof(OVERLAPPED));
		printf("Open comport success\n");
	}

	BOOL dcbSuccess = _setupdcb(baudrate);
	if (!dcbSuccess)
       printf("Could not configure serial port.\n");
	else
       printf("configure serial port success.\n");

	BOOL timeoutSuccess = _setuptimeout(0,0,0,0,0);
	if (!timeoutSuccess) 
       printf("Could not configure setuptimeout.\n");
	else
       printf("Configure setuptimeout success.\n");

	if (dcbSuccess && timeoutSuccess)
	{
	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", LPportname, baudrate);

	system("pause"); 
	lastStatus.packet_rx_drop_count = 0;
	status = 1;
	printf("\n");
	}
	return;
}


// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
HANDLE Serial_Port::_open_port(const char* LPportname)
{
   hComm = CreateFile(LPportname, // commport
                      GENERIC_READ | GENERIC_WRITE,
                      0,
                      NULL,
                      OPEN_EXISTING,
                      FILE_FLAG_OVERLAPPED, 
                      NULL);

   
   SetCommMask(hComm, EV_RXCHAR); 

return hComm;
}

void Serial_Port::close_serial()
{
	printf("CLOSE SERIAL PORT\n");

	// Enable all events in serial port FALSE 
    SetCommMask(hComm, 0) ;

    EscapeCommFunction( hComm, CLRDTR ) ;
 
    PurgeComm( hComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ) ;
    CloseHandle( hComm );
    hComm = NULL;
	
	status = false;

	printf("\n");
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup DCB struct 
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
BOOL Serial_Port::_setupdcb(int rate_arg)
{
   DCB  dcb;
   int rate= rate_arg;
   memset(&dcb,0,sizeof(dcb));
if (!GetCommState(hComm,&dcb))// get current DCB configuration
   {
      return FALSE;
    }
 /* -------------------------------------------------------------------- */
 // set DCB to configure the serial port
        dcb.DCBlength       = sizeof(dcb);
 /* -------------- Serial Port Config -------------- */
        dcb.BaudRate        = rate;
        dcb.Parity      = NOPARITY;
        dcb.fParity     = 0;
        dcb.StopBits        = ONESTOPBIT;
        dcb.ByteSize        = 8;
        dcb.fOutxCtsFlow    = 0;
        dcb.fOutxDsrFlow    = 0;
        dcb.fDtrControl     = DTR_CONTROL_DISABLE;
        dcb.fDsrSensitivity = 0;
        dcb.fRtsControl     = RTS_CONTROL_DISABLE;
        dcb.fOutX           = 0;
        dcb.fInX            = 0;
/* ----------------- misc parameters --------------- */
        dcb.fErrorChar      = 0;
        dcb.fBinary         = 1;
        dcb.fNull           = 0;
        dcb.fAbortOnError   = 0;
        dcb.wReserved       = 0;
        dcb.XonLim          = 2;
        dcb.XoffLim         = 4;
        dcb.XonChar         = 0x13;
        dcb.XoffChar        = 0x19;
        dcb.EvtChar         = 0;
/* -------------------------------------------------- */
        // set DCB
if (!SetCommState(hComm,&dcb))
   {
       return false;
    }
else
       return true;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup timeout
// ------------------------------------------------------------------------------
BOOL Serial_Port::_setuptimeout(DWORD ReadInterval,DWORD ReadTotalMultiplier,DWORD ReadTotalconstant,DWORD WriteTotalMultiplier,DWORD WriteTotalconstant)
{
   COMMTIMEOUTS timeouts;
   timeouts.ReadIntervalTimeout=ReadInterval; //Read Interval Timeout
   timeouts.ReadTotalTimeoutConstant=ReadTotalconstant; // read time coefficient
   timeouts.ReadTotalTimeoutMultiplier=ReadTotalMultiplier; // read time constant
   timeouts.WriteTotalTimeoutConstant=WriteTotalconstant; // write time coefficient
   timeouts.WriteTotalTimeoutMultiplier=WriteTotalMultiplier; //write time constant
if (!SetCommTimeouts(hComm, &timeouts))
   {
      return false;
    }
else
      return true;

}


void Serial_Port::start()
{
	open_serial();
}

void Serial_Port::stop()
{
	close_serial();
}