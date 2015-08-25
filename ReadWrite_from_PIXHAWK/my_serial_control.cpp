#include "my_serial_control.h"

void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}

int
main(int argc, char **argv)
{
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;
	
	parse_commandline(argc, argv, uart_name, baudrate);
	
	Serial_Port my_serial(uart_name, baudrate); 
	LPmy_serial         = &my_serial;

	Autopilot_Interface autopilot_interface(LPmy_serial); 
	LPautopilot_interface = &autopilot_interface;

	LPmy_serial->start();
	LPautopilot_interface->start();

	while(1);

	LPautopilot_interface->stop();
	LPmy_serial->stop();
	
	return 0;
}


