#include "autopilot_interface.h"
#include <cmath>
#include <string.h>
#include "mavlink\include\mavlink\v1.0\common\mavlink.h"

using namespace std;

int main(int argc, char **argv);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
Serial_Port *LPmy_serial;
Autopilot_Interface *LPautopilot_interface;