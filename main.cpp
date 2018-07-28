#include <asm/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include "libini.hpp"
#include "joystick.h"

struct js_event 
{
	__u32 time;     /* event timestamp in milliseconds */
	__s16 value;    /* value */
	__u8 type;      /* event type */
	__u8 number;    /* axis/button number */
};

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		std::cout << "error from tcgetattr " << errno;
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		std::cout << "error from tcsetattr " << errno;
		return -1;
	}
	return 0;
}

void usage(void)
{
    printf(
        "rctx by RespawnDespair. Modfied from joystick_ppm_converter. GPL2\n"
        "\n"
        "Usage: rctx <config file> <mappings file> <trim file> <interfaces>\n"
        "\n"
        "Example:\n"
        "  rctx /boot/joyconfig.ini /boot/mappings.ini /boot/user_trims.ini wlan0\n"
        "\n");
    exit(1);
}

int main(int argc, char *argv[])
{
	if (argc < 4) {
		usage();
	}

	std::cout << "Loading configuration ..." << argv[1] << std::endl;

	std::ifstream t(argv[1]);
	std::string str((std::istreambuf_iterator<char>(t)),
                 std::istreambuf_iterator<char>());
	auto ini = libini::parse(str);

	std::string joystickPath;
	std::string outputSerialUsbPath;
	std::string outputSerialOrWbc;
	
	

	for(auto kvp: ini)
	{
		auto section_name = kvp.first;
		auto& section_values = kvp.second;

		if (section_name == "joystick")
		{
			// "/dev/input/js0"
			joystickPath = section_values["joystick_path"];
		}

		if (section_name == "output")
		{
			//"/dev/ttyUSB0"
			outputSerialUsbPath = section_values["output_serial_usb_path"];
			outputSerialOrWbc = section_values["output_serial_or_wbc"];
		}
	}

	std::cout << "Configuration:" << std::endl
				<< "joystickPath: " << joystickPath << std::endl
				<< "outputSerialUsbPath: " << outputSerialUsbPath << std::endl
				<< "outputSerialOrWbc: " << outputSerialOrWbc << std::endl
				<< std::endl;

	std::cout << "opening USB" << std::endl;
	int fdc = open (outputSerialUsbPath.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	set_interface_attribs (fdc, B115200, 0);

	ppm_target *ppm;

	if (outputSerialOrWbc == "wbc") {
		ppm_file ppm_file(fdc);
		ppm = &ppm_file;
	} else {
		ppm_stream ppm_stream(std::cout);
		ppm = &ppm_stream;
	}

	sleep(2);
	
	std::cout << "opening joystick" << std::endl;

	// argv2 is the mappings file
	// argv3 is the trim file
	joystick joy(argv[2], argv[3], *ppm);
	
	// open joystick
	int fd = open(joystickPath.c_str(), O_RDONLY);
	
	js_event e;
	
	// read from joystick
	while(read(fd, &e, sizeof(e)) > 0)
	{
		e.type = e.type &~JS_EVENT_INIT;
		
		if (e.type == JS_EVENT_AXIS)
		{
			joy.put_axis(e.number, (float)e.value / 32768.0f);
		}
		else if (e.type == JS_EVENT_BUTTON)
		{
			joy.put_button(e.number, e.value != 0);
		}
		else
		{
			std::cout << "unknown type " << (int)e.type;
		}
	}
	return 0;
}
