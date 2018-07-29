#include "ppm_stream.h"
#include <sstream>
#include <unistd.h>
#include <iostream>
#include <cstring>

#include "lib.h"				// EZWB Specific 
#include <sys/mman.h>			// shm_open
#include <sys/stat.h>			// shm_open
#include <fcntl.h>				// O_ constants

#include <arpa/inet.h>			// inet
#include <sys/socket.h>			// socket
#include <netpacket/packet.h> 	// socketaddr_ll
#include <net/if.h> 			// ifreq
#include <netinet/ether.h>
#include <sys/ioctl.h>			// SIOCGIFHWADDR

/******************************************************************************
 * Arduino serial related entries
 *****************************************************************************/ 
ppm_file::ppm_file(int fd)
	: m_fd(fd)
{}

void ppm_file::setChannelValue(unsigned char channel, float value)
{
	//std::cout << "we are here" << std::flush;
	std::stringstream ss;

	// convert value to PPM us
	// value is -1 ... 1 -> 1000 .. 2000
	value = value * 500.0f + 1500.0f;
	// define some upper and lower limit
	if (value > 2100.0f)
		value = 2100.0f;
	if (value < 900.0f)
		value = 900.0f;
	
	// output that value as int
	ss << "ppm:" << (int)channel << "," << (int)value << "\n" << std::flush;

	// write to fd
	write(this->m_fd, ss.str().data(), ss.str().size());
}

/******************************************************************************
 * WBC related entries
 *****************************************************************************/ 

// structs
struct framedata_s {
    // 171 bits of data (11 bits per channel * 16 channels) = 22 bytes
    uint8_t rt1;
    uint8_t rt2;
    uint8_t rt3;
    uint8_t rt4;
    uint8_t rt5;
    uint8_t rt6;
    uint8_t rt7;
    uint8_t rt8;

    uint8_t rt9;
    uint8_t rt10;
    uint8_t rt11;
    uint8_t rt12;

    uint8_t fc1;
    uint8_t fc2;
    uint8_t dur1;
    uint8_t dur2;

    uint8_t seqnumber;

    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    unsigned int chan16 : 11;
} __attribute__ ((__packed__));

// Variables
struct framedata_s framedata;
static uint16_t rcData[16];
int sock = 0;
int socks[5];

// WBC Methods
static int open_sock (char *ifname) {
	std::cout << "Open_socket on interface: " << ifname << std::endl;
	
    struct sockaddr_ll ll_addr;
    struct ifreq ifr;

    sock = socket (AF_PACKET, SOCK_RAW, 0);
    if (sock == -1) {
		fprintf(stderr, "Error:\tSocket failed\n");
		exit(1);
    }

    ll_addr.sll_family = AF_PACKET;
    ll_addr.sll_protocol = 0;
    ll_addr.sll_halen = ETH_ALEN;

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
		fprintf(stderr, "Error:\tioctl(SIOCGIFINDEX) failed\n");
		exit(1);
    }

    ll_addr.sll_ifindex = ifr.ifr_ifindex;

    if (ioctl(sock, SIOCGIFHWADDR, &ifr) < 0) {
		fprintf(stderr, "Error:\tioctl(SIOCGIFHWADDR) failed\n");
		exit(1);
    }

    memcpy(ll_addr.sll_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);

    if (bind (sock, (struct sockaddr *)&ll_addr, sizeof(ll_addr)) == -1) {
		fprintf(stderr, "Error:\tbind failed\n");
		close(sock);
		exit(1);
    }

    if (sock == -1 ) {
        fprintf(stderr,
        "Error:\tCannot open socket\n"
        "Info:\tMust be root with an 802.11 card with RFMON enabled\n");
        exit(1);
    }

    return sock;
}

void telemetry_init(telemetry_data_t *td) {
    td->rx_status = telemetry_wbc_status_memory_open();
}

ppm_wbc::ppm_wbc(int interfacecount, char *interfaces[])
{
	int x = 0;
    int num_interfaces = 0;
    
	while(x < interfacecount && num_interfaces < 8) {
		socks[num_interfaces] = open_sock(interfaces[x]);
		++num_interfaces;
		++x;
		usleep(20000); // wait a bit between configuring interfaces to reduce Atheros and Pi USB flakiness
    }

	// Initialize all wbc strcutures
	framedata.rt1 = 0; 		// <-- radiotap version
	framedata.rt2 = 0; 		// <-- radiotap version

	framedata.rt3 = 12; 	// <- radiotap header length
	framedata.rt4 = 0; 		// <- radiotap header length

	framedata.rt5 = 4; 		// <-- radiotap present flags
	framedata.rt6 = 128; 	// <-- radiotap present flags
	framedata.rt7 = 0; 		// <-- radiotap present flags
	framedata.rt8 = 0; 		// <-- radiotap present flags

	framedata.rt9 = 24; 	// <-- radiotap rate
	framedata.rt10 = 0; 	// <-- radiotap stuff
	framedata.rt11 = 0; 	// <-- radiotap stuff
	framedata.rt12 = 0; 	// <-- radiotap stuff

	framedata.fc1 = 180; 	// <-- frame control field (0xb4)
	framedata.fc2 = 191; 	// <-- frame control field (0xbf)
	framedata.dur1 = 0;  	// <-- duration
	framedata.dur2 = 0;  	// <-- duration

	// Default values for all channels
	rcData[0] = 1;
	rcData[1] = 1;
	rcData[2] = 1;
	rcData[3] = 1;
	rcData[4] = 1;
	rcData[5] = 1;
	rcData[6] = 1;
	rcData[7] = 1;
	rcData[8] = 1;
	rcData[9] = 1;
	rcData[10] = 1;
	rcData[11] = 1;
	rcData[12] = 1;
	rcData[13] = 1;
	rcData[14] = 1;
	rcData[15] = 1;

	std::cout << "ppm_wbc Initialized" << std::endl;

	// init RSSI shared memory, used to determine optimal adapter to send trough later on
	telemetry_data_t td;
	telemetry_init(&td);
}

// Figure out wether this is something we want
uint16_t *rc_channels_memory_open(void) {
	int fd = shm_open("/wifibroadcast_rc_channels", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);

	if(fd < 0) {
		std::cerr << "rc shm_open" << std::endl;
		//fprintf(stderr,"rc shm_open\n");
		exit(1);
	}

	if (ftruncate(fd, 9 * sizeof(uint16_t)) == -1) {
		fprintf(stderr,"rc ftruncate\n");
		exit(1);
	}

	void *retval = mmap(NULL, 9 * sizeof(uint16_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (retval == MAP_FAILED) {
		fprintf(stderr,"rc mmap\n");
		exit(1);
	}

	return (uint16_t *)retval;
}

wifibroadcast_rx_status_t *telemetry_wbc_status_memory_open(void) {
    int fd = 0;
    int sharedmem = 0;

    while(sharedmem == 0) {
        fd = shm_open("/wifibroadcast_rx_status_0", O_RDONLY, S_IRUSR | S_IWUSR);
	    if(fd < 0) {
			fprintf(stderr, "Could not open wifibroadcast rx status - will try again ...\n");
	    } else {
			sharedmem = 1;
	    }
	    usleep(100000);
    }

	void *retval = mmap(NULL, sizeof(wifibroadcast_rx_status_t), PROT_READ, MAP_SHARED, fd, 0);
	if (retval == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}

	return (wifibroadcast_rx_status_t*)retval;
}

void sendRC(unsigned char seqno, telemetry_data_t *td) {
    uint8_t i;
    uint8_t z;

    framedata.seqnumber = seqno;
    framedata.chan1 = rcData[0];
    framedata.chan2 = rcData[1];
    framedata.chan3 = rcData[2];
    framedata.chan4 = rcData[3];
    framedata.chan5 = rcData[4];
    framedata.chan6 = rcData[5];
    framedata.chan7 = rcData[6];
    framedata.chan8 = rcData[7];
    framedata.chan9 = rcData[8];
    framedata.chan10 = rcData[9];
    framedata.chan11 = rcData[10];
    framedata.chan12 = rcData[11];
    framedata.chan13 = rcData[12];
    framedata.chan14 = rcData[13];
    framedata.chan15 = rcData[14];
    framedata.chan16 = rcData[15];

    int best_adapter = 0;
    if(td->rx_status != NULL) {
		int j = 0;
		int ac = td->rx_status->wifi_adapter_cnt;
		int best_dbm = -1000;

		// find out which card has best signal and ignore ralink (type=1) ones
		for(j=0; j<ac; ++j) {
			if ((best_dbm < td->rx_status->adapter[j].current_signal_dbm)&&(td->rx_status->adapter[j].type == 0)) {
				best_dbm = td->rx_status->adapter[j].current_signal_dbm;
				best_adapter = j;
			}
		}

		if (write(socks[best_adapter], &framedata, sizeof(framedata)) < 0 ) fprintf(stderr, "!");	/// framedata_s = 28 or 29 bytes
    } else {
		printf ("ERROR: Could not open rx status memory!");
    }
}

int16_t parsetoMultiWii(float value) {
	return (int16_t)(((((double)value)+32768.0)/65.536)+1000);
}

void ppm_wbc::setChannelValue(unsigned char channel, float value)
{
	// Maybe preprocess the value to align with the expectations of WBC MultiWII
	rcData[(int)channel] = parsetoMultiWii(value);
}
