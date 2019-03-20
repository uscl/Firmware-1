
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_serial.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/serial_com.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_outputs.h>

#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

#include <lib/conversion/rotation.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.cpp>
#include <drivers/device/integrator.h>


#include <board_config.h>

//#include "serial_com_parser.h"


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define SERIAL_COM_CONVERSION_INTERVAL	10000 /* SF02 (40m, 12 Hz)*/ //1,000,000 simulink 100hz 83333
#define SERIAL_COM_TAKE_RANGE_REG		'd'

// designated TELEMETRY2 on Pixhawk
#define SERIAL_COM_DEFAULT_PORT		"/dev/ttyS6"	// Serial 4

#define HILS_RXBUF_SIZE 64
#define HILS_PACKET_SIZE 42
#define HILS_TXBUF_SIZE 32

/* CRC table for CCITT-16 (Polynomial: 0x1021) */
unsigned int crc_table[256]=
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};


class SERIAL_COM : public device::CDev
{
public:
	SERIAL_COM(const char *port = SERIAL_COM_DEFAULT_PORT);
	virtual ~SERIAL_COM();

	virtual int 			init();
	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int				ioctl(struct file *filp, int cmd, unsigned long arg);
	void				print_info();
	char							log_status;
	char					tx_status;

protected:
	virtual int			probe();

private:
	char 							_port[20];
	work_s							_work;
	ringbuffer::RingBuffer			*_reports;
	int								_measure_ticks;
	bool							_collect_phase;
	int								_fd;
	char							_linebuf[43];
	unsigned						_linebuf_index;
//	enum SERIAL_COM_PARSE_STATE		_parse_state;
	int								_class_instance;
	int								_orb_class_instance;
	orb_advert_t					_serial_com_topic;
	int 							actuator_outputs_fd;
	unsigned						_consecutive_fail_count;
	perf_counter_t					_sample_perf;
	perf_counter_t					_comms_errors;
	unsigned int 					hils_pktlen;
	unsigned char 					crc_flag_Hils;
	unsigned int 					frame_sync_Hils;
	unsigned char 					*p_gse;
	unsigned int					crc_Hils;
	unsigned int					_HilsTx_FrameLength;
	float 							_actuator_pwm[8];

	// HILS Receiving buffer & Pointer
	unsigned char 					_HilsRxBuffer[HILS_RXBUF_SIZE];
	unsigned int 					_HilsRxBuf_StartPtr;
	unsigned int 					_HilsRxBuf_EndPtr;
	unsigned char 					packet_buf[HILS_PACKET_SIZE];
	unsigned char					_HilsTxBuffer[HILS_TXBUF_SIZE];

	void				start();
	void				stop();
	void				cycle();
	int					measure();
	int					collect();
	unsigned int		send_pwm_packet(unsigned int muxidx);
	unsigned int		Hils_packet_parser(unsigned char *buf);
	void 				build_HILS_packet(unsigned short *pwm_data);

	unsigned short 		TxUint8(unsigned short address, unsigned char value, unsigned char *p);
	unsigned short      TxUint16(unsigned short address, unsigned short value, unsigned char *p);
	unsigned short 		TxUint32(unsigned short address, unsigned int value, unsigned char *p);
	unsigned short 		UpdateCRC(unsigned short crc, unsigned char bytedata);

	int16_t 			RxInt16(unsigned char MSB, unsigned char LSB);
	static void			cycle_trampoline(void *arg);


};

extern "C" __EXPORT int serial_com_main(int argc, char *argv[]);

SERIAL_COM::SERIAL_COM(const char *port) :
	CDev("SERIAL_COM", SERIAL_DEVICE_PATH),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
//	_parse_state(SERIAL_COM_PARSE_STATE0_UNSYNC),
	_class_instance(-1),
	_orb_class_instance(-1),
	_serial_com_topic(nullptr),
	actuator_outputs_fd(-1),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "serial_com_read")),
	_comms_errors(perf_alloc(PC_COUNT, "serial_com_err"))



{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';


	// Hils Rx Buffer initialization
	for (int i=0; i<HILS_RXBUF_SIZE; i++)
	{
		_HilsRxBuffer[i] = 0;
	}

	for (int i=0; i<HILS_TXBUF_SIZE; i++)
	{
		_HilsTxBuffer[i] = 0;
	}

	for (int i=0; i<HILS_PACKET_SIZE; i++)
	{
		packet_buf[i] = 0;
	}

	_HilsTx_FrameLength = 0;

	/* open fd */
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		warnx("FAIL: laser fd");
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	unsigned speed = B115200;	//Baud 115200 bps

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR CFG: %d ISPD", termios_state);
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR CFG: %d OSPD\n", termios_state);
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR baud %d ATTR", termios_state);
	}

	// disable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	hils_pktlen = 0;
	crc_flag_Hils = 0;
	frame_sync_Hils = 0;
	_HilsRxBuf_StartPtr = 0;
	_HilsRxBuf_EndPtr = 0;
	crc_Hils = 0;

	for (int i=0; i<8; i++)
	{
		_actuator_pwm[i] = 0.0f;
	}
	log_status = 0;
	tx_status = 0;

}

SERIAL_COM::~SERIAL_COM()
{
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}


	if (_serial_com_topic != nullptr) {
		orb_unadvertise(_serial_com_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(SERIAL_BASE_DEVICE_PATH, _class_instance);
	}


	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
SERIAL_COM::init()
{
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */
		ret = CDev::init();
		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(serial_com_s));

		if (_reports == nullptr) {
			warnx("mem err");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(SERIAL_BASE_DEVICE_PATH);
		struct serial_com_s ds_report = {};

		_serial_com_topic = orb_advertise_multi(ORB_ID(serial_com), &ds_report,	&_orb_class_instance, ORB_PRIO_MAX);
		if (_serial_com_topic == nullptr) {
			DEVICE_LOG("failed to create serial_com object. Did you start uOrb?");
		}

	} while (0);

	/* close the fd */
	::close(_fd);
	_fd = -1;

	return OK;
}

int
SERIAL_COM::probe()
{
	return measure();
}


int
SERIAL_COM::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd)
	{

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
SERIAL_COM::read(struct file *filp, char *buffer, size_t buflen)  ////////////////////////////read/////////////////////////
{
	unsigned count = buflen / sizeof(struct serial_com_s);
	struct serial_com_s *rbuf = reinterpret_cast<struct serial_com_s *>(buffer);
	int ret = 0;

	if (count < 1) {
		return -ENOSPC;
	}

	if (_measure_ticks > 0)
	{
			 // While there is space in the caller's buffer, and reports, copy them.
			 // Note that we may be pre-empted by the workq thread while we are doing this;
			 // we are careful to avoid racing with them.

		while (count--) {
					if (_reports->get(rbuf)) {
						ret += sizeof(*rbuf);
						rbuf++;
					}
				}


			// if there was no data, warn the caller
			warnx("SERIAL_COM::read while out ret : %d", ret); //64
			return ret ? ret : -EAGAIN; // try again
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		warnx("read start");
		/* wait for it to complete */
		usleep(SERIAL_COM_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect())
		{
			warnx("collect %d", collect());
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

		warnx("SERIAL_COM::collect ret, collect : %d, %d" , ret,collect()); // -1 3 10

	} while (0);

	return ret;
}

int
SERIAL_COM::measure()
{

	/* get polling data from the actuator output */
	struct actuator_outputs_s actuators;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = actuator_outputs_fd;
	fds[0].events = POLLIN;

	/* wait for sensor update of 1 file descriptor for 10 ms (1 second) */
	int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

	/* handle the poll result */
	if (poll_ret == 0)
	{
		/* this means none of our providers is giving us data */
		//PX4_ERR("Got no data within a specified time");

	}
	else if (poll_ret < 0)
	{
	/* this is seriously bad - should be an emergency */
		//PX4_ERR("Critical error return value from poll(): %d", poll_ret);
	}
	else
	{
		if (fds[0].revents & POLLIN)
		{
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(actuator_outputs), actuator_outputs_fd, &actuators);

			for (int i=0; i<8; i++)
			{
				_actuator_pwm[i] = actuators.output[i];
			}

			//send_pwm_packet(1);
		}
	}


	return 0;
}

int16_t SERIAL_COM::RxInt16(unsigned char MSB, unsigned char LSB)
{
	int16_t dummy1, dummy2;
	dummy1 = (int16_t)MSB;
	dummy2 = (int16_t)LSB;

	return (dummy1 <<8) | dummy2;
}


unsigned short SERIAL_COM::TxUint32(unsigned short address, unsigned int value, unsigned char *p)
{

	*(p+0) = (value>>24) & 0xFF;
	*(p+1) = (value>>16) & 0xFF;
	*(p+2) = (value>>8) & 0xFF;
	*(p+3) =  value & 0xFF;

	return (address+4);
}

unsigned short SERIAL_COM::TxUint8(unsigned short address, unsigned char value, unsigned char *p)
{
	*(p+0) = value & 0xFF;

	return (address+1);
}


unsigned short SERIAL_COM::TxUint16(unsigned short address, unsigned short value, unsigned char *p)
{

	*(p+0) = (value>>8)&0xFF;
	*(p+1) = value&0xFF;

	return (address+2);
}

unsigned short SERIAL_COM::UpdateCRC(unsigned short crc, unsigned char bytedata)
{
	unsigned short crcidx, datain;

	datain  = 0x00ff & (unsigned short) bytedata;

    crcidx = (crc >> 8) ^ (datain);
	crc = (crc << 8) ^ crc_table[crcidx];

	return crc;
}

unsigned int SERIAL_COM::Hils_packet_parser(unsigned char *buf)
{
	float *float_ptr;

	int16_t roll_angle, pitch_angle, yaw_angle;
	int16_t roll_rate, pitch_rate, yaw_rate;
	int16_t a_n, a_e, a_d;
	int16_t v_n, v_e, v_d;
	float latitude, longitude, altitude;
	unsigned int muxidx;

	muxidx = ((int16_t)buf[2] << 8) | (int16_t)buf[3];

	float_ptr  = (float*)&buf[4];

	latitude = *(float_ptr+0);		// 4  4567
	longitude = *(float_ptr+1);		// 8  891011
	altitude = *(float_ptr+2);		// 12 12131415

	roll_angle = ((int16_t)buf[16] << 8) | (int16_t)buf[17];
	pitch_angle =  ((int16_t)buf[18] << 8) | (int16_t)buf[19];
	yaw_angle =  ((int16_t)buf[20] << 8) | (int16_t)buf[21];

	roll_rate =  ((int16_t)buf[22] << 8) | (int16_t)buf[23];
	pitch_rate = ((int16_t)buf[24] << 8) | (int16_t)buf[25];
	yaw_rate = ((int16_t)buf[26] << 8) | (int16_t)buf[27];

	a_n = ((int16_t)buf[28] << 8) | (int16_t)buf[29];
	a_e = ((int16_t)buf[30] << 8) | (int16_t)buf[31];
	a_d = ((int16_t)buf[32] << 8) | (int16_t)buf[33];

	v_n = ((int16_t)buf[34] << 8) | (int16_t)buf[35];
	v_e = ((int16_t)buf[36] << 8) | (int16_t)buf[37];
	v_d = ((int16_t)buf[38] << 8) | (int16_t)buf[39];

	struct serial_com_s report = {};
	report.timestamp = hrt_absolute_time();

	report.roll_angle = roll_angle; //degree
	report.pitch_angle = pitch_angle; //degree
	report.yaw_angle = yaw_angle; //degree

	report.roll_rate = roll_rate;
	report.pitch_rate = pitch_rate;
	report.yaw_rate = yaw_rate;

	report.x_n = a_n; //cm/s^2
	report.y_e = a_e; //cm/s^2
	report.z_d = a_d; //cm/s^2

	report.v_n = v_n; //cm/s
	report.v_e = v_e; //cm/s
	report.v_d = v_d; //cm/s

	report.latitude = latitude;
	report.longitude = longitude;
	report.altitude = altitude;


	if (_serial_com_topic != nullptr)
	{
		orb_publish(ORB_ID(serial_com), _serial_com_topic, &report);

		if (log_status)
		{
			warnx("p=%d, q=%d, r=%d", report.roll_rate,report.pitch_rate,report.yaw_rate); // 0 3
		}
		//warnx("\n roll=%d, pitch=%d, yaw=%d", report.roll_angle,report.pitch_angle,report.yaw_angle);
		//warnx("\n roll_rate=%d   pitch_rate=%d   yaw_rate=%d", report.roll_rate,report.pitch_rate,report.yaw_rate); // 0 3
		//warnx("\n x_n=%d, y_e=%d, z_d=%d", report.x_n,report.y_e,report.z_d);
		//warnx("\n lat=%4.3f, lon=%4.3f, alt=%4.3f", (double)report.latitude,(double)report.longitude,(double)report.altitude);

	}
	else
	{
		_serial_com_topic = orb_advertise_multi(ORB_ID(serial_com), &report,
						 &_orb_class_instance, ORB_PRIO_MAX);

		if (_serial_com_topic == nullptr)
		{
			DEVICE_DEBUG("ADVERT FAIL");
		}
	}

	return muxidx;
}

unsigned int SERIAL_COM::send_pwm_packet(unsigned int muxidx)
{
	int ret = 0;

	unsigned short test[8];

	for (int i=0; i<8; i++)
	{
		test[i] = (unsigned short)(_actuator_pwm[i]*10.0f);
	}

	// Build packet
	build_HILS_packet(&test[0]);

	// Send packet
	ret = ::write(_fd, &_HilsTxBuffer, _HilsTx_FrameLength);

	return ret;
}

void SERIAL_COM::build_HILS_packet(unsigned short *pwm_data)
{
	int i;
	unsigned short crc = 0x0;
	unsigned short packet_len = 0;

	_HilsTxBuffer[packet_len++] = 0x7E;
	_HilsTxBuffer[packet_len++] = 0x81;

	// Data length
	packet_len = TxUint8(packet_len, 0, &_HilsTxBuffer[packet_len]);

	// Data
	packet_len = TxUint16(packet_len, pwm_data[0], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[1], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[2], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[3], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[4], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[5], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[6], &_HilsTxBuffer[packet_len]);
	packet_len = TxUint16(packet_len, pwm_data[7], &_HilsTxBuffer[packet_len]);

	_HilsTxBuffer[2] = packet_len+2;

	for ( i = 2 ; i < packet_len; i++)
		crc = UpdateCRC(crc, _HilsTxBuffer[i]);

	// Add CRC bytes
	_HilsTxBuffer[packet_len++] = (crc>>8) & 0xFF;
	_HilsTxBuffer[packet_len++] = crc & 0xFF;

	// Update the frame length
	_HilsTx_FrameLength = packet_len;

}

int
SERIAL_COM::collect()
{
	int ret;
	int i;
	unsigned int 	crc_index;
	unsigned int 	muxidx;

	perf_begin(_sample_perf);

	// the buffer for read chars is buflen minus null termination
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	// read from the Hils (uart buffer)
	ret = ::read(_fd , &readbuf[0], readlen);

	//warnx("\n a = %d, b = %d", readlen,ret);//warnx("\n %d", 1);

	bool valid = true;

	for (i = 0; i < ret; i++)
	{
		// Put the read binary data into packet
		_HilsRxBuffer[_HilsRxBuf_EndPtr++] = readbuf[i];
		_HilsRxBuf_EndPtr %= HILS_RXBUF_SIZE;

		// Get hils packet
		while (_HilsRxBuf_EndPtr != _HilsRxBuf_StartPtr)
		{
			if (crc_flag_Hils == 0)
			{
				frame_sync_Hils = (frame_sync_Hils << 8) | _HilsRxBuffer[_HilsRxBuf_StartPtr++];
				_HilsRxBuf_StartPtr %= HILS_RXBUF_SIZE;
				if (frame_sync_Hils == 0x7E81)
				{
					//warnx("\n %d", 2);
					p_gse = packet_buf + 2;
					hils_pktlen = 2;
					crc_Hils = 0;
					crc_flag_Hils = 1; // Ready for CRC test
				}
			}
			else
			{
				*p_gse = _HilsRxBuffer[_HilsRxBuf_StartPtr++];
				_HilsRxBuf_StartPtr %= HILS_RXBUF_SIZE;

				crc_index = (crc_Hils>>8)^(*p_gse++);
				crc_Hils = (crc_Hils<<8)^crc_table[crc_index & 0x00FF];
				if ((++hils_pktlen) >= HILS_PACKET_SIZE)
				{
					//warnx("\n %d", 4);
					if ((crc_Hils&0xFFFF) == 0)		// Success on receiving a complete packet
					{
						//warnx("\n %d", 5);
						p_gse = packet_buf + 2;
						packet_buf[0] = 0x7E;
						packet_buf[1] = 0x81;

						//muxidx = Hils_packet_parser(&packet_buf[0]);
						muxidx = Hils_packet_parser(&packet_buf[0]);

						if (tx_status)
						{
							send_pwm_packet(muxidx);
						}
						valid = true;

					}
					crc_flag_Hils = 0; // Ready for search 0x7E81
					hils_pktlen = 0;
					frame_sync_Hils = 0;
				}
			}
		}
	}
	//warnx("\n crc = %x, ret = %d", crc_Hils, ret);

	perf_end(_sample_perf);

	if (!valid) {
		return -EAGAIN;
	}
	//warnx("\n %d", 6);
	return OK;
}

void
SERIAL_COM::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	work_queue(HPWORK, &_work, (worker_t)&SERIAL_COM::cycle_trampoline, this, 1);

}

void
SERIAL_COM::stop()
{
	work_cancel(HPWORK, &_work);
}

void
SERIAL_COM::cycle_trampoline(void *arg)
{
	SERIAL_COM *dev = static_cast<SERIAL_COM *>(arg);

	dev->cycle();
}

void
SERIAL_COM::cycle()
{

	/* actuator output subscription is valid? */

	if (actuator_outputs_fd < 0)
	{
		/* New subscription */
		actuator_outputs_fd = orb_subscribe(ORB_ID(actuator_outputs));
		orb_set_interval(actuator_outputs_fd, 10);
	}

	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			work_queue(HPWORK, &_work, (worker_t)&SERIAL_COM::cycle_trampoline, this, USEC2TICK(1042 * 8));
			return;
		}

		if (OK != collect_ret) //-1
		{
			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				DEVICE_LOG("collection error #%u", _consecutive_fail_count);
				warnx("collect_return %d", collect_ret);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;
		}

		else
		{
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL))
		{
			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK, &_work, (worker_t)&SERIAL_COM::cycle_trampoline, this, _measure_ticks - USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL));
			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_LOG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK, &_work, (worker_t)&SERIAL_COM::cycle_trampoline, this, USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL));
}

void
SERIAL_COM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("tx_status: %d, log_status: %d\n", tx_status, log_status);
}


namespace serial_com
{

#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

SERIAL_COM	*g_dev;

void	start(const char *port);
void	stop();
void	test();
void 	test2();
void	reset();
void	info();
void 	enable_log();
void 	disable_log();
void 	enable_PWMout();
void 	disable_PWMout();

/**
 * Start the driver.
 */
void
start(const char *port)
{
	int fd;
	int ioc;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new SERIAL_COM(port);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SERIAL_DEVICE_PATH, 0);

	warnx("/dev/serial0 open success : %d", fd); //3

	if (fd < 0) {
		warnx("device open fail");
		goto fail;
	}
	ioc = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);

	if (ioc < 0) {
		warnx("ioctl < 0 : %d", ioc);
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

void
test()
{
	struct serial_com_s report;
	ssize_t sz;

	char testbyte;
	int j;

	//int fd = open(SERIAL_DEVICE_PATH, O_RDONLY);//O_RDWR);

	int fd = open("/dev/ttyS6", O_WRONLY);

	testbyte = 'A';
	for (j=0; j<100; j++)
	{
		sz = write(fd, &testbyte, 1);
		usleep(10000);
	}

	// do a simple demand read
	// start the sensor polling at 2 Hz rate
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	// read the sensor 5x and report each value
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		// wait for data to be ready
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out");
			break;
		}

		// now go get it
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warnx("read failed: got %d vs exp. %d", sz, sizeof(report));
			break;
		}
	}


	// reset the sensor polling to the default rate
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "ERR: DEF RATE");
	}

	errx(0, "PASS");

}

void test2()
{

	int actuator_outputs_fd = orb_subscribe(ORB_ID(actuator_outputs));
	//orb_set_interval(actuator_outputs_fd, 50);
	orb_set_interval(actuator_outputs_fd, 10);

	struct actuator_outputs_s actuators;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = actuator_outputs_fd;
	fds[0].events = POLLIN;

	int error_counter = 0;

	for (int i = 0; i < 5000; i++) {
		/* wait for sensor update of 1 file descriptor for 20 ms (1 second) */
//		int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);
		int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		usleep(5000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			//PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d"
					, poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(actuator_outputs), actuator_outputs_fd, &actuators);
				PX4_WARN("PWM:\t%3.1f\t%3.1f\t%3.1f\t%3.1f",
						(double)actuators.output[0],
						(double)actuators.output[1],
						(double)actuators.output[2],
						(double)actuators.output[3]);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

}

void
enable_log()
{
	g_dev->log_status = 1;
	exit(0);
}

void disable_log()
{
	g_dev->log_status = 0;
	exit(0);
}

void
enable_PWMout()
{
	g_dev->tx_status = 1;
	exit(0);
}

void disable_PWMout()
{
	g_dev->tx_status = 0;
	exit(0);
}


void
reset()
{
	int fd = open(SERIAL_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

void
info()
{
	if (g_dev == nullptr)	errx(1, "driver not running");
	printf("state @ %p\n", g_dev);
	g_dev->print_info();
	exit(0);
}

} // namespace

int
serial_com_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (argc > 2) {
			serial_com::start(argv[2]);

		} else {
			serial_com::start(SERIAL_COM_DEFAULT_PORT);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		serial_com::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		serial_com::test();
	}

	if (!strcmp(argv[1], "test2")) {
		serial_com::test2();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		serial_com::reset();
	}

	/*
	 * Display logging on
	 */
	if (!strcmp(argv[1], "logon")) {
		serial_com::enable_log();
	}

	/*
	 * Display logging off
	 */
	if (!strcmp(argv[1], "logoff")) {
		serial_com::disable_log();
	}

	/*
	 * Tx PWM enable
	 */
	if (!strcmp(argv[1], "txON")) {
		serial_com::enable_PWMout();
	}

	/*
	 * Tx PWM disable
	 */
	if (!strcmp(argv[1], "txOFF")) {
		serial_com::disable_PWMout();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		serial_com::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info' or 'logon' or 'logoff' or 'test2' or 'txON' or 'txOFF");
}
