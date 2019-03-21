
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

//#include <systemlib/perf_counter.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
//#include <drivers/drv_serial.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/serial_com.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/mission_info.h>
#include <uORB/topics/start_goal_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/avoidance_path.h>
#include <uORB/topics/calculate_next_path.h>
#include <uORB/topics/mavlink_log.h>

#include <lib/conversion/rotation.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.cpp>
#include <drivers/device/integrator.h>
#include <matrix/matrix/math.hpp>
#include <systemlib/mavlink_log.h>
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

#define INTERFACE_RXBUF_SIZE 50
#define INTERFACE_PACKET_SIZE 42
#define INTERFACE_TXBUF_SIZE 100

typedef enum JETSON_STATE {
	JETSON_STATE_INIT = 0,
	JETSON_STATE_PTHREAD_CREATED = 1,
	JETSON_STATE_LIDAR_ERROR = 2,
	JETSON_STATE_PLANNING_ERROR = 3,
	JETSON_STATE_SMOOTHING_ERROR = 4,
	JETSON_STATE_GUIDANCE_ERROR = 5,
	JETSON_STATE_COMM_ERROR = 6,
} JETSON_STATE;
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

protected:
	virtual int			probe();

private:
	char 							_port[20];
	work_s							_work;
	ringbuffer::RingBuffer			*_reports;
	int								_measure_ticks;
	bool							_collect_phase;
	int								_fd;
	char							_linebuf[50];
	unsigned						_linebuf_index;
//	enum SERIAL_COM_PARSE_STATE		_parse_state;
	int								_class_instance;
	int								_orb_class_instance;
	orb_advert_t					_serial_com_topic;
	orb_advert_t					_mavlink_log_pub;
	int 							actuator_outputs_fd;
	int								vehicle_global_position_fd;
	int								vehicle_attitude_fd;
	int								_Mission_info_sub;
	int								_home_pos_sub;
	int								_vehicle_control_mode_sub;
	int								_armed_sub;
	int								_input_rc_sub;
	int								_calc_next_path_sub;
	unsigned						_consecutive_fail_count;
	perf_counter_t					_sample_perf;
	perf_counter_t					_comms_errors;
	unsigned int 					hils_pktlen;
	unsigned char 					crc_flag_Hils;
	unsigned int 					frame_sync_Hils;
	unsigned char 					*p_gse;
	unsigned int					crc_Hils;
	unsigned short					_actuator_pwm[8];

	float							_quarternion[4];

	// HILS Receiving buffer & Pointer
	unsigned char 					_HilsRxBuffer[HILS_RXBUF_SIZE];
	unsigned int 					_HilsRxBuf_StartPtr;
	unsigned int 					_HilsRxBuf_EndPtr;
	unsigned char 					packet_buf[HILS_PACKET_SIZE];
	unsigned char					_HilsTxBuffer[HILS_TXBUF_SIZE];

	bool m_bpx4ready;
	uint8_t _statusJetson;
	uint8_t _prev_statusJetson;
	bool _mavLog_status_flag;
	int m_bRecievedReadyACK;
	int m_bReceivedMissionInfoACK;
	int m_bRecievedStart_and_GoalACK;
	bool m_bRecievedChange_Start_and_Goal;
	int m_bSendStopFlag;
	int m_bReceivedStopACK;
	bool m_bInterface_Processing;

	float _mission_area_NW[2], _mission_area_NE[2], _mission_area_SW[2], _mission_area_SE[2];
	bool m_bUpdateMissionArea;
	float _start_position[3],_end_position[3], _home_position[3];
	float _cellwidth;
	uint32_t _width, _height,_thres0, _thresd;
	bool m_bUpdateStart_and_Goal;
	bool m_bSendpx4ready, m_bSendMissionArea, m_bSendStart_and_Goal;
	int m_bSendMissionFlag, m_bSendMissionInfo;
	int m_bReceivedMissionFlag_ACK;
	bool m_bChangedMode;
	int _CalcPathFlag;
	int m_bCalcPathFlag;
	uint8_t _command_calc_path;

	//Interface between jetson and pixhawk Receiving buffer & pointer


	void				start();
	void				stop();
	void				cycle();
	int					measure();
	int					collect();
	unsigned int		Interface_packet_parser(unsigned char *buf);

	unsigned short		TxFloat32(unsigned short address, float value, unsigned char *p);
	unsigned short 		TxUint8(unsigned short address, unsigned char value, unsigned char *p);
	unsigned short      TxUint16(unsigned short address, unsigned short value, unsigned char *p);
	unsigned short 		TxUint32(unsigned short address, unsigned int value, unsigned char *p);
	unsigned short 		UpdateCRC(unsigned short crc, unsigned char bytedata);



	int16_t 			RxInt16(unsigned char MSB, unsigned char LSB);
	static void			cycle_trampoline(void *arg);

public:
	float							_mc_lat;
	float							_mc_lon;
	float							_mc_alt;
	float							_mc_yaw;
	float							_home_alt;

	unsigned int		send_mission_info_packet(void);
	unsigned int		send_mission_flag_packet(uint8_t mission_flag_cmd);
	unsigned int 		send_stop_packet(uint8_t stop_flag_cmd);
	unsigned int 		send_calculate_path(uint8_t command_calc_path);
	unsigned int		send_state_packet(void);
	void 				build_mission_info_packet();
	void 				build_mission_flag_packet(uint8_t mission_flag_cmd);
	void 				build_state_packet();

	unsigned char 					_InterfaceRxBuffer[INTERFACE_RXBUF_SIZE];
	unsigned int 					_InterfaceRxBuf_StartPtr;
	unsigned int 					_InterfaceRxBuf_EndPtr;
	unsigned char 					_Interface_packet_buf[INTERFACE_PACKET_SIZE];
	unsigned char					_InterfaceTxBuffer[INTERFACE_TXBUF_SIZE];
	unsigned int					_HilsTx_FrameLength;

};

extern "C" __EXPORT int serial_com_main(int argc, char *argv[]);

SERIAL_COM::SERIAL_COM(const char *port) :
	CDev("SERIAL_COM", "/dev/serial_com0"),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
//	_parse_state(SERIAL_COM_PARSE_STATE0_UNSYNC),
	_class_instance(-1),
	_orb_class_instance(-1),
	_serial_com_topic(nullptr),
	_mavlink_log_pub(nullptr),
	actuator_outputs_fd(-1),
	vehicle_global_position_fd(-1),
	vehicle_attitude_fd(-1),
	_Mission_info_sub(-1),
	_home_pos_sub(-1),
	_vehicle_control_mode_sub(-1),
	_armed_sub(-1),
	_input_rc_sub(-1),
	_calc_next_path_sub(-1),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "serial_com_read")),
	_comms_errors(perf_alloc(PC_COUNT, "serial_com_err")),
	m_bpx4ready(false),
	_statusJetson(0),
	_prev_statusJetson(0),
	_mavLog_status_flag(false),
	m_bRecievedReadyACK(-1),
	m_bReceivedMissionInfoACK(-1),
	m_bRecievedStart_and_GoalACK(-1),
	m_bRecievedChange_Start_and_Goal(false),
	m_bSendStopFlag(-1),
	m_bReceivedStopACK(-1),
	m_bInterface_Processing(false),
	m_bUpdateMissionArea(false),
	_cellwidth(0.0f),
	_width(0),
	_height(0),
	_thres0(0),
	_thresd(0),
	m_bUpdateStart_and_Goal(false),
	m_bSendpx4ready(false),
	m_bSendMissionArea(false),
	m_bSendStart_and_Goal(false),
	m_bSendMissionFlag(-1),
	m_bSendMissionInfo(-1),
	m_bReceivedMissionFlag_ACK(-1),
	m_bChangedMode(false),
	_CalcPathFlag(-1),
	m_bCalcPathFlag(-1),
	_command_calc_path(-1),
	_mc_lat(0.0f),
	_mc_lon(0.0f),
	_mc_alt(0.0f),
	_mc_yaw(0),
	_home_alt(0.0f)

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
///////////////////////////////////////////////////
	for (int i=0; i<INTERFACE_RXBUF_SIZE; i++)
	{
		_InterfaceRxBuffer[i] = 0;
	}

	for (int i=0; i<INTERFACE_TXBUF_SIZE; i++)
	{
		_InterfaceTxBuffer[i] = 0;
	}

	for (int i=0; i<INTERFACE_PACKET_SIZE; i++)
	{
		_Interface_packet_buf[i] = 0;
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
	p_gse = nullptr;
	_HilsRxBuf_StartPtr = 0;
	_HilsRxBuf_EndPtr = 0;
	crc_Hils = 0;

	_InterfaceRxBuf_StartPtr = 0;
	_InterfaceRxBuf_EndPtr = 0;

	for (int i=0; i<8; i++)
	{
		_actuator_pwm[i] = 0;
	}
	for(int i = 0; i< 50; i++)
	{
		_linebuf[i] = 0;
	}
	log_status = 0;

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

	if(_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}

	if (_class_instance != -1) {
		unregister_class_devname("/dev/serial_com", _class_instance);
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

		_class_instance = register_class_devname("/dev/serial_com");
		struct serial_com_s ds_report = {};

		_serial_com_topic = orb_advertise_multi(ORB_ID(avoidance_path), &ds_report,	&_orb_class_instance, ORB_PRIO_MAX);
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

	case SENSORIOCSPOLLRATE:

	{
			switch (arg)
			{

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
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
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	//case SENSORIOCGQUEUEDEPTH:
		//return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
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
	if(_statusJetson == JETSON_STATE_PTHREAD_CREATED) {
		struct actuator_armed_s armed;
		bool arming_updated = false;

		orb_check(_armed_sub, &arming_updated);

		if(arming_updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &armed);

			bool mission_area_updated = false;
			bool home_pos_updated = false;
			bool updated = false;
			bool rc_updated = false;
			//bool InMissionArea = false;

			uint8_t mission_flag_cmd = 0;
			uint8_t stop_flag_cmd = 0;

			/* get polling data from the actuator output */
			struct vehicle_global_position_s global_pos;
			struct vehicle_attitude_s mc_att;
			struct mission_info_s mission_info;
			struct vehicle_control_mode_s control_mode;
			struct input_rc_s input_rc;
			struct home_position_s home_pos;
			struct calculate_next_path_s calc_path;

			orb_check(_Mission_info_sub, &mission_area_updated);
			orb_check(_home_pos_sub, &home_pos_updated);

			updated = mission_area_updated;
			updated |= home_pos_updated;

			if(updated)
			{
				if(m_bSendMissionInfo < 0)
				{
					orb_copy(ORB_ID(mission_info), _Mission_info_sub, &mission_info);
					orb_copy(ORB_ID(home_position), _home_pos_sub, &home_pos);
					_mission_area_NW[0] = (mission_info.mission_area_NW[0]);
					_mission_area_NW[1] = (mission_info.mission_area_NW[1]);

					_mission_area_NE[0] =(mission_info.mission_area_NE[0]);
					_mission_area_NE[1] = (mission_info.mission_area_NE[1]);

					_mission_area_SW[0] = mission_info.mission_area_SW[0];
					_mission_area_SW[1] = mission_info.mission_area_SW[1];

					_mission_area_SE[0] = (mission_info.mission_area_SE[0]);
					_mission_area_SE[1] = (mission_info.mission_area_SE[1]);

					_start_position[0] = (mission_info.start_position[0]);
					_start_position[1] = (mission_info.start_position[1]);
					_start_position[2] = mission_info.start_position[2];

					_end_position[0] = (mission_info.end_position[0]);
					_end_position[1] = (mission_info.end_position[1]);
					_end_position[2] = mission_info.end_position[2];

					_home_position[0] = mission_info.mission_area_SW[0];
					_home_position[1] = mission_info.mission_area_SW[1];
					_home_position[2] = home_pos.alt;

					_cellwidth = mission_info.cellwidth;
					_width = mission_info.width;
					_height = mission_info.height;
					_thres0 = mission_info.thres0;
					_thresd = mission_info.thresd;

					printf("MissionNW = %3.6f,   %3.6f,   %3.6f\n", (double)_mission_area_NW[0], (double)_mission_area_NW[1], (double)_home_position[2]);
					printf("MissionEnd = %3.6f,   %3.6f\n", (double)_end_position[0], (double)_end_position[1]);
					printf("MissionStart = %3.6f,    %3.6f\n", (double)_start_position[0], (double)_start_position[1]);
					if((_start_position[0] > 1.0f &&_start_position[0] < 39.0f )&& (_start_position[1] > 1.0f && _start_position[1] < 128.0f)
							&& ( _end_position[0] > 1.0f && _end_position[0] < 39.0f) && (_end_position[1] > 1.0f && _end_position[1] < 128.0f)
							&& ((_home_position[2] > 0.3f && _home_position[2] < 300.0f)|| (_home_position[2] < -0.3f && _home_position[2] > -300.0f) ))
					{
						m_bUpdateMissionArea = true;
						m_bUpdateStart_and_Goal = true;
						send_mission_info_packet();
						m_bSendMissionInfo = -1;
					}
				}
			}
			if(armed.armed) {
				if(m_bSendMissionFlag < 0)
				{
					orb_check(_vehicle_control_mode_sub, &updated);
					orb_check(_input_rc_sub, &rc_updated);
					if(updated && rc_updated)
					{
						orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &control_mode);
						orb_copy(ORB_ID(input_rc), _input_rc_sub, &input_rc);
						if(control_mode.flag_control_auto_enabled)
						{
							if(input_rc.values[5] > 1500)
							{
								mission_flag_cmd = true;
								send_mission_flag_packet(mission_flag_cmd);
								m_bSendMissionFlag = -1;
								printf("send mission flag true first\n");
							}
						}
					}
				} else if(m_bSendMissionFlag > 0) {
					orb_check(_vehicle_control_mode_sub, &updated);
					orb_check(_input_rc_sub, &rc_updated);
					if(updated && rc_updated)
					{
						orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &control_mode);
						orb_copy(ORB_ID(input_rc), _input_rc_sub, &input_rc);
						if(control_mode.flag_control_auto_enabled)
						{
							if(input_rc.values[5] < 1500)
							{
								mission_flag_cmd = false;
								send_mission_flag_packet(mission_flag_cmd);
								m_bSendMissionFlag = 1;
								m_bChangedMode = true;
								printf("send mission flag false low 1500\n");
							} else if(input_rc.values[5] > 1500 && m_bChangedMode == true) {
								m_bChangedMode = false;
								m_bSendMissionFlag = -1;
								printf("send mission flag true\n");
							}
						} else if(!control_mode.flag_control_auto_enabled){
							m_bChangedMode = true;
							mission_flag_cmd = false;
							send_mission_flag_packet(mission_flag_cmd);
							m_bSendMissionFlag = 1;
							printf("send mission flag manual\n");
						}
					}
				}

				if(m_bReceivedMissionFlag_ACK > 0)
				{
					bool calc_updated = false;
					orb_check(_calc_next_path_sub, &calc_updated);
					if(calc_updated) {
						orb_copy(ORB_ID(calculate_next_path), _calc_next_path_sub, &calc_path);
						_CalcPathFlag = (int)calc_path.CalcPathFlag;
						_command_calc_path = (uint8_t)calc_path.calculate_next;
						m_bCalcPathFlag = -1;
					}

					if(m_bCalcPathFlag < 0) {
						if(_CalcPathFlag > 0) {
							send_calculate_path(_command_calc_path);
							m_bCalcPathFlag = -1;
						}
					}

					px4_pollfd_struct_t fds[2] = {};

					fds[0].fd = vehicle_global_position_fd;
					fds[0].events = POLLIN;

					fds[1].fd = vehicle_attitude_fd;
					fds[1].events = POLLIN;
					/* wait for sensor update of 1 file descriptor for 10 ms (1 second) */

					int poll_ret0 = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 200);

					/* handle the poll result */
					if (poll_ret0 == 0)
					{
						/* this means none of our providers is giving us data */
						PX4_ERR("Got no gps, att data within a specified time");

					}
					else if (poll_ret0 < 0)
					{
					/* this is seriously bad - should be an emergency */
						PX4_ERR("Critical error return value from poll(): %d", poll_ret0);
					}
					else
					{
						if (fds[0].revents & POLLIN)
						{
							/* copy sensors raw data into local buffer */
							orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_fd, &global_pos);

							_mc_lat = (float)global_pos.lat;
							_mc_lon = (float)global_pos.lon;
							_mc_alt = (float)global_pos.alt;
						}

						if (fds[1].revents & POLLIN)
						{
							/* copy sensors raw data into local buffer */
							orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_fd, &mc_att);

							float dcm[3][3];
							float a = mc_att.q[0];
							float b = mc_att.q[1];
							float c = mc_att.q[2];
							float d = mc_att.q[3];
							float aa = a * a;
							float ab = a * b;
							float ac = a * c;
							float ad = a * d;
							float bb = b * b;
							float bc = b * c;
							float bd = b * d;
							float cc = c * c;
							float cd = c * d;
							float dd = d * d;
							dcm[0][0] = aa + bb - cc - dd;
							dcm[0][1] = 2 * (bc - ad);
							dcm[0][2] = 2 * (ac + bd);
							dcm[1][0] = 2 * (bc + ad);
							dcm[1][1] = aa - bb + cc - dd;
							dcm[1][2] = 2 * (cd - ab);
							dcm[2][0] = 2 * (bd - ac);
							dcm[2][1] = 2 * (ab + cd);
							dcm[2][2] = aa - bb - cc + dd;

							//float phi_val = float(atan2f(dcm[2][1], dcm[2][2]));
							float theta_val = float(asinf(-dcm[2][0]));
							float psi_val = float(atan2f(dcm[1][0], dcm[0][0]));
							float pi = float(M_PI);

							if (float(fabs(theta_val - pi / float(2))) < float(1.0e-3)) {
								//phi_val = float(0.0);
								psi_val = float(atan2f(dcm[1][2], dcm[0][2]));

							} else if (float(fabs(theta_val + pi / float(2))) < float(1.0e-3)) {
								//phi_val = float(0.0);
								psi_val = float(atan2f(-dcm[1][2], -dcm[0][2]));
							}
							_mc_yaw = psi_val;
							//printf("roll = %3.6f, pitch = %3.6f, yaw = %3.6f\n",math::degrees((double)phi_val), math::degrees((double)theta_val), math::degrees((double)psi_val));
						}
						send_state_packet();
						//send_State_packet() //all state;
					}
				}
				m_bInterface_Processing = true;
			}

			if(m_bInterface_Processing) {
				if(!armed.armed) {
					if(m_bSendStopFlag < 0) {
						stop_flag_cmd = true;
						send_stop_packet(stop_flag_cmd);
						m_bSendStopFlag = -1;
					}
					//m_bSendMissionInfo = -1;
					//m_bSendMissionFlag = -1;
					//m_bReceivedMissionFlag_ACK = -1;
					//m_bReceivedStopACK = -1;
				}
			}
			if(m_bReceivedStopACK > 0) {
				m_bInterface_Processing = false;
			}
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

unsigned short SERIAL_COM::TxFloat32(unsigned short address, float value, unsigned char *p)
{
	unsigned short *p1, *p2;
	unsigned short lv[4];

	p1 = (unsigned short*)&value;
	p2 = lv;

	*p2++ = *(p1+0)&0xFF;	*p2++ = *(p1+0)>>8;
	*p2++ = *(p1+1)&0xFF;	*p2++ = *(p1+1)>>8;

	for(int i = 0; i < 4; i++)
	{
		*p++ = lv[i] &0xFF;
	}
	return (address + 4);
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

unsigned int SERIAL_COM::Interface_packet_parser(unsigned char *buf)
{
	float *float_ptr;

	//float roll_cmd = 0.0f, pitch_cmd = 0.0f, yaw_cmd = 0.0f;
	//float es = 0.0f, ed = 0.0f;
	//float sdot = 0.0f, ud = 0.0f, vd = 0.0f, omegad = 0.0f;

	int pathidx;
	char mflag;
	int k;
	int l;
	float alpha;
	bool isValidPoint = false;
	uint8_t isLastPoint = 0;
//	uint8_t length;

	uint8_t packet_id;

	//length = (uint8_t)buf[2];
	//printf("length = %d\n",length);


	packet_id = (uint8_t)buf[3];

	//printf("ID = %d\n",packet_id);
	float_ptr  = (float*)&buf[4];

	if(packet_id == 0) {
		_statusJetson = (uint8_t)buf[4];

		if(_prev_statusJetson !=_statusJetson) {
			_mavLog_status_flag = false;
			//mavlink_log_info(&_mavlink_log_pub, "(%d)Jetson TX2 Status", (int)_statusJetson);
		}

		if(!_mavLog_status_flag) {
			mavlink_log_info(&_mavlink_log_pub, "(%d)Jetson TX2 Status", (int)_statusJetson);
			_mavLog_status_flag = true;
		}

		_prev_statusJetson = _statusJetson;
	}
	else if(packet_id == 1)
	{
		//printf("debug=%d\n",2);
		m_bReceivedMissionInfoACK = 1;
		m_bSendMissionInfo = 1;
		mavlink_log_info(&_mavlink_log_pub, "Jetson TX2 received mission info");
	}

	else if(packet_id == 2)
	{
		m_bReceivedMissionFlag_ACK = 1;
		m_bSendMissionFlag = 1;
		mavlink_log_info(&_mavlink_log_pub, "Jetson TX2 received mission flag");
	}
	else if(packet_id == 3) {
		_CalcPathFlag = -1;
		m_bCalcPathFlag = 1;
		mavlink_log_info(&_mavlink_log_pub, "Jetson TX2 received calculate next path");
	}

	else if(packet_id == 4)
	{
		alpha = *(float_ptr+0); //4 5 6 7
		pathidx = (int)(buf[8] << 24) | (int)(buf[9] << 16) | (int)(buf[10] << 8) | (int)buf[11]; //8 9 10 11
		mflag = (char)buf[12]; //12
		k = (int)(buf[13] << 24) | (int)(buf[14] << 16) | (int)(buf[15] << 8) | (int)buf[16]; //13 14 15 16
		l = (int)(buf[17] << 24) | (int)(buf[18] << 16) | (int)(buf[19] << 8) | (int)buf[20]; //17 18 19 20
		isLastPoint = (uint8_t)buf[21];
		isValidPoint = (bool)buf[22];

		struct avoidance_path_s report = {};
		report.timestamp = hrt_absolute_time();

		report.pathidx = pathidx;
		report.mflag = mflag;
		report.k = k;
		report.l = l;
		report.alpha = alpha;
		report.isLastPoint = isLastPoint;
		report.isValidPoint = isValidPoint;

		if (_serial_com_topic != nullptr)
		{

			orb_publish(ORB_ID(avoidance_path), _serial_com_topic, &report);
			mavlink_log_info(&_mavlink_log_pub, "calculate path complete");
			printf("receive path\n");
			printf("subscribe path\n");
			printf("path index = %d\n",report.pathidx);
			printf("mflag = %d\n",(int)report.mflag);
			printf("k = %d\n",report.k);
			printf("l = %d\n",report.l);
			printf("isLastPoint = %d\n",(int)report.isLastPoint);
			printf("isValidPoint = %d\n",(int)report.isValidPoint);
		}

		else
		{
			_serial_com_topic = orb_advertise_multi(ORB_ID(avoidance_path), &report, &_orb_class_instance, ORB_PRIO_MAX);

			if (_serial_com_topic == nullptr)
			{
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	} else if(packet_id == 5) {
		m_bSendStopFlag = 1;
		m_bReceivedStopACK = 1;
	}
	return packet_id;
}

unsigned int SERIAL_COM::send_mission_info_packet(void)
{
	int ret = 0;

	build_mission_info_packet();
	// Send packet
	ret = ::write(_fd, &_InterfaceTxBuffer, _HilsTx_FrameLength);

	return ret;
}

unsigned int SERIAL_COM::send_mission_flag_packet(uint8_t mission_flag_cmd)
{
	int ret = 0;

	build_mission_flag_packet(mission_flag_cmd);
	// Send packet
	ret = ::write(_fd, &_InterfaceTxBuffer, _HilsTx_FrameLength);

	return ret;
}

unsigned int SERIAL_COM::send_stop_packet(uint8_t stop_flag_cmd)
{
	int ret = 0;

	//Total  7 bytes

	int i;
	unsigned short crc = 0x0;
	unsigned short packet_len = 0;

	_InterfaceTxBuffer[packet_len++] = 0x7E; //0
	_InterfaceTxBuffer[packet_len++] = 0x81; //1

	// Data length
	packet_len = TxUint8(packet_len, 0, &_InterfaceTxBuffer[packet_len]); //2

	// Data ID
	packet_len = TxUint8(packet_len, 4, &_InterfaceTxBuffer[packet_len]); //3

	// Data
	packet_len = TxUint8(packet_len, stop_flag_cmd, &_InterfaceTxBuffer[packet_len]); //4


	_InterfaceTxBuffer[2] = packet_len+2;

	for ( i = 2 ; i < packet_len; i++)
		crc = UpdateCRC(crc, _InterfaceTxBuffer[i]);

	// Add CRC bytes
	_InterfaceTxBuffer[packet_len++] = (crc>>8) & 0xFF; //5
	_InterfaceTxBuffer[packet_len++] = crc & 0xFF; //6

	// Update the frame length
	_HilsTx_FrameLength = packet_len;
	// Send packet
	ret = ::write(_fd, &_InterfaceTxBuffer, _HilsTx_FrameLength);

	return ret;
}

unsigned int SERIAL_COM::send_calculate_path(uint8_t command_calc_path)
{
	int ret = 0;
	int i;
	unsigned short crc = 0x0;
	unsigned short packet_len = 0;

	_InterfaceTxBuffer[packet_len++] = 0x7E; //0
	_InterfaceTxBuffer[packet_len++] = 0x81; //1

	// Data length
	packet_len = TxUint8(packet_len, 0, &_InterfaceTxBuffer[packet_len]); //2

	// Data ID
	packet_len = TxUint8(packet_len, 3, &_InterfaceTxBuffer[packet_len]); //3

	// Data
	packet_len = TxUint8(packet_len, command_calc_path, &_InterfaceTxBuffer[packet_len]); //4


	_InterfaceTxBuffer[2] = packet_len+2;

	for ( i = 2 ; i < packet_len; i++)
		crc = UpdateCRC(crc, _InterfaceTxBuffer[i]);

	// Add CRC bytes
	_InterfaceTxBuffer[packet_len++] = (crc>>8) & 0xFF; //5
	_InterfaceTxBuffer[packet_len++] = crc & 0xFF; //6

	// Update the frame length
	_HilsTx_FrameLength = packet_len;
	// Send packet
	ret = ::write(_fd, &_InterfaceTxBuffer, _HilsTx_FrameLength);

	return ret;
}


unsigned int SERIAL_COM::send_state_packet()
{
	int ret = 0;

	build_state_packet();
	// Send packet
	ret = ::write(_fd, &_InterfaceTxBuffer, _HilsTx_FrameLength);

	return ret;
}

void SERIAL_COM::build_mission_info_packet()

{
	int i;
	unsigned short crc = 0x0;
	unsigned short packet_len = 0;

	//total 66 bytes

	_InterfaceTxBuffer[packet_len++] = 0x7E; //0
	_InterfaceTxBuffer[packet_len++] = 0x81; //1

	// Data length
	packet_len = TxUint8(packet_len, 0, &_InterfaceTxBuffer[packet_len]); //2

	// ID
	packet_len = TxUint8(packet_len, 0, &_InterfaceTxBuffer[packet_len]); //3

	// Data
	packet_len = TxFloat32(packet_len, _mission_area_NW[0], &_InterfaceTxBuffer[packet_len]); //4 5 6 7
	packet_len = TxFloat32(packet_len, _mission_area_NW[1], &_InterfaceTxBuffer[packet_len]); //8 9 10 11

	packet_len = TxFloat32(packet_len, _mission_area_NE[0], &_InterfaceTxBuffer[packet_len]); //12 13 14 15
	packet_len = TxFloat32(packet_len, _mission_area_NE[1], &_InterfaceTxBuffer[packet_len]); //16 17 18 19

	packet_len = TxFloat32(packet_len, _mission_area_SW[0], &_InterfaceTxBuffer[packet_len]); //20 21 22 23
	packet_len = TxFloat32(packet_len, _mission_area_SW[1], &_InterfaceTxBuffer[packet_len]); //24 25 26 27

	packet_len = TxFloat32(packet_len, _mission_area_SE[0], &_InterfaceTxBuffer[packet_len]); //28 29 30 31
	packet_len = TxFloat32(packet_len, _mission_area_SE[1], &_InterfaceTxBuffer[packet_len]); //32 33 34 35

	packet_len = TxFloat32(packet_len, _start_position[0], &_InterfaceTxBuffer[packet_len]); //36 37 38 39
	packet_len = TxFloat32(packet_len, _start_position[1], &_InterfaceTxBuffer[packet_len]); //40 41 42 43

	packet_len = TxFloat32(packet_len, _end_position[0], &_InterfaceTxBuffer[packet_len]); //44 45 46 47
	packet_len = TxFloat32(packet_len, _end_position[1], &_InterfaceTxBuffer[packet_len]); //48 49 50 51

	packet_len = TxFloat32(packet_len, _home_position[0], &_InterfaceTxBuffer[packet_len]); //52 53 54 55
	packet_len = TxFloat32(packet_len, _home_position[1], &_InterfaceTxBuffer[packet_len]); //56 57 58 59
	packet_len = TxFloat32(packet_len, _home_position[2], &_InterfaceTxBuffer[packet_len]); //60 61 62 63

	packet_len = TxFloat32(packet_len, _cellwidth, &_InterfaceTxBuffer[packet_len]); //64 65 66 67
	packet_len = TxUint32(packet_len, _width, &_InterfaceTxBuffer[packet_len]); //68 69 70 71
	packet_len = TxUint32(packet_len, _height, &_InterfaceTxBuffer[packet_len]); //72 73 74 75
	packet_len = TxUint32(packet_len, _thres0, &_InterfaceTxBuffer[packet_len]); //76 77 78 79
	packet_len = TxUint32(packet_len, _thresd, &_InterfaceTxBuffer[packet_len]); //80 81 82 83

	_InterfaceTxBuffer[2] = packet_len+2;

	for ( i = 2 ; i < packet_len; i++)
		crc = UpdateCRC(crc, _InterfaceTxBuffer[i]);

	// Add CRC bytes
	_InterfaceTxBuffer[packet_len++] = (crc>>8) & 0xFF; //84
	_InterfaceTxBuffer[packet_len++] = crc & 0xFF; //85

	// Update the frame length
	_HilsTx_FrameLength = packet_len;


}

void SERIAL_COM::build_mission_flag_packet(uint8_t mission_flag_cmd)
{
	//Total  7 bytes

	int i;
	unsigned short crc = 0x0;
	unsigned short packet_len = 0;

	_InterfaceTxBuffer[packet_len++] = 0x7E; //0
	_InterfaceTxBuffer[packet_len++] = 0x81; //1

	// Data length
	packet_len = TxUint8(packet_len, 0, &_InterfaceTxBuffer[packet_len]); //2

	// Data ID
	packet_len = TxUint8(packet_len, 1, &_InterfaceTxBuffer[packet_len]); //3

	// Data
	packet_len = TxUint8(packet_len, mission_flag_cmd, &_InterfaceTxBuffer[packet_len]); //4


	_InterfaceTxBuffer[2] = packet_len+2;

	for ( i = 2 ; i < packet_len; i++)
		crc = UpdateCRC(crc, _InterfaceTxBuffer[i]);

	// Add CRC bytes
	_InterfaceTxBuffer[packet_len++] = (crc>>8) & 0xFF; //5
	_InterfaceTxBuffer[packet_len++] = crc & 0xFF; //6

	// Update the frame length
	_HilsTx_FrameLength = packet_len;


}

void SERIAL_COM::build_state_packet()
{
	//Total 30 bytes
	int i;
	unsigned short crc = 0x0;
	unsigned short packet_len = 0;


	_InterfaceTxBuffer[packet_len++] = 0x7E; //0
	_InterfaceTxBuffer[packet_len++] = 0x81; //1

	// Data length
	packet_len = TxUint8(packet_len, 0, &_InterfaceTxBuffer[packet_len]); //2

	// Data ID
	packet_len = TxUint8(packet_len, 2, &_InterfaceTxBuffer[packet_len]); //3

	// Data
	packet_len = TxFloat32(packet_len, _mc_lat, &_InterfaceTxBuffer[packet_len]); //4 5 6 7
	packet_len = TxFloat32(packet_len, _mc_lon, &_InterfaceTxBuffer[packet_len]); //8 9 10 11
	packet_len = TxFloat32(packet_len, _mc_alt, &_InterfaceTxBuffer[packet_len]); //12 13 14 15
	packet_len = TxFloat32(packet_len, _mc_yaw, &_InterfaceTxBuffer[packet_len]); //16 17 18 19

	_InterfaceTxBuffer[2] = packet_len+2;

	for ( i = 2 ; i < packet_len; i++)
		crc = UpdateCRC(crc, _InterfaceTxBuffer[i]);

	// Add CRC bytes
	_InterfaceTxBuffer[packet_len++] = (crc>>8) & 0xFF; //28
	_InterfaceTxBuffer[packet_len++] = crc & 0xFF; //29

	// Update the frame length
	_HilsTx_FrameLength = packet_len;


}


int
SERIAL_COM::collect()
{
	int ret;
	int i;
	unsigned int 	crc_index;
	//unsigned int 	muxidx;

	perf_begin(_sample_perf);

	// the buffer for read chars is buflen minus null termination
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	// read from the Hils (uart buffer)
	ret = ::read(_fd , &readbuf[0], readlen);

	//warnx("\n a = %d, b = %d", readlen,ret);//warnx("\n %d", 1);

	bool valid = true;

	if(ret > 0)
	{
		for (i = 0; i < ret; i++)
		{
			// Put the read binary data into packet
			_InterfaceRxBuffer[_InterfaceRxBuf_EndPtr++] = readbuf[i];
			_InterfaceRxBuf_EndPtr %= INTERFACE_RXBUF_SIZE;

			// Get hils packet
			while (_InterfaceRxBuf_EndPtr != _InterfaceRxBuf_StartPtr)
			{
				if (crc_flag_Hils == 0)
				{
					frame_sync_Hils = (frame_sync_Hils << 8) | _InterfaceRxBuffer[_InterfaceRxBuf_StartPtr++];
					_InterfaceRxBuf_StartPtr %= INTERFACE_RXBUF_SIZE;
					if (frame_sync_Hils == 0x7E81)
					{
						p_gse = _Interface_packet_buf + 2;
						hils_pktlen = 2;
						crc_Hils = 0;
						crc_flag_Hils = 1; // Ready for CRC test
					}
				}
				else
				{
					*p_gse = _InterfaceRxBuffer[_InterfaceRxBuf_StartPtr++];
					_InterfaceRxBuf_StartPtr %= INTERFACE_RXBUF_SIZE;

					crc_index = (crc_Hils>>8)^(*p_gse++);
					crc_Hils = (crc_Hils<<8)^crc_table[crc_index & 0x00FF];

					if ((++hils_pktlen) >= _Interface_packet_buf[2])
					{
						if ((crc_Hils&0xFFFF) == 0)		// Success on receiving a complete packet
						{
							p_gse = _Interface_packet_buf + 2;
							_Interface_packet_buf[0] = 0x7E;
							_Interface_packet_buf[1] = 0x81;
							Interface_packet_parser(&_Interface_packet_buf[0]);
							//printf("receieved = %d\n", 000);
							valid = true;
						}
						crc_flag_Hils = 0; // Ready for search 0x7E81
						hils_pktlen = 0;
						frame_sync_Hils = 0;
					}
				}
			}
		}

	}

	perf_end(_sample_perf);

	if (!valid) {
		return -EAGAIN;
	}
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

	if(_vehicle_control_mode_sub < 0)
	{
		_vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
		orb_set_interval(_vehicle_control_mode_sub, 200);
	}
	if(_armed_sub < 0)
	{
		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		orb_set_interval(_armed_sub, 200);
	}

	if (vehicle_attitude_fd < 0)
	{
		/* New subscription */// vehicle_attitude_fd;vehicle_global_position_fd
		vehicle_attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
		orb_set_interval(vehicle_attitude_fd, 200);
	}
	if(vehicle_global_position_fd < 0)
	{
		vehicle_global_position_fd = orb_subscribe(ORB_ID(vehicle_global_position));
		orb_set_interval(vehicle_global_position_fd, 200);
	}

	if(_Mission_info_sub < 0) {
		_Mission_info_sub= orb_subscribe(ORB_ID(mission_info));
	}

	if(_home_pos_sub < 0) {
		_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	}

	if(_input_rc_sub < 0) {
		_input_rc_sub = orb_subscribe(ORB_ID(input_rc));
	}

	if(_calc_next_path_sub < 0) {
		_calc_next_path_sub = orb_subscribe(ORB_ID(calculate_next_path));
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
void	interface_test();

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
	fd = open("/dev/serial_com0", 0);

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
			PX4_ERR("Got no data within a second");

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


/*
	for (i = 0; i < 1000; i++)
	{
		orb_check(actuator_outputs_sub, &updated);

		if (updated) {
		orb_copy(ORB_ID(actuator_outputs), actuator_outputs_sub, &actuators);

			PX4_WARN("PWM:\t%f\t%f\t%f\t%f",
					(double)actuators.output[0],
					(double)actuators.output[1],
					(double)actuators.output[2],
					(double)actuators.output[3]);

		}
	}

	PX4_INFO("exiting");
*/
}
void interface_test()
{/*
	int sz;
	int fd = open("/dev/ttyS6", O_WRONLY);

	g_dev-> _mc_lat = 37.5f;
	g_dev-> _mc_lon = 126.5f;
	g_dev-> _mc_roll = 1.1f;
	g_dev-> _mc_pitch = 2.1f;
	g_dev-> _mc_yaw = 3.1f;
	g_dev->build_state_packet(g_dev-> _mc_lat, g_dev-> _mc_lon, g_dev-> _mc_roll, g_dev-> _mc_pitch, g_dev-> _mc_yaw);

	sz = write(fd, &(g_dev->_InterfaceTxBuffer), g_dev->_HilsTx_FrameLength);

	printf("%d\n", sz);
	exit(0);
	*/
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
reset()
{
	int fd = open("/dev/serial_com0", O_RDONLY);

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
	 * Test interface jetson and pixhawk
	 */
	if(!strcmp(argv[1], "interface")) {
		serial_com::interface_test();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		serial_com::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info' or 'logon' or 'logoff' or 'test2'");
}
