
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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/serial_com.h>

#include <board_config.h>

#include "serial_com_parser.h"


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define SERIAL_COM_CONVERSION_INTERVAL	10000 /* SF02 (40m, 12 Hz)*/ //1,000,000 simulink 100hz
#define SERIAL_COM_TAKE_RANGE_REG		'd'

// designated SERIAL4/5 on Pixhawk
#define SERIAL_COM_DEFAULT_PORT		"/dev/ttyS2"

class SERIAL_COM : public device::CDev
{
public:
	SERIAL_COM(const char *port = SERIAL_COM_DEFAULT_PORT);
	virtual ~SERIAL_COM();

	virtual int 			init();
	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int				ioctl(struct file *filp, int cmd, unsigned long arg);
	void				print_info();

protected:
	virtual int			probe();

private:
	char 							_port[20];
	work_s							_work;
	ringbuffer::RingBuffer			*_reports;
	bool							_sensor_ok;
	int								_measure_ticks;
	bool							_collect_phase;
	int								_fd;
	char							_linebuf[10];
	unsigned						_linebuf_index;
	enum SERIAL_COM_PARSE_STATE		_parse_state;
	hrt_abstime						_last_read;

	int								_class_instance;
	int								_orb_class_instance;

	orb_advert_t					_serial_com_topic;

	unsigned						_consecutive_fail_count;

	perf_counter_t					_sample_perf;
	perf_counter_t					_comms_errors;
	perf_counter_t					_buffer_overflows;

	void				start();
	void				stop();
	void				cycle();
	int					measure();
	int					collect();
	static void			cycle_trampoline(void *arg);

};

extern "C" __EXPORT int serial_com_main(int argc, char *argv[]);

SERIAL_COM::SERIAL_COM(const char *port) :
	CDev("SERIAL_COM", "/dev/serial_com"),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
	_parse_state(SERIAL_COM_PARSE_STATE0_UNSYNC),
	_last_read(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_serial_com_topic(nullptr),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "serial_com_read")),
	_comms_errors(perf_alloc(PC_COUNT, "serial_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "serial_com_buf_of"))
{
	/* store port name */
	strncpy(_port, SERIAL_COM_DEFAULT_PORT, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

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

	unsigned speed = B57600;//b9600

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
}

SERIAL_COM::~SERIAL_COM()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname("/dev/serial_com", _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
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

		/* get a publish handle on the range finder topic */
		struct serial_com_s ds_report = {};

		_serial_com_topic = orb_advertise_multi(ORB_ID(serial_com), &ds_report, &_orb_class_instance, ORB_PRIO_HIGH);

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

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

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
	warnx("SERIAL_COM::READ _measure_ticks : %d", _measure_ticks); // 100
	warnx("SERIAL_COM::READ count : %d", count); // 1
	warnx("SERIAL_COM::READ sizeof(struct serial_com_s) : %d",sizeof(struct serial_com_s)); //16
	warnx("SERIAL_COM::READ sizeof(rbuf) : %d", sizeof(rbuf)); //4
	warnx("SERIAL_COM::READ buflen : %d",buflen); // 16

	if (_measure_ticks > 0)
	{
			 // While there is space in the caller's buffer, and reports, copy them.
			 // Note that we may be pre-empted by the workq thread while we are doing this;
			 // we are careful to avoid racing with them.

		warnx("SERIAL_COM::READ if _measure_ticks > 0 : %d", _measure_ticks); // 10

			while (count--)
			{
				warnx("SERIAL_COM::READ while start");

				if (_reports->get(rbuf))
				{
					warnx("SERIAL_COM get(rbuf) : %d" , sizeof(*rbuf));

					ret += sizeof(*rbuf);

					warnx("SERIAL_COM::READ ret : %d", ret);
					warnx("SERIAL_COM::READ rbuf: %d", sizeof(*rbuf));

					rbuf++;
				}
			}
			// if there was no data, warn the caller
			warnx("SERIAL_COM::read while out ret : %d", ret);
			return ret ? ret : -EAGAIN; // try again
	}

	/* manual measurement - run one conversion */
	do {

		warnx("read do start");

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
	/*
	int ret;
	//Send the command to begin a measurement.


	char cmd = SERIAL_COM_TAKE_RANGE_REG;
	ret = ::write(_fd, &cmd, 1);

	if (ret != sizeof(cmd))
	{
		perf_count(_comms_errors);
		DEVICE_LOG("write fail %d", ret);
		return ret;
	}
	ret = OK;
	*/
	return 0;
}

int
SERIAL_COM::collect()
{
	int	ret;

	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	uint64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	//ret = ::read(_fd, &readbuf[0], readlen);
	ret = ::read(3 , &readbuf[0], 9);


	warnx("collect return : %d, %d, %d", ret, _fd, readlen); // -1 3 9

	if (ret < 0)
	{
		DEVICE_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		//warnx("collect return loop : %d, %d", ret, readlen);
		/* only throw an error if we time out */
		if (read_elapsed > (SERIAL_COM_CONVERSION_INTERVAL * 2))
		{
			return ret;
		}
		else
		{
			return -EAGAIN;
		}


		warnx("collect return if loop : %d, %d", ret, readlen); //  -1  0/161  9


	}

	else if (ret == 0)
	{
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();

	float distance_m = -1.0f;
	bool valid = false;

	for (int i = 0; i < ret; i++)
	{
		if (OK == serial_com_parser(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m))
		{
			valid = true;
		}
	}

	warnx("collect ret<-OK : %d", OK);

	if (!valid) {
		return -EAGAIN;
	}

	DEVICE_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((valid) ? "OK" : "NO"));

	struct serial_com_s report;
	report.timestamp = hrt_absolute_time();
	/*
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = 8;
	report.current_distance = 0.684f;//distance_m;
	report.min_distance = 0.3f;
	report.max_distance = 40.0f;
	report.covariance = 0.0f;
	report.id = 0;
	*/
	report.distance_test = distance_m;
	/* publish it */
	orb_publish(ORB_ID(serial_com), _serial_com_topic, &report);

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
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
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&SERIAL_COM::cycle_trampoline,
				   this,
				   USEC2TICK(1042 * 8));
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
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&SERIAL_COM::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL));

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
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&SERIAL_COM::cycle_trampoline,
		   this,
		   USEC2TICK(SERIAL_COM_CONVERSION_INTERVAL));
}

void
SERIAL_COM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
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
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *port)
{
	int fd;

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
	fd = open("/dev/serial_com", 0);

	if (fd < 0) {
		warnx("device open fail");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
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

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct serial_com_s report;
	ssize_t sz;

	int fd = open("/dev/serial_com", O_RDONLY);//fd == 3

	if (fd < 0)	err(1, "%s open failed (try 'serial_com start' if the driver is not running", "/dev/serial_com");

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report)); //sz == -1

	if (sz != sizeof(report)) {  // size of report == 16
		//err(1, "immediate read failed");
		err(1, "immediate read failed %d,%d,%d",fd,sz,sizeof(report));		//3  -1  16
		//err(1, "%s,%d","/dev/serial_com",fd);
	}

	warnx("single read");
	warnx("measurement:  %0.2f m", (double)report.distance_test);

	/* start the sensor polling at 2 Hz rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out");
			break;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warnx("read failed: got %d vs exp. %d", sz, sizeof(report));
			break;
		}

		warnx("read #%u", i);
		warnx("measurement:  %0.3f m", (double)report.distance_test);
	}

	/* reset the sensor polling to the default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "ERR: DEF RATE");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open("/dev/serial_com", O_RDONLY);

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

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		serial_com::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		serial_com::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
