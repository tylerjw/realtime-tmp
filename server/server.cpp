#define _GNU_SOURCE
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>

#include <thread>
#include <iostream>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <errno.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string>
#include <vector>
#include <algorithm>
#include "motor.h"
#include "motor_manager.h"
#include <fstream>
#include "controller.h"

#include "messages.pb.h"

#define gettid() syscall(__NR_gettid)

#define SCHED_DEADLINE	6

/* XXX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr		314
#define __NR_sched_getattr		315
#endif

#ifdef __i386__
#define __NR_sched_setattr		351
#define __NR_sched_getattr		352
#endif

#ifdef __arm__
#define __NR_sched_setattr		380
#define __NR_sched_getattr		381
#endif

static volatile int done;

struct sched_attr {
	__u32 size;

	__u32 sched_policy;
	__u64 sched_flags;

	/* SCHED_NORMAL, SCHED_BATCH */
	__s32 sched_nice;

	/* SCHED_FIFO, SCHED_RR */
	__u32 sched_priority;

	/* SCHED_DEADLINE (nsec) */
	__u64 sched_runtime;
	__u64 sched_deadline;
	__u64 sched_period;
};

int sched_setattr(pid_t pid,
		const struct sched_attr *attr,
		unsigned int flags)
{
return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid,
		struct sched_attr *attr,
		unsigned int size,
		unsigned int flags)
{
return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

struct Data {
	std::vector<Status> statuses;
	std::vector<Command> commands;
	std::vector<int32_t> delay;
	std::chrono::steady_clock::time_point time_start, last_time_start,
		last_time_end, aread_time, read_time, control_time, write_time;
};

template <class T>
class CStack {
 public:
	void push(T &t) {
		int future_pos = pos_ + 1;
		if (future_pos >= 100) {
			future_pos = 0;
		}
		data_[future_pos] = t;
		pos_ = future_pos;
	}
	T top() {
		return data_[pos_];
	}
 private:
	T data_[100];
	int pos_ = 0;
};

class MotorDeadlineThread {
 public:
	MotorDeadlineThread(
		CStack<Data> &cstack,
		int n_motors,
		MotorManager &motor_manager,
		Controller &controller)
		: cstack_(cstack)
		, n_motors_(n_motors)
		, motor_manager_(motor_manager)
		, controller_(controller)
	{
		motor_manager_.open();
		std::cout << "Connecting to motors:" << std::endl;
		for (auto m : motor_manager_.motors()) {
			std:: cout << m->name() << std::endl;
		}
		data_.commands.resize(n_motors_);
		data_.statuses.resize(n_motors_);
		data_.delay.resize(n_motors_);
	}
	~MotorDeadlineThread() {
		motor_manager_.close();
	}

	Controller& get_controller() { return controller_; }

	void run() { done_ = 0;
		start_time_ = std::chrono::steady_clock::now();
		next_time_ = start_time_;
		thread_ = new std::thread([=]{run_realtime_server();}); }
	void done() { controller_.set_mode(Controller::OPEN);
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		done_ = 1; }
	void join() { thread_->join(); }
 private:
	void run_realtime_server()
	{
		struct sched_attr attr;
		int ret;
		unsigned int flags = 0;

		printf("deadline thread started [%ld]\n", gettid());

		attr.size = sizeof(attr);
		attr.sched_flags = 0;
		attr.sched_nice = 0;
		attr.sched_priority = 0;

		attr.sched_policy = SCHED_DEADLINE;
		attr.sched_runtime =  300 * 1000;
		attr.sched_deadline = period_ns_*3/5;
		attr.sched_period =  period_ns_;

		int not_root = 0;
		ret = sched_setattr(0, &attr, flags);
		if (ret < 0) {
			perror("sched_setattr");
			not_root = 1;
		}

		while (!done_) {
			next_time_ += std::chrono::nanoseconds(period_ns_);
			data_.last_time_start = data_.time_start;
			data_.time_start = std::chrono::steady_clock::now();

			// start a read on all motors
			motor_manager_.aread();
			data_.aread_time = std::chrono::steady_clock::now();

			// blocking io to get the data alread set up and wait
			// if not ready yet
			data_.statuses = motor_manager_.read();
			data_.read_time = std::chrono::steady_clock::now();

			// Generate the commands to send to the motor
			// this is where the feedback control for the motor is
			data_.commands = controller_.build_commands(data_.statuses);
			data_.control_time = std::chrono::steady_clock::now();

			// writes the commands in the motor buffer
			motor_manager_.write(data_.commands);
			data_.write_time = std::chrono::steady_clock::now();

			// push the data onto the stack
			cstack_.push(data_);
			data_.last_time_end = std::chrono::steady_clock::now();

			if(not_root) {
				std::this_thread::sleep_until(next_time_);
			} else {
				sched_yield();
			}
		}

		printf("deadline thread dies [%ld]\n", gettid());
	}
	std::thread *thread_;
	int done_;
	CStack<Data> &cstack_;
	Data data_;

	std::chrono::steady_clock::time_point start_time_, next_time_;
	long period_ns_ =   500 * 1000;
	int fid_;
	int fid_flags_;
	int n_motors_;
	MotorManager &motor_manager_;
	Controller &controller_;
};


#include <csignal>
sig_atomic_t volatile running = 1;

int main (int argc, char **argv)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION; // verify protbuf version

	MotorManager motor_manager;
	motor_manager.get_connected_motors();
	int n_motors = motor_manager.motors().size();
	Controller controller(n_motors);

	printf("main thread [%ld]\n", gettid());
	CStack<Data> cstack;
	MotorDeadlineThread task(cstack, n_motors, motor_manager, controller);
	task.run();
	std::chrono::steady_clock::time_point system_start =
		std::chrono::steady_clock::now();
	auto last_step = system_start;
	double direction = 1;
	uint32_t count = 0;

	controller.set_current(0);
	controller.set_mode(Controller::POSITION);
	controller.set_position(0);

	signal(SIGINT, [] (int signum) {running = 0;});

	while (true) {
		if (!running) {
			break;
		}

		// TODO: this should block if there isn't a new data object
		Data data = cstack.top();

		// TODO: publish protobuf status from here

		if (data.statuses.size() > 0 && count != data.statuses[0].count) {
			count = data.statuses[0].count;
			std::cout << data.time_start.time_since_epoch().count()
				<< ", " << data.commands << data.statuses << std::endl;

			double position_measured = data.statuses[0].position_measured;

			auto period_ctr = std::chrono::duration_cast<std::chrono::seconds>(
				data.time_start - last_step).count();

			if (period_ctr > 1) {
				controller.set_mode(Controller::POSITION);
				controller.set_position(position_measured + M_PI*10*direction);
				direction = (direction > 0) ? -1 : 1;
				last_step = std::chrono::steady_clock::now();
			}
		}

		// TODO: remove this
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	task.done();
	task.join();

	google::protobuf::ShutdownProtobufLibrary();

	printf("main dies [%ld]\n", gettid());
	return 0;
}
