#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "motor.h"
#include <vector>
#include <mutex>

class PositionController {
 public:
    double update(double position_desired,
        double position_measured, int32_t count);
 private:
    int32_t last_count_;
    double position_last_;

    double kp = .5;
    double kd = .005;
};

class Controller {
 public:
    Controller(int n_motors)
        : n_motors_(n_motors)
        , position_controllers_(n_motors)
        , position_desired_(n_motors, 0)
        , current_desired_(n_motors, 0)
        , mode_desired_(n_motors, OPEN)
        , command_count_(0)
    {}
    enum Mode{OPEN, BRAKE, CURRENT, POSITION};
    void set_mode(Mode);
    void set_mode(std::vector<Mode> modes);
    std::vector<Mode> get_mode() const { return mode_desired_; }
    void set_position(double position);
    void set_position(std::vector<double> position);
    void set_current(double current);
    void set_current(std::vector<double> current);
    std::vector<Command> build_commands(std::vector<Status> statuses);
 private:
    std::vector<double> position_desired_;
    std::vector<double> current_desired_;
    std::vector<Mode> mode_desired_;
    std::vector<PositionController> position_controllers_;
    int n_motors_;
    std::mutex lock_;
    uint32_t command_count_;
};

#endif
