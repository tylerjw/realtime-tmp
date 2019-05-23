#include "controller.h"
#include <cassert>
#include <cmath>

void Controller::set_mode(Mode mode) {
    std::vector<Mode> modes(n_motors_, mode);
    set_mode(modes);
}

void Controller::set_mode(std::vector<Mode> modes) {
    std::lock_guard<std::mutex> guard(lock_);
    mode_desired_ = modes;
}

void Controller::set_position(double position) {
    std::vector<double> positions(n_motors_, position);
    set_position(positions);
}

void Controller::set_position(std::vector<double> position) {
    std::lock_guard<std::mutex> guard(lock_);
    position_desired_ = position;
}

void Controller::set_current(double current) {
    std::vector<double> currents(n_motors_, current);
    set_current(currents);
}

void Controller::set_current(std::vector<double> current) {
    std::lock_guard<std::mutex> guard(lock_);
    current_desired_ = current;
}

std::vector<Command> Controller::build_commands(
    std::vector<Status> statuses) {
    assert(statuses.size() <= n_motors_);
    std::lock_guard<std::mutex> guard(lock_);
    std::vector<Command> commands(statuses.size());
    for (int i=0; i<statuses.size(); i++) {
        commands[i].count = command_count_;

        switch (mode_desired_[i]) {
            case OPEN:
            default:
                commands[i].mode = 0;
                break;
            case BRAKE:
                commands[i].mode = 1;
                break;
            case CURRENT:
                commands[i].mode = 2;
                commands[i].current_desired = current_desired_[i];
                break;
            case POSITION:
                commands[i].mode = 2;
                commands[i].current_desired = current_desired_[i]
                    + position_controllers_[i].update(
                        position_desired_[i],
                        statuses[i].position_measured,
                        statuses[i].count);
                break;
        }
    }
    command_count_++;
    return commands;
}

double PositionController::update(double position_desired,
    double position_measured, int32_t count) {
    double dt = (count - last_count_)/180e6;
    double velocity_measured = (position_measured - position_last_)/dt;
    position_last_ = position_measured;
    last_count_ = count;

    double current_desired =
        kp*(position_desired - position_measured) - kd*velocity_measured;
    return current_desired;
}