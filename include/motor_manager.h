#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
#include <string>
#include <ostream>
class Motor;

#include "motor.h"

class MotorManager {
 public:
    std::vector<std::shared_ptr<Motor>> get_connected_motors();
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
    std::vector<Command> commands() const { return commands_; }
    void open();
    std::vector<Status> read();
    void write(std::vector<Command>);
    void write_saved_commands();
    void aread();
    void close();

    void set_commands(std::vector<Command> commands);
    void set_command_count(int32_t count);
    void set_command_mode(uint8_t mode);
    void set_command_current(std::vector<float> current);
    void set_command_position(std::vector<float> position);
 private:
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<Command> commands_;
};

inline std::ostream& operator<<(std::ostream& os, const std::vector<Command> command)
{
   for (auto c : command) {
      os << c.count << ", ";
   }
   for (auto c : command) {
      os << +c.mode << ", ";
   }
   for (auto c : command) {
      os << c.current_desired << ", ";
   }
   for (auto c : command) {
      os << c.position_desired << ", ";
   }

   return os;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<Status> status)
{
   for (auto s : status) {
      os << s.count << ", ";
   }
   for (auto s : status) {
      os << s.count_received << ", ";
   }
   for (auto s : status) {
      os << s.current_measured << ", ";
   }
   for (auto s : status) {
      os << s.position_measured << ", ";
   }
   for (auto s : status) {
      os << s.res[0] << ", ";
   }
    return os;
}

#endif
