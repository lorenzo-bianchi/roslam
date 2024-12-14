/**
 * Joystick module auxiliary routines.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * October 26, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <joystick/joystick.hpp>

namespace joystick
{

/**
 * @brief Thread routine to parse joystick values from device file
 */
void JoystickNode::joy_routine()
{
  RCLCPP_INFO(this->get_logger(), "Trying to connect to joystick...");
  joy_msg.connected = false;
  while (!stop_thread.load(std::memory_order_acquire))
  {
    js_ = open(joy_path_.c_str(), O_RDONLY);

    // Wait for 1 second and then try to reconnect
    if (js_ == -1)
    {
      std::this_thread::sleep_for(1s);
      continue;
    }

    // Joystick connected
    RCLCPP_INFO(this->get_logger(), "Joystick connected on device file %s", joy_path_.c_str());
    joy_msg.connected = true;
    while (true)
    {
      if (stop_thread.load(std::memory_order_acquire)) return;

      int re = read_event(js_, &event_);
      if (re == 1) continue;      // Timeout
      else if (re == -1) break;   // Error

      switch (event_.type)
      {
        case JS_EVENT_INIT + 1:
          // *buttons[event_.number] = (uint8_t) event_.value;
          break;
        case JS_EVENT_INIT + 2:
          *axes[event_.number] = event_.value / axis_max_val_;
          break;
        case JS_EVENT_BUTTON:
          // *buttons[event_.number] = (uint8_t) event_.value;
          break;
        case JS_EVENT_AXIS:
          switch (event_.number)
          {
            case LX: case RX:
              *axes[event_.number] = abs(event_.value) > axis_deadzone_val_ ?
                event_.value / axis_max_val_ : 0;
              break;
            case LY: case RY:
              *axes[event_.number] = abs(event_.value) > axis_deadzone_val_ ?
                event_.value / axis_max_val_ : 0;
              break;
            case DY:
              *axes[event_.number] = event_.value / axis_max_val_;
              break;
            default:
              *axes[event_.number] = event_.value / axis_max_val_;
              break;
          }

          break;
      }
      joy_pub_->publish(joy_msg);
      fflush(stdout);
    }
    RCLCPP_ERROR(this->get_logger(), "Joystick disconnected, waiting for reconnection...");
    joy_msg.connected = false;
    joy_pub_->publish(joy_msg);
  }
}

/**
 * @brief Reads a joystick event from the joystick device.
 *
 * @return 0 on success, -1 otherwise.
 */
int JoystickNode::read_event(int fd, struct js_event *event)
{
  fd_set set;
  FD_ZERO(&set); /* clear the set */
  FD_SET(fd, &set); /* add our file descriptor to the set */

  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  int rv;
  rv = select(fd + 1, &set, NULL, NULL, &timeout);
  if (rv == -1) return -1;     // Error
  else if (rv == 0) return 1;  // Timeout

  ssize_t bytes;
  bytes = read(fd, event, sizeof(*event));

  if (bytes == sizeof(*event))
      return 0;

  /* Error, could not read full event. */
  return -1;
}

} // namespace joystick
