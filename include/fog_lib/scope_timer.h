// clang: MatousFormat
/**  \file
     \brief Defines the scope_timer helper class.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SCOPE_TIMER_H
#define SCOPE_TIMER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <memory>

namespace fog_lib
{

  /**
   * \brief A helper class that prints its scope duration if it's longer than a threshold.
   */
  class scope_timer
  {
    public:
      /*!
       * \brief Default constructor.
       *
       * \param enable if false, nothing will be printed.
       * \param label printed at the start of each message.
       * \param logger used for rclcpp printing.
       * \param min_dur if the `scope_timer`'s life is shorter than this duration, nothing is printed.
       * \param throttle minimal period between successive messages.
       * \param clock_ptr used for rclcpp clock- and time-related stuff.
       */
      scope_timer(
          const bool enable, // whether to print anything - if false, nothing will be done
          const std::string& label, // label of this timer used to uniquely identify it and for printing
          const rclcpp::Logger& logger,
          const rclcpp::Duration& min_dur = {0, 0}, // shorter durations will be ignored
          const rclcpp::Duration& throttle = {0, 0}, // prints will not be output with a shorter period than this
          rclcpp::Clock::SharedPtr clock_ptr = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME) // which clock to use
        );

      ~scope_timer();

    private:
      const bool enable_;
      const std::string label_;
      const rclcpp::Logger logger_;
      const rclcpp::Duration throttle_;
      const rclcpp::Duration min_dur_;
      const rclcpp::Time start_time_;
      const rclcpp::Clock::SharedPtr clock_ptr_;
  };

}

#endif // SCOPE_TIMER_H
