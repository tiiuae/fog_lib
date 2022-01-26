#include <fog_lib/scope_timer.h>

namespace fog_lib
{

  scope_timer::scope_timer(
        const bool enable,
        const std::string& label,
        const rclcpp::Logger& logger,
        const rclcpp::Duration& min_dur,
        const rclcpp::Duration& throttle,
        rclcpp::Clock::SharedPtr clock_ptr
      )
      : enable_(enable), label_(label), logger_(logger), throttle_(throttle), min_dur_(min_dur), start_time_(clock_ptr->now()), clock_ptr_(clock_ptr)
  {}

  scope_timer::~scope_timer()
  {
    if (!enable_)
      return;

    const rclcpp::Time end_time = clock_ptr_->now();
    const rclcpp::Duration dur = end_time - start_time_;
    // ignore shorter durations than min_dur_
    if (dur < min_dur_)
      return;

    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_ptr_, throttle_.seconds()*1000, label_ << " took " << dur.seconds() << "s");
  }

}
