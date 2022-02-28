#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

/* author: Daniel Hert */

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

class MedianFilter {

public:
  MedianFilter(int buffer_size, double max_valid_value, double min_valid_value, double max_difference, rclcpp::Node& node);
  bool   isValid(double input, rclcpp::Node& node);
  bool   isFilled();
  double getMedian();

private:
  std::vector<double> buffer;
  int                 buffer_size;
  int                 next;
  double              max_valid_value;
  double              min_valid_value;
  double              max_difference;
  bool                is_filled;
  double              m_median;
  std::mutex          mutex_median;
};

#endif
