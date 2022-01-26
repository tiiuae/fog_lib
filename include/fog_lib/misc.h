#ifndef MISC_H
#define MISC_H

#include <string>

namespace fog_lib
{
  /*!
   * \brief Appends the `reason` string to `to_str` if `condition` is met.
   *
   * \param reason the string to add to `to_str`.
   * \param condition if true, the `reason` will be added to `to_str`.
   * \param to_str the input-output string that will be modified only if `condition` is met.
   */
  void add_reason_if(const std::string& reason, const bool condition, std::string& to_str);
}

#endif // MISC_H
