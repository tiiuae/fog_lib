#include <fog_lib/misc.h>

namespace fog_lib
{
  void add_reason_if(const std::string& reason, const bool condition, std::string& to_str)
  {
    if (condition)
    {
      if (to_str.empty())
        to_str = reason;
      else
        to_str = to_str + ", " + reason;
    }
  }
}
