// clang: MatousFormat
/**  \file
     \brief Implements the cyclic class for calculations with cyclic quantities. Originally from mrs_lib: https://github.com/ctu-mrs/mrs_lib
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#include <fog_lib/geometry/cyclic.h>

namespace fog_lib
{
  namespace geometry
  {

    // to ensure these classes are generated
    template struct cyclic<double, radians>;
    template struct cyclic<double, sradians>;
    template struct cyclic<double, degrees>;
    template struct cyclic<double, sdegrees>;

  }  // namespace geometry
}  // namespace fog_lib

