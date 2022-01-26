#ifndef GEOMETRY_MISC_H
#define GEOMETRY_MISC_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <fog_lib/geometry/cyclic.h>

namespace fog_lib
{
  namespace geometry
  {
    using quat_t = Eigen::Quaterniond;
    using anax_t = Eigen::AngleAxisd;
    using vec3_t = Eigen::Vector3d;
    using vec4_t = Eigen::Vector4d;

    /*!
     * \brief Calculates heading from the quaternion.
     *
     * The heading is the counterclockwise angle of a projection of the quaternion's direction vector to the XY plane from the X axis.
     *
     * \param q the quaterion to obtain the heading from.
     * \return the heading in range [-pi, pi].
     */
    double quat2heading(const geometry_msgs::msg::Quaternion& q);

    /*!
     * \brief Calculates heading from the quaternion.
     *
     * The heading is the counterclockwise angle of a projection of the quaternion's direction vector to the XY plane from the X axis.
     *
     * \param q the quaterion to obtain the heading from.
     * \return the heading in range [-pi, pi].
     */
    double quat2heading(const tf2::Quaternion& q);

    /*!
     * \brief Calculates quaternion from the heading.
     *
     * The heading is the counterclockwise angle of a projection of the quaternion's direction vector to the XY plane from the X axis.
     *
     * \param heading the desired heading.
     * \return a quaternion, representing a rotation from [1, 0, 0] to the desired heading.
     */
    geometry_msgs::msg::Quaternion heading2quat(const double heading);

    /*!
     * \brief Checks if any value in the input is nan.
     *
     * \param pose the input to check for nans.
     * \return true if any element of `pose` is nan.
     */
    bool has_nans(const geometry_msgs::msg::Pose& pose);

    /*!
     * \brief Wraps the fourth element of the input vector to [-pi, pi].
     *
     * \param xyzheading a four dimensional vector with the fourth element representing heading in radians.
     * \return the same vector with the fourth element wrapped to range [-pi, pi].
     */
    vec4_t wrap_heading(const vec4_t& xyzheading);
  }  // namespace geometry
}  // namespace fog_lib

#endif // GEOMETRY_MISC_H
