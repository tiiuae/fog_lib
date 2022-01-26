#include <fog_lib/geometry/misc.h>

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
     * \return the heading in range [-pi, pi]
     */
    double quat2heading(const geometry_msgs::msg::Quaternion& q)
    {
      quat_t eq;
      eq.x() = q.x;
      eq.y() = q.y;
      eq.z() = q.z;
      eq.w() = q.w;
      // obtain the direction vector of q
      const vec3_t dir = eq*vec3_t::UnitX();
      // calculate the angle of its projection to the XY plane from the X axis
      return std::atan2(dir.y(), dir.x());
    }

    /*!
     * \brief Calculates quaternion from the heading.
     *
     * The heading is the counterclockwise angle of a projection of the quaternion's direction vector to the XY plane from the X axis.
     *
     * \param heading the desired heading.
     * \return a quaternion, representing a rotation from [1, 0, 0] to the desired heading.
     */
    geometry_msgs::msg::Quaternion heading2quat(const double heading)
    {
      const quat_t q(anax_t(heading, vec3_t::UnitZ()));
      geometry_msgs::msg::Quaternion msg;
      msg.w = q.w();
      msg.x = q.x();
      msg.y = q.y();
      msg.z = q.z();
      return msg;
    }

    /*!
     * \brief Checks if any value in the input is nan.
     *
     * \param pose the input to check for nans.
     * \return true if any element of `pose` is nan.
     */
    bool has_nans(const geometry_msgs::msg::Pose& pose)
    {
      return std::isnan(pose.position.x)
       || std::isnan(pose.position.y)
       || std::isnan(pose.position.z)
       || std::isnan(pose.orientation.x)
       || std::isnan(pose.orientation.y)
       || std::isnan(pose.orientation.z)
       || std::isnan(pose.orientation.w);
    }

    /*!
     * \brief Wraps the fourth element of the input vector to [-pi, pi].
     *
     * \param xyzheading a four dimensional vector with the fourth element representing heading in radians.
     * \return the same vector with the fourth element wrapped to range [-pi, pi].
     */
    vec4_t wrap_heading(const vec4_t& xyzheading)
    {
      return vec4_t(xyzheading.x(), xyzheading.y(), xyzheading.z(), sradians::wrap(xyzheading.w()));
    }

  }  // namespace geometry
}  // namespace fog_lib
