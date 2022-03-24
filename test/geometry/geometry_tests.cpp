#include <fog_lib/geometry/cyclic.h>
#include <fog_lib/geometry/misc.h>
#include <cmath>
#include <iostream>
#include <complex>

#include <gtest/gtest.h>

#include <random>

using namespace fog_lib::geometry;
using namespace std;

/* randd() //{ */

double randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* diffAngleTest() //{ */

double diffAngleTest(const double a, const double b) {
  return std::arg(std::polar<double>(1, a) * std::conj(std::polar<double>(1, b)));
}

//}

/* distAngleTest() //{ */

static double distAngleTest(const double a, const double b) {
  return std::abs(diffAngleTest(a, b));
}

//}

/* wrapAngleTest() //{ */

double wrapAngleTest(const double angle_in, const double angle_min = -M_PI, const double angle_max = M_PI) {

  const double angle_range   = angle_max - angle_min;
  double       angle_wrapped = angle_in;

  while (angle_wrapped > angle_max) {
    angle_wrapped -= angle_range;
  }

  while (angle_wrapped < angle_min) {
    angle_wrapped += angle_range;
  }

  return angle_wrapped;
}

//}

/* unwrapAngleTest() //{ */

double unwrapAngleTest(const double ang, const double ang_previous, const double angle_range = 2 * M_PI) {
  const double nega     = -angle_range / 2.0;
  const double posa     = angle_range / 2.0;
  const double ang_diff = wrapAngleTest(ang - ang_previous, nega, posa);
  return ang_previous + ang_diff;
}

//}

/* interpolateAngles() //{ */

double interpolateAngleTest(const double a1, const double a2, const double coeff) {
  Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);

  Eigen::Quaterniond quat1 = Eigen::Quaterniond(Eigen::AngleAxis<double>(a1, axis));
  Eigen::Quaterniond quat2 = Eigen::Quaterniond(Eigen::AngleAxis<double>(a2, axis));

  Eigen::Quaterniond new_quat = quat1.slerp(coeff, quat2);

  Eigen::Vector3d vecx = new_quat * Eigen::Vector3d(1, 0, 0);

  return atan2(vecx[1], vecx[0]);
}

//}

/* TEST(TESTSuite, diffAngle) //{ */

TEST(TESTSuite, diffAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double prev_angle = randd(-10000, 10000);
    double angle      = randd(-10000, 10000);

    double output   = radians::diff(angle, prev_angle);
    double expected = diffAngleTest(angle, prev_angle);

    if (fabs(output - expected) > 1e-6) {
      printf("diffAngle() #%d failed for radians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sradians::diff(angle, prev_angle);
    expected = wrapAngleTest(diffAngleTest(angle, prev_angle), -M_PI, M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("diffAngle() #%d failed for sradians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, distAngle) //{ */

TEST(TESTSuite, distAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double prev_angle = randd(-10000, 10000);
    double angle      = randd(-10000, 10000);

    double output   = radians::dist(angle, prev_angle);
    double expected = distAngleTest(angle, prev_angle);

    if (fabs(output - expected) > 1e-6) {
      printf("distAngle() #%d failed for radians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sradians::dist(angle, prev_angle);
    expected = wrapAngleTest(distAngleTest(angle, prev_angle), -M_PI, M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("distAngle() #%d failed for sradians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, wrapAngle) //{ */

TEST(TESTSuite, wrapAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double angle = randd(-10000, 10000);

    double output   = radians::wrap(angle);
    double expected = wrapAngleTest(angle, 0.0, 2 * M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sradians::wrap(angle);
    expected = wrapAngleTest(angle, -M_PI, M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = degrees::wrap(angle);
    expected = wrapAngleTest(angle, 0.0, 360.0);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sdegrees::wrap(angle);
    expected = wrapAngleTest(angle, -180.0, 180.0);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, unwrapAngle) //{ */

TEST(TESTSuite, unwrapAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double previous_angle = randd(-10000, 10000);
    double current_angle  = randd(-10000, 10000);

    double output   = radians::unwrap(current_angle, previous_angle);
    double expected = unwrapAngleTest(current_angle, previous_angle, 2 * M_PI);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for radians, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }

    output   = sradians::unwrap(current_angle, previous_angle);
    expected = unwrapAngleTest(current_angle, previous_angle, 2 * M_PI);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for sradians, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }

    output   = degrees::unwrap(current_angle, previous_angle);
    expected = unwrapAngleTest(current_angle, previous_angle, 360.0);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for degrees, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }

    output   = sdegrees::unwrap(current_angle, previous_angle);
    expected = unwrapAngleTest(current_angle, previous_angle, 360.0);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for sdegrees, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, interpolateAngle) //{ */

TEST(TESTSuite, interpolateAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double previous_angle = randd(-10000, 10000);
    double ang_diff       = randd(-M_PI + 1e-9, M_PI - 1e-9);
    double period_diff    = round(randd(-1000, 1000)) * 2 * M_PI;
    double current_angle  = previous_angle + ang_diff + period_diff;  // ensure tha angular difference not M_PI, which would make the solution ambiguous
    double coeff          = randd(-10000, 10000);

    double output   = sradians::interp(previous_angle, current_angle, coeff);
    double expected = interpolateAngleTest(previous_angle, current_angle, coeff);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr,
              "interpolateAngle() #%d failed for radians, input (prev %.2frad current %.2frad ~ prev %.2frad, cur %.2frad), output %.2f, expected %.2f\n", i,
              previous_angle, wrapAngleTest(previous_angle), current_angle, wrapAngleTest(current_angle), output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
