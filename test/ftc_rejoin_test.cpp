#include "ftc_controller/rejoin.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

using namespace open_mower_next::ftc_controller;

namespace
{
Eigen::Affine3d pose2d(double x, double y, double yaw)
{
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, 0.0);
  t.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return t;
}

// The FTC FOLLOWING turn law with the worx_nav2.yaml gains (kp_ang 3, kp_lat 12,
// lateral_priority_distance 0.15, max_cmd_vel_ang 2.8, max_drive_angle 45) on a
// unicycle with the Worx's ~0.3 s command lag. Path: the x axis; the carrot runs
// `lead` ahead of the robot. Returns the robot's offset over time.
struct SimResult
{
  double max_heading = 0.0;  // rad, |robot yaw| (= heading relative to the path)
  double stalled = 0.0;      // s with v < 0.05 while > 0.05 m off
  double settle = -1.0;      // s until the offset stayed < 3 cm
  double final_offset = 0.0;
};

SimResult simulate(double y0, bool rejoin, double seconds = 30.0)
{
  const RejoinConfig c;
  const double kp_ang = 3.0, kp_lat = 12.0, prio = 0.15, w_max = 2.8, max_drive = 45.0 * M_PI / 180.0;
  const double v_carrot = 0.2, lead = 0.3, dt = 0.05, lag = 0.3;
  double x = 0.0, y = y0, yaw = 0.0, v = 0.0, w = 0.0;
  SimResult r;
  for (double t = 0.0; t < seconds; t += dt) {
    const Eigen::Affine3d carrot_in_base = pose2d(x, y, yaw).inverse() * pose2d(x + lead, 0.0, 0.0);
    const Eigen::Matrix3d rot = carrot_in_base.rotation();
    const double angle_error = std::atan2(rot(1, 0), rot(0, 0));
    const double lat_error = carrot_in_base.translation().y();
    const double lon_error = carrot_in_base.translation().x();
    const double ct = crossTrack(carrot_in_base);
    const double wr = rejoin ? rejoinWeight(ct, c) : 0.0;
    const double approach = approachAngle(ct, c);

    const double gain = std::max(std::abs(lat_error) >= prio ? 0.0 : (prio - std::abs(lat_error)) / prio, 0.1);
    double ang = gain * angle_error * kp_ang + lat_error * kp_lat;
    ang = (1.0 - wr) * ang + wr * kp_ang * (angle_error + approach);
    ang = std::clamp(ang, -w_max, w_max);
    const double gate = std::abs(angle_error + wr * approach);
    double lin = v_carrot + 1.5 * (lon_error - lead);
    lin = gate > max_drive ? 0.0 : std::clamp(lin * std::cos(gate), 0.0, 0.3);

    v += (lin - v) * dt / lag;
    w += (ang - w) * dt / lag;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw += w * dt;
    r.max_heading = std::max(r.max_heading, std::abs(yaw));
    if (v < 0.05 && std::abs(y) > 0.05) r.stalled += dt;
    if (std::abs(y) >= 0.03) r.settle = -1.0;
    else if (r.settle < 0.0) r.settle = t;
  }
  r.final_offset = y;
  return r;
}
}  // namespace

TEST(FtcRejoin, CrossTrackIsIndependentOfHeading)
{
  // Robot 0.4 m right of the path (path to its left), carrot 0.3 m ahead on the path.
  for (double yaw : {0.0, 0.5, -0.5, 1.2}) {
    const Eigen::Affine3d c = pose2d(0.0, -0.4, yaw).inverse() * pose2d(0.3, 0.0, 0.0);
    EXPECT_NEAR(crossTrack(c), 0.4, 1e-9) << "yaw " << yaw;
  }
  const Eigen::Affine3d left = pose2d(0.0, 0.4, 0.3).inverse() * pose2d(0.3, 0.0, 0.0);
  EXPECT_NEAR(crossTrack(left), -0.4, 1e-9);
}

TEST(FtcRejoin, ApproachAngleAndBlend)
{
  const RejoinConfig c;
  EXPECT_NEAR(approachAngle(0.0, c), 0.0, 1e-12);
  EXPECT_NEAR(approachAngle(0.1, c), std::atan(0.2), 1e-12);
  EXPECT_NEAR(approachAngle(5.0, c), 30.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(approachAngle(-5.0, c), -30.0 * M_PI / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(rejoinWeight(0.05, c), 0.0);
  EXPECT_DOUBLE_EQ(rejoinWeight(-0.175, c), 0.5);
  EXPECT_DOUBLE_EQ(rejoinWeight(0.4, c), 1.0);
}

TEST(FtcRejoin, DrivesBackToThePathFromBesideIt)
{
  for (double y0 : {0.3, 0.6, 1.0, -0.3, -0.6, -1.0}) {
    const SimResult r = simulate(y0, true);
    std::printf("offset %+.1f m: back within 3 cm after %.1f s, max heading %.0f deg, stalled %.1f s\n", y0, r.settle,
                r.max_heading * 180.0 / M_PI, r.stalled);
    EXPECT_GE(r.settle, 0.0) << "offset " << y0 << " never settled (final " << r.final_offset << ")";
    EXPECT_LT(r.settle, 15.0) << "offset " << y0;
    // Keeps driving: capped approach angle (30 deg + overshoot) stays below max_drive_angle.
    EXPECT_LT(r.max_heading, 40.0 * M_PI / 180.0) << "offset " << y0;
    EXPECT_LT(r.stalled, 1.0) << "offset " << y0;
  }
}

TEST(FtcRejoin, WithoutItTheRobotTurnsInPlaceOrCreeps)
{
  // The law it replaces: from 0.6 m off the lateral term turns the robot beyond
  // max_drive_angle - it stands turning instead of driving back (the slope case).
  const SimResult r = simulate(0.6, false);
  std::printf("without rejoin, offset +0.6 m: max heading %.0f deg, stalled %.1f s, offset after 30 s %.2f m\n",
              r.max_heading * 180.0 / M_PI, r.stalled, r.final_offset);
  EXPECT_GT(r.max_heading, 45.0 * M_PI / 180.0);
  EXPECT_GT(r.stalled, 1.0);
}
