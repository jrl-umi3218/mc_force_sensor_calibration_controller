#pragma once

#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include <RBDyn/Jacobian.h>

#include <unordered_map>

struct CalibrationMotionLogging : mc_control::fsm::State
{

  inline void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  // sensor to calibrate
  std::vector<std::string> sensors_;
  // For each sensor store the jacobian and its decomposition
  struct Jacobian
  {
    Jacobian() {}
    Jacobian(const mc_rbdyn::Robot & robot, const std::string & body)
    : jacobian(robot.mb(), body), svd(Eigen::MatrixXd::Identity(6, jacobian.dof()))
    {
    }
    rbd::Jacobian jacobian;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  };
  std::unordered_map<std::string, Jacobian> jacobians_;
  double singularityThreshold_;
  std::unordered_map<std::string, std::pair<double, double>> measurementsCount_;
};
