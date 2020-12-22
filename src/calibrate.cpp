#include "calibrate.h"

#include <ceres/ceres.h>

#include <mc_rtc/constants.h>
#include <RBDyn/FK.h>
#include <SpaceVecAlg/SpaceVecAlg>

/** We redefined sva::Rot. functions to make them work with non-scalar types */

template<typename T>
inline Eigen::Matrix3<T> RotX(T theta)
{
  T s = sin(theta), c = cos(theta);
  return (Eigen::Matrix3<T>() << T(1), T(0), T(0), T(0), c, s, T(0), -s, c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotY(T theta)
{
  T s = sin(theta), c = cos(theta);
  return (Eigen::Matrix3<T>() << c, T(0), -s, T(0), T(1), T(0), s, T(0), c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotZ(T theta)
{
  T s = sin(theta), c = cos(theta);
  return (Eigen::Matrix3<T>() << c, s, T(0), -s, c, T(0), T(0), T(0), T(1)).finished();
}

template<typename T>
Eigen::Matrix3<T> rpyToMat(const T & r, const T & p, const T & y)
{
  return RotX<T>(r) * RotY<T>(p) * RotZ<T>(y);
}

struct CostFunctor
{
  CostFunctor(const sva::PTransformd & pos, const sva::ForceVecd & measure, const sva::PTransformd & X_p_f)
  : pos_(pos), measure_(measure), X_p_f_(X_p_f)
  {
  }

  template<typename T>
  bool operator()(const T * const mass, const T * const rpy, const T * const com, const T * const offset, T * residual)
      const
  {
    const T gravity(mc_rtc::constants::GRAVITY);
    sva::ForceVec<T> vf(Eigen::Vector3<T>::Zero(), Eigen::Vector3<T>(T(0), T(0), -mass[0] * gravity));
    sva::PTransform<T> X_s_ds = sva::PTransform<T>(rpyToMat(rpy[0], rpy[1], rpy[2]));
    sva::PTransform<T> X_p_vb = sva::PTransform<T>(Eigen::Vector3<T>(com[0], com[1], com[2]));
    sva::ForceVec<T> off = sva::ForceVec<T>(Eigen::Vector3<T>(offset[0], offset[1], offset[2]),
                                            Eigen::Vector3<T>(offset[3], offset[4], offset[5]));
    sva::PTransform<T> X_0_p = pos_.cast<T>();
    sva::PTransform<T> X_0_vb = sva::PTransform<T>(Eigen::Matrix3<T>::Identity(), (X_p_vb * X_0_p).translation());
    sva::PTransform<T> X_p_ds = X_s_ds * X_p_f_.cast<T>();
    sva::PTransform<T> X_ds_vb = X_0_vb * (X_p_ds * X_0_p).inv();
    sva::ForceVec<T> vb_f = off + X_ds_vb.transMul(vf);
    sva::ForceVec<T> diff = measure_.cast<T>() - vb_f;
    residual[0] = diff.couple().x();
    residual[1] = diff.couple().y();
    residual[2] = diff.couple().z();
    residual[3] = diff.force().x();
    residual[4] = diff.force().y();
    residual[5] = diff.force().z();
    return true;
  }

private:
  const sva::PTransformd pos_;
  const sva::ForceVecd measure_;
  const sva::PTransformd X_p_f_;
};

struct Minimize
{
  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[0];
    residual[1] = x[1];
    residual[2] = x[2];
    return true;
  }
};

CalibrationResult calibrate(const mc_rbdyn::Robot & robot,
                            const std::string & sensorN,
                            const Measurements & measurements,
                            bool verbose)
{
  CalibrationResult result;
  const auto & sensor = robot.forceSensor(sensorN);
  result.mass = robot.mb().body(robot.bodyIndexByName(sensor.parentBody())).inertia().mass();

  // Build the problem.
  ceres::Problem problem;

  // For each measurement add a residual block
  for(const auto & measurement : measurements)
  {
    const auto & pos = measurement.X_0_p;
    const auto & f = measurement.measure;
    ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 6, 1, 3, 3, 6>(new CostFunctor(pos, f, sensor.X_p_f()));
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), &result.mass, result.rpy, result.com,
                             result.offset);
  }
  ceres::CostFunction * min_rpy = new ceres::AutoDiffCostFunction<Minimize, 3, 3>(new Minimize());
  problem.AddResidualBlock(min_rpy, nullptr, result.rpy);
  problem.SetParameterLowerBound(&result.mass, 0, 0);
  problem.SetParameterLowerBound(result.rpy, 0, -2 * M_PI);
  problem.SetParameterLowerBound(result.rpy, 1, -2 * M_PI);
  problem.SetParameterLowerBound(result.rpy, 2, -2 * M_PI);
  problem.SetParameterUpperBound(result.rpy, 0, 2 * M_PI);
  problem.SetParameterUpperBound(result.rpy, 1, 2 * M_PI);
  problem.SetParameterUpperBound(result.rpy, 2, 2 * M_PI);

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 1000;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  result.success = summary.IsSolutionUsable();

  // clang-format off
  mc_rtc::log::info(
R"({},
success     : {}
mass        : {}
rpy         : {}, {}, {}
com         : {}, {}, {}
force offset: {}, {}, {}, {}, {}, {})",
      summary.BriefReport(),
      result.success,
      result.mass,
      result.rpy[0], result.rpy[1], result.rpy[2],
      result.com[0], result.com[1], result.com[2],
      result.offset[0], result.offset[1], result.offset[2], result.offset[3], result.offset[4], result.offset[5]);
  // clang-format on
  return result;
}
