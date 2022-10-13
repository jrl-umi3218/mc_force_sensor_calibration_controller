#include "calibrate.h"

#include <ceres/ceres.h>

#include <mc_rtc/constants.h>
#include <mc_rtc/io_utils.h>
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

inline std::vector<std::string> getSuccessorBodies(const mc_rbdyn::Robot & robot_,
                                                   const std::string & rootBody,
                                                   bool includeRoot = false)
{
  auto & robot = const_cast<mc_rbdyn::Robot &>(robot_); // For successorJoints,
                                                        // should be const
  auto bIdx = robot.bodyIndexByName(rootBody);
  // Graph of successor joint built using robot's root as the root of the graph
  // This returns a map of
  // - BODY1 -> [SUCCESSOR JOINT1 of BODY1, SUCCESSOR JOINT2 of BODY1.. ]
  // - BODYN -> [SUCCESSOR JOINT1 of BODYN, SUCCESSOR JOINT2 of BODYN.. ]
  auto successorJointsGraph = robot.mbg().successorJoints(robot.mb().body(0).name());

  std::function<std::vector<std::string>(const std::vector<std::string> & succJoints)> computeSuccBodyNames;

  std::vector<std::string> successorBodyNames;
  if(includeRoot)
  {
    successorBodyNames.push_back(rootBody);
  }
  computeSuccBodyNames = [&successorBodyNames, &successorJointsGraph, &robot,
                          &computeSuccBodyNames](const std::vector<std::string> & succJoints) {
    for(const auto & joint : succJoints)
    {
      auto successorBodyIdx = robot.mb().successor(robot.mb().jointIndexByName(joint));
      const auto & successorBodyName = robot.mb().body(successorBodyIdx).name();
      const auto & successorJoints = successorJointsGraph.at(successorBodyName);
      // Name of the successor body of this joint + name of all its successors
      successorBodyNames.push_back(successorBodyName);
      computeSuccBodyNames(successorJoints);
    }
    return successorBodyNames;
  };

  return computeSuccBodyNames(successorJointsGraph[rootBody]);
}

CalibrationResult calibrate(const mc_rbdyn::Robot & robot,
                            const std::string & sensorN,
                            const Measurements & measurements,
                            const InitialGuess & initialGuess,
                            bool verbose)
{
  CalibrationResult result{initialGuess};
  const auto & sensor = robot.forceSensor(sensorN);

  // Build the problem.
  ceres::Problem problem;

  auto & mass = result.mass;
  auto * com = result.com.data();
  auto * rpy = result.rpy.data();
  auto * offset = result.offset.data();

  // For each measurement add a residual block
  for(const auto & measurement : measurements)
  {
    const auto & pos = measurement.X_0_p;
    const auto & f = measurement.measure;
    ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 6, 1, 3, 3, 6>(new CostFunctor(pos, f, sensor.X_p_f()));
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), &mass, rpy, com, offset);
  }
  ceres::CostFunction * min_rpy = new ceres::AutoDiffCostFunction<Minimize, 3, 3>(new Minimize());
  problem.AddResidualBlock(min_rpy, nullptr, rpy);
  problem.SetParameterLowerBound(&mass, 0, 0);
  problem.SetParameterLowerBound(rpy, 0, -2 * M_PI);
  problem.SetParameterLowerBound(rpy, 1, -2 * M_PI);
  problem.SetParameterLowerBound(rpy, 2, -2 * M_PI);
  problem.SetParameterUpperBound(rpy, 0, 2 * M_PI);
  problem.SetParameterUpperBound(rpy, 1, 2 * M_PI);
  problem.SetParameterUpperBound(rpy, 2, 2 * M_PI);

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

InitialGuess computeInitialGuessFromModel(const mc_rbdyn::Robot & robot, const std::string & sensorN, bool verbose)
{
  InitialGuess guess;
  const auto & sensor = robot.forceSensor(sensorN);
  const auto & X_0_parent = robot.bodyPosW(sensor.parentBody());

  // Compute initial guess from the robot model:
  // - mass: mass of all links under the force sensor. This assumes:
  //   1/ That the force sensor is attached to its parent link in the model (and
  //   thus that its mass is accounted for in the parent link)
  //   2/ That the mass of all child links under the force sensor is correct
  // FIXME for now include the parent body in the computation as HRP2 model is
  // buggy. Remove "true" argument once fixed
  auto successorBodies = getSuccessorBodies(robot, sensor.parentBody(), true);
  double totalMass = 0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  if(successorBodies.size())
  {
    auto & mb = robot.mb();
    auto & mbc = robot.mbc();
    for(const auto & bodyName : successorBodies)
    {
      auto bodyIndex = robot.mb().bodyIndexByName(bodyName);
      const auto & body = robot.mb().body(bodyIndex);
      double mass = body.inertia().mass();
      totalMass += mass;
      auto X_parent_body = mbc.bodyPosW[bodyIndex] * X_0_parent.inv();
      sva::PTransformd scaledBobyPosW(X_parent_body.rotation(), mass * X_parent_body.translation());
      com += (sva::PTransformd(body.inertia().momentum()) * scaledBobyPosW).translation();
    }
    if(totalMass > 0)
    {
      com /= totalMass;
    }
    else
    {
      mc_rtc::log::warning(
          "Warning: no mass provided for the bodies attached to force sensor \"{}\", assuming mass = 0 and CoM=[0,0,0]",
          sensor.name());
    }
  }

  guess.mass = totalMass;
  guess.com[0] = com[0];
  guess.com[1] = com[1];
  guess.com[2] = com[2];

  return guess;
}
