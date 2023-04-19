#include "RunCalibrationScript.h"

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "../ForceSensorCalibration.h"
#include "../calibrate.h"

RunCalibrationScript::~RunCalibrationScript()
{
  th_.join();
}

void RunCalibrationScript::configure(const mc_rtc::Configuration &) {}

void RunCalibrationScript::start(mc_control::fsm::Controller & ctl_)
{
  auto & robot = ctl_.robot();
  auto robotConf = ctl_.config()("robots")(robot.name());
  outputPath_ = "/tmp/calib-force-sensors-result-" + ctl_.robot().name();

  // Attempt to create the output folder
  if(!boost::filesystem::exists(outputPath_))
  {
    if(!boost::filesystem::create_directory(outputPath_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Could not create output folder {}", name(), outputPath_);
    }
  }

  sensors_ = robotConf("forceSensors");
  bool verbose = robotConf("verboseSolver", false);
  std::vector<InitialGuess> guess_;
  guess_.reserve(sensors_.size());

  for(const auto & s : sensors_)
  {
    InitialGuess initialGuess;
    // If we have an initial guess section provided and it does not have
    // autocompute: true then we use the provided initial guess
    if(robotConf.has("initialGuess") && robotConf("initialGuess").has(s)
       && !robotConf("initialGuess")(s)("autocompute", false))
    {
      initialGuess = robotConf("initialGuess")(s);
      mc_rtc::log::info("Using provided initial guess for \"{}\":\n{}", s, initialGuess);
    }
    else
    { // No initial guess was provided and we need to autocompute it
      bool verbose = false;
      bool includeParent = false;
      if(robotConf.has("initialGuess") && robotConf("initialGuess").has(s))
      {
        robotConf("initialGuess")(s)("verbose", verbose);
        robotConf("initialGuess")(s)("includeParent", includeParent);
      }
      initialGuess = computeInitialGuessFromModel(ctl_.robot(), s, includeParent, verbose);
      mc_rtc::log::info("Computed initial guess for \"{}\" from model (includeParent: {}, verbose: {}):\n{} ", s,
                        includeParent, verbose, initialGuess);
    }
    guess_.push_back(initialGuess);
  }

  auto & measurements = ctl_.datastore().get<SensorMeasurements>("measurements");
  th_ = std::thread([&, verbose, guess_, this]() {
    for(size_t i = 0; i < sensors_.size(); ++i)
    {
      const auto & s = sensors_[i];
      const auto & initialGuess = guess_[i];
      mc_rtc::log::info("Start calibration optimization for {}", s);
      auto result = calibrate(ctl_.robot(), s, measurements.at(s), initialGuess, verbose);
      success_ = result.success && success_;
      if(result.success)
      {
        mc_rtc::log::success("Calibration succeeded for {}", s);
        bfs::path out(outputPath_);
        out += "/calib_data." + s;
        std::ofstream ofs(out.string());
        if(!ofs.good())
        {
          mc_rtc::log::error("Could not write temporary calibration file to {}", out.string());
          continue;
        }
        ofs << result.mass << "\n";
        ofs << result.rpy[0] << "\n";
        ofs << result.rpy[1] << "\n";
        ofs << result.rpy[2] << "\n";
        ofs << result.com[0] << "\n";
        ofs << result.com[1] << "\n";
        ofs << result.com[2] << "\n";
        ofs << result.offset[0] << "\n";
        ofs << result.offset[1] << "\n";
        ofs << result.offset[2] << "\n";
        ofs << result.offset[3] << "\n";
        ofs << result.offset[4] << "\n";
        ofs << result.offset[5] << "\n";
        mc_rtc::log::info("Wrote temporary calibration file to {}", out.string());
      }
      else
      {
        mc_rtc::log::error("Calibration failed for {}", s);
      }
    }
    completed_ = true;
  });
#ifndef WIN32
  // Lower thread priority so that it has a lesser priority than the real time
  // thread
  auto th_handle = th_.native_handle();
  int policy = 0;
  sched_param param{};
  pthread_getschedparam(th_handle, &policy, &param);
  param.sched_priority = 10;
  if(pthread_setschedparam(th_handle, SCHED_RR, &param) != 0)
  {
    mc_rtc::log::warning("[{}] Failed to lower calibration thread priority. If you are running on a real-time system, "
                         "this might cause latency to the real-time loop.",
                         name());
  }
#endif
}

bool RunCalibrationScript::run(mc_control::fsm::Controller &)
{
  if(completed_)
  {
    if(!success_)
    {
      mc_rtc::log::error("[ForceSensor] Calibration script failed");
      output("FAILED");
      return true;
    }
    mc_rtc::log::info("[ForceSensorCalibration] Calibration files written to {}", outputPath_);
    output("SUCCESS");
    return true;
  }
  return false;
}

void RunCalibrationScript::teardown(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeElement({}, "Status");
}

EXPORT_SINGLE_STATE("RunCalibrationScript", RunCalibrationScript)
