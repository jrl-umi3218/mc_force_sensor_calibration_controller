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
  auto robotConf = ctl_.config()(robot.name());
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
  auto & measurements = ctl_.datastore().get<SensorMeasurements>("measurements");
  th_ = std::thread([&, verbose, this]() {
    for(const auto & s : sensors_)
    {
      mc_rtc::log::info("Start calibration optimization for {}", s);
      auto result = calibrate(ctl_.robot(), s, measurements.at(s), verbose);
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
