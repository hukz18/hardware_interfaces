#include <unistd.h>
#include <memory>

#include <yaml-cpp/yaml.h>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <ati_netft/ati_netft.h>
#include <force_control/config_deserialize.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>

int main() {
  std::cout << "Force calibration demo\n";
  std::cout << "This program will collect wrench data from the FT sensor and "
               "solve for the offset, gravity and COM.\n";
  std::cout << "The robot will be in admittance control mode, where you are "
               "supposed to drag the robot slowly to different orientations "
               "within 10 seconds. The calibration result will be printed out "
               "in the end.\n";
  std::cout << "Note: 1. Make sure the FT sensor config has zeros for Foffset, "
               "Toffset, Gravity and PCom.\n";
  std::cout << "      2. Make sure the FT sensor config has the correct "
               "PoseSensorTool.\n";
  std::cout << "      3. Make sure you are calling the getWrenchTool() "
               "function to get wrench reading for calibration.\n";

  RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
  ATINetft::ATINetftConfig ati_config;

  // open file
  const std::string CONFIG_PATH =
      "/home/kukabot/Projects/force_ctl/hardware_interfaces/applications/ft_calibration/"
      "config/ft_calibration.yaml";
  YAML::Node config{};
  try {
    config = YAML::LoadFile(CONFIG_PATH);
    std::cout << "config: " << config << std::endl;
    robotiq_config.deserialize(config["robotiq_ft_modbus"]);
    ati_config.deserialize(config["ati_netft"]);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return -1;
  }

  std::shared_ptr<FTInterfaces> force_sensor_ptr;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::Vector6d wrench, wrench_WTr;


  // force sensor
  bool use_ati = config["use_ati"].as<bool>();
  if (use_ati) {
    force_sensor_ptr = std::shared_ptr<ATINetft>(new ATINetft);
    ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptr.get());
    if (!ati_ptr->init(time0, ati_config)) {
      std::cerr << "Failed to initialize ATI Netft. Exiting." << std::endl;
      return false;
    }
  } else {
    force_sensor_ptr = std::shared_ptr<RobotiqFTModbus>(new RobotiqFTModbus);
    RobotiqFTModbus* robotiq_ptr =
        static_cast<RobotiqFTModbus*>(force_sensor_ptr.get());
    if (!robotiq_ptr->init(time0, robotiq_config)) {
      std::cerr << "Failed to initialize Robotiq FT Modbus. Exiting."
                << std::endl;
      return false;
    }
  }

  // wait for FT300 to be ready
  std::cout << "Waiting for FT sensor to start streaming.\n";
  while (!force_sensor_ptr->is_data_ready()) {
    usleep(100000);
  }
  
  pose_ref = pose;
  wrench_WTr.setZero();

  timer.tic();

  double t_last_collect_ms = 0;
  const double collect_interval_ms = 0.1;
  double total_duration_ms = 20000;
  RUT::Vector3d wrench_direction;
  RUT::Vector3d wrench_direction_temp;
  double wrench_threshold = 1;
  int N = 500;
  int i = 0;
  while (i < N) {
    double dt = timer.toc_ms();
    if (dt > t_last_collect_ms + collect_interval_ms) {
      // robot.getWrenchTool(wrench_temp);
      force_sensor_ptr->getWrenchTool(wrench);
      wrench_direction_temp = wrench.head(3);
      t_last_collect_ms = dt;
      if (wrench_direction_temp.norm() < wrench_threshold) {
        continue;
      }
      // normalize the wrench direction
      wrench_direction_temp.normalize();
      wrench_direction += wrench_direction_temp;
      std::cout << i << ":wrench_dir: " << wrench_direction_temp.transpose() << "wrench:" << wrench.head(3).transpose() << std::endl;
      usleep(1000);
      i++;
    }
  }
  wrench_direction /= N;
  std::cout << "wrench_direction: " << wrench_direction.transpose() << std::endl;
//0.528953 0.836613
//   while (true) {

//     // get wrench from the sensor to be calibrated
//     double dt = timer.toc_ms();
//     if (dt > t_last_collect_ms + collect_interval_ms) {
//       force_sensor_ptr->getWrenchTool(wrench);
//       t_last_collect_ms = dt;
//       printf("t = %f, recorded wrench: %f %f %f %f %f %f\n", dt, wrench[0],
//              wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
//     }

//     if (dt > total_duration_ms)
//       break;

//   }

  return 0;
}