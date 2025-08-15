#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <ati_netft/ati_netft.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <unistd.h>
#include <ur_rtde/ur_rtde.h>
#include <yaml-cpp/yaml.h>
#include <force_control/config_deserialize.h>
bool deserialize_new(const YAML::Node& node,
                 AdmittanceController::AdmittanceControllerConfig& config) {
  std::cout << __LINE__ << std::endl;
  std::cout << "config.dt: " << config.dt << std::endl;
  std::cout << "node.dt: " << node["dt"] << std::endl;
  try {
    config.dt = node["dt"].as<double>();
  std::cout << __LINE__ << std::endl;
    config.log_to_file = node["log_to_file"].as<bool>();
    config.log_file_path = node["log_file_path"].as<std::string>();
  std::cout << __LINE__ << std::endl;
    config.compliance6d.stiffness = RUT::deserialize_vector<RUT::Vector6d>(
                                        node["compliance6d"]["stiffness"])
                                        .asDiagonal();
  std::cout << __LINE__ << std::endl;
    config.compliance6d.damping =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["damping"])
            .asDiagonal();
    config.compliance6d.inertia =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["inertia"])
            .asDiagonal();
    std::cout << "config.compliance6d.stiffness: " << config.compliance6d.stiffness
         << std::endl;
  std::cout << __LINE__ << std::endl;
    config.max_spring_force_magnitude =
        node["max_spring_force_magnitude"].as<double>();
    config.direct_force_control_gains.P_trans =
        node["direct_force_control_gains"]["P_trans"].as<double>();
    config.direct_force_control_gains.I_trans =
        node["direct_force_control_gains"]["I_trans"].as<double>();
    config.direct_force_control_gains.D_trans =
        node["direct_force_control_gains"]["D_trans"].as<double>();
  std::cout << __LINE__ << std::endl;
    config.direct_force_control_gains.P_rot =
        node["direct_force_control_gains"]["P_rot"].as<double>();
    config.direct_force_control_gains.I_rot =
        node["direct_force_control_gains"]["I_rot"].as<double>();
    config.direct_force_control_gains.D_rot =
        node["direct_force_control_gains"]["D_rot"].as<double>();
  std::cout << __LINE__ << std::endl;
    config.direct_force_control_I_limit =
        RUT::deserialize_vector<RUT::Vector6d>(
            node["direct_force_control_I_limit"]);
  std::cout << __LINE__ << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
    
  }
  return true;
}

Eigen::MatrixXd deserialize_matrix(const YAML::Node& node) {
  int nr = node.size();
  int nc = node[0].size();
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nr, nc);
  for (int r = 0; r < nr; ++r) {
    for (int c = 0; c < nc; ++c) {
      mat(r, c) = node[r][c].as<double>();
    }
  }
  return mat;
}

// load eigen
template <typename T>
T deserialize_vector(const YAML::Node& node) {
  std::vector<double> q = node.as<std::vector<double>>();
  return Eigen::Map<T, Eigen::Unaligned>(q.data(), q.size());
}

int main() {
  URRTDE::URRTDEConfig robot_config;
  ATINetft::ATINetftConfig ati_config;
  AdmittanceController::AdmittanceControllerConfig admittance_config;

  // open file
  const std::string CONFIG_PATH =
      "/home/kukabot/Projects/force_ctl/hardware_interfaces/applications/force_control_demo/"
      "config/force_control_demo.yaml";
  YAML::Node config{};
  try {
    config = YAML::LoadFile(CONFIG_PATH);

    robot_config.deserialize(config["ur_rtde"]);
    ati_config.deserialize(config["ati_netft"]);
    std::cout << config["admittance_controller"]["dt"] << std::endl;
    deserialize(config["admittance_controller"], admittance_config);
    std::cout << "Robot initializing." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return -1;
  }
  std::cout << "Robot initializing." << std::endl;

  URRTDE robot;
  std::shared_ptr<FTInterfaces> force_sensor_ptr;
  AdmittanceController controller;
  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::Vector6d wrench, wrench0, wrench_WTr;

  robot.init(time0, robot_config);
  std::cout << "Robot initialized." << std::endl;
  robot.getCartesian(pose);
  std::cout << "pose: " << pose.transpose() << std::endl;
  // robot.getWrenchTool(wrench0);


  controller.init(time0, admittance_config, pose);

  force_sensor_ptr = std::shared_ptr<ATINetft>(new ATINetft);
  ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptr.get());
  if (!ati_ptr->init(time0, ati_config)) {
    std::cerr << "Failed to initialize ATI Netft. Exiting." << std::endl;
    return false;
  }
  // wait for FT300 to be ready
  std::cout << "Waiting for FT sensor to start streaming.\n";
  while (!force_sensor_ptr->is_data_ready()) {
    usleep(100000);
  }
  
  // get average wrench
  force_sensor_ptr->getWrenchTool(wrench0);
  std::cout << "wrench0: " << wrench0.transpose() << std::endl;
  std::cout << "Starting in 2 seconds ..." << std::endl;
  int N = 200;
  for (int i = 0; i < N; ++i) {
    RUT::Vector6d wrench_temp;
    // robot.getWrenchTool(wrench_temp);
    force_sensor_ptr->getWrenchTool(wrench0);

    wrench0 += wrench_temp;
    usleep(10000);
  }
  wrench0 /= N;
  std::cout << "wrench0: " << wrench0.transpose() << std::endl;

  // wrench0.setZero();
  sleep(2.0);
  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // RUT::Matrix6d Tr;
  // // clang-format off
  // Tr << 0, 0, 0, 1, 0, 0,
  //       0, 0, 0, 0, 1, 0,
  //       0, 0, 0, 0, 0, 1,
  //       1, 0, 0, 0, 0, 0,
  //       0, 1, 0, 0, 0, 0,
  //       0, 0, 1, 0, 0, 0;
  // // clang-format on
  int n_af = 2;
  controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;
  wrench_WTr.setZero();


  int flag = 0;
  while (true) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    robot.getCartesian(pose);
    // robot.getWrenchTool(wrench);
    flag = force_sensor_ptr->getWrenchTool(wrench);
    if (flag != 0) {
      std::cout << "Failed to get wrench from sensor. Exiting." << std::endl;
      break;
    }
    wrench[0] = -wrench[0];
    wrench[1] = -wrench[1];
    controller.setRobotStatus(pose, wrench - wrench0);

    // Update the force in the control variable
    wrench_WTr[2] = -5 // z-axis force

    // Update robot reference
    controller.setRobotReference(pose_ref, wrench_WTr);

    // Compute the control output
    controller.step(pose_cmd);

    if (!robot.streamCartesian(pose_cmd)) {
      printf("streamCartesian failed\n");
      break;
    }

    double dt = timer.toc_ms();
    printf("t = %f, wrench: %f %f %f %f %f %f\n", dt, wrench[0], wrench[1],
           wrench[2], wrench[3], wrench[4], wrench[5]);

    if (dt > 300000)
      break;

    robot.rtde_wait_period(t_start);
  }

  return 0;
}