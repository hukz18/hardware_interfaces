#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <ati_netft/ati_netft.h>
#include <fcntl.h>  // O_CREAT, O_RDWR
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <sys/mman.h>  // shm_open, mmap, PROT_READ, PROT_WRITE
#include <sys/stat.h>  // ftruncate
#include <unistd.h>
#include <ur_rtde/ur_rtde.h>
#include <yaml-cpp/yaml.h>
#include <atomic>  // std::atomic
#include <cstring>
#include <iostream>
#include <string>

// #define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINT(msg) printf("%s:%d: %s\n", __FILE__, __LINE__, msg)
#else
// Otherwise, DEBUG_PRINT does nothing.
#define DEBUG_PRINT(msg)
#endif
// home pose for walking
// rope 1
const RUT::Vector7d HOME_POSE_STAND = {-0.352989,  -0.318514, 0.460323,
                                       0.00997964, -0.875181, -0.483633,
                                       -0.00758494};

// -0.338378  -0.374854   0.491989 0.00880862   -0.87413  -0.485484 -0.0111193
// rope 2
// const RUT::Vector7d HOME_POSE_STAND = {-0.336484,  -0.373739, 0.361814,
//                                        0.00874653, -0.874106, -0.485526,
//                                        -0.0112256};
// home pose for swinging
// const RUT::Vector7d HOME_POSE_STAND = {-0.349084, -0.407867, 0.616498, 0.0,
//                                        -0.576443, -0.817137, 0.0};
// const RUT::Vector7d HOME_POSE_LIFT = {-0.440704, -0.378338, 0.585577, 0.00983115, -0.875206, -0.483591, -0.00765516};
const RUT::Vector7d HOME_POSE_LIFT = {-0.353008,  -0.318511, 0.588373,
                                      0.0100216,  -0.875186, -0.483622,
                                      -0.00759084};
// const RUT::Vector7d HOME_POSE_RELAX = {-0.440701, -0.378321, 0.376626, 0.0098295, -0.875218, -0.483569, -0.00756529};
const RUT::Vector7d HOME_POSE_RELAX = {-0.352989,  -0.318514, 0.460323,
                                       0.00997964, -0.875181, -0.483633,
                                       -0.00758494};

bool deserialize_new(const YAML::Node& node,
                     AdmittanceController::AdmittanceControllerConfig& config) {
  std::cout << __LINE__ << std::endl;
  std::cout << "config.dt: " << config.dt << std::endl;
  std::cout << "node.dt: " << node["dt"] << std::endl;
  try {
    config.dt = node["dt"].as<double>();
    config.enable_force_control = node["enable_force_control"].as<bool>();
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
    std::cout << "config.compliance6d.stiffness: "
              << config.compliance6d.stiffness << std::endl;
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

// Function to interpolate and move the robot to a target pose
void moveToPose(URRTDE& robot, const RUT::Vector7d& start_pose,
                const RUT::Vector7d& target_pose, int steps = 500,
                int delay_us = 10000) {
  for (int i = 0; i < steps; ++i) {
    double alpha = static_cast<double>(i) / steps;
    RUT::Vector7d interpolated_pose =
        (1 - alpha) * start_pose + alpha * target_pose;
    robot.streamCartesian(interpolated_pose);
    usleep(delay_us);
  }
  std::cout << "Movement to target pose completed." << std::endl;
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
      "/home/kukabot/Projects/force_ctl/hardware_interfaces/applications/"
      "force_control_demo/"
      "config/force_control_demo_walk.yaml";
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
  RUT::Vector7d pose, pose_ref, pose_cmd, init_pose, init_pose_ref;
  RUT::Vector6d wrench, wrench0, wrench_WTr, velocity;

  robot.init(time0, robot_config);
  std::cout << "Robot initialized." << std::endl;
  robot.getCartesian(pose);
  robot.getCartesianVelocity(velocity);
  std::cout << "pose: " << pose.transpose() << std::endl;
  // Wait for enter to continue
  std::cout << "Press Enter to reset to HOME RELAX pose ...";
  std::cin.get();
  // Interpolate to home pose
  moveToPose(robot, pose, HOME_POSE_RELAX);

  robot.getCartesian(pose);
  robot.getCartesian(init_pose);
  robot.getCartesianVelocity(velocity);
  std::cout << "pose: " << pose.transpose() << std::endl;
  // robot.getWrenchTool(wrench0);

  // robot.setCartesian(pose);
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
  std::cout << "Press ENTER to getting average wrench for biasing, please lift "
               "the robot during the procudure..."
            << std::endl;
  std::cin.get();
  force_sensor_ptr->getWrenchTool(wrench0);
  // std::cout << "Starting in 2 seconds ..." << std::endl;
  int N = 200;
  for (int i = 0; i < N; ++i) {
    RUT::Vector6d wrench_temp;
    // robot.getWrenchTool(wrench_temp);
    force_sensor_ptr->getWrenchTool(wrench_temp);

    wrench0 += wrench_temp;
    usleep(10000);
  }
  wrench0 /= N;
  std::cout << "wrench0: " << wrench0.transpose() << std::endl;
  // if wrench0 is too large or max item of wrench0 is too large, exit and remind to bias.
  if (wrench0.norm() > 4.0 || wrench0.maxCoeff() > 4.0) {
    std::cout << "Wrench0 is too large. Please reset the sensor bias."
              << std::endl;
    return -1;
  }

  std::cout << "Press Enter to reset to HOME STAND pose ...";
  std::cin.get();
  // Interpolate to home pose
  moveToPose(robot, pose, HOME_POSE_STAND);

  robot.getCartesian(pose);
  robot.getCartesianVelocity(velocity);
  std::cout << "Home pose: " << pose.transpose() << std::endl;
  // wrench0.setZero();
  // sleep(2.0);
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
  // int n_af = 2;
  std::cout << "Setting force controlled axis to" << admittance_config.n_af
            << std::endl;
  controller.setForceControlledAxis(Tr, admittance_config.n_af);

  pose_ref = pose;
  init_pose_ref = pose;
  wrench_WTr.setZero();

  const char* shm_name = "/force_shm";

  // Define the shared memory structure
  struct SharedData {
    alignas(std::atomic<double>) std::atomic<double> force;
    alignas(std::atomic<double>) std::atomic<double> z_pose_delta;

    // Add the wrench (6D) and the first 3 dimensions of the pose (XYZ)
    alignas(std::atomic<double>) std::atomic<double> wrench[6];
    alignas(std::atomic<double>) std::atomic<double> pose_xyz[3];
    alignas(std::atomic<double>) std::atomic<double> vel_xyz[3];
  };

  // Open the shared memory object
  int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
  if (shm_fd == -1) {
    std::cerr << "Failed to create shared memory segment" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Set the size of the shared memory segment
  if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
    std::cerr << "Failed to set size of shared memory segment" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Map the shared memory segment into the process's address space
  void* ptr = mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE,
                   MAP_SHARED, shm_fd, 0);
  if (ptr == MAP_FAILED) {
    std::cerr << "Failed to map shared memory segment" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Access the shared data
  SharedData* shared_data = static_cast<SharedData*>(ptr);

  // Initialize the force value to zero
  shared_data->force.store(0.0);
  shared_data->z_pose_delta.store(0.0);
  for (int i = 0; i < 6; i++) {
    shared_data->wrench[i].store(wrench0[i]);
  }
  for (int i = 0; i < 3; i++) {
    shared_data->pose_xyz[i].store(pose[i]);
  }
  for (int i = 0; i < 3; i++) {
    shared_data->vel_xyz[i].store(velocity[i]);
  }
  std::cout << "shared_data->wrench_z: " << shared_data->wrench[2].load()
            << std::endl;
  std::cout << "shared_data->pose_z: " << shared_data->pose_xyz[2].load()
            << std::endl;
  std::cout << "Waiting for initial force value..." << std::endl;
  double current_force, current_z_pose_delta, prev_force;
  while (true) {
    current_force = shared_data->force.load();

    if (current_force > 0.0) {
      std::cout << "Received initial force: " << current_force << " N"
                << std::endl;
      break;
    }
    usleep(100000);  // Sleep for 100ms to prevent busy-waiting
  }
  timer.tic();
  double dt = timer.toc_ms();
  int flag = 0;
  while (true) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    DEBUG_PRINT("Updating robot status");
    robot.getCartesian(pose);
    DEBUG_PRINT("Getting pose");
    robot.getCartesianVelocity(velocity);
    DEBUG_PRINT("Getting velocity");

    // robot.getWrenchTool(wrench);
    flag = force_sensor_ptr->getWrenchTool(wrench);
    DEBUG_PRINT("Getting wrench");

    wrench -= wrench0;
    if (flag != 0) {
      std::cout << "Failed to get wrench from sensor. Exiting." << std::endl;
      break;
    }
    // Store updated wrench and pose values to shared memory
    for (int i = 0; i < 6; i++) {
      shared_data->wrench[i].store(wrench[i]);
    }
    for (int i = 0; i < 3; i++) {
      shared_data->pose_xyz[i].store(pose[i]);
    }
    for (int i = 0; i < 3; i++) {
      shared_data->vel_xyz[i].store(velocity[i]);
    }
    DEBUG_PRINT("Storing wrench and pose to shared memory");

    // printf("t = %f, wrench: %.4f %.4f %.4f\n", dt, wrench[0], wrench[1], wrench[2]);
    wrench[0] = -wrench[0];
    wrench[1] = -wrench[1];
    if (!admittance_config.enable_force_control) {
      for (int i = 1; i < 6; i++) {
        wrench[i] = 0.0;
      }
    }
    DEBUG_PRINT("Setting robot status");
    controller.setRobotStatus(pose, wrench);

    // Read the force value from shared memory
    DEBUG_PRINT("Reading force value from shared memory");
    current_force = shared_data->force.load();
    // Check for termination condition
    if (current_force == 0.0) {
      std::cout << "Force value set to zero. Terminating control loop."
                << std::endl;
      break;
    } else if (current_force == -0.5) {
      std::cout << "Force value set to -0.5. Waiting for reset" << std::endl;
      shared_data->wrench[0].store(0.0);
      shared_data->pose_xyz[0].store(init_pose[0]);
      while (shared_data->force.load() == -0.5) {
        usleep(100000);
      }
    }
    DEBUG_PRINT("Checking force value");
    if (current_force == -1.0) {
      std::cout << "Force value set to -1.0. Reseting the position."
                << std::endl;
      std::cout << "Previous force: " << prev_force << std::endl;
      robot.getCartesian(pose);
      moveToPose(robot, pose, HOME_POSE_LIFT);
      usleep(1000000);
      moveToPose(robot, HOME_POSE_LIFT, HOME_POSE_STAND);
      robot.getCartesian(pose);
      current_force = prev_force;
      shared_data->force.store(prev_force);
      controller.setRobotStatus(pose, wrench);

    } else if (current_force == -2.0) {
      std::cout << "Force value set to -2.0. Waiting for training."
                << std::endl;
      shared_data->wrench[0].store(0.0);
      while (shared_data->force.load() == -2.0) {
        usleep(100000);
      }
    }
    if (current_force != -0.5 && current_force != -1.0 &&
        current_force != -2.0) {
      prev_force = current_force;
    }
    DEBUG_PRINT("Updating pose reference");
    current_z_pose_delta = shared_data->z_pose_delta.load();
    // pose_ref[0] =
    // init_pose_ref[0] + current_z_pose_delta * 0.707;  // change to z_target
    // pose_ref[1] =
    //     init_pose_ref[1] + current_z_pose_delta;  // change to z_target
    pose_ref[2] =
        init_pose_ref[2] + current_z_pose_delta;  // change to z_target

    std::cout << "current_z_pose_delta: " << current_z_pose_delta << std::endl;
    // pose_ref[2] += current_z_pose_delta;
    // set the z_delta to zero
    DEBUG_PRINT("Setting z_pose_delta");
    // shared_data->z_pose_delta.store(0.0);
    // Update the force in the control variable
    // wrench_WTr[2] = -current_force; // z-axis force
    if (!admittance_config.enable_force_control) {
      for (int i = 1; i < 6; i++) {
        wrench_WTr[i] = 0.0;
      }
    }
    // Update robot reference
    DEBUG_PRINT("Setting robot reference");
    controller.setRobotReference(pose_ref, wrench_WTr);

    // Compute the control output
    DEBUG_PRINT("step");
    controller.step(pose_cmd);
    printf("t = %f, wrench: %.4f %.4f %.4f\n", dt, wrench[0], wrench[1],
           wrench[2]);
    DEBUG_PRINT("streaming Cartesian");
    if (!robot.streamCartesian(pose_cmd)) {
      printf("streamCartesian failed\n");
      break;
    }
    DEBUG_PRINT("streaming Cartesian done");
    dt = timer.toc_ms();
    DEBUG_PRINT("dt");
    if (dt > 30000000) {
      std::cout << "Time limit reached. Exiting." << std::endl;
      break;
    }
    robot.rtde_wait_period(t_start);
    DEBUG_PRINT("Waiting for period");
  }

  return 0;
}