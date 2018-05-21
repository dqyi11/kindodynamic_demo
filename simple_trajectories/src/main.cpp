#include <iostream>
#include <Eigen/Dense>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <libherb/Herb.hpp>

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

static const double planningTimeout{5.};
bool herbReal = false;


void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

void moveArmTo(herb::Herb& robot,
               MetaSkeletonStateSpacePtr armSpace,
               const Eigen::VectorXd& goalPos)
{
  auto trajectory = robot.planToConfiguration(
      armSpace, goalPos, planningTimeout, nullptr);

  if (!trajectory)
  {
    throw std::runtime_error("failed to find a solution");
  }

  auto smoothTrajectory = robot.smoothTrajectory(
      std::move(trajectory), true, true, nullptr);

  robot.executeTrajectory(std::move(smoothTrajectory)).wait();
}


int main(int argc, char** argv)
{
  // Default options for flags
  int target = 1;
  herbReal = false;

  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()
    ("help", "Produce help message")
    ("herbreal,h", po::bool_switch(&herbReal), "Run HERB in real")
    ("target,t", po::value<int>(&target)->default_value(1), "A target trajectory to execute")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    std::cout << "target 0: closing hands" << std::endl
              << "target 1: opening hands" << std::endl
              << "target 2: move arms to relaxed home positions" << std::endl
              << "target 3: move arms to menacing positions" << std::endl;
    return 0;
  }

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_trajectories");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("simple_trajectories"));

  // Load HERB either in simulation or real based on arguments
  ROS_INFO("Loading HERB.");
  herb::Herb robot(env, !herbReal);
  SkeletonPtr robotSkeleton = robot.getSkeleton();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_trajectories";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << execTopicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, execTopicName, baseFrameName);

  // Add HERB to the viewer.
  viewer.setAutoUpdate(true);

  auto leftArmSpace
      = std::make_shared<MetaSkeletonStateSpace>(robot.getLeftArm());
  auto rightArmSpace
      = std::make_shared<MetaSkeletonStateSpace>(robot.getRightArm());

  // Predefined positions ////////////////////////////////////////////////////

  Eigen::VectorXd leftHighHome(7);
  leftHighHome << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
  Eigen::VectorXd rightHighHome(7);
  rightHighHome << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
  
  Eigen::VectorXd leftHigherRelaxedHome(7);
  leftHigherRelaxedHome << 0.64, -1.50, 0.26, 1.96, 1.16, 0.87, 1.43;
  Eigen::VectorXd rightHigherRelaxedHome(7);
  rightHigherRelaxedHome << 5.65, -1.50, -0.26, 1.96, -1.15, 0.87, -1.43;

  Eigen::VectorXd leftRelaxedHome(7);
  leftRelaxedHome  << 0.47, -1.91, 0.53, 1.24, 0.34, 1.25, -0.58;
  Eigen::VectorXd rightRelaxedHome(7);
  rightRelaxedHome << 5.81, -1.91, -0.53, 1.24, -0.34, 1.25, 0.58;

  Eigen::VectorXd leftMenacing(7);
  leftMenacing << 2.60, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;
  Eigen::VectorXd rightMenacing(7);
  rightMenacing << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;

  Eigen::VectorXd neckPositionZero(2);
  neckPositionZero << 0, 0;
  Eigen::VectorXd neckPositionRelaxed(2);
  neckPositionRelaxed << 0, 10;

  waitForUser("Press [ENTER] to start: ");

  if (target == 0) // target 0: close hands //////////////////////////////////
  {
    robot.getLeftHand()->executePreshape("closed").wait();
    robot.getRightHand()->executePreshape("closed").wait();
    waitForUser("Press [ENTER] to exit: ");
    return 0;
  }
  else if (target == 1) // target 1: open hands //////////////////////////////
  {
    robot.getLeftHand()->executePreshape("open").wait();
    robot.getRightHand()->executePreshape("open").wait();
    waitForUser("Press [ENTER] to exit: ");
    return 0;
  }

  if (herbReal)
  {
    robot.switchFromGravityCompensationControllersToTrajectoryExecutors();
  }

  if (target == 2)  // target 2: move arms to relaxed home ///////////////////
  {
    ROS_INFO("Moving the left arm to relaxed home");
    moveArmTo(robot, leftArmSpace, leftRelaxedHome);
    ROS_INFO("Moving the right arm to relaxed home");
    moveArmTo(robot, rightArmSpace, rightRelaxedHome);
    
    ROS_INFO("Opening both hands");
    robot.getLeftHand()->executePreshape("open").wait();
    robot.getRightHand()->executePreshape("open").wait();

    ROS_INFO("Moving neck to (pan: 0, tilt: 10)");
    robot.setNeckPosition(neckPositionRelaxed);
    
    waitForUser("Press key to reset.");
    robot.setNeckPosition(neckPositionZero);
    moveArmTo(robot, leftArmSpace, leftHighHome);
    moveArmTo(robot, rightArmSpace, rightHighHome);
    waitForUser("Press [ENTER] to exit: ");
  }
  else if (target == 3) // target 3: move arms to menacing positions /////////
  {
    ROS_INFO("Moving the left arm to menacing position");
    moveArmTo(robot, leftArmSpace, leftMenacing);
    ROS_INFO("Moving the right arm to menacing position");
    moveArmTo(robot, rightArmSpace, rightMenacing);
    
    ROS_INFO("Opening both hands");
    robot.getLeftHand()->executePreshape("open").wait();
    robot.getRightHand()->executePreshape("open").wait();

    ROS_INFO("Moving neck to (pan: 0, tilt: 10)");
    robot.setNeckPosition(neckPositionRelaxed);
    
    waitForUser("Press key to reset.");
    robot.setNeckPosition(neckPositionZero);
    moveArmTo(robot, leftArmSpace, leftHighHome);
    moveArmTo(robot, rightArmSpace, rightHighHome);
    waitForUser("Press [ENTER] to exit: ");
  }

  if (herbReal)
  {
    robot.switchFromTrajectoryExecutorsToGravityCompensationControllers();
  }

  return 0;
}
