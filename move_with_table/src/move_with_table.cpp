#include <iostream>
#include <Eigen/Dense>
#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/perception/ObjectDatabase.hpp>
#include <aikido/perception/RcnnPoseModule.hpp>
#include <aikido/perception/YamlAprilTagsDatabase.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pr_tsr/can.hpp>
#include <libherb/herb.hpp>
#include <magi/action/Action.hpp>
#include <magi/action/SequenceAction.hpp>
#include <magi/action/WaitForInputAction.hpp>

namespace po = boost::program_options;

using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;

using aikido::constraint::CollisionFree;
using aikido::statespace::dart::MetaSkeletonStateSpace;

static const std::string dartTopicName("dart_markers");
static const std::string aprilTagMarkerTopicName("/apriltags/marker_array");
static const std::string rcnnPoseMarkerTopicName("/rcnn_demo/marker_array");
static const std::string aprilTagDataURI(
    "package://pr_ordata/data/objects/tag_data_apriltags.json");
static const std::string rcnnPoseDataURI(
    "package://pr_ordata/data/objects/tag_data_rcnnpose.json");
static const std::string herbFrameName("herb_frame");
static const std::string tableName("table127");
static const std::string baseFrameName("map");

static const double planningTimeout{5.};
static const double detectionTimeout{5.};
static const int tsrPlanningTrials{20};
static const double tsrPlanningTimeout{30.};

/* Vector-Field Planner Parameters (TODO: tune for each step) */

static const int eePlanningTrials{10};
static const double vfPositionTolerance{0.01};
static const double vfAngularTolerance{0.1};
static const double vfLinearGain{0.3};
static const double vfAngularGain{0.1};
static const double vfTimestep{0.1};

static const Eigen::IOFormat fmt(3, 0, " ", " "); // for debugging

using magi::action::ActionPtr;

const SkeletonPtr makeBodyFromURDF(
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  dart::utils::DartLoader urdfLoader;
  const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);
  return skeleton;
}

/// The command-line interface for this demo.
/// \param[in]  argc Number of arguments
/// \param[in]  argv Array of arguments
/// \param[out] herbSim Whether HERB should be run in simulation
/// \param[out] perceptionSim Whether perception should be run in simulation
/// \return whether the script should continue
bool cli(int argc, char** argv, bool& herbSim, bool& perceptionSim)
{
  // Command-line interface
  bool herbReal = false;
  bool perceptionReal = false;

  // clang-format off
  po::options_description po_desc("Soda handoff demo");
  po_desc.add_options()
    ("help", "produce help message")
    ("herbreal,h", po::bool_switch(&herbReal), "Run HERB in real")
    ("perceptionreal,p", po::bool_switch(&perceptionReal), "Run perception in real")
  ;
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    return true;
  }

  herbSim = !herbReal;
  perceptionSim = !perceptionReal;
  return false;
}

/// Find a table with AprilTags
/// \param[in]  perceptionSim Whether perception should be simulated
/// \param[in]  nh ROS node handle to use for AprilTag detection
/// \param[in]  resourceRetriever Resource retriever to resolve package URIs
/// \param[in]  robot Robot that the AprilTag transforms are defined relative to
/// \param[out] table Perceived table skeleton
/// \param[out] tablePose Perceived table pose
void perceiveTable(
    bool perceptionSim,
    ros::NodeHandle nh,
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    herb::Herb& robot,
    dart::dynamics::SkeletonPtr& table,
    Eigen::Isometry3d& tablePose)
{
  perceptionSim = true; // HACK: just set the table position in the real world
  if (perceptionSim)
  {
    const std::string tableURDFUri(
        "package://pr_ordata/data/furniture/uw_demo_table.urdf");

    // Poses for table
    tablePose = Eigen::Isometry3d::Identity();
    // tablePose.translation() = Eigen::Vector3d(1.0, 0.4, 0);
    tablePose.translation() = Eigen::Vector3d(1.0, 0.0, 0);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    tablePose.linear() = rot;

    // Load table
    table = makeBodyFromURDF(resourceRetriever, tableURDFUri, tablePose);

    // Add all objects to World
    robot.getWorld()->addSkeleton(table);
  }
  else
  {
    // Load and run AprilTags Detector from aikido/perception
    using aikido::perception::YamlAprilTagsDatabase;

    dart::dynamics::BodyNode* herbBaseNode
        = robot.getSkeleton()->getBodyNode(herbFrameName);

    std::shared_ptr<YamlAprilTagsDatabase> yamlLoader(
        new YamlAprilTagsDatabase(resourceRetriever, aprilTagDataURI));
    aikido::perception::AprilTagsModule atDetector(
        nh,
        aprilTagMarkerTopicName,
        yamlLoader,
        resourceRetriever,
        herbFrameName,
        herbBaseNode);

    atDetector.detectObjects(robot.getWorld(), ros::Duration(detectionTimeout));

    // Get skeletons of interest using names from robot's World
    table = robot.getWorld()->getSkeleton(tableName);
    // TODO : Is this the correct way to get object pose?
    tablePose = table->getJoint(0)->getChildBodyNode()->getTransform();
  }
}


int main(int argc, char** argv)
{
  // Parse command-line options
  bool herbSim, perceptionSim;
  if (cli(argc, argv, herbSim, perceptionSim))
    return 0;

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "soda_handoff");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("world"));

  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  // Load HERB either in simulation or real based on arguments
  ROS_INFO("Loading HERB.");
  herb::Herb robot(env, herbSim);

  // Initial table perception
  SkeletonPtr table;
  Eigen::Isometry3d tablePose;
  perceiveTable(perceptionSim, nh, resourceRetriever, robot, table, tablePose);

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << dartTopicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, dartTopicName, baseFrameName);
  viewer.setAutoUpdate(true);

  if (!herbSim)
    robot.switchFromGravityCompensationControllersToTrajectoryExecutors();

  // Set up planning spaces and collision constraints
  auto leftArmSpace
      = std::make_shared<MetaSkeletonStateSpace>(robot.getLeftArm());
  auto rightArmSpace
      = std::make_shared<MetaSkeletonStateSpace>(robot.getRightArm());

  CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<CollisionGroup> rightArmGroup
      = collisionDetector->createCollisionGroup(
          rightArmSpace->getMetaSkeleton().get(),
          robot.getRightHand()->getHand().get());
  std::shared_ptr<CollisionGroup> leftArmGroup
      = collisionDetector->createCollisionGroup(
          leftArmSpace->getMetaSkeleton().get(),
          robot.getLeftHand()->getHand().get());
  std::shared_ptr<CollisionGroup> envGroup
      = collisionDetector->createCollisionGroup(table.get());

  auto rightCollisionFreeConstraint
      = std::make_shared<CollisionFree>(rightArmSpace, collisionDetector);
  rightCollisionFreeConstraint->addPairwiseCheck(rightArmGroup, envGroup);

  // Set up start configurations
  Eigen::VectorXd rightStartConfiguration(7);
  rightStartConfiguration << 3.14 + 0.54, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;

  Eigen::VectorXd leftStartConfiguration(7);
  leftStartConfiguration << 3.14 - 0.54, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;

  robot.getRightHand()->executePreshape("partial_open");

  if (herbSim)
  {
    robot.setConfiguration(rightArmSpace, rightStartConfiguration);
    robot.setConfiguration(leftArmSpace, leftStartConfiguration);
  }
  else
  {
    ROS_INFO("Planning to right start.");
    auto rightStartAction = robot.getPlanToConfigurationAction(
        rightArmSpace,
        rightStartConfiguration,
        planningTimeout,
        rightCollisionFreeConstraint);

    auto rightStartSolution = rightStartAction->plan(env, robot.cloneRNG().get());

    if (!rightStartSolution)
      throw std::runtime_error("failed to find a solution");

    ROS_INFO("Smoothing right start trajectory.");
    auto rightStartExecutable = rightStartSolution->postprocess(env, robot.cloneRNG().get());

    ROS_INFO("Executing right start trajectory.");
    rightStartExecutable->execute(env);
  }


  ROS_INFO_STREAM("Done.");

  if (herbReal)
     robot.switchFromTrajectoryExecutorsToGravityCompensationControllers();

  return 0;
}
