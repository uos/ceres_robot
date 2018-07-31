#ifndef moveback_recovery_
#define moveback_recovery_

#include <mbf_costmap_core/costmap_recovery.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;

namespace moveback_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class MoveBackRecovery : public mbf_costmap_core::CostmapRecovery
{
public:
  
    /// Doesn't do anything: initialize is where the actual work happens
    MoveBackRecovery()
      : local_costmap_(NULL),
        tf_(NULL),
        initialized_(false),
        canceled_(true)
    {
        zero_twist_.linear.x = 0.0;
        zero_twist_.linear.y = 0.0;
        zero_twist_.linear.z = 0.0;
        zero_twist_.angular.x = 0.0;
        zero_twist_.angular.y = 0.0;
        zero_twist_.angular.z = 0.0;

    }

    /// Initialize the parameters of the behavior
    virtual void initialize (std::string n, tf::TransformListener* tf,
                     costmap_2d::Costmap2DROS* global_costmap,
                     costmap_2d::Costmap2DROS* local_costmap);

    /// Run the behavior
    virtual uint32_t runBehavior(std::string& message);

    virtual bool cancel();

    virtual ~MoveBackRecovery() { };

    private:
    gm::Pose2D getCurrentRobotPose() const;
    uint32_t moveBack() const;
    uint32_t publishStop() const;
    double getCurrentDiff(const gm::Pose2D referencePose) const;

    ros::NodeHandle nh_;
    costmap_2d::Costmap2DROS* local_costmap_;
    tf::TransformListener* tf_;
    ros::Publisher cmd_vel_pub_;
    bool initialized_;
    bool canceled_;

    gm::Twist zero_twist_;

    double controller_frequency_;
    double linear_vel_back_;
    double step_back_length_;
    double step_back_timeout_;
};

} // namespace moveback_recovery

#endif // moveback_recovery_