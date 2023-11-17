#pragma once

#include <ros/ros.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/mujoco_env.h>

namespace mujoco_ros_mocap_control_plugin {

class MocapControlPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MocapControlPlugin() override;

	// Overload entry point
	bool load(const mjModel *model, mjData *data) override;

	// Called on reset
	void reset() override;

	void controlCallback(const mjModel *model, mjData *data) override;

protected:
    int feeler_id_, mocap_id_;
    int feeler_pos_id_, feeler_quat_id_, mocap_pos_id_, mocap_quat_id_;

    bool only_one_mocap_in_mjcf_{false};

    // Timing
    ros::Duration control_period_;
    ros::Time last_update_sim_time_ros_;
    ros::Time last_write_sim_time_ros_;
};

} // namespace mujoco_ros_mocap_control_plugin
