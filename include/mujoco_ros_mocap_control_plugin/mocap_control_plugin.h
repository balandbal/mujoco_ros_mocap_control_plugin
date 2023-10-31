#pragma once

#include <ros/ros.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>

namespace mujoco_ros_mocap_control_plugin {

class MocapControlPlugin : public MujocoSim::MujocoPlugin
{
public:
	virtual ~MocapControlPlugin();

	// Overload entry point
	virtual bool load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d);

	// Called on reset
	virtual void reset();

	void controlCallback(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data);

protected:
    int feeler_id_, mocap_id_;
    int feeler_pos_id_, feeler_quat_id_, mocap_pos_id_, mocap_quat_id_;

    bool only_one_mocap_in_mjcf_{false};

    // Mujoco model and data pointers
    MujocoSim::mjModelPtr m_;
    MujocoSim::mjDataPtr d_;

    // Timing
    ros::Duration control_period_;
    ros::Time last_update_sim_time_ros_;
    ros::Time last_write_sim_time_ros_;
};

} // namespace mujoco_ros_mocap_control_plugin
