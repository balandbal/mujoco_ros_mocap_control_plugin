#include <mujoco_ros_mocap_control_plugin/mocap_control_plugin.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_ros_mocap_control_plugin {

MocapControlPlugin::~MocapControlPlugin() {}

bool MocapControlPlugin::load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	ROS_INFO_STREAM_NAMED("mujoco_ros_mocap_control_plugin", "Loading mujoco_ros_mocap_control_plugin plugin ...");

	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	// TODO: get these from the parameter server
	auto feeler_name = "robot_flange";
	auto mocap_name = "robot_flange_mocap";

	// Get body ids from body names
	feeler_id_ = mj_name2id(m.get(), mjOBJ_BODY, feeler_name);
	if (feeler_id_ == -1) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "Found no body named '" << feeler_name << "' in MJCF");
		return false;
	}
	auto mocap_body_id = mj_name2id(m.get(), mjOBJ_BODY, mocap_name);
	if (mocap_body_id == -1) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "Found no body named '" << mocap_name << "' in MJCF");
		return false;
	}

	// Get the id of the mocap body in the mocap-relevant lists
	mocap_id_ = m->body_mocapid[mocap_body_id];
	if (mocap_id_ == -1) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "The body named '" << mocap_name << "' is not a mocap body. Set the attribute 'mocap=\"true\"' your MJCF");
		return false;
	}

	// Translate ids to list indeces
	feeler_pos_id_  = feeler_id_ * 3;
	feeler_quat_id_ = feeler_id_ * 4;
	mocap_pos_id_   = mocap_id_ * 3;
	mocap_quat_id_  = mocap_id_ * 4;

	// if there is only one mocap body, use pointers to sync poses
	only_one_mocap_in_mjcf_ = m->nmocap == 1;
	if (only_one_mocap_in_mjcf_) {
		ROS_INFO_NAMED("mujoco_ros_mocap_control_plugin", "There is only one mocap body in the MJCF; using pointers instead of copying");
		d->mocap_pos  = &(d->xpos[feeler_id_ * 3]);
		d->mocap_quat = &(d->xquat[feeler_id_ * 4]);
	}

	ROS_INFO("Loaded mujoco_ros_mocap_control_plugin");
	return true;
}

void MocapControlPlugin::controlCallback(MujocoSim::mjModelPtr /*model*/, MujocoSim::mjDataPtr data)
{
	ros::Time sim_time_ros = ros::Time::now();

	ROS_WARN_STREAM_COND_NAMED(sim_time_ros < ros::Time(data->time), "mujoco_ros_mocap_control_plugin",
	                           "ROS time not in sync with mjData! (" << sim_time_ros << " < " << ros::Time(data->time)
	                                                                 << ")");
	if (sim_time_ros < last_update_sim_time_ros_) {
		ROS_INFO_NAMED("mujoco_ros_mocap_control_plugin", "Resetting mujoco_ros_mocap_control_plugin due to time reset");
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "sim time is " << sim_time_ros << " while last time was " << last_update_sim_time_ros_);
		last_update_sim_time_ros_ = sim_time_ros;
		last_write_sim_time_ros_  = sim_time_ros;
		ROS_WARN_STREAM_COND_NAMED(sim_time_ros < ros::Time::now(), "mujoco_ros_mocap_control_plugin",
		                           "Current time moved forward within control update! " << sim_time_ros << " -> "
		                                                                                << ros::Time::now());
	}

	if (!only_one_mocap_in_mjcf_) {
		for (unsigned int i = 0; i < 3; ++i) {
			data->mocap_pos[mocap_pos_id_ + i] = data->xpos[feeler_id_ * 3 + i];
		}
		for (unsigned int i = 0; i < 4; ++i) {
			data->mocap_quat[mocap_quat_id_ + i] = data->xquat[feeler_id_ * 4 + i];
		}
	}
}

void MocapControlPlugin::reset() {}

} // mujoco_ros_mocap_control_plugin

PLUGINLIB_EXPORT_CLASS(mujoco_ros_mocap_control_plugin::MocapControlPlugin, MujocoSim::MujocoPlugin)
