#include <mujoco_ros_mocap_control_plugin/mocap_control_plugin.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_ros_mocap_control_plugin {

MocapControlPlugin::~MocapControlPlugin() {}

bool MocapControlPlugin::load(const mjModel *model, mjData *data)
{
	ROS_INFO_STREAM_NAMED("mujoco_ros_mocap_control_plugin", "Loading mujoco_ros_mocap_control_plugin plugin ...");

	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	// TODO: handle robot prefix?
	std::string robot_prefix = "robot_";
	std::string feeler_name = robot_prefix + "flange";
	std::string mocap_name = robot_prefix + "flange_mocap";

	if (rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		if (rosparam_config_.hasMember("mocap_mapping")) {
			if (rosparam_config_["mocap_mapping"].getType() != XmlRpc::XmlRpcValue::TypeArray) {
				ROS_ERROR_NAMED("mujoco_ros_mocap_control_plugin", "The 'mocap_mapping' param for MocapControlPlugin must define an array");
				return false;
			}

			if (rosparam_config_["mocap_mapping"].size() != 0) {
				
				// TODO: vectorize functionality for multiple pairs of mocaps and bodies

				if (!rosparam_config_["mocap_mapping"][0].hasMember("body_name") or !rosparam_config_["mocap_mapping"][0].hasMember("mocap_name")) {
					ROS_ERROR_NAMED("mujoco_ros_mocap_control_plugin", "The elements of 'mocap_mapping' must have a 'body_name' and a 'mocap_name' key defined");
					return false;
				}
				if (rosparam_config_["mocap_mapping"][0]["body_name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
					ROS_ERROR_NAMED("mujoco_ros_mocap_control_plugin", "The 'body_name' params for MocapControlPlugin must define a string");
					return false;
				}
				if (rosparam_config_["mocap_mapping"][0]["mocap_name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
					ROS_ERROR_NAMED("mujoco_ros_mocap_control_plugin", "The 'mocap_name' params for MocapControlPlugin must define a string");
					return false;
				}

				feeler_name = (std::string)rosparam_config_["mocap_mapping"][0]["body_name"];
				ROS_INFO_STREAM_NAMED("mujoco_ros_mocap_control_plugin", "Using body name '" << feeler_name << "'");
				mocap_name = (std::string)rosparam_config_["mocap_mapping"][0]["mocap_name"];
				ROS_INFO_STREAM_NAMED("mujoco_ros_mocap_control_plugin", "Using mocap name '" << mocap_name << "'");
				}
			else
				ROS_WARN_NAMED("mujoco_ros_mocap_control_plugin", "The 'mocap_mapping' param for MocapControlPlugin has a size of 0. Ignoring it");
		}
	}

	// Get body ids from body names
	feeler_id_ = mj_name2id(const_cast<mjModel *>(model), mjOBJ_BODY, feeler_name.c_str());
	if (feeler_id_ == -1) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "Found no body named '" << feeler_name << "' in MJCF");
		return false;
	}
	auto mocap_body_id = mj_name2id(const_cast<mjModel *>(model), mjOBJ_BODY, mocap_name.c_str());
	if (mocap_body_id == -1) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_mocap_control_plugin",
		                       "Found no body named '" << mocap_name << "' in MJCF");
		return false;
	}

	// Get the id of the mocap body in the mocap-relevant lists
	mocap_id_ = model->body_mocapid[mocap_body_id];
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
	only_one_mocap_in_mjcf_ = model->nmocap == 1;
	if (only_one_mocap_in_mjcf_) {
		ROS_INFO_NAMED("mujoco_ros_mocap_control_plugin", "There is only one mocap body in the MJCF; using pointers instead of copying");
		data->mocap_pos  = &(data->xpos[feeler_id_ * 3]);
		data->mocap_quat = &(data->xquat[feeler_id_ * 4]);
	}

	ROS_INFO("Loaded mujoco_ros_mocap_control_plugin");
	return true;
}

void MocapControlPlugin::controlCallback(const mjModel */*model*/, mjData *data)
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

PLUGINLIB_EXPORT_CLASS(mujoco_ros_mocap_control_plugin::MocapControlPlugin, mujoco_ros::MujocoPlugin)
