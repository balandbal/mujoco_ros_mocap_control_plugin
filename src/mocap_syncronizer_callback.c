#include <mujoco/mujoco.h>

void mocap_syncronizer_callback(const mjModel* model, mjData* data)
{
    int body_id = mj_name2id(model, mjOBJ_BODY, "mocap_syncronizer:body");
    int mocap_id = model->body_mocapid[mj_name2id(model, mjOBJ_BODY, "mocap_syncronizer:mocap")];
    
    data->mocap_pos[mocap_id * 3]     = data->xpos[body_id * 3];
    data->mocap_pos[mocap_id * 3 + 1] = data->xpos[body_id * 3 + 1];
    data->mocap_pos[mocap_id * 3 + 2] = data->xpos[body_id * 3 + 2];

    data->mocap_quat[mocap_id * 4]     = data->xquat[body_id * 4];
    data->mocap_quat[mocap_id * 4 + 1] = data->xquat[body_id * 4 + 1];
    data->mocap_quat[mocap_id * 4 + 2] = data->xquat[body_id * 4 + 2];
    data->mocap_quat[mocap_id * 4 + 3] = data->xquat[body_id * 4 + 3];
}
