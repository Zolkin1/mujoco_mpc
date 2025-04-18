//
// Created by zolkin on 4/18/25.
//

#include "g1.h"

#include "mjpc/utilities.h"

namespace mjpc
{
    std::string G1::XmlPath() const {
        return GetModelPath("g1/task.xml");   // TODO: Change the path
    }
    std::string G1::Name() const { return "G1"; }

    void G1::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                        double* residual) const {
        int counter = 0;

        // Get the base position
        double base_pos[3];
        mju_copy(base_pos, data->qpos, 3);

        double* goal_pos = data->mocap_pos;

        for (int i = 0; i < 3; i++) {
            residual[counter] = goal_pos[i] - base_pos[i];
            counter++;
        }
    }

    void G1::TransitionLocked(mjModel* model, mjData* data) {
        // TODO: Here is where I can replicate the trajectory when the robot reaches the previous one's end
    }
}
