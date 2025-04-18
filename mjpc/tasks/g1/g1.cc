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

        // ---------- Position Tracking ---------- //
        double base_pos[3];
        mju_copy(base_pos, data->qpos, 3);

        double* goal_pos = data->mocap_pos;

        for (int i = 0; i < 3; i++) {
            residual[counter] = goal_pos[i] - base_pos[i];
            counter++;
        }

        // ---------- Quaternion Tracking ---------- //
        residual[counter] = data->qpos[3] - 1;
        counter++;
        for (int i = 0; i < 3; i++) {
            residual[counter] = data->qpos[4 + i];
            counter++;
        }

        // ---------- Joint Tracking ---------- //
        double target[G1::JOINT_DIM] = {0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0,
              0, 0, 0,
              0.2, 0.2, 0, 1.28, 0, 0, 0,
              0.2, -0.2, 0, 1.28, 0, 0, 0};

        JointTracking(model, data, target, residual, counter);

        // ---------- Base Velocity Tracking ---------- //
        for (int i = 0; i < 6; i++) {
            residual[counter] = data->qvel[i];
            counter++;
        }

        // ---------- Joint Velocity Tracking/Regularization ---------- //
        for (int i = 0; i < JOINT_DIM; i++) {
            residual[counter] = data->qvel[i + 6];
            counter++;
        }

        // ---------- Control Regularization ---------- //
        for (int i = 0; i < JOINT_DIM; i++) {
            residual[counter] = data->ctrl[i];
            counter++;
        }

    }

    void G1::JointTracking(const mjModel* model, const mjData* data, double* joint_target, double* residual, int& resid_idx) {
        // Get the joint configuration.
        for (int i = 0; i < JOINT_DIM; i++) {
            residual[i + resid_idx] = joint_target[i] - data->qpos[i + 7];
        }

        resid_idx += JOINT_DIM;
    }

    void G1::TransitionLocked(mjModel* model, mjData* data) {
        // TODO: Here is where I can replicate the trajectory when the robot reaches the previous one's end
    }
}
