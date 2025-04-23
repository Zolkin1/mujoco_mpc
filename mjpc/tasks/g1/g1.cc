//
// Created by zolkin on 4/18/25.
//

#include <fstream>

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
        static bool first_eval = true;
        static std::vector<std::vector<double>> ref_data;
        bool mocap_tracking = false;

        if (mocap_tracking && first_eval) {
            first_eval = false;
            static int load_counter = 0;
            std::cout << "Load counter: " << load_counter++ << std::endl;
            // Load the reference data
            const std::string path = "/home/zolkin/AmberLab/datasets/LAFAN1_Retargeting_Dataset/g1/walk1_subject1.csv";
            std::ifstream file(path);
            if (!file.is_open()) {
                throw std::runtime_error("Error opening file!");
            }

            double value;
            char delimiter;
            std::vector<double> row;

            while (file >> value) {
                row.push_back(value);
                if (file.peek() == ',') {
                    file >> delimiter; // Consume the comma
                } else if (file.peek() == '\n' || file.eof()) {
                    ref_data.push_back(row);
                    row.clear();
                }
            }

            file.close();

            // Modify its quaternion information
            for (int i = 0; i < ref_data.size(); i++) {
                std::array<double, 4> xyzw = {ref_data[i][3], ref_data[i][4], ref_data[i][5], ref_data[i][6]};
                ref_data[i][3] = xyzw[3];
                for (int j = 0; j < 3; j++) {
                    ref_data[i][4 + j] = xyzw[j];
                }
            }

            std::cout << "Loaded " << ref_data.size() << " rows from " << path << std::endl;
        }

        int counter = 0;

        if (!mocap_tracking) {
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
            // double target[G1::JOINT_DIM] = {0, 0, 0, 0, 0, 0,
            //       0, 0, 0, 0, 0, 0,
            //       0, 0, 0,
            //       0.2, 0.2, 0, 1.28, 0, 0, 0,
            //       0.2, -0.2, 0, 1.28, 0, 0, 0};
            double target[G1::JOINT_DIM] = {
                -0.42, 0, 0, 0.81, -0.4, 0,
                -0.42, 0, 0, 0.81, -0.4, 0,
                0, 0, 0,
                0, 0.27, 0, 0.5, 0, 0, 0,
                0, -0.27, 0, 0.5, 0, 0, 0};

            JointTracking(model, data, target, residual, counter);

            // ---------- Foot Height Tracking ---------- //
            // Get the left and right foot position
            const char* left_foot_pos = "left_ankle";
            const char* right_foot_pos = "right_ankle";
            // int left_id = mj_name2id(model, mjOBJ_SENSOR, left_foot_pos);
            // int right_id = mj_name2id(model, mjOBJ_SENSOR, right_foot_pos);

            double* left_pos = SensorByName(model, data, left_foot_pos);
            double* right_pos = SensorByName(model, data, right_foot_pos);

            // double left_pos[3];
            // double right_pos[3];
            // for (int i = 0; i < 3; i++) {
            //     left_pos[i] = data->sensordata[left_id + i];
            //     right_pos[i] = data->sensordata[right_id + i];
            // }

            for (int i = 0; i < 3; i++) {
                if (i == 2) {
                    std::cout << "Left: " << left_pos[i] << " Right: " << right_pos[i] << std::endl;
                    residual[counter] = 0; //left_pos[i] - GetDesFootPos(0, data->time);
                    counter++;

                    residual[counter] = right_pos[i] - GetDesFootPos(1, data->time);
                    counter++;
                } else {
                    residual[counter] = left_pos[i] - 0.1;    // TODO: Make this an offset relative to the base
                    counter++;

                    residual[counter] = right_pos[i] + 0.1;   // TOOD: See above
                    counter++;
                }
            }
        } else {
            // Works best with a longer horizon (about a second) and 0.02s discretization
            std::vector<double> reference = GetMocapReference(data->time, ref_data);

            // ---------- Position Tracking ---------- //
            double base_pos[3];
            mju_copy(base_pos, data->qpos, 3);

            for (int i = 0; i < 3; i++) {
                residual[counter] = reference[i] - base_pos[i];
                counter++;
            }

            // ---------- Quaternion Tracking ---------- //
            for (int i = 0; i < 4; i++) {
                residual[counter] = data->qpos[3 + i] - reference[3 + i];
                counter++;
            }

            // ---------- Joint Tracking ---------- //
            double joint_ref[JOINT_DIM];
            for (int i = 0; i < JOINT_DIM; i++) {
                joint_ref[i] = reference[i + 7];
            }
            JointTracking(model, data, joint_ref, residual, counter);

            // ---------- Foot Height Tracking ---------- //
            for (int i = 0; i < 6; i++) {
                residual[counter] = 0;
                counter++;
            }
        }

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

    std::vector<double> G1::GetMocapReference(double time, const std::vector<std::vector<double>>& reference_data) {
        // Start the motion after 5 seconds
        if (time < 10.0) {
            return reference_data[0];
        }

        // Get the index of the appropriate data
        // The data is assumed to be recorded at 30 fps
        double fps = 30.0;
        int index = std::floor((time - 10.0) * fps);

        // TODO: Deal with an index that is too large
        // std::cout << "Index: " << index << std::endl;
        // std::cout << "Ref size: " << reference_data.size() << std::endl;

        return reference_data[index];
    }

    void G1::TransitionLocked(mjModel* model, mjData* data) {
        // TODO: Here is where I can replicate the trajectory when the robot reaches the previous one's end
    }

    double G1::GetDesFootPos(int l_or_r, double time) {
        // Determine which foot is on the ground and which is in the air
        // const double swing_time = 0.5;

        // int cycle = std::floor(time / (swing_time*2));

        // double phase_time = time - cycle * swing_time*2;

        return 0.2*sin(time);

        // return 0.5;

        // if (phase_time < swing_time) {
        //     // Left Foot Up
        //     if (l_or_r == 0) {
        //         return 0.06;
        //     }
        //
        //     // Compute position based on a spline
        //     // std::cout << "t: " << phase_time << ", s: " << ComputeSpline(phase_time, swing_time) << std::endl;
        //     return 0.06; //ComputeSpline(phase_time, swing_time);
        // } else {
        //     // Right Foot up
        //     if (l_or_r == 1) {
        //         return 0.06;
        //     }
        //
        //     // Compute position based on a spline
        //     return ComputeSpline(phase_time - swing_time, swing_time);
        // }

        throw std::runtime_error("DesFootPosError!");
    }

    double G1::ComputeSpline(double time, double total_time) {
        const double swing_height = 0.08;
        const double stance_height = 0.06;

        if (time < total_time*0.5) {
            // First spline
            double a0 = stance_height;
            double a1 = 0;
            double a2 = -std::pow(total_time*0.5, -2)*(3*(-swing_height));
            double a3 = std::pow(total_time*0.5, -3)*(2*(-swing_height));

            // std::cout << "time: " << time << ", total time: " << total_time << std::endl;

            return a0 + time*a1 + time*time*a2 + time*time*time*a3;
        } else {
            time = time - total_time*0.5;
            // Second spline
            double a0 = stance_height + swing_height;
            double a1 = 0;
            double a2 = -std::pow(total_time*0.5, -2)*(3*(swing_height));
            double a3 = std::pow(total_time*0.5, -3)*(2*(swing_height));

            return a0 + time*a1 + time*time*a2 + time*time*time*a3;
        }
    }

}
