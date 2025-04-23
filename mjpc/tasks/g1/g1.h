//
// Created by zolkin on 4/18/25.
//

#ifndef G1_H
#define G1_H
#include "mjpc/task.h"

namespace mjpc
{
    class G1 : public Task {
    public:
        std::string Name() const override;
        std::string XmlPath() const override;
        class ResidualFn : public mjpc::BaseResidualFn {
        public:
            explicit ResidualFn(const G1* task) : mjpc::BaseResidualFn(task) {}

            void Residual(const mjModel* model, const mjData* data,
                          double* residual) const override;
        };

        G1() : residual_(this) {}
        void TransitionLocked(mjModel* model, mjData* data) override;

    protected:
        std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
            return std::make_unique<ResidualFn>(this);
        }
        ResidualFn* InternalResidual() override { return &residual_; }

        static void JointTracking(const mjModel* model, const mjData* data, double* joint_target,
            double* residual, int& resid_idx);

        static std::vector<double> GetMocapReference(double time, const std::vector<std::vector<double>>& reference_data);

        static double GetDesFootPos(int l_or_r, double time);

        static double ComputeSpline(double time, double total_time);

    private:
        ResidualFn residual_;
        static constexpr int JOINT_DIM = 29;
    };
}

#endif //G1_H
