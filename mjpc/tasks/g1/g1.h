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
            // -------- Residuals for particle task -------
            //   Number of residuals: 3
            //     Residual (0): position - goal_position
            //     Residual (1): velocity
            //     Residual (2): control
            // --------------------------------------------
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

    private:
        ResidualFn residual_;
    };
}

#endif //G1_H
