#ifndef RK4_INTEGRATOR_H_
#define RK4_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {

template <class TSystem, class TState>
class RK4Integrator : public IntegratorBase<TSystem, TState> {
    public:
        TState Integrate(const TSystem& system,
                        const TState& state,
                        float start_time,
                        float dt) const override {
            // k1 = f(x, t)
            // k2 = f(x + dt/2 k1, t + dt/2)
            // k3 = f(x + dt/2 k2, t + dt/2)
            // k4 = f(x + dt k3,   t + dt)
            // x_{n+1} = x + dt/6 (k1 + 2k2 + 2k3 + k4)
            auto k1 = system.ComputeTimeDerivative(state, start_time);
            auto k2 = system.ComputeTimeDerivative(state + 0.5f * dt * k1, start_time + 0.5f * dt);
            auto k3 = system.ComputeTimeDerivative(state + 0.5f * dt * k2, start_time + 0.5f * dt);
            auto k4 = system.ComputeTimeDerivative(state + dt * k3, start_time + dt);
            return state + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
    }
};

}  // namespace GLOO
#endif