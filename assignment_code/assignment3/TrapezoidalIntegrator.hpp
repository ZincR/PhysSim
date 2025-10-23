#ifndef TRAPEZOIDAL_INTEGRATOR_H_
#define TRAPEZOIDAL_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class TrapezoidalIntegrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    auto f_0 = system.ComputeTimeDerivative(state, start_time);
    auto f_1 = system.ComputeTimeDerivative(dt*f_0+state, start_time+dt);
    return state + dt/2 * (f_0 + f_1);
  }
};
}  // namespace GLOO

#endif
