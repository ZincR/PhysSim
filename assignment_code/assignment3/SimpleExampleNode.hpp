#include <memory>
#include "gloo/SceneNode.hpp"
#include "IntegratorType.hpp"
#include "IntegratorBase.hpp"
#include "ParticleState.hpp"
#include "IntegratorBase.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO {

class SimpleExampleNode : public GLOO::SceneNode {
    public:
        SimpleExampleNode(IntegratorType type, float dt);
        void Update(double delta_time) override;
    private:
        std::unique_ptr<IntegratorBase<ParticleSystemBase, ParticleState>> integrator_;
        ParticleState state_;
        float dt_;
        float t_{0.0f};
};
}  // namespace GLOO