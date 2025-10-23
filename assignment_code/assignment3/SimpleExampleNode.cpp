#include "SimpleExampleNode.hpp"

#include <algorithm>
#include <glm/vec3.hpp>
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"

#include "ParticleSystemBase.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorFactory.hpp"

namespace GLOO{

class SimpleSystem : public ParticleSystemBase {
    public:
        ParticleState ComputeTimeDerivative(const ParticleState& state, float) const override {
            ParticleState d;
            d.positions.resize(state.positions.size());
            d.velocities.resize(state.velocities.size());
            if (!state.positions.empty()) {
                const glm::vec3 p = state.positions[0];
                d.positions[0]  = glm::vec3(-p.y, p.x, 0.0f);
            }
                if (!state.velocities.empty()) d.velocities[0] = glm::vec3(0.0f);
            return d;
        }
};

SimpleExampleNode::SimpleExampleNode(IntegratorType type, float dt) : dt_(dt) {
    CreateComponent<RenderingComponent>(
        std::shared_ptr<GLOO::VertexObject>(
            PrimitiveFactory::CreateSphere(0.2f, 16, 12).release()));
    auto shader = std::make_shared<PhongShader>();
    CreateComponent<ShadingComponent>(shader);

    state_.positions = {glm::vec3(0.5f, 0.0f, 0.0f)};
    state_.velocities = {glm::vec3(0.0f)};
    GetTransform().SetPosition(state_.positions[0]);
    integrator_ = IntegratorFactory::CreateIntegrator<ParticleSystemBase, ParticleState>(type);
}

void SimpleExampleNode::Update(double delta_time) {
    static SimpleSystem system;
    float remain = static_cast<float>(delta_time); 
    // Take delta_time//dt_ updates
    while (remain > 0.0f) {
        const float h = std::min(dt_, remain);
        state_ = integrator_->Integrate(system, state_, t_, h);
        t_ += h;
        remain -= h;
    }
    GetTransform().SetPosition(state_.positions[0]);
}

}