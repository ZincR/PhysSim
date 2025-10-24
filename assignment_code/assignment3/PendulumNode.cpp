#include "PendulumNode.hpp"

#include <algorithm>
#include <memory>
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"

namespace GLOO {

PendulumNode::PendulumNode(IntegratorType type, float dt) : dt_(dt) {
    const float m = 1.0f, L0 = 0.3f, k = 100.0f, N = 6;

    state_.positions.resize(N);
    state_.velocities.resize(N);
    for (int i = 0; i < N; ++i) {
        state_.positions[i]  = {1.0f, 0.0f - i * L0, 0.0f};
        state_.velocities[i] = {0.0f, 0.0f, 0.0f};
        system_.AddParticle(m, false);
    }
    system_.Fix(0, true);
    for (int i = 0; i < N-1; ++i) {
        system_.AddSpring(i, i+1, k, L0);

    }

    // 4 small spheres
    auto shader = std::make_shared<PhongShader>();
    particles_.clear();
    for (int i = 0; i < N; ++i) {
        auto child = make_unique<SceneNode>();
        auto sphere_vo = std::shared_ptr<VertexObject>(PrimitiveFactory::CreateSphere(0.1f, 16, 12).release());
        child->CreateComponent<RenderingComponent>(sphere_vo);
        child->CreateComponent<ShadingComponent>(shader);
        child->GetTransform().SetPosition(state_.positions[i]);
        particles_.push_back(child.get());
        AddChild(std::move(child));
    }
    integrator_ = IntegratorFactory::CreateIntegrator<ParticleSystemBase, ParticleState>(type);
}

void PendulumNode::Update(double delta_time) {
    float remain = static_cast<float>(delta_time);
    while (remain > 0.0f) {
        const float h = std::min(dt_, remain);
        state_ = integrator_->Integrate(system_, state_, t_, h);
        t_ += h;
        remain -= h;
    }
    for (size_t i = 0; i < particles_.size(); ++i)
        particles_[i]->GetTransform().SetPosition(state_.positions[i]);
}

} // namespace GLOO