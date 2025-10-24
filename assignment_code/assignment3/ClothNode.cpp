#include "ClothNode.hpp"

#include <algorithm>
#include <memory>
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {

ClothNode::ClothNode(IntegratorType type, float dt) : dt_(dt) {
    const float m = 1.0f, L0 = 0.1f, k_st = 3000.0f, k_sh = 3000.0f, k_b = 1500.0f, N = 20;
    auto idx = [N](int i,int j){ return i*N + j; }; // Returns the flattened index of particle, row major

    state_.positions.resize(N*N);
    state_.velocities.assign(N*N, glm::vec3(0));
    for (int i=0;i<N;++i){
    for (int j=0;j<N;++j){
        state_.positions[idx(i,j)] = { j*L0 + 2.0f, 1.0f, i*L0 }; // Right and up
        system_.AddParticle(m, false);
    }
    }
    // fix top corners
    system_.Fix(idx(0,0), true);
    system_.Fix(idx(0,N-1), true);

    // add springs
    for (int i=0;i<N;++i){
    for (int j=0;j<N;++j){
        int a = idx(i,j);
        // structural
        if (i+1<N) system_.AddSpring(a, idx(i+1,j), k_st, L0);
        if (j+1<N) system_.AddSpring(a, idx(i,j+1), k_st, L0);
        // shear
        if (i+1<N && j+1<N) system_.AddSpring(a, idx(i+1,j+1), k_sh, L0*std::sqrt(2.f));
        if (i+1<N && j-1>=0) system_.AddSpring(a, idx(i+1,j-1), k_sh, L0*std::sqrt(2.f));
        // bend
        if (i+2<N) system_.AddSpring(a, idx(i+2,j), k_b, 2*L0);
        if (j+2<N) system_.AddSpring(a, idx(i,j+2), k_b, 2*L0);
    }
    }

    // Render
    auto sphere_shader = std::make_shared<PhongShader>();
    particles_.clear();
    for (int i = 0; i < N*N; ++i) {
        auto child = make_unique<SceneNode>();
        auto sphere_vo = std::shared_ptr<VertexObject>(PrimitiveFactory::CreateSphere(0.025f, 8, 6).release());
        child->CreateComponent<RenderingComponent>(sphere_vo);
        child->CreateComponent<ShadingComponent>(sphere_shader);
        child->GetTransform().SetPosition(state_.positions[i]);
        particles_.push_back(child.get());
        AddChild(std::move(child));
    }

    auto line_shader = std::make_shared<SimpleShader>();
    springs_vo_ = std::make_shared<VertexObject>();
    UpdateSpringPos();
    auto& lines = CreateComponent<RenderingComponent>(springs_vo_);
    lines.SetDrawMode(DrawMode::Lines); // (0, 1), (2, 3), ...
    CreateComponent<ShadingComponent>(line_shader);

    // Integrate
    integrator_ = IntegratorFactory::CreateIntegrator<ParticleSystemBase, ParticleState>(type);

}

void ClothNode::UpdateSpringPos() {
    auto spring_ps = make_unique<std::vector<glm::vec3>>();
    spring_ps->reserve(system_.GetSprings().size() * 2);
    for (const auto& sp : system_.GetSprings()) {
        spring_ps->push_back(state_.positions[sp.i]);
        spring_ps->push_back(state_.positions[sp.j]);
    }
    springs_vo_->UpdatePositions(std::move(spring_ps));
}

void ClothNode::Update(double delta_time) {
    float remain = static_cast<float>(delta_time);
    while (remain > 0.0f) {
        const float h = std::min(dt_, remain);
        state_ = integrator_->Integrate(system_, state_, t_, h);
        t_ += h;
        remain -= h;
    }
    for (size_t i = 0; i < particles_.size(); ++i)
        particles_[i]->GetTransform().SetPosition(state_.positions[i]);

    UpdateSpringPos();

    // Toggle Wind
    static bool prev_released = true;

    if (InputManager::GetInstance().IsKeyPressed('W')) {
        if (prev_released) {
            system_.ToggleWind();
        }
        prev_released = false;
    } else {
        prev_released = true;
    }
}

} // namespace GLOO