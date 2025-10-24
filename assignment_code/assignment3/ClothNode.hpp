#include <memory>
#include <vector>
#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "IntegratorType.hpp"
#include "ParticleState.hpp"
#include "SpringSystem.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorFactory.hpp"

namespace GLOO {

class ClothNode : public SceneNode {
    public:
        ClothNode(IntegratorType type, float dt);
        void Update(double delta_time) override;
        void UpdateSpringPos();    

    private:
        SpringSystem system_;
        ParticleState  state_;
        std::unique_ptr<IntegratorBase<ParticleSystemBase, ParticleState>> integrator_;
        float dt_;
        float t_{0.0f};
        std::vector<SceneNode*> particles_;
        std::shared_ptr<VertexObject> springs_vo_;
};

} // namespace GLOO