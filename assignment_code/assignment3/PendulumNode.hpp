#include <memory>
#include <vector>
#include "gloo/SceneNode.hpp"
#include "IntegratorType.hpp"
#include "ParticleState.hpp"
#include "SpringSystem.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorFactory.hpp"

namespace GLOO {

class PendulumNode : public SceneNode {
 public:
  PendulumNode(IntegratorType type, float dt);
  void Update(double delta_time) override;

 private:
  SpringSystem system_;
  ParticleState  state_;
  std::unique_ptr<IntegratorBase<ParticleSystemBase, ParticleState>> integrator_;
  float dt_;
  float t_{0.0f};
  std::vector<SceneNode*> particles_;
};

} // namespace GLOO