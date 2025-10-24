#ifndef SPRINGSYS_H_
#define SPRINGSYS_H_

#include <vector>
#include <random>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "ParticleSystemBase.hpp"
#include "ParticleState.hpp"

namespace GLOO {

class SpringSystem : public ParticleSystemBase {
 public:
  struct Spring { int i, j; float k, L0; };

  // Add particles/springs, fix endpoints
  int AddParticle(float mass, bool fixed=false) { masses_.push_back(mass); fixed_.push_back(fixed); return (int)masses_.size()-1; }
  void Fix(int idx, bool on=true) { fixed_.at(idx)=on; }
  void AddSpring(int i, int j, float k, float L0) { springs_.push_back({i,j,k,L0}); }
  const std::vector<Spring>& GetSprings() const { return springs_; }

  // Wind
  void SetWind(const glm::vec3& f) { wind_f_ = f; }
  void ToggleWind() { 
    wind_on_ = !wind_on_; 
    if (wind_on_) { wind_f_ = SampleWindForce_(); }
  }

  // x' = f(x,t)
  ParticleState ComputeTimeDerivative(const ParticleState& s, float /*t*/) const override {
    const size_t N = s.positions.size();
    ParticleState d; d.positions.resize(N); d.velocities.resize(N);

    // p' = v (or 0 if fixed)
    for (size_t i=0;i<N;++i) d.positions[i] = fixed_[i] ? glm::vec3(0) : s.velocities[i];

    // accumulate forces
    std::vector<glm::vec3> F(N, glm::vec3(0));
    for (size_t i=0;i<N;++i) {
        if (fixed_[i]) continue;
        F[i] += masses_[i]*gravity_;
        F[i] += -drag_c_*s.velocities[i];
        if (wind_on_) { F[i] += wind_f_; } // Add wind
    }
    for (const Spring& sp : springs_) {
        int i=sp.i, j=sp.j;
        glm::vec3 dpos = s.positions[j]-s.positions[i];
        float L = glm::length(dpos);
        glm::vec3 u = dpos/L;
        glm::vec3 f = sp.k*(L - sp.L0)*u;
        if (!fixed_[i]) F[i] += f;
        if (!fixed_[j]) F[j] += -f;
    }

    // v' = F/m (or 0 if fixed)
    for (size_t i=0;i<N;++i) d.velocities[i] = fixed_[i] ? glm::vec3(0) : (F[i]/masses_[i]);
    return d;
  }

  private:
    std::vector<float> masses_;
    std::vector<bool>  fixed_;
    std::vector<Spring> springs_;
    glm::vec3 gravity_ = {0.f, -9.81f, 0.f};
    float drag_c_ = 0.9f;
    
    bool wind_on_ = false;
    glm::vec3 wind_f_ = glm::vec3(0.0f); 
    std::mt19937 rng_{std::random_device{}()};

    glm::vec3 SampleWindForce_() {
      std::uniform_real_distribution<float> dir(-1.0f, 1.0f);
      std::uniform_real_distribution<float> mag(5.0f, 15.0f);
      glm::vec3 dir_vec(0.0f);
      while (glm::length(dir_vec) < 1e-6f) { dir_vec = { dir(rng_), dir(rng_), dir(rng_) }; }
      dir_vec = glm::normalize(dir_vec);
      return dir_vec * mag(rng_);
    }

};

} // namespace GLOO

#endif