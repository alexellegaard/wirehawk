#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <vector>
#include <mutex>
#include <algorithm>
#include <string>
#include <functional>
#include <cmath>
#include <chrono>

namespace wirehawk {

  /// Result of solving one cable's equilibrium.
  struct CatenaryResult {
    bool valid = false;                                    // false => slack (zero force)
    gz::math::Vector3d force = gz::math::Vector3d::Zero;   // force ON the payload
  };

  /// Elastic catenary cable element.
  ///
  /// Replaces the old massless Kelvin-Voigt spring with a cable that has both
  /// self-weight (linear density mu -> w = mu*g) and axial elasticity (EA).
  /// Given the anchor A, payload P and the commanded unstretched length L0 it
  /// solves for the cable tension and returns the force the cable exerts on
  /// the payload (tension tangent at the payload end).
  ///
  /// Model (Irvine, "Cable Structures"): the horizontal tension H is constant
  /// along the cable; the vertical tension grows linearly with unstretched arc
  /// length,  V(s) = Vp + w*s,  so Va = Vp + w*L0. The two projections are
  ///   l = (H/w)*[asinh(Va/H) - asinh(Vp/H)] + H*L0/EA
  ///   h = (H/w)*[sqrt(1+(Va/H)^2) - sqrt(1+(Vp/H)^2)] + (Vp*L0 + w*L0^2/2)/EA
  /// solved for (H, Vp) with damped Newton. A slack cable (L0 >= chord) carries
  /// no tension and returns zero force.
  static CatenaryResult SolveCatenary(
      const gz::math::Vector3d &_anchor,
      const gz::math::Vector3d &_payload,
      double _L0, double _mu, double _EA,
      double _g, double _maxTension)
  {
    CatenaryResult out;
    const double w = _mu * _g;                    // weight per unit length [N/m]

    gz::math::Vector3d d = _anchor - _payload;    // payload -> anchor
    const double l = std::sqrt(d.X()*d.X() + d.Y()*d.Y());   // horizontal span
    const double h = d.Z();                        // vertical drop (anchor above payload)
    const double chord = std::sqrt(l*l + h*h);

    // Slack: unstretched length already reaches the chord with no tension.
    if (_L0 <= 0.0 || chord <= _L0)
      return out;

    // Fallback: straight massless elastic cable (exact when sag -> 0).
    auto straightForce = [&]() -> gz::math::Vector3d {
      double stretch = (l < 1e-9) ? (std::fabs(h) - _L0) : (chord - _L0);
      double T = _EA * stretch / _L0;
      if (T < 0.0) T = 0.0;
      if (T > _maxTension) T = _maxTension;
      if (l < 1e-9)
        return gz::math::Vector3d(0.0, 0.0, (h >= 0.0 ? 1.0 : -1.0)) * T;
      return d / chord * T;
    };

    // Massless or vertical cable -> straight elastic (catenary degenerates).
    if (w < 1e-12 || l < 1e-9) {
      out.valid = true;
      out.force = straightForce();
      return out;
    }

    // Initial guess: straight taut cable, tension from elastic stretch.
    double T0 = _EA * (chord - _L0) / _L0;
    if (T0 < 1e-6) T0 = 1e-6;
    double H = T0 * l / chord;
    double Vp = T0 * h / chord;

    bool converged = false;
    for (int iter = 0; iter < 40; ++iter) {
      const double Va = Vp + w * _L0;

      const double ap = std::asinh(Vp / H);
      const double aa = std::asinh(Va / H);
      const double rp = std::sqrt(1.0 + (Vp/H)*(Vp/H));
      const double ra = std::sqrt(1.0 + (Va/H)*(Va/H));

      const double F1 = (H/w)*(aa - ap) + H*_L0/_EA - l;
      const double F2 = (H/w)*(ra - rp) + (Vp*_L0 + 0.5*w*_L0*_L0)/_EA - h;

      if (std::fabs(F1) < 1e-9 && std::fabs(F2) < 1e-9) { converged = true; break; }

      auto F = [&](double Hh, double Vv) {
        const double Vav = Vv + w * _L0;
        const double app = std::asinh(Vv / Hh);
        const double aav = std::asinh(Vav / Hh);
        const double rpp = std::sqrt(1.0 + (Vv/Hh)*(Vv/Hh));
        const double rav = std::sqrt(1.0 + (Vav/Hh)*(Vav/Hh));
        const double f1 = (Hh/w)*(aav - app) + Hh*_L0/_EA - l;
        const double f2 = (Hh/w)*(rav - rpp) + (Vv*_L0 + 0.5*w*_L0*_L0)/_EA - h;
        return std::make_pair(f1, f2);
      };

      const double dH = 1e-6 * std::fabs(H) + 1e-9;
      const double dV = 1e-6 * std::fabs(Vp) + 1e-9;
      auto Fph = F(H + dH, Vp);
      auto Fmh = F(H - dH, Vp);
      auto Fpv = F(H, Vp + dV);
      auto Fmv = F(H, Vp - dV);

      const double J11 = (Fph.first  - Fmh.first)  / (2*dH);
      const double J21 = (Fph.second - Fmh.second) / (2*dH);
      const double J12 = (Fpv.first  - Fmv.first)  / (2*dV);
      const double J22 = (Fpv.second - Fmv.second) / (2*dV);

      const double det = J11*J22 - J12*J21;
      if (std::fabs(det) < 1e-18) break;   // singular -> fall back

      const double sH = -( J22*F1 - J12*F2) / det;
      const double sV = -(-J21*F1 + J11*F2) / det;

      double alpha = 1.0;
      for (int bt = 0; bt < 12; ++bt) {
        const double Hn = H + alpha*sH;
        const double Vn = Vp + alpha*sV;
        if (Hn > 0.0 && std::isfinite(Hn) && std::isfinite(Vn)) break;
        alpha *= 0.5;
      }
      H += alpha*sH;
      Vp += alpha*sV;
      if (H < 1e-12) H = 1e-12;
    }

    double Tp = std::sqrt(H*H + Vp*Vp);
    if (!converged && (Tp <= 0.0 || !std::isfinite(Tp))) {
      out.valid = true;
      out.force = straightForce();
      return out;
    }
    if (Tp > _maxTension) { double s = _maxTension / Tp; H *= s; Vp *= s; }

    out.valid = true;
    out.force = gz::math::Vector3d(d.X()/l, d.Y()/l, 0.0) * H
              + gz::math::Vector3d(0.0, 0.0, 1.0) * Vp;
    return out;
  }

  class CableCatenaryPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
  private:
    gz::sim::Entity payload_link_entity_{gz::sim::kNullEntity};
    std::vector<gz::sim::Entity> cable_models_;
    std::vector<gz::math::Vector3d> anchors_;

    size_t num_cables_ = 0;
    std::vector<double> L0_;

    double linear_density_ = 0.0058;    // kg/m  (Dyneema DM20 3 mm ~5.8 g/m)
    double axial_stiffness_ = 707000.0; // N     (= E*A; Dyneema ~100 GPa x 7.07 mm^2)
    double max_tension_ = 5000.0;       // N     safety clamp on cable force
    double payload_drag_ = 15.0;        // N*s/m world-frame drag for stability
    const double g_ = 9.80665;

    gz::math::Vector3d prev_pos_{gz::math::Vector3d::Zero};
    std::chrono::steady_clock::duration last_time_{0};
    bool have_prev_ = false;

    gz::transport::Node node_;
    std::mutex msg_mutex_;

  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override
    {
      if (_sdf) {
        if (_sdf->HasElement("linear_density")) {
          linear_density_ = _sdf->Get<double>("linear_density");
        }
        if (_sdf->HasElement("axial_stiffness")) {
          axial_stiffness_ = _sdf->Get<double>("axial_stiffness");
        }
        if (_sdf->HasElement("max_tension")) {
          max_tension_ = _sdf->Get<double>("max_tension");
        }
        if (_sdf->HasElement("payload_drag")) {
          payload_drag_ = _sdf->Get<double>("payload_drag");
        }
      }

      gz::sim::Model model(_entity);
      payload_link_entity_ = model.LinkByName(_ecm, "mass_link");

      if (payload_link_entity_ == gz::sim::kNullEntity) {
        gzerr << "CableCatenaryPlugin: Could not find payload link named [mass_link]\n";
        return;
      }

      // Discover cable visual markers
      for (int i = 0; i < 4; ++i) {
        std::string c_name = "cable_vis_" + std::to_string(i);
        gz::sim::Entity c_ent = gz::sim::kNullEntity;
        _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_ent, const gz::sim::components::Model *,
              const gz::sim::components::Name *_nameComp) -> bool {
            if (_nameComp->Data() == c_name) {
              c_ent = _ent;
              return false;
            }
            return true;
          });
        cable_models_.push_back(c_ent);
      }

      // Discover pillars
      std::vector<std::string> pillar_names = {"pillar_1", "pillar_2", "pillar_3", "pillar_4"};
      for (const auto &name : pillar_names) {
        gz::sim::Entity pillar_entity = gz::sim::kNullEntity;
        _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_ent, const gz::sim::components::Model *,
              const gz::sim::components::Name *_nameComp) -> bool {
            if (_nameComp->Data() == name) {
              pillar_entity = _ent;
              return false;
            }
            return true;
          });

        if (pillar_entity != gz::sim::kNullEntity) {
          gz::math::Pose3d pose = gz::sim::worldPose(pillar_entity, _ecm);
          gz::math::Vector3d pos = pose.Pos();
          pos.Z() += 1.5; // Top of 3m pillar
          anchors_.push_back(pos);
        }
      }

      num_cables_ = anchors_.size();
      if (num_cables_ != 4) {
        gzerr << "CableCatenaryPlugin: Expected 4 pillars, found " << num_cables_ << ". Aborting.\n";
        return;
      }

      // Initial unstretched lengths from spawn pose (no pretension; the payload
      // sags to equilibrium as the cables load, same as the old model).
      gz::math::Vector3d p0 = gz::sim::worldPose(payload_link_entity_, _ecm).Pos();
      L0_.resize(num_cables_);
      for (size_t i = 0; i < num_cables_; ++i) {
        L0_[i] = (anchors_[i] - p0).Length();
      }

      if (!_ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(payload_link_entity_)) {
        _ecm.CreateComponent(payload_link_entity_, gz::sim::components::ExternalWorldWrenchCmd());
      }

      for (size_t i = 0; i < num_cables_; ++i) {
        std::string topic = "/cdpr/l" + std::to_string(i);
        std::function<void(const gz::msgs::Double &)> cb = [this, i](const gz::msgs::Double &_msg) {
          std::lock_guard<std::mutex> lock(this->msg_mutex_);
          if (i < this->L0_.size()) {
            this->L0_[i] = std::max(0.1, _msg.data());
          }
        };
        node_.Subscribe(topic, cb);
      }

      gzmsg << "CableCatenaryPlugin configured: mu=" << linear_density_
            << " kg/m, EA=" << axial_stiffness_ << " N, max_tension=" << max_tension_
            << " N, payload_drag=" << payload_drag_ << " N*s/m\n";
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
      if (_info.paused || payload_link_entity_ == gz::sim::kNullEntity) return;

      gz::sim::Link payload(payload_link_entity_);
      auto poseOpt = payload.WorldPose(_ecm);
      if (!poseOpt.has_value()) return;

      gz::math::Vector3d p = poseOpt.value().Pos();

      // Velocity via finite difference of the world pose. The LinearVelocity
      // component is not reliably populated by the physics engine in PreUpdate,
      // so WorldLinearVelocity() returns zero here (which silently killed the
      // old model's C*L_dot damping too).
      double dt = 0.0;
      if (have_prev_) {
        dt = std::chrono::duration<double>(_info.simTime - last_time_).count();
      }
      gz::math::Vector3d v = (have_prev_ && dt > 1e-9)
          ? (p - prev_pos_) / dt
          : gz::math::Vector3d::Zero;
      prev_pos_ = p;
      last_time_ = _info.simTime;
      have_prev_ = true;

      gz::math::Vector3d net_force(0.0, 0.0, 0.0);

      std::vector<double> current_L0(num_cables_);
      {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        current_L0 = L0_;
      }

      for (size_t i = 0; i < num_cables_; ++i) {
        CatenaryResult res = SolveCatenary(anchors_[i], p, current_L0[i],
                                           linear_density_, axial_stiffness_, g_, max_tension_);
        if (res.valid)
          net_force += res.force;

        // Visual: straight chord from payload toward anchor (sag not drawn).
        if (i < cable_models_.size() && cable_models_[i] != gz::sim::kNullEntity) {
          gz::math::Vector3d u = anchors_[i] - p;
          const double L = u.Length();
          // Guard: SetFrom2Axes is degenerate when u is (anti)parallel to UnitZ.
          if (L > 1e-6 && std::fabs(u.Z() / L) < 0.999) {
            u /= L;
            gz::math::Quaterniond rot;
            rot.SetFrom2Axes(gz::math::Vector3d::UnitZ, u);
            gz::sim::Model cable_model(cable_models_[i]);
            cable_model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(p, rot));
          }
        }
      }

      // World-frame drag: damps all modes uniformly (energy dissipation so the
      // otherwise-conservative cable system settles).
      net_force -= v * payload_drag_;

      gz::msgs::Wrench wrench_msg;
      gz::msgs::Set(wrench_msg.mutable_force(), net_force);
      gz::msgs::Set(wrench_msg.mutable_torque(), gz::math::Vector3d::Zero);

      _ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(
          payload_link_entity_, wrench_msg);
    }
  };
}

GZ_ADD_PLUGIN(wirehawk::CableCatenaryPlugin, gz::sim::System,
              wirehawk::CableCatenaryPlugin::ISystemConfigure,
              wirehawk::CableCatenaryPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(wirehawk::CableCatenaryPlugin, "wirehawk::CableCatenaryPlugin")
