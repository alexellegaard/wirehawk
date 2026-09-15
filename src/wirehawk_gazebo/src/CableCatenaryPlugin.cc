#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <sdf/Geometry.hh>
#include <sdf/Cylinder.hh>
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

  // Visual trim radii (must match the SDF payload sphere and pillar radii):
  // the drawn cable starts at the sphere surface and ends at the pillar surface
  // so it does not pierce the visuals.
  static constexpr double kPayloadVisualRadius = 0.3;
  static constexpr double kAnchorVisualRadius  = 0.2;

  /// Elastic catenary cable element.
  ///
  /// Self-weight (linear density mu -> w = mu*g) plus axial elasticity (EA).
  /// Given the anchor A, payload P and commanded unstretched length L0 it
  /// solves for the cable tension and returns the force the cable exerts on
  /// the payload (tension tangent at the payload end).
  ///
  /// Slack handling: a cable whose straight chord is at or below L0 carries
  /// no tension. The taut->slack switch is smoothed with a smoothstep ramp
  /// over a small strain band (<slack_transition_strain>) so the stiffness
  /// does not jump from 0 (slack) to ~EA/L0 (taut) instantaneously — a hard
  /// jump excites a spurious bounce near the workspace edge.
  static CatenaryResult SolveCatenary(
      const gz::math::Vector3d &_anchor,
      const gz::math::Vector3d &_payload,
      double _L0, double _mu, double _EA,
      double _g, double _maxTension, double _slackStrain)
  {
    CatenaryResult out;
    const double w = _mu * _g;                    // weight per unit length [N/m]

    gz::math::Vector3d d = _anchor - _payload;    // payload -> anchor
    const double l = std::sqrt(d.X()*d.X() + d.Y()*d.Y());   // horizontal span
    const double h = d.Z();                        // vertical drop (anchor above payload)
    const double chord = std::sqrt(l*l + h*h);

    if (_L0 <= 0.0)
      return out;

    const double stretch = chord - _L0;
    if (stretch <= 0.0)
      return out;                                  // fully slack (zero force)

    const double strain = stretch / _L0;

    // Smoothstep ramp: 0 at slack, 1 once the cable is clearly taut.
    double ramp = 1.0;
    if (_slackStrain > 0.0 && strain < _slackStrain) {
      const double t = strain / _slackStrain;
      ramp = t * t * (3.0 - 2.0 * t);
    }

    // Fallback: straight massless elastic cable (exact when sag -> 0).
    auto straightForce = [&]() -> gz::math::Vector3d {
      double stretchL = (l < 1e-9) ? (std::fabs(h) - _L0) : (chord - _L0);
      double T = _EA * stretchL / _L0;
      if (T < 0.0) T = 0.0;
      if (T > _maxTension) T = _maxTension;
      if (l < 1e-9)
        return gz::math::Vector3d(0.0, 0.0, (h >= 0.0 ? 1.0 : -1.0)) * T;
      return d / chord * T;
    };

    // Near-slack, massless, or vertical cable -> straight elastic (continuous
    // through the slack boundary), scaled by the ramp. Avoids the catenary
    // Newton solve in its near-singular low-tension regime.
    if ((_slackStrain > 0.0 && strain < _slackStrain) || w < 1e-12 || l < 1e-9) {
      out.valid = true;
      out.force = straightForce() * ramp;
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
      out.force = straightForce() * ramp;
      return out;
    }
    if (Tp > _maxTension) { double s = _maxTension / Tp; H *= s; Vp *= s; }

    out.valid = true;
    out.force = gz::math::Vector3d(d.X()/l, d.Y()/l, 0.0) * H
              + gz::math::Vector3d(0.0, 0.0, 1.0) * Vp;
    out.force *= ramp;
    return out;
  }

  class CableCatenaryPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
  private:
    gz::sim::Entity payload_link_entity_{gz::sim::kNullEntity};

    // Per-cable chain of visual segment models: cable_segments_[i][j] is the
    // j-th segment MODEL of cable i; cable_visuals_[i][j] is its visual entity
    // (whose cylinder geometry is resized each step).
    std::vector<std::vector<gz::sim::Entity>> cable_segments_;
    std::vector<std::vector<gz::sim::Entity>> cable_visuals_;

    std::vector<gz::math::Vector3d> anchors_;

    size_t num_cables_ = 0;
    std::vector<double> L0_;

    // All values come from the world SDF (<plugin> block) — single source of
    // truth. No code defaults, so a missing value fails loudly.
    double linear_density_ = 0.0;         // kg/m  cable mass per metre (weight -> sag)
    double axial_stiffness_ = 0.0;        // N     E*A, cable axial rigidity (stretch)
    double max_tension_ = 0.0;            // N     safety clamp on cable force
    double payload_drag_ = 0.0;           // N*s/m world-frame drag (settling)
    double anchor_height_ = 0.0;          // m     z of the anchor (top of the mast)
    double gravity_ = 0.0;                // m/s^2 gravity magnitude for cable self-weight
    double slack_transition_strain_ = 0.0;// [-]   strain band for the smooth taut->slack ramp

    gz::math::Vector3d prev_pos_{gz::math::Vector3d::Zero};
    std::chrono::steady_clock::duration last_time_{0};
    bool have_prev_ = false;

    // Visual update throttle (30 Hz) — geometry resize every physics step is
    // wasteful and can slow the renderer.
    std::chrono::steady_clock::duration last_visual_time_{0};
    bool have_visual_time_ = false;

    gz::transport::Node node_;
    std::mutex msg_mutex_;

  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override
    {
      bool missing = false;
      auto readParam = [&](const char *_name, double &_out) {
        if (_sdf && _sdf->HasElement(_name)) {
          _out = _sdf->Get<double>(_name);
        } else {
          gzerr << "CableCatenaryPlugin: missing required <" << _name << "> parameter\n";
          missing = true;
        }
      };
      readParam("linear_density", linear_density_);
      readParam("axial_stiffness", axial_stiffness_);
      readParam("max_tension", max_tension_);
      readParam("payload_drag", payload_drag_);
      readParam("anchor_height", anchor_height_);
      readParam("gravity", gravity_);
      readParam("slack_transition_strain", slack_transition_strain_);
      if (missing) return;

      gz::sim::Model model(_entity);
      payload_link_entity_ = model.LinkByName(_ecm, "mass_link");

      if (payload_link_entity_ == gz::sim::kNullEntity) {
        gzerr << "CableCatenaryPlugin: Could not find payload link named [mass_link]\n";
        return;
      }

      auto findModelByName = [&](const std::string &_name) -> gz::sim::Entity {
        gz::sim::Entity result = gz::sim::kNullEntity;
        _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_ent, const gz::sim::components::Model *,
              const gz::sim::components::Name *_nameComp) -> bool {
            if (_nameComp->Data() == _name) {
              result = _ent;
              return false;
            }
            return true;
          });
        return result;
      };

      // Discover the visual cable segments: cable_vis_<i>_<j> for j = 0,1,...
      // (the SDF defines a chain of unit-length cylinders per cable).
      for (int i = 0; i < 4; ++i) {
        std::vector<gz::sim::Entity> segs;
        std::vector<gz::sim::Entity> visuals;
        for (int j = 0; ; ++j) {
          std::string name = "cable_vis_" + std::to_string(i) + "_" + std::to_string(j);
          gz::sim::Entity ent = findModelByName(name);
          if (ent == gz::sim::kNullEntity) break;

          // Resolve the segment's visual entity (for runtime cylinder resize).
          gz::sim::Entity visual = gz::sim::kNullEntity;
          gz::sim::Model m(ent);
          auto links = m.Links(_ecm);
          if (!links.empty()) {
            gz::sim::Link link(links[0]);
            auto vis = link.Visuals(_ecm);
            if (!vis.empty()) visual = vis[0];
          }
          segs.push_back(ent);
          visuals.push_back(visual);
        }
        cable_segments_.push_back(segs);
        cable_visuals_.push_back(visuals);
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
          anchors_.push_back(gz::math::Vector3d(pose.Pos().X(), pose.Pos().Y(), anchor_height_));
        }
      }

      num_cables_ = anchors_.size();
      if (num_cables_ != 4) {
        gzerr << "CableCatenaryPlugin: Expected 4 pillars, found " << num_cables_ << ". Aborting.\n";
        return;
      }

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
            << " N, payload_drag=" << payload_drag_ << " N*s/m, anchor_h=" << anchor_height_
            << " m, g=" << gravity_ << ", slack_transition_strain=" << slack_transition_strain_
            << "\n";
    }

    /// Position one cable's visual segments along a parabolic sag curve from
    /// the payload to its anchor, resizing each cylinder to its segment length.
    void UpdateVisual(size_t _i, const gz::math::Vector3d &_p,
                      const CatenaryResult &_res,
                      gz::sim::EntityComponentManager &_ecm)
    {
      const size_t n = cable_segments_[_i].size();
      if (n == 0) return;

      const gz::math::Vector3d a = anchors_[_i];
      const gz::math::Vector3d d = a - _p;
      const double l = std::sqrt(d.X()*d.X() + d.Y()*d.Y());
      const double chord = d.Length();
      if (chord < 1e-6) return;
      const gz::math::Vector3d d_hat = d / chord;

      // Mid-span sag of a uniform cable under horizontal tension H:
      //   sag = w*l^2/(8H).  H is the horizontal magnitude of the cable force.
      const double w = linear_density_ * gravity_;
      const double H = std::sqrt(_res.force.X()*_res.force.X() +
                                 _res.force.Y()*_res.force.Y());
      double sagMax = 0.0;
      if (H > 1e-6) {
        sagMax = w * l * l / (8.0 * H);
        if (sagMax > 0.15 * chord) sagMax = 0.15 * chord;
      } else {
        sagMax = 0.10 * chord;   // slack: moderate droop (avoid sharp joint bends)
      }

      // Sag direction = gravity projected perpendicular to the chord, so the
      // cable still dips toward the ground when the chord is steep.
      gz::math::Vector3d sagDir = gz::math::Vector3d(0.0, 0.0, -1.0)
                                + d_hat.Z() * d_hat;
      const double sagLen = sagDir.Length();
      if (sagLen > 1e-9) sagDir /= sagLen;
      else sagDir = gz::math::Vector3d(0.0, 0.0, -1.0);

      // Trim the drawn cable to the visual surfaces so it does not pierce the
      // payload sphere or the anchor pillar.
      const gz::math::Vector3d p_start = _p + kPayloadVisualRadius * d_hat;
      const gz::math::Vector3d a_end   = a - kAnchorVisualRadius  * d_hat;
      const gz::math::Vector3d span = a_end - p_start;

      auto sagAt = [&](double t) { return 4.0 * sagMax * t * (1.0 - t); };

      for (size_t j = 0; j < n; ++j) {
        const double t0 = (double)j / (double)n;
        const double t1 = (double)(j + 1) / (double)n;
        gz::math::Vector3d p0 = p_start + span * t0 + sagDir * sagAt(t0);
        gz::math::Vector3d p1 = p_start + span * t1 + sagDir * sagAt(t1);
        gz::math::Vector3d seg = p1 - p0;
        const double segLen = seg.Length();
        if (segLen < 1e-6) continue;

        const gz::math::Vector3d mid = 0.5 * (p0 + p1);
        const gz::math::Vector3d u = seg / segLen;

        gz::math::Quaterniond rot;
        if (std::fabs(u.Z()) < 0.999)
          rot.SetFrom2Axes(gz::math::Vector3d::UnitZ, u);
        else
          rot.SetFrom2Axes(gz::math::Vector3d::UnitX, u);

        gz::sim::Model seg_model(cable_segments_[_i][j]);
        seg_model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(mid, rot));

        // Resize the unit cylinder to the actual segment length.
        gz::sim::Entity visual = cable_visuals_[_i][j];
        if (visual != gz::sim::kNullEntity) {
          auto *geomComp = _ecm.Component<gz::sim::components::Geometry>(visual);
          if (geomComp) {
            sdf::Geometry geom = geomComp->Data();
            if (const sdf::Cylinder *cyl = geom.CylinderShape()) {
              sdf::Cylinder c = *cyl;
              c.SetLength(segLen);
              geom.SetCylinderShape(c);
              _ecm.SetComponentData<gz::sim::components::Geometry>(visual, geom);
            }
          }
        }
      }
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
      if (_info.paused || payload_link_entity_ == gz::sim::kNullEntity) return;

      gz::sim::Link payload(payload_link_entity_);
      auto poseOpt = payload.WorldPose(_ecm);
      if (!poseOpt.has_value()) return;

      gz::math::Vector3d p = poseOpt.value().Pos();

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

      // Throttle the visual to ~30 Hz (geometry resize each physics step is
      // wasteful and can slow the renderer).
      bool update_visual = false;
      if (have_visual_time_) {
        double dt_vis = std::chrono::duration<double>(_info.simTime - last_visual_time_).count();
        update_visual = (dt_vis >= 1.0 / 30.0);
      } else {
        update_visual = true;
      }
      if (update_visual) {
        last_visual_time_ = _info.simTime;
        have_visual_time_ = true;
      }

      for (size_t i = 0; i < num_cables_; ++i) {
        CatenaryResult res = SolveCatenary(anchors_[i], p, current_L0[i],
                                           linear_density_, axial_stiffness_,
                                           gravity_, max_tension_,
                                           slack_transition_strain_);
        if (res.valid)
          net_force += res.force;

        if (update_visual)
          UpdateVisual(i, p, res, _ecm);
      }

      // World-frame drag: damps all modes uniformly.
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
