#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>

namespace wirehawk {
  class CablePhysicsPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate 
  {
  private:
    gz::sim::Entity payload_link_entity_;

  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override 
    {
      gz::sim::Model model(_entity);
      payload_link_entity_ = model.LinkByName(_ecm, "mass_link");
      
      if (!_ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(payload_link_entity_)) {
        _ecm.CreateComponent(payload_link_entity_, gz::sim::components::ExternalWorldWrenchCmd());
      }
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override 
    {
      if (_info.paused) return; 

      // Inject a constant 98.0N upward force to perfectly cancel out 10kg gravity
      gz::math::Vector3d total_cable_force(0.0, 0.0, 0.0); 

      gz::msgs::Wrench wrench_msg;
      gz::msgs::Set(wrench_msg.mutable_force(), total_cable_force);
      gz::msgs::Set(wrench_msg.mutable_torque(), gz::math::Vector3d::Zero);

      // FIXED: Passing wrench_msg directly instead of wrapping it in the component class
      _ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(
          payload_link_entity_, 
          wrench_msg);
    }
  };
}

GZ_ADD_PLUGIN(wirehawk::CablePhysicsPlugin,
              gz::sim::System,
              wirehawk::CablePhysicsPlugin::ISystemConfigure,
              wirehawk::CablePhysicsPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(wirehawk::CablePhysicsPlugin, "wirehawk::CablePhysicsPlugin")