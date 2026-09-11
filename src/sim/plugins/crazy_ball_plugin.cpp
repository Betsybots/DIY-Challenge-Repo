#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>

/// \brief Plugin that makes the ball move randomly and
/// resets its position every 3 seconds.
class CrazyBallPlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate {
public:
    /// \brief Constructor
    CrazyBallPlugin() = default;

    /// \brief Destructor
    ~CrazyBallPlugin() override = default;

    /// \brief Configure the plugin
    /// \param[in] _entity The entity this plugin is attached to.
    /// \param[in] _sdf SDF element containing configuration for this plugin.
    /// \param[in] _ecm Entity component manager.
    /// \param[in] _eventMgr The event manager.
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override {
        model = gz::sim::Model(_entity);
        modelEntity = _entity;

        auto modelName = _ecm.Component<gz::sim::components::Name>(modelEntity)->Data();

        auto poseComp = _ecm.Component<gz::sim::components::Pose>(modelEntity);
        if (poseComp)
            initialPose = poseComp->Data();

        // Ensure the ball is always reset to ground level
        initialPose.SetZ(0.11);
        model.SetWorldPoseCmd(_ecm, initialPose);

        // Apply initial force
        force = gz::math::Vector3d(1.0, 10.0, 0.0);
        applyForce(_ecm, force);

        // We'll initialize lastResetTime in the first PreUpdate call
        isFirstUpdate = true;
    }

    /// \brief Update the plugin
    /// \param[in] _info System update information.
    /// \param[in] _ecm Entity component manager.
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override {
        // Initialize lastResetTime with current sim time on first update
        if (isFirstUpdate) {
            lastResetTime = _info.simTime;
            isFirstUpdate = false;
            gzmsg << "CrazyBallPlugin: Initialized lastResetTime with simulation time: "
                  << std::chrono::duration_cast<std::chrono::seconds>(lastResetTime).count()
                  << " seconds" << std::endl;
        }

        applyForce(_ecm, force);

        auto currentTime = _info.simTime;
        auto duration =
            std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastResetTime).count();

        if (duration >= 7 /*seconds*/) {
            // Reset ball position
            auto pose = initialPose;
            model.SetWorldPoseCmd(_ecm, pose);

            // Update the last reset time
            lastResetTime = currentTime;

            gzmsg << "CrazyBallPlugin: Reset ball position at sim time: "
                  << std::chrono::duration_cast<std::chrono::seconds>(currentTime).count()
                  << " seconds" << std::endl;
        }
    }

    void applyForce(gz::sim::EntityComponentManager &_ecm, const gz::math::Vector3d &_force) {
        gz::sim::Link linkEntity(model.LinkByName(_ecm, "ball_link"));
        linkEntity.AddWorldForce(_ecm, _force);
    }

private:
    gz::sim::Model model;
    gz::sim::Entity modelEntity;
    gz::math::Pose3d initialPose;
    gz::math::Vector3d force;
    std::chrono::nanoseconds lastResetTime{0};
    bool isFirstUpdate{true};
};

GZ_ADD_PLUGIN(CrazyBallPlugin, gz::sim::System, CrazyBallPlugin::ISystemConfigure,
              CrazyBallPlugin::ISystemPreUpdate)  // Register the plugin

GZ_ADD_PLUGIN_ALIAS(CrazyBallPlugin, "gz::sim::systems::CrazyBallPlugin")
