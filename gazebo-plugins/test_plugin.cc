#include <gazebo/gazebo.hh>

namespace gazebo {
  class WorldPluginTest : public WorldPlugin {
    public: WorldPluginTest() : WorldPlugin() {
      printf("\n\n\n\nHello World\n\n\n\n");
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTest)
}
