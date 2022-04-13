# [Writing a plugin](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)

In its own folder crate a `plugin.cc` file with the following contents:

```c++
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginTutorial : public WorldPlugin
  {
    public: WorldPluginTutorial() : WorldPlugin()
            {
              printf("Hello World!\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```

This is the bare minimum for a plugin.

Then create a `CMakeLists.txt` file with the following contents:

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
```

# Compiling Gazebo Plugins

Create a build folder and cd into it.

```bash
mkdir build
cd build
```

From ./build run the following code to compile the library:

```bash
cmake ../plugin_name
make
```

This will result in the plugin file `plugin_name.so` being created in the build directory. You need to ad the library path to the `GAZEBO_PLUGIN_PATH` environment variable.

```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/path/to/build
```

Or to make this permanent add the line above to the end of the ~/.bashrc file.

# Using the plugins

You need to import the plugin in the world SDF file or the robots SDF text through xacro/URDF.

```xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <plugin name="hello_world" filename="libhello_world.so"/>
  </world>
</sdf>
```

Then if you run gazebo with th world above loaded you should see the output