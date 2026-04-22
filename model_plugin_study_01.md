# 🚀学习Gazebo插件写法 

## 第一个案例 HelloWorld的 World Plugin

### 你好，WorldPlugin！
- 为新插件创建一个目录和一个 .cpp 文件：
```bash
$ mkdir ~/gazebo_plugin_tutorial
$ cd ~/gazebo_plugin_tutorial
$ code .
```

- 然后在 VSCode 里创建 hello_world.cpp, 将以下内容复制到hello_world.cpp中：
```cpp
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
#### 代码解析
```cpp
#include <gazebo/gazebo.hh>

namespace gazebo
{
```
- gazebo/gazebo.hh 文件包含一套核心的 gazebo 基本功能。 它不包含 ， ， 或者说，这些应该具体情况都算在内。 所有插件必须在命名空间中。`gazebo/physics/physics.hh` `gazebo/rendering/rendering.hh` `gazebo/sensors/sensors.hh` `gazebo`
```cpp
  class WorldPluginTutorial : public WorldPlugin
  {
    public: WorldPluginTutorial() : WorldPlugin()
            {
              printf("Hello World!\n");
```
- 每个插件必须继承一个插件类型，这里指的是类。例如`WorldPlugin`
```cpp
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
```
- 唯一另一个必填函数是接收 SDF 元素的函数 包含加载后的 SDF 文件中指定的元素和属性。`load`
```cpp
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```
- 最后，插件必须通过宏向模拟器注册。 该宏唯一的参数是插件类的名称。 每种插件类型都有匹配的寄存器宏：、和。`GZ_REGISTER_WORLD_PLUGIN` `GZ_REGISTER_MODEL_PLUGIN` `GZ_REGISTER_SENSOR_PLUGIN` `GZ_REGISTER_GUI_PLUGIN` `GZ_REGISTER_SYSTEM_PLUGIN` `GZ_REGISTER_VISUAL_PLUGIN`
#### 插件编译
- 要编译上述插件，请创建：`~/gazebo_plugin_tutorial/CMakeLists.txt`
```bash
$ code ~/gazebo_plugin_tutorial/CMakeLists.txt
```
- CMakeLists.txt复制以下内容：
```cpp
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
```
