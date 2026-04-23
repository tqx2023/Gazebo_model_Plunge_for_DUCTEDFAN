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
- 旧版本Gazebo 6 以下 老版本 Gazebo 依赖这个变量`CMAKE_CXX_FLAGS`，把 Gazebo 的编译选项追加到全局 C++ 编译参数里
```cpp
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
```
- 新的写法是使用target函数保证有这个参数，set函数保证编译器为c++ 11
```cpp
target_compile_options(hello_world PRIVATE ${GAZEBO_CXX_FLAGS})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```
- 创建构建目录
```bash
$ mkdir ~/gazebo_plugin_tutorial/build
$ cd ~/gazebo_plugin_tutorial/build
```
- 编译代码。
```bash
$ cmake ../
$ make
```
- 编译后将得到一个共享库， 可以插入凉亭模拟中。`~/gazebo_plugin_tutorial/build/libhello_world.so`
- 最后，将你的库路径添加到：`GAZEBO_PLUGIN_PATH`
```bash
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/build
```
-注意：这只改变当前壳体的路径。如果你想使用 你每次打开新 Temrinal 的插件，在文件上加上上面的那行。`~/.bashrc`
#### 使用插件
- 一旦你编译了一个插件作为共享库（见上文）， 你可以将其附加到SDF文件中的世界或模型中 （更多信息请参见SDF文档）。 启动时，Gazebo解析SDF文件，定位插件，并加载代码。 Gazebo 能够找到该插件非常重要。 要么指定了插件的完整路径，要么插件存在于 环境变量中的一条路径。`GAZEBO_PLUGIN_PATH`
- 创建一个世界文件，然后把下面的代码复制进去。示例世界文件 也可以在 `examples/plugins/hello_world/hello.world `中找到。
```bash
$ code ~/gazebo_plugin_tutorial/hello.world
```
```bash
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <plugin name="hello_world" filename="libhello_world.so"/>
  </world>
</sdf>
```
- 现在用：`gzserver`
```bash
$ gzserver ~/gazebo_plugin_tutorial/hello.world --verbose
```
- 你应该会看到类似的输出：
```bash
Gazebo multi-robot simulator, version 6.1.0
Copyright (C) 2012-2015 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.23.1.52
Hello World!
```