# 🚀学习Gazebo插件写法 

## model plugin案例复现
- study_01中创建了gazebo_plugin_tutorial包
### 代码
- 插件允许完全访问模型及其底层元素（链接、关节、碰撞对象）的物理属性。以下插件将对其父模型应用线性速度。
```bash
$ cd ~/gazebo_plugin_tutorial
$ code model_push.cpp
```
- 插件代码：
```cpp
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // 存储该模型的指针
      this->model = _parent;

      // 监听更新事件。该事件在每次仿真迭代时广播
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // 由世界更新开始事件调用
    public: void OnUpdate()
    {
      // 为模型施加一个微小的线速度
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // 模型指针
    private: physics::ModelPtr model;

    //更新事件连接指针
    private: event::ConnectionPtr updateConnection;
  };

  // 向仿真器注册此插件
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
```
### 代码解析
```cpp
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
```
- 头文件部分，要确定包含了这些头文件
```cpp
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
```
- `Load()` 是插件被 `Gazebo` 成功加载后自动调用的入口函数。
- `_parent` 类型是：`physics::ModelPtr`，表示当前插件所挂载的那个模型指针。也就是说，如果你把这个插件挂在飞机模型上，`_parent` 就是那个飞机模型。
- `_sdf` 类型是：`sdf::ElementPtr`，表示插件在 SDF 里的参数节点。
```cpp
this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ModelPush::OnUpdate, this));
```
- `Load()` 内部第二部分
- 把 `OnUpdate()` 函数注册到 `Gazebo` 的“世界更新开始事件”上。
- 每一次仿真迭代开始时，`Gazebo` 都会调用一次：`ModelPush::OnUpdate()`
```cpp
private: event::ConnectionPtr updateConnection;
```
- `updateConnection` 为什么要保存？
- 因为这个连接对象必须保存着。如果不保存，连接可能立刻失效，事件回调就不会持续工作。
### 插件编译
- 在`~/gazebo_plugin_tutorial/CMakeLists.txt`中添加如下几行
```cpp
add_library(model_push SHARED model_push.cpp)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
```
- 保存`CMakeLists.txt`文件，打开命令行
```bash
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
```
- 编译这些代码会生成一个共享库 ，可以插入 Gazebo 模拟中。`~/gazebo_plugin_tutorial/build/libmodel_push.so`
### 运行插件
```bash
$ cd ~/gazebo_plugin_tutorial
$ code model_push.world
```
- 该插件用于世界文件 。`~/gazebo_plugin_tutorial/model_push.world`
- 在`model_push.world`中复制如下代码
```cpp
<?xml version="1.0"?> 
<sdf version="1.4">
  <world name="default">

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>

      <plugin name="model_push" filename="libmodel_push.so"/>
    </model>        
  </world>
</sdf>
```
```cpp
<plugin name="model_push" filename="libmodel_push.so"/>
```
- 在模型元素代码块末尾，通过以下方式指定将插件挂载至模型的挂载钩子：
```bash
$ export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH
```
- 将你的库文件路径添加到 `GAZEBO_PLUGIN_PATH` 环境变量中
```bash
#!/bin/bash

# ⭐ 必须加这一句 , 要让gazebo本身的路径能加载进来。
source /usr/share/gazebo/setup.bash

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:${PROJECT_ROOT}/build"

echo "PROJECT_ROOT=${PROJECT_ROOT}"
echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"

```
- `study_01`中创建了`gazebo_plugin_tutorial`包中有一个`run_gazebo.sh` ，它保证了这是一个干净的系统环境。

### 开始模拟
- 运行我们的`run_gazebo.sh`脚本，确保把插件挂在了`GAZEBO_PLUGIN_PATH`上
```bash
source ~/gazebo_plugin_tutorial/run_gazebo.sh
```
- 在这个命令行窗口内，运行
```bash
$ cd ~/gazebo_plugin_tutorial/
$ gzserver -u model_push.world
```
- 该选项会让服务器处于暂停状态。`-u`
- 在一个独立终端中，启动图形界面
```bash
$ gzclient
```
- 点击界面中的播放按钮来解除模拟暂停，你应该会看到盒子移动。