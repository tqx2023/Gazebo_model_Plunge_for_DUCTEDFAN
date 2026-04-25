# 🚀学习Gazebo插件写法 
## Ducted-FAN_plugin
### 新建插件源码文件
- 要先在项目文件下建立我们的`ductedfan_plugin`文件夹
```bash
cd ~/DuctedFanUAV-Autopilot/Tools/sitl_gazebo/src
mkdir ductedfan_plugin
code ductedfan_plugin/ductedfan_plugin.cpp
```
- 目录结构变成：
```bash
DuctedFanUAV-Autopilot/
└── Tools/
    └── sitl_gazebo/
        └── src/
            └── ductedfan_plugin/
                └── ductedfan_plugin.cpp
```
### ductedfan_plugin.cpp 内容 (HELLOWORLD代码作为示例代码。实际代码在本项目，plugins/ductedfan_plugin.cpp)
```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class DuctedFanPlugin : public ModelPlugin
  {
    public: DuctedFanPlugin() : ModelPlugin()
    {
      printf("DuctedFanPlugin constructor called.\n");
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      printf("Hello World from DuctedFanPlugin!\n");

      this->model = _model;
    }

    private: physics::ModelPtr model;
  };

  GZ_REGISTER_MODEL_PLUGIN(DuctedFanPlugin)
}
```

### 修改CMakeLists.txt
- 打开
```bash
code ~/DuctedFanUAV-Autopilot/Tools/sitl_gazebo/CMakeLists.txt
```
- 找到 `LiftDragPlugin` 附近，例如：
```bash
add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
list(APPEND plugins LiftDragPlugin)
```
- 在它附近加入
```bash
add_library(DuctedFanPlugin SHARED src/ductedfan_plugin/ductedfan_plugin.cpp)
list(APPEND plugins DuctedFanPlugin)
```
- 编译后生成的库文件是：`libDuctedFanPlugin.so`
- 后面 SDF 里必须写这个名字

