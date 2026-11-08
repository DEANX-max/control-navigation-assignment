# 来源说明

本仓库为 Trident 内部教学派生工程。旧 D* Lite 和 PID 实现来自
`trident-rm/2027-navi-demo` 中保留的 HKU ROS 1 导航工程。
来源文件、固定 SHA、文件哈希与适配范围见 `docs/provenance.md` 和
`docs/provenance.json`。原代码作者注释保持在对应提取文件内。

本说明不替上游代码授予新的开源许可。ROS 2、Ubuntu 和 NumPy 等镜像内
安装依赖仍适用其各自许可证；本仓库不复制其源码。

Gazebo仿真复用 `trident-rm/Omni_sim` 的 sentry 机器人资源与 OmniDrive 插件，
来源固定提交 `2c64cee25776b361714ac8af5011acf3d39e2b56`。对应 Apache-2.0
许可证、资源清单与原始声明保留在 nav_simulation/third_party。森林生成适配来自
已有森林场景模块，涉及 GPL-3.0-only 的说明和许可证也保留在同一目录。
