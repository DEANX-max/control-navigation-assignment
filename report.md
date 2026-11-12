Trident 控制组导航大作业报告：实时局部障碍地图

徐祥騰
3036589208

1. 任务概述

场景中共有 120 根柱子，全局先验只包含其中 60 根。本作业要求根据实时雷达点云生成局部障碍地图，让机器人能够发现先验中缺失的柱子，再交给已经提供的 D* Lite 规划器和 PID 控制器完成避障导航。

我负责实现 student_map.py 中的两个函数：

函数和作用
project_cloud(points, spec)	｜ 把三维点云投影成二维占据栅格（0 = 空闲，100 = 占据）
inflate_grid(occupied, spec) ｜ 对占据格子做圆盘膨胀，半径为 inflation_radius 米

规划、控制、地图融合节点和仿真场景均未修改。

2. 算法设计
2.1 点云投影 project_cloud

栅格约定

返回数组形状为 (height, width)，类型为 uint8，按 grid[iy, ix] 索引，即先行（y）后列（x）
格子编号：ix = floor((x - origin_x) / resolution)，iy = floor((y - origin_y) / resolution)
每个格子是左闭右开区间：左边界、下边界属于该格子，右边界、上边界属于下一个格子
每一帧都从全零栅格重新生成，不保留历史

处理步骤

新建全零栅格 np.zeros((height, width), dtype=np.uint8)
把输入统一转换为 float64 的 (N, 3) 数组，空输入会得到 (0, 3)
删除含 NaN 或 Inf 的点：np.isfinite(pts).all(axis=1)
高度过滤，保留 z_min <= z <= z_max 的点（两端都包含），这样可以去掉地面和过高的点
用 np.floor 计算格子编号，再转换为 int64
删除超出地图范围的点：0 <= ix < width 且 0 <= iy < height
用花式索引一次性赋值：grid[iy, ix] = 100
python
grid = np.zeros((spec.height, spec.width), dtype=np.uint8)
pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
pts = pts[np.isfinite(pts).all(axis=1)]
z = pts[:, 2]
pts = pts[(z >= spec.z_min) & (z <= spec.z_max)]
ix = np.floor((pts[:, 0] - spec.origin_x) / spec.resolution).astype(np.int64)
iy = np.floor((pts[:, 1] - spec.origin_y) / spec.resolution).astype(np.int64)
inside = (ix >= 0) & (ix < spec.width) & (iy >= 0) & (iy < spec.height)
grid[iy[inside], ix[inside]] = 100
return grid

设计要点

使用 floor 而不是 int()：int(-0.5) 会得到 0，而正确结果应为 -1。用 floor 可以保证负坐标落在正确的格子里，之后再被范围检查过滤掉
先做范围检查再赋值：NumPy 的负索引会从数组末尾倒着取，如果不先过滤，越界的点会被错误地写到地图另一侧
多个点落在同一格：重复赋值为 100，不影响结果
2.2 圆盘膨胀 inflate_grid

机器人有实际尺寸，规划器却把机器人当作一个点。把障碍物向外扩大 inflation_radius，就等于在规划时为机器人的半径预留了安全距离。

处理步骤

复制输入：out = occupied.copy()，保证不修改调用者的数组
计算搜索范围：dmax = ceil(r / res) 个格子。ceil 只用来确定搜索范围
预先生成圆形偏移表：遍历 dx, dy ∈ [-dmax, dmax]，只保留满足 hypot(dx, dy) * res <= r + 1e-9 的偏移
种子只从原始输入中取：iy, ix = np.nonzero(occupied == 100)
对每个偏移，把所有种子一起平移，裁掉越界的部分，再标记为 100
python
out = occupied.copy()
r, res = spec.inflation_radius, spec.resolution
h, w = occupied.shape
dmax = int(np.ceil(r / res))
offsets = []
for dy in range(-dmax, dmax + 1):
    for dx in range(-dmax, dmax + 1):
        if np.hypot(dx, dy) * res <= r + 1e-9:
            offsets.append((dy, dx))
iy, ix = np.nonzero(occupied == 100)
for dy, dx in offsets:
    ny, nx = iy + dy, ix + dx
    ok = (ny >= 0) & (ny < h) & (nx >= 0) & (nx < w)
    out[ny[ok], nx[ok]] = 100
return out

设计要点

欧氏圆形，不是方形：用 hypot 计算真实距离，对角方向的格子不会被多膨胀
单位换算：距离以格子数计算，乘以 resolution 换算成米，再和以米为单位的半径比较
+ 1e-9 容差：浮点误差可能让 0.2 算成 0.20000000000000004，加上容差后，正好在半径上的格子也会被膨胀
不会连锁膨胀：种子只取自原始输入，新膨胀出来的格子不会再作为种子继续扩散
边界裁剪：同样是为了避免负索引从数组末尾倒着取
3. 复杂度分析

设 N 为点云点数，H × W 为栅格尺寸，K 为占据格子数，S 为圆形偏移数（约等于 π(r/res)²）。

函数	时间复杂度	空间复杂度	说明
project_cloud	O(N + HW)	O(N + HW)	过滤和索引都是向量化的 O(N)；新建栅格需要 O(HW)
inflate_grid	O(HW + S² + K·S)	O(HW + K + S)	复制 O(HW)；生成偏移表要遍历 (2·dmax+1)² 个位置；每个偏移对 K 个种子做一次向量化操作

以默认参数为例（分辨率 0.1 m，半径 0.3 m）：dmax = 3，偏移表只需检查 49 个位置，保留约 29 个，所以膨胀的实际开销主要取决于占据格子数 K。Python 层面只对偏移数做循环，种子的计算全部交给 NumPy 完成。

4. 边界案例
案例	处理方式
空点云	reshape(-1, 3) 得到 (0, 3)，返回全零栅格
NaN / Inf 点	isfinite 过滤删除
z 正好等于 z_min 或 z_max	保留（闭区间）
点正好落在格子左边界 / 下边界	属于当前格子
点正好落在地图右边界 / 上边界	ix == width，超出范围，删除
负坐标、越界点	floor 后被范围检查删除，不会因为负索引写到地图另一侧
多个点落在同一格	结果仍为 100
非方形地图	统一按 (height, width) 和 [iy, ix] 处理，宽高没有写死
膨胀半径为 0	偏移表只有 (0, 0)，输出等于输入
半径不是分辨率的整数倍	ceil 确定搜索范围，hypot 判断是否在圆内
障碍贴近地图边缘	膨胀结果被裁剪在地图内部
多个相邻障碍	各自膨胀后取并集，不会连锁扩散
输入数组	只读，输出是新数组
5. 测试与验收结果
5.1 官方单元测试
bash
PYTHONPATH=src/nav/obstacle-map/nav_obstacle_map/python python3 -m pytest -q src/nav/obstacle-map/nav_obstacle_map/tests/test_student_map.py

结果：5 passed。覆盖了投影边界与重复点、空快照、圆盘膨胀（不连锁、不修改输入）、角落与小数半径、非方形地图与多个种子。

5.2 实时地图自检
bash
python3 scripts/check_live_map.py

结果：

PASS: live scan appearance/movement/disappearance, inflation, rolling origin, timestamps, old-scan rejection
5.4 闭环自检
bash
python3 scripts/check_closed_loop.py

结果：

PASS: STUDENT CLOSED LOOP; arrival=21.03s error=0.1711m; no collision; max_speed=0.6500m/s; stale-input stop
指标	结果
到达时间	21.03 s
到点误差	0.1711 m
碰撞	无
最大速度	0.65 m/s
输入断流	机器人正确停车

运行过程中状态依次为 waiting_for_map_and_odom → waiting_for_map_goal → tracking → goal_reached，第二段任务出现 stale_input 后停车，符合预期。

6. Gazebo 仿真任务观察

按 README 第 3 节分别启动 Gazebo、地图、规划、控制和 RViz，机器人从起点 (29.325, 8.475) 出发。

观察结果

勾选 Local inflated map 后，机器人周围出现一个随机器人移动的局部窗口。窗口内的柱子按雷达扫描实时出现，柱子外有一圈膨胀区
LiDAR 点落在柱子的黑色占据区域边缘，说明投影的坐标换算正确
/nav/map 同时保留了先验的 60 根柱子，并叠加了局部窗口中新发现的柱子
发送目标后，规划出的绿色路径绕开了所有黑色区域。机器人横穿柱子林到达目标，没有碰撞

【插入截图】

![局部膨胀地图](docs/images/local_map.png)
![规划路径绕开柱子](docs/images/path.png)
![到达目标](docs/images/goal_reached.png)

关于"看起来像撞上了"：在 RViz 中，机器人周围的红色同心圆是雷达打到地面的点，柱子显示的是膨胀后的大小，因此视觉上会显得非常贴近。以 check_closed_loop.py 的 no collision 作为最终判断依据。

7. 环境与复现
项目	配置
宿主电脑	MacBook（Apple Silicon，ARM 架构）
虚拟机	VMware Fusion，Ubuntu 24.04.3 arm64，Xorg（X11）会话
容器	Docker，镜像 trident-control-course:2026，在 aarch64 上构建成功
软件	ROS 2 Humble、Gazebo Fortress、RViz2
开发方式	在 Mac 上用 VS Code 修改代码，通过 GitHub 同步到虚拟机，在虚拟机中用 TRIDENT_DEV=1 ./scripts/dev.sh 挂载源码运行

复现命令（容器内，/opt/nav_ws）：

bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch nav_simulation environment.launch.py
ros2 launch nav_obstacle_map mapping.launch.py use_sim_time:=true use_global_prior:=true
ros2 launch nav_planning planning.launch.py use_sim_time:=true
ros2 launch nav_legacy_control control.launch.py use_sim_time:=true
ros2 launch nav_bringup rviz.launch.py
ros2 topic pub --once -w 1 /nav/goal geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 1.475, y: 4.375, z: 0.0}, orientation: {w: 1.0}}}'

（每条 launch 命令在单独的终端中运行。）

8. 遇到的问题与解决方法
问题	原因	解决方法
Mac 无法直接运行 Linux 图形环境	作业要求 Linux 桌面和 X11	安装 VMware Fusion 和 Ubuntu 24.04 arm64，登录时选择 Xorg 会话（submit.py 需要录制 X11 桌面）
Gazebo 场景灰屏，RTF 为 0%，整个虚拟机卡死	虚拟机的 3D 加速与 Gazebo 的 ogre2 渲染不兼容	启动 Gazebo 和 RViz 前执行 export LIBGL_ALWAYS_SOFTWARE=1，改用软件渲染。之后 RTF 约 98%
check_live_map.py 报 no map preserving scan timestamp	手动启动的仿真和地图模块仍在运行，脚本伪造的点云被真实雷达数据淹没	脚本会自己启动地图节点，必须在没有其他模块运行时单独执行。关闭所有模块后重跑即通过
RViz 中地图一直闪烁，状态为 no_path	重复启动导致同一节点有 2–3 份（ros2 node list 出现重名警告），多份地图交替覆盖	用 docker rm -f 删除容器后重新创建，每个模块只启动一次
在容器里执行 docker exec 报 command not found	已经在容器内部，容器里没有 docker 命令	通过提示符判断位置：dean@... 是虚拟机，root@... 是容器
终端关闭后提示 No such container	运行 dev.sh 的终端关闭后，容器会被自动删除	重新运行 TRIDENT_DEV=1 ./scripts/dev.sh
9. 总结

两个函数全部完成，并通过了官方单元测试、实时地图自检和闭环自检。实现完全基于输入参数，没有写死地图尺寸、分辨率或柱子位置，可以适应教师更换的参数复测。

通过这次作业，我理解了点云到占据栅格的坐标换算（floor、左闭右开、[iy, ix] 顺序）、膨胀在路径规划中的作用，以及 ROS 2 中节点、话题和 launch 的基本工作方式。