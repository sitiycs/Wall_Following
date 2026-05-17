# Wall Following Robot with Vision Navigation

基于 CoppeliaSim 的移动机器人，融合超声波传感器与视觉识别，实现沿墙行走、避障、红色目标跟踪。

## 一. Features

- **沿墙行走**：P 控制器保持恒定距离（目标 0.25m），Kp=20 时响应快且稳定
- **多传感器避障**：两个超声波传感器 + 加权融合，无死角、无锯齿
- **视觉导航**：HSV 颜色空间识别红色目标，视觉伺服控制转向
- **状态机**：搜索红色 → 导航 → 到达，自动切换控制模式

## 二. Quick Start

### 仿真场景

在 CoppeliaSim 中打开场景文件

[Pioneer P3DX 小车 + 红色方块 + 视觉传感器](Coppeliasim_Scene/pioneer_position_control_with_PID_with_turning.ttt)

### 依赖

- Python 3.8+

- `pip install coppeliasim-zmqremoteapi-client opencv-python numpy`

### 运行

```bash
git clone https://github.com/sitiycs/Wall_Following.git
cd Wall_Following/src/final
python position_control_with_imagesensor_v3.py
```

## 三. 迭代记录

[历史版本合集 + 调参对比 + 最终版：视觉导航 v2.0](docs/Iteration_document.md)

## 四. Demo Video

[videos](docs/video_link.md)

## 五. 实验报告

[report](docs/report.pdf)
