# 工程实现式方案：基于 nuPlan 的纯向量反事实自动驾驶数据生成系统

**(Engineering Spec: nuPlan-based Vector Counterfactual Data Generation System)**

## 0. 目标与边界

### 目标
- 从 nuPlan 场景中批量生成反事实轨迹，用于训练/评测规划控制与行为预测。
- 保证高效率、可复现、可控扰动、物理合理。

### 边界
- 仅做向量/状态级仿真，不渲染图像。
- 不重建全量感知，仅使用 nuPlan 轨迹 + 地图 + 简单规则控制。

---

## 1. 输入/输出契约

### 输入
- `nuplan_db_path`：nuPlan 数据库根目录
- `map_root`：nuPlan 地图根目录
- `scene_tokens`：待处理场景列表（或通过过滤规则生成）
- `config.yaml`：参数配置

### 输出
- `output_root/`：生成数据目录
- `output_root/manifest.json`：全量索引与统计
- `output_root/scenes/<scene_token>/`：单场景结果
  - `meta.json`：场景元信息、扰动参数、seed
  - `trajectory.parquet`：主轨迹数据
  - `labels.json`：碰撞/恢复/最小 TTC 等标签
  - `debug.json`（可选）：过滤原因、异常信息

---

## 2. 依赖与环境

- Python 3.9+（建议与 nuPlan devkit 兼容）
- nuPlan devkit（本地安装，供数据读取与地图 API）
- 常用依赖：numpy, pandas, shapely, pyarrow

---

## 3. 目录与模块划分（推荐）

```
Counterfactual_Data_Synthesis/
  doc/
  src/
    ingest/        # nuPlan 数据读取与场景选择
    map/           # 地图解析与几何查询
    perturb/       # 扰动策略库
    sim/           # 运动学仿真与专家控制
    label/         # 标签、碰撞、TTC 计算
    io/            # 数据写出与manifest
    utils/
  config/
    default.yaml
  tools/
    run_pipeline.py
```

---

## 4. 数据结构定义（内部统一接口）

### 4.1 基本结构

- `EgoState`：`t, x, y, yaw, v, a, steer`
- `AgentState`：`track_token, t, x, y, yaw, v, size(l,w), type`
- `Frame`：`t, ego: EgoState, agents: List[AgentState]`
- `Scenario`：`scene_token, map_name, frames: List[Frame]`

### 4.2 轨迹表 schema（输出）

`trajectory.parquet` 字段（按时间升序）：
- `t`：时间戳（秒）
- `x, y, yaw`：位置与朝向（ENU）
- `v`：速度（m/s）
- `a`：加速度（m/s^2）
- `steer`：前轮转角（rad）
- `cmd_a, cmd_steer`：控制命令（用于诊断）
- `perturb_on`：是否处于扰动时段

---

## 5. Pipeline（工程流程）

1. **Scene Selection**
   - 根据地图名、速度区间、车道曲率等规则筛选场景
2. **Load**
   - 读取 `scene_token` 对应 Ego/Agents 轨迹与地图要素
3. **Seed**
   - `seed = hash(scene_token) ^ global_seed`
4. **Perturb Plan**
   - 随机选择扰动类型、强度、触发时刻 `t0`
5. **Sim Loop**
   - `perception`：车道中心线、前车距离等
   - `control`：PID + IDM 输出 `cmd_a, cmd_steer`
   - `inject`：在扰动窗口内覆盖/叠加控制
   - `propagate`：运动学模型更新状态
   - `record`：写入轨迹
6. **Validate & Label**
   - 碰撞检测 / Off-road / 恢复判定 / Min TTC
7. **Write**
   - 落盘 `meta.json + trajectory.parquet + labels.json`
8. **Manifest Update**
   - 统计成功率、事故率、过滤原因分布

---

## 6. 关键模块实现细节

### 6.1 Ingest（nuPlan 数据读取）

- 输入 `scene_token`，从 nuPlan DB 拉取：
  - Ego 历史/未来状态序列
  - 邻车 track 列表及其时间对齐状态
  - 地图名与可用地图 API

**注意**：采样间隔 `Δt` 以 nuPlan 原始时间戳为准，不硬编码。

### 6.2 Map（几何查询）

- `get_lane_centerline(x, y)`：返回最近车道中心线片段（waypoints）
- `is_off_road(ego_polygon)`：判断是否超出道路边界
- `get_speed_limit(x, y)`：用于纵向控制上限

### 6.3 Perturb（扰动库）

**扰动类型**
- Impulse：单次强干预（1~3 帧）
- Continuous：持续干预（τ 秒）
- Semantic：意图偏移（沿相邻车道偏移）

**扰动参数化**
- `steer_delta`：`[min, max]`
- `acc_delta`：`[min, max]`
- `duration`：`[min, max]`
- `trigger_t`：按时间窗或事件条件采样

### 6.4 Sim（运动学模型）

**状态**：`[x, y, yaw, v]`

**更新**：
- `x_{t+1} = x_t + v_t * cos(yaw_t) * Δt`
- `y_{t+1} = y_t + v_t * sin(yaw_t) * Δt`
- `yaw_{t+1} = yaw_t + v_t / L * tan(δ_t) * Δt`
- `v_{t+1} = v_t + a_t * Δt`

**物理约束**
- `|δ| <= δ_max`
- `a_min <= a <= a_max`
- `|a_lat| <= a_lat_max`，其中 `a_lat = v^2 * tan(δ) / L`

### 6.5 Expert（PID + IDM）

**PID 横向控制**
- 输入：最近中心线点、CTE、Heading Error
- 输出：`cmd_steer`

**IDM 纵向控制**
- 输入：前车距离 `d`、相对速度 `Δv`
- 输出：`cmd_a`

### 6.6 Label（碰撞/TTC/恢复）

- **碰撞**：Ego polygon 与任一 agent polygon 相交
- **Off-road**：Ego polygon 不在道路区域
- **Recovery**：在 `T_recover` 内满足 `CTE <= eps_cte` 且 `|heading_err| <= eps_yaw`
- **Min TTC**：基于相对速度与距离的近似 TTC 计算

---

## 7. 配置文件规范（config.yaml）

```yaml
seed: 2026
sample:
  t_hist: 2.0      # 秒
  t_fut: 6.0       # 秒
  dt: null         # 若为 null，使用 nuPlan 原始间隔
vehicle:
  wheel_base: 2.8
  steer_limit: 0.6
  a_min: -6.0
  a_max: 3.0
  a_lat_max: 4.0
perturb:
  types: [impulse, continuous, semantic]
  impulse:
    steer_delta: [-0.4, 0.4]
    acc_delta: [-2.0, 2.0]
    duration_steps: [1, 3]
  continuous:
    steer_delta: [-0.2, 0.2]
    acc_delta: [-1.0, 1.0]
    duration_sec: [0.5, 2.0]
  semantic:
    lateral_offset: [-1.5, 1.5]
controller:
  pid:
    kp: 1.2
    ki: 0.0
    kd: 0.2
  idm:
    desired_speed: 13.9
    min_gap: 2.0
    time_headway: 1.2
label:
  recover_time: 3.0
  eps_cte: 0.3
  eps_yaw: 0.1
output:
  format: parquet
  overwrite: false
```

---

## 8. 复现性策略

- 固定 `global seed`
- `scene_token` 参与扰动参数采样
- 完整记录：扰动类型、强度、触发时刻、控制器参数

---

## 9. 性能与并行策略

- 以场景为单位并行（多进程/多线程）
- Map 解析结果缓存（按 `map_name`）
- 轨迹写出采用批量 Parquet

---

## 10. 运行方式（CLI 形态）

```bash
python tools/run_pipeline.py \
  --config config/default.yaml \
  --nuplan_db /data/nuplan/db \
  --map_root /data/nuplan/maps \
  --output /data/counterfactual_out
```

---

## 11. 测试与验证

- **单元测试**：运动学模型、PID/IDM、碰撞检测
- **一致性测试**：同一 `scene_token` 生成结果应稳定一致
- **性能测试**：统计每秒生成样本数

---

## 12. 交付清单

- 文档：本工程规范
- 代码：核心模块 + 运行入口
- 数据：反事实轨迹集 + manifest
