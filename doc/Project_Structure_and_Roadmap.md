# 项目结构与完成顺序（工程实施版）

## 1. 项目结构（建议落地）

```
Counterfactual_Data_Synthesis/
  README.md
  doc/
    Vector-based Counterfactual Data Generation System for Autonomous Driving.md
    Project_Structure_and_Roadmap.md

  config/
    default.yaml
    perturbations.yaml
    controller.yaml

  src/
    cfdg/
      __init__.py
      ingest/
      map/
      perturb/
      sim/
      control/
      label/
      io/
      metrics/
      utils/

  tools/
    run_pipeline.py
    build_manifest.py
    export_debug_case.py

  tests/
    test_sim.py
    test_control.py
    test_label.py
    test_map.py

  data/
    raw/
    cache/
    output/
```

---

## 2. 完成顺序（里程碑分解）

### 阶段 1：数据可读
1. 搭建 `ingest/`，实现 nuPlan 场景读取（ego + agents + map_name）。√
2. 搭建 `map/`，实现中心线查询与道路边界判断接口（先空实现，后补细节）。√

### 阶段 2：仿真可跑
3. 搭建 `sim/`，实现运动学模型与状态更新。√
4. 搭建 `control/`，实现 PID + IDM 控制输出。√
5. 搭建 `perturb/`，实现扰动库与参数采样。√
6. 集成 `run_pipeline.py`，实现单场景闭环仿真。

### 阶段 3：标注可用
7. 搭建 `label/`，实现碰撞/Off-road/恢复判定与 TTC。√
8. 搭建 `io/`，落盘 Parquet + labels + meta。√

### 阶段 4：规模化与稳定性
9. 实现多进程批量处理与 map 缓存。
10. 构建 `metrics/`，输出统计指标与 manifest。
11. 写基础测试（运动学、控制、标注）。√

---

## 3. 可交付验收点

- **M1**：给定 `scene_token`，输出单条反事实轨迹。
- **M2**：支持 1000+ 场景批处理，产出完整数据集。
- **M3**：通过一致性与性能测试。

---

## 4. 实施记录

- 2026-01-27：完成目录结构创建与包初始化文件（`config/`, `src/cfdg/*`, `tools/`, `tests/`, `data/`）框架落地。
- 2026-01-27：完成工程骨架文件与最小可运行入口（配置文件、核心模块桩、CLI 工具与基础测试）。
- 2026-01-27：补强基础仿真与控制组件（IDM 控制器实现、仿真 rollout 回调式控制、配置深度合并），同步更新控制配置与测试。
- 2026-01-27：实现 nuPlan 场景读取适配（基于场景对象抽取 ego/agents/map_name，支持回调式 scenario_provider），补充相关测试。
- 2026-01-27：实现 map API 包装（中心线查询与 off-road 判断），补充对应测试。
- 2026-01-27：补充测试环境路径初始化（`tests/conftest.py`），确保 `src/` 可被 pytest 发现。
- 2026-01-27：本地测试通过（`python -m pytest -q`，7 passed）。
- 2026-01-27：在 `.venv` 安装依赖并通过测试（`uv pip install -e .[dev]`，`.venv/bin/python -m pytest -q`，7 passed）。
- 2026-01-27：实现扰动库与采样逻辑（`PerturbationFactory`），并补充确定性测试。
- 2026-01-27：扰动模块变更后测试通过（`.venv/bin/python -m pytest -q`，8 passed）。
- 2026-01-27：在 `tools/run_pipeline.py` 接入 nuPlan 官方 ScenarioBuilder/ScenarioFilter，用推荐方式加载 `scenario.map_api` 与场景数据（输出加载概览）。
- 2026-01-27：扫描本地 nuPlan DB，生成完整 ScenarioFilter 配置与数据清单（`config/scenario_filter_full.yaml`，`doc/nuplan_data_inventory.md`）。
- 2026-01-27：`run_pipeline.py` 支持 `--scenario_filter` 额外配置文件，便于加载完整过滤清单。
- 2026-01-27：从完整清单生成轻量过滤配置（`config/scenario_filter_light.yaml`）。
- 2026-01-27：补充 `ScenarioFilter` 必需参数与扩展项（`expand_scenarios`/`remove_invalid_goals` 等），同步更新过滤配置与运行入口。
- 2026-01-27：修正轻量过滤配置的地图匹配（`config/scenario_filter_light.yaml` 指向 `us-nv-las-vegas-strip`）。
- 2026-01-27：补齐 nuPlan 运行依赖（geopandas、rasterio、aioboto3/botocore、retry、requests、tqdm、scipy、opencv-python-headless、pillow、matplotlib、pyquaternion、psutil）。
- 2026-01-27：使用轻量过滤配置成功加载场景（`tools/run_pipeline.py` 输出 token/map/frames）。
- 2026-01-27：同步依赖后再次验证场景加载成功（`tools/run_pipeline.py` 输出 token/map/frames）。
- 2026-01-27：实现 Labeler 与 OutputWriter（碰撞/Off-road/恢复/TTC 计算与落盘输出），并在 `run_pipeline.py` 串联仿真、扰动、标注与写出。
- 2026-01-27：`run_pipeline.py` 增加控制器默认参数兜底，避免缺少 controller 配置导致失败。
- 2026-01-27：`run_pipeline.py` 轻量过滤配置下完成一次闭环输出（生成 `data/output/scenes/<scene_token>/`）。
- 2026-01-27：新增 GIF 可视化脚本（`tools/visualize_gif.py`）用于渲染地图车道与轨迹。
- 2026-01-27：生成 GIF 预览（`data/output/scene_7f61acafc8f45bb9.gif`）。
- 2026-01-28：新增高精地图叠加 GIF 脚本（`tools/visualize_gif_hdmap.py`），支持 ENU 与相对坐标双输出。
- 2026-01-28：高精地图 GIF 生成完成（`data/output/scene_7f61acafc8f45bb9_hdmap_enu.gif`，`data/output/scene_7f61acafc8f45bb9_hdmap_rel.gif`）。
- 2026-01-28：新增原始 vs 反事实轨迹对比渲染（`tools/visualize_gif_hdmap.py`），输出对比 GIF（`data/output/scene_7f61acafc8f45bb9_hdmap_compare_enu.gif`，`data/output/scene_7f61acafc8f45bb9_hdmap_compare_rel.gif`）。
- 2026-01-28：对比 GIF 样式优化，按 nuPlan 渲染风格调整图层配色/顺序与背景（`tools/visualize_gif_hdmap.py` 重渲染对比 GIF）。
- 2026-01-28：开始实施“恢复模式”控制（状态机 + 回正窗口 + 速度下调），接入 `tools/run_pipeline.py` 与 `config/default.yaml`。
- 2026-01-28：恢复模式控制下生成新反事实输出（`data/output/scenes/c0547f04131c51e4/`）。
- 2026-01-28：生成恢复模式对比 GIF（`data/output/scene_c0547f04131c51e4_hdmap_compare_enu.gif`，`data/output/scene_c0547f04131c51e4_hdmap_compare_rel.gif`）。
- 2026-01-28：导出真实坐标系 JSON 轨迹（反事实/真值）：`data/output/c0547f04131c51e4_ego_cf.json`、`data/output/c0547f04131c51e4_ego_gt.json`。
- 2026-01-28：合并导出 Ego(gt/cf)+agents 到单一 JSON（`data/output/c0547f04131c51e4_all.json`）。

---

## 6. 反事实“恢复”增强方案（待实施）

### 6.1 目标
- 从“单次扰动后持续偏离”升级为“偏离 → 纠偏 → 回归车道”的闭环轨迹。
- 确保恢复过程可解释、可控、可复现。

### 6.2 诊断结论
- 当前控制器未形成“回正目标”：中心线为空或误差不足时不会回正。
- 扰动结束后没有恢复阶段/触发条件。
- 纵向控制使用近似恒速，不参与恢复。

### 6.3 改进策略（优先级顺序）
1. **恢复模式状态机**  
   - 条件：`|CTE| > cte_recover_threshold` 或 `|heading_err| > yaw_recover_threshold`  
   - 行为：进入“强纠偏”模式（提高 PID 增益，或加前瞻点）。  
   - 退出：`|CTE| <= eps_cte` 且 `|heading_err| <= eps_yaw` 持续 `N` 帧。

2. **中心线稳健化**  
   - 当 `lane_centerline` 为空时，fallback 到最近 `lane` 对象的 baseline。  
   - 如果仍为空，使用历史中心线缓存。

3. **扰动结束回正窗口**  
   - 扰动结束后触发 `T_recover` 秒的纠偏窗口。  
   - 在窗口内固定使用恢复模式。

4. **纵向控制配合**  
   - 恢复阶段降低期望速度（`desired_speed_recover`）。  
   - 避免在大偏角时高速前进造成轨迹漂移。

### 6.4 配置参数（拟新增）
- `label.recover_time`（已存在，可复用）  
- `recover.cte_threshold`  
- `recover.yaw_threshold`  
- `recover.pid_gain_scale`  
- `recover.min_frames`  
- `recover.desired_speed_recover`  

### 6.5 影响模块
- `tools/run_pipeline.py`：控制逻辑与状态机接入  
- `src/cfdg/map/map_api.py`：中心线 fallback  
- `config/default.yaml`：恢复相关参数  

### 6.6 验收标准
- 反事实轨迹出现明显“回正”趋势（视觉上回归车道）。  
- `Is_Recovered=True` 比例显著高于当前。  
- GIF 对比中能观察到纠偏动作。  

---

## 5. 完成项文件对应表

- 阶段 1-1（ingest）：`src/cfdg/ingest/scene_loader.py`
- 阶段 1-2（map）：`src/cfdg/map/map_api.py`
- 阶段 2-3（sim）：`src/cfdg/sim/bicycle_model.py`，`src/cfdg/sim/rollout.py`
- 阶段 2-4（control）：`src/cfdg/control/pid.py`，`src/cfdg/control/idm.py`，`config/controller.yaml`
- 阶段 2-5（perturb）：`src/cfdg/perturb/perturbation.py`，`tests/test_perturb.py`
- 阶段 3-7（label）：`src/cfdg/label/labeler.py`，`tests/test_label.py`
- 阶段 3-8（io）：`src/cfdg/io/writer.py`，`tests/test_io.py`
- 阶段 4-11（tests）：`tests/test_sim.py`，`tests/test_control.py`，`tests/test_label.py`，`tests/test_map.py`，`tests/conftest.py`
