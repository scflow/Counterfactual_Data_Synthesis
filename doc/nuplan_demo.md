切换到 **nuPlan** 是一个非常明智的选择，因为它是专为规划（Planning）和矢量仿真设计的，不像 nuScenes 那样是“为了感知而生，顺便做做规划”。nuPlan 的 API 设计得更工程化（但也更复杂）。

在 nuPlan 中，你不直接去 `new` 一个 `ScenarioProvider`，而是通常使用 **`NuPlanScenarioBuilder`** 工厂类。它是官方推荐的入口，帮你管理数据库连接、地图加载和场景过滤。

以下是初始化 nuPlan 并获取 `AbstractScenario` 的标准工程化代码方案：

### 1. 准备工作：环境变量与路径

nuPlan 极其依赖路径设置。你需要确保你有以下三个路径：

1. **NUPLAN_DATA_ROOT**: 存放 `.db` 文件（Log 数据库）的地方。
2. **NUPLAN_MAPS_ROOT**: 存放 `.gpkg` 矢量地图的地方。
3. **NUPLAN_EXP_ROOT**: (可选) 存放实验结果的地方。

### 2. 核心代码：获取 AbstractScenario

你需要用到两个核心组件：

* **`NuPlanScenarioBuilder`**: 负责连接数据库和地图工厂。
* **`ScenarioFilter`**: 负责筛选你想要的数据（比如：只跑“变道”场景，或者指定某个 Log ID）。

```python
import os
import logging
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters

# 1. 设置路径 (请修改为你自己的实际路径)
NUPLAN_DATA_ROOT = "/data/sets/nuplan/nuplan-v1.1/splits/mini" # 存放 .db 文件的目录
NUPLAN_MAPS_ROOT = "/data/sets/nuplan/maps"                   # 存放地图文件夹的目录

# 2. 初始化 Scenario Builder
# 这里不需要显式初始化 MapProvider，Builder 内部会处理 MapFactory
scenario_builder = NuPlanScenarioBuilder(
    data_root=NUPLAN_DATA_ROOT,
    map_root=NUPLAN_MAPS_ROOT,
    sensor_root=None, # 如果只做纯向量仿真，不需要加载相机/雷达数据，设为 None 即可加速
    db_files=None,    # None 表示自动扫描 data_root 下的所有 .db 文件
    map_version="nuplan-maps-v1.0" # 默认地图版本
)

# 3. 配置过滤器 (ScenarioFilter) - 这一步至关重要
# 如果不加过滤，它会试图加载数据库里几十万条场景，内存会爆炸
scenario_filter = ScenarioFilter(
    scenario_types=None,        # 比如 ['lane_change']，None 表示不过滤类型
    scenario_tokens=None,       # 如果只想跑特定的场景，填入 Token 列表
    log_names=None,             # 指定只加载某个 .db 文件 (不带后缀)
    map_names=None,             # 指定只跑某个城市的地图 (如 ['us-nv-las-vegas-strip'])
    
    # --- 关键过滤参数 ---
    num_scenarios_per_type=10,  # 每个类型只取 10 个用于测试
    limit_total_scenarios=50,   # 总共只取 50 个
    timestamp_threshold_s=None, # 过滤短时间场景
    ego_displacement_minimum_m=None, # 过滤静止车
    
    # --- 是否打乱顺序 ---
    shuffle=True 
)

# 4. 获取 Scenarios (返回的是一个 Generator)
# nuPlan 设计为 Lazy Loading，遍历到它时才会真正去读盘
scenarios = scenario_builder.get_scenarios(scenario_filter, worker=None)

# 5. 验证并提取 AbstractScenario
for scenario in scenarios:
    # 这里的 scenario 对象就是你想要的 nuplan.planning.scenario_builder.abstract_scenario.AbstractScenario
    print(f"Scenario Type: {scenario.scenario_type}")
    print(f"Token: {scenario.token}")
    print(f"Map Name: {scenario.map_api.map_name}")
    
    # 获取初始状态 (用于你的仿真器初始化)
    initial_state = scenario.initial_ego_state
    print(f"Start Pose: x={initial_state.rear_axle.x}, y={initial_state.rear_axle.y}")
    
    # 你的逻辑代码放在这里...
    break

```

---

### 3. 如何单独获取 MapProvider (如果需要)

通常 `AbstractScenario.map_api` 已经把地图封装好了（它是 `AbstractMap` 接口）。但如果你想**不通过 Scenario，直接读取地图**（比如你想在没有场景的情况下单独测试找车道功能），你需要初始化 `NuPlanMapFactory`。

```python
from nuplan.common.maps.nuplan_map.map_factory import NuPlanMapFactory

# 1. 初始化工厂
map_factory = NuPlanMapFactory(
    map_root=NUPLAN_MAPS_ROOT,
    map_version="nuplan-maps-v1.0"
)

# 2. 构建 MapProvider (其实就是具体的 Map 对象)
# 注意：你需要知道具体的地图名字，比如 'us-nv-las-vegas-strip'
# 这些名字通常对应 map_root 下的文件夹名
map_name = "us-nv-las-vegas-strip"
abstract_map = map_factory.build_map_from_name(map_name)

# 3. 使用地图 API (这就是你要用来查最近车道的方法)
# 假设有一个点 (x, y)
point = (664000.0, 3990000.0) 
layers = abstract_map.get_proximal_map_objects(point, radius=5.0, layers=['lane'])

print(layers['lane'])

```

### 4. 针对你的“反事实仿真”任务的特别提示

因为你要做反事实数据生成，nuPlan 的 `AbstractScenario` 对象对你来说有几个**极其有用**的属性，请务必关注：

1. **`scenario.initial_ego_state`**:
* 这是  时刻的绝对真值。你的 `BicycleModel` 初始化就用这个。


2. **`scenario.get_ego_future_trajectory(...)`**:
* 这是人类司机的**未来真值**。你可以用来画可视化的“绿线”，对比你的仿真结果。


3. **`scenario.map_api`**:
* 这是一个极其强大的对象。你之前的需求“找最近车道”，在 nuPlan 里可以直接调用：
```python
# 查找最近车道
# 需要传入 Point2D 对象
from nuplan.common.geometry.torch_geometry_types import Point2D

current_pos = Point2D(x=..., y=...)
# return_distance=True 会告诉你偏离了多少米
lane, distance = scenario.map_api.get_proximal_map_objects(current_pos, radius=2.0, layers=['lane'])

```




4. **`scenario.get_traffic_light_status_at_iteration(...)`**:
* **千万别忘了红绿灯**。如果是纯向量仿真，专家策略看到红灯如果不停车，数据就废了。nuPlan 提供了完美的 API 来查询当前时刻某个车道是不是红灯。



### 总结

1. 不要手动去写 `ScenarioProvider` 的实现类。
2. 使用 `NuPlanScenarioBuilder` + `ScenarioFilter` 组合拳。
3. 确保环境变量 `NUPLAN_MAPS_ROOT` 和 `NUPLAN_DATA_ROOT` 指向正确。
4. 拿到 `scenario` 对象后，直接调用 `scenario.map_api` 做几何计算。