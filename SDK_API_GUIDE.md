# Acquisition SDK 0.1.1 接口使用说明

本文对应开发发布包中的 `acquisition_sdk.pyi`（Python 3.11、Windows x64）。`.pyi` 是类型提示文件，实际执行的是同目录的 `acquisition_sdk.cp311-win_amd64.pyd`；不是单独可运行的 SDK。示例从发布包目录运行，并保留包内 `model/` 和配置文件。`Any` 表示提示文件未限定类型，不能据此认为可以传入任意对象。

下文覆盖本次最小接口文件的全部公开名称及其方法。以下划线开头的名称是内部辅助定义；末尾单独说明，不建议业务代码依赖。源代码依据：`tools/sdk_public_api.json`、`acquisition_sdk/_internal/` 和发布包中的 `acquisition_sdk.pyi`。项目实际接线参见 `control/PipelineRuntime.py` 和 `control/DataProcessingProcess.py`。

逐帧激光异常恢复及配置方法见同包的 README。

## 版本与基本类型

| 名称 | 功能及输入 | 输出 |
| --- | --- | --- |
| `__version__` | SDK 版本常量，本版为字符串 `"0.1.1"`。 | `str` |
| `SDK_API_VERSION` | Python 接口版本常量，本版为 `1`。 | `int` |
| `Int16()` | `typing.Annotated` 的有符号 16 位整数字段标记，无参数。 | 标记对象，占 2 字节。 |
| `Int32()` | 有符号 32 位整数字段标记，无参数。 | 标记对象，占 4 字节。 |
| `Repeat(count: int)` | 报文列表反序列化默认元素数量。`count` 为元素个数。 | 标记对象；运行时可用 `<字段名>_count` 覆盖。 |
| `PlcFrame` | 使用 `@dataclass` 和字段注解定义报文的基类，默认 `BYTE_ORDER='>'`（大端）。 | 帧对象；帧字段由子类定义。 |

`PlcFrame` 的 `to_bytes()` 接收帧对象并返回载荷 `bytes`；`from_bytes(data: bytes, offset=0, **kwargs)` 从指定偏移解析，返回 `(帧实例, 下一个偏移)`。这两个运行时方法当前未列入精简 `.pyi`，IDE 可能不能提示；项目已有报文定义见 `model/MovingFrameData.py`。`to_bytes()` 本身不加帧头尾，`PlcManager` 返回的 PLC 对象在发送时添加帧头尾并按 `FRAME_SIZE` 补齐。

```python
from model.MovingFrameData import SendMovingFrameData

frame = SendMovingFrameData(Enable=1, Operate=2)
payload = frame.to_bytes()
restored, offset = SendMovingFrameData.from_bytes(payload)
assert offset == len(payload) == 302
```

报文字段、位和轴对应关系见发布包 `README.md` 的 PLC 报文部分；字段注释与当前设备配置尚需 PLC 联调确认。

## PLC 通讯

### `PlcManager(config_path: str)`

读取 PLC TOML 并创建管理对象。`config_path` 是 `PlcConfig.toml` 路径；构造时需要配置中有 PLC `"1"`，底层还从当前工作目录的 `model/tomls/ReadDataConfig.toml` 读取接收超时。输出为管理对象，读取或配置错误会抛异常。

| 方法 | 输入与功能 | 输出 |
| --- | --- | --- |
| `get_plc(plc_id: str)` | 根据字符串 ID（配置中 `"1"` 至 `"5"`）获取 PLC。 | PLC 连接对象；不存在时抛 `ValueError`。 |

返回对象在 `.pyi` 中标记为 `_BasePlc`，不应直接依赖其内部类名：

| 方法 | 输入与功能 | 输出 |
| --- | --- | --- |
| `connect(connection_type='tcp')` | 连接 `tcp` 或绑定 `udp`；无参数默认 TCP。 | `bool`，连接成功为 `True`，失败为 `False`。 |
| `disconnect(connection_type='tcp')` | 关闭已建立的对应连接。 | `None`；无对应连接时抛 `ValueError`。 |
| `scan(connection_type='tcp', **kwargs)` | 接收并解析 PLC 帧；TCP 会先发送读请求。`kwargs` 可传 `<字段名>_count` 覆盖列表长度。 | `model.MovingFrameData.ReceiveMovingFrameData`；通讯超时或数据错误可能抛异常。 |
| `send_frame(frame)` | 输入 `PlcFrame` 子类实例，序列化并通过 TCP 发送。 | `None`；未连接 TCP 时抛 `ConnectionError`。 |

## 点云处理

| 接口 | 输入与功能 | 输出 |
| --- | --- | --- |
| `DataFilter()` | 构造滤波对象。 | `DataFilter` 实例。 |
| `DataFilter.voxelReductionFilter(points, voxel_size=10.0)` | `points` 为 NumPy `N×3` XYZ 点数组；`voxel_size` 是体素边长，使用点云坐标单位。 | 降采样后的 `M×3` NumPy 数组；空输入返回 `(0, 3)` 数组。 |
| `DataSplitting()` | 构造分割工具；本版精简 `.pyi` 没有提供其方法声明。 | 实例；不把未声明方法视作本交付接口。 |
| `DataFindBlocks()` | 构造完整工件识别对象，从当前工作目录加载 `model/tomls/ProcessConfig.toml`。 | 实例。 |
| `DataFindBlocks.start_process(point_cloud, lidar_status, data_dir)` | `point_cloud` 为 `N×3` XYZ NumPy 点云；`lidar_status` 是激光状态整数；`data_dir` 为方向编号。执行滤波、分层、工件分区和框体识别。 | 按配置 `send_data_type`：1 返回逐帧分区列表，其他值返回 `BlockData`；点数不足及处理内部异常均可能返回 `[]`，不能单靠空列表区分两者。 |
| `build_side_frame(accum, side, process_config)` | `accum` 是方向名到点云数组列表的字典；`side` 为 `left`、`right`、`left_upper` 或 `right_upper`；`process_config` 是分层参数字典。侧面按 Y 分层统计 X 范围，顶部按 X 分层统计 Y 范围。 | `AxisFrameData`；无有效点的层为零。 |

```python
import numpy as np
from acquisition_sdk import DataFilter, build_side_frame

points = np.array([[100., 1200., 30.], [120., 1201., 30.]])
reduced = DataFilter().voxelReductionFilter(points, voxel_size=10.0)
frame = build_side_frame({'left': [reduced]}, 'left',
                         {'combined_y_min': 1100, 'combined_y_max': 1300, 'y_threshold': 10})
print(frame.FrameData)
```

## 数据对象

以下均为 dataclass，构造参数是表中字段，默认值均为 `None`。调用构造器返回对应数据对象；读取字段即取得对应值，并不自动执行识别或分枪。

| 对象 | 字段输入与含义 |
| --- | --- |
| `AxisFrameData` | `FrameData`：分层数据列表。每项有 `H_Axis`（分层坐标）、`V_Axis_Max`、`V_Axis_Min`（该层另一坐标最大/最小值）。侧面为 Y/X，顶部为 X/Y。列表项在精简提示中显示为 `_FrameAxisData`，由 SDK 生成，不是 PLC 的轴数据。 |
| `BlockData` | `lidar_status`：激光状态（0 正常、1 遮挡、2 严重异常）；`fifo_frame_pos`：工件前端相对激光距离（mm）；`data_dir`：1 左、2 右；`jig_data`、`outside_data`、`inside_data`：挂具、外框、内框列表；`distribe_gun_list`：设备分枪结果列表。 |
| `DistribeGunData` | `gun_id` 枪编号；`gun_y_enable` Y 轴选通；`gun_y_upper`、`gun_y_downer` Y 轴往复上下位置；`gun_r_angle` R 轴角度。 |
| `GunGroupData` | `group_type` 分组类型（如 `outside`、`inside`、`up`、`down`）；`group_id` 分组编号；`gundata_list` 为 `DistribeGunData` 列表。 |
| `SingleMachineData` | `machine_id` 设备编号；`gun_groups` 为 `GunGroupData` 列表。 |

`BlockData` 的挂具、外框、内框数据类型在精简 `.pyi` 中分别为 `_JigData`、`_OutsideData`、`_InsideData`、`_SubInsideData`：挂具字段为 `jig_id` 和 `jig_y_min/max`、`jig_z_min/max`；外框字段为 `outside_x/y/z_min/max`；内框有 `inside_id`、`subinside_datalist`；子内框有 `subinside_id`、`subinside_x/y/z_min/max`。这些是结果的结构说明，不建议从带下划线的名称直接构造。

## 帧和工件队列

### `FrameQueueManager(system_config_path, stack_size=1500, y_min=1100, y_max=3800, y_threshold=10, x_min=0, x_max=900, x_threshold=10, strategy_name='frame_by_frame', machine_config=None)`

输入分别为系统配置路径、队列长度、侧面 Y 和顶部 X 的分层起止范围及间隔、策略名称和设备配置字典。输出为队列管理对象；范围上限不包含在分层点中。策略为 `frame_by_frame`、`complete_workpiece` 或 `continuous_bidirectional`。完整工件策略按配置中的设备安装方向和 SN 建队列；连续策略按激光方向建立三个工位。

| 方法 | 输入与功能 | 输出 |
| --- | --- | --- |
| `create_empty_frame_y()` / `create_empty_frame_x()` | 无参数，按构造时 Y/X 范围生成全零分层帧。 | `AxisFrameData`。 |
| `push_frame(direction, frame_data, repeat_count=1)` | 逐帧模式，将 `AxisFrameData` 按方向前插指定次数并截断尾部；方向不存在或次数不大于零时不写入。 | `None`。 |
| `push_workpiece(direction, sn, data)` | 完整工件模式，向方向和设备 SN 对应队列写入工件的深拷贝；队列满时左移。 | `(成功: bool, 插入下标: int 或 None, 是否因满而左移: bool)`。 |
| `shift_queue(direction, sn)` | 完整工件队列左移，丢弃最旧项。 | 成功 `True`，方向或 SN 不存在为 `False`。 |
| `clear(direction)` | 清空指定方向的逐帧或完整工件队列。 | `None`；连续模式不支持手动清空，抛 `ValueError`。 |
| `continuous_slot_empty(index)` | 连续模式，检查三个工位的下标 `0`、`1`、`2` 是否在所有方向均为空。 | `bool`。 |
| `put_continuous_workpiece(packet, plc_status)` | 连续模式，将工件写入激光工位；`packet` 以 `translate_data_origin` 选择 `all_data` 或各方向 `<direction>_data`，`plc_status` 为 PLC 状态位整数。 | 成功 `True`；工位占用为 `False`；缺少方向数据抛 `ValueError`。 |
| `update_continuous_stations(plc_status)` | 连续模式根据 PLC 到位、放行反馈推进工位。 | `int`，PC 侧工位放行位掩码。 |

## 进程与点云保存

### `LidarAcquisitionProcess(...)`

构造采集子进程：`pulse_queue` 是 PLC 脉冲/状态输入队列；`raw_data_queue` 是原始处理数据输出队列；`viz_queue` 是预览输出队列；`lidar_config` 是方向到激光配置的字典；`config_dir` 是配置目录；`strategy_name` 默认 `frame_by_frame`；`heartbeat` 是共享心跳值或 `None`，`heartbeat_interval_s` 默认 1 秒。可选 `save_pool_descriptor`、`save_task_queue`、`save_ack_queue`、`save_status_queue`、`stop_event` 用于连接保存服务，通常从 `PointCloudSaveService` 提供。

构造器返回 `LidarAcquisitionProcess` 实例，调用继承的 `start()` 后运行；结果通过队列输出而不是直接返回。它读取 `ReadDataConfig.toml`、`SprayConfig.toml`、`ProcessConfig.toml`，且可能创建数据目录。各队列消息具体结构以当前项目控制代码为准。

### `PointCloudFileProcess(files, viz_queue, status_queue, heartbeat, heartbeat_interval_s=1.0, read_data_config=None)`

开发用 TXT 预览子进程。`files` 为 TXT 路径序列；`viz_queue` 接收预览消息；`status_queue` 接收加载状态；`heartbeat` 为共享心跳值或 `None`；间隔单位秒；`read_data_config` 为坐标转换配置字典或 `None`。构造返回进程对象，`start()` 后载入并发送包含 `points`、`boxes`、`preview_processed` 的字典到预览队列，状态或错误通过状态队列发送；进程保持运行，需由管线停止。

### `resolve_txt_files(data_path, data_name=None)`

`data_path` 为 TXT 文件或目录；可选 `data_name` 在目录中过滤工件文件，支持 `combined_`、`left_`、`right_`、`left_upper_`、`right_upper_` 前缀。返回绝对路径字符串元组；路径不存在抛 `FileNotFoundError`，没有匹配 TXT 抛 `ValueError`。

### `PointCloudSaveService(config, output_dir, viz_queue=None)`

创建点云保存服务和共享内存资源。`config` 为配置字典，`output_dir` 为保存目录，`viz_queue` 为可选预览队列；构造返回服务对象。可使用下列方法；应在停止采集后释放资源，推荐由 `stop_pipeline` 统一关闭。

| 方法 | 输入与功能 | 输出 |
| --- | --- | --- |
| `from_config_file(config_path, output_dir, viz_queue=None)` | 类方法，读取 TOML 后创建服务。 | `PointCloudSaveService` 实例。 |
| `start(timeout_s=10)` | 启动保存子进程并等待就绪，超时单位秒。 | `None`；超时或启动失败抛 `RuntimeError`。 |
| `poll_status()` | 无参数，取出当前已到达的保存状态消息。 | 消息字典列表，可为空；同时记录错误和排空状态。 |

### `stop_pipeline(processes, acquisition, service, acquisition_timeout_s=15, save_timeout_s=120)`

停止进程并等待采集封存、保存排空：`processes` 为进程集合，`acquisition` 为采集进程或 `None`，`service` 为保存服务或 `None`，两个超时单位均为秒。返回错误说明字符串列表，空列表表示未记录关闭错误；采集进程无法退出时会抛 `RuntimeError`。此调用可能阻塞，应在 UI 线程之外运行。

## 内部声明和边界

精简 `.pyi` 中 `LidarAcquisitionProcess` 的 `_text(chinese, english)` 按策略选择文本、`_get_max_chaincountcm()` 返回链条环长度、`_publish_ui_status(status, force=False)` 发布状态、`_update_pulse_data()` 返回当前脉冲/FIFO/状态、`_get_fifo_step_delta(last_fifo, current_fifo)` 计算环形 FIFO 步长；`FrameQueueManager._is_empty_data(data)` 返回是否为空。这些方法虽出现在提示文件中，但以下划线开头，供项目内部使用，不承诺稳定接口。`_WireType`、`_BasePlc`、`_FrameAxisData` 及各 `_...Data` 是提示文件的辅助类型；公开数据由 SDK 返回或通过不带下划线的接口访问。

本文仅说明软件接口；PLC 报文字段和设备轴、设备使能位的正确对应仍需与实际 PLC 程序及现场设备核对。
