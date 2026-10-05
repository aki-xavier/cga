<!-- markdownlint-configure-file {"MD013": false} -->
# 跨平台方案调研

本文评估 cga 的跨平台移植面，对比候选方案，给出分阶段路线。

## 1. 现状：平台锁定面只有一层

| 层 | 平台依赖 | 可移植性 |
| --- | --- | --- |
| `cga-core`（代数核心） | 纯 Rust f64，无外部依赖 | 已跨平台，含 WASM |
| `bake`（网格烘焙） | 纯 CPU f64 | 已跨平台，Linux/CI 可运行 |
| CGS 语言 / 场景报告 / glTF / PNG | 纯 stdlib Rust | 已跨平台 |
| **逐像素内核（`cga-gpu`）** | **mlx-rs → MLX C++ → Metal，仅 Apple Silicon** | **唯一锁定点** |

结论：跨平台问题 = 替换 `cga-gpu` 里 13 个文件的 MLX 调用面。其余全部已经可移植。

## 2. 移植面量化

- 规模：13 个文件，约 6 000 行。`mlx_rs::` 直接命名导入仅 10 处，其余全部经 `mlxops.rs`（94 行）和 `Array` 方法。
- 算子词汇表：约 30 个唯一算子。三类：
  - 逐元素/广播：`add/sub/mul/div/sqrt/abs/sin/cos/atan/exp`，`select`（112 处）、`stack`（60 处）、`broadcast_to`（22 处）、`zeros/ones/full` 系列。
  - 规约：`sum_axes`。
  - 索引/排序：`take_axis`、`argsort_axis`、`sort_axis`、`concatenate`、`eye`。
- 同步点：`as_slice()` 读回 33 处（CSG 分类、包围盒、报告）。
- 精度：float32 单向数据流。控制流全部在 CPU 侧（Rust 循环、递归 CSG、分块）。

这是标准的 NumPy 式 eager 数组计算。没有自定义 Metal kernel。没有卷积。没有 autodiff。任何"张量库"或"计算着色器"都能表达这套词汇。

## 3. 候选方案对比

| 方案 | 目标平台 | 与现有代码的语义距离 | 主要风险 |
| --- | --- | --- | --- |
| **A. ndarray + rayon（CPU 后端）** | Linux/Windows/macOS/CI，理论可 WASM | 极小：NumPy 语义一一对应 | 朴素 eager 每算子一次遍历+分配，比 MLX 融合执行慢约一个量级；`argsort` 需手写 |
| **B. wgpu 计算着色器（WGSL）** | Vulkan/Metal/DX12/WebGPU，含浏览器 | 大：整个逐像素管线搬进 shader，CSG 递归控制流需拍平 | 工作量最大；float32 跨 GPU 不保证位一致 |
| **C. CubeCL（Rust `#[cube]` kernel）** | Metal/Vulkan/CUDA/WGPU，单一 Rust 源 | 中：用 Rust 写 kernel，保留宿主控制流 | 生态年轻，API 稳定性需核实 |
| **D. burn Tensor API（wgpu/ndarray 后端）** | 桌面全平台 + WASM | 小：eager 张量 API 与 MLX 风格最接近 | `sort/argsort/复杂索引` 各后端覆盖不一；eager 小算子在 wgpu 上调度开销高 |
| **E. candle** | CPU + CUDA + Metal | 小 | Metal 后端仍锁 Apple；CUDA 不覆盖 AMD/集显；WASM 仅 CPU |
| **F. 各平台原生 GPU API** | Metal(MLX)/CUDA/D3D12/Vulkan 各一份 | 中：算子词汇表任何 GPU 计算 API 都能表达 | N 个后端 = N 份 kernel 源码：每加一个图元各写一遍、各测一遍、各维护一套金样；D3D12 的 Rust 生态薄 |

原生路线补充（2026-10-06 核实，MLX 官方安装文档 + mlx-rs README）：

- MLX 上游（C++ 核心）的平台覆盖是"两个半平台"：macOS/Apple Silicon 一等（Metal + Accelerate）；Linux 官方支持，含正式维护的 CUDA 后端（`mlx[cuda12]`/`mlx[cuda13]`，要求 glibc 2.35+、SM 7.5+、驱动 550.54.14+（CUDA 13 需 580+）、CUDA toolkit + cuDNN）；Linux 纯 CPU 版 `mlx[cpu]`（需 BLAS/LAPACK）。**Windows 不支持**（构建系统中 `WIN32` 仅出现在排除分支）；AMD/ROCm、Vulkan、WebGPU 无后端，浏览器不可达。
- mlx-rs（本项目依赖的绑定）feature flags 只有 `metal` 与 `accelerate`，**没有 `cuda` 开关**：上游 CUDA 后端存在，但绑定未暴露 `MLX_BUILD_CUDA=ON` 构建路径。
- 因此 "MLX 直通 Linux+NVIDIA" 的 spike 具体化为：给 mlx-rs 的 build.rs 补 `cuda` feature（传 `-DMLX_BUILD_CUDA=ON`、链 CUDA/cuDNN），可能需向 oxiglade/mlx-rs 提 PR 或本地 patch。成功后现有约 6 000 行内核代码不改一个算子即可在 Linux+NVIDIA 运行。
- 精度红利：Metal 无 f64 是渲染内核锁 f32 的根因；CUDA 有原生 f64。MLX-CUDA 走通后，未来把区间算术/高精度求交下放 GPU 有理论通道（当前代码写死 f32，需另立项）。
- 结论不变：MLX 覆盖不到 Windows 原生、AMD/集显、浏览器；便携后端（wgpu/CubeCL）与 CPU 参考后端的必要性不变。

**结论：原生后端是性能优化，不是移植手段。** 先用单源码便携后端把"全平台能跑且正确"做成事实，再按实测差距（> 2×）决定为哪个平台付一份原生源码。混合策略——trait 之下三类后端分工：

1. **CPU 参考后端（ndarray + rayon）**：所有平台的正确性基准和 CI 底线。
2. **单源码便携后端（wgpu 或 CubeCL）**：覆盖全部平台和浏览器，是"能跑"的保证。
3. **原生性能后端，只留收益大的**：MLX on macOS 已在位，继续当性能基准；Linux+NVIDIA 的 cheapest 路径是给 mlx-rs 补 `cuda` feature（见方案 F 补充），CUDA 后端只在目标场景（NVIDIA 工作站批量合成数据）需要时再加；D3D12/Vulkan 原生不做——便携后端已覆盖。

## 4. 推荐路线：先抽 trait，再三后端

1. **抽出 `GpuOps` trait**。把 `mlxops.rs` + 约 30 个算子定义成 trait。MLX 作为 macOS 后端原样保留。预期结果：行为零变化，236 个测试与 RMSE = 0 金样全部保持。
2. **实现 ndarray + rayon 后端**。解锁 Linux/Windows/CI。无头渲染、报告、bake、全部测试可在 GitHub Linux runner 跑。作为正确性后端，性能可接受（`demo_bake` 已证明 CPU 路径可行）。
3. **实现 GPU 通用后端**。优先 CubeCL（保持 Rust 表达、一条源码到四个图形 API）。若其成熟度不达标，退到 wgpu/WGSL 手写 kernel。浏览器演示是该后端的副产品（cga-core 编译到 WASM 无阻碍）。

关键设计约束：trait 粒度停留在"逐算子"会让 CPU 后端慢。可接受的折中——CPU 后端定位为 CI/正确性后端；性能后端（MLX、CubeCL/wgpu）在 trait 内部自由融合算子。

## 5. 风险与限制

- **金样不可跨后端位一致**。float32 求和顺序与超越函数实现因 GPU 而异。RMSE = 0 只在同后端成立。跨后端金样必须改为容差断言（如 RMSE < 1e-3）或按后端各存一份。
- **CubeCL 生态成熟度未验证**。本调研未联网确认其最新 API 稳定性。动手前需要一次 spike：`sphere_intersect` + `select` + `argsort` 三件套移植试验。
- **CSG 递归在 shader 内不可行**。wgpu 路线必须把逐区间分类器拍平成固定迭代或分派多 pass。CubeCL 路线的宿主侧控制流不受影响。
- **Apple 路径不变**。MLX 后端继续是 macOS 上的性能基准。

## 6. 验证方式

- 阶段 1 完成判据：`make test` 236 个测试全过，金样 RMSE = 0 不变。
- 阶段 2 完成判据：`cargo test --no-default-features --features cpu-backend` 在 Linux CI 全过。`render_cgs orbit.cgs` 输出与 MLX 金样 RMSE < 1e-3。
- 阶段 3 完成判据：同一代码在 macOS（Metal）、Linux（Vulkan）、Windows（DX12）渲染金样达标。WASM 构建产出浏览器可跑的 `render_cgs`。
