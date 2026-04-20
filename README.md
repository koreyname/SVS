# ORB-SLAM PatchDefense 防御实现说明（基于 `src/ORBSLAM2`）

本文档面向本仓库当前的 **论文实现版本**，目标不是介绍仓库怎么跑，而是把 `src/ORBSLAM2` 里这套 PatchDefense / MotionGuard 防御链路讲清楚：模块放在哪里、每一步实际算了什么、哪些参数真的被代码读取、最终怎样作用到前端、后端和回环。

相关核心代码入口：

- PatchDefense：`src/ORBSLAM2/include/PatchDefense.h`、`src/ORBSLAM2/src/PatchDefense.cc`
- MotionGuard：`src/ORBSLAM2/include/MotionGuard.h`、`src/ORBSLAM2/src/MotionGuard.cc`
- Tracking 接入：`src/ORBSLAM2/src/Tracking.cc`
- 权重落地：`src/ORBSLAM2/include/Frame.h`、`src/ORBSLAM2/src/KeyFrame.cc`
- 前端/后端加权：`src/ORBSLAM2/src/Initializer.cc`、`src/ORBSLAM2/src/PnPsolver.cc`、`src/ORBSLAM2/src/Sim3Solver.cc`、`src/ORBSLAM2/src/Optimizer.cc`
- 回环防御：`src/ORBSLAM2/src/LoopClosing.cc`

---

## 1. 目标与威胁模型

### 1.1 目标

在尽量不损伤正常跟踪与建图稳定性的前提下，抑制“特征点伪造补丁（patch attack）”对位姿估计、局部优化和回环校正造成的漂移或跳变。

### 1.2 威胁模型（工程视角）

攻击者在图像中注入局部补丁，使该区域产生：

- **异常的特征点密度**：局部爆点
- **异常的描述子统计**：过于同质或异常离散
- **异常的方向分布**：梯度方向过度集中
- **异常的纹理统计**：局部熵与周围上下文不一致
- **时间上的突发或持续偏离**：某些统计量在连续帧中异常稳定或异常突变

---

## 2. 总体流程（每帧）

当 `use_patch_defense=1` 时，PatchDefense 会在 `Tracking::GrabImage*()` 中于当前帧 ORB 特征提取完成后、正式进入 `Track()` 前执行。

整条链路可以概括为：

1. **Stage-A：多尺度局部异常统计**
   计算密度、描述子离散度、方向集中度三类局部风险，得到每个点的
   $r_D, r_\sigma, r_\theta$
2. **Candidate Gate：候选点门控**
   用加权分数筛出候选集合 $\mathcal{C}$
3. **Stage-B：RiskMask 风险掩膜**
   在全图或候选附近计算纹理/方向风险，得到 $r_{\text{mask}}$
4. **Stage-T：时间一致性**
   用 EMA + burst 风险得到 $r_{\text{temp}}$，并转成 $w_{\text{temp}}$
5. **融合**
   组合 `risk mask` 与 `temporal` 两支，得到最终权重 $w_i$

最终，权重通过以下路径影响 SLAM：

- **前端**：Initializer、PnP RANSAC、Sim3 RANSAC、PoseOptimization
- **后端**：BA 信息矩阵缩放
- **回环**：safe ratio + cover + 结构分布 KL + 滞后开关
- **动态干扰**：MotionGuard 冻结位姿并可选禁止关键帧

实现上的两个细节：

- 如果 `use_patch_defense=0`，`PatchDefense::ProcessFrame()` 会直接返回，不做任何防御
- 如果启用了 candidate gate 但本帧没有候选点，RiskMask 阶段会被整体跳过，`mvPatchRisk` 全部置零

---

## 3. 配置加载（`args.yaml`）

PatchDefense 参数从根目录 `args.yaml` 读取，格式是 OpenCV `FileStorage` YAML。

真实查找顺序是：

1. 环境变量 `ORB_SLAM_ARGS`
2. `args.yaml`
3. `../args.yaml`
4. `../../args.yaml`
5. `../../../args.yaml`
6. `../../../../args.yaml`
7. `../../../../../args.yaml`

若未找到：使用 `PatchDefenseConfig` 的默认值，并打印提示。

当前仓库里两个最相关的文件是：

- `args.yaml`
  当前实验实际使用的参数快照

顶层结构：

```yaml
use_patch_defense: 1
patch_defense:
  enable_risk_mask: 1
  ...
```
