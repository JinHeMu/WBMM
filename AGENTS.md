# AGENTS.md

> 状态：ACTIVE  
> 本文件是项目级硬规则。详细规则以 `docs/` 下标准为准；如有冲突，以用户当前明确指令和本文件最高优先级规则为准。

## 0. 最高优先级

1. 人必须完整审阅一遍 Agent 输出，未审阅内容不能作为实现依据。
2. 文档任务只能改 `docs/` 和 `docs/img/`，不得修改源码。
3. 代码任务必须先说明改动边界，再改代码；不得顺手重构无关模块。
4. 安全、力控、实机执行相关内容必须人工逐条确认。
5. 不确定的内容必须标记 `TBD`，禁止编造结果、数据和结论。

## 1. 任务分类

开始前先声明任务类型：

```text
DOC       文档任务
CODE      代码任务
MATH      数学/接口契约任务
REVIEW    审查任务
MIXED     混合任务，必须拆开执行
```

## 2. 文档任务

只允许修改：

```text
docs/
docs/img/
```

不得修改：

```text
src/**
CMakeLists.txt
package.xml
launch/
config/*.yaml
test/
vendor/
```

要求：

- 公式使用 Typora 可读的 `$$ ... $$`；
- 图片放在 `docs/img/`，Markdown 用相对路径引用；
- 文档通俗易懂，术语符合移动机械臂、全身规划、MPC、力控领域；
- 明确区分 `CURRENT`、`PROPOSED`、`TBD`；
- 文档状态默认 `DRAFT`，人工审阅通过后才能写 `APPROVED`。

## 3. 代码任务

要求：

- 先写清目标、边界、输入输出和测试方式；
- 优先复用现有 `wbmm_core`、`ta_wbmp`、`wbmm_visualization`、`tracer_jaka_ocs2`；
- 不为未来假想需求提前抽象接口；
- 先出现第二个真实实现，再抽接口；
- 不修改 `vendor/`，除非用户明确要求并单独审查；
- 保持现有可编译、可测试状态；
- 改代码后必须说明：改了哪些文件、为什么、如何验证。

## 4. 数学与接口

以 `docs/math_contract.md` 为准：

- 状态：`x = [x_b, y_b, yaw_b, q1..q6]`，9D；
- 输入：`u = [v, omega, qdot1..qdot6]`，8D；
- 坐标系、单位、轨迹、任务、力控语义不得静默修改；
- 涉及差速底盘时必须处理非完整约束；
- 涉及末端任务时必须区分 `TaskTrajectory` 与 `WholeBodyTrajectory`；
- 涉及力控时必须说明力/力矩坐标系、期望/测量 wrench、导纳/阻抗结构和安全限幅。

## 5. 安全与力控

- 实机默认 `execution_enabled=false`；
- 没有安全门、急停、看门狗、超时和故障恢复说明时，不得建议实机执行；
- OCS2 负责跟踪和局部动态修正，不负责大尺度全局重规划；
- 规划器负责大尺度空间决策和任务可行参考；
- 力控、接触判断、SAFE_HOLD、FAULT 必须人工逐条审查。

## 6. 输出格式

Agent 完成任务后必须给出：

```text
任务类型：
修改文件：
是否修改源码：
验证方式：
不确定项：
人工审查结论：DRAFT / APPROVED / REJECTED
```

## 7. 必读文档

根据任务读取：

- 文档任务：`docs/agent_docs_standard.md`
- 代码任务：`docs/agent_code_standard.md`
- 数学/接口：`docs/math_contract.md`
- 文档总约定：`docs/README.md`

## 8. 最终原则

> 先契约，后代码。  
> 先最小闭环，后抽象。  
> Agent 可以写文档，但不能在文档任务中改源码。  
> 所有 Agent 输出必须经过人完整审阅。
