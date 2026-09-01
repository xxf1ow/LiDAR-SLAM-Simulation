# Agent Note: Project documentation ownership

Status: implemented

## Problem

项目文档容易积累某次开发会话产生的计划、迁移进度、验收脚本和结果报告。这些材料会把历史状态、临时路径和一次性检查带入当前文档，增加搜索噪音并与代码事实漂移。

## Decision

维护文档只保留当前事实、长期合同和确实会重复使用的贡献者命令。根 README 是项目入口；`docs/architecture.md`、`docs/development.md` 和 `docs/testing.md` 分别拥有当前系统结构、开发流程和测试策略；模块 README 拥有消费者合同、配置语义、限制和可见效果；Agent Notes 只保留未来仍可能影响决策的理由、取舍和活动提案。

一次性计划、阶段状态、迁移清单、验收脚本、运行日志和结果报告不进入维护文档。任务完成后，当前事实更新到 owner，运行证据保存在仓库外，过程由 Git 历史或外部工作记录承载。没有稳定重复需求时不创建 cookbook、postmortem 或其他分类目录。

`scripts/doc-budgets.manifest.json` 对项目自有 Markdown 设置正数上限。提高上限必须记录持久理由；历史材料和过程叙述不能成为扩容理由。

## Alternatives considered

**保留每次开发和验收材料。** 它能复现当时过程，但会形成手工状态目录，引用已经结束的阶段、提交和环境，且与当前合同重复。

**把所有信息集中在根 README。** 单文件便于浏览，但会混合架构、开发、测试、模块细节和历史状态，无法稳定分配所有权。

## Consequences

维护语料更小，读者默认看到当前可验证事实。需要复盘历史时依赖 Git 或仓库外证据，而不是维护一次性文档。删除材料前仍需保留影响当前使用的安全条件、失败模式、所有权和兼容性承诺。
