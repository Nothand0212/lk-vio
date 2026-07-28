# Domain docs

本文规定工程技能在探索代码库时如何读取本仓库的域文档。

## 探索前读取

- 仓库根目录的 `CONTEXT.md`。
- 如果根目录存在 `CONTEXT-MAP.md`，则按其指向读取与当前主题相关的各个 `CONTEXT.md`。
- 读取 `docs/adr/` 中与当前工作区域相关的 ADR。
- 在 multi-context 仓库中，还需检查 `src/<context>/docs/adr/` 中的 context-scoped ADR。

如果这些文件不存在，继续执行即可，不报告其缺失，也不预先建议创建。`/domain-modeling` 技能会在术语或决策真正明确时按需创建这些文件。

## 文件结构

本仓库采用 single-context 布局：

```text
/
├── CONTEXT.md
├── docs/adr/
│   ├── 0001-example-decision.md
│   └── 0002-another-decision.md
└── src/
```

如果以后迁移为 multi-context 布局，则根目录使用 `CONTEXT-MAP.md`，系统级 ADR 存放在 `docs/adr/`，各 context 的 `CONTEXT.md` 和 ADR 存放在对应源码目录下。

## 使用 glossary 中的词汇

当输出中命名域概念时，例如 Issue 标题、重构建议、假设或测试名称，应使用 `CONTEXT.md` 中定义的术语，不要改用 glossary 明确排除的同义词。

如果所需概念尚未出现在 glossary 中，应先判断是否正在引入项目未使用的语言；若确有缺口，则记录下来供 `/domain-modeling` 处理。

## 标明 ADR 冲突

如果输出与现有 ADR 冲突，应明确指出，不能静默覆盖。例如：

> 与 ADR-0007 冲突，但值得重新讨论，因为……
