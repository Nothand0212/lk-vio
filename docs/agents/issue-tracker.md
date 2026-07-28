# Issue tracker：GitHub

本仓库的 Issue 和 PRD 存放在 GitHub Issues 中。所有操作使用 `gh` CLI。

## 约定

- 创建 Issue：`gh issue create --title "..." --body "..."`。多行正文使用 heredoc。
- 读取 Issue：`gh issue view <number> --comments`，同时获取标签；需要时使用 `jq` 过滤评论。
- 列出 Issue：`gh issue list --state open --json number,title,body,labels,comments --jq '[.[] | {number, title, body, labels: [.labels[].name], comments: [.comments[].body]}]'`，并根据需要添加 `--label` 和 `--state` 过滤条件。
- 评论 Issue：`gh issue comment <number> --body "..."`
- 添加或移除标签：`gh issue edit <number> --add-label "..."` / `--remove-label "..."`
- 关闭 Issue：`gh issue close <number> --comment "..."`

在仓库 clone 内运行时，由 `git remote -v` 推断仓库；`gh` 会自动完成此操作。

## 将 Pull Request 作为 triage 请求入口

**PRs as a request surface: no.**

如设为 `yes`，PR 将使用与 Issue 相同的标签和状态，并使用对应的 `gh pr` 命令：

- 读取 PR：`gh pr view <number> --comments`；使用 `gh pr diff <number>` 读取差异。
- 列出待 triage 的外部 PR：`gh pr list --state open --json number,title,body,labels,author,authorAssociation,comments`，仅保留 `authorAssociation` 为 `CONTRIBUTOR`、`FIRST_TIME_CONTRIBUTOR` 或 `NONE` 的 PR，排除 `OWNER`、`MEMBER` 和 `COLLABORATOR`。
- 评论、标记或关闭：使用 `gh pr comment`、`gh pr edit --add-label` / `--remove-label` 和 `gh pr close`。

GitHub 的 Issue 和 PR 共用编号空间，因此 `#42` 可能是其中任意一种。先执行 `gh pr view 42`；若失败，再执行 `gh issue view 42`。

## 当技能要求“发布到 issue tracker”时

创建一个 GitHub Issue。

## 当技能要求“获取相关 ticket”时

执行 `gh issue view <number> --comments`。

## Wayfinding 操作

供 `/wayfinder` 使用。Map 是单个 Issue，child ticket 是其子 Issue。

- **Map**：带有 `wayfinder:map` 标签的单个 Issue，其正文保存 Notes、Decisions-so-far 和 Fog。使用 `gh issue create --label wayfinder:map` 创建。
- **Child ticket**：通过 sub-issues API 关联至 Map 的 Issue。如果仓库未启用 sub-issues，则在 Map 正文的任务列表中加入 child，并在 child 正文顶部写入 `Part of #<map>`。标签为 `wayfinder:<type>`，其中类型为 `research`、`prototype`、`grilling` 或 `task`。Ticket 被认领后，分配给负责实现的开发者。
- **Blocking**：优先使用 GitHub 原生 Issue dependencies。使用 `gh api --method POST repos/<owner>/<repo>/issues/<child>/dependencies/blocked_by -F issue_id=<blocker-db-id>` 添加依赖边；`<blocker-db-id>` 必须是 blocker 的数字 database ID，可通过 `gh api repos/<owner>/<repo>/issues/<n> --jq .id` 获取，不能使用 Issue 编号或 `node_id`。若 dependencies 不可用，则在 child 正文顶部添加 `Blocked by: #<n>, #<n>`。所有 blocker 关闭后，ticket 才解除阻塞。
- **Frontier query**：按 Map 的 sub-issues 或任务列表范围列出未关闭 child，排除仍有 open blocker 或已有 assignee 的项，按 Map 中的顺序选择第一项。
- **Claim**：执行 `gh issue edit <n> --add-assignee @me`；这是会话中的第一次写操作。
- **Resolve**：执行 `gh issue comment <n> --body "<answer>"`，然后执行 `gh issue close <n>`，最后在 Map 的 Decisions-so-far 中追加 context pointer（gist 与链接）。
