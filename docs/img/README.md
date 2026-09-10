# docs/img

> Status: DRAFT  
> Author: Agent  
> Reviewer: TBD  
> Reviewed at: TBD  
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

本目录用于存放 `docs/` 下所有 Markdown 文档使用的图片。

## 约定

- 文件名使用小写字母、数字和下划线，例如：

```text
frame_tree.svg
force_control_loop.svg
whole_body_planner_pipeline.svg
```

- 优先使用 SVG，适合 Typora / GitHub / VS Code 阅读。
- 图片引用时必须使用相对路径：

```markdown
![坐标系树](img/frame_tree.svg)
```

- 不要在 Markdown 中使用绝对路径。
- 不要使用带空格的路径。
