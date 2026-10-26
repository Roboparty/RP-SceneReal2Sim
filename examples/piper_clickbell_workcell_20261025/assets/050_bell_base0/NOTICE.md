# 050_bell / base0（RoboTwin）

click_bell 采集用的铃铛，三个文件和 INTACT-data-generation 采集时校验的完全相同：

| 文件 | 用途 | sha256 |
|---|---|---|
| `model_data0.json` | 尺寸、缩放（0.05）、接触点 | `cee44b84c415189c45f415e92f8708cbfadd5e350b311e06fbf9087f25eaadcb` |
| `visual/base0.glb` | 外观网格 | `c7983e86f99a94f7f07733f8b0712c5465dcf6dae13544522bb67868ee258e93` |
| `collision/base0.glb` | 碰撞网格 | `c53dd199633820484c031ee9328e0de952adacccc44e48af49283ee5e7d29796` |

来源：[RoboTwin](https://github.com/RoboTwin-Platform/RoboTwin) `assets/objects/050_bell`，上游 commit `13c3c47ff4312dd62484bcd51be034af55c062d1`。
RoboTwin 2.0 代码与数据集（[Hugging Face](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0)）以 MIT 许可发布；这里原样再分发，沿用上游条款，不重新授权。

真机铃按实测 85 mm 直径、45 mm 高摆放，INTACT 采集时按此尺寸缩放（`--bell-appearance real85-chrome` 另带金属材质）。
在 INTACT 中使用：

```bash
rpa collect ... --bell-source-root examples/piper_clickbell_workcell_20261025/assets/050_bell_base0
```
