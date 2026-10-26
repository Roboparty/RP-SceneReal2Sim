# Piper 双臂 click_bell 工作台

真实工作台的仿真场景，INTACT-data-generation 的 click_bell 数采现在用的就是这个场景。这里把它单独做成 scene package，交给数采端使用，详见 [RP-Real2Sim2Real](https://github.com/Roboparty/RP-Real2Sim2Real)。

这是“从 INTACT 配准导入”的特例，输入已经含旧头部外参和转平矩阵。
新用户使用[确定性工作台工具](../../tools/workcell/README.md)，从自己的头部 RGB、
板 A 规格和实测量重新求 PnP；[真实 RGB 验收](../piper_rgb_workcell_20261026/README.md)
单独报告新旧场景偏差。本目录产物和历史验收结论保留。

## 里面有什么

| 内容 | 说明 |
|---|---|
| 桌面 | 配准的 1.8 m×0.805 m 桌面，30 mm 厚，视觉和碰撞分开 |
| 板 A | 打印 ChArUco 板，0.252 m×0.18 m，纸贴在桌面上，物理高度 0，不单独建碰撞 |
| 桌面贴图 | 按配准的四角贴平到桌面，用去掉铃印的纹理 |
| 铃 | 真机原位（板 A 系 (0.037, -0.142) m），场景里放 85×85×45 mm 碰撞代理体；RoboTwin `050_bell/base0` 原始三个文件（外观、碰撞、尺寸配置）在 [assets/050_bell_base0](assets/050_bell_base0/NOTICE.md)，随场景包一起发布，INTACT 用 `--bell-source-root` 直接读取 |
| 头部相机 | authored 外参，随整台转平 |

机器人、腕部相机、相机支架不在场景包里，在 robot package 里。板 B 真机已经撤掉，这里也不放。

## 怎么来的

1、世界系用板 A 定义，打印图案左上角是原点，x 沿列、y 沿行，+z 指向桌内；
2、头部相机对板 A 做 PnP，重投影 0.66 px 左右，桌面、铃、桌面贴图按实测放进板 A 系；
3、`to_nominal` 把板 A 系变到 PhysX 原生世界；
4、`level_matrix` 把整个工作台统一转平。配准带了 0.80° 桌面倾斜，真机桌面是平的，所以整台刚体转平，相对位姿不变，只改重力方向。

输入都在 `inputs/`，取自 INTACT-data-generation `profiles/piper_real_aligned_20261022`（commit `8260dead`），哈希记录在 `inputs/registration.json`，构建时会逐个核对。

## 生成

```bash
python build_scene.py --out scene    # 需要 numpy、usd-core；输出目录必须不存在
rp-real2sim2real validate --check-files scene/manifest.json
```

`scene/` 里是生成好的 `workcell.usda`、纹理和 `manifest.json`。这份清单是 2026-10-25 生成的，已被 RP-Real2Sim2Real 的运行记录引用哈希，所以 `producer.repo` 保留仓库旧名；重新生成会写新名。

## 和 INTACT 现行场景对过

在空 stage 上跑 INTACT 的 `real_scene.apply_scene`（加载 unified r5 转平），和这里生成的场景逐点比较世界坐标：

| 部件 | 最大差 |
|---|---|
| 桌面（视觉/碰撞） | 0.000055 mm |
| 板 A | 0.000011 mm |
| 板 A 纸边 | 0.000012 mm |
| 桌面贴图 | 0.000007 mm |

转平后桌面法线和 +Z 的夹角为 0°。

## 踩过的坑

1、板 A 和桌面配准差了 8 mm，最初被建成一个 8 mm 台阶，铃放不上去。真机板 A 就是一张纸，现在改成平贴；
2、配准变换带来 0.80° 桌面倾斜（桌边差 25 mm），杂物在斜桌面上落定判不稳，后来整台转平；
3、旧的纹理 DomeLight 会让 Isaac RGB 全黑，要换成中性 DomeLight；
4、桌面贴图原纹理上有铃的印子，换成去掉铃印的纹理。

## 接下来

1、补白天光照层，和现有夜间层并列；
2、头部相机畸变现在放在标定 JSON 里，USD 存 pinhole，渲染端按 JSON 去畸变。
