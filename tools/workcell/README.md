# 确定性工作台生成

用头部 RGB、相机内参、打印板 A 规格和尺量数据生成工作台。
工具只调用 RP-RoboReal2Sim 的检测与 `solve_head_pnp`，不调用 LLM、
不启动 Blender、不访问设备。旧照片重建流程继续保留。

## 安装

在独立 Python 3.10+ 环境中安装两个仓库。以下命令在本仓库根目录执行：

```bash
python -m pip install --upgrade pip
python -m pip install -e ../RP-RoboReal2Sim
python -m pip install -e "./workflow[workcell]"
```

`../RP-RoboReal2Sim` 是已获访问权限的机器人仓库克隆；该仓库若需鉴权，
先用自己的 GitHub 权限克隆，不需要把访问令牌写入配置或场景包。

依赖包括 `numpy`、OpenCV contrib、`usd-core`，以及机器人包依赖的 SciPy。
安装时只保留一种提供 `cv2` 的 OpenCV 包；需要 contrib 中的 ArUco。

## 准备新工作台

按步骤带图的做法（工位俯视/侧视图、头部拍照要求、量哪些尺寸、叠加图怎么看）见
[RP-Real2Sim2Real 实施手册第 5 节](https://github.com/Roboparty/RP-Real2Sim2Real/blob/main/docs/new-user-quickstart.md#5-工作台场景rp-scenereal2sim)。
照片里只需要板 A，不需要另贴二维码。

1. 在桌面固定打印板 A，实测格长。板规格沿用
   [RP-RoboReal2Sim targets 配置](https://github.com/Roboparty/RP-RoboReal2Sim/blob/main/configs/piper_dual.yaml)，
   支持完整配置外层 `targets`，或直接填写 targets 映射。
2. 保存一张或多张头部 RGB。每张都需要能检测板 A；不读取深度。
3. 复制本目录的三个 `*.template.json` 到自己的输入目录，重命名为
   `measurements.json`、`targets.json`、`intrinsics.json`，填写实测值。
   支持 UTF-8 JSON/YAML，所有路径相对于 measurements 文件。
4. 填写 RGB 的原生 `K`、OpenCV 畸变系数 `D` 和
   `image_size: [width,height]`。分辨率不符直接报错，不静默缩放内参。
5. 填写桌面四角 `top_corners_board_m`、厚度、每个物体的尺寸和底面中心。
   模板数字只是示例；工具不会从照片猜尺寸或读取 INTACT profile。

板 A 系原点是图案左上角，x 沿列、y 沿行，右手 +z 指向桌内，长度为 m。
纸面和物理桌面均应为 z=0；纸只用 -0.1 mm 的视觉偏移防闪烁，不建纸板碰撞体。
历史配准若有非零桌面高度，必须显式填写并解释，工具不会暗中修正。

世界原点固定在板 A；默认 `T_world_board=diag(1,-1,-1,1)`，使世界 +Z
朝桌外，便于模拟器使用。板 A 原始坐标和变换保存在 `calibration.json`。
`level_table=true` 根据实测桌面法线，把所有几何和相机绕板 A 原点一起转平。
它只改变共同坐标系，不会提高重投影精度。

物体 `base_center_board_m` 是底面中心，尺寸 `[x,y,height]`，高度向板 A 的
-z 生长。`yaw_deg` 绕板 A +z。物体默认是带碰撞的静态尺寸代理体。
可加 `asset: "bell.usda"`：给定带 defaultPrim 的 Z-up USD，工具将资产和
依赖打包成 USDZ，再按实测三轴尺寸缩放、底部对齐，另建尺寸碰撞代理。
资产不联网检索、不估计实际形状、质量或运动学。

## 生成与验收

```bash
rp-workcell --measurements my_inputs/measurements.json --out my_scene
# 未安装 CLI 时，在源码根目录使用：
python tools/workcell/build_scene.py --measurements my_inputs/measurements.json --out my_scene
rp-real2sim2real validate --check-files my_scene/manifest.json
```

输出目录必须不存在。失败时保留当前诊断，不覆盖已有目录或挑选最好帧。
`rp-real2sim2real` 来自
[整合仓库](https://github.com/Roboparty/RP-Real2Sim2Real)，只在最后校验时使用。

| 文件 | 内容 |
|---|---|
| `workcell.usdc` | 桌面视觉／碰撞、零物理厚度的板 A、物体、每张 RGB 的相机 |
| `manifest.json` | scene package，`source.kind=manual`，真实 RGB 相机 observed=true |
| `inputs/` | 本次使用的图像、量测、板规格、内参副本与清单哈希 |
| `detection_*.json`、`calibration.json` | 原始角点、全部 PnP 分支诊断、K／D、坐标变换和 warning |
| `overlay_*.png` | 原图上叠加桌角／边、板 A 和物体轮廓、检测角点 |
| `reprojection.json` | 逐帧板 A 拟合 RMS、投影坐标、可选独立观测误差 |

USD 相机保存针孔 K，畸变 D 留在 JSON；叠加图投影包含 D。
绿色点是板 A 检测，蓝色是板轮廓，橙色是桌边，红色是物体代理轮廓，
紫色是可选人工轮廓。桌角可能落在画面外，仍保留数值，绘图裁切到画幅。

每张图片可添加独立标注：

```json
{
  "path": "head_rgb.png",
  "landmarks": [{"name":"measured_corner", "point_board_m":[0.1,-0.2,0], "pixel":[510,320]}],
  "object_contours_px": {"bell":[[500,300],[550,300],[550,350],[500,350]]}
}
```

`landmarks` 报对应点欧氏误差；物体轮廓报 1 px 采样的双向边界距离。
板 A 角点用于求 PnP，其 RMS 是拟合残差，不能代表独立场景精度。
未提供实拍地标／轮廓的部件只交付投影，不伪造它的误差。
误差超限只记 warning；没有有效板 A、输入不可读或量测无效会报错。
工具不执行机器人任务，也不写真机固件。

## 主流程 scene package

主流程第 8 步的 `stage_worker` 在现有 Blender/USD 导出完成后自动增加
`manifest.json` 和 `package_export_receipt.json`。根入口为 `scene.usdc`。
清单按 `geometry_audit.json` 映射实际 USD prim；相机只有与已接收的
`camera_observations` 位姿匹配时才标为 observed。未导出的观测和未映射部件
保留在 receipt 中，不推定为已导出或已验收。

清单不修改已有 USD、Blender、GLB、几何审计或视觉／物理结论。
可在历史导出目录的**副本**中补清单：

```bash
rp-scene-package --out copied_export --id my_scene --camera-observations accepted_constraints.json
```

`accepted_constraints.json` 使用流程已接收的 `cameras[]` 规范记录，
或 `cameras[].T_world_camera`（OpenCV 光学系）。不提供观测时，所有相机
observed=false，未知绝对尺度写 `none`。不能通过补清单把旧案例升格为新精度结论。

[真实工作台验收与已知偏差](../../examples/piper_rgb_workcell_20261026/README.md)。
