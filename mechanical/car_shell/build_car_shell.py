#!/usr/bin/env python3
"""陆空两栖机器人小车外壳生成脚本（Blender 5.2 无头运行）。

设计目标（对应任务书）：
- 一体式主壳体 `main_shell`：除后部滑盖外的整个外壳为一个连续零件，不做上下壳
  分体；内部中空，容纳电机、电池、Jetson Orin Nano 和线材。
- 独立后部滑盖 `rear_sliding_cover`：内嵌于后部设备舱顶部，与车顶齐平，沿前后
  方向滑动，可从车尾完全抽出。
- 前轮左右各一个驱动轮，采用半包覆式轮罩（包覆上半部分与部分前侧），轮舱同时
  容纳整个车轮和电机，预留 5 mm 转动间隙。
- 电机固定螺丝孔：每侧两个，孔距 17 mm，安装面距轮子外缘 32 mm。
- 后部底部预留从动轮安装孔位。

坐标约定：+X 向前，+Y 向右，Z=0 为地面。单位 mm。

尺寸推导（关键约束与取舍，详见 README）：
- 宽度 210：单侧“车轮+电机”95 mm 由外缘向内占用，两侧共 190 mm，若车身只有
  190 mm 则两电机在中心直接相碰、无任何中间结构，因此取 210 mm 留出中央隔墙。
- 高度 72：轮舱顶面在 z=70，顶盖板需在其上；同时设备舱要能容纳叠加放置的
  电池（27）与 Jetson（35），故设备舱净高取 65.5 mm。
- 长度 305：前端轮舱 + 150 mm 电池 + 后部从动轮安装块。

工程实现要点：
1. 所有挖除体必须逐个单独布尔求差。把多个互相重叠的挖除体合并成一个网格再一次
   性求差会让 EXACT 求解器失败（实测会把主体切成碎片或退化成薄板）。
2. 尺寸统计一律使用网格顶点实际坐标，不使用 object.bound_box —— 后者在布尔
   修改器应用后仍是过期缓存，读数不可信。
3. 电机占位长度、车轮宽度、Jetson 外形均为推定值，代码中以注释标出，需按实物复核。

运行方式（开发机需要临时提供 libjsoncpp.so.27）：
    LD_LIBRARY_PATH=~/.local/blender-libs/usr/lib \
        blender --background --factory-startup --python build_car_shell.py
"""

from __future__ import annotations

import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

# =============================================================================
# 一、参数
# =============================================================================

# ---- 车轮与电机 ----
# 任务书给定：前轮直径 65 mm；单侧车轮+电机总长 95 mm。
WHEEL_R = 32.5                    # 前轮半径
WHEEL_WIDTH = 25.0                # 车轮轴向宽度（推定值）
WHEEL_CLEAR = 5.0                 # 轮子与轮罩/壳体间隙（要求 4~5 mm）
WELL_R = WHEEL_R + WHEEL_CLEAR    # 轮舱半径 37.5 mm

# 任务书给定：电机两螺丝孔中心距 17 mm，安装面距轮子外缘 32 mm。
MOUNT_HOLE_SPACING = 17.0         # 电机两个固定螺丝孔中心距
MOUNT_FROM_WHEEL_EDGE = 32.0      # 电机安装面距轮子外缘
MOUNT_PILOT_R = 1.35              # M3 自攻螺丝底孔半径（Ø2.7）
SHAFT_HOLE_R = 6.0                # 隔板中心让轴孔半径（推定值，待电机确认）
# 推定值：36 mm 减速电机，长度按“总长 95 mm”约束反推。
MOTOR_DIA = 36.0
MOTOR_LEN = 59.0
BULKHEAD_T = 4.0                  # 电机安装隔板厚度

# ---- 电池与主控 ----
BATTERY = (150.0, 43.0, 27.0)     # 电池 长(前后) × 宽(左右) × 高（任务书给定）
JETSON = (105.0, 85.0, 35.0)      # Jetson Orin Nano 占位（含散热高度，待实测确认）

# ---- 主壳体总体 ----
BODY_L = 305.0                    # 总长
BODY_W = 210.0                    # 总宽
BODY_H = 74.0                     # 车体自身高度（不含离地间隙）
# 车体底面离地高度由后部从动轮决定：从动轮“固定底座到地面”为 33 mm，底座贴在
# 车体底面上，所以车底必须正好在 33 mm 高处，从动轮才能触地且车体保持水平。
# 该值同时让前轮恰好露出一半（前轮半径 32.5 mm，车底 33 mm ≈ 半轴高度），
# 满足“包覆上半部分、下半部分露出”的半包覆轮罩要求。
Z_BOTTOM = 33.0                   # 车体底面离地高度
Z_TOP = Z_BOTTOM + BODY_H         # 车顶高度 105 mm
# 设备舱顶面之上的顶盖板厚度。滑槽底面比舱内顶面高出的部分即为顶板净厚：
#   顶板净厚 = TOP_PLATE_T - COVER_T - COVER_BOTTOM_GAP = 5.5 - 2.0 - 0.2 = 3.3 mm
# 满足任务书提到的滑轨/导向结构局部加厚到 3~3.5 mm 的要求。
TOP_PLATE_T = 5.5
X_REAR = -BODY_L / 2.0            # 车尾 -152.5
X_FRONT = BODY_L / 2.0            # 车头
Y_MAX = BODY_W / 2.0              # 外侧平面
WALL = 2.5                        # 建议壁厚（任务书 2.5 mm）
FILLET_V = 8.0                    # 竖直外棱圆角
FILLET_TOP = 4.0                  # 顶部外棱圆角

# ---- 前轮与电机布置 ----
WHEEL_CX = 100.0                            # 轮心前后位置
WHEEL_Y_OUT = Y_MAX                          # 轮子外缘（与车体侧面齐平）
WHEEL_Y_IN = WHEEL_Y_OUT - WHEEL_WIDTH       # 轮子内侧面 80 mm
WHEEL_CY = (WHEEL_Y_OUT + WHEEL_Y_IN) / 2.0  # 轮心横向位置 92.5 mm
MOUNT_Y = WHEEL_Y_OUT - MOUNT_FROM_WHEEL_EDGE  # 电机安装面 73 mm
BULKHEAD_Y_IN = MOUNT_Y - BULKHEAD_T          # 隔板内侧面 69 mm
MOTOR_Y_IN = WHEEL_Y_OUT - 95.0               # 电机内端 10 mm（总长 95 mm 约束）
WELL_Y_IN = MOTOR_Y_IN - 2.0                   # 轮舱内端 8 mm（电机装入余量 2 mm）
BULKHEAD_R = WELL_R + 1.0                      # 隔板半径（略大于轮舱以便融合）
# 电机与车轮同轴，电机固定螺丝孔位于半轴高度（32.5 mm），而车底在 33 mm 高处，
# 即螺丝孔比车底低 0.5 mm，因此隔板必须向下延伸一点才能给螺丝孔留出材料。
# 这一段是轮罩内侧壁（同时也是电机安装板），会略低于车底，是同轴电机布局
# 在“车底高度由从动轮决定”条件下的必然结果，伸出量按够用即可取值。
BULKHEAD_Z_MIN = WHEEL_R - 3.0                 # 隔板下缘 29.5 mm（低于车底 3.5 mm）

# ---- 后部设备舱 ----
# 舱内后壁一直延伸到车尾内壁（只留 2.5 mm 尾壁），即整个车尾空间都属于舱内。
# 早期版本把舱后壁停在 -100，导致车尾有 52.5 mm 深的整块实心结构，实测占全机
# 46% 的材料却只占 17% 车长，既厚重又浪费；现在改为掏空，车尾只在底板局部加厚出
# 一圈从动轮安装垫。
BAY_X_REAR = X_REAR + WALL                   # 舱内后壁 -150（仅留 2.5 mm 尾壁）
BAY_X_FRONT = 60.0                           # 舱内前壁（与轮舱区之间留 2.5 mm 隔墙）
BAY_Y = Y_MAX - WALL                         # 舱内侧壁 ±102.5
BAY_Z_BOTTOM = Z_BOTTOM + WALL               # 舱内底面 35.5（其下是 2.5 mm 底板）
BAY_Z_TOP = Z_TOP - TOP_PLATE_T              # 舱内顶面（其上是顶盖与滑槽底板）
BAY = (BAY_X_REAR, BAY_X_FRONT, -BAY_Y, BAY_Y, BAY_Z_BOTTOM, BAY_Z_TOP)

# ---- 前部轮舱区（与设备舱由舱前壁隔开，避免轮部泥水进入电子舱）----
FRONT_CAVITY = (62.5, 137.5, -70.0, 70.0, BAY_Z_BOTTOM, Z_TOP - 12.0)

# ---- 后部滑盖 ----
COVER_T = 2.0                     # 滑盖厚度（任务书建议 ≈2 mm）
COVER_SIDE_GAP = 0.5              # 滑盖两侧与滑槽的配合间隙
COVER_BOTTOM_GAP = 0.2            # 滑盖底面与滑槽底面的间隙（避免滑动摩擦）
FINGER_HOLE_R = 7.0               # 指孔半径（Ø14）

# ---- 顶部维护开口与滑槽 ----
# 开口只覆盖设备区（电池与 Jetson 的安装范围），后端固定在这里，
# 使车尾保持完整的上盖结构（车尾现在是掏空空腔＋底板安装垫）。
OPENING = (-100.0, 57.0, -45.0, 45.0, BAY_Z_BOTTOM, Z_TOP + 1.0)
# 滑槽：底面前后贯通到车尾之外，滑盖可沿 -X 方向整体推出车体；
# 盖顶面与车顶齐平，底面留 0.2 mm 间隙。
GROOVE_HALF_W = 53.0
GROOVE_Z0 = Z_TOP - COVER_T - COVER_BOTTOM_GAP
GROOVE_Z1 = Z_TOP + 0.5
GROOVE_X0 = X_REAR - 1.5
GROOVE_X1 = 63.0
COVER = (X_REAR + 16.5, 60.0, -(GROOVE_HALF_W - COVER_SIDE_GAP),
         GROOVE_HALF_W - COVER_SIDE_GAP, Z_TOP - COVER_T, Z_TOP)

# ---- 后部从动轮（万向脚轮，用户给定尺寸）----
# 给定：带四螺丝孔的底座、可 360° 旋转、四孔组成 23 × 30 mm 矩形、
# 固定底座到地面 33 mm。底座从下方贴到车体底板上，因此车底高度
# 取 33 mm；底座以下（旋转部与轮子）全部位于车底之下，旋转时不会与车体干涉，
# 无需在车底另开让位凹槽。
#
# 位置：从动轮整体前移，避免贴着车尾。旋转包络（CASTER_SWIVEL_R）之后仍留有
# 约 15 mm 车体，见 check() 中的包络距车尾校核。
CASTER_X = -122.0                 # 从动轮/底座中心
CASTER_Y = 0.0
CASTER_HEIGHT = 33.0              # 底座上表面到地面（给定）
CASTER_HOLE_PITCH_X = 23.0        # 四孔孔距：前后方向（给定）
CASTER_HOLE_PITCH_Y = 30.0        # 四孔孔距：左右方向（给定）

# 从动轮安装垫：底座是从下方贴到车体底板上、再向上锁紧的，所以固定面就是底板。
# 底板本身只有 2.5 mm，太薄无法可靠受螺，因此在底板**内侧**局部加厚成一圈安装垫，
# 底面（与底座贴合的 z = 33 平面）保持平整不变。垫子只在腔体底部隆起 5 mm，
# 不高出、不挡手，装配时从下方直接把四颗螺丝拧进即可。
# 早期版本把这里做成从底板贯通到顶盖板的高柱，既挡手又逼着用很长的螺丝从上方
# 上螺母，已废弃。
PAD_X = (-140.0, -104.0)          # 安装垫前后范围（前后各留 6.5 mm 螺孔边距）
PAD_HALF_Y = 23.0                 # 安装垫左右半宽（底座半宽 21，两侧各留 2 mm）
PAD_T = 5.0                       # 安装垫高出舱内底面的厚度
# 四个安装孔为上下贯通孔（M3 间隙）：从底板底面一直穿到安装垫顶面，
# 受螺总厚度 = 底板 2.5 + 安装垫 5.0 = 7.5 mm，可用短螺丝直接拧。
# 安装垫顶面位于舱内空腔中，螺母可从滑盖侧的开口伸手放入（或用热熔铜螺母）。
CASTER_HOLE_R = 1.7               # 通孔半径（Ø3.4，M3 间隙）
# 四个安装孔做成上下贯通的通孔（M3 间隙孔），便于用螺丝+螺母直接锁紧，
# 而不是在厚壁上攻自攻牙；配合底板内侧的安装垫，M3 螺丝可直接可靠锁紧。
CASTER_HOLE_R = 1.7               # 通孔半径（Ø3.4，M3 间隙）
CASTER_PLATE_X = 34.0             # 底座外形 X（推定，须大于孔距）
CASTER_PLATE_Y = 42.0             # 底座外形 Y（推定，须大于孔距）
CASTER_PLATE_T = 3.0              # 底座板厚（推定）
CASTER_WHEEL_R = 10.0             # 从动轮占位半径（Ø20，推定）
CASTER_WHEEL_W = 12.0             # 从动轮占位宽度（推定）
CASTER_TRAIL = 5.0                # 轮心相对旋转轴的偏移（推定，脚轮拖尾）
CASTER_SWIVEL_R = 15.0            # 旋转部包络半径（= 拖尾 + 轮半径，校核 360° 空间）

# ---- 内部安装特征 ----
# 电池限位：前后各一条挡边（从动轮安装垫只在底部隆起，不能充当电池后限位）。
# 电池位置显式给定，不再从舱后壁推导（舱后壁已延伸到车尾）。
# 挡边与电池端面留装配间隙，保证电池能顺利放入且不会顶死（间隙同时让干涉检查有效）。
BAT_X_REAR = -98.0                        # 电池后端面（其后方为车尾空腔）
BAT_X_FRONT = BAT_X_REAR + BATTERY[0]     # 52 mm，电池正好 150 mm 长
BAT_RIB_CLEAR = 0.4                       # 挡边与电池端面的装配间隙
BAT_RIB_T = 3.0                           # 挡边厚度
BAT_RIB_HALF_W = 25.0                     # 挡边外缘（电池半宽 21.5，两侧各留 3.5）
BAT_RIB_H = 20.0                          # 挡边高度
BAT_PLACE_LIFT = 0.3                      # 占位电池抬离舱底的高度（避免共面接触）
# Jetson 支撑导轨：内侧面兼作电池左右限位，顶面通过铜柱承载 Jetson。
RAIL_X = (-95.0, 0.0)
RAIL_Y = (24.0, 40.0)
RAIL_H = 28.0                             # 导轨高 28 mm，顶面 z = 63.5
JETSON_STANDOFF = 0.5                     # 占位 Jetson 与导轨顶面的间隙（实物为铜柱）
# Jetson 固定底孔（名义 85 × 70 孔位，需按实际载板确认）
JETSON_HOLES = ((-90.0, -35.0), (-5.0, -35.0), (-90.0, 35.0), (-5.0, 35.0))
JETSON_HOLE_R = 1.25              # M3 自攻底孔半径
# 舱内走线压线柱：布置在电池两侧的走线通道内（电池半宽 21.5、导轨外缘 40，
# 因此 y=±47~52 是沿车身纵向贯通的空闲通道），不能落在电池占位体积里。
WIRE_HOOK_XS = (-70.0, -40.0)
WIRE_HOOK_Y0, WIRE_HOOK_Y1 = 47.0, 52.0
HOOK_SIZE = (6.0, 5.0, 6.0)
# 舱前壁走线过孔（高度取舱底之上 22 mm，位于电池顶面与导轨之间的走线区）
WIRE_PASS = ((BAY_Z_BOTTOM + 22.0, 45.0), (BAY_Z_BOTTOM + 22.0, -45.0))     # (z, y)
WIRE_PASS_R = 7.0
# 侧壁散热槽（Jetson 基本散热；高度对齐 Jetson 所在区间）
VENT_X = (-95.0, -65.0)
VENT_Z = (BAY_Z_BOTTOM + 30.0, BAY_Z_BOTTOM + 36.0, BAY_Z_BOTTOM + 42.0)
VENT_H = 3.0

SEG = 64                          # 圆柱/圆弧分段数


# =============================================================================
# 二、几何辅助函数
# =============================================================================

def box(bm: bmesh.types.BMesh, x0, x1, y0, y1, z0, z1) -> None:
    """向 bmesh 追加一个轴对齐长方体（由两个角点定义）。"""

    size = Vector((x1 - x0, y1 - y0, z1 - z0))
    center = Vector(((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0))
    matrix = Matrix.Translation(center) @ Matrix.Diagonal(size.to_4d())
    bmesh.ops.create_cube(bm, size=1.0, matrix=matrix)


def cylinder(bm: bmesh.types.BMesh, radius: float, depth: float, center, axis: str = "Z",
             segments: int = SEG) -> None:
    """向 bmesh 追加一个圆柱；axis 指定轴向。"""

    matrix = Matrix.Translation(Vector(center))
    if axis == "X":
        matrix = matrix @ Matrix.Rotation(math.radians(90.0), 4, "Y")
    elif axis == "Y":
        matrix = matrix @ Matrix.Rotation(math.radians(90.0), 4, "X")
    bmesh.ops.create_cone(
        bm, cap_ends=True, cap_tris=False, segments=segments,
        radius1=radius, radius2=radius, depth=depth, matrix=matrix,
    )


def d_prism(bm: bmesh.types.BMesh, xc: float, zc: float, radius: float, z_min: float,
            y0: float, y1: float, segments: int = SEG) -> None:
    """追加一个“圆被 z>=z_min 裁剪后沿 Y 拉伸”的 D 形棱柱。

    用作电机安装隔板：隔板理论上是圆盘，但圆盘下半部分会超出车体底面，
    直接并集会在车底形成多余凸起，因此先在 X-Z 平面裁剪轮廓再拉伸。
    """

    if radius <= (zc - z_min) + 1e-9:
        angles = [2.0 * math.pi * i / segments for i in range(segments)]
    else:
        dx = math.sqrt(max(radius * radius - (zc - z_min) ** 2, 0.0))
        a0 = math.atan2(z_min - zc, -dx)      # 左交点
        if a0 < 0.0:
            a0 += 2.0 * math.pi
        a1 = math.atan2(z_min - zc, dx)       # 右交点
        # 从 a0 递减到 a1，即经过圆顶（90°）的那一段弧。
        angles = [a0 + (a1 - a0) * i / (segments - 1) for i in range(segments)]

    verts = [
        bm.verts.new((xc + radius * math.cos(a), y0, zc + radius * math.sin(a)))
        for a in angles
    ]
    face = bm.faces.new(verts)
    result = bmesh.ops.extrude_face_region(bm, geom=[face])
    moved = [e for e in result["geom"] if isinstance(e, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, verts=moved, vec=(0.0, y1 - y0, 0.0))


def new_object(name: str, bm: bmesh.types.BMesh, shade_smooth: bool = False) -> bpy.types.Object:
    """把 bmesh 转成场景对象，并统一重算法线方向。"""

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    if shade_smooth:
        for poly in obj.data.polygons:
            poly.use_smooth = True
    return obj


def bevel_edges(bm: bmesh.types.BMesh, predicate, offset: float, segments: int) -> None:
    """按条件筛选边并倒角。"""

    edges = [edge for edge in bm.edges if predicate(edge)]
    if not edges:
        return
    bmesh.ops.bevel(
        bm, geom=edges, offset=offset, offset_type="OFFSET",
        segments=segments, profile=0.5, affect="EDGES", clamp_overlap=True,
    )


def mesh_is_empty(obj: bpy.types.Object) -> bool:
    """判断网格是否已经被布尔运算破坏成空网格。"""

    return len(obj.data.polygons) == 0


def apply_boolean(target: bpy.types.Object, name: str, bm: bmesh.types.BMesh,
                  operation: str) -> None:
    """用 bmesh 构造工具对象并对 target 应用一次布尔运算，并校验结果。"""

    if not bm.faces:
        bm.free()
        return
    tool = new_object(name, bm)
    modifier = target.modifiers.new(name=f"bool_{name}", type="BOOLEAN")
    modifier.operation = operation
    modifier.object = tool
    modifier.solver = "EXACT"
    bpy.ops.object.select_all(action="DESELECT")
    target.select_set(True)
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=modifier.name)
    mesh = tool.data
    bpy.data.objects.remove(tool, do_unlink=True)
    bpy.data.meshes.remove(mesh)

    # 任何一步布尔把主体清空都说明几何构造有问题，立即报错而不是继续生成错模型。
    if mesh_is_empty(target):
        raise RuntimeError(f"布尔运算 '{name}'（{operation}）把主壳体破坏成空网格")


def cut_many(target: bpy.types.Object, cuts: list[tuple[str, bmesh.types.BMesh]]) -> None:
    """按顺序逐个执行求差（必须逐个执行，原因见模块文档字符串）。"""

    for name, bm in cuts:
        apply_boolean(target, name, bm, "DIFFERENCE")


def set_material(obj: bpy.types.Object, color, roughness: float = 0.5) -> None:
    """给对象设置一个简单 Principled 材质，便于预览渲染区分零件。"""

    material = bpy.data.materials.new(name=f"{obj.name}_mat")
    material.use_nodes = True
    node = material.node_tree.nodes.get("Principled BSDF")
    if node is not None:
        node.inputs["Base Color"].default_value = (*color, 1.0)
        node.inputs["Roughness"].default_value = roughness
    obj.data.materials.append(material)


# =============================================================================
# 三、主壳体
# =============================================================================

def shell_outline() -> bpy.types.Object:
    """生成圆角长方体外形（尚未开腔）。"""

    bm = bmesh.new()
    box(bm, X_REAR, X_FRONT, -Y_MAX, Y_MAX, Z_BOTTOM, Z_TOP)
    # 竖直棱圆角
    bevel_edges(
        bm,
        lambda e: abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-6
        and abs(e.verts[0].co.x - e.verts[1].co.x) < 1e-6
        and abs(e.verts[0].co.y - e.verts[1].co.y) < 1e-6,
        FILLET_V, 8,
    )
    # 顶部棱圆角
    bevel_edges(bm, lambda e: all(abs(v.co.z - Z_TOP) < 1e-6 for v in e.verts), FILLET_TOP, 5)
    return new_object("main_shell", bm)


def shell_cavities() -> list[tuple[str, bmesh.types.BMesh]]:
    """构造所有主腔体与开口的挖除体。"""

    cuts: list[tuple[str, bmesh.types.BMesh]] = []

    # 后部设备舱（容纳电池、Jetson、线材）
    bm = bmesh.new()
    box(bm, *BAY)
    cuts.append(("cut_bay", bm))

    # 前部轮舱区：连接左右轮舱，形成连续内部空间，便于走线与整体打印
    bm = bmesh.new()
    box(bm, *FRONT_CAVITY)
    cuts.append(("cut_front_cavity", bm))

    # 左右轮舱：半径 = 轮半径 + 5 mm 间隙，沿 Y 贯通到车体侧面之外
    for sign in (1.0, -1.0):
        bm = bmesh.new()
        y0, y1 = sorted((sign * WELL_Y_IN, sign * (Y_MAX + 2.0)))
        cylinder(bm, WELL_R, y1 - y0, (WHEEL_CX, (y0 + y1) / 2.0, WHEEL_R), "Y")
        cuts.append((f"cut_well_{int(sign):+d}", bm))

    # 顶部维护开口
    bm = bmesh.new()
    box(bm, *OPENING)
    cuts.append(("cut_opening", bm))

    # 滑槽：整条横向贯通到车尾之外，与顶部开口在高度上重叠。
    # 注意必须做成一条完整通道、且与开口区域重叠，不能拆成左右两条与开口
    # 正好共面（y=±45），否则布尔运算会产生退化边（非流形边）。
    bm = bmesh.new()
    box(bm, GROOVE_X0, GROOVE_X1, -GROOVE_HALF_W, GROOVE_HALF_W, GROOVE_Z0, GROOVE_Z1)
    cuts.append(("cut_groove", bm))

    # 舱前壁走线过孔：把设备舱与轮舱区连通
    for index, (pz, py) in enumerate(WIRE_PASS):
        bm = bmesh.new()
        cylinder(bm, WIRE_PASS_R, 60.0, ((BAY_X_FRONT + FRONT_CAVITY[0]) / 2.0, py, pz), "X", 24)
        cuts.append((f"cut_wire_pass_{index}", bm))

    return cuts


def shell_additions() -> bmesh.types.BMesh:
    """构造需要并集进主壳体的内部安装特征。"""

    add = bmesh.new()

    # 电机安装隔板：位于距轮子外缘 32 mm 处，同时作为轮舱与电机腔的分隔壁。
    # 下缘延伸到车底之下（BULKHEAD_Z_MIN），以便在车底以下仍能给同轴电机的
    # 固定螺丝留出材料。
    for sign in (1.0, -1.0):
        y0, y1 = sorted((sign * BULKHEAD_Y_IN, sign * MOUNT_Y))
        d_prism(add, WHEEL_CX, WHEEL_R, BULKHEAD_R, BULKHEAD_Z_MIN, y0, y1)

    # 从动轮安装垫：在底板内侧局部加厚，底面保持平整（z = 33 的贴合面不变），
    # 只在腔体底部隆起 PAD_T。下端埋进底板内 0.5 mm，避免与舱内底面共面。
    box(add, PAD_X[0], PAD_X[1], -PAD_HALF_Y, PAD_HALF_Y,
        BAY_Z_BOTTOM - 0.5, BAY_Z_BOTTOM + PAD_T)

    # 电池限位挡边：前端一条、后端一条（车尾安装垫不能当电池后限位，它只在底部隆起）
    box(add, BAT_X_FRONT + BAT_RIB_CLEAR, BAT_X_FRONT + BAT_RIB_CLEAR + BAT_RIB_T,
        -BAT_RIB_HALF_W, BAT_RIB_HALF_W, BAY_Z_BOTTOM, BAY_Z_BOTTOM + BAT_RIB_H)
    box(add, BAT_X_REAR - BAT_RIB_CLEAR - BAT_RIB_T, BAT_X_REAR - BAT_RIB_CLEAR,
        -BAT_RIB_HALF_W, BAT_RIB_HALF_W, BAY_Z_BOTTOM, BAY_Z_BOTTOM + BAT_RIB_H)

    # Jetson 支撑导轨（兼作电池左右限位）
    for sign in (1.0, -1.0):
        y0, y1 = sorted((sign * RAIL_Y[0], sign * RAIL_Y[1]))
        box(add, RAIL_X[0], RAIL_X[1], y0, y1, BAY_Z_BOTTOM, BAY_Z_BOTTOM + RAIL_H)

    # 舱底压线柱：布置在电池两侧的纵向走线通道内
    for sign in (1.0, -1.0):
        y0, y1 = sorted((sign * WIRE_HOOK_Y0, sign * WIRE_HOOK_Y1))
        for hx in WIRE_HOOK_XS:
            box(add, hx, hx + HOOK_SIZE[0], y0, y1,
                BAY_Z_BOTTOM, BAY_Z_BOTTOM + HOOK_SIZE[2])

    return add


def shell_holes() -> list[tuple[str, bmesh.types.BMesh]]:
    """构造各类功能孔的挖除体（电机螺丝孔、Jetson 底孔、散热槽）。"""

    holes: list[tuple[str, bmesh.types.BMesh]] = []

    for sign in (1.0, -1.0):
        yc = sign * MOUNT_Y
        # 电机轴/联轴器过孔
        bm = bmesh.new()
        cylinder(bm, SHAFT_HOLE_R, 24.0, (WHEEL_CX, yc, WHEEL_R), "Y", 32)
        holes.append((f"cut_shaft_{int(sign):+d}", bm))

        # 电机固定螺丝孔：孔距 17 mm，位于距轮子外缘 32 mm 的安装面上
        for index, dx in enumerate((-MOUNT_HOLE_SPACING / 2.0, MOUNT_HOLE_SPACING / 2.0)):
            bm = bmesh.new()
            cylinder(bm, MOUNT_PILOT_R, 24.0, (WHEEL_CX + dx, yc, WHEEL_R), "Y", 24)
            holes.append((f"cut_motor_screw_{int(sign):+d}_{index}", bm))

    # Jetson 固定底孔（自导轨顶面向下钻）
    for index, (px, py) in enumerate(JETSON_HOLES):
        bm = bmesh.new()
        cylinder(bm, JETSON_HOLE_R, 20.0,
                 (px, py, BAY_Z_BOTTOM + RAIL_H - 5.0), "Z", 16)
        holes.append((f"cut_jetson_hole_{index}", bm))

    # 侧壁散热槽
    for sign in (1.0, -1.0):
        for index, z0 in enumerate(VENT_Z):
            bm = bmesh.new()
            y0, y1 = sorted((sign * (Y_MAX - WALL - 1.0), sign * (Y_MAX + 1.0)))
            box(bm, VENT_X[0], VENT_X[1], y0, y1, z0, z0 + VENT_H)
            holes.append((f"cut_vent_{int(sign):+d}_{index}", bm))

    # 从动轮底座四个安装孔：必须在安装垫并集之后才能切，否则安装垫会把孔重新填实
    # （安装垫是后加的实体，而 cavities 阶段的切除发生在它之前）。
    # 孔从底板底面（z = Z_BOTTOM）一直贯通到安装垫顶面，贯穿"底板 + 安装垫"。
    # 因为不再有螺母沉孔，孔只到安装垫顶面即可，不需要再往上钻到滑槽底面。
    bolt_z_lo = Z_BOTTOM - 1.0
    bolt_z_hi = BAY_Z_BOTTOM + PAD_T + 1.0
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            hole_x = CASTER_X + sign_x * CASTER_HOLE_PITCH_X / 2.0
            hole_y = CASTER_Y + sign_y * CASTER_HOLE_PITCH_Y / 2.0
            tag = f"x{int(sign_x):+d}y{int(sign_y):+d}"
            bm = bmesh.new()
            cylinder(bm, CASTER_HOLE_R, bolt_z_hi - bolt_z_lo,
                     (hole_x, hole_y, (bolt_z_lo + bolt_z_hi) / 2.0), "Z", 24)
            holes.append((f"cut_caster_through_{tag}", bm))

    return holes


def build_main_shell() -> bpy.types.Object:
    """生成一体式主壳体。"""

    shell = shell_outline()
    cut_many(shell, shell_cavities())
    apply_boolean(shell, "add_internal", shell_additions(), "UNION")
    cut_many(shell, shell_holes())
    set_material(shell, (0.85, 0.86, 0.88), 0.55)
    return shell


# =============================================================================
# 四、后部滑盖
# =============================================================================

def build_rear_cover() -> bpy.types.Object:
    """生成可从车尾完全抽出的后部滑盖。"""

    bm = bmesh.new()
    box(bm, *COVER)
    # 竖直棱倒圆角，避免尖角并便于滑入滑槽
    bevel_edges(
        bm,
        lambda e: abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-6
        and abs(e.verts[0].co.x - e.verts[1].co.x) < 1e-6
        and abs(e.verts[0].co.y - e.verts[1].co.y) < 1e-6,
        4.0, 6,
    )
    cover = new_object("rear_sliding_cover", bm)

    bm = bmesh.new()
    cylinder(bm, FINGER_HOLE_R, 20.0,
             ((COVER[0] + COVER[1]) / 2.0, 0.0, (COVER[4] + COVER[5]) / 2.0), "Z", 32)
    apply_boolean(cover, "cut_finger_hole", bm, "DIFFERENCE")
    set_material(cover, (0.35, 0.42, 0.55), 0.45)
    return cover


# =============================================================================
# 五、占位模型（仅用于干涉与空间校核，不属于最终外壳）
# =============================================================================

def build_placeholders() -> list[bpy.types.Object]:
    """生成车轮、电机、电池、Jetson、从动轮的简单占位模型。"""

    parts: list[bpy.types.Object] = []

    for sign, tag in ((1.0, "R"), (-1.0, "L")):
        # 车轮
        bm = bmesh.new()
        cylinder(bm, WHEEL_R, WHEEL_WIDTH, (WHEEL_CX, WHEEL_CY * sign, WHEEL_R), "Y")
        wheel = new_object(f"ph_wheel_{tag}", bm, shade_smooth=True)
        set_material(wheel, (0.16, 0.16, 0.18), 0.8)
        parts.append(wheel)

        # 电机：内端面位于“车轮外缘向内 95 mm”处
        motor_y = sign * (MOTOR_Y_IN + MOTOR_LEN / 2.0)
        bm = bmesh.new()
        cylinder(bm, MOTOR_DIA / 2.0, MOTOR_LEN, (WHEEL_CX, motor_y, WHEEL_R), "Y", 32)
        motor = new_object(f"ph_motor_{tag}", bm, shade_smooth=True)
        set_material(motor, (0.78, 0.62, 0.2), 0.4)
        parts.append(motor)

    # 电池（离地 0.3 mm 放置，避免与舱底共面接触）
    bz = BAY_Z_BOTTOM + BAT_PLACE_LIFT
    bm = bmesh.new()
    box(bm, BAT_X_REAR, BAT_X_FRONT, -BATTERY[1] / 2.0, BATTERY[1] / 2.0,
        bz, bz + BATTERY[2])
    battery = new_object("ph_battery", bm)
    set_material(battery, (0.2, 0.36, 0.26), 0.6)
    parts.append(battery)

    # Jetson Orin Nano（坐落在导轨顶面的铜柱上，建模留 0.5 mm 间隙）
    jz = BAY_Z_BOTTOM + RAIL_H + JETSON_STANDOFF
    jy0, jy1 = JETSON[1] / -2.0, JETSON[1] / 2.0
    jx0, jx1 = JETSON_HOLES[0][0] - 5.0, JETSON_HOLES[1][0] + 5.0
    bm = bmesh.new()
    box(bm, jx0, jx1, jy0, jy1, jz, jz + JETSON[2])
    jetson = new_object("ph_jetson", bm)
    set_material(jetson, (0.26, 0.32, 0.48), 0.6)
    parts.append(jetson)

    # 后部从动轮：底座板 + 旋转部 + 轮子（占位）。
    # 底座上表面与车底之间留 0.2 mm 间隙，避免共面接触导致干涉检查误报；
    # 轮子下缘正好落在 z = 0，与地面接触。
    bm = bmesh.new()
    plate_top = Z_BOTTOM - 0.2
    box(bm, CASTER_X - CASTER_PLATE_X / 2.0, CASTER_X + CASTER_PLATE_X / 2.0,
        CASTER_Y - CASTER_PLATE_Y / 2.0, CASTER_Y + CASTER_PLATE_Y / 2.0,
        plate_top - CASTER_PLATE_T, plate_top)
    # 旋转部（万向支架）占位
    swivel_top = plate_top - CASTER_PLATE_T
    swivel_bottom = 2.0 * CASTER_WHEEL_R
    cylinder(bm, 7.0, swivel_top - swivel_bottom,
             (CASTER_X, CASTER_Y, (swivel_top + swivel_bottom) / 2.0), "Z", 24)
    # 轮子（相对旋转轴有拖尾偏移，符合脚轮结构）
    cylinder(bm, CASTER_WHEEL_R, CASTER_WHEEL_W,
             (CASTER_X - CASTER_TRAIL, CASTER_Y, CASTER_WHEEL_R), "Y", 32)
    caster = new_object("ph_caster", bm, shade_smooth=False)
    set_material(caster, (0.16, 0.16, 0.18), 0.8)
    parts.append(caster)

    return parts


# =============================================================================
# 六、校核
# =============================================================================

def mesh_bounds(obj: bpy.types.Object) -> tuple[Vector, Vector]:
    """用网格顶点实际坐标计算包围盒；不使用过期的 bound_box。"""

    coords = [v.co for v in obj.data.vertices]
    low = Vector((min(c.x for c in coords), min(c.y for c in coords), min(c.z for c in coords)))
    high = Vector((max(c.x for c in coords), max(c.y for c in coords), max(c.z for c in coords)))
    return low, high


def mesh_stats(obj: bpy.types.Object) -> tuple[int, int, int, int, float]:
    """返回 (顶点, 面, 连通块, 非流形边, 体积cm3)。"""

    bm = bmesh.new()
    bm.from_mesh(obj.data)
    seen: set[int] = set()
    islands = 0
    for vert in bm.verts:
        if vert.index in seen:
            continue
        islands += 1
        stack = [vert]
        seen.add(vert.index)
        while stack:
            current = stack.pop()
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other.index not in seen:
                    seen.add(other.index)
                    stack.append(other)
    non_manifold = sum(1 for e in bm.edges if not e.is_manifold)
    volume = abs(bm.calc_volume(signed=True)) / 1000.0
    stats = (len(bm.verts), len(bm.faces), islands, non_manifold, volume)
    bm.free()
    return stats


def point_is_solid(bvh: BVHTree, point: Vector) -> bool:
    """用射线穿越次数奇偶性判断某点是否位于实体内部。

    从远处沿 +Y 发射射线，只统计“位于采样点之前”的交点个数：奇数表示该点
    在实体内部。注意不能把越过采样点的交点计入，否则会把实心判成空洞。
    """

    count = 0
    origin = Vector((point.x, -1000.0, point.z))
    direction = Vector((0.0, 1.0, 0.0))
    for _ in range(200):
        location = bvh.ray_cast(origin, direction)[0]
        if location is None or location.y >= point.y:
            break
        count += 1
        # 沿射线方向微移，避免在同一点反复命中。
        origin = location + direction * 0.01
    return count % 2 == 1


def verify_solids(shell: bpy.types.Object) -> bool:
    """用点包含测试验证关键内部结构的实际形状。

    这些检查用于确认“设计的结构真的建出来了”，避免出现安装隔板被轮舱切除、
    螺丝孔钻在空气里之类肉眼难以发现的错误。
    """

    ok = True
    bm = bmesh.new()
    bm.from_mesh(shell.data)
    bvh = BVHTree.FromBMesh(bm)

    # (说明, 取样点, 期望是否在实体内)
    # 注意采样点要避开让轴孔（半径 SHAFT_HOLE_R）与螺丝孔本身。
    samples = [
        ("电机安装隔板材料存在", (WHEEL_CX, MOUNT_Y - 2.0, WHEEL_R + 12.5), True),
        ("两螺丝孔之间为实心", (WHEEL_CX, MOUNT_Y - 2.0, WHEEL_R + 27.5), True),
        ("电机螺丝孔为通孔", (WHEEL_CX + MOUNT_HOLE_SPACING / 2.0, MOUNT_Y - 2.0, WHEEL_R), False),
        ("轮舱内为空洞（轮子转动空间）", (WHEEL_CX, WHEEL_CY, WHEEL_R), False),
        ("电机腔为空洞（电机安装空间）", (WHEEL_CX, MOTOR_Y_IN + MOTOR_LEN / 2.0, WHEEL_R), False),
        ("轮舱上方为实心（半包覆轮罩）", (WHEEL_CX, WHEEL_CY, WELL_R + WHEEL_R + 4.0), True),
        ("设备舱电池空间为空洞", (-50.0, 0.0, BAY_Z_BOTTOM + 12.0), False),
        ("设备舱 Jetson 空间为空洞", (-50.0, 0.0, BAY_Z_TOP - 6.0), False),
        ("Jetson 支撑导轨为实心", (-50.0, RAIL_Y[0] + 2.0, BAY_Z_BOTTOM + RAIL_H - 4.0), True),
        ("车尾尾壁为实心", (X_REAR + 1.2, 0.0, 60.0), True),
        ("从动轮安装垫为实心", (CASTER_X, CASTER_Y, BAY_Z_BOTTOM + PAD_T - 1.0), True),
        ("安装垫上方腔体为空洞（不挡手）",
         (CASTER_X, CASTER_Y, BAY_Z_BOTTOM + PAD_T + 8.0), False),
        ("安装垫左右两侧为空洞", (CASTER_X, 60.0, BAY_Z_BOTTOM + PAD_T - 1.0), False),
        ("安装垫为空洞以外处不隆起（车尾底部只有垫子）",
         (X_REAR + 6.0, 0.0, BAY_Z_BOTTOM + PAD_T - 1.0), False),
        ("车尾底板为实心", (X_REAR + 6.0, 0.0, Z_BOTTOM + 1.2), True),
        ("车尾腔体上部为空洞（掏空后车尾内部为空腔）",
         (X_REAR + 6.0, 0.0, BAY_Z_TOP - 5.0), False),
        ("轮罩内侧壁延伸到车底以下（电机螺丝孔有材料）",
         (WHEEL_CX + 10.0, MOUNT_Y - 2.0, BULKHEAD_Z_MIN + 0.5), True),
        ("隔板中心让轴孔处为空洞", (WHEEL_CX, MOUNT_Y - 2.0, BULKHEAD_Z_MIN + 0.5), False),
        ("从动轮区域车底以下为空洞（脚轮可 360° 旋转）",
         (CASTER_X, CASTER_Y + CASTER_SWIVEL_R - 3.0, Z_BOTTOM - 12.0), False),
        ("车头前壁为实心", (145.0, 0.0, 40.0), True),
        ("滑槽为空洞", (-80.0, 45.0, Z_TOP - 1.0), False),
        ("车顶盖板为实心", (80.0, 0.0, Z_TOP - 2.0), True),
        ("侧壁散热槽为空洞", (-85.0, Y_MAX, VENT_Z[0] + 1.0), False),
    ]

    # 从动轮四个安装孔是上下贯通的通孔，逐个在三个高度上验证：
    # 底板内、底板与安装垫交界处、安装垫上部。孔在安装垫顶面之上不应存在
    # （上面是空腔，孔到此为止）。
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            hx = CASTER_X + sign_x * CASTER_HOLE_PITCH_X / 2.0
            hy = CASTER_Y + sign_y * CASTER_HOLE_PITCH_Y / 2.0
            tag = f"({'右' if sign_y > 0 else '左'}{'前' if sign_x > 0 else '后'}孔)"
            samples.append((f"从动轮通孔{tag}·底板内", (hx, hy, Z_BOTTOM + 1.2), False))
            samples.append((f"从动轮通孔{tag}·垫下部", (hx, hy, BAY_Z_BOTTOM + 1.5), False))
            samples.append((f"从动轮通孔{tag}·垫上部",
                            (hx, hy, BAY_Z_BOTTOM + PAD_T - 1.0), False))
    # 安装垫上表面之外、车尾空腔处不应有材料（验证没有残留的高柱）
    samples.append(("车尾空腔中部为空洞（无高柱残留）", (-122.0, 0.0, 75.0), False))
    samples.append(("电池后方空腔为空洞", (BAT_X_REAR - 12.0, 0.0, BAY_Z_BOTTOM + 12.0), False))

    print("\n---------------- 内部结构实体校验 ----------------")
    for label, coords, expect_solid in samples:
        actual = point_is_solid(bvh, Vector(coords))
        flag = "OK " if actual == expect_solid else "错误"
        if actual != expect_solid:
            ok = False
        want = "实体" if expect_solid else "空洞"
        got = "实体" if actual else "空洞"
        print(f"  [{flag}] {label:<30s} 期望{want} 实测{got}  点={coords}")

    bm.free()
    return ok


def check_interference(shell: bpy.types.Object, cover: bpy.types.Object,
                       parts: list[bpy.types.Object]) -> bool:
    """用 BVH 面片重叠检测各零件之间是否发生干涉。

    相比肉眼看渲染图，这能可靠地发现“轮子其实蹭到轮罩”“滑盖塞不进滑槽”之类
    的装配问题。expect_zero 为 True 的组合要求完全不接触。
    """

    def bvh_of(obj: bpy.types.Object) -> BVHTree:
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        tree = BVHTree.FromBMesh(bm)
        bm.free()
        return tree

    trees = {"main_shell": bvh_of(shell), "rear_sliding_cover": bvh_of(cover)}
    for part in parts:
        trees[part.name] = bvh_of(part)

    ok = True
    print("\n---------------- 装配干涉检查 ----------------")
    # (对象A, 对象B, 说明)：这三组都必须零干涉，否则装配或运动会有问题。
    pairs = [
        ("rear_sliding_cover", "main_shell", "滑盖与壳体（应能在滑槽内自由滑动）"),
        ("ph_wheel_R", "main_shell", "右前轮与壳体（应留转动间隙）"),
        ("ph_wheel_L", "main_shell", "左前轮与壳体（应留转动间隙）"),
        ("ph_battery", "main_shell", "电池与壳体（应能放入且不顶死）"),
        ("ph_jetson", "main_shell", "Jetson 与壳体（应能放入且不顶死）"),
        ("ph_caster", "main_shell", "从动轮与壳体"),
    ]
    for name_a, name_b, label in pairs:
        tree_a, tree_b = trees.get(name_a), trees.get(name_b)
        if tree_a is None or tree_b is None:
            continue
        overlaps = tree_a.overlap(tree_b)
        count = len(overlaps)
        flag = "OK " if count == 0 else "错误"
        if count != 0:
            ok = False
        print(f"  [{flag}] {label:<34s} 重叠面片对={count}")
    return ok


def check(shell: bpy.types.Object, cover: bpy.types.Object) -> bool:
    """打印关键尺寸并校核任务书中的硬性约束。"""

    ok = True
    print("\n================ 尺寸与间隙校核 ================")

    low, high = mesh_bounds(shell)
    size = high - low
    # 车体本体高度为 BODY_H；轮罩内侧壁为给同轴电机螺丝留材料会略低于车底，
    # 因此实际包围盒高度会比 BODY_H 略大，这里分别说明。
    lowest = min(Z_BOTTOM, BULKHEAD_Z_MIN)
    print(f"主壳体包围盒 : {size.x:.1f} × {size.y:.1f} × {size.z:.1f} mm "
          f"(车体 {BODY_L:.0f} × {BODY_W:.0f} × {BODY_H:.0f}，"
          f"含轮罩内侧壁伸出 {Z_BOTTOM - BULKHEAD_Z_MIN:.1f} mm)")
    if abs(size.x - BODY_L) > 1e-6 or abs(size.y - BODY_W) > 1e-6:
        print(f"  [错误] 平面尺寸与设计不符（设计 {BODY_L:.0f} × {BODY_W:.0f}）")
        ok = False
    if abs(size.z - (Z_TOP - lowest)) > 1e-6:
        print(f"  [错误] 总高与预期不符（预期 {Z_TOP - lowest:.1f}）")
        ok = False
    clow, chigh = mesh_bounds(cover)
    csize = chigh - clow
    print(f"滑盖包围盒   : {csize.x:.1f} × {csize.y:.1f} × {csize.z:.1f} mm "
          f"(设计 {COVER[1] - COVER[0]:.0f} × {COVER[3] - COVER[2]:.0f} × {COVER_T:.0f})")

    sv, sf, sislands, snm, svol = mesh_stats(shell)
    cv, cf, cislands, cnm, cvol = mesh_stats(cover)
    print(f"主壳体网格   : 顶点 {sv}, 面 {sf}, 连通块 {sislands}, 非流形边 {snm}, 体积 {svol:.1f} cm³")
    print(f"滑盖网格     : 顶点 {cv}, 面 {cf}, 连通块 {cislands}, 非流形边 {cnm}, 体积 {cvol:.1f} cm³")

    def fail(message: str) -> None:
        nonlocal ok
        print(f"  [错误] {message}")
        ok = False

    def report(label: str, value: float, expect: str) -> None:
        print(f"  {label:<32s} {value:8.2f} mm   (要求 {expect})")

    # 一体式零件（任务书核心要求）
    if sislands != 1:
        fail("主壳体不是单一连通零件，违反了“一体式”要求")

    # 轮子转动间隙
    report("轮子与轮舱单边间隙", WELL_R - WHEEL_R, "4~5")
    if not 4.0 <= WELL_R - WHEEL_R <= 5.0:
        fail("轮子与轮罩间隙不在 4~5 mm 区间")

    # 电机螺丝孔硬性尺寸（任务书给定，必须精确）
    report("电机螺丝孔中心距", MOUNT_HOLE_SPACING, "17")
    report("安装面距轮子外缘", WHEEL_Y_OUT - MOUNT_Y, "32")
    if abs(MOUNT_HOLE_SPACING - 17.0) > 1e-9 or abs((WHEEL_Y_OUT - MOUNT_Y) - 32.0) > 1e-9:
        fail("电机螺丝孔位置不符合给定尺寸")

    # 单侧车轮 + 电机总长
    report("单侧车轮+电机总长", WHEEL_Y_OUT - MOTOR_Y_IN, "95")
    if abs((WHEEL_Y_OUT - MOTOR_Y_IN) - 95.0) > 1e-9:
        fail("单侧车轮+电机总长不是 95 mm")

    # 滑盖配合与可抽出性
    report("滑盖单边侧向间隙", GROOVE_HALF_W - COVER[3], f"≈{COVER_SIDE_GAP}")
    report("滑盖底面与滑槽底面间隙", COVER[4] - GROOVE_Z0, f"≈{COVER_BOTTOM_GAP}")
    report("滑盖厚度", COVER[5] - COVER[4], "≈2")
    report("滑盖与车顶高差", Z_TOP - COVER[5], "0（齐平）")
    if abs((COVER[5] - COVER[4]) - 2.0) > 1e-6 or abs(Z_TOP - COVER[5]) > 1e-6:
        fail("滑盖厚度或齐平关系不满足要求")
    if GROOVE_HALF_W - COVER[3] <= 0.0 or COVER[4] - GROOVE_Z0 <= 0.0:
        fail("滑盖与滑槽之间没有配合间隙，无法滑动")
    # 滑槽后端超出车尾，说明尾端无阻挡，滑盖可沿 -X 整体推出。
    report("滑槽后端超出车尾", X_REAR - GROOVE_X0, ">0（尾端无阻挡，可整体抽出）")
    if GROOVE_X0 > X_REAR:
        fail("滑槽未贯通到车尾，滑盖无法完全抽出")
    report("滑盖总长 / 开口长", (COVER[1] - COVER[0]) - (OPENING[1] - OPENING[0]),
           ">0（完全盖住开口）")
    if (COVER[1] - COVER[0]) < (OPENING[1] - OPENING[0]):
        fail("滑盖短于开口，关闭后无法完全覆盖")
    # 抽出方向无障碍：滑盖向后滑动时不应撞到任何结构，只需滑槽尾端开放。
    report("滑盖可后退行程（至脱离车体）", COVER[1] - COVER[0], "≥0（可整块取出）")

    # 后部从动轮（用户给定的硬性尺寸）
    report("从动轮孔距（前后方向）", CASTER_HOLE_PITCH_X, "=23（给定）")
    report("从动轮孔距（左右方向）", CASTER_HOLE_PITCH_Y, "=30（给定）")
    report("从动轮底座到地面", CASTER_HEIGHT, "=33（给定）")
    if abs(CASTER_HOLE_PITCH_X - 23.0) > 1e-9 or abs(CASTER_HOLE_PITCH_Y - 30.0) > 1e-9:
        fail("从动轮四孔孔距与给定值不符")
    # 车底高度必须等于从动轮底座高度，否则从动轮要么悬空、要么把车顶起来。
    report("车底离地高度", Z_BOTTOM, "=从动轮底座高度 33")
    if abs(Z_BOTTOM - CASTER_HEIGHT) > 1e-9:
        fail("车底高度与从动轮底座高度不一致，从动轮无法同时触地")
    # 从动轮安装垫必须完全覆盖底座，并给螺孔留出足够材料。
    # 底座从下方贴到车体底板，固定面就是底板外表面（z = 33），安装垫只在底板
    # 内侧加厚，不改变贴合面，也不挡手。
    hole_margin_x = min(
        (CASTER_X - CASTER_HOLE_PITCH_X / 2.0) - PAD_X[0],
        PAD_X[1] - (CASTER_X + CASTER_HOLE_PITCH_X / 2.0),
    )
    hole_margin_y = PAD_HALF_Y - CASTER_HOLE_PITCH_Y / 2.0
    report("从动轮螺孔到安装垫边距（前后）", hole_margin_x, ">3（保证螺孔周围有材料）")
    report("从动轮螺孔到安装垫边距（左右）", hole_margin_y, ">3（保证螺孔周围有材料）")
    if min(hole_margin_x, hole_margin_y) < 3.0:
        fail("从动轮螺孔距安装垫边缘过近，材料不足")
    report("安装垫前后尺寸 - 底座深度", (PAD_X[1] - PAD_X[0]) - CASTER_PLATE_X,
           ">0（底座完全贴合安装垫）")
    if (PAD_X[1] - PAD_X[0]) < CASTER_PLATE_X:
        fail("安装垫比从动轮底座还小，底座无法完全贴合")
    report("安装垫左右尺寸 - 底座宽度", 2.0 * PAD_HALF_Y - CASTER_PLATE_Y,
           ">0（底座完全贴合安装垫）")
    if 2.0 * PAD_HALF_Y < CASTER_PLATE_Y:
        fail("安装垫比从动轮底座还窄，底座无法完全贴合")
    # 受力厚度 = 底板 + 安装垫，必须足够让 M3 螺丝可靠受螺/夹紧
    clamp_t = WALL + PAD_T
    report("从动轮固定处受力厚度", clamp_t, "≥6（底板+安装垫，M3 可靠锁紧）")
    if clamp_t < 6.0:
        fail("从动轮固定处太薄，M3 螺丝无法可靠锁紧")
    # 从动轮位置：旋转包络（含拖尾与轮半径）之后应留有足够车体，
    # 避免轮子看起来挂在车尾之外。
    overhang = (CASTER_X - CASTER_SWIVEL_R) - X_REAR
    report("从动轮旋转包络到车尾的距离", overhang, "≥10（不要贴住车尾）")
    if overhang < 10.0:
        fail("从动轮旋转包络过于贴近车尾，轮子会看起来挂在车尾外")
    # 通孔应贯穿到滑槽底面，长度即车尾块净高
    through_len = (BAY_Z_BOTTOM + PAD_T) - Z_BOTTOM
    report("从动轮通孔长度", through_len, "= 底板+安装垫 厚度（通孔）")
    report("从动轮通孔直径", 2.0 * CASTER_HOLE_R, "≥3.2（M3 间隙孔）")
    if 2.0 * CASTER_HOLE_R < 3.2:
        fail("通孔直径小于 M3 间隙孔（Ø3.2）")
    # 螺丝从下方穿过底座与"底板+安装垫"，长度按受力厚度估算。
    bolt_need = through_len + CASTER_PLATE_T + 4.0
    print(f"  {'所需 M3 螺丝长度':<32s} {bolt_need:7.1f} mm   "
          f"(底板+垫 {through_len:.1f} + 底座 {CASTER_PLATE_T:.0f} + 余量 4)")
    print(f"  {'安装方式':<32s} 从下方把底座贴到车体底板，四颗 M3 螺丝向上拧紧")

    # 前轮半包覆：车底高度决定轮子露出比例，应接近一半
    wheel_exposure = Z_BOTTOM / (2.0 * WHEEL_R) * 100.0
    print(f"  {'前轮露出比例':<32s} {wheel_exposure:7.1f} %    (要求 ≈50%（半包覆轮罩）)")
    if not 40.0 <= wheel_exposure <= 60.0:
        fail("前轮露出比例偏离半包覆要求（应在 40%~60%）")

    # 设备舱与装载
    # 电池放在舱底、Jetson 坐落在导轨顶面，两者沿高度叠加，所需净高为
    # 电池高 + 导轨顶面与电池顶面间隙 + Jetson 高。
    stack_need = BATTERY[2] + (RAIL_H - BATTERY[2]) + JETSON[2]
    bay_height = BAY_Z_TOP - BAY_Z_BOTTOM
    report("电池占位长度（应与实物一致）", BAT_X_FRONT - BAT_X_REAR, "=150")
    if abs((BAT_X_FRONT - BAT_X_REAR) - BATTERY[0]) > 1e-9:
        fail("电池占位长度与实物尺寸不一致")
    report("舱内长度 / 电池长度", (BAY_X_FRONT - BAY_X_REAR) - BATTERY[0], ">0")
    report("舱内净高 / 叠加所需高度", bay_height - stack_need, ">0（电池+Jetson 叠加）")
    if (BAY_X_FRONT - BAY_X_REAR) < BATTERY[0]:
        fail("设备舱装不下电池")
    if bay_height < stack_need:
        fail("设备舱净高不足以叠加放置电池与 Jetson")
    # 顶部开口必须完全罩住电池的安装范围，否则电池无法竖直放入或取出。
    report("开口长度 / 电池长度", (OPENING[1] - OPENING[0]) - BATTERY[0], ">0")
    report("开口前端超出电池前端", OPENING[1] - BAT_X_FRONT, ">0（可竖直取出）")
    report("开口后端超出电池后端", BAT_X_REAR - OPENING[0], ">0（可竖直取出）")
    report("开口宽 / Jetson 宽", (OPENING[3] - OPENING[2]) - JETSON[1], ">0")
    if (OPENING[1] - OPENING[0]) < BATTERY[0] or (OPENING[3] - OPENING[2]) < JETSON[1]:
        fail("顶部开口不足以竖直取出电池或 Jetson")
    if OPENING[0] > BAT_X_REAR or OPENING[1] < BAT_X_FRONT:
        fail("顶部开口没有完全罩住电池安装范围，电池无法竖直取放")
    # 开口后端固定在这里，车尾上盖保持完整（从动轮已改为底板安装，不依赖开口）
    report("开口后端 - 安装垫前端", OPENING[0] - PAD_X[1], ">0（开口在安装垫之前）")
    if OPENING[0] < PAD_X[1]:
        fail("顶部开口延伸到了从动轮安装垫上，会切掉安装垫")

    # 打印相关
    if snm > 0:
        fail(f"主壳体存在 {snm} 条非流形边，会影响切片")
    if cnm > 0:
        fail(f"滑盖存在 {cnm} 条非流形边，会影响切片")

    if not verify_solids(shell):
        fail("内部结构实体校验未通过（见上方逐项结果）")

    print("校核结果：", "通过" if ok else "存在不满足项")
    return ok


# =============================================================================
# 七、渲染与导出
# =============================================================================

def setup_render() -> None:
    """配置渲染引擎、地面和灯光。"""

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 960
    scene.render.resolution_y = 700
    try:
        scene.eevee.taa_render_samples = 24
    except AttributeError:
        pass

    world = bpy.data.worlds.new("World")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.6, 0.68, 1.0)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
    scene.world = world

    bm = bmesh.new()
    box(bm, -600.0, 600.0, -600.0, 600.0, -2.0, 0.0)
    ground = new_object("ground", bm)
    set_material(ground, (0.72, 0.73, 0.75), 0.9)

    for name, energy, rotation in (
        ("key", 2.2, (48.0, 0.0, 40.0)),
        ("fill", 0.9, (65.0, 0.0, -130.0)),
        ("rim", 0.7, (75.0, 0.0, 180.0)),
    ):
        data = bpy.data.lights.new(name=name, type="SUN")
        data.energy = energy
        light = bpy.data.objects.new(name, data)
        light.rotation_euler = tuple(math.radians(a) for a in rotation)
        bpy.context.collection.objects.link(light)


def make_render_camera() -> bpy.types.Object:
    """创建渲染用相机对象。"""

    camera_data = bpy.data.cameras.new("cam")
    camera_data.lens = 52.0
    camera = bpy.data.objects.new("cam", camera_data)
    bpy.context.collection.objects.link(camera)
    bpy.context.scene.camera = camera
    return camera


def render_view(camera: bpy.types.Object, out_dir: str, name: str, location, target,
                hide: tuple[str, ...] = ()) -> None:
    """把相机放到指定位置并渲染一张图；hide 中的对象在本次渲染中隐藏。"""

    hidden: list[bpy.types.Object] = []
    for obj_name in hide:
        obj = bpy.data.objects.get(obj_name)
        if obj is not None and not obj.hide_render:
            obj.hide_render = True
            hidden.append(obj)

    camera.location = Vector(location)
    camera.rotation_euler = (camera.location - Vector(target)).to_track_quat("Z", "Y").to_euler()
    bpy.context.scene.render.filepath = os.path.join(out_dir, f"{name}.png")
    bpy.ops.render.render(write_still=True)
    print(f"[RENDER] {name}.png")

    for obj in hidden:
        obj.hide_render = False


def build_cutaway(shell: bpy.types.Object, cover: bpy.types.Object) -> list[bpy.types.Object]:
    """复制外壳并剖掉 y<0 的一半，用于目视检查内部空间。"""

    made: list[bpy.types.Object] = []
    for source in (shell, cover):
        duplicate = source.copy()
        duplicate.data = source.data.copy()
        duplicate.name = f"{source.name}_section"
        bpy.context.collection.objects.link(duplicate)
        bm = bmesh.new()
        box(bm, -600.0, 600.0, -600.0, 0.0, -200.0, 400.0)
        apply_boolean(duplicate, f"section_{source.name}", bm, "DIFFERENCE")
        made.append(duplicate)
    return made


def export_objects(out_dir: str, objects: list[bpy.types.Object]) -> None:
    """把指定对象分别导出为 STL。"""

    for obj in objects:
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        path = os.path.join(out_dir, f"{obj.name}.stl")
        try:
            bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True)
        except TypeError:
            bpy.ops.export_mesh.stl(filepath=path, use_selection=True)
        print(f"[EXPORT] {obj.name}.stl")


def remove_objects(names: list[str]) -> None:
    """按名称删除对象；仅对网格对象释放其网格数据。"""

    for name in names:
        obj = bpy.data.objects.get(name)
        if obj is None:
            continue
        mesh = obj.data if obj.type == "MESH" else None
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh is not None:
            bpy.data.meshes.remove(mesh)


# =============================================================================
# 八、入口
# =============================================================================

def main() -> None:
    out_dir = os.path.dirname(os.path.abspath(__file__))
    render_dir = os.path.join(out_dir, "renders")
    os.makedirs(render_dir, exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)

    shell = build_main_shell()
    cover = build_rear_cover()
    parts = build_placeholders()

    passed = check(shell, cover)
    if not check_interference(shell, cover, parts):
        passed = False

    setup_render()
    camera = make_render_camera()
    target = (0.0, 0.0, 42.0)
    views = {
        "view_iso_front": (520.0, -470.0, 380.0),
        "view_iso_rear": (-520.0, 470.0, 380.0),
        "view_side": (0.0, -700.0, 90.0),
        "view_top": (0.0, 0.0, 700.0),
        "view_front": (640.0, 0.0, 90.0),
        "view_bottom": (-150.0, -300.0, -360.0),
    }
    # 仰视图需要把地面隐藏，否则相机在地面下方时视线被地面挡死。
    for name, location in views.items():
        render_view(camera, render_dir, name, location, target,
                    hide=("ground",) if name == "view_bottom" else ())
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out_dir, "car_shell_assembly.blend"))

    # 后部底面近景：隐藏从动轮占位与地面，专门目视核对 4 个安装通孔。
    # 底面朝下，主光来自上方照不到，因此临时加一盏朝上的补光，
    # 让通孔在亮起来的底面上显示为暗点。
    fill_up_data = bpy.data.lights.new(name="fill_up", type="SUN")
    fill_up_data.energy = 3.0
    fill_up = bpy.data.objects.new("fill_up", fill_up_data)
    fill_up.rotation_euler = (math.radians(-110.0), 0.0, 0.0)
    bpy.context.collection.objects.link(fill_up)
    render_view(camera, render_dir, "view_caster_holes",
                (CASTER_X - 10.0, -35.0, Z_BOTTOM - 85.0), (CASTER_X, 0.0, Z_BOTTOM),
                hide=("ground", "ph_caster"))
    remove_objects(["fill_up"])

    # 剖视图：只显示剖开的一半，并可看到内部占位零件
    sections = build_cutaway(shell, cover)
    for obj in (shell, cover):
        obj.hide_render = True
    render_view(camera, render_dir, "view_section", (60.0, -480.0, 300.0), target)
    # 车尾剖视：同样剖开一半，但相机移到后上方，用于检查车尾掏空效果与安装垫
    render_view(camera, render_dir, "view_section_rear",
                (-330.0, -330.0, 300.0), (-120.0, 0.0, 60.0))

    # 滑盖抽出状态：壳体完整可见、滑盖沿 -X 推出车尾，展示顶部开口暴露的样子
    for obj in sections:
        obj.hide_render = True
    shell.hide_render = False
    cover.location = Vector((-(OPENING[1] - OPENING[0]) - 30.0, 0.0, 0.0))
    render_view(camera, render_dir, "view_cover_open", (-430.0, -420.0, 380.0), target)

    # 清理辅助对象并导出最终外壳
    cover.location = Vector((0.0, 0.0, 0.0))
    for obj in (shell, cover):
        obj.hide_render = False
    remove_objects([obj.name for obj in sections])
    remove_objects([obj.name for obj in parts] + ["ground", "key", "fill", "rim", "cam"])
    export_objects(out_dir, [shell, cover])
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out_dir, "car_shell.blend"))

    print(f"\n[DONE] 主壳体面数 {len(shell.data.polygons)}，滑盖面数 {len(cover.data.polygons)}")
    print(f"[DONE] 输出目录 {out_dir}")
    if not passed:
        sys.exit(2)


if __name__ == "__main__":
    main()
