"""
Fusion 360 脚本: 六边形镂空 (Honeycomb Hollow)
手动选择要镂空的面，自动生成六边形图案并切除。

实际打孔逻辑委托给 hollow_lib.punch_hex_grid，享受批量优化：
- sketch.isComputeDeferred = True (避免 O(N²) 重算)
- 共享 SketchPoint (确保 100+ 孔时 profile 闭环)
- 缓存 cos/sin 表
- 显式 participantBodies
- profile 面积上限过滤 (避免整面被误切)
"""

import os
import sys
import math
import traceback

import adsk.core
import adsk.fusion

# 让脚本能 import 同目录下的 hollow_lib
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)
import hollow_lib  # noqa: E402

app = adsk.core.Application.get()
ui = app.userInterface

# ===== 可调参数 (单位: cm, Fusion 内部单位, 1cm = 10mm) =====
HEX_RADIUS = 0.5          # 六边形外接圆半径 (5mm)
WALL_THICK = 0.2          # 六边形之间最小壁厚 (2mm)
MARGIN = 0.2              # 图案距面边缘留距 (2mm)
CUT_DEPTH_EXPR = '5 mm'   # 切割深度表达式（穿透薄壁即可）


def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('请先打开设计文件')
            return

        faces = []
        while True:
            try:
                sel = ui.selectEntity(
                    f'点选要镂空的面（已选 {len(faces)} 个，按 ESC 完成选择并开始）',
                    'Faces')
                face = sel.entity
                if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                    ui.messageBox('请选择平面，已跳过此曲面。')
                    continue
                if face in faces:
                    faces.remove(face)
                    app.log(f'已取消选择一个面，当前 {len(faces)} 个')
                else:
                    faces.append(face)
            except Exception:
                break

        if not faces:
            ui.messageBox('未选择任何面，已退出。')
            return

        face_done = 0
        hex_total = 0
        timings_summary = []

        timeline = design.timeline
        start_index = timeline.count

        progress = ui.createProgressDialog()
        progress.cancelButtonText = '取消'
        progress.isBackgroundTranslucent = False
        progress.isCancelButtonShown = True
        progress.show('六边形镂空', '初始化...', 0, len(faces), 1)

        for fi, face in enumerate(faces):
            if progress.wasCancelled:
                break

            progress.progressValue = fi
            progress.message = (
                f'处理面 {fi + 1}/{len(faces)}'
                f'（已完成 {face_done} 面, {hex_total} 孔）'
            )
            adsk.doEvents()

            try:
                body = face.body
                result = hollow_lib.punch_hex_grid(
                    body=body,
                    face=face,
                    hex_radius_cm=HEX_RADIUS,
                    hex_wall_cm=WALL_THICK,
                    margin_cm=MARGIN,
                    cut_depth_expr=CUT_DEPTH_EXPR,
                    sketch_name=f'sk_hex_{fi}',
                )
                if result.get('ok'):
                    face_done += 1
                    hex_total += result.get('hex_count', 0)
                    timings_summary.append(
                        f"  面{fi+1}: {result.get('hex_count')}孔 "
                        f"draw={result.get('draw_ms')}ms "
                        f"recompute={result.get('recompute_ms')}ms "
                        f"cut={result.get('cut_ms')}ms"
                    )
                else:
                    app.log(f"面{fi+1} 跳过: {result.get('err', 'unknown')}")
            except Exception as e:
                app.log(f'处理面时出错: {e}\n{traceback.format_exc()}')
                continue

        progress.hide()

        end_index = timeline.count - 1
        if end_index > start_index:
            group = timeline.timelineGroups.add(start_index, end_index)
            group.name = '六边形镂空'

        msg_lines = [f'完成！处理了 {face_done} 个面，共 {hex_total} 个六边形。']
        if timings_summary:
            msg_lines.append('')
            msg_lines.extend(timings_summary)
        msg = '\n'.join(msg_lines)
        if progress.wasCancelled:
            msg = '已取消。\n' + msg
        ui.messageBox(msg)

    except Exception:
        ui.messageBox(f'出错:\n{traceback.format_exc()}')
