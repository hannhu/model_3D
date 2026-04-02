"""
Fusion 360 脚本: 六边形镂空 (Honeycomb Hollow)
手动选择要镂空的面，自动生成六边形图案并切除。
"""

import traceback
import math
import adsk.core
import adsk.fusion

app = adsk.core.Application.get()
ui = app.userInterface

# ===== 可调参数 (单位: cm, Fusion 内部单位, 1cm = 10mm) =====
HEX_RADIUS = 0.5         # 六边形外接圆半径 (5mm)
WALL_THICK = 0.2          # 六边形之间最小壁厚 (2mm)
MARGIN = 0.2              # 图案距面边缘留距 (2mm)
CUT_DEPTH = 0.5           # 切割深度 (5mm)，设为墙壁厚度即可穿透且不伤背后结构


def draw_hex(lines, cx, cy, r):
    """绘制一个 pointy-top 正六边形"""
    pts = []
    for i in range(6):
        a = math.radians(60 * i + 90)
        pts.append(adsk.core.Point3D.create(
            cx + r * math.cos(a),
            cy + r * math.sin(a), 0))
    for i in range(6):
        lines.addByTwoPoints(pts[i], pts[(i + 1) % 6])


def process_face(root, face, dx, dy, r):
    """在一个面上生成六边形网格并切除，返回切除的六边形数量"""
    sketch = root.sketches.add(face)

    p1 = sketch.modelToSketchSpace(face.boundingBox.minPoint)
    p2 = sketch.modelToSketchSpace(face.boundingBox.maxPoint)

    x0 = min(p1.x, p2.x) + MARGIN
    x1 = max(p1.x, p2.x) - MARGIN
    y0 = min(p1.y, p2.y) + MARGIN
    y1 = max(p1.y, p2.y) - MARGIN

    if x1 - x0 < 2 * r or y1 - y0 < 2 * r:
        return 0

    lines = sketch.sketchCurves.sketchLines
    row = 0
    drawn = 0
    y = y0 + r

    while y + r <= y1:
        offset = dx / 2 if row % 2 else 0
        x = x0 + r + offset
        while x + r <= x1:
            inside = True
            for i in range(6):
                a = math.radians(60 * i + 90)
                px = x + r * math.cos(a)
                py = y + r * math.sin(a)
                if px < x0 or px > x1 or py < y0 or py > y1:
                    inside = False
                    break
            if inside:
                draw_hex(lines, x, y, r)
                drawn += 1
            x += dx
        y += dy
        row += 1

    if drawn == 0:
        return 0

    profs = adsk.core.ObjectCollection.create()
    for pi in range(sketch.profiles.count):
        prof = sketch.profiles.item(pi)
        if prof.profileLoops.count == 1:
            loop = prof.profileLoops.item(0)
            if loop.profileCurves.count == 6:
                profs.add(prof)

    if profs.count == 0:
        return 0

    ext = root.features.extrudeFeatures
    dist = adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByReal(CUT_DEPTH))

    for direction in (adsk.fusion.ExtentDirections.PositiveExtentDirection,
                      adsk.fusion.ExtentDirections.NegativeExtentDirection):
        try:
            inp = ext.createInput(profs, adsk.fusion.FeatureOperations.CutFeatureOperation)
            inp.setOneSideExtent(dist, direction)
            ext.add(inp)
            return profs.count
        except Exception:
            continue

    app.log(f'警告: 面 (area={face.area:.2f}) 切除失败，已跳过')
    return 0


def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('请先打开设计文件')
            return

        root = design.rootComponent

        # 手动选面
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

        grid_r = HEX_RADIUS + WALL_THICK / math.sqrt(3)
        dx = math.sqrt(3) * grid_r
        dy = 1.5 * grid_r

        face_done = 0
        hex_total = 0

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
            progress.message = f'处理面 {fi + 1}/{len(faces)}（已完成 {face_done} 面, {hex_total} 孔）'
            adsk.doEvents()

            try:
                n = process_face(root, face, dx, dy, HEX_RADIUS)
                if n > 0:
                    face_done += 1
                    hex_total += n
            except Exception as e:
                app.log(f'处理面时出错: {e}')
                continue

        progress.hide()

        end_index = timeline.count - 1
        if end_index > start_index:
            group = timeline.timelineGroups.add(start_index, end_index)
            group.name = '六边形镂空'

        msg = f'完成！处理了 {face_done} 个面，共 {hex_total} 个六边形。'
        if progress.wasCancelled:
            msg = '已取消。' + msg
        ui.messageBox(msg)

    except Exception:
        ui.messageBox(f'出错:\n{traceback.format_exc()}')
