"""Reusable hex-hole punch helpers for Fusion 360.

Use either as an import inside the MCP execute_code sandbox
(the script directory is added to sys.path automatically by Fusion's API),
or paste the function bodies into ad-hoc snippets.

Performance notes (2026-05-02 profile, 250 mm panel, ~270 hex):
- Without `sketch.isComputeDeferred = True`: ~5 minutes (Fusion recomputes
  the sketch graph after every line addition -> O(N^2)).
- With deferred compute: ~12 seconds end-to-end. Mandatory for any batch hex.

Conventions:
- All numeric input is in Fusion internal units (cm). Use ValueInput/expressions
  for parameter-bound inputs.
- `face.geometry.normal` is the unsigned plane normal; do NOT match by sign.
  Use `find_outer_face(body, axis_idx, sign)` instead.
- Cut depth uses an expression (e.g. `'board_thickness + 1 mm'`) so the cut
  follows the parametric panel thickness.
"""

import math
import time
import adsk.core
import adsk.fusion


def find_outer_face(body, axis_idx, sign):
    """Return the planar face of `body` whose normal aligns with axis `axis_idx`
    (0=X, 1=Y, 2=Z) and whose centroid is the most extreme along `sign*axis`.

    Works around the fact that `face.geometry.normal` may report the unsigned
    plane normal, so two parallel faces can have identical normals. The outer
    face is identified by its centroid coordinate instead.
    """
    best = None
    best_score = -1e18
    for f in body.faces:
        if f.geometry.objectType != adsk.core.Plane.classType():
            continue
        n = f.geometry.normal
        coord = (n.x, n.y, n.z)[axis_idx]
        if abs(abs(coord) - 1.0) > 0.05:
            continue
        c = f.centroid
        v = (c.x, c.y, c.z)[axis_idx] * sign
        if v > best_score:
            best_score = v
            best = f
    return best


def punch_hex_grid(body, face, hex_radius_cm, hex_wall_cm, margin_cm,
                   cut_depth_expr, sketch_name=None):
    """Punch a pointy-top hex grid through `face` of `body` as one cut feature.

    Parameters
    ----------
    body : adsk.fusion.BRepBody
        Target body to cut.
    face : adsk.fusion.BRepFace
        Outer planar face to sketch on. The cut tries both extrude directions.
    hex_radius_cm : float
        Circumscribed-circle radius of each hex (Fusion internal cm).
    hex_wall_cm : float
        Minimum wall thickness between adjacent hexes.
    margin_cm : float
        Distance from face boundary to outermost hex.
    cut_depth_expr : str
        Distance expression for the through-cut, e.g. ``'board_thickness + 1 mm'``.
    sketch_name : str | None
        Optional name for the created sketch.

    Returns
    -------
    dict
        Profile timing data plus ``hex_count`` and ``ok`` flag.
    """
    timings = {}
    t0 = time.time()

    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent

    sketch = root.sketches.add(face)
    if sketch_name:
        sketch.name = sketch_name
    sketch.isComputeDeferred = True   # critical for batch perf
    # IMPORTANT: do NOT set sketch.areProfilesShown = False -- in current
    # Fusion versions it silently DISABLES profile computation entirely,
    # making sketch.profiles.count return 1 (only the host face) regardless
    # of how many closed hex loops you draw. This was the cause of mysterious
    # "no hex profiles" failures at 50+ hex counts.

    bb = face.boundingBox
    p_min = sketch.modelToSketchSpace(bb.minPoint)
    p_max = sketch.modelToSketchSpace(bb.maxPoint)
    x0 = min(p_min.x, p_max.x) + margin_cm
    x1 = max(p_min.x, p_max.x) - margin_cm
    y0 = min(p_min.y, p_max.y) + margin_cm
    y1 = max(p_min.y, p_max.y) - margin_cm
    if x1 - x0 < 2 * hex_radius_cm or y1 - y0 < 2 * hex_radius_cm:
        sketch.deleteMe()
        return {'ok': False, 'err': 'panel too small for hex grid'}

    grid_r = hex_radius_cm + hex_wall_cm / math.sqrt(3)
    dx = math.sqrt(3) * grid_r
    dy = 1.5 * grid_r

    angles = [math.radians(60 * i + 90) for i in range(6)]
    cos_a = [math.cos(a) for a in angles]
    sin_a = [math.sin(a) for a in angles]

    P = adsk.core.Point3D.create
    lines = sketch.sketchCurves.sketchLines
    sketch_points = sketch.sketchPoints
    drawn = 0
    row = 0
    y = y0 + hex_radius_cm
    # IMPORTANT: pre-create one SketchPoint per hex vertex and pass the
    # SketchPoint object (not Point3D) to addByTwoPoints. Otherwise Fusion
    # treats each Point3D as a brand-new point and the 6 lines of one hex
    # are NOT topologically connected -- profile detection fails silently
    # at high counts (works at <=10 hex by proximity, fails at 100+).
    while y + hex_radius_cm <= y1:
        offset = dx / 2 if row % 2 else 0
        x = x0 + hex_radius_cm + offset
        while x + hex_radius_cm <= x1:
            sp = [sketch_points.add(P(x + hex_radius_cm * cos_a[i],
                                       y + hex_radius_cm * sin_a[i], 0))
                   for i in range(6)]
            for i in range(6):
                lines.addByTwoPoints(sp[i], sp[(i + 1) % 6])
            drawn += 1
            x += dx
        y += dy
        row += 1
    timings['draw_ms'] = round((time.time() - t0) * 1000, 1)
    timings['hex_count'] = drawn
    if drawn == 0:
        sketch.deleteMe()
        return {'ok': False, 'err': 'no hex drawn', **timings}

    t1 = time.time()
    sketch.isComputeDeferred = False
    timings['recompute_ms'] = round((time.time() - t1) * 1000, 1)

    t2 = time.time()
    profs = adsk.core.ObjectCollection.create()
    # Expected hex area = (3*sqrt(3)/2) * r^2; cap at 1.5x to keep tolerance but
    # exclude the host face profile that can ALSO be 6-curve / 1-loop when the
    # face has 4 sides + 2 corner fillet arcs.
    hex_area_max = 1.5 * 1.5 * math.sqrt(3) * hex_radius_cm ** 2
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        loops = prof.profileLoops
        if loops.count != 1 or loops.item(0).profileCurves.count != 6:
            continue
        if prof.areaProperties().area > hex_area_max:
            continue  # this is the host face, not a hex
        profs.add(prof)
    timings['filter_profiles_ms'] = round((time.time() - t2) * 1000, 1)
    timings['profile_count'] = profs.count
    if profs.count == 0:
        return {'ok': False, 'err': 'no hex profiles', **timings}

    t3 = time.time()
    cut_dist = adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByString(cut_depth_expr))
    success_dir = None
    last_err = None
    for direction in (adsk.fusion.ExtentDirections.NegativeExtentDirection,
                      adsk.fusion.ExtentDirections.PositiveExtentDirection):
        try:
            inp = root.features.extrudeFeatures.createInput(
                profs, adsk.fusion.FeatureOperations.CutFeatureOperation)
            inp.setOneSideExtent(cut_dist, direction)
            inp.participantBodies = [body]
            root.features.extrudeFeatures.add(inp)
            success_dir = ('neg'
                           if direction == adsk.fusion.ExtentDirections.NegativeExtentDirection
                           else 'pos')
            break
        except Exception as e:
            last_err = str(e)[:100]
    timings['cut_ms'] = round((time.time() - t3) * 1000, 1)
    timings['cut_dir'] = success_dir
    timings['total_ms'] = round((time.time() - t0) * 1000, 1)
    timings['ok'] = success_dir is not None
    if success_dir is None:
        timings['err'] = last_err
    return timings


def punch_hex_panel(body_name, axis_idx, sign,
                    hex_radius_param='hex_radius',
                    hex_wall_param='hex_wall',
                    hex_margin_param='hex_margin',
                    cut_depth_expr='board_thickness + 1 mm'):
    """Convenience wrapper: read params from the active design, locate the
    outer face of the panel and punch a hex grid through it.
    """
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent

    body = root.bRepBodies.itemByName(body_name)
    if body is None:
        return {'ok': False, 'err': f'body {body_name} not found'}
    face = find_outer_face(body, axis_idx, sign)
    if face is None:
        return {'ok': False, 'err': f'no outer face on axis {axis_idx} sign {sign}'}
    hex_r = design.userParameters.itemByName(hex_radius_param).value
    hex_w = design.userParameters.itemByName(hex_wall_param).value
    margin = design.userParameters.itemByName(hex_margin_param).value
    return punch_hex_grid(body, face, hex_r, hex_w, margin,
                          cut_depth_expr, sketch_name=f'sk_hex_{body_name}')
