"""centerline_thicken.py

Thicken construction (or solid) sketch curves into a closed band by offsetting
each curve by +/- OFFSET on each side, then capping dead-end points.

Use case: design a wire-like centerline as construction (dashed) geometry,
then call this to convert it into real 2D band profiles (e.g., a 3 mm wide
ribbon) suitable for extrusion or further sketch work.

Approach
--------
For each input curve, two parallel offsets are created:
  - SketchLine  -> two parallel lines at +/- OFFSET (extended a bit past
                   non-dead-end endpoints so adjacent offsets overlap).
  - SketchArc   -> two concentric arcs at radius +/- OFFSET (with a small
                   sweep extension at non-dead-end endpoints).
  - SketchCircle-> two concentric circles at radius +/- OFFSET.

Endpoints that touch only one curve (open ends of the chain) get a 3 mm cap
line connecting the two offset endpoints, perpendicular to the curve tangent.

Limitations
-----------
- Branched chains (3+ curves meeting at a vertex) produce overlapping offsets;
  the union of the resulting profiles is the desired band, but several small
  sliver profiles may also appear at the branch points.
- A closed sub-loop in the centerline produces an extra "interior cavity"
  profile in addition to the band profiles. Use ``get_band_profiles`` to
  filter them out.
- No corner fillet/chamfer; offsets simply overlap at corners.
"""

import math
from collections import defaultdict

import adsk.core
import adsk.fusion


def _curve_geom_key(curve):
    """Return a canonical hashable key for a SketchCurve based on its geometry.

    Useful for detecting duplicates: two curves with the same key have
    identical geometric shape, regardless of stored direction.
    """
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()
    SCIRC = adsk.fusion.SketchCircle.classType()
    if curve.objectType == SLINE:
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        ka = (round(a.x * 10000), round(a.y * 10000))
        kb = (round(b.x * 10000), round(b.y * 10000))
        return ('L',) + tuple(sorted([ka, kb]))
    if curve.objectType == SARC:
        ctr = curve.centerSketchPoint.geometry
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        kctr = (round(ctr.x * 10000), round(ctr.y * 10000))
        kr = round(curve.radius * 10000)
        ka = (round(a.x * 10000), round(a.y * 10000))
        kb = (round(b.x * 10000), round(b.y * 10000))
        return ('A', kctr, kr) + tuple(sorted([ka, kb]))
    if curve.objectType == SCIRC:
        ctr = curve.centerSketchPoint.geometry
        kctr = (round(ctr.x * 10000), round(ctr.y * 10000))
        kr = round(curve.radius * 10000)
        return ('C', kctr, kr)
    return None


def _deduplicate_curves(curves):
    """Return curves with duplicates (same geometry) removed."""
    seen = set()
    unique = []
    for c in curves:
        k = _curve_geom_key(c)
        if k is None:
            unique.append(c)
        elif k not in seen:
            seen.add(k)
            unique.append(c)
    return unique


def _find_arc_chord_lines(curves, tol_mm=0.05):
    """Detect straight centerlines whose two endpoints are both shared with
    the endpoints of some arc in the same set. Such lines are redundant
    "chords" of an arc transition (the arc already provides the routing) and
    thickening them produces parallel band offsets that clutter the corner.

    Returns the set of entityTokens of redundant chord lines (only the LINES
    are flagged; arcs are always kept).
    """
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()
    tol = tol_mm / 10.0  # cm
    arc_endpoints = []
    for c in curves:
        if c.objectType == SARC:
            arc_endpoints.append((c.startSketchPoint.geometry,
                                  c.endSketchPoint.geometry))

    def _near(p, q):
        return abs(p.x - q.x) < tol and abs(p.y - q.y) < tol

    redundant = set()
    for c in curves:
        if c.objectType != SLINE:
            continue
        a = c.startSketchPoint.geometry
        b = c.endSketchPoint.geometry
        for ea, eb in arc_endpoints:
            if (_near(a, ea) and _near(b, eb)) or (_near(a, eb) and _near(b, ea)):
                redundant.add(c.entityToken)
                break
    return redundant


def thicken_centerline(sketch, offset_mm=1.5, only_construction=True,
                       extend_corners=True, deduplicate=True,
                       skip_arc_chord_lines=True):
    """Create offset curves on both sides of each input curve, plus caps at
    dead-end points.

    Parameters
    ----------
    sketch : adsk.fusion.Sketch
        Target sketch (must be open for editing).
    offset_mm : float
        Half-width of the band (each side gets ``offset_mm`` of offset).
        Total band thickness = 2 * offset_mm.
    only_construction : bool
        If True, only thicken curves where ``isConstruction`` is True. If
        False, thicken every curve in the sketch (construction + solid).
    extend_corners : bool
        If True (recommended), extend each offset by ``offset_mm`` past
        non-dead-end endpoints so adjacent offsets overlap and form closed
        profiles.
    deduplicate : bool
        If True, skip centerline curves with identical geometry (helps
        when the original sketch contains accidental duplicates).
    skip_arc_chord_lines : bool
        If True (recommended), skip thickening straight centerlines whose
        two endpoints coincide with the endpoints of an existing arc — the
        arc already provides the corner transition, and a parallel line
        chord generates redundant offsets that clutter the band. The
        construction line itself is NEVER deleted; only its band offsets
        are skipped.

    Returns
    -------
    new_curves : list of SketchCurve
        Every offset/cap curve added to the sketch.
    """
    OFF = offset_mm / 10.0  # cm (Fusion internal units)
    P3D = adsk.core.Point3D.create
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()
    SCIRC = adsk.fusion.SketchCircle.classType()

    centerline_curves = [c for c in sketch.sketchCurves
                          if c.isConstruction or not only_construction]
    if deduplicate:
        centerline_curves = _deduplicate_curves(centerline_curves)
    skip_tokens = (_find_arc_chord_lines(centerline_curves)
                   if skip_arc_chord_lines else set())
    centerline_curves = [c for c in centerline_curves
                         if c.entityToken not in skip_tokens]

    pt_info = defaultdict(list)
    for c in centerline_curves:
        if c.objectType in (SLINE, SARC):
            for is_start, p in [(True, c.startSketchPoint.geometry),
                                (False, c.endSketchPoint.geometry)]:
                key = (round(p.x * 1000), round(p.y * 1000))
                pt_info[key].append((c, is_start, p))
    dead_keys = {k for k, v in pt_info.items() if len(v) == 1}

    sketch.isComputeDeferred = True
    new_curves = []
    try:
        for c in centerline_curves:
            if c.objectType == SLINE:
                a = c.startSketchPoint.geometry
                b = c.endSketchPoint.geometry
                tx, ty = b.x - a.x, b.y - a.y
                L = math.hypot(tx, ty)
                if L < 1e-6:
                    continue
                ux, uy = tx / L, ty / L
                nx, ny = -uy, ux  # 90 deg CCW from tangent
                ka = (round(a.x * 1000), round(a.y * 1000))
                kb = (round(b.x * 1000), round(b.y * 1000))
                ea = 0.0 if (ka in dead_keys or not extend_corners) else OFF
                eb = 0.0 if (kb in dead_keys or not extend_corners) else OFF
                ax, ay = a.x - ux * ea, a.y - uy * ea
                bx, by = b.x + ux * eb, b.y + uy * eb
                for s in (+1, -1):
                    sa = P3D(ax + s * nx * OFF, ay + s * ny * OFF, 0)
                    sb = P3D(bx + s * nx * OFF, by + s * ny * OFF, 0)
                    new_curves.append(
                        sketch.sketchCurves.sketchLines.addByTwoPoints(sa, sb))

            elif c.objectType == SARC:
                ctr = c.centerSketchPoint.geometry
                r = c.radius
                a = c.startSketchPoint.geometry
                b = c.endSketchPoint.geometry
                ang_a = math.atan2(a.y - ctr.y, a.x - ctr.x)
                ang_b = math.atan2(b.y - ctr.y, b.x - ctr.x)
                sweep = ang_b - ang_a
                while sweep > math.pi:
                    sweep -= 2 * math.pi
                while sweep < -math.pi:
                    sweep += 2 * math.pi
                sgn = 1 if sweep >= 0 else -1
                ka = (round(a.x * 1000), round(a.y * 1000))
                kb = (round(b.x * 1000), round(b.y * 1000))
                ea = 0.0 if (ka in dead_keys or not extend_corners) else OFF / r
                eb = 0.0 if (kb in dead_keys or not extend_corners) else OFF / r
                new_ang_a = ang_a - sgn * ea
                new_sweep = sweep + sgn * (ea + eb)
                for s in (+1, -1):
                    new_r = r + s * OFF
                    if new_r < 0.01:
                        continue
                    sa_pt = P3D(ctr.x + math.cos(new_ang_a) * new_r,
                                ctr.y + math.sin(new_ang_a) * new_r, 0)
                    new_curves.append(
                        sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
                            ctr, sa_pt, new_sweep))

            elif c.objectType == SCIRC:
                ctr = c.centerSketchPoint.geometry
                r = c.radius
                for s in (+1, -1):
                    new_r = r + s * OFF
                    if new_r < 0.01:
                        continue
                    new_curves.append(
                        sketch.sketchCurves.sketchCircles.addByCenterRadius(
                            ctr, new_r))

        for k in dead_keys:
            (c, is_start, p) = pt_info[k][0]
            if c.objectType == SLINE:
                a = c.startSketchPoint.geometry
                b = c.endSketchPoint.geometry
                if is_start:
                    tx, ty = a.x - b.x, a.y - b.y
                else:
                    tx, ty = b.x - a.x, b.y - a.y
                L = math.hypot(tx, ty)
                ux, uy = tx / L, ty / L
                nx, ny = -uy, ux
            else:
                ctr = c.centerSketchPoint.geometry
                dx, dy = p.x - ctr.x, p.y - ctr.y
                d = math.hypot(dx, dy)
                nx, ny = dx / d, dy / d
            new_curves.append(
                sketch.sketchCurves.sketchLines.addByTwoPoints(
                    P3D(p.x - nx * OFF, p.y - ny * OFF, 0),
                    P3D(p.x + nx * OFF, p.y + ny * OFF, 0)))
    finally:
        sketch.isComputeDeferred = False

    return new_curves


def get_band_profiles(sketch, offset_mm=1.5, ratio_factor=2.0):
    """Filter sketch profiles, returning only those that look like bands
    (thin strips of width ~= 2 * offset_mm).

    Heuristic: profile is a band if ``area / max(bbox_w, bbox_h) <=
    ratio_factor * offset_mm``. This works because a long thin band has
    area ~= length * width, so area / length ~= width = 2 * offset_mm.
    Cavities (areas inside closed centerline loops) are roughly square
    or filled, with a much larger ratio.

    Parameters
    ----------
    sketch : adsk.fusion.Sketch
    offset_mm : float
        Half-width of the band that was created (per-side offset).
    ratio_factor : float
        Tolerance multiplier. 2.0 means the band's effective width can be
        up to 2 * 2 * offset_mm = 4 * offset_mm before being rejected.

    Returns
    -------
    band_profiles : list of (index, Profile, area_mm2, ratio_mm)
    """
    threshold_cm = ratio_factor * offset_mm / 10.0
    band = []
    for i in range(sketch.profiles.count):
        p = sketch.profiles.item(i)
        bb = p.boundingBox
        w = bb.maxPoint.x - bb.minPoint.x
        h = bb.maxPoint.y - bb.minPoint.y
        long_dim = max(w, h)
        if long_dim < 1e-6:
            continue
        a = p.areaProperties().area
        ratio = a / long_dim
        if ratio <= threshold_cm:
            band.append((i, p, a * 100.0, ratio * 10.0))
    return band


def _curve_midpoint(curve):
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()
    P3D = adsk.core.Point3D.create
    if curve.objectType == SLINE:
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        return P3D((a.x + b.x) / 2, (a.y + b.y) / 2, 0)
    if curve.objectType == SARC:
        ctr = curve.centerSketchPoint.geometry
        r = curve.radius
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        ang_a = math.atan2(a.y - ctr.y, a.x - ctr.x)
        ang_b = math.atan2(b.y - ctr.y, b.x - ctr.x)
        sweep = ang_b - ang_a
        while sweep > math.pi:
            sweep -= 2 * math.pi
        while sweep < -math.pi:
            sweep += 2 * math.pi
        mid_ang = ang_a + sweep / 2
        return P3D(ctr.x + math.cos(mid_ang) * r,
                   ctr.y + math.sin(mid_ang) * r, 0)
    return None


def _dist_pt_line(pt, line):
    a = line.startSketchPoint.geometry
    b = line.endSketchPoint.geometry
    abx, aby = b.x - a.x, b.y - a.y
    L2 = abx * abx + aby * aby
    if L2 < 1e-12:
        return math.hypot(pt.x - a.x, pt.y - a.y)
    t = ((pt.x - a.x) * abx + (pt.y - a.y) * aby) / L2
    t = max(0.0, min(1.0, t))
    cx = a.x + t * abx
    cy = a.y + t * aby
    return math.hypot(pt.x - cx, pt.y - cy)


def _dist_pt_arc(pt, arc):
    ctr = arc.centerSketchPoint.geometry
    r = arc.radius
    dx, dy = pt.x - ctr.x, pt.y - ctr.y
    d = math.hypot(dx, dy)
    if d < 1e-12:
        return r
    ang_p = math.atan2(dy, dx)
    a = arc.startSketchPoint.geometry
    b = arc.endSketchPoint.geometry
    ang_a = math.atan2(a.y - ctr.y, a.x - ctr.x)
    ang_b = math.atan2(b.y - ctr.y, b.x - ctr.x)
    sweep = ang_b - ang_a
    while sweep > math.pi:
        sweep -= 2 * math.pi
    while sweep < -math.pi:
        sweep += 2 * math.pi
    sgn = 1 if sweep >= 0 else -1
    rel = (ang_p - ang_a) * sgn
    while rel > 2 * math.pi:
        rel -= 2 * math.pi
    while rel < -1e-6:
        rel += 2 * math.pi
    if -1e-6 <= rel <= abs(sweep) + 1e-6:
        return abs(d - r)
    return min(math.hypot(pt.x - a.x, pt.y - a.y),
               math.hypot(pt.x - b.x, pt.y - b.y))


def _distance_to_curve(pt, curve):
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()
    SCIRC = adsk.fusion.SketchCircle.classType()
    if curve.objectType == SLINE:
        return _dist_pt_line(pt, curve)
    if curve.objectType == SARC:
        return _dist_pt_arc(pt, curve)
    if curve.objectType == SCIRC:
        ctr = curve.centerSketchPoint.geometry
        d = math.hypot(pt.x - ctr.x, pt.y - ctr.y)
        return abs(d - curve.radius)
    return float('inf')


def _intersect_curves(c1, c2):
    """Return list of Point3D intersections between two SketchCurves."""
    try:
        result = c1.geometry.intersectWithCurve(c2.geometry)
    except Exception:
        return []
    return [result.item(i) for i in range(result.count)]


def _is_curve_endpoint(pt, curve, tol=0.001):
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()
    if curve.objectType in (SLINE, SARC):
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        return ((abs(pt.x - a.x) < tol and abs(pt.y - a.y) < tol) or
                (abs(pt.x - b.x) < tol and abs(pt.y - b.y) < tol))
    return False


def _split_curve_at_points(sketch, curve, points, tol=0.001):
    """Replace `curve` with sub-curves split at the given Point3D list.

    Returns list of new sub-curves (or [curve] if no split was needed).
    """
    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()

    if curve.objectType == SLINE:
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        unique = []
        seen = set()
        for p in points:
            key = (round(p.x * 10000), round(p.y * 10000))
            if key in seen:
                continue
            seen.add(key)
            if abs(p.x - a.x) < tol and abs(p.y - a.y) < tol:
                continue
            if abs(p.x - b.x) < tol and abs(p.y - b.y) < tol:
                continue
            unique.append(p)
        if not unique:
            return [curve]
        unique.sort(key=lambda p: (p.x - a.x) ** 2 + (p.y - a.y) ** 2)
        all_pts = [a] + unique + [b]
        new_curves = []
        for i in range(len(all_pts) - 1):
            new_curves.append(sketch.sketchCurves.sketchLines.addByTwoPoints(
                all_pts[i], all_pts[i + 1]))
        try:
            curve.deleteMe()
        except Exception:
            pass
        return new_curves

    if curve.objectType == SARC:
        ctr = curve.centerSketchPoint.geometry
        a = curve.startSketchPoint.geometry
        b = curve.endSketchPoint.geometry
        ang_a = math.atan2(a.y - ctr.y, a.x - ctr.x)
        ang_b = math.atan2(b.y - ctr.y, b.x - ctr.x)
        sweep_full = ang_b - ang_a
        while sweep_full > math.pi:
            sweep_full -= 2 * math.pi
        while sweep_full < -math.pi:
            sweep_full += 2 * math.pi
        sgn = 1 if sweep_full >= 0 else -1

        unique = []
        seen = set()
        for p in points:
            key = (round(p.x * 10000), round(p.y * 10000))
            if key in seen:
                continue
            seen.add(key)
            if abs(p.x - a.x) < tol and abs(p.y - a.y) < tol:
                continue
            if abs(p.x - b.x) < tol and abs(p.y - b.y) < tol:
                continue
            unique.append(p)
        if not unique:
            return [curve]

        def rel_ang(p):
            ang = math.atan2(p.y - ctr.y, p.x - ctr.x)
            d = (ang - ang_a) * sgn
            while d > 2 * math.pi:
                d -= 2 * math.pi
            while d < 0:
                d += 2 * math.pi
            return d

        unique.sort(key=rel_ang)
        all_pts = [a] + unique + [b]
        new_curves = []
        for i in range(len(all_pts) - 1):
            p1 = all_pts[i]
            p2 = all_pts[i + 1]
            ang1 = math.atan2(p1.y - ctr.y, p1.x - ctr.x)
            ang2 = math.atan2(p2.y - ctr.y, p2.x - ctr.x)
            sweep = ang2 - ang1
            while sweep * sgn < -1e-6:
                sweep += sgn * 2 * math.pi
            while sweep * sgn > 2 * math.pi - 1e-6:
                sweep -= sgn * 2 * math.pi
            try:
                new_curves.append(
                    sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
                        ctr, p1, sweep))
            except Exception:
                pass
        try:
            curve.deleteMe()
        except Exception:
            pass
        return new_curves

    return [curve]


def trim_band_interior(sketch, centerline_curves, offset_mm=1.5,
                       interior_factor=0.95, stub_factor=1.2):
    """Clean up the band created by ``thicken_centerline``:

    Pass 1 -- break offset curves at every mutual intersection, then delete
    sub-segments whose midpoint lies strictly inside the band region (closer
    than ``interior_factor * offset_mm`` to any centerline curve).

    Pass 2 -- iteratively remove dangling stubs: short curves
    (length <= ``stub_factor * offset_mm``) that have at least one endpoint
    not shared with any other band curve. These are the corner-extension
    leftovers that fall outside the strict-interior threshold but still
    clutter the boundary.

    Caps (offsets that did not split because they have no intersection with
    any other band curve) are NEVER deleted in pass 1 because their midpoint
    sits on a centerline endpoint; they are still considered for pass 2 but
    survive because both their endpoints are shared with offset endpoints.

    Returns
    -------
    n_deleted : int
        Total number of sub-segments deleted across both passes.
    """
    OFF = offset_mm / 10.0
    THRESHOLD = OFF * interior_factor
    STUB_LEN = OFF * stub_factor

    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()

    centerline_set = {c.entityToken for c in centerline_curves}
    centerline_list = list(centerline_curves)

    sketch.isComputeDeferred = True
    n_deleted = 0
    try:
        offset_curves = [c for c in sketch.sketchCurves
                         if c.entityToken not in centerline_set
                         and not c.isConstruction
                         and c.objectType in (SLINE, SARC)]

        pts_per_curve = defaultdict(list)
        for i, ci in enumerate(offset_curves):
            for cj in offset_curves[i + 1:]:
                for pt in _intersect_curves(ci, cj):
                    if not _is_curve_endpoint(pt, ci):
                        pts_per_curve[ci.entityToken].append(pt)
                    if not _is_curve_endpoint(pt, cj):
                        pts_per_curve[cj.entityToken].append(pt)

        broken_segments = []
        unbroken_segments = []
        for c in offset_curves:
            pts = pts_per_curve.get(c.entityToken, [])
            if not pts:
                unbroken_segments.append(c)
            else:
                broken_segments.extend(_split_curve_at_points(sketch, c, pts))

        # Pass 1: only check broken sub-segments against the strict interior
        # threshold. Unbroken curves (caps and isolated offsets) are kept.
        for c in broken_segments:
            mid = _curve_midpoint(c)
            if mid is None:
                continue
            d_min = min(_distance_to_curve(mid, cc) for cc in centerline_list)
            if d_min < THRESHOLD:
                try:
                    c.deleteMe()
                    n_deleted += 1
                except Exception:
                    pass

        # Pass 2: iteratively prune short dangling stubs.
        while True:
            current = [c for c in sketch.sketchCurves
                       if c.entityToken not in centerline_set
                       and not c.isConstruction
                       and c.objectType in (SLINE, SARC)]
            pt_count = defaultdict(int)
            for c in current:
                for sp in (c.startSketchPoint, c.endSketchPoint):
                    p = sp.geometry
                    key = (round(p.x * 1000), round(p.y * 1000))
                    pt_count[key] += 1
            stubs = []
            for c in current:
                if c.length > STUB_LEN:
                    continue
                a = c.startSketchPoint.geometry
                b = c.endSketchPoint.geometry
                ka = (round(a.x * 1000), round(a.y * 1000))
                kb = (round(b.x * 1000), round(b.y * 1000))
                if pt_count[ka] == 1 or pt_count[kb] == 1:
                    stubs.append(c)
            if not stubs:
                break
            for c in stubs:
                try:
                    c.deleteMe()
                    n_deleted += 1
                except Exception:
                    pass
    finally:
        sketch.isComputeDeferred = False

    return n_deleted


def trim_concave_corners(sketch, centerline_curves, offset_mm=1.5,
                         add_coincident=True, tol_mm=0.05):
    """Trim the "fish-mouth" self-intersection that appears when two adjacent
    centerline arcs share an endpoint and curve in the same rotational sense
    (e.g. an "S"-bend like a necking transition: arc-up then arc-down).

    For such a pair, the *outer* offset arcs (radius = centerline_radius +
    offset_mm on the convex side) over-shoot the shared centerline endpoint
    and cross each other inside the band region, leaving a kissing-tongue
    pair instead of a clean cusp.

    For each detected pair, this function:
      1. Computes the analytic intersection of the two outer R-circles.
      2. Picks the intersection that lies on the *outer* side of the
         shared centerline endpoint (the kissing point).
      3. Deletes the two outer offset arcs and rebuilds each one going
         from its leg-side endpoint to the kissing point (preserving the
         original sweep direction).
      4. Optionally adds a coincident constraint between the two new arc
         endpoints at the kissing point.

    Inner offsets (radius = centerline_radius - offset_mm) DO NOT need
    trimming because their arcs naturally fall short of the centerline
    endpoint by `offset_mm`, leaving a clean inner cusp.

    Pre-conditions
    --------------
    Run AFTER ``thicken_centerline`` and AFTER ``trim_band_interior``.

    Returns
    -------
    dict {
        'pairs_fixed': int,         # how many concave-arc pairs were trimmed
        'arcs_rebuilt': int,        # new outer arcs created (= 2 * pairs_fixed)
        'coincident_added': int,    # constraints created at kissing points
    }
    """
    SARC = adsk.fusion.SketchArc.classType()
    P3D = adsk.core.Point3D.create
    OFF = offset_mm / 10.0  # cm

    centerline_arcs = [c for c in centerline_curves
                       if c.objectType == SARC]

    pairs_fixed = 0
    arcs_rebuilt = 0
    coincident_added = 0

    sketch.isComputeDeferred = True
    try:
        # Index centerline arcs by their endpoints to find "shared endpoint"
        # pairs cheaply.
        from collections import defaultdict
        pt_index = defaultdict(list)
        for ca in centerline_arcs:
            for sp in (ca.startSketchPoint, ca.endSketchPoint):
                p = sp.geometry
                key = (round(p.x * 1000), round(p.y * 1000))
                pt_index[key].append(ca)

        # Process each shared-endpoint vertex with exactly 2 centerline arcs.
        seen_pairs = set()
        for key, arcs in pt_index.items():
            if len(arcs) != 2:
                continue
            ca1, ca2 = arcs
            pair_key = tuple(sorted([ca1.entityToken, ca2.entityToken]))
            if pair_key in seen_pairs:
                continue
            seen_pairs.add(pair_key)

            ctr1 = ca1.centerSketchPoint.geometry
            ctr2 = ca2.centerSketchPoint.geometry
            r1 = ca1.radius
            r2 = ca2.radius

            # Outer offset radii (convex side)
            R1 = r1 + OFF
            R2 = r2 + OFF

            # Distance between centers
            dx = ctr2.x - ctr1.x
            dy = ctr2.y - ctr1.y
            d = math.hypot(dx, dy)
            if d < 1e-9 or d > R1 + R2 or d < abs(R1 - R2):
                continue  # offsets do not intersect

            # Two intersection points (on the line connecting the centers,
            # offset perpendicular by h).
            a = (d * d + R1 * R1 - R2 * R2) / (2 * d)
            h2 = R1 * R1 - a * a
            if h2 < 0:
                continue
            h = math.sqrt(h2)
            mx = ctr1.x + a * dx / d
            my = ctr1.y + a * dy / d
            px = -dy / d
            py = dx / d
            cand1 = (mx + h * px, my + h * py)
            cand2 = (mx - h * px, my - h * py)

            # The kissing point is the one OPPOSITE the shared centerline
            # endpoint relative to the line between centers (i.e. on the
            # outer side of the bend).
            shared_key = key
            shared_x = shared_key[0] / 1000.0
            shared_y = shared_key[1] / 1000.0
            # Project the shared endpoint onto the centers' normal
            sproj = (shared_x - mx) * px + (shared_y - my) * py
            # We want the candidate with sproj of OPPOSITE sign
            if sproj >= 0:
                kiss = cand2
            else:
                kiss = cand1

            # Find the corresponding outer-offset arcs in the sketch:
            # arcs concentric with each centerline arc whose radius is R.
            def _find_outer_offset(parent_ca, R_target):
                pctr = parent_ca.centerSketchPoint.geometry
                for c in sketch.sketchCurves:
                    if c.isConstruction or c.objectType != SARC:
                        continue
                    if abs(c.radius - R_target) > tol_mm / 10.0:
                        continue
                    cctr = c.centerSketchPoint.geometry
                    if abs(cctr.x - pctr.x) > 1e-3 or abs(cctr.y - pctr.y) > 1e-3:
                        continue
                    return c
                return None

            oa1 = _find_outer_offset(ca1, R1)
            oa2 = _find_outer_offset(ca2, R2)
            if oa1 is None or oa2 is None:
                continue

            # Skip if BOTH outer arcs already have an endpoint at the kiss
            # point (e.g. user/script already trimmed this corner).
            def _has_endpoint_near(arc, pt, tol=0.005):  # 0.05 mm
                a = arc.startSketchPoint.geometry
                b = arc.endSketchPoint.geometry
                return ((abs(a.x - pt[0]) < tol and abs(a.y - pt[1]) < tol)
                        or (abs(b.x - pt[0]) < tol and abs(b.y - pt[1]) < tol))
            if _has_endpoint_near(oa1, kiss) and _has_endpoint_near(oa2, kiss):
                continue

            # For each outer arc, identify the "leg-side" endpoint = the one
            # NOT on the shared-endpoint side of the centerline arc.
            def _leg_endpoint(oa, parent_ca):
                # Endpoints of the outer arc - the one whose angle (relative
                # to the parent center) matches the centerline's leg-side
                # endpoint angle (within tol).
                pctr = parent_ca.centerSketchPoint.geometry
                # Centerline's leg-side endpoint = NOT the shared one.
                p_a = parent_ca.startSketchPoint.geometry
                p_b = parent_ca.endSketchPoint.geometry
                if (abs(p_a.x - shared_x) < 1e-3
                        and abs(p_a.y - shared_y) < 1e-3):
                    leg_pt = p_b
                else:
                    leg_pt = p_a
                ang_leg = math.atan2(leg_pt.y - pctr.y, leg_pt.x - pctr.x)
                # Pick the outer-arc endpoint whose angle matches ang_leg
                o_a = oa.startSketchPoint.geometry
                o_b = oa.endSketchPoint.geometry
                ang_oa = math.atan2(o_a.y - pctr.y, o_a.x - pctr.x)
                ang_ob = math.atan2(o_b.y - pctr.y, o_b.x - pctr.x)
                d_a = abs((ang_oa - ang_leg + math.pi) % (2 * math.pi) - math.pi)
                d_b = abs((ang_ob - ang_leg + math.pi) % (2 * math.pi) - math.pi)
                return o_a if d_a < d_b else o_b

            leg1 = _leg_endpoint(oa1, ca1)
            leg2 = _leg_endpoint(oa2, ca2)

            # Rebuild each outer arc: from leg endpoint, sweep to kiss
            # (preserving the parent centerline's sweep direction).
            def _rebuild(oa, parent_ca, leg_pt, kiss_pt):
                pctr = parent_ca.centerSketchPoint.geometry
                # Determine original sweep direction
                p_a = parent_ca.startSketchPoint.geometry
                p_b = parent_ca.endSketchPoint.geometry
                ang_pa = math.atan2(p_a.y - pctr.y, p_a.x - pctr.x)
                ang_pb = math.atan2(p_b.y - pctr.y, p_b.x - pctr.x)
                sweep = ang_pb - ang_pa
                while sweep > math.pi:
                    sweep -= 2 * math.pi
                while sweep < -math.pi:
                    sweep += 2 * math.pi
                sgn = 1 if sweep >= 0 else -1
                # Sweep from leg angle to kiss angle in same direction
                ang_leg = math.atan2(leg_pt.y - pctr.y, leg_pt.x - pctr.x)
                ang_kiss = math.atan2(kiss_pt[1] - pctr.y,
                                       kiss_pt[0] - pctr.x)
                new_sweep = ang_kiss - ang_leg
                while new_sweep * sgn < 0:
                    new_sweep += sgn * 2 * math.pi
                while abs(new_sweep) > math.pi:
                    new_sweep -= sgn * 2 * math.pi
                ctr_pt = P3D(pctr.x, pctr.y, 0)
                start_pt = P3D(leg_pt.x, leg_pt.y, 0)
                new_arc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
                    ctr_pt, start_pt, new_sweep)
                oa.deleteMe()
                return new_arc

            new_oa1 = _rebuild(oa1, ca1, leg1, kiss)
            new_oa2 = _rebuild(oa2, ca2, leg2, kiss)
            arcs_rebuilt += 2
            pairs_fixed += 1

            if add_coincident:
                # Find the kissing-side endpoint of each new arc and coincide.
                def _kiss_endpoint(arc, kiss_pt):
                    a = arc.startSketchPoint
                    b = arc.endSketchPoint
                    da = math.hypot(a.geometry.x - kiss_pt[0],
                                    a.geometry.y - kiss_pt[1])
                    db = math.hypot(b.geometry.x - kiss_pt[0],
                                    b.geometry.y - kiss_pt[1])
                    return a if da < db else b

                k1 = _kiss_endpoint(new_oa1, kiss)
                k2 = _kiss_endpoint(new_oa2, kiss)
                try:
                    sketch.geometricConstraints.addCoincident(k1, k2)
                    coincident_added += 1
                except Exception:
                    pass
    finally:
        sketch.isComputeDeferred = False

    return {
        'pairs_fixed': pairs_fixed,
        'arcs_rebuilt': arcs_rebuilt,
        'coincident_added': coincident_added,
    }


def add_band_constraints(sketch, centerline_curves=None, offset_mm=1.5,
                          add_parallel=True, add_concentric=True,
                          add_distance_dims=True, half_width_param=None,
                          tol_mm=0.05):
    """Add coincident + parallel + concentric constraints (and optional
    offset/radius dimensions) to keep the cleaned band parametrically locked
    to the centerline.

    Constraint types
    ----------------
    - Coincident       : at every point where >=2 band-curve endpoints land
                         on the same coordinate.
    - Parallel         : between each band line and its parent centerline
                         line (the one that runs parallel at OFFSET
                         distance).
    - Concentric       : between each band arc and its parent centerline arc
                         (sharing the same center).
    - Offset distance  : (only if add_distance_dims=True) the perpendicular
                         distance between each band line and its parent
                         centerline line is dimensioned to ``half_width_param``
                         (or numeric ``offset_mm`` if no param is given).
                         Locks band thickness so dimension changes elsewhere
                         do not distort it.
    - Radius equation  : (only if add_distance_dims=True) each band arc's
                         radius is dimensioned to ``parent_radius +/-
                         half_width_param`` so it follows centerline arc
                         radius changes.

    Parameters
    ----------
    sketch : adsk.fusion.Sketch
    centerline_curves : iterable of SketchCurve or None
        Original centerline (construction) curves. If None, all
        ``isConstruction == True`` curves in the sketch are used (after
        deduplication).
    offset_mm : float
        Half-width of the band. Used as the numeric offset distance when
        ``half_width_param`` is None.
    add_parallel, add_concentric, add_distance_dims : bool
        Toggle each constraint family individually.
    half_width_param : str or None
        Name of an existing user parameter that holds the band's half-width
        (e.g. ``'band_half_width'``). When provided, distance dimensions
        reference the parameter expression so editing the parameter rescales
        the whole band. If None, raw numeric ``offset_mm`` is used.
    tol_mm : float
        Tolerance (mm) for matching parallel/concentric pairs.

    Returns
    -------
    counts : dict
        Counts of each constraint/dimension category added.
    """
    OFF = offset_mm / 10.0
    TOL = tol_mm / 10.0

    SLINE = adsk.fusion.SketchLine.classType()
    SARC = adsk.fusion.SketchArc.classType()

    if centerline_curves is None:
        centerline_curves = _deduplicate_curves(
            [c for c in sketch.sketchCurves if c.isConstruction])
    centerline_set = {c.entityToken for c in centerline_curves}
    centerline_lines = [c for c in centerline_curves if c.objectType == SLINE]
    centerline_arcs = [c for c in centerline_curves if c.objectType == SARC]

    band_curves = [c for c in sketch.sketchCurves
                   if c.entityToken not in centerline_set
                   and not c.isConstruction
                   and c.objectType in (SLINE, SARC)]

    constraints = sketch.geometricConstraints
    dims = sketch.sketchDimensions
    counts = {'coincident': 0, 'parallel': 0, 'concentric': 0,
              'offset_dim': 0, 'radius_dim': 0}

    if half_width_param is not None:
        half_expr = half_width_param
    else:
        half_expr = '{} mm'.format(offset_mm)

    sketch.isComputeDeferred = True
    try:
        pt_groups = defaultdict(list)
        for c in band_curves:
            for sp in (c.startSketchPoint, c.endSketchPoint):
                p = sp.geometry
                key = (round(p.x * 10000), round(p.y * 10000))
                pt_groups[key].append(sp)

        for sps in pt_groups.values():
            if len(sps) < 2:
                continue
            anchor = sps[0]
            for other in sps[1:]:
                try:
                    constraints.addCoincident(other, anchor)
                    counts['coincident'] += 1
                except Exception:
                    pass

        # Track which centerline a band line is paired with so we add
        # at most ONE offset distance dim per centerline line per side.
        offset_dim_added = set()  # (centerline_token, sign)

        if add_parallel:
            for ol in band_curves:
                if ol.objectType != SLINE:
                    continue
                oa = ol.startSketchPoint.geometry
                ob = ol.endSketchPoint.geometry
                otx, oty = ob.x - oa.x, ob.y - oa.y
                oL = math.hypot(otx, oty)
                if oL < 1e-6:
                    continue
                mid = ((oa.x + ob.x) / 2, (oa.y + ob.y) / 2)
                for cl in centerline_lines:
                    ca = cl.startSketchPoint.geometry
                    cb = cl.endSketchPoint.geometry
                    ctx, cty = cb.x - ca.x, cb.y - ca.y
                    cL = math.hypot(ctx, cty)
                    if cL < 1e-6:
                        continue
                    cross = (otx * cty - oty * ctx) / (oL * cL)
                    if abs(cross) > 0.01:
                        continue
                    cnx, cny = -cty / cL, ctx / cL
                    perp = (mid[0] - ca.x) * cnx + (mid[1] - ca.y) * cny
                    if abs(abs(perp) - OFF) > TOL:
                        continue
                    try:
                        constraints.addParallel(ol, cl)
                        counts['parallel'] += 1
                    except Exception:
                        pass
                    if add_distance_dims:
                        sign = 1 if perp >= 0 else -1
                        key = (cl.entityToken, sign)
                        if key not in offset_dim_added:
                            text_pt = adsk.core.Point3D.create(
                                mid[0] + cnx * OFF * 2 * sign,
                                mid[1] + cny * OFF * 2 * sign, 0)
                            try:
                                dim = dims.addOffsetDimension(cl, ol, text_pt)
                                dim.parameter.expression = half_expr
                                counts['offset_dim'] += 1
                                offset_dim_added.add(key)
                            except Exception:
                                pass
                    break

        radius_dim_added = set()  # (centerline_token, +1 or -1 for outer/inner)
        if add_concentric:
            for oa in band_curves:
                if oa.objectType != SARC:
                    continue
                octr = oa.centerSketchPoint.geometry
                for ca in centerline_arcs:
                    cctr = ca.centerSketchPoint.geometry
                    if not (abs(octr.x - cctr.x) < TOL
                            and abs(octr.y - cctr.y) < TOL):
                        continue
                    try:
                        constraints.addConcentric(oa, ca)
                        counts['concentric'] += 1
                    except Exception:
                        pass
                    if add_distance_dims:
                        sign = 1 if oa.radius > ca.radius else -1
                        key = (ca.entityToken, sign)
                        if key not in radius_dim_added:
                            mid = oa.centerSketchPoint.geometry
                            r = oa.radius
                            text_pt = adsk.core.Point3D.create(
                                mid.x + r * 1.4, mid.y + r * 1.4, 0)
                            try:
                                rdim = dims.addRadialDimension(oa, text_pt)
                                # Express band radius in terms of centerline
                                # radius +/- half-width. Round to a clean
                                # 0.001 mm precision to avoid floating-point
                                # noise like "9.99999999 mm + ...".
                                cr_param = round(ca.radius * 10.0, 3)
                                if sign > 0:
                                    rdim.parameter.expression = (
                                        '{} mm + {}'.format(
                                            cr_param, half_expr))
                                else:
                                    rdim.parameter.expression = (
                                        '{} mm - {}'.format(
                                            cr_param, half_expr))
                                counts['radius_dim'] += 1
                                radius_dim_added.add(key)
                            except Exception:
                                pass
                    break
    finally:
        sketch.isComputeDeferred = False

    return counts


def extrude_band(sketch, distance_mm, offset_mm=1.5, ratio_factor=2.0,
                 operation=None, body_name=None):
    """One-shot helper: thicken (already done) -> filter band profiles ->
    extrude as a single new body of the given thickness.

    Note: requires ``thicken_centerline`` to have been run first so that the
    sketch already contains the offset curves and resulting profiles.

    Parameters
    ----------
    sketch : adsk.fusion.Sketch
    distance_mm : float
        Extrusion distance in mm.
    offset_mm : float
        Half-width used in ``thicken_centerline``.
    ratio_factor : float
        Same heuristic factor as ``get_band_profiles``.
    operation : adsk.fusion.FeatureOperations or None
        Defaults to NewBody.
    body_name : str or None
        Optional name for the resulting body.

    Returns
    -------
    extrude : adsk.fusion.ExtrudeFeature
    """
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent

    if operation is None:
        operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation

    band_profiles = get_band_profiles(sketch, offset_mm, ratio_factor)
    if not band_profiles:
        raise RuntimeError("No band profiles found; nothing to extrude.")

    coll = adsk.core.ObjectCollection.create()
    for _, prof, _, _ in band_profiles:
        coll.add(prof)

    inp = root.features.extrudeFeatures.createInput(coll, operation)
    dist = adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByString('{} mm'.format(distance_mm)))
    inp.setOneSideExtent(dist,
                          adsk.fusion.ExtentDirections.PositiveExtentDirection)
    extrude = root.features.extrudeFeatures.add(inp)
    if body_name and extrude.bodies.count > 0:
        extrude.bodies.item(0).name = body_name
    return extrude
