"""Tercio S1 -> NEMA 23 adapter plate (parametric, CadQuery / OpenCascade).

A compact cap for the rear of a NEMA 23 that replaces the S1's NEMA 17 cover.
Its outline follows the motor's octagonal rear end cap, 0.2 mm inside it, so
nothing sticks out past the motor. The S1 PCB drops into a pocket and screws
into four bosses at its own mounting holes, which puts the AS5600 exactly on
the motor axis. The plate is held by four M3 screws in the motor's rear screw
holes (where the motor's assembly screws come through the rear end cap).

The motor has no rear shaft extension: its shaft end is flush with the rear
face and the Ø6 × 2.5 mm diametric magnet is glued onto it. The plate sets the
magnet-to-AS5600 air gap to 1.0 mm (AS5600 datasheet: 0.5-3 mm).

Reference motor: 57 mm NEMA 23 per the supplied drawing (56.4 max frame, 47.14
± 0.2 holes Ø5.1, pilot Ø38.1 × 1.6, shaft Ø8, flange 5 mm), body length 56 mm.
The rear screw pattern (MOTOR_SCREWS) varies between makers: measure yours.

Coordinates: mm, Z = motor axis, Z = 0 on the motor's rear face, +Z towards the S1.
Origin = AS5600. The S1 sits turned 90° (S1_ROT) so both side-entry connectors
face sides clear of the motor screws: J5 faces -Y (the motor's lead side), J2 +X.

    .venv/bin/python adapter.py            # writes step/ and viewer/models/
"""
from __future__ import annotations

import json
import math
import os

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_STEP = os.path.join(HERE, "step")
OUT_WEB = os.path.join(HERE, "viewer", "models")

# ---------------------------------------------------------------- motor (per drawing)
FRAME = 56.4            # square, max
BODY_L = 56.0           # flange face to rear face ("L max")
FLANGE_T = 5.0
HOLE_PITCH = 47.14
HOLE_D = 5.1
PILOT_D, PILOT_H = 38.1, 1.6
SHAFT_D, SHAFT_L1 = 8.0, 21.0
BODY_CUT = 43.2         # body corners chamfered to |x| + |y| <= BODY_CUT (room for the flange's M5 screws and nuts)
# Assembly screws: cross-head heads on the flange face (drawing), threads open in the rear end cap.
MOTOR_SCREWS = [(sx * HOLE_PITCH / 2, sy * 14.5) for sx in (1, -1) for sy in (1, -1)]
MOTOR_SCREW = "M3"
MOTOR_TAP = 2.5

# ---------------------------------------------------------------- S1 (from the KiCad PCB and the S1 STEP)
PCB = 38.0
PCB_R = 4.0
PCB_T = 1.6
S1_ROT = 90             # S1 turned counter-clockwise in the plate
# In the S1's own frame (KiCad y flipped to point up, origin = AS5600):
S1_HOLES_PCB = [(-15.3, 15.3), (15.346, 15.288), (-15.352, -15.3), (15.3, -15.3)]   # H1-H4
J5_PCB = (-20.1, -13.39, -11.47, 0.87)    # JST PH side entry (x0, x1, y0, y1): overhangs the left edge ...
J5_BELOW = 0.59                           # ... and dips below the PCB
CHIP_BELOW_PCB = 1.84   # AS5600 package underside below the PCB bottom
PARTS_BELOW = 2.79      # lowest bottom-side part below the PCB bottom


def rot(p, deg=S1_ROT):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (round(c * p[0] - s * p[1], 4), round(s * p[0] + c * p[1], 4))


def rot_box(b, deg=S1_ROT):
    pts = [rot((x, y), deg) for x in b[:2] for y in b[2:]]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


S1_HOLES = [rot(p) for p in S1_HOLES_PCB]
J5 = rot_box(J5_PCB)

# ---------------------------------------------------------------- magnet
MAGNET_D, MAGNET_T = 6.0, 2.5
MAGNET_BASE = 0.0       # shaft end flush with the rear face
AIR_GAP = 1.0

Z_PCB = MAGNET_BASE + MAGNET_T + AIR_GAP + CHIP_BELOW_PCB     # PCB underside
Z_TOP = Z_PCB + 1.0                                           # plate top: the rim holds 1 mm of the board's edge

# ---------------------------------------------------------------- plate
PLATE = FRAME - 0.4     # 0.2 mm inside the motor's end cap all round
PLATE_CUT = BODY_CUT - 0.4
POCKET = PCB + 0.4
POCKET_R = PCB_R + 0.2
WINDOW = 38.2           # open under the board (bottom-side parts, magnet)
BOSS_D = 6.4            # supports the PCB at its four holes (nearest bottom part is 3.47 mm from a hole)
TAP_M3 = 2.5
SCREW_CLEAR = 3.4       # M3 clearance for the motor screws
PLATE_SCREW_L = 10.0    # M3 x 10 button head: 4 mm into the motor's rear threads
LOGO = "TERCIO"


def box(x0, y0, z0, dx, dy, dz) -> cq.Workplane:
    return cq.Workplane().add(cq.Solid.makeBox(dx, dy, dz, cq.Vector(x0, y0, z0)))


def octagon(size: float, cut: float, z0: float, h: float) -> cq.Workplane:
    c = size / 2
    k = cut - c
    pts = [(k, c), (-k, c), (-c, k), (-c, -k), (-k, -c), (k, -c), (c, -k), (c, k)]
    return cq.Workplane("XY", origin=(0, 0, z0)).polyline(pts).close().extrude(h)


def adapter() -> cq.Workplane:
    p = octagon(PLATE, PLATE_CUT, 0, Z_TOP).edges("|Z").fillet(1.0)
    p = p.faces(">Z").edges().chamfer(0.6).faces("<Z").edges().chamfer(0.3)
    # Pocket for the PCB.
    p = p.cut(cq.Workplane("XY", origin=(0, 0, Z_PCB)).rect(POCKET, POCKET).extrude(Z_TOP).edges("|Z").fillet(POCKET_R))
    # Window under the board, leaving four bosses (tied to their corners) at the PCB holes.
    win = cq.Workplane("XY", origin=(0, 0, -1)).rect(WINDOW, WINDOW).extrude(Z_PCB + 1.001).edges("|Z").fillet(3.0)
    for x, y in S1_HOLES:
        sx, sy = math.copysign(1, x), math.copysign(1, y)
        boss = cq.Workplane("XY", origin=(x, y, -2)).circle(BOSS_D / 2).extrude(Z_PCB + 3)
        corner = box(x if sx > 0 else x - 6.0, y if sy > 0 else y - 6.0, -2, 6.0, 6.0, Z_PCB + 3)
        win = win.cut(boss).cut(corner)
    p = p.cut(win)
    # M3 threads in the bosses (tap drill; blind from the top, 0.8 mm floor).
    p = p.cut(cq.Workplane("XY", origin=(0, 0, 0.8)).pushPoints(S1_HOLES).circle(TAP_M3 / 2).extrude(Z_TOP))
    # J5 (JST PH, side entry): room for its overhang and a channel out to the edge for the plug.
    x0, x1, y0, y1 = J5
    floor = Z_PCB - J5_BELOW - 0.5
    p = p.cut(box(x0 - 0.7, -PLATE, floor, x1 - x0 + 1.4, PLATE - POCKET / 2 + 0.5, Z_TOP).edges("|Z").fillet(0.6))
    # Motor screws.
    p = p.cut(cq.Workplane("XY", origin=(0, 0, -1)).pushPoints(MOTOR_SCREWS).circle(SCREW_CLEAR / 2).extrude(Z_TOP + 2))
    # Engraved mark on the +X side.
    zc = (0.3 + Z_TOP - 0.6) / 2
    mark = cq.Workplane(cq.Plane(origin=(PLATE / 2, 0, zc), xDir=(0, 1, 0), normal=(1, 0, 0)))
    try:
        p = p.cut(mark.text(LOGO, 2.6, -0.25, combine=False, kind="bold", halign="center", valign="center"))
    except Exception as e:                        # no usable font: skip the mark
        print("logo skipped:", e)
    return p


def magnet() -> tuple[cq.Workplane, cq.Workplane]:
    half = lambda ang: cq.Workplane().add(cq.Solid.makeCylinder(MAGNET_D / 2, MAGNET_T, cq.Vector(0, 0, MAGNET_BASE), cq.Vector(0, 0, 1), 180)
                                          .rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), ang))
    return half(90), half(270)


def socket_screw(d: float, head_d: float, head_h: float, length: float, top: tuple, key: float) -> cq.Workplane:
    """Socket head cap screw whose head sits on `top` (x, y, z), shank pointing down."""
    x, y, z = top
    head = cq.Workplane("XY", origin=(x, y, z)).circle(head_d / 2).extrude(head_h).faces(">Z").edges().chamfer(head_h * 0.12)
    head = head.cut(cq.Workplane("XY", origin=(x, y, z + head_h * 0.45)).polygon(6, key / math.cos(math.pi / 6)).extrude(head_h))
    shank = cq.Workplane("XY", origin=(x, y, z - length)).circle(d / 2).extrude(length).faces("<Z").edges().chamfer(d * 0.08)
    return head.union(shank)


def button_screw(d: float, head_d: float, head_h: float, length: float, top: tuple, key: float) -> cq.Workplane:
    """ISO 7380 button head screw whose head sits on `top` (x, y, z), shank pointing down."""
    x, y, z = top
    head = cq.Workplane("XZ", origin=(x, y, z)).moveTo(0, 0).lineTo(head_d / 2, 0).lineTo(head_d / 2, head_h * 0.25) \
        .threePointArc((head_d * 0.36, head_h * 0.82), (key * 0.62, head_h)).lineTo(0, head_h).close().revolve(360, (0, 0, 0), (0, 1, 0))
    head = head.cut(cq.Workplane("XY", origin=(x, y, z + head_h * 0.35)).polygon(6, key / math.cos(math.pi / 6)).extrude(head_h))
    shank = cq.Workplane("XY", origin=(x, y, z - length)).circle(d / 2).extrude(length).faces("<Z").edges().chamfer(d * 0.08)
    return head.union(shank)


def motor() -> dict[str, cq.Workplane]:
    s = FRAME / 2
    zf = -BODY_L                                         # flange (mounting) face
    parts = {}
    hp = HOLE_PITCH / 2
    screw_pts = MOTOR_SCREWS

    flange = cq.Workplane("XY", origin=(0, 0, zf)).rect(FRAME, FRAME).extrude(FLANGE_T).edges("|Z").fillet(3.0)
    flange = flange.cut(cq.Workplane("XY", origin=(0, 0, zf - 1)).pushPoints([(hp, hp), (-hp, hp), (hp, -hp), (-hp, -hp)]).circle(HOLE_D / 2).extrude(FLANGE_T + 2))
    flange = flange.faces("<Z").edges().chamfer(0.4)
    flange = flange.cut(cq.Workplane("XY", origin=(0, 0, zf - 0.1)).pushPoints(screw_pts).circle(1.7).extrude(3))
    front_band = octagon(FRAME, BODY_CUT, zf + FLANGE_T, 4.0).edges("|Z").fillet(1.0)
    parts["front_cap"] = flange.union(front_band)

    lam0, lam1 = zf + FLANGE_T + 4.0, -8.0
    stator = octagon(FRAME - 0.3, BODY_CUT - 0.3, lam0, lam1 - lam0).edges("|Z").fillet(1.0)
    n = int((lam1 - lam0) / 1.25)
    for k in range(1, n):
        zz = lam0 + k * (lam1 - lam0) / n
        ring = octagon(FRAME + 2, BODY_CUT + 2, zz - 0.18, 0.36).cut(octagon(FRAME - 1.0, BODY_CUT - 1.0, zz - 0.18, 0.36))
        stator = stator.cut(ring)
    parts["stator"] = stator

    rear = octagon(FRAME, BODY_CUT, -8.0, 8.0).edges("|Z").fillet(1.0).faces(">Z").edges().chamfer(0.6)
    rear = rear.cut(cq.Workplane("XY", origin=(0, 0, -0.35)).circle(11.0).extrude(1))          # rear bearing seat
    rear = rear.cut(cq.Workplane("XY", origin=(0, 0, -7)).pushPoints(screw_pts).circle(MOTOR_TAP / 2).extrude(8))  # assembly screw threads
    rear = rear.edges(cq.selectors.BoxSelector((-25, -25, -0.1), (25, 25, 0.1))).edges("%CIRCLE").chamfer(0.25)
    parts["rear_cap"] = rear
    parts["bearing"] = (cq.Workplane("XY", origin=(0, 0, -0.35)).circle(10.9).extrude(0.3)
                        .cut(cq.Workplane("XY", origin=(0, 0, -1)).circle(SHAFT_D / 2 + 0.6).extrude(2)))
    # Shaft: flush with the rear face, Ø8 × L1 out of the front.
    shaft = cq.Workplane("XY", origin=(0, 0, zf - SHAFT_L1)).circle(SHAFT_D / 2).extrude(SHAFT_L1 + BODY_L - 0.05)
    parts["shaft"] = shaft.faces("<Z").edges().chamfer(0.5)
    parts["pilot"] = cq.Workplane("XY", origin=(0, 0, zf - PILOT_H)).circle(PILOT_D / 2).extrude(PILOT_H).faces("<Z").edges().chamfer(0.3)
    # Cross-head pan screws on the flange face.
    heads = cq.Workplane("XY", origin=(0, 0, zf - 1.8)).pushPoints(screw_pts).circle(2.8).extrude(1.8)
    heads = heads.faces("<Z").edges().fillet(0.6)
    for x, y in screw_pts:
        heads = heads.cut(box(x - 1.6, y - 0.35, zf - 1.9, 3.2, 0.7, 1.0)).cut(box(x - 0.35, y - 1.6, zf - 1.9, 0.7, 3.2, 1.0))
    parts["screws"] = heads
    # Lead exit on the -Y side near the rear.
    parts["lead_exit"] = cq.Workplane("XY").box(10.0, 4.0, 5.0).edges("|Y").fillet(1.2).translate((0, -s - 1.6, -4.2))
    return parts


def leads() -> dict[str, cq.Workplane]:
    out = {}
    for name, off in {"lead_black": -3.0, "lead_green": -1.0, "lead_red": 1.0, "lead_blue": 3.0}.items():
        pts = [(off, -FRAME / 2 - 3.2, -4.2), (off, -FRAME / 2 - 8, -8.0), (off * 1.2, -FRAME / 2 - 10, -26), (off * 1.6, -FRAME / 2 - 9, -60)]
        path = cq.Workplane().spline(pts, includeCurrent=False)
        prof = cq.Workplane(cq.Plane(origin=pts[0], xDir=(1, 0, 0), normal=(0, -1, 0))).circle(0.8)
        out[name] = prof.sweep(path)
    return out


# ---------------------------------------------------------------- build + export
COLOURS = {
    "adapter": (0.78, 0.79, 0.81), "magnet_n": (0.75, 0.12, 0.12), "magnet_s": (0.72, 0.74, 0.77),
    "plate_screws": (0.08, 0.08, 0.09), "s1_screws": (0.08, 0.08, 0.09),
    "front_cap": (0.13, 0.13, 0.14), "stator": (0.07, 0.07, 0.08), "rear_cap": (0.13, 0.13, 0.14), "bearing": (0.6, 0.6, 0.62),
    "shaft": (0.82, 0.83, 0.85), "pilot": (0.15, 0.15, 0.16), "screws": (0.55, 0.56, 0.58), "lead_exit": (0.05, 0.05, 0.05),
    "lead_black": (0.05, 0.05, 0.05), "lead_green": (0.10, 0.55, 0.25), "lead_red": (0.80, 0.12, 0.12), "lead_blue": (0.12, 0.30, 0.80),
}


def check() -> dict:
    chip = Z_PCB - CHIP_BELOW_PCB
    info = dict(
        thickness=round(Z_TOP, 3), z_pcb=round(Z_PCB, 3), z_top=round(Z_TOP, 3), magnet_top=round(MAGNET_BASE + MAGNET_T, 3),
        as5600_package=round(chip, 3), air_gap=round(chip - MAGNET_BASE - MAGNET_T, 3), parts_clearance=round(Z_PCB - PARTS_BELOW, 3),
        bolt_pattern=HOLE_PITCH, s1_holes=S1_HOLES, body_length=BODY_L, frame=FRAME, shaft=[SHAFT_D, SHAFT_L1], magnet=[MAGNET_D, MAGNET_T],
    )
    assert 0.5 <= info["air_gap"] <= 3.0, "air gap outside the AS5600 range"
    assert info["parts_clearance"] > MAGNET_BASE, "bottom-side parts would touch the motor"
    engage = PLATE_SCREW_L - Z_TOP
    assert engage >= 3.0, "motor screws too short"
    walls = [min(abs(x) - SCREW_CLEAR / 2 - WINDOW / 2, PLATE / 2 - abs(x) - SCREW_CLEAR / 2,
                 (PLATE_CUT - abs(x) - abs(y)) / math.sqrt(2) - SCREW_CLEAR / 2) for x, y in MOTOR_SCREWS]
    assert min(walls) >= 1.5, f"wall around a motor screw only {min(walls):.2f} mm"
    info.update(plate=PLATE, plate_cut=PLATE_CUT, motor_screws=MOTOR_SCREWS, motor_screw=MOTOR_SCREW, plate_screw_l=PLATE_SCREW_L,
                screw_engagement=round(engage, 2), min_wall=round(min(walls), 2), s1_rot=S1_ROT)
    return info


def main() -> None:
    for d in (OUT_STEP, OUT_WEB):
        os.makedirs(d, exist_ok=True)
        for f in os.listdir(d):
            if f.endswith((".step", ".glb")) and f != "tercio-s1.glb":
                os.remove(os.path.join(d, f))
    info = check()
    print(json.dumps(info, indent=1))

    parts = {"adapter": adapter()}
    parts["magnet_n"], parts["magnet_s"] = magnet()
    screws = None
    for x, y in MOTOR_SCREWS:
        b = button_screw(3.0, 5.7, 1.65, PLATE_SCREW_L, (x, y, Z_TOP), 2.0)
        screws = b if screws is None else screws.union(b)
    parts["plate_screws"] = screws
    s1s = None
    for x, y in S1_HOLES:
        b = socket_screw(3.0, 5.5, 3.0, 6.0, (x, y, Z_PCB + PCB_T), 2.5)
        s1s = b if s1s is None else s1s.union(b)
    parts["s1_screws"] = s1s
    mot = motor()
    mot.update(leads())

    plate = parts["adapter"].val()
    info["mass_g"] = round(plate.Volume() * 2.70e-3, 1)
    print("plate valid", plate.isValid(), "solids", len(parts["adapter"].solids().vals()),
          "mass 6061 %.1f g" % info["mass_g"], "faces", len(plate.Faces()))
    cq.exporters.export(parts["adapter"], os.path.join(OUT_STEP, "Tercio-S1_NEMA23-Adapter.step"), opt={"write_pcurves": True})

    # Web meshes: one mesh per part (the viewer's section caps need closed parts), faces kept separate so
    # edges stay sharp, 0.01 mm tessellation. glTF is Y-up: CAD (x, y, z) -> (x, z, -y).
    import numpy as np
    import trimesh

    def glb(name, wp, key):
        verts, tris, base = [], [], 0
        for face in [f for v in wp.vals() for f in v.Faces()]:
            vs, ts = face.tessellate(0.01, 0.08)
            if not ts:
                continue
            verts.extend((v.x, v.z, -v.y) for v in vs)
            tris.extend((a + base, b + base, c + base) for a, b, c in ts)
            base += len(vs)
        mesh = trimesh.Trimesh(np.array(verts), np.array(tris), process=False)
        rgb = [int(255 * c) for c in COLOURS.get(key, (0.5, 0.5, 0.5))]
        mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=np.tile(rgb + [255], (len(tris), 1)))
        _ = mesh.vertex_normals
        mesh.export(os.path.join(OUT_WEB, f"{name}.glb"))

    for k, v in parts.items():
        glb(k, v, k)
    for k, v in mot.items():
        glb("motor_" + k, v, k)

    # Full assembly: motor, plate, magnet, screws and the S1 (without its NEMA 17 cover, which the plate replaces).
    assy = cq.Assembly(name="Tercio-S1_NEMA23")
    m_assy = cq.Assembly(name="NEMA23_56mm")
    for k, v in mot.items():
        m_assy.add(v, name=k, color=cq.Color(*COLOURS[k]))
    assy.add(m_assy)
    for k in ("adapter", "magnet_n", "magnet_s", "plate_screws", "s1_screws"):
        assy.add(parts[k], name=k, color=cq.Color(*COLOURS[k]))
    s1_path = os.path.join(HERE, "..", "Tercio-S1.step")
    if os.path.exists(s1_path):
        s1 = cq.importers.importStep(s1_path)
        keep = []
        for sol in s1.solids().vals():
            bb = sol.BoundingBox()
            if abs(bb.xlen - 41.3) < 0.05 and abs(bb.ylen - 41.3) < 0.05 and bb.zmax < 0:
                continue                                  # the NEMA 17 cover
            keep.append(sol)
        s1b = (cq.Workplane().add(cq.Compound.makeCompound(keep)).translate((-15.0, 19.0, Z_PCB))
               .rotate((0, 0, 0), (0, 0, 1), S1_ROT))
        assy.add(s1b, name="Tercio_S1", color=cq.Color(0.06, 0.07, 0.08))
    assy.save(os.path.join(OUT_STEP, "Tercio-S1_NEMA23-Assembly.step"))

    with open(os.path.join(OUT_WEB, "layout.json"), "w") as f:
        json.dump(info, f, indent=1)
    print("wrote", sorted(os.listdir(OUT_STEP)))


if __name__ == "__main__":
    main()
