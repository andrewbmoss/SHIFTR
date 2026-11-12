"""
SHIFTR bridge case for a WT32-ETH01 with soldered (bottom) headers.

Two printed parts:
  base  - open box with RJ45 slot, two board pillars and a DC-005 barrel jack window in the far end
  lid   - snap-fit lid with three tactile-switch pockets, LED hole and engraved labels

Coordinates (mm): origin at the inside floor corner on the RJ45 end.
  x = along the board (RJ45 end at x = 0), y = across, z = up.
Every dimension is a parameter below; change it and re-run:
  pip install cadquery && python shiftr_case.py
The STEP files open in FreeCAD (File > Open) and are fully editable there.
"""

import os
import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# ---------------------------------------------------------------- printing
WALL = 2.4          # side walls
FLOOR = 2.4         # base floor
LID = 2.4           # lid plate (switch actuator protrusion depends on this)
FIT = 0.2           # clearance per side for the lid lip and RJ45
CORNER_R = 3.0      # outer vertical edge rounding

# ---------------------------------------------------------------- inside
IN_L = 88.0         # inside length (x)
IN_W = 54.0         # inside width (y)
IN_H = 44.0         # inside height (z), floor top to lid underside

# ---------------------------------------------------------------- board (WT32-ETH01)
PCB_L, PCB_W, PCB_T = 55.5, 25.5, 1.6
PCB_X0 = 2.5                     # gap between RJ45-end board edge and inner wall
PCB_Y0 = (IN_W - PCB_W) / 2      # centred
TAPE = 1.0                       # double-sided foam tape on the pillars
PILLAR_TOP = 24.0                # board bottom = PILLAR_TOP + TAPE = 25 (room for Dupont plugs)
PCB_Z0 = PILLAR_TOP + TAPE
RJ_W, RJ_H, RJ_L, RJ_OVERHANG = 16.0, 13.5, 21.0, 4.5
RJ_Y = IN_W / 2                  # RJ45 is centred on the board

# ---------------------------------------------------------------- lid features
# (x position, kind, label). Buttons sit beside the board in a row, labels below.
ITEMS = [
    (11.0, "led", "LED"),
    (33.0, "switch", "FUNCTION"),
    (55.0, "switch", "RESET"),
    (77.0, "switch", "RECOVERY"),
]
ITEM_Y = 12.0       # hole centre line (y)
LABEL_Y = 4.5       # label centre line (y)
LABEL_H = 3.2       # text height
ENGRAVE = 0.6       # engraving depth
SW_BODY = 6.0       # 6 x 6 mm tactile switch
SW_HOLE = 3.9       # 3.5 mm round actuator
SW_POCKET_DEPTH = 2.5   # shorter than the 3.5 mm body, so the legs come out below the walls
SW_POCKET_WALL = 1.2
LED_HOLE = 5.1      # 5 mm LED, flange stops on the underside (use 3.1 for a 3 mm LED)

LIP_T, LIP_H = 1.6, 4.0
SNAP_R, SNAP_LEN, SNAP_Z = 0.5, 30.0, 41.5

# ---------------------------------------------------------------- power jack (DC-005, 5.5 x 2.1 mm)
# The jack slides into a window in the far end wall from inside, its face flush with the outside.
DC_L, DC_W, DC_H = 14.2, 9.0, 11.0   # body: depth (x) x width (y) x height (z); measure yours
DC_Z0 = 4.5                          # jack bottom above the floor, leaves room for the solder tabs
DC_Y = IN_W / 2                      # centred in the end wall
DC_FIT = 0.15                        # window clearance per side
DC_SUPPORT = 5.0                     # length of the shelf under the front of the jack (tabs are further back)
DC_STOP_GAP = 0.3                    # gap between the jack's back and the stops


def box(x0, y0, z0, x1, y1, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_x(xc, yc, zc, r, length):
    return cq.Workplane("YZ").circle(r).extrude(length).translate((xc - length / 2, yc, zc))


def make_base():
    outer = (cq.Workplane("XY").box(IN_L + 2 * WALL, IN_W + 2 * WALL, IN_H + FLOOR, centered=False)
             .translate((-WALL, -WALL, -FLOOR)).edges("|Z").fillet(CORNER_R))
    base = outer.cut(box(0, 0, 0, IN_L, IN_W, IN_H + 1))

    # RJ45 U-slot, open to the rim so the board drops straight in
    slot_bottom = PCB_Z0 - 1.0 + PCB_T - 0.3   # still clears the jack if no tape is used
    base = base.cut(box(-WALL - 1, RJ_Y - RJ_W / 2 - FIT - 0.05, slot_bottom, 1, RJ_Y + RJ_W / 2 + FIT + 0.05, IN_H + 1))

    # board pillars under the centre of the board, clear of the RJ45 pins and the header rows
    for px in (26.0, 50.0):
        base = base.union(box(px - 3, RJ_Y - 5, 0, px + 3, RJ_Y + 5, PILLAR_TOP))

    # DC-005 barrel jack in the far end wall (opposite the RJ45)
    front = IN_L + WALL                  # jack face flush with the outside of the wall
    rear = front - DC_L
    y0, y1 = DC_Y - DC_W / 2, DC_Y + DC_W / 2
    # shelf under the front of the jack, clear of the solder tabs further back
    base = base.union(box(IN_L - DC_SUPPORT, y0, 0, IN_L, y1, DC_Z0))
    # side rails to keep it straight (and something for glue to grip)
    for ry0, ry1 in ((y0 - DC_FIT - 1.6, y0 - DC_FIT), (y1 + DC_FIT, y1 + DC_FIT + 1.6)):
        base = base.union(box(rear, ry0, 0, IN_L, ry1, DC_Z0 + 6.0))
    # two stops behind the corners of the jack take the push of the plug; the middle stays open for the tabs
    stop_x1 = rear - DC_STOP_GAP
    for sy0, sy1 in ((y0, y0 + 2.0), (y1 - 2.0, y1)):
        base = base.union(box(stop_x1 - 2.0, sy0, 0, stop_x1, sy1, DC_Z0 + DC_H * 0.7))
    # the window itself
    base = base.cut(box(IN_L, y0 - DC_FIT, DC_Z0 - DC_FIT, IN_L + WALL + 1, y1 + DC_FIT, DC_Z0 + DC_H + DC_FIT))

    # snap grooves for the lid ridges (long walls)
    for gy in (0.0, IN_W):
        base = base.cut(cyl_x(IN_L / 2, gy, SNAP_Z, SNAP_R + 0.2, SNAP_LEN + 4))
    return base


def make_lid():
    lid = (cq.Workplane("XY").box(IN_L + 2 * WALL, IN_W + 2 * WALL, LID, centered=False)
           .translate((-WALL, -WALL, IN_H)).edges("|Z").fillet(CORNER_R))

    # lip that drops inside the walls
    lx0, ly0, lx1, ly1 = FIT, FIT, IN_L - FIT, IN_W - FIT
    lip = box(lx0, ly0, IN_H - LIP_H, lx1, ly1, IN_H).cut(
        box(lx0 + LIP_T, ly0 + LIP_T, IN_H - LIP_H - 1, lx1 - LIP_T, ly1 - LIP_T, IN_H + 1))
    lip = lip.cut(box(lx0 - 1, RJ_Y - RJ_W / 2 - 1.5, IN_H - LIP_H - 1, lx0 + LIP_T + 1, RJ_Y + RJ_W / 2 + 1.5, IN_H))
    lid = lid.union(lip)

    # snap ridges on the lip's long sides
    for ry in (ly0, ly1):
        lid = lid.union(cyl_x(IN_L / 2, ry, SNAP_Z, SNAP_R, SNAP_LEN))

    # tab that closes the RJ45 slot above the jack
    rj_top = PCB_Z0 + PCB_T + RJ_H
    lid = lid.union(box(-WALL, RJ_Y - RJ_W / 2 - FIT + 0.15, rj_top + 0.5, 0, RJ_Y + RJ_W / 2 + FIT - 0.15, IN_H))

    for x, kind, label in ITEMS:
        if kind == "switch":
            p_in = SW_BODY + 2 * FIT
            p_out = p_in + 2 * SW_POCKET_WALL
            pocket = box(x - p_out / 2, ITEM_Y - p_out / 2, IN_H - SW_POCKET_DEPTH, x + p_out / 2, ITEM_Y + p_out / 2, IN_H)
            pocket = pocket.cut(box(x - p_in / 2, ITEM_Y - p_in / 2, IN_H - SW_POCKET_DEPTH - 1, x + p_in / 2, ITEM_Y + p_in / 2, IN_H))
            lid = lid.union(pocket)
            hole = SW_HOLE
        else:
            hole = LED_HOLE
        lid = lid.cut(cq.Workplane("XY").circle(hole / 2).extrude(LID + 10).translate((x, ITEM_Y, IN_H - 5)))

        text = (cq.Workplane("XY").workplane(offset=IN_H + LID - ENGRAVE)
                .text(label, LABEL_H, ENGRAVE + 0.5, fontPath=FONT, halign="center", valign="center")
                .translate((x, LABEL_Y, 0)))
        lid = lid.cut(text)
    return lid


def make_dummies():
    """Rough board, plugs and switches, only used for the fit check and the preview."""
    parts = {}
    parts["pcb"] = box(PCB_X0, PCB_Y0, PCB_Z0, PCB_X0 + PCB_L, PCB_Y0 + PCB_W, PCB_Z0 + PCB_T)
    rz = PCB_Z0 + PCB_T
    parts["rj45"] = box(PCB_X0 - RJ_OVERHANG, RJ_Y - RJ_W / 2, rz, PCB_X0 - RJ_OVERHANG + RJ_L, RJ_Y + RJ_W / 2, rz + RJ_H)
    parts["module"] = box(PCB_X0 + PCB_L - 18, PCB_Y0 + 1.5, rz, PCB_X0 + PCB_L, PCB_Y0 + PCB_W - 1.5, rz + 3.1)
    plugs = None
    for row_y in (PCB_Y0 + 1.27, PCB_Y0 + PCB_W - 1.27):
        p = box(PCB_X0 + 18, row_y - 1.3, PCB_Z0 - 2.5 - 14.0, PCB_X0 + PCB_L - 1, row_y + 1.3, PCB_Z0)
        plugs = p if plugs is None else plugs.union(p)
    parts["dupont"] = plugs
    sw = None
    for x, kind, _ in ITEMS:
        if kind == "switch":
            s = box(x - 3, ITEM_Y - 3, IN_H - 3.5, x + 3, ITEM_Y + 3, IN_H).union(
                box(x - 3.5, ITEM_Y - 2.5, IN_H - 7.0, x + 3.5, ITEM_Y + 2.5, IN_H - 3.5))
        else:
            s = cq.Workplane("XY").circle(2.5).extrude(9).translate((x, ITEM_Y, IN_H - 1))
        sw = s if sw is None else sw.union(s)
    parts["switches"] = sw
    parts["jack"] = box(IN_L + WALL - DC_L, DC_Y - DC_W / 2, DC_Z0, IN_L + WALL, DC_Y + DC_W / 2, DC_Z0 + DC_H)
    return parts


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    base, lid = make_base(), make_lid()
    dummies = make_dummies()

    # fit check: nothing may intersect
    problems = 0
    for pname, part in (("base", base), ("lid", lid)):
        for dname, d in dummies.items():
            v = part.intersect(d).val().Volume() if part.intersect(d).vals() else 0.0
            if v > 1e-3:
                problems += 1
                print(f"INTERFERENCE {pname} x {dname}: {v:.2f} mm3")
    v = base.intersect(lid).val().Volume() if base.intersect(lid).vals() else 0.0
    print(f"base x lid overlap (snap ridges only expected): {v:.2f} mm3")
    print("fit check:", "OK" if problems == 0 else f"{problems} problem(s)")

    bb = base.val().BoundingBox()
    print(f"outside size: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen + LID:.1f} mm (with lid)")

    # print orientation: base as is (floor down), lid flipped (top on the bed)
    base_print = base.translate((WALL, WALL, FLOOR))
    lid_print = lid.rotate((0, 0, 0), (1, 0, 0), 180).translate((WALL, IN_W + WALL, IN_H + LID))
    cq.exporters.export(base_print, os.path.join(OUT, "shiftr_case_base.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(lid_print, os.path.join(OUT, "shiftr_case_lid.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(base, os.path.join(OUT, "shiftr_case_base.step"))
    cq.exporters.export(lid, os.path.join(OUT, "shiftr_case_lid.step"))

    assy = cq.Assembly(name="shiftr_case")
    assy.add(base, name="base", color=cq.Color(0.25, 0.27, 0.30))
    assy.add(lid, name="lid", color=cq.Color(0.85, 0.45, 0.20))
    for dname, d in dummies.items():
        assy.add(d, name=f"ref_{dname}", color=cq.Color(0.2, 0.6, 0.3))
    assy.save(os.path.join(OUT, "shiftr_case_assembly.step"))
    print("exported to", OUT)
