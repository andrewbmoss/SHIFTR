"""Offscreen VTK renders of the case for review."""
import os
import sys
import vtk
import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shiftr_case as sc

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def poly(shape, tol=0.05):
    path = os.path.join("/tmp/claude-0/-home-claude/88291094-6de5-565b-b885-f685fbce28d2/scratchpad", "r.stl")
    cq.exporters.export(shape, path, tolerance=tol, angularTolerance=0.15)
    r = vtk.vtkSTLReader()
    r.SetFileName(path)
    r.Update()
    pd = vtk.vtkPolyData()
    pd.DeepCopy(r.GetOutput())
    return pd


def actor(pd, color, opacity=1.0, edges=False):
    n = vtk.vtkPolyDataNormals()
    n.SetInputData(pd)
    n.SetFeatureAngle(30)
    m = vtk.vtkPolyDataMapper()
    m.SetInputConnection(n.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(m)
    a.GetProperty().SetColor(*color)
    a.GetProperty().SetOpacity(opacity)
    pr = a.GetProperty()
    pr.SetAmbient(0.35)
    pr.SetDiffuse(0.75)
    pr.SetSpecular(0.15)
    return a


def edge_actor(pd):
    fe = vtk.vtkFeatureEdges()
    fe.SetInputData(pd)
    fe.BoundaryEdgesOn()
    fe.FeatureEdgesOn()
    fe.SetFeatureAngle(35)
    fe.ManifoldEdgesOff()
    fe.NonManifoldEdgesOff()
    m = vtk.vtkPolyDataMapper()
    m.SetInputConnection(fe.GetOutputPort())
    m.ScalarVisibilityOff()
    a = vtk.vtkActor()
    a.SetMapper(m)
    a.GetProperty().SetColor(0.1, 0.1, 0.1)
    a.GetProperty().SetLineWidth(1.2)
    return a


def render(actors, name, pos, focal, up=(0, 0, 1), size=(1400, 1000), parallel=None):
    ren = vtk.vtkRenderer()
    ren.SetBackground(1, 1, 1)
    for a in actors:
        ren.AddActor(a)
    cam = ren.GetActiveCamera()
    cam.SetPosition(*pos)
    cam.SetFocalPoint(*focal)
    cam.SetViewUp(*up)
    if parallel:
        cam.ParallelProjectionOn()
        cam.SetParallelScale(parallel)
    else:
        ren.ResetCamera()
        cam.Zoom(1.25)
    ren.ResetCameraClippingRange()
    kit = vtk.vtkLightKit()
    kit.AddLightsToRenderer(ren)
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.AddRenderer(ren)
    win.SetSize(*size)
    win.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(win)
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(os.path.join(OUT, name))
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()


base, lid = sc.make_base(), sc.make_lid()
d = sc.make_dummies()
bp, lp = poly(base), poly(lid, 0.02)
ref = [poly(d[k]) for k in ("pcb", "rj45", "module", "dupont", "switches", "cable")]
grey, orange, green = (0.55, 0.58, 0.62), (0.93, 0.55, 0.28), (0.25, 0.65, 0.4)
c = (sc.IN_L / 2, sc.IN_W / 2, sc.IN_H / 2)

grey, orange = (0.78, 0.80, 0.83), (0.95, 0.60, 0.30)
# 1. lid top, labels readable
render([actor(lp, orange), edge_actor(lp)], "view_lid_top.png", (c[0], c[1], 300), (c[0], c[1], 0), up=(0, 1, 0), parallel=33)

# 2. exploded: lid lifted, board and plugs inside the base
lift = vtk.vtkTransform()
lift.Translate(0, 0, 38)
def moved(pd, t):
    f = vtk.vtkTransformPolyDataFilter()
    f.SetInputData(pd)
    f.SetTransform(t)
    f.Update()
    o = vtk.vtkPolyData()
    o.DeepCopy(f.GetOutput())
    return o
lp_up = moved(lp, lift)
sw_up = moved(ref[4], lift)
render([actor(bp, grey), edge_actor(bp), actor(lp_up, orange), edge_actor(lp_up), actor(sw_up, (0.2, 0.2, 0.2))]
       + [actor(r, green) for r in ref[:4]] + [actor(ref[5], (0.15, 0.15, 0.15))],
       "view_exploded.png", (c[0] - 150, c[1] - 190, 190), (c[0], c[1], 35))

# 3. lid underside
render([actor(lp, orange), edge_actor(lp), actor(ref[4], (0.2, 0.2, 0.2))], "view_lid_under.png",
       (c[0] + 60, c[1] - 120, -150), (c[0], c[1], sc.IN_H), up=(0, 0, 1))

# 4. section through the FUNCTION switch, lid on
cut_x = 33.0
plane = vtk.vtkPlane()
plane.SetOrigin(cut_x, 0, 0)
plane.SetNormal(1, 0, 0)
acts = []
for pd, col in [(bp, grey), (lp, orange)] + [(r, green) for r in ref[:4]] + [(ref[4], (0.2, 0.2, 0.2))]:
    cl = vtk.vtkClipPolyData()
    cl.SetInputData(pd)
    cl.SetClipFunction(plane)
    cl.Update()
    out = vtk.vtkPolyData()
    out.DeepCopy(cl.GetOutput())
    acts += [actor(out, col), edge_actor(out)]
render(acts, "view_section.png", (cut_x - 250, c[1], c[2]), (cut_x, c[1], c[2]), up=(0, 0, 1), parallel=30)

# 5. both ends
render([actor(bp, grey), edge_actor(bp), actor(lp, orange), edge_actor(lp), actor(ref[1], green)],
       "view_rj45_end.png", (-200, c[1] - 140, 110), (0, c[1], 22))
render([actor(bp, grey), edge_actor(bp), actor(lp, orange), edge_actor(lp), actor(ref[5], (0.15, 0.15, 0.15))],
       "view_cable_end.png", (sc.IN_L + 200, c[1] + 140, 110), (sc.IN_L, c[1], 22))
print("rendered")
