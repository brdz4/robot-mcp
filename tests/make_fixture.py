"""Crea en el Robot ABIERTO un pórtico 3D de prueba (2 pisos, 1 vano) en un proyecto nuevo sin guardar.

Uso:  .venv\\Scripts\\python.exe tests\\make_fixture.py
ATENCIÓN: reemplaza el proyecto activo de Robot por uno nuevo. No lo ejecutes con un
modelo real abierto sin guardar.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import clr  # noqa: E402

from robot_structural_mcp import robot_com as rc  # noqa: E402

clr.AddReference(rc.interop_path())
import RobotOM as om  # type: ignore[import-not-found]  # noqa: E402

if rc.robot_process_count() == 0:
    sys.exit("Robot no está abierto.")

for attempt in range(60):  # Robot puede seguir arrancando
    try:
        app = rc.create_application(om)
        print("Robot", app.ProgramVersion)
        break
    except Exception as exc:
        print("esperando a Robot…", type(exc).__name__)
        time.sleep(3)
else:
    sys.exit("Robot no respondió por COM.")

project = app.Project
project.New(om.IRobotProjectType.I_PT_SHELL)  # SHELL admite barras y paneles
st = project.Structure
LT, OT = om.IRobotLabelType, om.IRobotObjectType
labels = st.Labels

# --- nudos: planta 5 x 4 m, niveles 0, 3 y 6 m
plan = [(0, 0), (5, 0), (5, 4), (0, 4)]
n = 0
for z in (0.0, 3.0, 6.0):
    for x, y in plan:
        n += 1
        st.Nodes.Create(n, x, y, z)

# --- barras: columnas 1-8, vigas 9-16
b = 0
for level in (0, 1):
    for i in range(4):
        b += 1
        st.Bars.Create(b, level * 4 + i + 1, (level + 1) * 4 + i + 1)
for level in (1, 2):
    for i in range(4):
        b += 1
        st.Bars.Create(b, level * 4 + i + 1, level * 4 + (i + 1) % 4 + 1)

# --- secciones de hormigón
CV = om.IRobotBarSectionConcreteDataValue


def concrete_section(name: str, shape, b_m: float, h_m: float, reduction: float | None) -> None:
    label = labels.Create(LT.I_LT_BAR_SECTION, name)
    data = om.IRobotBarSectionData(label.Data)
    data.ShapeType = shape
    conc = data.Concrete
    conc.SetValue(CV.I_BSCDV_BEAM_B, b_m)
    conc.SetValue(CV.I_BSCDV_BEAM_H, h_m)
    if reduction:
        conc.SetReduction(True, 1.0, reduction, reduction)
    data.CalcNonstdGeometry()
    labels.Store(label)


concrete_section("C 40x40", om.IRobotBarSectionShapeType.I_BSST_CONCR_COL_R, 0.40, 0.40, 0.8)
concrete_section("V 30x50", om.IRobotBarSectionShapeType.I_BSST_CONCR_BEAM_RECT, 0.30, 0.50, 0.5)

def pick(obj_type, numbers):
    """Selección por números: FromText depende del idioma de Robot ('1to8' en español = TODO)."""
    sel = st.Selections.Create(obj_type)
    for number in numbers:
        sel.AddOne(number)
    return sel


st.Bars.SetLabel(pick(OT.I_OT_BAR, range(1, 9)), LT.I_LT_BAR_SECTION, "C 40x40")
st.Bars.SetLabel(pick(OT.I_OT_BAR, range(9, 17)), LT.I_LT_BAR_SECTION, "V 30x50")

# --- apoyo empotrado en la base
sup = labels.Create(LT.I_LT_SUPPORT, "Empotrado MCP")
sd = om.IRobotNodeSupportData(sup.Data)
sd.UX = sd.UY = sd.UZ = sd.RX = sd.RY = sd.RZ = 1
labels.Store(sup)
st.Nodes.SetLabel(pick(OT.I_OT_NODE, range(1, 5)), LT.I_LT_SUPPORT, "Empotrado MCP")

# --- pisos
st.Storeys.Create("Piso 1", 0.0, 3.0)
st.Storeys.Create("Piso 2", 3.0, 3.0)

# --- casos y combinación
CN, CAT = om.IRobotCaseNature, om.IRobotCaseAnalizeType
pp = st.Cases.CreateSimple(1, "PP", CN.I_CN_PERMANENT, CAT.I_CAT_STATIC_LINEAR)
rec = om.IRobotLoadRecord(pp.Records.Create(om.IRobotLoadRecordType.I_LRT_DEAD))
rec.SetValue(int(om.IRobotDeadRecordValues.I_DRV_Z), -1.0)
rec.SetValue(int(om.IRobotDeadRecordValues.I_DRV_ENTIRE_STRUCTURE), 1.0)
cv = st.Cases.CreateSimple(2, "CV", CN.I_CN_EXPLOATATION, CAT.I_CAT_STATIC_LINEAR)
rec = om.IRobotLoadRecord(cv.Records.Create(om.IRobotLoadRecordType.I_LRT_BAR_UNIFORM))
rec.SetValue(int(om.IRobotBarUniformRecordValues.I_BURV_PZ), -10000.0)  # 10 kN/m
for bar_number in range(9, 17):
    rec.Objects.AddOne(bar_number)
combo = st.Cases.CreateCombination(
    3, "1.2PP+1.6CV", om.IRobotCombinationType.I_CBT_ULS, CN.I_CN_PERMANENT, CAT.I_CAT_COMB
)
combo.CaseFactors.New(1, 1.2)
combo.CaseFactors.New(2, 1.6)

# --- panel (losa) en el nivel 3 m; opcional, no bloquea el resto.
#     Con --no-panel queda un pórtico puro, que es el que se usa para calcular (state_test.py).
try:
    if "--no-panel" in sys.argv:
        raise RuntimeError("pedido con --no-panel")
    th = labels.Create(LT.I_LT_PANEL_THICKNESS, "Losa 20")
    td = om.IRobotThicknessData(th.Data)
    td.ThicknessType = om.IRobotThicknessType.I_TT_HOMOGENEOUS
    om.IRobotThicknessHomoData(td.Data).ThickConst = 0.20
    # sin material, una losa de elementos finitos no pasa la verificación del cálculo
    td.MaterialName = om.IRobotBarSectionData(labels.Get(LT.I_LT_BAR_SECTION, "C 40x40").Data).MaterialName
    labels.Store(th)
    pts = om.IRobotPointsArray(app.CmpntFactory.Create(om.IRobotComponentType.I_CT_POINTS_ARRAY))
    pts.SetSize(4)
    for i, (x, y) in enumerate(plan, start=1):
        pts.Set(i, x, y, 3.0)
    st.Objects.CreateContour(100, pts)
    panel = om.IRobotObjObject(st.Objects.Get(100))
    panel.Main.Attribs.Meshed = 1
    panel.SetLabel(LT.I_LT_PANEL_THICKNESS, "Losa 20")
    panel.Initialize()
    panel.Update()
    print("panel 100 creado")
except Exception as exc:
    print("panel omitido:", type(exc).__name__, exc)

print("fixture listo: 12 nudos, 16 barras, 3 casos, 2 pisos")
