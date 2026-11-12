"""Prueba de paneles y cargas superficiales: modelo de cálculo, robot_list_loads y reparto a vigas.

Uso (con Robot CERRADO):  .venv\\Scripts\\python.exe tests\\panel_test.py

Abre su propio Robot, crea el pórtico de make_fixture.py CON su losa (panel 100, 5 x 4 m a 3 m),
la convierte en panel de reparto de cargas (sin rigidez, como las losas de un edificio de
prefabricados), le pone 2 kN/m² en el caso 2, calcula y lo contrasta con la cuenta a mano.
Al final cierra Robot SIN GUARDAR. Se niega a empezar si ya hay un Robot abierto.
"""

from __future__ import annotations

import asyncio
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import clr  # noqa: E402
from mcp import Client, StdioServerParameters  # noqa: E402

import robot_com as rc  # noqa: E402

clr.AddReference(rc.interop_path())
import RobotOM as om  # type: ignore[import-not-found]  # noqa: E402

checks: list[tuple[bool, str]] = []


def check(ok: bool, text: str) -> None:
    checks.append((bool(ok), text))
    print(("   [OK] " if ok else "   [FALLO] ") + text)


async def call(client: Client, name: str, args: dict) -> tuple[bool, object]:
    result = await client.call_tool(name, args)
    text = "".join(getattr(block, "text", "") for block in result.content)
    try:
        return not result.is_error, json.loads(text)
    except ValueError:
        return not result.is_error, text


async def main() -> int:
    if rc.robot_process_count() != 0:
        sys.exit("Hay un Robot abierto. Ciérralo: esta prueba abre el suyo y lo cierra sin guardar.")
    app = rc.create_application(om)
    app.Visible = 1
    app.Interactive = 0
    fixture = subprocess.run([sys.executable, str(ROOT / "tests" / "make_fixture.py")], capture_output=True, text=True)
    print("   " + " | ".join(fixture.stdout.strip().splitlines()[-2:]))
    st = app.Project.Structure
    lt = om.IRobotLabelType
    if rc.robot_process_count() != 1 or not st.Objects.Exist(100):
        sys.exit("No quedó una única instancia con el pórtico y su losa; se aborta.")
    label = st.Labels.Create(lt.I_LT_PANEL_CALC_MODEL, "Reparto MCP")
    model = om.IRobotPanelCalcModelData(label.Data)
    model.FinElemType = om.IRobotPanelCalcModelFinElemType.I_PCMFET_NONE
    model.LoadTransfer = om.IRobotPanelCalcModelLoadTransfer.I_PCMLT_SIMPLIFIED_TWO_WAY
    model.Diaphragm = om.IRobotPanelCalcModelDiaphragm.I_PCMD_NONE
    st.Labels.Store(label)
    panel = om.IRobotObjObject(st.Objects.Get(100))
    panel.SetLabel(lt.I_LT_PANEL_CALC_MODEL, "Reparto MCP")
    panel.Update()
    live = om.IRobotSimpleCase(st.Cases.Get(2))
    record = live.Records.Get(live.Records.New(om.IRobotLoadRecordType.I_LRT_UNIFORM))
    record.SetValue(int(om.IRobotUniformRecordValues.I_URV_PZ), -2000.0)  # 2 kN/m²
    record.Objects.AddOne(100)
    print("   cálculo:", app.Project.CalcEngine.CalculateEx(om.IRobotCalculationMode.I_CM_LOCAL))
    while rc.robot_starting():
        time.sleep(2)

    params = StdioServerParameters(command=sys.executable, args=[str(ROOT / "server.py")], cwd=str(ROOT), env={"ROBOT_MCP_READ_ONLY": "1"})
    async with Client(params) as client:
        ok, out = await call(client, "robot_list_panels", {})
        calc = out["items"][0].get("calc_model") if ok and out["items"] else out
        check(
            ok and calc == {"name": "Reparto MCP", "finite_elements": False, "load_transfer": "SIMPLIFIED_TWO_WAY", "diaphragm": "NONE"},
            f"modelo de cálculo del panel: {calc}",
        )
        ok, out = await call(client, "robot_list_loads", {"case": 2})
        got = [(r["type"], r["objects"], r["values"].get("PZ"), r["units"]) for r in out["items"]] if ok else out
        check(got == [("BAR_UNIFORM", "9to16", -10.0, "P kN/m"), ("UNIFORM", "100", -2.0, "P kN/m²")], f"cargas del caso 2: {got}")
        check(ok and out["total"] == 2 and out["generated_count"] == 4, "Robot genera 4 cargas (una por viga bajo la losa) y se ocultan")
        ok, out = await call(client, "robot_list_loads", {"case": 2, "include_generated": True})
        generated = [(r["type"], r["objects"]) for r in out["items"] if r.get("generated")] if ok else out
        print("   generadas:", [(r["type"], r["objects"], r.get("values")) for r in out["items"] if r.get("generated")] if ok else out)
        check(len(generated) == 4 and {o for _, o in generated} == {"9", "10", "11", "12"}, f"generadas en las vigas 9-12 del nivel de la losa: {generated}")
        lengths = {"9": 5.0, "10": 4.0, "11": 5.0, "12": 4.0}  # vigas del nivel 3 m: 5, 4, 5, 4 m

        def resultant(record: dict) -> float:
            pts = record.get("points", [])
            scale = lengths[record["objects"]] if record["values"].get("RELATIVE") else 1.0
            return sum((b["x"] - a["x"]) * scale * (a["pz"] + b["pz"]) / 2 for a, b in zip(pts, pts[1:]))

        spread = sum(resultant(r) for r in out["items"] if r.get("generated")) if ok else 0.0
        check(abs(spread + 40.0) < 0.5, f"las cargas trapezoidales generadas suman {spread:.2f} kN (esperado -40 = 2 kN/m² × 20 m²)")
        ok, out = await call(client, "robot_list_load_cases", {"kind": "simple"})
        live_case = next(c for c in out["items"] if c["number"] == 2) if ok else {}
        check(live_case.get("load_records") == 2 and live_case.get("generated_records") == 4, f"load_records del caso 2: {live_case}")
        ok, out = await call(client, "robot_get_reactions", {"cases": "2", "limit": 1})
        total = out["totals"][0]["sum_fz_kn"] if ok else out
        # 10 kN/m en 8 vigas de 5, 4, 5 y 4 m (36 m) + 2 kN/m² en 5 x 4 m = 360 + 40 kN
        check(ok and abs(total - 400.0) < 0.5, f"reacción vertical del caso 2 = {total} kN (esperado 400: la losa reparte su carga a las vigas)")

    # Escritura sobre un caso con cargas generadas: la cuenta de comprobación ignora las de Robot
    writer = StdioServerParameters(command=sys.executable, args=[str(ROOT / "server.py")], cwd=str(ROOT), env={})
    async with Client(writer) as client:
        load = {"case": 2, "loads": [{"type": "node_force", "nodes": "9", "fz_kn": -1}], "dry_run": False, "discard_results": True}
        ok, out = await call(client, "robot_add_loads", load)
        check(ok and out.get("records_before") == 2 and out.get("records_after") == 3, f"carga añadida a un caso con cargas generadas: {str(out)[:200]}")
        ok, out = await call(client, "robot_list_loads", {"case": 2})
        print("   tras escribir:", out.get("total") if ok else out, "propias |", out.get("generated_count") if ok else "", "generadas")

    app.Quit(om.IRobotQuitOption.I_QO_DISCARD_CHANGES)
    app = st = None
    time.sleep(6)
    failed = [text for ok, text in checks if not ok]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} comprobaciones correctas | procesos de Robot: {rc.robot_process_count()}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
