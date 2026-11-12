"""Prueba de las herramientas de combinaciones NEC, flechas, paneles, cuantías ACI y borrado.

Uso (con Robot CERRADO):  .venv\\Scripts\\python.exe tests\\design_test.py

Abre su propio Robot, crea el pórtico de make_fixture.py con su losa (panel 100, elementos
finitos), calcula y usa el servidor (con escritura) como lo haría un chat. Cada resultado se
contrasta con otra herramienta o con la cuenta a mano. Al final cierra Robot SIN GUARDAR.
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

import design  # noqa: E402
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


async def main() -> int:  # noqa: C901
    if rc.robot_process_count() != 0:
        sys.exit("Hay un Robot abierto. Ciérralo: esta prueba abre el suyo y lo cierra sin guardar.")
    app = rc.create_application(om)
    app.Visible = 1
    app.Interactive = 0
    fixture = subprocess.run([sys.executable, str(ROOT / "tests" / "make_fixture.py")], capture_output=True, text=True)
    print("   " + " | ".join(fixture.stdout.strip().splitlines()[-2:]))
    if rc.robot_process_count() != 1 or not app.Project.Structure.Objects.Exist(100):
        sys.exit("No quedó una única instancia con el pórtico y su losa; se aborta.")
    while rc.robot_starting():
        time.sleep(2)

    params = StdioServerParameters(command=sys.executable, args=[str(ROOT / "server.py")], cwd=str(ROOT), env={})
    async with Client(params) as client:
        ok, out = await call(client, "robot_run_calculation", {})
        check(ok and out["calculation"] == "COMPLETED", f"cálculo del pórtico con losa: {out}")

        print("== combinaciones NEC")
        args = {"dead": [1], "live": [2], "seismic_x": [], "prefix": "T"}
        ok, out = await call(client, "robot_create_nec_combinations", args)
        names = [c["name"] for c in out["to_create"]] if ok else out
        check(names == ["T1 1.4D", "T2 1.2D+1.6L", "T3 D+L"] and out["applied"] is False, f"ensayo: {names}")
        ok, out = await call(client, "robot_create_nec_combinations", {**args, "dry_run": False})
        check(ok and out.get("created") == 3 and out["results_status_after"] == "AVAILABLE", "3 combinaciones creadas sin perder resultados")
        numbers = {c["name"]: c["number"] for c in out["to_create"]} if ok else {}
        ultimate, service = f'{numbers.get("T1 1.4D")} {numbers.get("T2 1.2D+1.6L")}', numbers.get("T3 D+L")
        ok, out = await call(client, "robot_list_combinations", {})
        made = {c["name"]: {f["case"]: f["factor"] for f in c["factors"]} for c in out["items"]} if ok else {}
        check(made.get("T2 1.2D+1.6L") == {1: 1.2, 2: 1.6} and made.get("T3 D+L") == {1: 1.0, 2: 1.0}, "factores releídos de Robot")
        ok, out = await call(client, "robot_create_nec_combinations", {**args, "dry_run": False})
        check(not ok and "Ya existe" in str(out), "repetir con el mismo prefijo: rechazado")

        print("== flechas")
        ok, out = await call(client, "robot_get_bar_deflections", {"cases": f"1 2 {service}", "selection": "9", "limit_ratio": 240})
        rows = {r["case"]: r for r in out["items"][0]["by_case"]} if ok else {}
        print("   ", rows)
        total = rows.get(service, {}).get("uz_mm", 0)
        parts = rows.get(1, {}).get("uz_mm", 0) + rows.get(2, {}).get("uz_mm", 0)
        check(total > 0 and abs(total - parts) < 0.02 * total, f"flecha D+L = D + L ({total} ≈ {parts:.3f} mm)")
        check(0.3 < rows.get(service, {}).get("x_uz", 0) < 0.7 and rows[service]["passes"] is True, "máximo hacia el centro del vano y cumple L/240")

        print("== esfuerzos en paneles")
        ok, out = await call(client, "robot_get_panel_forces", {"cases": "1", "selection": "100", "components": ["MXX", "MYY", "QXX"]})
        print("   ", str(out)[:400])
        values = {e["component"]: e for e in out["extremes"]} if ok else {}
        check(ok and values["MXX"]["max"] and values["MXX"]["max"]["node"] > 0, "MXX del panel 100 por peso propio, con su nudo")
        check(ok and values["MXX"]["min"]["value"] < 0 < values["MXX"]["max"]["value"], "momentos de losa con ambos signos (vano y apoyo)")

        print("== cuantías ACI")
        ok, out = await call(client, "robot_check_rc_members", {"cases": ultimate, "fc_mpa": 28, "fy_mpa": 420, "selection": "1 9"})
        print("   ", str(out)[:700])
        items = {i["bar"]: i for i in out["items"]} if ok else {}
        check(ok and out["summary"]["checked"] == 2 and not out["summary"]["skipped"], "una columna y una viga revisadas")
        ok_f, forces = await call(client, "robot_get_bar_forces", {"cases": ultimate, "selection": "9", "points": 5})
        my = [st["my_knm"] for case in forces["items"][0]["by_case"] for st in case["stations"]] if ok_f else [0]
        steel = max(design.beam_steel(max(my) * 1e6, 300, 440, 28, 420), max(0.25 * 28**0.5, 1.4) / 420 * 300 * 440) / 100
        beam = items.get(9, {}).get("flexure", {})
        check(abs(beam.get("as_bottom_cm2", 0) - steel) < 0.02, f"As inferior de la viga 9 = {beam.get('as_bottom_cm2')} cm² (a mano con MY de robot_get_bar_forces: {steel:.2f})")
        column = items.get(1, {})
        check(column.get("rho_required") == 0.01 and column.get("ok"), f"columna 1 con la cuantía mínima: {column.get('rho_required')} ({column.get('governs')})")
        ok, out = await call(client, "robot_check_rc_members", {"cases": "1 999", "fc_mpa": 28, "fy_mpa": 420})
        check(not ok and "999" in str(out), "caso inexistente: rechazado")

        print("== panel nuevo y carga superficial")
        square = [{"x": x, "y": y, "z": 6.0} for x, y in ((0, 0), (5, 0), (5, 4), (0, 4))]
        ok, out = await call(client, "robot_create_panel", {"points": square, "thickness": "Losa 20"})
        check(ok and out["panel"]["area_m2"] == 20.0 and out["invalidates_results"], f"ensayo del panel: {out.get('panel') if ok else out}")
        bent = [dict(p) for p in square]
        bent[2]["z"] = 6.1
        ok, out = await call(client, "robot_create_panel", {"points": bent, "thickness": "Losa 20"})
        check(not ok and "coplanares" in str(out), "contorno no plano: rechazado")
        ok, out = await call(client, "robot_create_panel", {"points": square, "thickness": "Losa 20", "calc_model": "no existe"})
        check(not ok and "modelo de cálculo" in str(out), "modelo de cálculo inexistente: rechazado")
        ok, out = await call(client, "robot_create_panel", {"points": square, "thickness": "Losa 20", "dry_run": False})
        check(not ok and "discard_results" in str(out), "sin discard_results no toca un modelo calculado")
        ok, out = await call(client, "robot_create_panel", {"points": square, "thickness": "Losa 20", "dry_run": False, "discard_results": True})
        check(ok and out["panel_after"]["thickness"] == "Losa 20", f"panel creado: {out.get('panel_after') if ok else out}")
        panel = out["panel"]["number"] if ok else 0
        load = {"case": 2, "loads": [{"type": "panel_uniform", "panels": str(panel), "pz_kn_m2": -3}], "dry_run": False}
        ok, out = await call(client, "robot_add_loads", load)
        check(ok and out.get("records_after") == 2, f"carga de 3 kN/m² en el panel {panel}: {str(out)[:200]}")
        ok, out = await call(client, "robot_list_loads", {"case": 2})
        last = out["items"][-1] if ok else {}
        check(last.get("type") == "UNIFORM" and last.get("values", {}).get("PZ") == -3.0 and last.get("objects") == str(panel), f"releída: {last}")
        ok, out = await call(client, "robot_run_calculation", {})
        ok, out = await call(client, "robot_get_reactions", {"cases": "2", "limit": 1})
        check(ok and abs(out["totals"][0]["sum_fz_kn"] - 420.0) < 0.5, f"reacción del caso 2 = {out['totals'][0]['sum_fz_kn'] if ok else out} kN (360 + 3·20 = 420)")

        print("== borrado")
        ok, out = await call(client, "robot_delete_loads", {"case": 2, "indexes": [last.get("index", 0)], "dry_run": False})
        check(not ok and "discard_results" in str(out), "borrar una carga con resultados vigentes exige discard_results")
        ok, out = await call(client, "robot_delete_loads", {"case": 2, "indexes": [last.get("index", 0)], "dry_run": False, "discard_results": True})
        check(ok and out.get("records_after") == 1 and out["to_delete"][0]["type"] == "UNIFORM", f"carga del panel borrada: {str(out)[:200]}")
        ok, out = await call(client, "robot_delete_loads", {"case": 2, "indexes": [9]})
        check(not ok and "no tiene las cargas" in str(out), "índice inexistente: rechazado")
        ok, out = await call(client, "robot_delete_elements", {"kind": "panels", "selection": str(panel), "dry_run": False, "discard_results": True})
        check(ok and out.get("deleted") == 1, "panel nuevo borrado")
        ok, out = await call(client, "robot_list_panels", {})
        check(ok and [p["number"] for p in out["items"]] == [100], "solo queda la losa original")
        ok, out = await call(client, "robot_delete_elements", {"kind": "cases", "selection": "1"})
        check(not ok and "usa los casos [1]" in str(out), "caso usado por una combinación: rechazado")
        ok, out = await call(client, "robot_delete_elements", {"kind": "nodes", "selection": "1"})
        check(not ok and "barras" in str(out), "nudo usado por barras: rechazado")
        ok, out = await call(client, "robot_delete_elements", {"kind": "cases", "selection": " ".join(str(n) for n in numbers.values())})
        check(ok and out["invalidates_results"] is False and len(out["cases"]) == 3, "borrar solo combinaciones no invalida resultados")
        ok, out = await call(client, "robot_delete_elements", {"kind": "cases", "selection": " ".join(str(n) for n in numbers.values()), "dry_run": False})
        check(ok and out.get("deleted") == 3, "3 combinaciones borradas")
        ok, out = await call(client, "robot_list_combinations", {})
        check(ok and [c["name"] for c in out["items"]] == ["1.2PP+1.6CV"], "queda solo la combinación del pórtico")
        ok, out = await call(client, "robot_create_nodes", {"nodes": [{"x": 9.0, "y": 9.0, "z": 0.0}], "dry_run": False, "discard_results": True})
        free = out["to_create"][0]["number"] if ok else 0
        ok, out = await call(client, "robot_delete_elements", {"kind": "nodes", "selection": str(free), "dry_run": False, "discard_results": True})
        check(ok and out.get("deleted") == 1, f"nudo suelto {free} borrado")
        ok, out = await call(client, "robot_delete_elements", {"kind": "bars", "selection": "16", "dry_run": False, "discard_results": True})
        ok2, info = await call(client, "robot_get_project_info", {})
        check(ok and out.get("deleted") == 1 and info["counts"]["bars"] == 15, "barra 16 borrada; quedan 15")

    app.Quit(om.IRobotQuitOption.I_QO_DISCARD_CHANGES)
    app = None
    time.sleep(6)
    failed = [text for ok, text in checks if not ok]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} comprobaciones correctas | procesos de Robot: {rc.robot_process_count()}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
