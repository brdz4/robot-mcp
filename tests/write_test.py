"""Prueba de escritura de extremo a extremo, solo con las herramientas MCP:
construir un pórtico de 2 pisos, cargarlo, combinarlo, calcularlo y contrastarlo a mano.

Uso (con Robot CERRADO):  .venv\\Scripts\\python.exe tests\\write_test.py

Abre su propio Robot en modo INTERACTIVO (como el de un usuario) para comprobar que el
servidor escribe sin que Robot pregunte nada, y al final lo cierra SIN GUARDAR. Se niega a
empezar si ya hay un Robot abierto.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import clr  # noqa: E402
from mcp import Client, StdioServerParameters  # noqa: E402

from robot_structural_mcp import robot_com as rc  # noqa: E402
from dialog_watch import DialogWatch  # noqa: E402

clr.AddReference(rc.interop_path())
import RobotOM as om  # type: ignore[import-not-found]  # noqa: E402

PLAN = [(0.0, 0.0), (5.0, 0.0), (5.0, 4.0), (0.0, 4.0)]
CONCRETE_M3 = 8 * 0.4 * 0.4 * 3.0 + 36.0 * 0.3 * 0.5  # 8 columnas de 3 m + 36 m de viga = 9.24 m³
checks: list[tuple[bool, str]] = []


def check(ok: bool, text: str) -> bool:
    checks.append((bool(ok), text))
    print(("   [OK] " if ok else "   [FALLO] ") + text)
    return bool(ok)


async def call(client: Client, name: str, args: dict | None = None) -> tuple[bool, object]:
    result = await client.call_tool(name, args or {})
    text = "".join(getattr(block, "text", "") for block in result.content)
    try:
        return not result.is_error, json.loads(text)
    except ValueError:
        return not result.is_error, text


async def main() -> int:  # noqa: C901
    if rc.robot_process_count() != 0:
        sys.exit("Hay un Robot abierto. Ciérralo: esta prueba abre el suyo y lo cierra sin guardar.")
    watch = DialogWatch(rc.robot_pids)
    watch.__enter__()
    app = rc.create_application(om)
    app.Visible = 1
    app.Interactive = 1  # como el Robot de un usuario: si el servidor no lo gestiona, Robot preguntará
    app.Project.New(om.IRobotProjectType.I_PT_FRAME_3D)
    while rc.robot_starting():
        time.sleep(2)
    saved = os.path.join(tempfile.gettempdir(), "robot_mcp_write_test.rtd")
    for leftover in (saved, saved + ".bak"):
        if os.path.exists(leftover):
            os.remove(leftover)

    params = StdioServerParameters(command=sys.executable, args=["-m", "robot_structural_mcp.server"], cwd=str(ROOT))
    async with Client(params) as client:
        tools = {tool.name: tool for tool in (await client.list_tools()).tools}
        writers = [n for n, t in tools.items() if not getattr(t.annotations, "read_only_hint", True)]
        print(f"== {len(tools)} herramientas, {len(writers)} de escritura")
        check(len(writers) == 15, "15 herramientas marcadas como no de solo lectura")

        print("== 1. nudos")
        nodes = [{"x": x, "y": y, "z": z} for z in (0.0, 3.0, 6.0) for x, y in PLAN]
        ok, out = await call(client, "robot_create_nodes", {"nodes": nodes})
        check(ok and out["applied"] is False and len(out["to_create"]) == 12, "ensayo: planifica 12 nudos sin aplicar")
        ok, info = await call(client, "robot_get_project_info")
        check(ok and info["counts"]["nodes"] == 0, "tras el ensayo el modelo sigue vacío")
        ok, out = await call(client, "robot_create_nodes", {"nodes": nodes, "dry_run": False})
        check(ok and out.get("created") == 12 and [n["number"] for n in out["to_create"]] == list(range(1, 13)), "aplica: 12 nudos numerados 1-12")
        ok, out = await call(client, "robot_create_nodes", {"nodes": [{"x": 5.0, "y": 4.0, "z": 3.0}, {"x": 2.5, "y": 2.0, "z": 6.0}], "dry_run": False})
        print("   ", str(out)[:260])
        check(ok and out["already_existing"][0]["number"] == 7 and out["to_create"][0]["number"] == 13, "nudo coincidente no se duplica; el nuevo recibe el 13")
        ok, out = await call(client, "robot_create_nodes", {"nodes": [{"x": 9.0, "y": 9.0, "z": 9.0, "number": 1}], "dry_run": False})
        check(not ok and "Ya existen" in str(out), "número de nudo existente: rechazado")
        ok, out = await call(client, "robot_list_nodes", {"selection": "1"})
        check(ok and (out["items"][0]["x"], out["items"][0]["z"]) == (0.0, 0.0), "el nudo 1 conserva sus coordenadas")

        print("== 2. secciones")
        ok, out = await call(client, "robot_define_concrete_section", {"name": "C40x40", "kind": "column", "b_cm": 40, "h_cm": 40, "iy_factor": 0.8, "iz_factor": 0.8, "dry_run": False})
        print("   ", str(out)[:330])
        check(ok and out["section_after"]["iy_stored_over_gross"] == 0.8, "columna 40x40 con inercia agrietada 0.8 guardada")
        ok, out = await call(client, "robot_define_concrete_section", {"name": "V30x50", "kind": "beam", "b_cm": 30, "h_cm": 50, "iy_factor": 0.5, "iz_factor": 0.5, "dry_run": False})
        check(ok and out["section_after"]["iy_stored_over_gross"] == 0.5 and out["section_after"]["area_cm2"] == 1500.0, "viga 30x50 con inercia agrietada 0.5 guardada")
        material = out["section_after"]["material"] if ok else None
        ok, out = await call(client, "robot_define_concrete_section", {"name": "V30x50", "kind": "beam", "b_cm": 30, "h_cm": 60, "dry_run": False})
        check(not ok and "ya existe" in str(out), "redefinir sin overwrite: rechazado")
        ok, out = await call(client, "robot_define_concrete_section", {"name": "X", "kind": "beam", "b_cm": 30, "h_cm": 60, "material": "no_existe", "dry_run": False})
        check(not ok and "material" in str(out), "material inexistente: rechazado")

        print("== 3. barras")
        columns = [{"start_node": level * 4 + i + 1, "end_node": (level + 1) * 4 + i + 1, "section": "C40x40"} for level in (0, 1) for i in range(4)]
        beams = [{"start_node": level * 4 + i + 1, "end_node": level * 4 + (i + 1) % 4 + 1, "section": "V30x50"} for level in (1, 2) for i in range(4)]
        ok, out = await call(client, "robot_create_bars", {"bars": columns + beams, "dry_run": False})
        print("   ", str(out)[:260])
        check(ok and out.get("created") == 16 and out["to_create"][0]["length_m"] == 3.0, "16 barras creadas con su sección")
        ok, out = await call(client, "robot_create_bars", {"bars": [{"start_node": 5, "end_node": 1}], "dry_run": False})
        check(not ok and "ya existe" in str(out), "barra duplicada (nudos invertidos): rechazada")
        ok, out = await call(client, "robot_create_bars", {"bars": [{"start_node": 1, "end_node": 999}], "dry_run": False})
        check(not ok and "No existen los nudos" in str(out), "barra con nudo inexistente: rechazada")
        ok, out = await call(client, "robot_create_bars", {"bars": [{"start_node": 1, "end_node": 3, "section": "nada"}], "dry_run": False})
        check(not ok and "sección" in str(out), "barra con sección inexistente: rechazada")
        ok, out = await call(client, "robot_list_sections")
        counts = {item["name"]: item["bars_count"] for item in out["items"]} if ok else {}
        check(counts == {"C40x40": 8, "V30x50": 8}, f"secciones en uso: {counts}")

        print("== 4. apoyos")
        ok, out = await call(client, "robot_define_support", {"name": "Empotrado MCP", "dry_run": False})
        check(ok and out["applied"], "apoyo empotrado definido")
        ok, out = await call(client, "robot_assign_support", {"selection": "1to4", "support": "Empotrado MCP"})
        check(ok and out["applied"] is False and out["nodes_changing"] == 4, "ensayo de asignación: 4 nudos cambiarían")
        ok, out = await call(client, "robot_assign_support", {"selection": "1to4", "support": "Empotrado MCP", "dry_run": False})
        check(ok and out.get("assigned") == 4, "apoyo asignado a los nudos 1-4")
        ok, out = await call(client, "robot_assign_support", {"selection": "", "support": "Empotrado MCP", "dry_run": False})
        check(not ok and "explícitamente" in str(out), "selección vacía en escritura: rechazada")
        ok, out = await call(client, "robot_list_supports")
        check(ok and out["items"][0]["nodes"] == "1to4", "robot_list_supports confirma 1to4")

        print("== 5. casos, cargas y combinación")
        ok, out = await call(client, "robot_create_load_case", {"name": "PP", "nature": "PERMANENT", "dry_run": False})
        check(ok and out["case"]["number"] == 1, "caso 1 PP creado")
        ok, out = await call(client, "robot_create_load_case", {"name": "CV", "nature": "LIVE", "dry_run": False})
        check(ok and out["case"]["number"] == 2, "caso 2 CV creado")
        ok, out = await call(client, "robot_create_load_case", {"name": "pp", "nature": "PERMANENT", "dry_run": False})
        check(not ok and "Ya existe un caso llamado" in str(out), "nombre de caso repetido: rechazado")
        ok, out = await call(client, "robot_add_loads", {"case": 1, "loads": [{"type": "self_weight"}], "dry_run": False})
        check(ok and out.get("records_after") == 1, "peso propio añadido al caso 1")
        ok, out = await call(client, "robot_add_loads", {"case": 2, "loads": [{"type": "bar_uniform", "bars": "9to16", "pz_kn_m": -10}], "dry_run": False})
        print("   ", str(out)[:300])
        check(ok and out["loads"][0]["objects_count"] == 8 and out["loads"][0]["objects"] == "9to16", "10 kN/m en las 8 vigas")
        ok, out = await call(client, "robot_add_loads", {"case": 2, "loads": [{"type": "node_force", "nodes": "9"}], "dry_run": False})
        check(not ok and "cero" in str(out), "carga sin valores: rechazada")
        ok, out = await call(client, "robot_add_loads", {"case": 2, "loads": [{"type": "bar_uniform", "bars": "500to510", "pz_kn_m": -1}], "dry_run": False})
        check(not ok and "existe" in str(out), "carga sobre barras inexistentes: rechazada")
        combo = {"name": "1.2PP+1.6CV", "factors": [{"case": 1, "factor": 1.2}, {"case": 2, "factor": 1.6}]}
        ok, out = await call(client, "robot_create_combinations", {"combinations": [combo], "dry_run": False})
        check(ok and out.get("created") == 1 and out["to_create"][0]["expression"] == "1.2·PP + 1.6·CV", "combinación 3 creada")
        ok, out = await call(client, "robot_create_combinations", {"combinations": [{"name": "mala", "factors": [{"case": 77, "factor": 1.0}]}], "dry_run": False})
        check(not ok and "no existen" in str(out), "combinación con caso inexistente: rechazada")

        print("== 6. cálculo y contraste a mano")
        ok, out = await call(client, "robot_run_calculation", {"timeout_s": 120})
        print("   ", out)
        check(ok and out["calculation"] == "COMPLETED" and out["results_status"] == "AVAILABLE", "cálculo completado")
        ok, materials = await call(client, "robot_list_materials")
        unit_weight = next(m["unit_weight_kn_m3"] for m in materials["items"] if m["name"] == material)
        weight, live = CONCRETE_M3 * unit_weight, 36.0 * 10.0
        ok, out = await call(client, "robot_get_reactions", {"cases": "1 2 3"})
        totals = {t["case"]: t["sum_fz_kn"] for t in out["totals"]} if ok else {}
        print("   totales:", totals)
        check(ok and abs(totals[1] - weight) < 0.01 * weight, f"peso propio = {weight:.1f} kN a mano")
        check(ok and abs(totals[2] - live) < 0.01 * live, f"carga viva = {live:.1f} kN a mano")
        check(ok and abs(totals[3] - (1.2 * weight + 1.6 * live)) < 0.02 * weight, "combinación = 1.2·PP + 1.6·CV")

        print("== 7. protección de resultados vigentes")
        ok, out = await call(client, "robot_create_nodes", {"nodes": [{"x": 1.0, "y": 1.0, "z": 6.0}]})
        check(ok and out["invalidates_results"] is True and "discard_results=true" in out["next_step"], "el ensayo avisa de que invalidaría resultados")
        ok, out = await call(client, "robot_create_nodes", {"nodes": [{"x": 1.0, "y": 1.0, "z": 6.0}], "dry_run": False})
        check(not ok and "resultados vigentes" in str(out), "aplicar sin discard_results: rechazado")
        ok, info = await call(client, "robot_get_project_info")
        check(ok and info["results_status"] == "AVAILABLE" and info["counts"]["nodes"] == 13, "el modelo y sus resultados siguen intactos")
        ok, out = await call(client, "robot_create_combinations", {"combinations": [{"name": "Servicio", "type": "SLS", "factors": [{"case": 1, "factor": 1.0}, {"case": 2, "factor": 1.0}]}], "dry_run": False})
        check(ok and out["results_status_after"] == "AVAILABLE", "crear una combinación no invalida resultados")
        ok, out = await call(client, "robot_get_reactions", {"cases": "4"})
        check(ok and abs(out["totals"][0]["sum_fz_kn"] - (weight + live)) < 0.01 * (weight + live), "la combinación nueva ya tiene resultados sin recalcular")
        ok, out = await call(client, "robot_define_concrete_section", {"name": "V25x40", "kind": "beam", "b_cm": 25, "h_cm": 40, "iy_factor": 0.5, "iz_factor": 0.5, "dry_run": False})
        check(ok and out["results_status_after"] == "AVAILABLE", "definir una sección sin usar no invalida resultados")
        ok, out = await call(client, "robot_assign_section", {"selection": "9to12", "section": "V25x40"})
        print("   ", str(out)[:300])
        check(ok and out["current_sections"] == {"V30x50": 4} and out["invalidates_results"], "ensayo de reasignación: muestra la sección actual y avisa")
        t0 = time.time()
        ok, out = await call(client, "robot_assign_section", {"selection": "9to12", "section": "V25x40", "dry_run": False, "discard_results": True})
        check(ok and out.get("assigned") == 4 and out["results_status_after"] == "OUT_OF_DATE", f"con discard_results=true se aplica y queda desactualizado ({time.time() - t0:.1f} s)")
        check(bool(app.Interactive), "Robot vuelve a quedar interactivo tras escribir")
        ok, out = await call(client, "robot_list_bars", {"section": "V25x40"})
        check(ok and out["total"] == 4, "4 vigas con la sección nueva")

        print("== 7b. redefinir una sección en uso, fuerza en nudos y recálculo")
        ok, out = await call(client, "robot_define_concrete_section", {"name": "V30x50", "kind": "beam", "b_cm": 30, "h_cm": 50, "iy_factor": 0.35, "iz_factor": 0.35, "overwrite": True})
        check(ok and out["action"] == "redefine" and out["bars_affected"] == 4 and out["current"]["inertia_reduction"]["iy"] == 0.5, "ensayo de redefinición: 4 barras afectadas y muestra el valor actual")
        ok, out = await call(client, "robot_define_concrete_section", {"name": "V30x50", "kind": "beam", "b_cm": 30, "h_cm": 50, "iy_factor": 0.35, "iz_factor": 0.35, "overwrite": True, "dry_run": False})
        check(ok and out["section_after"]["iy_stored_over_gross"] == 0.35 and out["section_after"]["bars_count"] == 4, "sección en uso redefinida a 0.35")
        ok, out = await call(client, "robot_add_loads", {"case": 2, "loads": [{"type": "node_force", "nodes": "9 10", "fx_kn": 5}], "dry_run": False})
        check(ok and out.get("records_after") == 2 and out["loads"][0]["objects_count"] == 2, "fuerza de 5 kN en X en dos nudos")
        ok, out = await call(client, "robot_list_loads", {"case": 2})
        got = [(r["type"], r["objects_count"], r["values"].get("PZ"), r["values"].get("FX")) for r in out["items"]] if ok else out
        check(got == [("BAR_UNIFORM", 8, -10.0, None), ("NODE_FORCE", 2, None, 5.0)], f"robot_list_loads relee lo escrito: {got}")
        ok, out = await call(client, "robot_list_loads", {"case": 1})
        first = out["items"][0] if ok else out
        check(ok and first["type"] == "BAR_DEAD" and first["values"].get("Z") == -1.0, f"peso propio con Z = -1: {first}")
        ok, out = await call(client, "robot_run_calculation", {"timeout_s": 120})
        check(ok and out["calculation"] == "COMPLETED", "recálculo completado")
        ok, out = await call(client, "robot_get_reactions", {"cases": "2"})
        total = out["totals"][0] if ok else {}
        print("   totales caso 2:", total)
        check(ok and abs(total["sum_fx_kn"] + 10.0) < 0.01 and abs(total["sum_fz_kn"] - live) < 0.01 * live, "reacción horizontal = −10 kN (equilibra los 2 × 5 kN)")

        print("== 8. guardar")
        ok, out = await call(client, "robot_save_project", {})
        check(not ok and "nunca" in str(out), "guardar un proyecto sin archivo y sin ruta: rechazado")
        ok, out = await call(client, "robot_save_project", {"path": saved})
        check(ok and out["applied"] is False and not os.path.exists(saved), "ensayo de guardar: no escribe")
        ok, out = await call(client, "robot_save_project", {"path": saved, "dry_run": False})
        print("   ", out)
        check(ok and os.path.isfile(saved), "guardar como: archivo escrito")
        stamp = os.path.getmtime(saved)
        await asyncio.sleep(1.2)
        ok, out = await call(client, "robot_save_project", {"dry_run": False})
        check(ok and out["action"] == "save" and os.path.getmtime(saved) > stamp, "guardar sobre el archivo actual: reescrito")
        other = saved.replace(".rtd", "_otro.rtd")
        open(other, "w").close()
        ok, out = await call(client, "robot_save_project", {"path": other, "dry_run": False})
        check(not ok and "Ya existe" in str(out) and os.path.getsize(other) == 0, "guardar como sobre un archivo ajeno sin overwrite: rechazado")
        os.remove(other)

    print("== cierre")
    app.Quit(om.IRobotQuitOption.I_QO_DISCARD_CHANGES)
    app = None
    time.sleep(6)
    for leftover in (saved, saved + ".bak"):
        if os.path.exists(leftover):
            os.remove(leftover)
    watch.__exit__()
    questions = [item for item in watch.seen if "&Sí" in item[3] or "&No" in item[3]]
    check(not questions, f"Robot no hizo ninguna pregunta ({len(watch.seen)} ventanas informativas)")
    check(not os.path.exists(rc._INTERACTIVE_MARKER), "no queda marcador de modo no interactivo")
    failed = [text for ok, text in checks if not ok]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} comprobaciones correctas")
    for text in failed:
        print("   FALLO:", text)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
