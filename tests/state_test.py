"""Prueba de estados con UN solo proceso de servidor:
sin resultados -> calculado -> desactualizado -> Robot cerrado -> Robot reabierto.

Uso (con Robot CERRADO):  .venv\\Scripts\\python.exe tests\\state_test.py

La prueba abre su propio Robot, crea el pórtico de make_fixture.py, lo calcula, lo modifica,
cierra y reabre Robot, y al final lo cierra SIN GUARDAR. Se niega a empezar si ya hay un Robot
abierto, para no tocar nunca un modelo real.
"""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import sys
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

CONCRETE_M3 = 8 * 0.4 * 0.4 * 3.0 + (2 * (5.0 + 4.0) * 2) * 0.3 * 0.5  # columnas + vigas = 9.24 m³
BEAM_LENGTH_M = 2 * (5.0 + 4.0) * 2  # 36 m con 10 kN/m en el caso CV
checks: list[tuple[bool, str]] = []


def check(ok: bool, text: str) -> None:
    checks.append((ok, text))
    print(("   [OK] " if ok else "   [FALLO] ") + text)


async def call(client: Client, name: str, args: dict | None = None) -> tuple[bool, object]:
    result = await client.call_tool(name, args or {})
    text = "".join(getattr(block, "text", "") for block in result.content)
    try:
        return not result.is_error, json.loads(text)
    except ValueError:
        return not result.is_error, text


def wait_for(condition, seconds: float) -> bool:
    deadline = time.time() + seconds
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(1)
    return False


async def main() -> int:
    if rc.robot_process_count() != 0:
        sys.exit("Hay un Robot abierto. Ciérralo: esta prueba abre el suyo y lo cierra sin guardar.")
    print("== 0. abrir Robot por COM (una sola instancia) y crear el pórtico")
    watch = DialogWatch(rc.robot_pids)
    watch.__enter__()
    app = rc.create_application(om)
    app.Visible = 1
    # Interactive = 0: Robot no pregunta. Con 1, modificar un modelo calculado abre el diálogo
    # "Los resultados de cálculos pueden desactualizarse. ¿Desea continuar?" y la llamada COM
    # queda bloqueada hasta que alguien responde.
    app.Interactive = 0
    fixture = subprocess.run(
        [sys.executable, str(ROOT / "tests" / "make_fixture.py"), "--no-panel"], capture_output=True, text=True
    )
    print("   " + (fixture.stdout.strip().splitlines() or [fixture.stderr.strip()[-200:]])[-1])
    project = app.Project
    if rc.robot_process_count() != 1 or project.FileName or project.Structure.Nodes.GetAll().Count != 12:
        sys.exit("No quedó una única instancia con el pórtico de prueba; se aborta sin tocar nada.")
    robot_exe = os.path.join(os.path.dirname(rc.interop_path()), "robot.exe")
    while rc.robot_starting():  # el servidor no conecta con un Robot recién abierto
        time.sleep(2)

    params = StdioServerParameters(command=sys.executable, args=["-m", "robot_structural_mcp.server"], cwd=str(ROOT))
    async with Client(params) as client:
        print("== 1. sin resultados")
        ok, health = await call(client, "robot_health")
        print("   health:", health)
        check(ok and health.get("results_status") == "NONE", "results_status = NONE antes de calcular")
        for tool, args in (
            ("robot_get_reactions", {"cases": "1 2 3"}),
            ("robot_get_bar_forces", {"cases": "1"}),
            ("robot_get_storey_results", {"cases": "1"}),
            ("robot_get_modal_results", {}),
            ("robot_get_result_extremes", {"quantity": "bar_forces", "cases": "3"}),
        ):
            ok, out = await call(client, tool, args)
            print(f"   {tool}: {str(out)[:150]}")
            check(not ok and "no tiene resultados" in str(out), f"{tool} avisa de que no hay resultados")

        print("== 2. calculado")
        t0 = time.time()
        status = project.CalcEngine.CalculateEx(om.IRobotCalculationMode.I_CM_LOCAL)
        print(f"   CalculateEx -> {status.ToString()} en {time.time() - t0:.1f} s")
        ok, health = await call(client, "robot_health")
        check(ok and health.get("results_status") == "AVAILABLE", "results_status = AVAILABLE tras calcular")
        ok, materials = await call(client, "robot_list_materials")
        unit_weight = materials["items"][0]["unit_weight_kn_m3"]
        ok, out = await call(client, "robot_get_reactions", {"cases": "1 2 3"})
        totals = {t["case"]: t for t in out["totals"]} if ok else {}
        print("   totals:", out["totals"] if ok else out)
        weight = CONCRETE_M3 * unit_weight
        live = BEAM_LENGTH_M * 10.0
        check(ok and abs(totals[1]["sum_fz_kn"] - weight) < 0.01 * weight, f"peso propio = {weight:.1f} kN calculado a mano")
        check(ok and abs(totals[2]["sum_fz_kn"] - live) < 0.01 * live, f"carga viva = {live:.1f} kN calculada a mano")
        combo = 1.2 * weight + 1.6 * live
        check(ok and abs(totals[3]["sum_fz_kn"] - combo) < 0.01 * combo, f"combinación 1.2PP+1.6CV = {combo:.1f} kN")
        check(ok and out["total"] == 4, "4 nudos apoyados")
        node_sum = sum(item["by_case"][0]["fz_kn"] for item in out["items"]) if ok else 0
        check(ok and abs(node_sum - totals[1]["sum_fz_kn"]) < 0.01, "suma de reacciones por nudo = total del caso")

        ok, out = await call(client, "robot_get_bar_forces", {"cases": "2", "selection": "9", "points": 3})
        print("   viga 9, caso CV:", json.dumps(out, ensure_ascii=False)[:420])
        if ok:
            stations = out["items"][0]["by_case"][0]["stations"]
            jump = stations[0]["fz_kn"] - stations[-1]["fz_kn"]
            check(abs(abs(jump) - 50.0) < 0.5, "viga de 5 m con 10 kN/m: salto de cortante = 50 kN")
        ok, out = await call(client, "robot_get_result_extremes", {"quantity": "bar_forces", "cases": "3", "components": ["FX", "MY"]})
        print("   extremos caso 3:", json.dumps(out, ensure_ascii=False)[:420])
        check(ok and out["extremes"][0]["max"]["value"] > 0, "envolvente: axial máximo de compresión positivo")
        ok, out = await call(client, "robot_get_node_displacements", {"cases": "3", "selection": "9to12"})
        check(ok and all(item["by_case"][0]["uz_mm"] < 0 for item in out["items"]), "nudos de cubierta bajan (uz < 0)")
        ok, out = await call(client, "robot_get_storey_results", {"cases": "1 3"})
        print("   pisos:", json.dumps(out, ensure_ascii=False)[:420])
        check(ok and len(out["storeys"]) == 2, "resultados para los 2 pisos")
        ok, out = await call(client, "robot_get_modal_results")
        print("   modal:", str(out)[:150])
        check(not ok and "modal" in str(out), "sin caso modal: error claro")

        print("== 3. desactualizado")
        record = om.IRobotLoadRecord(om.IRobotSimpleCase(project.Structure.Cases.Get(2)).Records.Get(1))
        t0 = time.time()
        record.SetValue(int(om.IRobotBarUniformRecordValues.I_BURV_PZ), -12000.0)
        check(time.time() - t0 < 2.0, f"cambiar una carga no espera a nadie ({time.time() - t0:.1f} s)")
        ok, health = await call(client, "robot_health")
        check(ok and health.get("results_status") == "OUT_OF_DATE", "results_status = OUT_OF_DATE tras cambiar una carga")
        for tool, args in (
            ("robot_get_reactions", {"cases": "2"}),
            ("robot_get_storey_results", {"cases": "1"}),
            ("robot_get_result_extremes", {"quantity": "reactions", "cases": "3"}),
        ):
            ok, out = await call(client, tool, args)
            print(f"   {tool}: {str(out)[:150]}")
            check(not ok and "desactualizados" in str(out), f"{tool} avisa de resultados desactualizados")
        ok, out = await call(client, "robot_list_bars", {"limit": 1})
        check(ok and out["total"] == 16, "las herramientas de modelo siguen funcionando")

        print("== 4. Robot cerrado con el servidor en marcha")
        app.Quit(om.IRobotQuitOption.I_QO_DISCARD_CHANGES)
        app = project = record = None
        check(wait_for(lambda: rc.robot_process_count() == 0, 40), "Robot se cerró")
        ok, health = await call(client, "robot_health")
        print("   health:", health)
        check(ok and health.get("robot_running") is False, "robot_health informa de Robot cerrado")
        ok, out = await call(client, "robot_list_nodes")
        print("   list_nodes:", str(out)[:160])
        check(not ok and "no está abierto" in str(out), "herramienta de datos: error claro con Robot cerrado")

        print("== 5. Robot reabierto: el mismo servidor debe reconectar")
        # Se abre como lo haría el usuario y se llama al servidor DURANTE el arranque.
        subprocess.Popen([robot_exe])
        ready, saw_starting, max_processes, last = False, False, 0, ""
        t0 = time.time()
        while time.time() - t0 < 150:
            await asyncio.sleep(2)
            ok, health = await call(client, "robot_health")
            max_processes = max(max_processes, rc.robot_process_count())
            state = str(health.get("hint") or "listo") if isinstance(health, dict) else str(health)
            if state != last:
                print(f"   t={time.time() - t0:5.1f} s  procesos={rc.robot_process_count()}  {state[:110]}")
                last = state
            if isinstance(health, dict) and health.get("ready") is False:
                saw_starting = True
            if ok and isinstance(health, dict) and "robot_version" in health:
                ready = True
                break
        check(ready, "robot_health reconecta con el Robot nuevo")
        check(saw_starting, "mientras Robot arranca, robot_health responde ready=false en vez de conectar")
        await asyncio.sleep(3)
        check(
            max_processes == 1 and rc.robot_process_count() == 1,
            f"no se abrió ninguna instancia oculta (máximo de procesos: {max_processes})",
        )
        ok, out = await call(client, "robot_list_nodes")
        print("   list_nodes:", str(out)[:160])
        check(not ok and "proyecto" in str(out), "sin proyecto activo: error claro")

    print("== cierre")
    if rc.robot_process_count():
        fresh = rc.create_application(om)
        if not fresh.Project.IsActive:
            fresh.Quit(om.IRobotQuitOption.I_QO_DISCARD_CHANGES)
            print("   Robot de prueba cerrado")
    watch.__exit__()
    questions = [item for item in watch.seen if "&Sí" in item[3] or "&No" in item[3]]
    check(not questions, f"Robot no hizo ninguna pregunta durante la prueba ({len(watch.seen)} ventanas informativas)")
    failed = [text for ok, text in checks if not ok]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} comprobaciones correctas")
    for text in failed:
        print("   FALLO:", text)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
