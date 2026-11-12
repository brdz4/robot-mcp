"""Prueba de las verificaciones NEC (robot_check_*) contra cuentas hechas aparte.

Uso (con Robot CERRADO):  .venv\\Scripts\\python.exe tests\\checks_test.py

Abre su propio Robot, crea el pórtico de make_fixture.py, le añade dos casos laterales de
valor conocido (uno asimétrico en X, que provoca torsión, y uno simétrico en Y), lo calcula y
compara cada verificación con el mismo cálculo hecho a partir de las herramientas de lectura.
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

# Cargas laterales (kN por nudo). Borde y=0: nudos 5,6 y 9,10; borde y=4: nudos 7,8 y 11,12.
SX = {5: 20, 6: 20, 7: 10, 8: 10, 9: 40, 10: 40, 11: 20, 12: 20}  # total 180 kN, asimétrica
SY = {n: 15 for n in range(5, 13)}  # total 120 kN, simétrica
checks: list[tuple[bool, str]] = []


def check(ok: bool, text: str) -> bool:
    checks.append((bool(ok), text))
    print(("   [OK] " if ok else "   [FALLO] ") + text)
    return bool(ok)


def close(a: float, b: float, tolerance: float = 0.01) -> bool:
    return abs(a - b) <= tolerance * max(abs(a), abs(b), 1e-9)


async def call(client: Client, name: str, args: dict | None = None) -> tuple[bool, object]:
    result = await client.call_tool(name, args or {})
    text = "".join(getattr(block, "text", "") for block in result.content)
    try:
        return not result.is_error, json.loads(text)
    except ValueError:
        return not result.is_error, text


async def main() -> int:  # noqa: C901
    import checks as nec

    print("== funciones puras")
    check(nec.interior_deviation([(0, 10.0), (5, 15.0), (10, 20.0)]) < 1e-9, "variación lineal entre bordes: desviación 0 (diafragma rígido)")
    measured = nec.interior_deviation([(0, 20.0), (8.0, 32.0), (16.0, 44.0), (24.0, 40.0)])
    check(measured > nec.DIAPHRAGM_TOLERANCE, f"pórticos independientes (sin diafragma): desviación {measured:.2f}")
    check(nec.interior_deviation([(0, 1.0), (4, 2.0)]) is None, "solo dos líneas: no se puede juzgar")
    check([nec.p_delta_class(q) for q in (0.05, 0.10, 0.30, 0.31)] == ["DESPRECIABLE", "AMPLIFICAR", "AMPLIFICAR", "INESTABLE"], "clases de P-Δ en los umbrales 0.10 y 0.30")
    if rc.robot_process_count() != 0:
        sys.exit("Hay un Robot abierto. Ciérralo: esta prueba abre el suyo y lo cierra sin guardar.")
    app = rc.create_application(om)
    app.Visible = 1
    app.Interactive = 0
    fixture = subprocess.run([sys.executable, str(ROOT / "tests" / "make_fixture.py"), "--no-panel"], capture_output=True, text=True)
    print("   " + (fixture.stdout.strip().splitlines() or [fixture.stderr.strip()[-200:]])[-1])
    st = app.Project.Structure
    if rc.robot_process_count() != 1 or st.Nodes.GetAll().Count != 12:
        sys.exit("No quedó una única instancia con el pórtico de prueba; se aborta.")
    nature, static = om.IRobotCaseNature.I_CN_SEISMIC, om.IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR
    for number, name, loads, component in (
        (4, "SX", SX, om.IRobotNodeForceRecordValues.I_NFRV_FX),
        (5, "SY", SY, om.IRobotNodeForceRecordValues.I_NFRV_FY),
    ):
        case = st.Cases.CreateSimple(number, name, nature, static)
        for value in sorted(set(loads.values())):
            record = case.Records.Get(case.Records.New(om.IRobotLoadRecordType.I_LRT_NODE_FORCE))
            record.SetValue(int(component), value * 1e3)
            for node, load in loads.items():
                if load == value:
                    record.Objects.AddOne(node)
    status = app.Project.CalcEngine.CalculateEx(om.IRobotCalculationMode.I_CM_LOCAL).ToString()
    print("   cálculo:", status)
    while rc.robot_starting():
        time.sleep(2)

    params = StdioServerParameters(
        command=sys.executable, args=[str(ROOT / "server.py")], cwd=str(ROOT), env={"ROBOT_MCP_READ_ONLY": "1"}
    )
    async with Client(params) as client:
        tools = [t.name for t in (await client.list_tools()).tools]
        check(len(tools) == 26 and sum(t.startswith("robot_check_") for t in tools) == 5, f"modo solo lectura: {len(tools)} herramientas (26), 5 de verificación")

        ok, table = await call(client, "robot_get_storey_results", {"cases": "3 4 5"})
        storeys = table["storeys"]
        raw = {st_["name"]: {c["case"]: c for c in st_["by_case"]} | {"h": st_["height_m"], "m": st_["mass_t"]} for st_ in storeys}
        names = [st_["name"] for st_ in storeys]
        print("   pisos:", names, "| cortante basal SX:", raw[names[0]][4]["shear_fx_kn"], "| SY:", raw[names[0]][5]["shear_fy_kn"])

        print("== cortante basal")
        ok, out = await call(client, "robot_check_base_shear", {"cases": [{"case": 4}, {"case": 5}], "static_shear_kn": 200})
        rows = {r["case"]: r for r in out["cases"]} if ok else {}
        check(ok and close(rows[4]["raw_shear_kn"], 180.0) and rows[4]["direction"] == "X", "SX: 180 kN en X (suma de las cargas aplicadas)")
        check(ok and close(rows[5]["raw_shear_kn"], 120.0) and rows[5]["direction"] == "Y", "SY: 120 kN en Y")
        check(ok and close(rows[4]["ratio"], 0.9) and rows[4]["passes"] and not rows[5]["passes"] and not out["passes"], "180/200 = 0.90 cumple; 120/200 = 0.60 no cumple")
        check(ok and close(rows[5]["scale_up_factor"], 0.8 * 200 / 120), "factor de ajuste = 0.80·200/120 = 1.333")
        ok, out = await call(client, "robot_check_base_shear", {"cases": [{"case": 4, "scale": 0.5}], "static_shear_kn": 100, "min_ratio": 0.85})
        check(ok and close(out["cases"][0]["dynamic_shear_kn"], 90.0) and out["passes"], "con scale 0.5: 90 kN ≥ 0.85·100")
        ok, mats = await call(client, "robot_get_reactions", {"cases": "1", "limit": 1})
        weight = mats["totals"][0]["sum_fz_kn"]
        ok, out = await call(client, "robot_check_base_shear", {"cases": [{"case": 4}], "seismic_coefficient": 0.5, "weight_case": 1})
        check(ok and close(out["static_shear_kn"], 0.5 * weight) and close(out["cases"][0]["ratio"], 180 / (0.5 * weight)), f"por coeficiente: V_est = 0.5·{weight:.1f} kN")
        ok, out = await call(client, "robot_check_base_shear", {"cases": [{"case": 4}]})
        check(not ok and "static_shear_kn" in str(out), "sin cortante estático: error claro")
        ok, out = await call(client, "robot_check_base_shear", {"cases": [{"case": 99}], "static_shear_kn": 10})
        check(not ok and "99" in str(out), "caso inexistente: error claro")

        print("== derivas")
        ok, out = await call(client, "robot_check_drifts", {"cases": [{"case": 4, "scale": 0.5}, {"case": 5, "scale": 0.5}], "r": 8})
        good = ok
        worst = 0.0
        for st_ in out["storeys"] if ok else []:
            for c in st_["by_case"]:
                source = raw[st_["storey"]][c["case"]]
                expected_x = 0.75 * 8 * 0.5 * abs(source["drift_ratio_x"])
                expected_y = 0.75 * 8 * 0.5 * abs(source["drift_ratio_y"])
                good = good and abs(c["inelastic_x"] - expected_x) < 2e-5 and abs(c["inelastic_y"] - expected_y) < 2e-5
                worst = max(worst, expected_x, expected_y)
        check(good, "ΔM = 0.75·8·0.5·(dr/h) en cada piso y caso")
        check(ok and abs(out["summary"]["storey_table_max"] - worst) < 2e-5, f"máximo de la tabla de pisos = {worst:.5f}")
        ok_low, low = await call(client, "robot_check_drifts", {"cases": [{"case": 4, "scale": 0.5}], "r": 8, "limit": worst * 0.5 if worst else 0.0001})
        ok_high, high = await call(client, "robot_check_drifts", {"cases": [{"case": 4, "scale": 0.5}], "r": 8, "limit": 0.05})
        check(ok_low and ok_high and not low["summary"]["passes"] and high["summary"]["passes"], "no cumple con un límite menor y cumple con uno mayor")
        # Deriva por línea de pórtico: SX líneas y=0 (nudos 1,2) e y=4 (3,4); SY líneas x=0 (1,4) y x=5 (2,3).
        ok_d, disp = await call(client, "robot_get_node_displacements", {"cases": "4 5", "selection": "1to12", "limit": 12})
        u = {item["node"]: {c["case"]: c for c in item["by_case"]} for item in disp["items"]}
        line_groups = {4: ("ux_mm", ((1, 2), (3, 4))), 5: ("uy_mm", ((1, 4), (2, 3)))}
        good, line_worst = ok and ok_d, 0.0
        for level, st_ in enumerate(out["storeys"] if ok else [], start=1):
            for c in st_["by_case"]:
                component, groups = line_groups[c["case"]]
                drifts = [abs(sum(u[n + 4 * level][c["case"]][component] - u[n + 4 * (level - 1)][c["case"]][component] for n in g) / len(g)) for g in groups]
                expected = 0.75 * 8 * 0.5 * max(drifts) / 1000 / st_["height_m"]
                line_worst = max(line_worst, expected)
                good = good and "frame_line" in c and abs(c["frame_line"]["inelastic"] - expected) < 2e-5
        check(good, "deriva por línea = 0.75·8·0.5·máx(Δlínea)/h en cada piso y caso")
        summary = out["summary"] if ok else {}
        check(ok and abs(summary["max_inelastic"] - max(worst, line_worst)) < 2e-5 and summary["source"] == ("frame_line" if line_worst > worst else "storey_table"), f"resumen = máx(tabla {worst:.5f}, línea {line_worst:.5f}), fuente {summary.get('source')}")

        print("== P-Delta")
        ok, out = await call(client, "robot_check_p_delta", {"seismic_cases": "4 5", "gravity_case": 3})
        good = ok
        for case in out["cases"] if ok else []:
            axis = case["direction"].lower()
            for st_ in case["storeys"]:
                source = raw[st_["storey"]]
                p = abs(source[3]["axial_fz_kn"])
                drift = abs(source[case["case"]][f"drift_{axis}_mm"]) / 1000
                shear = abs(source[case["case"]][f"shear_f{axis}_kn"])
                expected = p * drift / (shear * source["h"])
                good = good and close(st_["q"], expected, 0.02) and st_["classification"] == ("DESPRECIABLE" if expected < 0.1 else "AMPLIFICAR" if expected <= 0.3 else "INESTABLE")
        check(good, "Qi = Pi·Δi/(Vi·hi) con P del caso de gravedad, en cada piso y caso")
        check(ok and out["cases"][0]["direction"] == "X" and out["cases"][1]["direction"] == "Y", "dirección principal: SX en X, SY en Y")
        print("   resumen:", out.get("summary") if ok else out)

        print("== irregularidades")
        ok, out = await call(client, "robot_check_irregularities", {"cases": "4 5", "mass_case": 1})
        ok_m, weights = await call(client, "robot_get_storey_results", {"cases": "1"})
        for st_ in weights["storeys"]:
            raw[st_["name"]]["m"] = st_["mass_t"]  # masa del caso de peso propio, la que se pidió
        print("   masas por piso (caso 1):", [raw[n]["m"] for n in names], "| mass_case:", out.get("mass_case") if ok else out)
        ok_d, disp = await call(client, "robot_get_node_displacements", {"cases": "4 5", "selection": "1to12", "limit": 12})
        u = {item["node"]: {c["case"]: c for c in item["by_case"]} for item in disp["items"]}

        def expected_ratio(case: int, component: str, edges: tuple[tuple[int, ...], tuple[int, ...]], level: int) -> float:
            def edge(nodes, lvl):
                return sum(u[n + 4 * lvl][case][component] for n in nodes) / len(nodes)

            d = [abs(edge(nodes, level) - edge(nodes, level - 1)) for nodes in edges]
            return max(d) / (sum(d) / 2)

        torsion = {t["case"]: t for t in out["torsional"]} if ok else {}
        good = ok
        for level, name in enumerate(names, start=1):
            exp_x = expected_ratio(4, "ux_mm", ((1, 2), (3, 4)), level)  # bordes y=0 e y=4
            exp_y = expected_ratio(5, "uy_mm", ((1, 4), (2, 3)), level)  # bordes x=0 y x=5
            got_x = next(s_["ratio"] for s_ in torsion[4]["storeys"] if s_["storey"] == name)
            got_y = next(s_["ratio"] for s_ in torsion[5]["storeys"] if s_["storey"] == name)
            print(f"   {name}: torsión SX {got_x} (esperado {exp_x:.3f}) | SY {got_y} (esperado {exp_y:.3f})")
            good = good and close(got_x, exp_x, 0.01) and close(got_y, exp_y, 0.01)
        check(good, "relación Δmax/Δprom igual a la calculada con los desplazamientos de los nudos de borde")
        check(ok and all(close(s_["ratio"], 1.0, 0.01) and not s_["irregular"] for s_ in torsion[5]["storeys"]), "carga simétrica en Y: relación 1.0, sin torsión")
        check(ok and all(s_["ratio"] > 1.02 for s_ in torsion[4]["storeys"]) and torsion[4]["approximate"] is False, "carga asimétrica en X: relación mayor que 1 y caso estático exacto")
        mass = out["mass"] if ok else []
        check(ok and close(mass[0]["mass_t"], raw[names[0]]["m"]) and mass[-1]["irregular"] is False, "masas iguales a la tabla de pisos; la cubierta no se marca")
        check(ok and close(mass[0]["ratio_to_adjacent"], raw[names[0]]["m"] / raw[names[1]]["m"], 0.01), "relación de masa con el piso adyacente")
        soft = {t["case"]: t for t in out["soft_storey"]} if ok else {}
        k = [abs(raw[n][4]["shear_fx_kn"]) / abs(raw[n][4]["drift_x_mm"]) for n in names]
        check(ok and close(soft[4]["storeys"][0]["ratio_to_above"], k[0] / k[1], 0.01), f"rigidez K1/K2 = {k[0] / k[1]:.3f}")
        print("   resumen:", out.get("summary") if ok else out)

    app.Quit(om.IRobotQuitOption.I_QO_DISCARD_CHANGES)
    app = st = None
    time.sleep(6)
    failed = [text for ok, text in checks if not ok]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} comprobaciones correctas | procesos de Robot: {rc.robot_process_count()}")
    for text in failed:
        print("   FALLO:", text)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
