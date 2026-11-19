"""Verificaciones sísmicas según NEC-SE-DS 2015 sobre los resultados de Robot. Solo lectura.

Robot no conoce la NEC: aquí se aplican sus fórmulas a la tabla de pisos, las reacciones y los
desplazamientos que Robot ya calculó. Los datos normativos (R, factores de escala, cortante
estático) los aporta quien llama; las herramientas no los inventan.

  Derivas        §6.3.9  ΔM = 0.75·R·ΔE ≤ límite
  P-Δ            §6.3.8  Qi = Pi·Δi / (Vi·hi)
  Cortante basal §6.2.2  V dinámico ≥ 80 % (regular) u 85 % (irregular) del estático
  Irregularidad  §5.2    torsional, piso flexible y masa
"""

from __future__ import annotations

from typing import Annotated, Any

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, ConfigDict, Field

from .results import MAX_READS, _cases, _modal_case, _results
from .robot_com import Session, num, on_robot
from .schema import READ_ONLY, Cases

LEVEL_TOLERANCE_M = 0.01
NODES_PER_LINE = 3  # nudos leídos por línea de pórtico (bajo diafragma rígido todos valen lo mismo)
DIAPHRAGM_TOLERANCE = 0.10  # desviación interior admitida para considerar el piso diafragma rígido


class ScaledCase(BaseModel):
    model_config = ConfigDict(extra="forbid")

    case: int = Field(ge=1, description="Número del caso sísmico (ver robot_list_load_cases).")
    scale: float = Field(
        default=1.0,
        gt=0,
        description="Factor que lleva los resultados del caso al nivel de diseño reducido. 1.0 si el espectro "
        "ya se cargó reducido; si se cargó elástico, el factor con que ese caso entra en las combinaciones "
        "(ver robot_list_combinations), que equivale a I/(R·ΦP·ΦE) por el ajuste de unidades.",
    )


ScaledCases = Annotated[
    list[ScaledCase],
    Field(min_length=1, max_length=10, description="Casos sísmicos a verificar, cada uno con su factor de escala."),
]


# ---------- lectura común ----------


def storey_raw(s: Session, res: Any, case_numbers: list[int], mass_case: int | None = None) -> list[dict[str, Any]]:
    """Tabla de pisos en unidades de la API (m, N, kg), de abajo hacia arriba.

    La masa de piso que da Robot depende del caso consultado cuando el modelo no tiene masas
    dinámicas definidas, así que se lee de `mass_case` (por defecto, el primer caso).
    """
    manager = s.structure.Storeys
    if manager.Count == 0:
        raise ToolError("El modelo no tiene pisos definidos; créalos en Robot para poder verificar por piso.")
    if manager.Count * len(case_numbers) * 2 > MAX_READS:
        raise ToolError("Demasiados pisos × casos para una sola llamada; verifica menos casos cada vez.")
    rows = []
    for index in range(1, manager.Count + 1):
        storey = manager.Get(index)
        height = float(storey.Height)
        by_case = {}
        for number in case_numbers:
            moved = res.Storeys.Displacements(index, number)
            forces = res.Storeys.ReducedForces(index, number)
            by_case[number] = {
                "dr_x": abs(moved.DrUX),
                "dr_y": abs(moved.DrUY),
                "v_x": abs(forces.FX),
                "v_y": abs(forces.FY),
                "p": abs(forces.FZ),
            }
        rows.append(
            {
                "name": str(storey.Name),
                "level": float(storey.Level),
                "top": float(storey.Level) + height,
                "height": height,
                "mass": float(res.Storeys.Values(index, mass_case or case_numbers[0]).Mass),
                "by_case": by_case,
            }
        )
    rows.sort(key=lambda row: row["level"])
    return rows


def dominant_direction(rows: list[dict[str, Any]], case: int) -> str:
    """Dirección principal del caso: la del mayor cortante en el piso inferior."""
    base = rows[0]["by_case"][case]
    return "x" if base["v_x"] >= base["v_y"] else "y"


def interior_deviation(lines: list[tuple[float, float]]) -> float | None:
    """Cuánto se apartan las líneas interiores de la recta que une las dos líneas de borde.

    `lines` = [(coordenada, desplazamiento)] ordenadas. Un diafragma rígido hace que el
    desplazamiento varíe linealmente con la coordenada (traslación + giro), y entonces el
    resultado es ~0. Se normaliza con el desplazamiento medio de los bordes. None si no hay
    líneas interiores.
    """
    if len(lines) < 3:
        return None
    (c1, v1), (c2, v2) = lines[0], lines[-1]
    scale = (abs(v1) + abs(v2)) / 2
    if scale == 0 or c2 == c1:
        return None
    return max(abs(v - (v1 + (v2 - v1) * (c - c1) / (c2 - c1))) for c, v in lines[1:-1]) / scale


def level_nodes(s: Session, rows: list[dict[str, Any]]) -> list[list[tuple[int, float, float]]]:
    """Nudos (número, X, Y) de cada nivel: la base del primer piso y el techo de cada piso."""
    om = s.om
    levels = [rows[0]["level"]] + [row["top"] for row in rows]
    col = s.structure.Nodes.GetMany(s.select(om.IRobotObjectType.I_OT_NODE))
    by_level: list[list[tuple[int, float, float]]] = [[] for _ in levels]
    for i in range(1, col.Count + 1):
        node = om.IRobotNode(col.Get(i))
        for index, level in enumerate(levels):
            if abs(node.Z - level) <= LEVEL_TOLERANCE_M:
                by_level[index].append((int(node.Number), float(node.X), float(node.Y)))
                break
    return by_level


def frame_lines(res: Any, by_level: list[list[tuple[int, float, float]]], case: int, axis: str, reads: int) -> tuple[list[list[tuple[float, float]] | None], int]:
    """Por nivel, desplazamiento medio (m) de cada línea de pórtico paralela a `axis`: [(coordenada, valor)].

    None en los niveles con menos de dos líneas. Devuelve también las lecturas acumuladas.
    """
    displacements = res.Nodes.Displacements
    component = "UX" if axis == "x" else "UY"
    out: list[list[tuple[float, float]] | None] = []
    for nodes in by_level:
        groups: dict[float, list[int]] = {}
        for n, x, y in nodes:
            groups.setdefault(round(y if axis == "x" else x, 2), []).append(n)
        if len(groups) < 2 or max(groups) - min(groups) < 0.5:
            out.append(None)
            continue
        lines = []
        for coordinate in sorted(groups):
            sample = groups[coordinate][:NODES_PER_LINE]
            reads += len(sample)
            if reads > MAX_READS:
                raise ToolError("Demasiados nudos × casos para una sola llamada; verifica menos casos cada vez.")
            values = [getattr(displacements.Value(n, case), component) for n in sample]
            lines.append((coordinate, sum(values) / len(values)))
        out.append(lines)
    return out, reads


def p_delta_class(q: float) -> str:
    if q < 0.10:
        return "DESPRECIABLE"
    return "AMPLIFICAR" if q <= 0.30 else "INESTABLE"


def register(mcp: MCPServer) -> None:  # noqa: C901
    @mcp.tool(name="robot_check_drifts", title="Verificar derivas inelásticas (NEC §6.3.9)", annotations=READ_ONLY)
    async def robot_check_drifts(
        cases: ScaledCases,
        r: Annotated[float, Field(gt=0, le=12, description="Factor de reducción R del sistema estructural (NEC Tablas 15 y 16).")],
        limit: Annotated[float, Field(gt=0, le=0.05, description="Deriva máxima admisible: 0.02 hormigón, acero y madera; 0.01 mampostería.")] = 0.02,
        amplification: Annotated[float, Field(gt=0, le=1, description="Coeficiente de la NEC en ΔM = 0.75·R·ΔE.")] = 0.75,
    ) -> dict[str, Any]:
        """Deriva inelástica máxima por piso, ΔM = 0.75·R·ΔE, frente al límite de la NEC-SE-DS §6.3.9.

        ΔE es la deriva de entrepiso de Robot (dr/altura) multiplicada por el factor 'scale' del caso.
        Además de la tabla de pisos de Robot (un valor por planta), calcula la deriva de cada línea
        de pórtico paralela a la dirección principal del caso, con el desplazamiento medio de sus
        nudos, y reporta la mayor (frame_line). Sin diafragma rígido cada pórtico se mueve por su
        cuenta y es esa la que gobierna; la tabla de pisos puede quedarse corta.
        Devuelve {results_status, formula, r, limit, storeys:[{storey, height_m, by_case:[{case, scale,
        elastic_x, elastic_y, inelastic_x, inelastic_y, governing, passes, frame_line?:{direction,
        coordinate_m, drift_mm, inelastic, passes, approximate}}]}], summary:{max_inelastic, storey,
        case, direction, source ('storey_table' | 'frame_line'), coordinate_m?, utilization, passes,
        storey_table_max}}. Las derivas son adimensionales (0.015 = 1.5 %). coordinate_m es la Y (caso
        en X) o la X (caso en Y) de la línea. approximate=true en casos espectrales: la deriva de línea
        sale de restar desplazamientos ya combinados. Usa los casos sísmicos (no las combinaciones con
        gravedad). R y scale son datos del proyecto: si no los conoces, pregúntalos; un scale
        equivocado invalida la verificación.
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, " ".join(str(c.case) for c in cases), strict=True)
            numbers = [c["number"] for c in case_list]
            scales = {c.case: c.scale for c in cases}
            analysis = {c["number"]: c["analysis"] for c in case_list}
            rows = storey_raw(s, res, numbers)
            by_level = level_nodes(s, rows)
            lines: dict[int, tuple[str, list[list[tuple[float, float]] | None]]] = {}
            reads = 0
            for number in numbers:
                axis = dominant_direction(rows, number)
                level_lines, reads = frame_lines(res, by_level, number, axis, reads)
                lines[number] = (axis, level_lines)
            worst: dict[str, Any] = {"max_inelastic": 0.0}
            worst_line: dict[str, Any] = {"max_inelastic": 0.0}
            storeys = []
            for i, row in enumerate(rows):
                by_case = []
                for number in numbers:
                    data = row["by_case"][number]
                    elastic = {axis: data[f"dr_{axis}"] / row["height"] * scales[number] for axis in ("x", "y")}
                    inelastic = {axis: amplification * r * value for axis, value in elastic.items()}
                    axis = max(inelastic, key=inelastic.get)
                    by_case.append(
                        {
                            "case": number,
                            "scale": scales[number],
                            "elastic_x": num(elastic["x"], 6),
                            "elastic_y": num(elastic["y"], 6),
                            "inelastic_x": num(inelastic["x"], 5),
                            "inelastic_y": num(inelastic["y"], 5),
                            "governing": num(inelastic[axis], 5),
                            "passes": inelastic[axis] <= limit,
                        }
                    )
                    if inelastic[axis] > worst["max_inelastic"]:
                        worst = {"max_inelastic": inelastic[axis], "storey": row["name"], "case": number, "direction": axis.upper()}
                    line_axis, level_lines = lines[number]
                    below, here = level_lines[i], level_lines[i + 1]
                    lower = dict(below or [])
                    drifts = [(c, abs(v - lower[c])) for c, v in here or [] if c in lower]
                    if drifts:
                        coordinate, drift = max(drifts, key=lambda item: item[1])
                        line_inelastic = amplification * r * drift / row["height"] * scales[number]
                        by_case[-1]["frame_line"] = {
                            "direction": line_axis.upper(),
                            "coordinate_m": num(coordinate, 2),
                            "drift_mm": num(drift * 1e3, 3),
                            "inelastic": num(line_inelastic, 5),
                            "passes": line_inelastic <= limit,
                            "approximate": analysis[number] != "STATIC_LINEAR",
                        }
                        if line_inelastic > worst_line["max_inelastic"]:
                            worst_line = {
                                "max_inelastic": line_inelastic,
                                "storey": row["name"],
                                "case": number,
                                "direction": line_axis.upper(),
                                "coordinate_m": num(coordinate, 2),
                            }
                storeys.append({"storey": row["name"], "height_m": num(row["height"], 3), "by_case": by_case})
            table_max = worst["max_inelastic"]
            if worst_line["max_inelastic"] > table_max:
                worst = worst_line | {"source": "frame_line"}
            else:
                worst["source"] = "storey_table"
            worst["utilization"] = num(worst["max_inelastic"] / limit, 3)
            worst["passes"] = worst["max_inelastic"] <= limit
            worst["max_inelastic"] = num(worst["max_inelastic"], 5)
            worst["storey_table_max"] = num(table_max, 5)
            return {
                "results_status": status,
                "formula": f"ΔM = {amplification:g}·R·ΔE, con ΔE = (dr/h)·scale; límite {limit:g} (NEC-SE-DS §6.3.9)",
                "r": r,
                "limit": limit,
                "cases": case_list,
                "storeys": storeys,
                "summary": worst,
            }

        return await on_robot(work)

    @mcp.tool(name="robot_check_p_delta", title="Verificar efectos P-Δ (NEC §6.3.8)", annotations=READ_ONLY)
    async def robot_check_p_delta(
        seismic_cases: Cases,
        gravity_case: Annotated[
            int,
            Field(ge=1, description="Caso o combinación con la carga vertical total SIN mayorar (muerta + viva), p. ej. una combinación 'CM+CV'."),
        ],
    ) -> dict[str, Any]:
        """Índice de estabilidad por piso, Qi = Pi·Δi / (Vi·hi), según NEC-SE-DS §6.3.8.

        Pi es la carga vertical acumulada del piso en 'gravity_case'; Δi y Vi son la deriva y el
        cortante del piso en el caso sísmico, en su dirección principal. Como Δi y Vi salen del
        mismo caso, el factor de escala del sismo no influye en Qi. Devuelve {results_status,
        formula, gravity_case, cases:[{case, direction, storeys:[{storey, p_kn, drift_mm, shear_kn,
        height_m, q, classification, amplification_factor?}], max_q, classification}], summary}.
        classification: DESPRECIABLE (Q < 0.10), AMPLIFICAR (0.10 a 0.30: multiplicar derivas y
        fuerzas por 1/(1−Q), que se da en amplification_factor) o INESTABLE (Q > 0.30: rigidizar).
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            seismic = _cases(s, seismic_cases, strict=True)
            gravity = _cases(s, str(gravity_case))[0]
            numbers = [c["number"] for c in seismic]
            rows = storey_raw(s, res, numbers + [gravity["number"]])
            out_cases = []
            overall = 0.0
            for number in numbers:
                axis = dominant_direction(rows, number)
                storeys = []
                worst = 0.0
                for row in rows:
                    data = row["by_case"][number]
                    p = row["by_case"][gravity["number"]]["p"]
                    drift, shear = data[f"dr_{axis}"], data[f"v_{axis}"]
                    entry: dict[str, Any] = {
                        "storey": row["name"],
                        "p_kn": num(p / 1e3, 1),
                        "drift_mm": num(drift * 1e3, 3),
                        "shear_kn": num(shear / 1e3, 1),
                        "height_m": num(row["height"], 3),
                    }
                    if shear > 0 and row["height"] > 0:
                        q = p * drift / (shear * row["height"])
                        entry["q"] = num(q, 4)
                        entry["classification"] = p_delta_class(q)
                        if entry["classification"] == "AMPLIFICAR":
                            entry["amplification_factor"] = num(1 / (1 - q), 3)
                        worst = max(worst, q)
                    else:
                        entry["q"] = None
                        entry["classification"] = "SIN_CORTANTE"
                    storeys.append(entry)
                overall = max(overall, worst)
                out_cases.append(
                    {"case": number, "direction": axis.upper(), "storeys": storeys, "max_q": num(worst, 4), "classification": p_delta_class(worst)}
                )
            return {
                "results_status": status,
                "formula": "Qi = Pi·Δi / (Vi·hi); Q < 0.10 se desprecia, 0.10–0.30 amplificar por 1/(1−Q), > 0.30 inestable (NEC-SE-DS §6.3.8)",
                "gravity_case": gravity,
                "cases": out_cases,
                "summary": {"max_q": num(overall, 4), "classification": p_delta_class(overall)},
            }

        return await on_robot(work)

    @mcp.tool(name="robot_check_base_shear", title="Verificar cortante basal dinámico (NEC §6.2.2)", annotations=READ_ONLY)
    async def robot_check_base_shear(
        cases: ScaledCases,
        static_shear_kn: Annotated[
            float | None, Field(gt=0, description="Cortante basal estático V en kN (NEC §6.3.2). Alternativa: seismic_coefficient + weight_case.")
        ] = None,
        seismic_coefficient: Annotated[
            float | None, Field(gt=0, le=2, description="Coeficiente sísmico C = I·Sa/(R·ΦP·ΦE), como fracción de W (0.12 = 12 %).")
        ] = None,
        weight_case: Annotated[
            int | None, Field(ge=1, description="Caso o combinación cuya reacción vertical total es la carga reactiva W (p. ej. la combinación de carga muerta).")
        ] = None,
        min_ratio: Annotated[float, Field(gt=0, le=1, description="Mínimo exigido: 0.80 estructuras regulares, 0.85 irregulares.")] = 0.80,
    ) -> dict[str, Any]:
        """Compara el cortante basal del análisis dinámico con el estático, según NEC-SE-DS §6.2.2.

        V dinámico = cortante basal del caso en Robot × scale, en su dirección principal. El estático
        se da directamente (static_shear_kn) o como coeficiente × W, con W leído de 'weight_case'.
        Devuelve {results_status, formula, static_shear_kn, weight_kn?, min_ratio, cases:[{case,
        direction, raw_shear_kn, scale, dynamic_shear_kn, ratio, passes, scale_up_factor,
        required_scale}], passes}. Si no cumple, scale_up_factor = min_ratio·V_est/V_din es por
        cuánto hay que multiplicar el caso, y required_scale el factor total resultante.
        """
        direct = static_shear_kn is not None
        by_weight = seismic_coefficient is not None or weight_case is not None
        if direct == by_weight or (by_weight and (seismic_coefficient is None or weight_case is None)):
            raise ToolError("Indica static_shear_kn, o bien seismic_coefficient junto con weight_case (una de las dos formas).")

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, " ".join(str(c.case) for c in cases), strict=True)
            numbers = [c["number"] for c in case_list]
            reactions = res.Nodes.Reactions
            out: dict[str, Any] = {"results_status": status}
            if direct:
                static = float(static_shear_kn) * 1e3
            else:
                weight_info = _cases(s, str(weight_case))[0]
                weight = abs(reactions.Sum(weight_info["number"]).FZ)
                if weight <= 0:
                    raise ToolError(f"El caso {weight_case} no tiene reacción vertical; no sirve como carga reactiva W.")
                static = float(seismic_coefficient) * weight
                out["weight_case"] = weight_info
                out["weight_kn"] = num(weight / 1e3, 1)
                out["seismic_coefficient"] = seismic_coefficient
            out["formula"] = f"V_din = V_Robot·scale ≥ {min_ratio:g}·V_est (NEC-SE-DS §6.2.2)"
            out["static_shear_kn"] = num(static / 1e3, 1)
            out["min_ratio"] = min_ratio
            rows = []
            for item in cases:
                total = reactions.Sum(item.case)
                fx, fy = abs(total.FX), abs(total.FY)
                raw = max(fx, fy)
                dynamic = raw * item.scale
                ratio = dynamic / static
                passes = ratio >= min_ratio
                factor = 1.0 if passes or dynamic == 0 else min_ratio * static / dynamic
                rows.append(
                    {
                        "case": item.case,
                        "direction": "X" if fx >= fy else "Y",
                        "raw_shear_kn": num(raw / 1e3, 1),
                        "scale": item.scale,
                        "dynamic_shear_kn": num(dynamic / 1e3, 1),
                        "ratio": num(ratio, 4),
                        "passes": passes,
                        "scale_up_factor": num(factor, 4),
                        "required_scale": num(item.scale * factor, 5),
                    }
                )
            out["cases"] = rows
            out["passes"] = all(row["passes"] for row in rows)
            return out

        return await on_robot(work)

    @mcp.tool(name="robot_check_irregularities", title="Detectar irregularidades (NEC §5.2)", annotations=READ_ONLY)
    async def robot_check_irregularities(
        cases: Cases,
        mass_case: Annotated[
            int | None,
            Field(ge=1, description="Caso del que leer la masa de cada piso. Omitir = el caso modal si existe; si no, el primero de 'cases'."),
        ] = None,
    ) -> dict[str, Any]:
        """Detecta irregularidad torsional, piso flexible e irregularidad de masa, según NEC-SE-DS §5.2.

        - Torsional (planta): por piso, Δmax > 1.2·(Δ1+Δ2)/2, con Δ1 y Δ2 las derivas de los dos
          bordes opuestos de la planta en la dirección del caso, leídas de los nudos de cada nivel.
        - Piso flexible (elevación): rigidez K = V/Δ menor que el 70 % de la del piso superior, o
          que el 80 % del promedio de los tres superiores.
        - Masa (elevación): masa del piso mayor que 1.5 veces la de un piso adyacente (no la cubierta).
          La masa se lee de mass_case; sin análisis modal, Robot da una masa distinta por caso.
        Devuelve {results_status, cases, mass_case, mass:[…], soft_storey:[{case, direction, storeys:[…]}],
        torsional:[{case, direction, approximate, storeys:[{storey, edge_drift_1_mm, edge_drift_2_mm,
        ratio, irregular, rigid_diaphragm, interior_deviation?, line_displacements_mm?}]}],
        summary:{torsional, soft_storey, mass, non_rigid_floor}, notes}.
        rigid_diaphragm indica si los desplazamientos de las líneas de pórtico del piso varían
        linealmente entre bordes (como exige un diafragma rígido). Si es false, los pórticos se
        mueven por separado, la relación de bordes no mide torsión e 'irregular' queda en null:
        en ese caso avisa al usuario de que el modelo no tiene diafragma rígido, enséñale
        line_displacements_mm ([coordenada m, desplazamiento mm] por línea) y pregúntale si el
        edificio real lo tiene.
        Límites que hay que decir al usuario: en casos espectrales la deriva de borde se aproxima
        como diferencia de desplazamientos combinados (approximate=true); la torsional debe
        evaluarse con la excentricidad accidental incluida en el caso; y no se revisan las demás
        irregularidades de la norma (retrocesos, discontinuidad de diafragma, ejes no paralelos,
        geométrica), que dependen de la geometría.
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, cases, strict=True)
            numbers = [c["number"] for c in case_list]
            analysis = {c["number"]: c["analysis"] for c in case_list}
            source = mass_case
            if source is None:
                source = _modal_case(s) or numbers[0]
            elif not s.structure.Cases.Exist(source):
                raise ToolError(f"No existe el caso {source} indicado en mass_case.")
            rows = storey_raw(s, res, numbers, mass_case=source)
            top = len(rows) - 1

            mass = []
            for i, row in enumerate(rows):
                neighbours = [rows[j]["mass"] for j in (i - 1, i + 1) if 0 <= j <= top and rows[j]["mass"] > 0]
                ratio = max((row["mass"] / other for other in neighbours), default=None)
                mass.append(
                    {
                        "storey": row["name"],
                        "mass_t": num(row["mass"] / 1e3, 2),
                        "ratio_to_adjacent": num(ratio, 3) if ratio else None,
                        "irregular": bool(ratio and ratio > 1.5 and i != top),
                    }
                )

            soft = []
            for number in numbers:
                axis = dominant_direction(rows, number)
                stiffness = []
                for row in rows:
                    data = row["by_case"][number]
                    drift = data[f"dr_{axis}"]
                    stiffness.append(data[f"v_{axis}"] / drift if drift > 0 else None)
                storeys = []
                for i, row in enumerate(rows):
                    above = [k for k in stiffness[i + 1 : i + 4] if k]
                    k = stiffness[i]
                    to_above = k / stiffness[i + 1] if k and i < top and stiffness[i + 1] else None
                    to_three = k / (sum(above) / 3) if k and len(above) == 3 else None
                    storeys.append(
                        {
                            "storey": row["name"],
                            "stiffness_kn_per_mm": num(k / 1e6, 2) if k else None,
                            "ratio_to_above": num(to_above, 3) if to_above else None,
                            "ratio_to_3_above": num(to_three, 3) if to_three else None,
                            "irregular": bool((to_above and to_above < 0.7) or (to_three and to_three < 0.8)),
                        }
                    )
                soft.append({"case": number, "direction": axis.upper(), "storeys": storeys})

            # Torsional: líneas de pórtico de cada nivel; los bordes son la primera y la última.
            by_level = level_nodes(s, rows)
            torsional = []
            reads = 0
            for number in numbers:
                axis = dominant_direction(rows, number)
                level_lines, reads = frame_lines(res, by_level, number, axis, reads)
                storeys = []
                for i, row in enumerate(rows):
                    below, here = level_lines[i], level_lines[i + 1]
                    if below is None or here is None:
                        storeys.append({"storey": row["name"], "ratio": None, "irregular": None})
                        continue
                    lower = dict(below)
                    (c1, v1), (c2, v2) = here[0], here[-1]
                    if c1 not in lower or c2 not in lower:
                        storeys.append({"storey": row["name"], "ratio": None, "irregular": None})
                        continue
                    d1, d2 = abs(v1 - lower[c1]), abs(v2 - lower[c2])
                    average = (d1 + d2) / 2
                    ratio = max(d1, d2) / average if average > 0 else None
                    deviation = interior_deviation(here)
                    rigid = None if deviation is None else deviation <= DIAPHRAGM_TOLERANCE
                    entry: dict[str, Any] = {
                        "storey": row["name"],
                        "edge_drift_1_mm": num(d1 * 1e3, 3),
                        "edge_drift_2_mm": num(d2 * 1e3, 3),
                        "ratio": num(ratio, 3) if ratio else None,
                        # Sin diafragma rígido la relación de bordes no mide torsión: no se juzga.
                        "irregular": None if rigid is False else bool(ratio and ratio > 1.2),
                        "rigid_diaphragm": rigid,
                    }
                    if deviation is not None:
                        entry["interior_deviation"] = num(deviation, 3)
                        entry["line_displacements_mm"] = [[num(c, 2), num(v * 1e3, 2)] for c, v in here]
                    storeys.append(entry)
                torsional.append(
                    {"case": number, "direction": axis.upper(), "approximate": analysis[number] != "STATIC_LINEAR", "storeys": storeys}
                )
            non_rigid = any(st.get("rigid_diaphragm") is False for case in torsional for st in case["storeys"])
            notes = [
                "Torsional: Δmax > 1.2·(Δ1+Δ2)/2; piso flexible: K < 70 % del superior o < 80 % del promedio de 3 superiores; masa: > 1.5 veces la de un piso adyacente (NEC-SE-DS §5.2).",
                "En casos espectrales la deriva de borde es aproximada (diferencia de desplazamientos combinados).",
                "No se revisan retrocesos, discontinuidad de diafragma, ejes no paralelos ni irregularidad geométrica.",
            ]
            if non_rigid:
                notes.append(
                    "AVISO: en algún piso los desplazamientos de las líneas de pórtico no varían linealmente entre bordes "
                    "(rigid_diaphragm=false): los pórticos se mueven por separado y el piso no actúa como diafragma rígido en "
                    "el modelo. Ahí la relación de bordes no mide torsión y no se juzga. Conviene confirmar con el usuario si "
                    "el edificio real tiene diafragma (p. ej. capa de compresión) y, si lo tiene, modelarlo."
                )

            return {
                "results_status": status,
                "cases": case_list,
                "mass_case": source,
                "mass": mass,
                "soft_storey": soft,
                "torsional": torsional,
                "summary": {
                    "torsional": any(st["irregular"] for case in torsional for st in case["storeys"]),
                    "soft_storey": any(st["irregular"] for case in soft for st in case["storeys"]),
                    "mass": any(item["irregular"] for item in mass),
                    "non_rigid_floor": non_rigid,
                },
                "notes": notes,
            }

        return await on_robot(work)
