"""Herramientas de escritura: crean, asignan y, con robot_delete_*, borran.

Reglas comunes (ver `_execute`):
- dry_run=true por defecto: se valida todo y se devuelve el plan, sin tocar el modelo.
- Un cambio que invalida resultados vigentes exige discard_results=true.
- Se valida ANTES de escribir, porque la API de Robot acepta en silencio operaciones dañinas:
  crear un nudo con un número existente le cambia las coordenadas, admite barras duplicadas y
  combinaciones con casos inexistentes, e ignora barras con nudos que no existen.
- Se escribe con Robot en modo no interactivo (si no, pregunta y bloquea la llamada) y se
  relee lo escrito para confirmarlo.
Entradas en m, cm, kN, kN/m y kN·m; la API trabaja en m, N y N·m.
"""

from __future__ import annotations

import math
import os
import time
from typing import Annotated, Any, Callable, Literal, Union

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, ConfigDict, Field

from .robot_com import (
    Session,
    canonical_selection,
    enum_name,
    generated_records,
    non_interactive,
    num,
    on_robot,
    parse_selection,
    project_labels,
    section_item,
)
from .schema import OVERWRITE, WRITE, DiscardResults, DryRun

NATURES = {
    "PERMANENT": "I_CN_PERMANENT",
    "LIVE": "I_CN_EXPLOATATION",
    "WIND": "I_CN_WIND",
    "SNOW": "I_CN_SNOW",
    "SEISMIC": "I_CN_SEISMIC",
    "ACCIDENTAL": "I_CN_ACCIDENTAL",
    "TEMPERATURE": "I_CN_TEMPERATURE",
}
Nature = Literal["PERMANENT", "LIVE", "WIND", "SNOW", "SEISMIC", "ACCIDENTAL", "TEMPERATURE"]
DOFS = ("UX", "UY", "UZ", "RX", "RY", "RZ")


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", str_strip_whitespace=True)


class NodeIn(_Strict):
    x: float = Field(description="Coordenada global X en m.")
    y: float = Field(description="Coordenada global Y en m.")
    z: float = Field(description="Coordenada global Z en m.")
    number: int | None = Field(default=None, ge=1, description="Número de nudo. Omitir = siguiente libre.")


class BarIn(_Strict):
    start_node: int = Field(ge=1, description="Nudo inicial (debe existir).")
    end_node: int = Field(ge=1, description="Nudo final (debe existir).")
    number: int | None = Field(default=None, ge=1, description="Número de barra. Omitir = siguiente libre.")
    section: str | None = Field(default=None, description="Sección existente a asignar (ver robot_list_sections).")


class SelfWeightLoad(_Strict):
    type: Literal["self_weight"]
    factor: float = Field(default=1.0, gt=0, description="Multiplicador del peso propio (dirección −Z, toda la estructura).")


class NodeForceLoad(_Strict):
    type: Literal["node_force"]
    nodes: str = Field(description="Nudos cargados: '9 10', '5to8'.")
    fx_kn: float = 0.0
    fy_kn: float = 0.0
    fz_kn: float = Field(default=0.0, description="Fuerza global Z en kN; negativa = hacia abajo.")
    mx_knm: float = 0.0
    my_knm: float = 0.0
    mz_knm: float = 0.0


class BarUniformLoad(_Strict):
    type: Literal["bar_uniform"]
    bars: str = Field(description="Barras cargadas: '9to16'.")
    px_kn_m: float = 0.0
    py_kn_m: float = 0.0
    pz_kn_m: float = Field(default=0.0, description="Carga por metro en Z (kN/m); negativa = hacia abajo.")
    local: bool = Field(default=False, description="true = ejes locales de la barra; false = ejes globales.")


class PanelUniformLoad(_Strict):
    type: Literal["panel_uniform"]
    panels: str = Field(description="Paneles cargados: '100', '340to391'.")
    px_kn_m2: float = 0.0
    py_kn_m2: float = 0.0
    pz_kn_m2: float = Field(default=0.0, description="Carga superficial en Z (kN/m²); negativa = hacia abajo.")
    local: bool = Field(default=False, description="true = ejes locales del panel; false = ejes globales.")


class PointIn(_Strict):
    x: float
    y: float
    z: float


LoadIn = Annotated[Union[SelfWeightLoad, NodeForceLoad, BarUniformLoad, PanelUniformLoad], Field(discriminator="type")]


class FactorIn(_Strict):
    case: int = Field(ge=1, description="Número de caso existente.")
    factor: float = Field(description="Factor de mayoración; distinto de 0.")


class CombinationIn(_Strict):
    name: str = Field(min_length=1, max_length=60, description="Nombre, p. ej. '1.2D+1.6L'.")
    factors: list[FactorIn] = Field(min_length=1, max_length=30)
    type: Literal["ULS", "SLS", "ALS"] = Field(default="ULS", description="Estado límite: último, de servicio o accidental.")
    nature: Nature = "PERMANENT"
    number: int | None = Field(default=None, ge=1, description="Número de caso. Omitir = siguiente libre.")


def nec_combinations(
    dead: list[int],
    live: list[int],
    roof: list[int],
    wind: list[int],
    seismic_x: list[int],
    seismic_y: list[int],
    *,
    seismic_live_factor: float = 1.0,
    seismic_scale_x: float = 1.0,
    seismic_scale_y: float = 1.0,
    both_signs: bool = True,
    orthogonal: bool = False,
    service: bool = True,
    prefix: str = "NEC",
) -> list[CombinationIn]:
    """Combinaciones LRFD de la NEC-SE-CG §3.4.3 (y de servicio D+L) desarrolladas por caso.

    Los casos de una misma categoría (D, L, Lr) se suman con el mismo factor; cada caso de viento y
    cada caso sísmico entra por separado. Con orthogonal, cada sismo X se acompaña del 30 % del
    sismo Y de la misma posición en la lista, y viceversa. La escala sísmica multiplica el factor
    de cada caso X o Y (E7 con escala 0.2 entra como 0.2; 0.3E10 con 0.25, como 0.075); el
    nombre conserva la forma de la norma.
    """
    groups = {"D": dead, "L": live, "Lr": roof, "W": wind, "Ex": seismic_x, "Ey": seismic_y}
    seen: dict[int, str] = {}
    for key, cases in groups.items():
        for case in cases:
            if case in seen:
                raise ToolError(f"El caso {case} está a la vez en {seen[case]} y en {key}; cada caso va en una sola categoría.")
            seen[case] = key
    if not dead:
        raise ToolError("Hace falta al menos un caso de carga muerta (dead).")
    if orthogonal and len(seismic_x) != len(seismic_y):
        raise ToolError("Con orthogonal=true, seismic_x y seismic_y deben tener el mismo número de casos (se emparejan por posición).")
    out: list[CombinationIn] = []
    scale = {e: seismic_scale_x for e in seismic_x} | {e: seismic_scale_y for e in seismic_y}

    def combo(terms: list[tuple[float, str, list[int]]], kind: str = "ULS", nature: str = "PERMANENT") -> None:
        """terms = [(factor, símbolo para el nombre, casos)]; los términos sin casos se omiten."""
        used = [(f, symbol, cases) for f, symbol, cases in terms if cases and f]
        label = ""
        factors: dict[int, float] = {}
        for i, (f, symbol, cases) in enumerate(used):
            sign = "-" if f < 0 else ("" if i == 0 else "+")
            label += f"{sign}{'' if abs(f) == 1 else f'{abs(f):g}'}{symbol}"
            for case in cases:
                factors[case] = round(factors.get(case, 0.0) + f * scale.get(case, 1.0), 4)
        name = f"{prefix}{len(out) + 1} {label}"[:60]
        out.append(CombinationIn(name=name, type=kind, nature=nature, factors=[FactorIn(case=c, factor=v) for c, v in factors.items() if v]))

    combo([(1.4, "D", dead)])
    combo([(1.2, "D", dead), (1.6, "L", live), (0.5, "Lr", roof)])
    if roof:
        combo([(1.2, "D", dead), (1.6, "Lr", roof), (1.0, "L", live)])
        for w in wind:
            combo([(1.2, "D", dead), (1.6, "Lr", roof), (0.5, f"W{w}", [w])], nature="WIND")
    for w in wind:
        combo([(1.2, "D", dead), (1.0, f"W{w}", [w]), (1.0, "L", live), (0.5, "Lr", roof)], nature="WIND")
    signs = (1.0, -1.0) if both_signs else (1.0,)
    if orthogonal:
        pairs = list(zip(seismic_x, seismic_y)) + list(zip(seismic_y, seismic_x))
        actions = [[(a, f"E{main}", [main]), (0.3 * b, f"E{other}", [other])] for main, other in pairs for a in signs for b in signs]
    else:
        actions = [[(a, f"E{e}", [e])] for e in seismic_x + seismic_y for a in signs]
    for action in actions:
        combo([(1.2, "D", dead), *action, (seismic_live_factor, "L", live)], nature="SEISMIC")
    for w in wind:
        combo([(0.9, "D", dead), (1.0, f"W{w}", [w])], nature="WIND")
    for action in actions:
        combo([(0.9, "D", dead), *action], nature="SEISMIC")
    if service and live:
        combo([(1.0, "D", dead), (1.0, "L", live)], kind="SLS")
    return out


# ---------- núcleo común ----------


def _execute(
    s: Session,
    *,
    dry_run: bool,
    discard_results: bool,
    plan: dict[str, Any],
    apply: Callable[[], dict[str, Any]],
    affects_results: bool = True,
) -> dict[str, Any]:
    """Devuelve el plan (ensayo) o aplica el cambio de forma segura y lo confirma."""
    status = enum_name(s.structure.Results.Status)
    invalidates = affects_results and status == "AVAILABLE"
    head = {"dry_run": dry_run, "applied": False, "results_status_before": status, "invalidates_results": invalidates}
    if dry_run:
        extra = " y discard_results=true (los resultados actuales se pierden y habrá que recalcular)" if invalidates else ""
        return {**head, **plan, "next_step": f"No se ha modificado nada. Para aplicar, repite con dry_run=false{extra}."}
    if invalidates and not discard_results:
        raise ToolError(
            "El modelo tiene resultados vigentes y este cambio los invalida: habrá que recalcular. "
            "No se ha modificado nada. Si el usuario está de acuerdo, repite con discard_results=true."
        )
    with non_interactive(s.app):
        outcome = apply()
    return {**head, "applied": True, **plan, **outcome, "results_status_after": enum_name(s.structure.Results.Status)}


def _resolve(s: Session, obj_type: Any, text: str, what: str) -> tuple[Any, int, int]:
    """Selección explícita -> (selección de Robot, cuántos existen, cuántos pedidos no existen)."""
    if not text or not text.strip():
        raise ToolError(f"Indica explícitamente qué {what} modificar (p. ej. '1to8'); aquí no se admite 'todos'.")
    selection = s.select(obj_type, text)
    count = selection.Count
    if count == 0:
        raise ToolError(f"Ninguno de los {what} '{text}' existe en el modelo.")
    requested = sum((last - first) // step + 1 for first, last, step in parse_selection(text))
    return selection, count, max(0, requested - count)


def _free_numbers(server: Any, reserved: set[int]) -> Callable[[], int]:
    """Generador de números libres consecutivos a partir de FreeNumber."""
    state = {"next": int(server.FreeNumber)}

    def take() -> int:
        while state["next"] in reserved or server.Exist(state["next"]):
            state["next"] += 1
        reserved.add(state["next"])
        return state["next"]

    return take


def _explicit_numbers(items: list[Any], server: Any, what: str, why: str) -> set[int]:
    numbers = [item.number for item in items if item.number is not None]
    repeated = sorted({n for n in numbers if numbers.count(n) > 1})
    if repeated:
        raise ToolError(f"Números de {what} repetidos en la petición: {repeated}.")
    taken = [n for n in numbers if server.Exist(n)]
    if taken:
        raise ToolError(f"Ya existen los {what} {taken}. {why} Usa otros números u omítelos.")
    return set(numbers)


def _usage(s: Session, obj_type: Any, label_type: Any, text: str) -> dict[str, int]:
    """Cuántos elementos de la selección tienen hoy cada etiqueta de ese tipo."""
    usage = {}
    for label in project_labels(s, label_type, only_used=True):
        name = str(label.Name)
        count = s.select(obj_type, text, label=(label_type, name)).Count
        if count:
            usage[name] = count
    return usage


def _case_table(s: Session) -> dict[int, tuple[str, str]]:
    col = s.structure.Cases.GetAll()
    table = {}
    for i in range(1, col.Count + 1):
        case = s.om.IRobotCase(col.Get(i))
        table[int(case.Number)] = (str(case.Name), enum_name(case.Type))
    return table


def _combinations(s: Session, combinations: list[CombinationIn], dry_run: bool) -> dict[str, Any]:
    """Valida y crea combinaciones lineales (robot_create_combinations y robot_create_nec_combinations)."""
    om = s.om
    server = s.structure.Cases
    table = _case_table(s)
    names = {name.strip().lower() for name, _ in table.values()}
    reserved = _explicit_numbers(combinations, server, "casos", "No se sobrescriben casos.")
    take = _free_numbers(server, reserved)
    to_create = []
    for combo in combinations:
        if combo.name.lower() in names:
            raise ToolError(f"Ya existe un caso o combinación llamado '{combo.name}'. Usa otro nombre.")
        names.add(combo.name.lower())
        missing = [f.case for f in combo.factors if f.case not in table]
        if missing:
            raise ToolError(f"La combinación '{combo.name}' usa casos que no existen: {missing}.")
        if any(f.factor == 0 for f in combo.factors):
            raise ToolError(f"La combinación '{combo.name}' tiene un factor 0.")
        if len({f.case for f in combo.factors}) != len(combo.factors):
            raise ToolError(f"La combinación '{combo.name}' repite un caso.")
        to_create.append(
            {
                "number": combo.number or take(),
                "name": combo.name,
                "type": combo.type,
                "nature": combo.nature,
                "expression": " + ".join(f"{f.factor:g}·{table[f.case][0]}" for f in combo.factors),
                "factors": [f.model_dump() for f in combo.factors],
            }
        )

    def apply() -> dict[str, Any]:
        created = 0
        for item in to_create:
            made = server.CreateCombination(
                item["number"],
                item["name"],
                getattr(om.IRobotCombinationType, f"I_CBT_{item['type']}"),
                getattr(om.IRobotCaseNature, NATURES[item["nature"]]),
                om.IRobotCaseAnalizeType.I_CAT_COMB,
            )
            for factor in item["factors"]:
                made.CaseFactors.New(factor["case"], factor["factor"])
            created += int(made.CaseFactors.Count) == len(item["factors"])
        if created != len(to_create):
            raise ToolError(f"Robot confirmó {created} de {len(to_create)} combinaciones; revísalas en Robot.")
        return {"created": created}

    return _execute(s, dry_run=dry_run, discard_results=False, plan={"to_create": to_create}, apply=apply, affects_results=False)


def register(mcp: MCPServer) -> None:  # noqa: C901 - un registro por herramienta
    # ---------- geometría ----------

    @mcp.tool(name="robot_create_nodes", title="Crear nudos", annotations=WRITE)
    async def robot_create_nodes(
        nodes: Annotated[list[NodeIn], Field(min_length=1, max_length=500, description="Nudos a crear.")],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Crea nudos por coordenadas (m). Nunca mueve ni renumera nudos existentes.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, to_create:[{number,
        x, y, z}], already_existing:[{number, x, y, z}], next_step? | created, results_status_after}.
        Si en unas coordenadas ya hay un nudo, no se duplica: aparece en already_existing con su
        número, para usarlo al crear barras. Un número explícito que ya exista es un error.
        """

        def work(s: Session) -> dict[str, Any]:
            server = s.structure.Nodes
            reserved = _explicit_numbers(nodes, server, "nudos", "Crear un nudo con un número existente le cambiaría las coordenadas.")
            take = _free_numbers(server, reserved)
            seen: set[tuple[float, float, float]] = set()
            to_create, existing = [], []
            for node in nodes:
                key = (round(node.x, 6), round(node.y, 6), round(node.z, 6))
                if key in seen:
                    raise ToolError(f"Coordenadas repetidas en la petición: {key}.")
                seen.add(key)
                found = int(server.FindXYZ(node.x, node.y, node.z))
                entry = {"x": key[0], "y": key[1], "z": key[2]}
                if found:
                    existing.append({"number": found, **entry})
                else:
                    to_create.append({"number": node.number or take(), **entry})

            def apply() -> dict[str, Any]:
                for item in to_create:
                    server.Create(item["number"], item["x"], item["y"], item["z"])
                verified = 0
                for item in to_create:
                    if server.Exist(item["number"]):
                        made = s.om.IRobotNode(server.Get(item["number"]))
                        verified += math.dist((made.X, made.Y, made.Z), (item["x"], item["y"], item["z"])) < 1e-6
                if verified != len(to_create):
                    raise ToolError(f"Robot confirmó {verified} de {len(to_create)} nudos; revisa el modelo antes de seguir.")
                return {"created": verified}

            return _execute(
                s,
                dry_run=dry_run,
                discard_results=discard_results,
                plan={"to_create": to_create, "already_existing": existing},
                apply=apply,
                affects_results=bool(to_create),
            )

        return await on_robot(work)

    @mcp.tool(name="robot_create_bars", title="Crear barras", annotations=WRITE)
    async def robot_create_bars(
        bars: Annotated[list[BarIn], Field(min_length=1, max_length=500, description="Barras a crear.")],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Crea barras entre nudos existentes y, opcionalmente, les asigna una sección existente.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, to_create:[{number,
        start_node, end_node, length_m, section?}], next_step? | created, results_status_after}.
        Falla sin crear nada si algún nudo o sección no existe, o si ya hay una barra entre los
        mismos dos nudos. Crea antes los nudos (robot_create_nodes) y las secciones
        (robot_define_concrete_section).
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            server, nodes = s.structure.Bars, s.structure.Nodes
            section_type = om.IRobotLabelType.I_LT_BAR_SECTION
            reserved = _explicit_numbers(bars, server, "barras", "No se sobrescriben barras.")
            needed = {n for bar in bars for n in (bar.start_node, bar.end_node)}
            missing = sorted(n for n in needed if not nodes.Exist(n))
            if missing:
                raise ToolError(f"No existen los nudos {missing}. Créalos antes con robot_create_nodes.")
            for name in {bar.section for bar in bars if bar.section}:
                if not s.structure.Labels.Exist(section_type, name):
                    raise ToolError(f"No existe la sección '{name}'. Consulta robot_list_sections o defínela antes.")
            pairs: dict[frozenset[int], int] = {}
            col = server.GetAll()
            for i in range(1, col.Count + 1):
                bar = om.IRobotBar(col.Get(i))
                pairs[frozenset((int(bar.StartNode), int(bar.EndNode)))] = int(bar.Number)
            coords = {}
            for n in needed:
                node = om.IRobotNode(nodes.Get(n))
                coords[n] = (node.X, node.Y, node.Z)
            take = _free_numbers(server, reserved)
            to_create = []
            for bar in bars:
                if bar.start_node == bar.end_node:
                    raise ToolError(f"Barra con el mismo nudo inicial y final ({bar.start_node}).")
                key = frozenset((bar.start_node, bar.end_node))
                if key in pairs:
                    origin = f"la barra {pairs[key]}" if pairs[key] else "otra barra de esta misma petición"
                    raise ToolError(f"Entre los nudos {bar.start_node} y {bar.end_node} ya existe {origin}; no se duplica.")
                pairs[key] = 0
                entry: dict[str, Any] = {
                    "number": bar.number or take(),
                    "start_node": bar.start_node,
                    "end_node": bar.end_node,
                    "length_m": num(math.dist(coords[bar.start_node], coords[bar.end_node]), 4),
                }
                if bar.section:
                    entry["section"] = bar.section
                to_create.append(entry)

            def apply() -> dict[str, Any]:
                verified = 0
                for item in to_create:
                    server.Create(item["number"], item["start_node"], item["end_node"])
                    if not server.Exist(item["number"]):
                        continue
                    made = om.IRobotBar(server.Get(item["number"]))
                    if item.get("section"):
                        made.SetLabel(section_type, item["section"])
                    ok = (int(made.StartNode), int(made.EndNode)) == (item["start_node"], item["end_node"])
                    if item.get("section"):
                        ok = ok and str(made.GetLabelName(section_type)) == item["section"]
                    verified += ok
                if verified != len(to_create):
                    raise ToolError(f"Robot confirmó {verified} de {len(to_create)} barras; revisa el modelo antes de seguir.")
                return {"created": verified}

            return _execute(s, dry_run=dry_run, discard_results=discard_results, plan={"to_create": to_create}, apply=apply)

        return await on_robot(work)

    # ---------- secciones y apoyos ----------

    @mcp.tool(name="robot_define_concrete_section", title="Definir sección rectangular de hormigón", annotations=OVERWRITE)
    async def robot_define_concrete_section(
        name: Annotated[str, Field(min_length=1, max_length=40, description="Nombre de la sección, p. ej. 'V30x50'.")],
        kind: Annotated[Literal["column", "beam"], Field(description="Columna o viga de hormigón armado.")],
        b_cm: Annotated[float, Field(gt=0, le=500, description="Ancho b en cm.")],
        h_cm: Annotated[float, Field(gt=0, le=500, description="Altura h en cm.")],
        material: Annotated[
            str | None, Field(description="Material existente (robot_list_materials con only_used=false). Omitir = el de Robot por defecto.")
        ] = None,
        iy_factor: Annotated[
            float | None, Field(gt=0, le=1, description="Coeficiente de inercia agrietada en el eje fuerte (NEC: 0.5 vigas, 0.8 columnas).")
        ] = None,
        iz_factor: Annotated[float | None, Field(gt=0, le=1, description="Coeficiente de inercia agrietada en el eje débil.")] = None,
        overwrite: Annotated[
            bool, Field(description="true = redefinir una sección que ya existe (cambia la rigidez de las barras que la usan).")
        ] = False,
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Crea (o redefine, con overwrite) una sección rectangular de hormigón con inercia agrietada.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, action (create |
        redefine), section:{name, kind, b_cm, h_cm, material?, inertia_reduction?}, bars_affected,
        current?, next_step? | section_after, results_status_after}. section_after es la sección
        releída de Robot, con iy/iz guardadas y su relación con la inercia bruta. Crear una
        sección nueva no cambia ninguna barra: asígnala con robot_assign_section o al crear barras.
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            labels = s.structure.Labels
            lt, material_type = om.IRobotLabelType.I_LT_BAR_SECTION, om.IRobotLabelType.I_LT_MATERIAL
            exists = bool(labels.Exist(lt, name))
            current = section_item(s, labels.Get(lt, name)) if exists else None
            affected = current["bars_count"] if current else 0
            if exists and not overwrite:
                raise ToolError(
                    f"La sección '{name}' ya existe y la usan {affected} barras. Usa otro nombre, o overwrite=true "
                    "para redefinirla (cambiará la rigidez de esas barras)."
                )
            if material and not (labels.Exist(material_type, material) or labels.IsAvailable(material_type, material)):
                raise ToolError(f"No existe el material '{material}'. Consulta robot_list_materials con only_used=false.")
            reduce = iy_factor is not None or iz_factor is not None
            wanted: dict[str, Any] = {"name": name, "kind": kind, "b_cm": b_cm, "h_cm": h_cm}
            if material:
                wanted["material"] = material
            if reduce:
                wanted["inertia_reduction"] = {"ix": 1.0, "iy": iy_factor or 1.0, "iz": iz_factor or 1.0}
            plan: dict[str, Any] = {"action": "redefine" if exists else "create", "section": wanted, "bars_affected": affected}
            if current:
                plan["current"] = current

            def apply() -> dict[str, Any]:
                cv = om.IRobotBarSectionConcreteDataValue
                shapes = om.IRobotBarSectionShapeType
                label = labels.Get(lt, name) if exists else labels.Create(lt, name)
                data = om.IRobotBarSectionData(label.Data)
                data.ShapeType = shapes.I_BSST_CONCR_COL_R if kind == "column" else shapes.I_BSST_CONCR_BEAM_RECT
                concrete = data.Concrete
                concrete.SetValue(cv.I_BSCDV_BEAM_B, b_cm / 100)
                concrete.SetValue(cv.I_BSCDV_BEAM_H, h_cm / 100)
                concrete.SetReduction(reduce, 1.0, iy_factor or 1.0, iz_factor or 1.0)
                if material:
                    data.MaterialName = material
                data.CalcNonstdGeometry()
                labels.Store(label)
                after = section_item(s, labels.Get(lt, name))
                if abs(after.get("b_cm", 0) - b_cm) > 0.01 or abs(after.get("h_cm", 0) - h_cm) > 0.01:
                    raise ToolError(f"Robot guardó la sección con otras dimensiones: {after}.")
                return {"section_after": after}

            return _execute(
                s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply, affects_results=affected > 0
            )

        return await on_robot(work)

    @mcp.tool(name="robot_assign_section", title="Asignar sección a barras", annotations=OVERWRITE)
    async def robot_assign_section(
        selection: Annotated[str, Field(description="Barras a modificar: '1to8', '9 12 15'. Obligatorio.")],
        section: Annotated[str, Field(description="Sección existente (ver robot_list_sections con only_used=false).")],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Asigna una sección existente a las barras indicadas, reemplazando la que tuvieran.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, section, bars_count,
        not_found, current_sections:{nombre: nº de barras}, bars_changing, next_step? | assigned,
        results_status_after}. current_sections es lo que tienen hoy esas barras: muéstralo al
        usuario antes de aplicar, porque la asignación anterior se pierde.
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            bar_type, lt = om.IRobotObjectType.I_OT_BAR, om.IRobotLabelType.I_LT_BAR_SECTION
            chosen, count, not_found = _resolve(s, bar_type, selection, "barras")
            if not s.structure.Labels.Exist(lt, section):
                raise ToolError(f"No existe la sección '{section}'. Consulta robot_list_sections con only_used=false.")
            current = _usage(s, bar_type, lt, selection)
            plan = {
                "section": section,
                "bars_count": count,
                "not_found": not_found,
                "current_sections": current,
                "bars_changing": count - current.get(section, 0),
            }

            def apply() -> dict[str, Any]:
                s.structure.Bars.SetLabel(chosen, lt, section)
                assigned = s.select(bar_type, selection, label=(lt, section)).Count
                if assigned != count:
                    raise ToolError(f"Robot asignó la sección a {assigned} de {count} barras; revisa el modelo.")
                return {"assigned": assigned}

            return _execute(
                s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply, affects_results=plan["bars_changing"] > 0
            )

        return await on_robot(work)

    @mcp.tool(name="robot_define_support", title="Definir tipo de apoyo", annotations=OVERWRITE)
    async def robot_define_support(
        name: Annotated[str, Field(min_length=1, max_length=40, description="Nombre del apoyo, p. ej. 'Empotrado'.")],
        ux: Annotated[bool, Field(description="Desplazamiento X restringido.")] = True,
        uy: Annotated[bool, Field(description="Desplazamiento Y restringido.")] = True,
        uz: Annotated[bool, Field(description="Desplazamiento Z restringido.")] = True,
        rx: Annotated[bool, Field(description="Giro X restringido.")] = True,
        ry: Annotated[bool, Field(description="Giro Y restringido.")] = True,
        rz: Annotated[bool, Field(description="Giro Z restringido.")] = True,
        overwrite: Annotated[bool, Field(description="true = redefinir un apoyo existente (afecta a los nudos que lo usan).")] = False,
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Crea (o redefine, con overwrite) un tipo de apoyo rígido indicando qué grados de libertad restringe.

        Por defecto, empotramiento (los seis restringidos); articulado = rx, ry, rz en false.
        Devuelve {dry_run, applied, results_status_before, invalidates_results, action, support:{name,
        fixed}, nodes_affected, next_step? | results_status_after}. Definirlo no lo coloca en
        ningún nudo: usa robot_assign_support.
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            labels = s.structure.Labels
            lt = om.IRobotLabelType.I_LT_SUPPORT
            exists = bool(labels.Exist(lt, name))
            affected = s.structure.Selections.CreateByLabel(om.IRobotObjectType.I_OT_NODE, lt, name).Count if exists else 0
            if exists and not overwrite:
                raise ToolError(
                    f"El apoyo '{name}' ya existe y lo usan {affected} nudos. Usa otro nombre, o overwrite=true para redefinirlo."
                )
            fixed = dict(zip(DOFS, (ux, uy, uz, rx, ry, rz)))
            plan = {"action": "redefine" if exists else "create", "support": {"name": name, "fixed": fixed}, "nodes_affected": affected}

            def apply() -> dict[str, Any]:
                label = labels.Get(lt, name) if exists else labels.Create(lt, name)
                data = om.IRobotNodeSupportData(label.Data)
                for dof, value in fixed.items():
                    setattr(data, dof, 1 if value else 0)
                labels.Store(label)
                stored = om.IRobotNodeSupportData(labels.Get(lt, name).Data)
                if {dof: bool(getattr(stored, dof)) for dof in DOFS} != fixed:
                    raise ToolError("Robot no guardó el apoyo con las restricciones pedidas; revísalo en Robot.")
                return {}

            return _execute(
                s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply, affects_results=affected > 0
            )

        return await on_robot(work)

    @mcp.tool(name="robot_assign_support", title="Asignar apoyo a nudos", annotations=OVERWRITE)
    async def robot_assign_support(
        selection: Annotated[str, Field(description="Nudos a apoyar: '1to4', '1 3 5'. Obligatorio.")],
        support: Annotated[str, Field(description="Apoyo existente (ver robot_list_supports con only_used=false).")],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Asigna un tipo de apoyo existente a los nudos indicados, reemplazando el que tuvieran.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, support, nodes_count,
        not_found, current_supports:{nombre: nº de nudos}, nodes_changing, next_step? | assigned,
        results_status_after}.
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            node_type, lt = om.IRobotObjectType.I_OT_NODE, om.IRobotLabelType.I_LT_SUPPORT
            chosen, count, not_found = _resolve(s, node_type, selection, "nudos")
            if not s.structure.Labels.Exist(lt, support):
                raise ToolError(f"No existe el apoyo '{support}'. Consulta robot_list_supports con only_used=false o defínelo.")
            current = _usage(s, node_type, lt, selection)
            plan = {
                "support": support,
                "nodes_count": count,
                "not_found": not_found,
                "current_supports": current,
                "nodes_changing": count - current.get(support, 0),
            }

            def apply() -> dict[str, Any]:
                s.structure.Nodes.SetLabel(chosen, lt, support)
                assigned = s.select(node_type, selection, label=(lt, support)).Count
                if assigned != count:
                    raise ToolError(f"Robot asignó el apoyo a {assigned} de {count} nudos; revisa el modelo.")
                return {"assigned": assigned}

            return _execute(
                s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply, affects_results=plan["nodes_changing"] > 0
            )

        return await on_robot(work)

    # ---------- cargas ----------

    @mcp.tool(name="robot_create_load_case", title="Crear caso de carga", annotations=WRITE)
    async def robot_create_load_case(
        name: Annotated[str, Field(min_length=1, max_length=60, description="Nombre del caso, p. ej. 'Carga Muerta'.")],
        nature: Annotated[Nature, Field(description="Naturaleza: PERMANENT (muerta), LIVE (viva), WIND, SNOW, SEISMIC…")],
        number: Annotated[int | None, Field(ge=1, description="Número de caso. Omitir = siguiente libre.")] = None,
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Crea un caso de carga simple de análisis estático lineal, vacío.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, case:{number, name,
        nature}, next_step? | results_status_after}. Añádele cargas con robot_add_loads. No crea
        casos modales ni espectrales (se definen en Robot).
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            server = s.structure.Cases
            table = _case_table(s)
            if number is not None and number in table:
                raise ToolError(f"Ya existe el caso {number} ('{table[number][0]}'). Usa otro número u omítelo.")
            same = [n for n, (existing, _) in table.items() if existing.strip().lower() == name.lower()]
            if same:
                raise ToolError(f"Ya existe un caso llamado '{name}' (número {same[0]}). Usa otro nombre.")
            chosen = number or _free_numbers(server, set())()
            plan = {"case": {"number": chosen, "name": name, "nature": nature}}

            def apply() -> dict[str, Any]:
                server.CreateSimple(chosen, name, getattr(om.IRobotCaseNature, NATURES[nature]), om.IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR)
                if not server.Exist(chosen):
                    raise ToolError(f"Robot no creó el caso {chosen}.")
                return {}

            return _execute(s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply)

        return await on_robot(work)

    @mcp.tool(name="robot_add_loads", title="Añadir cargas a un caso", annotations=WRITE)
    async def robot_add_loads(
        case: Annotated[int, Field(ge=1, description="Número de un caso simple estático (ver robot_list_load_cases).")],
        loads: Annotated[
            list[LoadIn],
            Field(
                min_length=1,
                max_length=100,
                description="Cargas a añadir. type: 'self_weight' (peso propio), 'node_force' (fuerza/momento en nudos, kN y kN·m), "
                "'bar_uniform' (carga repartida en barras, kN/m) o 'panel_uniform' (carga superficial en paneles, kN/m²). "
                "Ejes globales; Z negativo = hacia abajo.",
            ),
        ],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Añade cargas a un caso simple existente. No modifica ni borra las cargas que ya tenga.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, case:{number, name},
        records_before, loads:[{type, objects?, objects_count?, valores…}], next_step? | records_after,
        results_status_after}. records_* cuentan solo las cargas del usuario: si los paneles son de
        reparto, Robot genera además las cargas sobre las vigas al calcular. Tras aplicar, comprueba
        el total con robot_get_reactions una vez recalculado.
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            ot = om.IRobotObjectType
            table = _case_table(s)
            if case not in table:
                raise ToolError(f"No existe el caso {case}. Consulta robot_list_load_cases o créalo.")
            robot_case = s.structure.Cases.Get(case)
            if table[case][1] != "SIMPLE" or enum_name(robot_case.AnalizeType) != "STATIC_LINEAR":
                raise ToolError(f"El caso {case} ('{table[case][0]}') no es un caso simple estático; no admite estas cargas.")
            records = om.IRobotSimpleCase(robot_case).Records

            def user_records() -> int:  # sin las que Robot genera al repartir paneles: cambian al modificar
                return int(records.Count) - len(generated_records(om, records))

            before = user_records()
            described: list[dict[str, Any]] = []
            writers: list[Callable[[], None]] = []
            kinds = om.IRobotLoadRecordType

            def writer(kind: Any, values: dict[Any, float], targets: Any | None) -> Callable[[], None]:
                def write() -> None:
                    record = records.Get(records.New(kind))
                    for key, value in values.items():
                        record.SetValue(int(key), value)
                    if targets is not None:
                        record.Objects.Add(targets)

                return write

            for load in loads:
                if isinstance(load, SelfWeightLoad):
                    dead = om.IRobotDeadRecordValues
                    described.append({"type": "self_weight", "factor": load.factor, "direction": "-Z", "objects": "toda la estructura"})
                    writers.append(writer(kinds.I_LRT_DEAD, {dead.I_DRV_ENTIRE_STRUCTURE: 1.0, dead.I_DRV_Z: -load.factor}, None))
                elif isinstance(load, NodeForceLoad):
                    force = om.IRobotNodeForceRecordValues
                    values = {
                        force.I_NFRV_FX: load.fx_kn, force.I_NFRV_FY: load.fy_kn, force.I_NFRV_FZ: load.fz_kn,
                        force.I_NFRV_CX: load.mx_knm, force.I_NFRV_CY: load.my_knm, force.I_NFRV_CZ: load.mz_knm,
                    }  # fmt: skip
                    if not any(values.values()):
                        raise ToolError("Una carga 'node_force' tiene todas sus componentes en cero.")
                    targets, count, not_found = _resolve(s, ot.I_OT_NODE, load.nodes, "nudos")
                    described.append({**load.model_dump(exclude={"nodes"}), "objects": canonical_selection(str(targets.ToText())), "objects_count": count, "not_found": not_found})
                    writers.append(writer(kinds.I_LRT_NODE_FORCE, {k: v * 1e3 for k, v in values.items() if v}, targets))
                elif isinstance(load, PanelUniformLoad):
                    surface = om.IRobotUniformRecordValues
                    values = {surface.I_URV_PX: load.px_kn_m2, surface.I_URV_PY: load.py_kn_m2, surface.I_URV_PZ: load.pz_kn_m2}
                    if not any(values.values()):
                        raise ToolError("Una carga 'panel_uniform' tiene todas sus componentes en cero.")
                    targets, count, not_found = _resolve(s, ot.I_OT_PANEL, load.panels, "paneles")
                    scaled = {k: v * 1e3 for k, v in values.items() if v}
                    if load.local:
                        scaled[surface.I_URV_LOCAL_SYSTEM] = 1.0
                    described.append({**load.model_dump(exclude={"panels"}), "objects": canonical_selection(str(targets.ToText())), "objects_count": count, "not_found": not_found})
                    writers.append(writer(kinds.I_LRT_UNIFORM, scaled, targets))
                else:
                    uniform = om.IRobotBarUniformRecordValues
                    values = {uniform.I_BURV_PX: load.px_kn_m, uniform.I_BURV_PY: load.py_kn_m, uniform.I_BURV_PZ: load.pz_kn_m}
                    if not any(values.values()):
                        raise ToolError("Una carga 'bar_uniform' tiene todas sus componentes en cero.")
                    targets, count, not_found = _resolve(s, ot.I_OT_BAR, load.bars, "barras")
                    scaled = {k: v * 1e3 for k, v in values.items() if v}
                    if load.local:
                        scaled[uniform.I_BURV_LOCAL] = 1.0
                    described.append({**load.model_dump(exclude={"bars"}), "objects": canonical_selection(str(targets.ToText())), "objects_count": count, "not_found": not_found})
                    writers.append(writer(kinds.I_LRT_BAR_UNIFORM, scaled, targets))
            plan = {"case": {"number": case, "name": table[case][0]}, "records_before": before, "loads": described}

            def apply() -> dict[str, Any]:
                for write in writers:
                    write()
                after = user_records()
                if after != before + len(writers):
                    raise ToolError(f"El caso tiene {after} cargas y se esperaban {before + len(writers)}; revísalo en Robot.")
                return {"records_after": after}

            return _execute(s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply)

        return await on_robot(work)

    @mcp.tool(name="robot_create_combinations", title="Crear combinaciones de carga", annotations=WRITE)
    async def robot_create_combinations(
        combinations: Annotated[list[CombinationIn], Field(min_length=1, max_length=50, description="Combinaciones a crear.")],
        dry_run: DryRun = True,
    ) -> dict[str, Any]:
        """Crea combinaciones manuales (lineales) con sus factores por caso. No invalida resultados.

        Devuelve {dry_run, applied, results_status_before, invalidates_results:false, to_create:[{number,
        name, type, nature, expression, factors}], next_step? | created, results_status_after}.
        Robot calcula las combinaciones al vuelo, así que quedan disponibles sin recalcular si los
        casos que usan ya tienen resultados. Falla sin crear nada si un caso no existe o un nombre
        o número ya está usado. Las combinaciones de norma (NEC, ACI) hay que darlas ya desarrolladas.
        """

        return await on_robot(lambda s: _combinations(s, combinations, dry_run))

    @mcp.tool(name="robot_create_nec_combinations", title="Crear las combinaciones NEC-SE-CG", annotations=WRITE)
    async def robot_create_nec_combinations(
        dead: Annotated[list[int], Field(min_length=1, max_length=20, description="Casos de carga muerta D (peso propio y sobrecarga permanente); se suman.")],
        live: Annotated[list[int], Field(max_length=20, description="Casos de carga viva L; se suman.")] = [],  # noqa: B006
        roof_live: Annotated[list[int], Field(max_length=20, description="Casos de carga viva de cubierta Lr (o granizo); se suman.")] = [],  # noqa: B006
        wind: Annotated[list[int], Field(max_length=8, description="Casos de viento W; cada uno entra por separado.")] = [],  # noqa: B006
        seismic_x: Annotated[list[int], Field(max_length=6, description="Casos sísmicos en X (estático o espectral); cada uno por separado.")] = [],  # noqa: B006
        seismic_y: Annotated[list[int], Field(max_length=6, description="Casos sísmicos en Y; cada uno por separado.")] = [],  # noqa: B006
        seismic_live_factor: Annotated[
            float, Field(ge=0.5, le=1.0, description="Factor de L en 1.2D+E+L: 1.0, o 0.5 donde la NEC lo permite (L ≤ 4.8 kN/m², salvo garajes y lugares de reunión).")
        ] = 1.0,
        seismic_scale_x: Annotated[
            float, Field(gt=0, le=100, description="Escala de los casos sísmicos X (p. ej. I·g/R, o el ajuste al cortante estático). 1 = el caso ya está escalado.")
        ] = 1.0,
        seismic_scale_y: Annotated[float, Field(gt=0, le=100, description="Escala de los casos sísmicos Y; igual que seismic_scale_x.")] = 1.0,
        both_signs: Annotated[bool, Field(description="true = cada sismo con + y − (los espectrales no tienen signo físico).")] = True,
        orthogonal: Annotated[bool, Field(description="true = 100 % de una dirección + 30 % de la otra, emparejando seismic_x[i] con seismic_y[i].")] = False,
        service: Annotated[bool, Field(description="true = añadir la combinación de servicio D+L (SLS) para flechas.")] = True,
        prefix: Annotated[str, Field(min_length=1, max_length=10, description="Prefijo del nombre de cada combinación.")] = "NEC",
        dry_run: DryRun = True,
    ) -> dict[str, Any]:
        """Genera y crea las combinaciones LRFD de la NEC-SE-CG §3.4.3 a partir de los casos indicados.

        1.4D · 1.2D+1.6L+0.5Lr · 1.2D+1.6Lr+(L o 0.5W) · 1.2D+W+L+0.5Lr · 1.2D±E+L · 0.9D±W ·
        0.9D±E, más D+L de servicio. Las categorías vacías se omiten. Devuelve lo mismo que
        robot_create_combinations: {dry_run, applied, …, to_create:[{number, name, type, nature,
        expression, factors}], next_step? | created}. El nombre lleva el número de caso (E7 = caso
        7). Si la escala del espectro (I·g/R, ajuste al cortante estático) no está en los casos
        sísmicos sino en las combinaciones, pásala en seismic_scale_x/y: multiplica el factor de cada
        caso sísmico, también en el 30 % ortogonal. Es un dato del proyecto; si no lo sabes, pregunta
        o copia el de las combinaciones existentes (robot_list_combinations). No invalida resultados. Falla sin crear nada si ya existe una combinación con el mismo nombre
        (cambia prefix) o si un caso no existe.
        """
        combos = nec_combinations(
            dead, live, roof_live, wind, seismic_x, seismic_y,
            seismic_live_factor=seismic_live_factor, seismic_scale_x=seismic_scale_x, seismic_scale_y=seismic_scale_y, both_signs=both_signs, orthogonal=orthogonal, service=service, prefix=prefix,
        )  # fmt: skip
        if len(combos) > 80:
            raise ToolError(f"Saldrían {len(combos)} combinaciones (máximo 80). Usa menos casos de viento o sismo, o orthogonal=false.")
        return await on_robot(lambda s: _combinations(s, combos, dry_run))

    # ---------- paneles ----------

    @mcp.tool(name="robot_create_panel", title="Crear panel (losa o muro)", annotations=WRITE)
    async def robot_create_panel(
        points: Annotated[list[PointIn], Field(min_length=3, max_length=50, description="Vértices del contorno en orden, en m, sin repetir el primero.")],
        thickness: Annotated[str, Field(description="Espesor existente (robot_list_thicknesses).")],
        calc_model: Annotated[
            str | None,
            Field(description="Modelo de cálculo existente (robot_list_panels muestra los usados), p. ej. el de reparto de cargas sin rigidez. Omitir = elementos finitos, el de Robot por defecto."),
        ] = None,
        number: Annotated[int | None, Field(ge=1, description="Número del panel. Omitir = siguiente libre.")] = None,
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Crea un panel plano por su contorno, con su espesor y, opcionalmente, su modelo de cálculo.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, panel:{number,
        thickness, calc_model?, area_m2, vertices, warning?}, next_step? | panel_after, results_status_after}.
        Falla sin crear nada si los puntos no son coplanares (tolerancia 1 mm), el contorno no tiene
        área, el espesor o el modelo de cálculo no existen o el número ya está usado. Las cargas
        superficiales se añaden luego con robot_add_loads (type 'panel_uniform').
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            lt = om.IRobotLabelType
            labels, server = s.structure.Labels, s.structure.Objects
            if not labels.Exist(lt.I_LT_PANEL_THICKNESS, thickness):
                raise ToolError(f"No existe el espesor '{thickness}'. Consulta robot_list_thicknesses o defínelo en Robot.")
            if calc_model and not labels.Exist(lt.I_LT_PANEL_CALC_MODEL, calc_model):
                raise ToolError(f"No existe el modelo de cálculo '{calc_model}'. Los usados aparecen en robot_list_panels; defínelo en Robot.")
            if number is not None and server.Exist(number):
                raise ToolError(f"Ya existe un objeto con el número {number}. Usa otro u omítelo.")
            vertices = [(p.x, p.y, p.z) for p in points]
            if len({tuple(round(c, 6) for c in v) for v in vertices}) != len(vertices):
                raise ToolError("Hay vértices repetidos en el contorno.")
            normal = [0.0, 0.0, 0.0]  # método de Newell: normal y doble del área del polígono
            for (x1, y1, z1), (x2, y2, z2) in zip(vertices, vertices[1:] + vertices[:1]):
                normal[0] += (y1 - y2) * (z1 + z2)
                normal[1] += (z1 - z2) * (x1 + x2)
                normal[2] += (x1 - x2) * (y1 + y2)
            size = math.hypot(*normal)
            if size < 1e-6:
                raise ToolError("El contorno no tiene área (puntos alineados).")
            off = max(abs(sum(n / size * (c - c0) for n, c, c0 in zip(normal, v, vertices[0]))) for v in vertices)
            if off > 0.001:
                raise ToolError(f"Los vértices no son coplanares: uno se separa {off * 1e3:.1f} mm del plano.")
            chosen = number or _free_numbers(server, set())()
            panel_plan: dict[str, Any] = {"number": chosen, "thickness": thickness, "area_m2": num(size / 2, 3), "vertices": len(vertices)}
            if calc_model:
                panel_plan["calc_model"] = calc_model
            if not str(om.IRobotThicknessData(labels.Get(lt.I_LT_PANEL_THICKNESS, thickness).Data).MaterialName):
                panel_plan["warning"] = "El espesor no tiene material: con elementos finitos el cálculo fallará en la verificación."

            def apply() -> dict[str, Any]:
                contour = om.IRobotPointsArray(s.app.CmpntFactory.Create(om.IRobotComponentType.I_CT_POINTS_ARRAY))
                contour.SetSize(len(vertices))
                for i, (x, y, z) in enumerate(vertices, start=1):
                    contour.Set(i, x, y, z)
                server.CreateContour(chosen, contour)
                if not server.Exist(chosen):
                    raise ToolError(f"Robot no creó el panel {chosen}.")
                panel = om.IRobotObjObject(server.Get(chosen))
                panel.Main.Attribs.Meshed = 1
                panel.SetLabel(lt.I_LT_PANEL_THICKNESS, thickness)
                if calc_model:
                    panel.SetLabel(lt.I_LT_PANEL_CALC_MODEL, calc_model)
                panel.Initialize()
                panel.Update()
                after = {"number": chosen, "thickness": str(panel.GetLabelName(lt.I_LT_PANEL_THICKNESS))}
                if calc_model:
                    after["calc_model"] = str(panel.GetLabelName(lt.I_LT_PANEL_CALC_MODEL))
                if after.get("thickness") != thickness or after.get("calc_model", calc_model) != calc_model:
                    raise ToolError(f"Robot creó el panel pero no guardó sus etiquetas: {after}. Revísalo en Robot.")
                return {"panel_after": after}

            return _execute(s, dry_run=dry_run, discard_results=discard_results, plan={"panel": panel_plan}, apply=apply)

        return await on_robot(work)

    # ---------- borrado ----------

    @mcp.tool(name="robot_delete_elements", title="Borrar barras, nudos, paneles o casos", annotations=OVERWRITE)
    async def robot_delete_elements(
        kind: Annotated[Literal["bars", "nodes", "panels", "cases"], Field(description="Qué borrar: barras, nudos, paneles o casos/combinaciones.")],
        selection: Annotated[str, Field(description="Números a borrar: '17to20', '5 9'. Obligatorio; no se admite 'todos'.")],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Borra elementos del modelo. NO hay deshacer: haz el ensayo, enséñalo y aplica solo con permiso expreso.

        Devuelve {dry_run, applied, results_status_before, invalidates_results, kind, count, numbers,
        not_found, cases?:[{number, name, type}], next_step? | deleted, results_status_after}.
        Se niega a borrar nudos que usan barras (borra antes las barras) y casos que usa una
        combinación que no se borra a la vez. Borrar solo combinaciones no invalida resultados.
        Recomienda al usuario guardar antes (robot_save_project con path, como copia).
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            ot = om.IRobotObjectType
            obj_type, server, what = {
                "bars": (ot.I_OT_BAR, s.structure.Bars, "barras"),
                "nodes": (ot.I_OT_NODE, s.structure.Nodes, "nudos"),
                "panels": (ot.I_OT_PANEL, s.structure.Objects, "paneles"),
                "cases": (ot.I_OT_CASE, s.structure.Cases, "casos"),
            }[kind]
            chosen, count, not_found = _resolve(s, obj_type, selection, what)
            numbers = [int(chosen.Get(i)) for i in range(1, count + 1)]
            plan: dict[str, Any] = {"kind": kind, "count": count, "numbers": canonical_selection(str(chosen.ToText())), "not_found": not_found}
            affects = True
            if kind == "nodes":
                used = server.GetConnectedBars(str(chosen.ToText()))
                if used.Count:
                    bars = sorted({int(used.Get(i)) for i in range(1, used.Count + 1)})
                    raise ToolError(f"Esos nudos los usan las barras {bars[:30]}{'…' if len(bars) > 30 else ''}. Bórralas antes.")
            if kind == "cases":
                table = _case_table(s)
                plan["cases"] = [{"number": n, "name": table[n][0], "type": table[n][1]} for n in numbers]
                doomed = set(numbers)
                for n, (name, case_type) in table.items():
                    if case_type != "COMBINATION" or n in doomed:
                        continue
                    factors = om.IRobotCaseCombination(server.Get(n)).CaseFactors
                    used = {int(factors.Get(j).CaseNumber) for j in range(1, factors.Count + 1)} & doomed
                    if used:
                        raise ToolError(f"La combinación {n} ('{name}') usa los casos {sorted(used)}. Bórrala a la vez o quita esos casos.")
                affects = any(table[n][1] != "COMBINATION" for n in numbers)

            def apply() -> dict[str, Any]:
                server.DeleteMany(chosen)
                left = [n for n in numbers if server.Exist(n)]
                if left:
                    raise ToolError(f"Robot no borró {left[:30]}; revisa el modelo.")
                return {"deleted": len(numbers)}

            return _execute(s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply, affects_results=affects)

        return await on_robot(work)

    @mcp.tool(name="robot_delete_loads", title="Borrar cargas de un caso", annotations=OVERWRITE)
    async def robot_delete_loads(
        case: Annotated[int, Field(ge=1, description="Número del caso simple.")],
        indexes: Annotated[list[int], Field(min_length=1, max_length=100, description="Campo 'index' de las cargas en robot_list_loads.")],
        dry_run: DryRun = True,
        discard_results: DiscardResults = False,
    ) -> dict[str, Any]:
        """Borra cargas (registros) de un caso por su índice. NO hay deshacer.

        Consulta antes robot_list_loads para obtener los índices: cambian tras cada borrado, así que
        vuelve a listar antes de borrar otra vez. Devuelve {dry_run, applied, results_status_before,
        invalidates_results, case:{number, name}, records_before, to_delete:[{index, type, objects}],
        next_step? | records_after, results_status_after}. Se niega a borrar las cargas que Robot
        genera al repartir paneles (desaparecen solas al borrar la carga del panel).
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            table = _case_table(s)
            if case not in table:
                raise ToolError(f"No existe el caso {case}. Consulta robot_list_load_cases.")
            if table[case][1] != "SIMPLE":
                raise ToolError(f"El caso {case} es una combinación; no tiene cargas propias.")
            records = om.IRobotSimpleCase(s.structure.Cases.Get(case)).Records
            generated = generated_records(om, records)
            wrong = sorted({i for i in indexes if not 1 <= i <= records.Count})
            if wrong:
                raise ToolError(f"El caso {case} no tiene las cargas {wrong} (tiene {records.Count}). Consulta robot_list_loads.")
            robot_made = sorted(set(indexes) & generated)
            if robot_made:
                raise ToolError(f"Las cargas {robot_made} las genera Robot al repartir paneles; borra la carga del panel.")
            doomed = sorted(set(indexes), reverse=True)  # de atrás adelante: los índices se corren al borrar
            listed = []
            for index in sorted(doomed):
                record = records.Get(index)
                try:
                    objects = canonical_selection(str(record.Objects.ToText())) or None
                except Exception:  # registro sin elementos (espectro)
                    objects = None
                listed.append({"index": index, "type": enum_name(record.Type), "objects": objects})
            before = int(records.Count) - len(generated)
            plan = {"case": {"number": case, "name": table[case][0]}, "records_before": before, "to_delete": listed}

            def apply() -> dict[str, Any]:
                for index in doomed:
                    records.Delete(index)
                after = int(records.Count) - len(generated_records(om, records))
                if after != before - len(doomed):
                    raise ToolError(f"El caso tiene {after} cargas y se esperaban {before - len(doomed)}; revísalo en Robot.")
                return {"records_after": after}

            return _execute(s, dry_run=dry_run, discard_results=discard_results, plan=plan, apply=apply)

        return await on_robot(work)

    # ---------- cálculo y guardado ----------

    @mcp.tool(name="robot_run_calculation", title="Calcular el modelo", annotations=WRITE)
    async def robot_run_calculation(
        timeout_s: Annotated[int, Field(ge=30, le=3600, description="Segundos máximos de espera al cálculo.")] = 600,
    ) -> dict[str, Any]:
        """Lanza el cálculo completo del modelo en Robot y espera a que termine.

        Devuelve {calculation (COMPLETED | FAILED_VERIFICATION | FAILED_CALCULATION | FAILED_GENERATION |
        CANCELLED_BY_USER…), duration_s, results_status, hint?}. Sustituye los resultados que hubiera.
        Mientras calcula, Robot no responde a otras herramientas. Si no termina en COMPLETED, la
        causa está en la ventana «Mensajes de cálculo» de Robot (inestabilidades, barras sin sección,
        nudos sueltos…). Pide confirmación al usuario antes de lanzarlo en modelos grandes.
        """

        def work(s: Session) -> dict[str, Any]:
            started = time.time()
            with non_interactive(s.app):
                outcome = enum_name(s.project.CalcEngine.CalculateEx(s.om.IRobotCalculationMode.I_CM_LOCAL))
            out: dict[str, Any] = {
                "calculation": outcome,
                "duration_s": num(time.time() - started, 1),
                "results_status": enum_name(s.structure.Results.Status),
            }
            if outcome != "COMPLETED":
                out["hint"] = "El cálculo no terminó bien. Revisa la ventana «Mensajes de cálculo» en Robot y corrige el modelo."
            return out

        return await on_robot(work, timeout=timeout_s)

    @mcp.tool(name="robot_save_project", title="Guardar el proyecto", annotations=OVERWRITE)
    async def robot_save_project(
        path: Annotated[
            str | None,
            Field(description="Ruta .rtd para 'guardar como'. Omitir = guardar sobre el archivo actual del proyecto."),
        ] = None,
        overwrite: Annotated[bool, Field(description="true = permitir que 'guardar como' reemplace un archivo existente.")] = False,
        dry_run: DryRun = True,
    ) -> dict[str, Any]:
        """Guarda el proyecto en disco: sobre su archivo actual o, con path, como archivo nuevo.

        Devuelve {dry_run, applied, action (save | save_as), file, file_exists, next_step? |
        size_kb, results_status_after}. Guardar sobre el archivo actual reemplaza la versión en
        disco: hazlo solo si el usuario lo pidió. 'Guardar como' deja el proyecto de Robot
        apuntando al archivo nuevo.
        """

        def work(s: Session) -> dict[str, Any]:
            current = str(s.project.FileName or "")
            if path is None:
                if not current:
                    raise ToolError("El proyecto no se ha guardado nunca. Indica 'path' con la ruta .rtd de destino.")
                target, action = current, "save"
            else:
                target, action = os.path.abspath(path), "save_as"
                if not target.lower().endswith(".rtd"):
                    raise ToolError("La ruta debe terminar en .rtd.")
                if not os.path.isdir(os.path.dirname(target)):
                    raise ToolError(f"No existe la carpeta {os.path.dirname(target)}.")
                same = os.path.normcase(target) == os.path.normcase(current)
                if os.path.exists(target) and not overwrite and not same:
                    raise ToolError(f"Ya existe {target}. Usa otra ruta, o overwrite=true si el usuario quiere reemplazarlo.")
                if same:
                    action = "save"
            plan = {"action": action, "file": target, "file_exists": os.path.exists(target)}

            def apply() -> dict[str, Any]:
                started = time.time()
                if action == "save":
                    s.project.Save()
                else:
                    s.project.SaveAs(target)
                if not os.path.isfile(target) or os.path.getmtime(target) < started - 2:
                    raise ToolError(f"Robot no escribió el archivo {target}.")
                return {"size_kb": round(os.path.getsize(target) / 1024)}

            out = _execute(s, dry_run=dry_run, discard_results=False, plan=plan, apply=apply, affects_results=False)
            out.pop("invalidates_results", None)
            return out

        return await on_robot(work)
