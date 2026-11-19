"""Herramientas de resultados: reacciones, desplazamientos, esfuerzos, envolventes, modos y pisos.

Solo lectura: nunca lanzan el cálculo. La API de Robot devuelve N, N·m, m, rad y kg;
aquí se convierte a kN, kN·m, mm, rad y t.
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import Field

from .robot_com import Session, enum_name, num, on_robot, page, parse_selection, project_labels, window
from .robot_com import READ_ONLY as READ_ONLY_MODE
from .robot_com import bar_selection
from .schema import READ_ONLY, Cases, Limit, Offset, SectionFilter, Selection, StoreyFilter, StructuralTypeFilter

MAX_CASES = 30
MAX_READS = 600  # lecturas de resultado por llamada (~10 ms cada una por COM)

FORCES = ("FX", "FY", "FZ", "MX", "MY", "MZ")
DISPLACEMENTS = ("UX", "UY", "UZ", "RX", "RY", "RZ")


def _force_dict(data: Any) -> dict[str, float]:
    """FX..MZ de Robot (N, N·m) -> fx_kn..mz_knm."""
    return {
        f"{name.lower()}_{'kn' if name[0] == 'F' else 'knm'}": num(getattr(data, name) / 1e3, 3) for name in FORCES
    }


def _displacement_dict(data: Any) -> dict[str, float]:
    """UX..RZ de Robot (m, rad) -> ux_mm..rz_rad."""
    out: dict[str, float] = {}
    for name in DISPLACEMENTS:
        if name[0] == "U":
            out[f"{name.lower()}_mm"] = num(getattr(data, name) * 1e3, 4)
        else:
            out[f"{name.lower()}_rad"] = num(getattr(data, name), 7)
    return out


def _results(s: Session) -> tuple[Any, str]:
    """Servidor de resultados y su estado; falla con una instrucción clara si no hay resultados."""
    res = s.structure.Results
    status = enum_name(res.Status)
    if not res.Available:
        how = (
            "Calcula en Robot (Análisis > Calcular) y reintenta; este servidor está en solo lectura."
            if READ_ONLY_MODE
            else "Calcula con robot_run_calculation (pide antes permiso al usuario) o en Robot (Análisis > Calcular) y reintenta."
        )
        if status == "OUT_OF_DATE":  # Robot no entrega resultados caducados por la API
            raise ToolError(f"Los resultados están desactualizados: el modelo cambió después del último cálculo. {how}")
        raise ToolError(f"El modelo no tiene resultados. {how}")
    return res, status


def _cases(s: Session, text: str, strict: bool = False) -> list[dict[str, Any]]:
    """'5 101to108' -> [{number, name, analysis}] de los casos que existen.

    strict=True (verificaciones): falla si alguno de los pedidos no existe, en vez de omitirlo.
    """
    numbers: dict[int, None] = {}
    for first, last, step in parse_selection(text):
        if (last - first) // step > 5000:
            raise ToolError(f"El rango de casos {first}-{last} es demasiado amplio.")
        numbers.update(dict.fromkeys(range(first, last + 1, step)))
    server = s.structure.Cases
    out = []
    for number in numbers:
        if server.Exist(number):
            case = server.Get(number)
            out.append({"number": number, "name": str(case.Name), "analysis": enum_name(case.AnalizeType)})
    if not out:
        raise ToolError(f"Ninguno de los casos '{text}' existe. Consulta los números con robot_list_load_cases.")
    missing = sorted(set(numbers) - {c["number"] for c in out})
    if strict and missing:
        raise ToolError(f"No existen los casos {missing}. Consulta robot_list_load_cases.")
    if len(out) > MAX_CASES:
        raise ToolError(f"Demasiados casos ({len(out)}); el máximo por llamada es {MAX_CASES}.")
    return out


def _modal_case(s: Session) -> int | None:
    """Número del primer caso de análisis modal, o None si el modelo no tiene."""
    col = s.structure.Cases.GetAll()
    for i in range(1, col.Count + 1):
        case = s.om.IRobotCase(col.Get(i))
        if enum_name(case.AnalizeType) == "DYNAMIC_MODAL":
            return int(case.Number)
    return None


def _check_budget(elements: int, per_element: int, what: str) -> None:
    reads = elements * per_element
    if reads > MAX_READS:
        allowed = max(1, MAX_READS // per_element)
        raise ToolError(
            f"La consulta pide {reads} lecturas ({elements} {what} × {per_element}) y el máximo es {MAX_READS}. "
            f"Usa limit ≤ {allowed}, menos casos, o robot_get_result_extremes si buscas máximos."
        )


def _supported_nodes(s: Session, selection: str | None) -> Any:
    om = s.om
    node, support = om.IRobotObjectType.I_OT_NODE, om.IRobotLabelType.I_LT_SUPPORT
    factory = s.structure.Selections
    supported = factory.Create(node)
    for label in project_labels(s, support, only_used=False):
        supported.Add(factory.CreateByLabel(node, support, label.Name))
    if selection and selection.strip():
        chosen = s.select(node, selection)
        chosen.And(supported)
        return chosen
    return supported


def register(mcp: MCPServer) -> None:
    @mcp.tool(name="robot_get_reactions", title="Reacciones en apoyos", annotations=READ_ONLY)
    async def robot_get_reactions(
        cases: Cases,
        selection: Selection = None,
        limit: Limit = 50,
        offset: Offset = 0,
    ) -> dict[str, Any]:
        """Reacciones en los nudos apoyados (ejes globales, kN y kN·m) y su suma por caso.

        Devuelve {results_status, cases:[{number, name, analysis}], totals:[{case, sum_fx_kn,
        sum_fy_kn, sum_fz_kn}], total, count, offset, has_more, next_offset?,
        items:[{node, by_case:[{case, fx_kn, fy_kn, fz_kn, mx_knm, my_knm, mz_knm}]}]}.
        'totals' es la suma de reacciones de toda la estructura: en casos de gravedad sum_fz_kn
        es el peso aplicado; en casos sísmicos sum_fx_kn / sum_fy_kn es el cortante basal del caso.
        FZ positivo = hacia arriba. 'selection' filtra nudos; por defecto, todos los apoyados.
        'totals' no depende de la paginación: si solo quieres el peso o el cortante basal,
        llama con limit=1.
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, cases)
            col = s.structure.Nodes.GetMany(_supported_nodes(s, selection))
            total = col.Count
            indexes = window(total, offset, limit)
            _check_budget(len(indexes), len(case_list), "nudos")
            reactions = res.Nodes.Reactions
            items = []
            for i in indexes:
                node = s.om.IRobotNode(col.Get(i)).Number
                by_case = [{"case": c["number"], **_force_dict(reactions.Value(node, c["number"]))} for c in case_list]
                items.append({"node": node, "by_case": by_case})
            totals = []
            for c in case_list:
                total_reaction = reactions.Sum(c["number"])
                totals.append(
                    {
                        "case": c["number"],
                        "sum_fx_kn": num(total_reaction.FX / 1e3, 3),
                        "sum_fy_kn": num(total_reaction.FY / 1e3, 3),
                        "sum_fz_kn": num(total_reaction.FZ / 1e3, 3),
                    }
                )
            return {"results_status": status, "cases": case_list, "totals": totals, **page(total, offset, limit, items)}

        return await on_robot(work)

    @mcp.tool(name="robot_get_node_displacements", title="Desplazamientos de nudos", annotations=READ_ONLY)
    async def robot_get_node_displacements(
        cases: Cases,
        selection: Selection = None,
        limit: Limit = 50,
        offset: Offset = 0,
    ) -> dict[str, Any]:
        """Desplazamientos (mm) y giros (rad) de nudos en ejes globales, por caso.

        Devuelve {results_status, cases, total, count, offset, has_more, next_offset?,
        items:[{node, by_case:[{case, ux_mm, uy_mm, uz_mm, rx_rad, ry_rad, rz_rad}]}]}.
        Son los valores del caso tal como se analizó, sin amplificar. Para el máximo de toda
        la estructura usa robot_get_result_extremes; para derivas, robot_get_storey_results.
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, cases)
            col = s.structure.Nodes.GetMany(s.select(s.om.IRobotObjectType.I_OT_NODE, selection))
            total = col.Count
            indexes = window(total, offset, limit)
            _check_budget(len(indexes), len(case_list), "nudos")
            displacements = res.Nodes.Displacements
            items = []
            for i in indexes:
                node = s.om.IRobotNode(col.Get(i)).Number
                by_case = [
                    {"case": c["number"], **_displacement_dict(displacements.Value(node, c["number"]))}
                    for c in case_list
                ]
                items.append({"node": node, "by_case": by_case})
            return {"results_status": status, "cases": case_list, **page(total, offset, limit, items)}

        return await on_robot(work)

    @mcp.tool(name="robot_get_bar_forces", title="Esfuerzos en barras", annotations=READ_ONLY)
    async def robot_get_bar_forces(
        cases: Cases,
        selection: Selection = None,
        section: SectionFilter = None,
        storey: StoreyFilter = None,
        structural_type: StructuralTypeFilter = None,
        points: Annotated[
            int, Field(ge=2, le=11, description="Estaciones por barra, equiespaciadas de 0 a 1 (2 = solo extremos).")
        ] = 3,
        limit: Limit = 20,
        offset: Offset = 0,
    ) -> dict[str, Any]:
        """Esfuerzos internos de barras en ejes locales (kN y kN·m), por caso y estación.

        Devuelve {results_status, cases, total, count, offset, has_more, next_offset?,
        items:[{bar, by_case:[{case, stations:[{x, fx_kn, fy_kn, fz_kn, mx_knm, my_knm, mz_knm}]}]}]}.
        x es la posición relativa (0 = nudo inicial, 1 = nudo final; cuáles son, en
        robot_list_bars: en una columna el inicial suele ser la base). Convención de Robot:
        FX > 0 = compresión; MY = flexión principal por gravedad (alrededor del eje local y;
        negativa en apoyos, positiva en vano), MZ = flexión lateral; FZ = cortante vertical,
        FY = cortante lateral; MX = torsión. En casos espectrales el signo no es físico: usa el
        valor absoluto o las combinaciones. Cada estación cuesta una lectura lenta: para
        máximos de muchas barras usa robot_get_result_extremes.
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, cases)
            col = s.structure.Bars.GetMany(bar_selection(s, selection, section, storey, structural_type))
            total = col.Count
            indexes = window(total, offset, limit)
            _check_budget(len(indexes), len(case_list) * points, "barras")
            forces = res.Bars.Forces
            stations = [k / (points - 1) for k in range(points)]
            items = []
            for i in indexes:
                bar = s.om.IRobotBar(col.Get(i)).Number
                by_case = []
                for c in case_list:
                    values = [{"x": num(x, 3), **_force_dict(forces.Value(bar, c["number"], x))} for x in stations]
                    by_case.append({"case": c["number"], "stations": values})
                items.append({"bar": bar, "by_case": by_case})
            return {"results_status": status, "cases": case_list, **page(total, offset, limit, items)}

        return await on_robot(work)

    @mcp.tool(name="robot_get_bar_deflections", title="Flechas de vigas", annotations=READ_ONLY)
    async def robot_get_bar_deflections(
        cases: Cases,
        selection: Selection = None,
        section: SectionFilter = None,
        storey: StoreyFilter = None,
        structural_type: StructuralTypeFilter = None,
        limit_ratio: Annotated[
            float | None, Field(gt=0, description="Denominador del límite L/n a comprobar (ACI 24.2.2: 240, 360 o 480). Omitir = no se comprueba.")
        ] = None,
        limit: Limit = 50,
        offset: Offset = 0,
    ) -> dict[str, Any]:
        """Flecha máxima de cada barra (mm) medida desde la cuerda que une sus extremos deformados, por caso.

        Devuelve {results_status, cases, total, count, offset, has_more, next_offset?, items:[{bar,
        length_m, by_case:[{case, uz_mm, uy_mm, x_uz, l_over_uz, passes?}]}], worst?}. uz es la
        flecha en el eje local z (vertical en vigas), uy la lateral; x_uz la posición relativa
        (0-1) del máximo; l_over_uz = L/|uz|. Es la flecha elástica inmediata con la rigidez del
        modelo: para la diferida (ACI 24.2.4) multiplica la parte permanente por ξ/(1+50ρ'). Usa
        una combinación de servicio (SLS) o los casos por separado. Para limitarla a vigas, structural_type='BEAM'.
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            res, status = _results(s)
            case_list = _cases(s, cases)
            col = s.structure.Bars.GetMany(bar_selection(s, selection, section, storey, structural_type))
            total = col.Count
            indexes = window(total, offset, limit)
            _check_budget(len(indexes), len(case_list) + 2, "barras")
            nodes, deflections = s.structure.Nodes, res.Bars.Deflections
            items, worst = [], None
            for i in indexes:
                bar = om.IRobotBar(col.Get(i))
                a, b = (om.IRobotNode(nodes.Get(n)) for n in (bar.StartNode, bar.EndNode))
                length = ((a.X - b.X) ** 2 + (a.Y - b.Y) ** 2 + (a.Z - b.Z) ** 2) ** 0.5
                by_case = []
                for c in case_list:
                    value = deflections.MaxValue(bar.Number, c["number"])
                    uz = abs(value.UZ)
                    row: dict[str, Any] = {
                        "case": c["number"],
                        "uz_mm": num(uz * 1e3, 3),
                        "uy_mm": num(abs(value.UY) * 1e3, 3),
                        "x_uz": num(value.PosUZ, 3),
                        "l_over_uz": num(length / uz, 0) if uz > 1e-9 else None,
                    }
                    if limit_ratio:
                        row["passes"] = uz <= length / limit_ratio
                    by_case.append(row)
                    if uz > 0 and (worst is None or length / uz < worst["l_over_uz"]):
                        worst = {"bar": bar.Number, "case": c["number"], "uz_mm": row["uz_mm"], "l_over_uz": row["l_over_uz"]}
                items.append({"bar": bar.Number, "length_m": num(length, 3), "by_case": by_case})
            out = {"results_status": status, "cases": case_list, **page(total, offset, limit, items)}
            if worst:
                out["worst"] = worst  # de esta página
            return out

        return await on_robot(work)

    @mcp.tool(name="robot_get_panel_forces", title="Esfuerzos en paneles (máximos)", annotations=READ_ONLY)
    async def robot_get_panel_forces(
        cases: Cases,
        selection: Annotated[str | None, Field(description="Paneles: '100', '1to20'. Omitir = todos.")] = None,
        components: Annotated[
            list[Literal["MXX", "MYY", "MXY", "NXX", "NYY", "NXY", "QXX", "QYY"]] | None,
            Field(description="Esfuerzos a evaluar. Omitir = los ocho."),
        ] = None,
    ) -> dict[str, Any]:
        """Máximo y mínimo de los esfuerzos de placa en paneles con elementos finitos (muros y losas).

        Devuelve {results_status, cases, panels_count, extremes:[{component, unit, max:{value,
        node, element, case}, min:{…}}]}. node es el nudo de la malla (robot_list_nodes da sus
        coordenadas); Robot no dice el panel, así que para saberlo consulta los paneles de uno en uno. Unidades: MXX, MYY, MXY en kN·m/m; NXX, NYY, NXY, QXX, QYY
        en kN/m. Ejes locales de cada panel (x según su definición en Robot). Los paneles de solo
        reparto de cargas (robot_list_panels: calc_model.finite_elements=false) no tienen malla:
        sus valores salen null.
        """
        chosen = list(dict.fromkeys(components or ["MXX", "MYY", "MXY", "NXX", "NYY", "NXY", "QXX", "QYY"]))

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            res, status = _results(s)
            case_list = _cases(s, cases)
            ot = om.IRobotObjectType
            panels = s.select(ot.I_OT_PANEL, selection)
            if panels.Count == 0:
                raise ToolError("No hay paneles en la selección; consulta robot_list_panels.")
            case_sel = s.structure.Selections.Create(ot.I_OT_CASE)
            for c in case_list:
                case_sel.AddOne(c["number"])
            params = om.IRobotFeExtremeParams(s.app.CmpntFactory.Create(om.IRobotComponentType.I_CT_FE_EXTREME_PARAMS))
            params.PanelSel = str(panels.ToText())
            params.CaseSel = str(case_sel.ToText())

            def describe(value: Any) -> dict[str, Any] | None:
                if not value.IsAvailable:
                    return None
                return {"value": num(value.Value / 1e3, 3), "node": value.Node, "element": value.Element, "case": value.Case}

            extremes = []
            for component in chosen:
                params.ResultId = getattr(om.IRobotFeResultType, f"I_FRT_DETAILED_{component}")
                extremes.append(
                    {
                        "component": component,
                        "unit": "kN·m/m" if component[0] == "M" else "kN/m",
                        "max": describe(res.FiniteElems.MaxValue(params)),
                        "min": describe(res.FiniteElems.MinValue(params)),
                    }
                )
            return {"results_status": status, "cases": case_list, "panels_count": panels.Count, "extremes": extremes}

        return await on_robot(work)

    @mcp.tool(name="robot_get_result_extremes", title="Máximos y mínimos (envolvente)", annotations=READ_ONLY)
    async def robot_get_result_extremes(
        quantity: Annotated[
            Literal["bar_forces", "reactions", "node_displacements"],
            Field(description="Qué resultado envolver: esfuerzos en barras, reacciones o desplazamientos de nudos."),
        ],
        cases: Cases,
        selection: Selection = None,
        section: SectionFilter = None,
        storey: StoreyFilter = None,
        structural_type: StructuralTypeFilter = None,
        components: Annotated[
            list[Literal["FX", "FY", "FZ", "MX", "MY", "MZ", "UX", "UY", "UZ", "RX", "RY", "RZ"]] | None,
            Field(description="Componentes a evaluar. Omitir = las seis de la magnitud (F/M o U/R)."),
        ] = None,
    ) -> dict[str, Any]:
        """Máximo y mínimo de cada componente sobre un conjunto de elementos y casos, con su ubicación.

        Es la forma rápida de obtener envolventes (Robot busca internamente). Devuelve
        {results_status, quantity, cases, elements_count, extremes:[{component, unit,
        max:{value, bar|node, case, x?}, min:{value, bar|node, case, x?}}]}.
        Unidades: fuerzas kN, momentos kN·m, desplazamientos mm, giros rad. x = posición
        relativa en la barra (se evalúan 11 estaciones). En bar_forces cada extremo trae además
        structural_type, section, storey, start_node y end_node de la barra, para no tener que
        consultarla aparte. section, storey y structural_type solo aplican a bar_forces (para
        «la columna más cargada» usa structural_type='COLUMN'). En node_displacements se
        incluyen los nudos de la malla de paneles.
        Mismas convenciones de signo que robot_get_bar_forces; en casos espectrales el
        signo no es físico.
        """
        valid = DISPLACEMENTS if quantity == "node_displacements" else FORCES
        chosen = list(dict.fromkeys(components)) if components else list(valid)
        wrong = [c for c in chosen if c not in valid]
        if wrong:
            raise ToolError(f"Componentes no válidas para {quantity}: {wrong}. Admitidas: {list(valid)}.")
        if quantity != "bar_forces" and (section or storey or structural_type):
            raise ToolError("section, storey y structural_type solo se pueden usar con quantity='bar_forces'.")

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            res, status = _results(s)
            case_list = _cases(s, cases)
            ot = om.IRobotObjectType
            if quantity == "bar_forces":
                bars = bar_selection(s, selection, section, storey, structural_type)
                obj_type, elements, prefix, key = ot.I_OT_BAR, bars, "FORCE_BAR", "bar"
            elif quantity == "reactions":
                obj_type, elements, prefix, key = ot.I_OT_NODE, _supported_nodes(s, selection), "REACTION", "node"
            else:
                obj_type, elements, prefix, key = ot.I_OT_NODE, s.select(ot.I_OT_NODE, selection), "DISPLACEMENT_NODE", "node"
            if elements.Count == 0:
                raise ToolError("La selección no contiene ningún elemento; revisa selection, section y storey.")
            case_sel = s.structure.Selections.Create(ot.I_OT_CASE)
            for c in case_list:
                case_sel.AddOne(c["number"])
            params = om.IRobotExtremeParams(s.app.CmpntFactory.Create(om.IRobotComponentType.I_CT_EXTREME_PARAMS))
            params.Selection.Set(obj_type, elements)
            params.Selection.Set(ot.I_OT_CASE, case_sel)
            params.BarDivision = 11

            storey_bars: dict[str, Any] = {}  # piso -> selección de sus barras, solo si hace falta

            def bar_context(number: int) -> dict[str, Any]:
                """Datos de la barra del extremo, para que no haga falta otra consulta."""
                bar = om.IRobotBar(s.structure.Bars.Get(number))
                section_type = om.IRobotLabelType.I_LT_BAR_SECTION
                context: dict[str, Any] = {
                    "structural_type": enum_name(bar.StructuralType),
                    "start_node": bar.StartNode,
                    "end_node": bar.EndNode,
                }
                if bar.HasLabel(section_type):
                    context["section"] = str(bar.GetLabelName(section_type))
                if not storey_bars:
                    for name in s.storey_names():
                        storey_bars[name] = s.structure.Selections.CreateByStorey(ot.I_OT_BAR, name)
                context["storey"] = next((name for name, group in storey_bars.items() if group.Contains(number)), None)
                return context

            def describe(value: Any, scale: float, digits: int) -> dict[str, Any] | None:
                if not value.IsAvailable:
                    return None
                out: dict[str, Any] = {"value": num(value.Value * scale, digits), "case": value.Case}
                if key == "bar":
                    out["bar"] = value.Bar
                    out["x"] = num(value.Position, 3)
                    out.update(bar_context(value.Bar))
                else:
                    out["node"] = value.Node
                return out

            extremes = []
            for component in chosen:
                if component[0] == "U":
                    unit, scale, digits = "mm", 1e3, 4
                elif component[0] == "R":
                    unit, scale, digits = "rad", 1.0, 7
                else:
                    unit, scale, digits = ("kN" if component[0] == "F" else "kN·m"), 1e-3, 3
                params.ValueType = getattr(om.IRobotExtremeValueType, f"I_EVT_{prefix}_{component}")
                extremes.append(
                    {
                        "component": component,
                        "unit": unit,
                        "max": describe(res.Extremes.MaxValue(params), scale, digits),
                        "min": describe(res.Extremes.MinValue(params), scale, digits),
                    }
                )
            return {
                "results_status": status,
                "quantity": quantity,
                "cases": case_list,
                "elements_count": elements.Count,
                "extremes": extremes,
            }

        return await on_robot(work)

    @mcp.tool(name="robot_get_modal_results", title="Modos de vibración y masas participantes", annotations=READ_ONLY)
    async def robot_get_modal_results(
        case: Annotated[
            int | None, Field(ge=1, description="Número del caso modal. Omitir = primer caso de análisis modal.")
        ] = None,
        max_modes: Annotated[int, Field(ge=1, le=200, description="Máximo de modos a listar.")] = 30,
    ) -> dict[str, Any]:
        """Periodos, frecuencias y masa participante por modo (% de la masa total), con acumulados.

        Devuelve {results_status, case, name, modes_count, total_mass_t:{x, y, z},
        cumulative_mass_pct:{ux, uy, uz}, modes_to_90_pct:{ux, uy}, modes:[{mode, period_s,
        frequency_hz, mass_ux_pct, mass_uy_pct, mass_uz_pct, cum_ux_pct, cum_uy_pct, cum_uz_pct}]}.
        cumulative_mass_pct es el acumulado con TODOS los modos calculados (no solo los listados);
        modes_to_90_pct es el primer modo en que el acumulado alcanza 90 % en X y en Y, o null
        si no se alcanza (la NEC-SE-DS exige ≥ 90 % en cada dirección horizontal).
        """

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            res, status = _results(s)
            server = s.structure.Cases
            if case is None:
                number = _modal_case(s)
                if number is None:
                    raise ToolError("El modelo no tiene ningún caso de análisis modal.")
            else:
                number = case
                if not server.Exist(number):
                    raise ToolError(f"No existe el caso {number}. Consulta robot_list_load_cases.")
            robot_case = server.Get(number)
            if enum_name(robot_case.Type) != "SIMPLE":
                raise ToolError(f"El caso {number} es una combinación; indica el caso modal.")
            count = om.IRobotSimpleCase(robot_case).ModesCount
            if count <= 0:
                raise ToolError(f"El caso {number} ('{robot_case.Name}') no tiene modos calculados.")
            eigen, mass = res.Advanced.Eigenvalues, res.Advanced.MassSum
            modes = []
            reach_90: dict[str, int | None] = {"ux": None, "uy": None}
            for mode in range(1, count + 1):
                relative = mass.Relative(number, mode)
                for axis, value in (("ux", relative.UX), ("uy", relative.UY)):
                    if reach_90[axis] is None and value >= 90.0:
                        reach_90[axis] = mode
                if mode > max_modes:
                    continue
                value, current = eigen.Value(number, mode), mass.Current(number, mode)
                modes.append(
                    {
                        "mode": mode,
                        "period_s": num(value.Period, 4),
                        "frequency_hz": num(value.Frequence, 4),
                        "mass_ux_pct": num(current.UX, 3),
                        "mass_uy_pct": num(current.UY, 3),
                        "mass_uz_pct": num(current.UZ, 3),
                        "cum_ux_pct": num(relative.UX, 3),
                        "cum_uy_pct": num(relative.UY, 3),
                        "cum_uz_pct": num(relative.UZ, 3),
                    }
                )
            last, total = mass.Relative(number, count), mass.Total(number, count)
            return {
                "results_status": status,
                "case": number,
                "name": str(robot_case.Name),
                "modes_count": count,
                "total_mass_t": {"x": num(total.UX / 1e3, 2), "y": num(total.UY / 1e3, 2), "z": num(total.UZ / 1e3, 2)},
                "cumulative_mass_pct": {"ux": num(last.UX, 3), "uy": num(last.UY, 3), "uz": num(last.UZ, 3)},
                "modes_to_90_pct": reach_90,
                "modes": modes,
            }

        return await on_robot(work)

    @mcp.tool(name="robot_get_storey_results", title="Resultados por piso: derivas y cortantes", annotations=READ_ONLY)
    async def robot_get_storey_results(cases: Cases) -> dict[str, Any]:
        """Tabla de pisos de Robot: desplazamientos, derivas, cortantes, masa y centros, por caso.

        Devuelve {results_status, cases, storeys:[{name, bottom_level_m, height_m, mass_t,
        center_of_mass:{x, y, z}, center_of_rigidity:{x, y, z}, by_case:[{case, max_ux_mm,
        max_uy_mm, node_max_ux, node_max_uy, drift_x_mm, drift_y_mm, drift_ratio_x, drift_ratio_y,
        shear_fx_kn, shear_fy_kn, axial_fz_kn}]}]} de abajo hacia arriba.
        drift_* es el 'dr' que Robot calcula para ese caso, SIN amplificar; drift_ratio = dr / altura
        del piso. Para verificar una norma hay que aplicar sus factores (NEC-SE-DS: ΔM = 0.75·R·ΔE,
        límite 0.02 en hormigón) y, si el espectro se cargó elástico y la reducción está en las
        combinaciones, ese mismo factor. shear_* es el cortante acumulado del piso (en el primero,
        el cortante basal). mass_t y los centros se leen del caso modal si existe (si no, del
        primer caso pedido; en una combinación Robot da masa 0). Requiere pisos definidos.
        """

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, cases)
            manager = s.structure.Storeys
            if manager.Count == 0:
                raise ToolError("El modelo no tiene pisos definidos; créalos en Robot para obtener derivas por piso.")
            _check_budget(manager.Count, len(case_list) * 3, "pisos")
            # Sin masas dinámicas, la masa de piso depende del caso (0 en combinaciones): mejor la del modal
            mass_case = _modal_case(s) or case_list[0]["number"]
            storeys = []
            for index in range(1, manager.Count + 1):
                storey = manager.Get(index)
                height = float(storey.Height)
                general = res.Storeys.Values(index, mass_case)
                by_case = []
                for c in case_list:
                    moved = res.Storeys.Displacements(index, c["number"])
                    forces = res.Storeys.ReducedForces(index, c["number"])
                    by_case.append(
                        {
                            "case": c["number"],
                            "max_ux_mm": num(moved.MaxUX * 1e3, 3),
                            "max_uy_mm": num(moved.MaxUY * 1e3, 3),
                            "node_max_ux": moved.NodeMaxUX,
                            "node_max_uy": moved.NodeMaxUY,
                            "drift_x_mm": num(moved.DrUX * 1e3, 3),
                            "drift_y_mm": num(moved.DrUY * 1e3, 3),
                            "drift_ratio_x": num(moved.DrUX / height, 6) if height > 0 else None,
                            "drift_ratio_y": num(moved.DrUY / height, 6) if height > 0 else None,
                            "shear_fx_kn": num(forces.FX / 1e3, 2),
                            "shear_fy_kn": num(forces.FY / 1e3, 2),
                            "axial_fz_kn": num(forces.FZ / 1e3, 2),
                        }
                    )
                storeys.append(
                    {
                        "name": str(storey.Name),
                        "bottom_level_m": num(storey.Level, 4),
                        "height_m": num(height, 4),
                        "mass_t": num(general.Mass / 1e3, 2),
                        "center_of_mass": {"x": num(general.G.X, 3), "y": num(general.G.Y, 3), "z": num(general.G.Z, 3)},
                        "center_of_rigidity": {
                            "x": num(general.R.X, 3),
                            "y": num(general.R.Y, 3),
                            "z": num(general.R.Z, 3),
                        },
                        "by_case": by_case,
                    }
                )
            storeys.sort(key=lambda item: item["bottom_level_m"])
            return {"results_status": status, "cases": case_list, "storeys": storeys}

        return await on_robot(work)
