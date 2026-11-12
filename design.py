"""Cuantías de acero requeridas en vigas y columnas rectangulares de hormigón según ACI 318-19. Solo lectura.

Robot da los esfuerzos de las combinaciones; aquí se dimensiona a mano, como en una hoja de cálculo:
  Vigas     flexión simple (bloque de Whitney, φ = 0.9), As mín §9.6.1.2, ρ máx 0.025 §18.6.3.1
  Columnas  flexocompresión por compatibilidad de deformaciones, φ según §21.2.2, biaxial por el
            contorno lineal Mx/φMnx + My/φMny ≤ 1 (conservador), ρ entre 1 % y 6 % (§18.7.4.1)
  Cortante  Vc = (0.17·√f'c + Nu/(6·Ag))·b·d (§22.5.5.1), φ = 0.75, Vs ≤ 0.66·√f'c·b·d
Unidades internas: N, mm, MPa. No hace diseño por capacidad (cortante por momentos probables,
columna fuerte-viga débil) ni esbeltez: son pasos del ingeniero sobre estos resultados.
"""

from __future__ import annotations

import math
from typing import Annotated, Any

from mcp.server.mcpserver import MCPServer
from pydantic import Field

from results import _check_budget, _cases, _results
from robot_com import Session, bar_selection, enum_name, num, on_robot, page, section_item, window
from schema import READ_ONLY, Cases, Limit, Offset, SectionFilter, Selection, StoreyFilter, StructuralTypeFilter

ES = 200_000.0  # MPa
EPS_CU = 0.003
RHO_MIN_COLUMN, RHO_MAX_COLUMN = 0.01, 0.06
RHO_MAX_BEAM = 0.025


def beta1(fc: float) -> float:
    return max(0.65, min(0.85, 0.85 - 0.05 * (fc - 28) / 7))


def beam_steel(mu: float, b: float, d: float, fc: float, fy: float) -> float | None:
    """As (mm²) para el momento último mu (N·mm) en una sección b × d; None si el hormigón no alcanza."""
    if mu <= 0:
        return 0.0
    rn = mu / (0.9 * b * d * d)
    root = 1 - 2 * rn / (0.85 * fc)
    if root < 0:
        return None
    return 0.85 * fc / fy * (1 - math.sqrt(root)) * b * d


def shear_steel(vu: float, nu: float, b: float, h: float, d: float, fc: float, fyt: float) -> dict[str, Any]:
    """Av/s (mm²/mm) requerido y mínimo para vu (N), con axial nu (N, compresión +)."""
    ag = b * h
    vc = max(0.0, (0.17 * math.sqrt(fc) + min(nu / (6 * ag), 0.05 * fc)) * b * d)
    vs = max(0.0, vu / 0.75 - vc)
    minimum = max(0.062 * math.sqrt(fc), 0.35) * b / fyt if vu > 0.5 * 0.75 * vc else 0.0
    return {"vc": vc, "av_s": vs / (fyt * d), "av_s_min": minimum, "ok": vs <= 0.66 * math.sqrt(fc) * b * d}


def column_curve(width: float, depth: float, cover: float, rho: float, per_face: int, fc: float, fy: float) -> list[tuple[float, float]]:
    """Diagrama de interacción uniaxial [(φMn, φPn)] (N·mm, N) de tracción pura a compresión máxima.

    Barras repartidas en el perímetro: per_face barras en cada cara (4·(per_face−1) en total).
    """
    total_bars = 4 * (per_face - 1)
    ast = rho * width * depth
    bar = ast / total_bars
    layers = [(cover + k * (depth - 2 * cover) / (per_face - 1), bar * (per_face if k in (0, per_face - 1) else 2)) for k in range(per_face)]
    b1, eps_y = beta1(fc), fy / ES
    p0 = 0.85 * fc * (width * depth - ast) + fy * ast
    cap = 0.65 * 0.80 * p0  # §22.4.2.1, columnas con estribos
    dt = depth - cover
    points = [(0.0, -0.9 * fy * ast)]
    for k in range(1, 121):
        c = depth * (k / 40) ** 1.5  # más puntos cerca de la flexión pura
        a = min(b1 * c, depth)
        pn = 0.85 * fc * a * width
        mn = pn * (depth / 2 - a / 2)
        for y, area in layers:
            fs = max(-fy, min(fy, ES * EPS_CU * (c - y) / c))
            force = area * (fs - (0.85 * fc if y < a else 0.0))
            pn += force
            mn += force * (depth / 2 - y)
        eps_t = EPS_CU * (dt - c) / c
        phi = 0.65 + 0.25 * min(1.0, max(0.0, (eps_t - eps_y) / 0.003))
        points.append((max(0.0, phi * mn), min(phi * pn, cap)))
    points.append((0.0, cap))
    return points


def moment_capacity(curve: list[tuple[float, float]], pu: float) -> float | None:
    """φMn (N·mm) del diagrama para la carga axial pu (N); None si pu está fuera del diagrama."""
    best = None
    for (m1, p1), (m2, p2) in zip(curve, curve[1:]):
        if min(p1, p2) <= pu <= max(p1, p2):
            m = m1 if p1 == p2 else m1 + (m2 - m1) * (pu - p1) / (p2 - p1)
            best = m if best is None else max(best, m)
    return best


def column_utilization(demands: list[tuple[float, float, float]], b: float, h: float, cover: float, rho: float, per_face: int, fc: float, fy: float) -> tuple[float, int]:
    """Máxima utilización Muy/φMny + Muz/φMnz de las demandas (Pu, Muy, Muz) y el índice de la que manda.

    Muy flexiona alrededor del eje local y (canto h); Muz alrededor de z (canto b).
    """
    about_y = column_curve(b, h, cover, rho, per_face, fc, fy)
    about_z = column_curve(h, b, cover, rho, per_face, fc, fy)
    worst, index = 0.0, 0
    for i, (pu, muy, muz) in enumerate(demands):
        cap_y, cap_z = moment_capacity(about_y, pu), moment_capacity(about_z, pu)
        if cap_y is None or cap_z is None:
            return math.inf, i
        use = (abs(muy) / cap_y if muy else 0.0) + (abs(muz) / cap_z if muz else 0.0) if cap_y > 0 and cap_z > 0 else math.inf
        if use > worst:
            worst, index = use, i
    return worst, index


def required_rho(demands: list[tuple[float, float, float]], b: float, h: float, cover: float, per_face: int, fc: float, fy: float) -> float | None:
    """Menor ρ entre 1 % y 6 % que resiste todas las demandas (precisión 0.01 %); None si ni el 6 % alcanza."""
    if column_utilization(demands, b, h, cover, RHO_MAX_COLUMN, per_face, fc, fy)[0] > 1:
        return None
    if column_utilization(demands, b, h, cover, RHO_MIN_COLUMN, per_face, fc, fy)[0] <= 1:
        return RHO_MIN_COLUMN
    low, high = RHO_MIN_COLUMN, RHO_MAX_COLUMN
    while high - low > 1e-4:
        mid = (low + high) / 2
        low, high = (low, mid) if column_utilization(demands, b, h, cover, mid, per_face, fc, fy)[0] <= 1 else (mid, high)
    return high


def register(mcp: MCPServer) -> None:
    @mcp.tool(name="robot_check_rc_members", title="Cuantías de vigas y columnas (ACI 318-19)", annotations=READ_ONLY)
    async def robot_check_rc_members(
        cases: Cases,
        fc_mpa: Annotated[float, Field(ge=17, le=80, description="Resistencia del hormigón f'c en MPa (dato del proyecto).")],
        fy_mpa: Annotated[float, Field(ge=240, le=550, description="Fluencia del acero longitudinal fy en MPa (dato del proyecto).")],
        fyt_mpa: Annotated[float | None, Field(ge=240, le=550, description="Fluencia de los estribos. Omitir = fy.")] = None,
        cover_cm: Annotated[float, Field(gt=2, le=15, description="Distancia de la cara al centro de las barras longitudinales (cm).")] = 6.0,
        bars_per_face: Annotated[int, Field(ge=2, le=8, description="Barras por cara en columnas (3 = 8 barras en total).")] = 3,
        selection: Selection = None,
        section: SectionFilter = None,
        storey: StoreyFilter = None,
        structural_type: StructuralTypeFilter = None,
        limit: Limit = 20,
        offset: Offset = 0,
    ) -> dict[str, Any]:
        """Acero longitudinal y transversal requerido en vigas y columnas rectangulares de hormigón (ACI 318-19).

        Pasa las combinaciones últimas (ULS), p. ej. las de robot_create_nec_combinations; un caso
        espectral suelto no sirve (sus esfuerzos no tienen signo). Devuelve {results_status, cases,
        materials, summary:{checked, failing, skipped}, total, count, offset, has_more, next_offset?,
        items:[…]}. Viga: {bar, kind:'BEAM', section, b_cm, h_cm, d_cm, flexure:{mu_neg_knm,
        mu_pos_knm, as_top_cm2, as_bottom_cm2, as_min_cm2, rho_top, rho_bottom}, shear:{vu_kn,
        phi_vc_kn, av_s_cm2_m, av_s_min_cm2_m}, ok, problems}. Columna: {bar, kind:'COLUMN', section,
        b_cm, h_cm, governs:{case, x, pu_kn, muy_knm, muz_knm}, rho_required, as_required_cm2,
        shear_y, shear_z, ok, problems}. As_top/bottom: la mayor entre la requerida y la mínima.
        av_s en cm²/m (área de todas las ramas por metro). Momentos en los nudos (no a la cara del
        apoyo: conservador). Secciones no rectangulares o no de hormigón (p. ej. dobles T
        prefabricadas) se omiten en 'skipped'. Una barra sin tipo estructural se clasifica por su
        geometría (kind_from_geometry: vertical = columna). No incluye diseño por capacidad ni esbeltez.
        """
        fyt = fyt_mpa or fy_mpa
        cover = cover_cm * 10

        def work(s: Session) -> dict[str, Any]:
            om = s.om
            res, status = _results(s)
            case_list = _cases(s, cases, strict=True)
            col = s.structure.Bars.GetMany(bar_selection(s, selection, section, storey, structural_type))
            total = col.Count
            indexes = window(total, offset, limit)
            _check_budget(len(indexes), len(case_list) * 5, "barras")
            forces = res.Bars.Forces
            lt = om.IRobotLabelType.I_LT_BAR_SECTION
            sections: dict[str, dict[str, Any]] = {}
            items, skipped, failing = [], [], 0
            for i in indexes:
                bar = om.IRobotBar(col.Get(i))
                kind = enum_name(bar.StructuralType)
                guessed = kind not in ("BEAM", "COLUMN")
                if guessed:  # barra sin tipo estructural (p. ej. creada por la API): vertical = columna
                    a, z = (om.IRobotNode(s.structure.Nodes.Get(n)) for n in (bar.StartNode, bar.EndNode))
                    kind = "COLUMN" if abs(a.Z - z.Z) > math.hypot(a.X - z.X, a.Y - z.Y) else "BEAM"
                name = str(bar.GetLabelName(lt)) if bar.HasLabel(lt) else None
                if name and name not in sections:
                    sections[name] = section_item(s, s.structure.Labels.Get(lt, name))
                info = sections.get(name or "", {})
                if info.get("shape") not in ("CONCR_BEAM_RECT", "CONCR_COL_R"):
                    skipped.append({"bar": bar.Number, "kind": kind, "section": name, "shape": info.get("shape")})
                    continue
                b, h = info["b_cm"] * 10, info["h_cm"] * 10
                d = h - cover
                stations = (0.0, 0.25, 0.5, 0.75, 1.0) if kind == "BEAM" else (0.0, 1.0)
                rows = []  # (caso, x, N, Vy, Vz, My, Mz) en N y N·mm
                for c in case_list:
                    for x in stations:
                        f = forces.Value(bar.Number, c["number"], x)
                        rows.append((c["number"], x, f.FX, f.FY, f.FZ, f.MY * 1e3, f.MZ * 1e3))
                item: dict[str, Any] = {"bar": bar.Number, "kind": kind, "section": name, "b_cm": info["b_cm"], "h_cm": info["h_cm"]}
                if guessed:
                    item["kind_from_geometry"] = True
                problems: list[str] = []
                if kind == "BEAM":
                    mu_pos = max(max(r[5] for r in rows), 0.0)
                    mu_neg = max(-min(r[5] for r in rows), 0.0)
                    as_min = max(0.25 * math.sqrt(fc_mpa), 1.4) / fy_mpa * b * d
                    flexure: dict[str, Any] = {"mu_neg_knm": num(mu_neg / 1e6, 2), "mu_pos_knm": num(mu_pos / 1e6, 2), "as_min_cm2": num(as_min / 100, 2)}
                    for face, mu in (("top", mu_neg), ("bottom", mu_pos)):
                        steel = beam_steel(mu, b, d, fc_mpa, fy_mpa)
                        if steel is None:
                            problems.append(f"flexión {'negativa' if face == 'top' else 'positiva'}: la sección no alcanza, aumenta el canto")
                            flexure[f"as_{face}_cm2"] = flexure[f"rho_{face}"] = None
                            continue
                        steel = max(steel, as_min)
                        rho = steel / (b * d)
                        flexure[f"as_{face}_cm2"], flexure[f"rho_{face}"] = num(steel / 100, 2), num(rho, 4)
                        if rho > RHO_MAX_BEAM:
                            problems.append(f"ρ {face} = {rho:.3%} > 2.5 % (§18.6.3.1)")
                    if max(abs(r[2]) for r in rows) > 0.1 * fc_mpa * b * h:
                        problems.append("axial > 0.1·f'c·Ag: dimensiónala como columna (§18.6.1)")
                    worst = max(rows, key=lambda r: abs(r[4]))
                    shear = shear_steel(abs(worst[4]), worst[2], b, h, d, fc_mpa, fyt)
                    item.update(d_cm=num(d / 10, 2), flexure=flexure)
                    item["shear"] = {
                        "vu_kn": num(abs(worst[4]) / 1e3, 2),
                        "case": worst[0],
                        "phi_vc_kn": num(0.75 * shear["vc"] / 1e3, 2),
                        "av_s_cm2_m": num(shear["av_s"] * 10, 3),
                        "av_s_min_cm2_m": num(shear["av_s_min"] * 10, 3),
                    }
                    if not shear["ok"]:
                        problems.append("cortante: Vs > 0.66·√f'c·b·d, aumenta la sección")
                else:
                    demands = [(r[2], r[5], r[6]) for r in rows]
                    rho = required_rho(demands, b, h, cover, bars_per_face, fc_mpa, fy_mpa)
                    use, index = column_utilization(demands, b, h, cover, rho or RHO_MAX_COLUMN, bars_per_face, fc_mpa, fy_mpa)
                    governs = rows[index]
                    item["governs"] = {"case": governs[0], "x": governs[1], "pu_kn": num(governs[2] / 1e3, 2), "muy_knm": num(governs[5] / 1e6, 2), "muz_knm": num(governs[6] / 1e6, 2)}
                    item["rho_required"] = num(rho, 4) if rho else None
                    item["as_required_cm2"] = num(rho * b * h / 100, 2) if rho else None
                    if rho is None:
                        problems.append(f"ni con ρ = 6 % resiste (utilización {use:.2f}): aumenta la sección")
                    for axis, column, depth, width in (("shear_z", 4, h, b), ("shear_y", 3, b, h)):
                        worst = max(rows, key=lambda r: abs(r[column]))
                        shear = shear_steel(abs(worst[column]), worst[2], width, depth, depth - cover, fc_mpa, fyt)
                        item[axis] = {"vu_kn": num(abs(worst[column]) / 1e3, 2), "case": worst[0], "av_s_cm2_m": num(shear["av_s"] * 10, 3), "av_s_min_cm2_m": num(shear["av_s_min"] * 10, 3)}
                        if not shear["ok"]:
                            problems.append(f"cortante ({axis[-1]}): Vs > 0.66·√f'c·b·d, aumenta la sección")
                item["ok"] = not problems
                item["problems"] = problems
                failing += bool(problems)
                items.append(item)
            out = page(total, offset, limit, items)
            summary = {"checked": len(items), "failing": failing, "skipped": skipped}
            materials = {"fc_mpa": fc_mpa, "fy_mpa": fy_mpa, "fyt_mpa": fyt, "cover_cm": cover_cm, "bars_per_face": bars_per_face}
            return {"results_status": status, "cases": case_list, "materials": materials, "summary": summary, **out}

        return await on_robot(work)
