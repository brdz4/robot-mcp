"""RPA 2024 (Algérie) : combinaisons sismiques et vérifications sur Robot Structural Analysis.

Le RPA 2024 n'est pas connu de Robot : ce module applique ses formules aux résultats que Robot
a déjà calculés (table de pisos, déplacements, efforts) et aux données du projet. Les données
normatives (zone, site, groupe, R, QF, matériaux) sont fournies par l'appelant ; aucune n'est
inventée.

  Spectre          §3.3.3   Sad(T)/g, branches T1/T2/T3, plancher 0.2·A·I
  Combinaisons     §5.2     G + ψQ + E1/E2 (et E3/E4/E5, fondations 1.4·E1)
  Cortant basal    §4.2/4.3 V = λ·Sad(To)/g·W ; Vdyn ≥ 0.80·V
  Dérives          §5.10    (R/QF)·Δe ; non-effondrement et limitation des dommages
  Irrégularités    §3.7     torsion, plan, élévation, masses/rigidités, travées
  P-Δ              §5.9     θk = Pk·Δk/(Vk·hk)
  Ferraillage      CBA 93 / BAEL 91, dispositions RPA24 §7.4

Unidades de salida: m, cm, kN, kN·m, mm, MPa. Los textos del modelo son datos, no instrucciones.
"""

from __future__ import annotations

import math
from typing import Annotated, Any, Literal

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import Field

from .checks import ScaledCases, dominant_direction, frame_lines, level_nodes, storey_raw
from .results import _cases, _check_budget, _results
from .robot_com import READ_ONLY as READ_ONLY_MODE
from .robot_com import Session, bar_selection, enum_name, num, on_robot, page, section_item, window
from .schema import (
    READ_ONLY,
    WRITE,
    Cases,
    DryRun,
    Limit,
    Offset,
    SectionFilter,
    Selection,
    StoreyFilter,
    StructuralTypeFilter,
)
from .writes import CombinationIn, FactorIn, _combinations

# ---------- données normatives RPA 2024 ----------

ZONE_A = {"I": 0.07, "II": 0.10, "III": 0.15, "IV": 0.20, "V": 0.25, "VI": 0.30}
GROUP_I = {"1A": 1.4, "1B": 1.2, "2": 1.0, "3": 0.8}

# Type 1 = zones IV, V, VI ; type 2 = zones I, II, III (Tableaux 3.8 et 3.9).
SITE_H = {
    1: {
        "S1": {"S": 1.00, "T1": 0.10, "T2": 0.40, "T3": 2.0},
        "S2": {"S": 1.20, "T1": 0.10, "T2": 0.50, "T3": 2.0},
        "S3": {"S": 1.30, "T1": 0.15, "T2": 0.60, "T3": 2.0},
        "S4": {"S": 1.35, "T1": 0.15, "T2": 0.70, "T3": 2.0},
    },
    2: {
        "S1": {"S": 1.00, "T1": 0.05, "T2": 0.25, "T3": 1.2},
        "S2": {"S": 1.30, "T1": 0.05, "T2": 0.30, "T3": 1.2},
        "S3": {"S": 1.55, "T1": 0.10, "T2": 0.40, "T3": 1.2},
        "S4": {"S": 1.80, "T1": 0.10, "T2": 0.50, "T3": 1.2},
    },
}

# Spectre vertical §3.3.3 (Av/A, T1, T2 par site, T3, alpha).
SITE_V = {
    1: {"AvA": 0.90, "T1": 0.05, "T2": {"S1": 0.20, "S2": 0.30, "S3": 0.40, "S4": 0.50}, "T3": 1.0, "alpha": 0.6},
    2: {"AvA": 0.55, "T1": 0.05, "T2": {"S1": 0.15, "S2": 0.20, "S3": 0.25, "S4": 0.30}, "T3": 1.0, "alpha": 0.8},
}

# Tableau 4.3 : CT pour T = CT·hN^(3/4).
CT = {
    "RC_FRAMES": 0.075,   # ossatures spatiales en BA sans remplissage
    "STEEL_FRAMES": 0.085,  # portiques en acier sans remplissage
    "INFILLED": 0.050,    # portiques BA ou acier avec remplissage
    "OTHER": 0.050,       # autres types
}

# Tableau 4.2 : coefficient d'accompagnement ψ de la charge d'exploitation.
PSI = {
    "HABITATION": 0.30,   # habitations, bureaux
    "ERP": 0.40,          # établissements recevant du public
    "ENTREPOT": 0.50,     # entrepôts, hangars
    "ARCHIVES": 1.00,     # archives, bibliothèques, réservoirs
    "AUTRES": 0.60,       # autres locaux
}

# Tableau 5.2 : déplacements inter-étages limites de non-effondrement (× hk).
DRIFT_LIMITS = {"STEEL": 0.0200, "RC": 0.0150, "PAF": 0.0100, "TIMBER": 0.0150, "MASONRY": 0.0100}

# §5.10.2 : limitation des dommages (× hk), avec ψA = 0.5.
DAMAGE_LIMITS = {"FRAGILE": 0.0050, "DUCTILE": 0.0075}

# §7.4.2 : pourcentage minimal d'armatures longitudinales des poteaux.
RHO_MIN = {"I": 0.008, "II": 0.008, "III": 0.009, "IV": 0.010, "V": 0.010, "VI": 0.010}

# Tableau 3.19 : pondérations du facteur de qualité QF.
QF_WEIGHTS = {
    "a": {"plan": 0.05, "elevation": 0.20, "storeys": 0.20, "bays": 0.10},
    "b": {"plan": 0.05, "elevation": 0.20, "redundancy": 0.05},
    "c": {},
}

# CBA 93 / BAEL 91.
ES_MPA = 200_000.0
EPS_CU = 0.0035
GAMMA_B = 1.5
GAMMA_S = 1.15
RHO_MAX_COLUMN = 0.04

Zone = Literal["I", "II", "III", "IV", "V", "VI"]
Site = Literal["S1", "S2", "S3", "S4"]
Group = Literal["1A", "1B", "2", "3"]
System = Literal["RC_FRAMES", "STEEL_FRAMES", "INFILLED", "OTHER"]

ZoneParam = Annotated[Zone, Field(description="Zone de sismicité du RPA 2024 (Tableau 3.3).")]
SiteParam = Annotated[Site, Field(description="Classe de site S1 à S4 (Tableaux 3.8 et 3.9).")]
GroupParam = Annotated[Group, Field(description="Groupe d'importance 1A, 1B, 2 ou 3 (Tableau 3.4).")]

# ---------- spectre ----------


def _spectrum_type(zone: str) -> int:
    """Type de spectre : 1 pour les zones IV-VI, 2 pour les zones I-III."""
    return 1 if zone in ("IV", "V", "VI") else 2


def _site_h(zone: str, site: str) -> dict[str, float]:
    return SITE_H[_spectrum_type(zone)][site]


def _elastic_h(t: float, a: float, i: float, s: float, t1: float, t2: float, t3: float, eta: float) -> float:
    """Spectre élastique horizontal Sae/g (§3.3.2)."""
    b = a * i * s
    e = 2.5 * eta
    if t <= 0:
        return b
    if t < t1:
        return b * (1 + (t / t1) * (e - 1))
    if t < t2:
        return b * e
    if t < t3:
        return b * e * (t2 / t)
    return b * e * (t2 * t3 / (t * t))


def _sad_h(t: float, a: float, i: float, s: float, t1: float, t2: float, t3: float, r: float, qf: float, floor: bool = True) -> float:
    """Spectre de calcul horizontal Sad/g (§3.3.3, Eq. 3.15).

    La branche initiale part de (2/3)·B et monte vers k·B = 2.5·(QF/R)·B.
    """
    b = a * i * s
    k = 2.5 * qf / r
    if t <= 0:
        value = (2.0 / 3.0) * b
    elif t < t1:
        value = b * ((2.0 / 3.0) + (t / t1) * (k - 2.0 / 3.0))
    elif t < t2:
        value = b * k
    elif t < t3:
        value = b * k * (t2 / t)
    else:
        value = b * k * (t2 * t3 / (t * t))
    return max(value, 0.2 * a * i) if floor else value


def _elastic_v(t: float, av: float, i: float, t1: float, t2: float, t3: float, eta: float, alpha: float) -> float:
    """Spectre élastique vertical Sae/g (§3.3.2)."""
    b = av * i
    e = 2.5 * eta
    if t <= 0:
        return b
    if t < t1:
        return b * (1 + (t / t1) * (e - 1))
    if t < t2:
        return b * e
    if t < t3:
        return b * e * (t2 / t) ** alpha
    return b * e * ((t2 * t3) / (t * t)) ** alpha


def _sad_v(t: float, av: float, i: float, t1: float, t2: float, t3: float, alpha: float, floor: bool = True) -> float:
    """Spectre de calcul vertical Sad/g (§3.3.3, Eq. 3.16) : S=1, R=1.5, QF=1."""
    b = av * i
    k = 2.5 / 1.5
    if t <= 0:  # discontinuité de la formule originale à T=0
        value = b
    elif t < t1:
        value = b * ((2.0 / 3.0) + (t / t1) * (k - 2.0 / 3.0))
    elif t < t2:
        value = b * k
    elif t < t3:
        value = b * k * (t2 / t) ** alpha
    else:
        value = b * k * ((t2 * t3) / (t * t)) ** alpha
    return max(value, 0.2 * av * i) if floor else value


def _vertical_params(zone: str, site: str) -> dict[str, float]:
    table = SITE_V[_spectrum_type(zone)]
    return {
        "Av": table["AvA"] * ZONE_A[zone],
        "AvA": table["AvA"],
        "T1": table["T1"],
        "T2": table["T2"][site],
        "T3": table["T3"],
        "alpha": table["alpha"],
    }

# ---------- géométrie et critères ----------


def _plan_bbox(s: Session) -> tuple[dict[str, float] | None, int]:
    """Enveloppe en plan de tous les nœuds (m). None si le modèle est trop grand pour une lecture fiable."""
    om = s.om
    col = s.structure.Nodes.GetMany(s.select(om.IRobotObjectType.I_OT_NODE))
    if col.Count == 0 or col.Count > 5000:
        return None, col.Count
    xs: list[float] = []
    ys: list[float] = []
    for index in range(1, col.Count + 1):
        node = om.IRobotNode(col.Get(index))
        xs.append(float(node.X))
        ys.append(float(node.Y))
    return {
        "x_min_m": num(min(xs), 3),
        "x_max_m": num(max(xs), 3),
        "y_min_m": num(min(ys), 3),
        "y_max_m": num(max(ys), 3),
        "length_x_m": num(max(xs) - min(xs), 3),
        "width_y_m": num(max(ys) - min(ys), 3),
    }, col.Count


def _criterion(
    cid: str,
    label: str,
    value: Any,
    limit: Any,
    ok: bool | None,
    weight: float,
    note: str | None = None,
) -> dict[str, Any]:
    return {
        "id": cid,
        "label": label,
        "value": value,
        "limit": limit,
        "status": "UNKNOWN" if ok is None else ("OK" if ok else "FAIL"),
        "qf_weight": weight,
        "note": note,
    }


def _group_status(criteria: list[dict[str, Any]], ids: set[str]) -> str:
    selected = [item for item in criteria if item["id"] in ids]
    if any(item["status"] == "FAIL" for item in selected):
        return "FAIL"
    if any(item["status"] == "UNKNOWN" for item in selected):
        return "UNKNOWN"
    return "OK"


def _worse(bucket: dict[str, Any], value: float, **extra: Any) -> None:
    if value > float(bucket.get("value", 0.0)):
        bucket.clear()
        bucket.update({"value": value, **extra})


def _summary(bucket: dict[str, Any], limit: float, label: str) -> dict[str, Any]:
    value = float(bucket.get("value", 0.0))
    rest = {key: val for key, val in bucket.items() if key != "value"}
    return {
        label: num(value, 5),
        **rest,
        "limit": num(limit, 5),
        "utilization": num(value / limit, 3) if limit else None,
        "passes": value <= limit,
    }

# ---------- CBA 93 / BAEL 91 ----------


def _bael_materials(fc28: float, fe: float) -> tuple[float, float, float]:
    fcd = 0.85 * fc28 / GAMMA_B
    fyd = fe / GAMMA_S
    ft28 = 0.6 + 0.06 * fc28
    return fcd, fyd, ft28


def _mu_lim(fe: float) -> float:
    """μlim du pivot A/B pour l'acier considéré (0.392 pour FeE400, 0.372 pour FeE500)."""
    eps_su = fe / (GAMMA_S * ES_MPA)
    alpha_lim = EPS_CU / (EPS_CU + eps_su)
    return 0.8 * alpha_lim * (1 - 0.4 * alpha_lim)


def _beam_steel(mu_nmm: float, b: float, d: float, fcd: float, fyd: float, mu_lim: float) -> tuple[float | None, float, float | None]:
    """As (mm²) en flexion simple BAEL ; None si μ > μlim (section insuffisante)."""
    if mu_nmm <= 0:
        return 0.0, 0.0, 0.0
    mu = mu_nmm / (b * d * d * fcd)
    if mu > mu_lim:
        return None, mu, None
    alpha = 1.25 * (1 - math.sqrt(max(0.0, 1 - 2 * mu)))
    z = d * (1 - 0.4 * alpha)
    return mu_nmm / (z * fyd), mu, alpha


def _bael_shear(vu_n: float, b: float, d: float, fc28: float, fe: float, ft28: float) -> dict[str, float | bool]:
    """Cisaillement BAEL 91 A.5.1.2 : τu, τu lim, At/st requis et minimal (mm²/mm)."""
    tau_u = vu_n / (b * d)
    tau_lim = min(0.2 * fc28 / GAMMA_B, 5.0)
    fyd = fe / GAMMA_S
    av_s_req = max(0.0, (tau_u - 0.3 * ft28) / (0.9 * fyd))
    av_s_min = 0.4 * b / fe
    return {
        "tau_u_mpa": tau_u,
        "tau_lim_mpa": tau_lim,
        "av_s_mm2_mm": max(av_s_req, av_s_min),
        "av_s_min_mm2_mm": av_s_min,
        "ok": tau_u <= tau_lim,
    }


def _column_curve_bael(
    width: float,
    depth: float,
    cover: float,
    rho: float,
    per_face: int,
    fcd: float,
    fyd: float,
) -> list[tuple[float, float]]:
    """Diagramme d'interaction (Mu, Nu) par compatibilité de déformations (bloc 0.8·c, εcu=0.0035)."""
    total_bars = 4 * (per_face - 1)
    if total_bars < 4:
        raise ToolError("bars_per_face debe ser al menos 2 (4 barras en total).")
    ast = rho * width * depth
    bar = ast / total_bars
    layers = []
    for k in range(per_face):
        y = cover + k * (depth - 2 * cover) / (per_face - 1)
        area = bar * (per_face if k in (0, per_face - 1) else 2)
        layers.append((y, area))
    cap = 0.85 * fcd * width * depth + fyd * ast
    points = [(0.0, -fyd * ast)]
    for k in range(1, 121):
        c = depth * (k / 40) ** 1.5
        a = min(0.8 * c, depth)
        pn = fcd * a * width
        mn = pn * (depth / 2 - a / 2)
        for y, area in layers:
            fs = max(-fyd, min(fyd, ES_MPA * EPS_CU * (c - y) / c))
            force = area * fs
            pn += force
            mn += force * (depth / 2 - y)
        points.append((max(0.0, mn), min(pn, cap)))
    points.append((0.0, cap))
    return points


def _moment_capacity(curve: list[tuple[float, float]], pu: float) -> float | None:
    best = None
    for (m1, p1), (m2, p2) in zip(curve, curve[1:]):
        if min(p1, p2) <= pu <= max(p1, p2):
            moment = m1 if p1 == p2 else m1 + (m2 - m1) * (pu - p1) / (p2 - p1)
            best = moment if best is None else max(best, moment)
    return best


def _column_utilization(
    demands: list[tuple[float, float, float]],
    b: float,
    h: float,
    cover: float,
    rho: float,
    per_face: int,
    fcd: float,
    fyd: float,
) -> tuple[float, int]:
    """Utilisation maximale Muy/Mny + Muz/Mnz (contour linéaire conservateur) et indice gouvernant."""
    about_y = _column_curve_bael(b, h, cover, rho, per_face, fcd, fyd)
    about_z = _column_curve_bael(h, b, cover, rho, per_face, fcd, fyd)
    worst, index = 0.0, 0
    for i, (pu, muy, muz) in enumerate(demands):
        cap_y, cap_z = _moment_capacity(about_y, pu), _moment_capacity(about_z, pu)
        if cap_y is None or cap_z is None:
            return math.inf, i
        use = (abs(muy) / cap_y if muy else 0.0) + (abs(muz) / cap_z if muz else 0.0) if cap_y > 0 and cap_z > 0 else math.inf
        if use > worst:
            worst, index = use, i
    return worst, index


def _required_rho(
    demands: list[tuple[float, float, float]],
    b: float,
    h: float,
    cover: float,
    per_face: int,
    fcd: float,
    fyd: float,
    rho_min: float,
    rho_max: float,
) -> float | None:
    """Plus petite quantité ρ entre ρmin et ρmax qui résiste à toutes les demandes."""
    if _column_utilization(demands, b, h, cover, rho_max, per_face, fcd, fyd)[0] > 1:
        return None
    if _column_utilization(demands, b, h, cover, rho_min, per_face, fcd, fyd)[0] <= 1:
        return rho_min
    low, high = rho_min, rho_max
    while high - low > 1e-4:
        mid = (low + high) / 2
        if _column_utilization(demands, b, h, cover, mid, per_face, fcd, fyd)[0] <= 1:
            high = mid
        else:
            low = mid
    return high


# ---------- outils ----------


def register(mcp: MCPServer) -> None:  # noqa: C901 - un registre par outil
    @mcp.tool(name="robot_rpa24_spectrum", title="Spectre RPA 2024 (calcul pur)", annotations=READ_ONLY)
    async def robot_rpa24_spectrum(
        zone: ZoneParam,
        site: SiteParam,
        group: GroupParam,
        r: Annotated[float, Field(gt=0, le=10, description="Coefficient de comportement R (Tableau 3.18).")],
        qf: Annotated[float, Field(ge=1, le=2, description="Facteur de qualité QF (Tableau 3.19).")] = 1.0,
        damping: Annotated[float, Field(ge=0, le=30, description="Amortissement ξ en % (5 % par défaut).")] = 5.0,
        periods: Annotated[
            list[float] | None,
            Field(description="Périodes T (s) à évaluer, entre 0 et 4 s. Omitir = points caractéristiques T1, T2, T3, 1, 2, 3, 4."),
        ] = None,
        direction: Annotated[Literal["H", "V"], Field(description="H = spectre horizontal ; V = composante verticale.")] = "H",
        floor: Annotated[bool, Field(description="Appliquer le plancher 0.2·A·I (0.2·Av·I en vertical).")] = True,
    ) -> dict[str, Any]:
        """Spectre élastique et spectre de calcul du RPA 2024, sans connexion à Robot.

        Renvoie {standard, direction, spectrum_type, parameters, points:[{t_s, elastic_sa_g,
        design_sa_g, elastic_sa_m_s2, design_sa_m_s2}], notes}. Le spectre horizontal suit
        l'Eq. (3.15) : B = A·I·S, k = 2.5·QF/R, première branche à (2/3)·B, plancher 0.2·A·I.
        Le spectre vertical suit l'Eq. (3.16) : Av = 0.90·A (type 1) ou 0.55·A (type 2),
        S = 1, R = 1.5, QF = 1. Les périodes sont limitées à 4 s. C'est un calcul de contrôle :
        ne remplace pas le spectre chargé dans Robot, dont les valeurs doivent être cohérentes.
        """
        a = ZONE_A[zone]
        i = GROUP_I[group]
        eta = math.sqrt(7.0 / (2.0 + damping))
        if periods is None:
            if direction == "H":
                table = _site_h(zone, site)
                periods = [0.0, table["T1"], table["T2"], table["T3"], 1.0, 2.0, 3.0, 4.0]
            else:
                table_v = _vertical_params(zone, site)
                periods = [0.0, table_v["T1"], table_v["T2"], table_v["T3"], 2.0, 3.0, 4.0]
        values = sorted({float(t) for t in periods})
        if not values or any(t < 0 or t > 4 for t in values):
            raise ToolError("Las períodos deben estar entre 0 y 4 s.")
        if direction == "H":
            table = _site_h(zone, site)
            b = a * i * table["S"]
            parameters = {
                "zone": zone,
                "site": site,
                "group": group,
                "spectrum_type": _spectrum_type(zone),
                "A": a,
                "I": i,
                "S": table["S"],
                "T1_s": table["T1"],
                "T2_s": table["T2"],
                "T3_s": table["T3"],
                "eta": num(eta, 4),
                "B": num(b, 4),
                "k": num(2.5 * qf / r, 4),
                "R": r,
                "QF": qf,
                "floor_sa_g": num(0.2 * a * i, 4),
            }
            points = [
                {
                    "t_s": num(t, 3),
                    "elastic_sa_g": num(_elastic_h(t, a, i, table["S"], table["T1"], table["T2"], table["T3"], eta), 5),
                    "design_sa_g": num(_sad_h(t, a, i, table["S"], table["T1"], table["T2"], table["T3"], r, qf, floor), 5),
                }
                for t in values
            ]
        else:
            vertical = _vertical_params(zone, site)
            parameters = {
                "zone": zone,
                "site": site,
                "group": group,
                "spectrum_type": _spectrum_type(zone),
                "A": a,
                "I": i,
                "Av": num(vertical["Av"], 4),
                "AvA": vertical["AvA"],
                "T1_s": vertical["T1"],
                "T2_s": vertical["T2"],
                "T3_s": vertical["T3"],
                "alpha": vertical["alpha"],
                "eta": num(eta, 4),
                "R": 1.5,
                "QF": 1.0,
                "floor_sa_g": num(0.2 * vertical["Av"] * i, 4),
            }
            points = [
                {
                    "t_s": num(t, 3),
                    "elastic_sa_g": num(
                        _elastic_v(t, vertical["Av"], i, vertical["T1"], vertical["T2"], vertical["T3"], eta, vertical["alpha"]), 5
                    ),
                    "design_sa_g": num(
                        _sad_v(t, vertical["Av"], i, vertical["T1"], vertical["T2"], vertical["T3"], vertical["alpha"], floor), 5
                    ),
                }
                for t in values
            ]
        for point in points:
            point["elastic_sa_m_s2"] = num(point["elastic_sa_g"] * 9.81, 4)
            point["design_sa_m_s2"] = num(point["design_sa_g"] * 9.81, 4)
        return {
            "standard": "RPA 2024",
            "direction": direction,
            "parameters": parameters,
            "points": points,
            "notes": [
                "Le spectre de calcul ne doit jamais être inférieur à 0.2·A·I (0.2·Av·I en vertical).",
                "Le spectre est défini jusqu'à T = 4 s.",
                "η = √(7/(2+ξ)) ne s'applique qu'au spectre élastique ; le spectre de calcul utilise k = 2.5·QF/R.",
            ],
        }

    @mcp.tool(name="robot_check_rpa24_base_shear", title="Cortant basal RPA 2024 (§4.2/4.3)", annotations=READ_ONLY)
    async def robot_check_rpa24_base_shear(
        seismic_cases: ScaledCases,
        gravity_case: Annotated[
            int,
            Field(ge=1, description="Caso o combinación con G + ψQ SIN mayorar (peso sísmico), p. ej. 'G+0.3Q'."),
        ],
        zone: ZoneParam,
        site: SiteParam,
        group: GroupParam,
        r: Annotated[float, Field(gt=0, le=10, description="Coefficient de comportement R (Tableau 3.18).")],
        qf: Annotated[float, Field(ge=1, le=2, description="Facteur de qualité QF (Tableau 3.19).")] = 1.0,
        system: Annotated[System, Field(description="Système de contreventement pour CT (Tableau 4.3).")] = "RC_FRAMES",
        period_s: Annotated[
            float | None,
            Field(gt=0, le=10, description="Période fondamentale calculée (s). Omitir = formule empirique To = CT·hN^(3/4)."),
        ] = None,
        weight_kn: Annotated[
            float | None,
            Field(gt=0, description="Poids sismique W (kN). Omitir = suma de masas de gravity_case × g."),
        ] = None,
        height_m: Annotated[
            float | None,
            Field(gt=0, description="Hauteur hN desde la base hasta la cubierta (m). Omitir = geometría de los pisos."),
        ] = None,
        direction: Annotated[Literal["AUTO", "X", "Y"], Field(description="Dirección de cálculo. AUTO = la del mayor cortante en el piso inferior.")] = "AUTO",
    ) -> dict[str, Any]:
        """Cortant basal estático del RPA 2024 y contraste con el cortante dinámico de Robot.

        Calcula W (peso sísmico = Σ(G + ψQ)), To = CT·hN^(3/4), λ = 0.85 si To ≤ 2·T2 y hay más
        de 2 niveles, y V = λ·Sad(To)/g·W. Compara cada caso sísmico con 0.80·V (§4.3.5) usando
        el cortante del primer piso de la tabla de pisos. Devuelve {results_status, formula,
        parameters:{W_kn, hN_m, To_empirical_s, To_used_s, lambda, Sa_g, V_static_kn,
        minimum_dynamic_kn}, cases:[{case, scale, direction, V_dynamic_kn, ratio, passes,
        required_scale}], summary:{min_ratio, governing_case, direction, passes, required_scale}}.
        Si el caso espectral está cargado elástico, 'scale' debe llevarlo al nivel de diseño;
        si ya está reducido, scale = 1. gravity_case debe ser G + ψQ, no una combinación mayorada.
        """
        numbers = [item.case for item in seismic_cases]
        if len(set(numbers)) != len(numbers):
            raise ToolError("Hay casos sísmicos repetidos en la petición.")
        scales = {item.case: item.scale for item in seismic_cases}

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, " ".join(str(number) for number in numbers), strict=True)
            rows = storey_raw(s, res, numbers, mass_case=gravity_case)
            weights = [row["mass"] * 9.81 / 1e3 for row in rows]
            w = float(weight_kn) if weight_kn is not None else sum(weights)
            if w <= 0:
                raise ToolError("El peso sísmico es 0: revisa gravity_case (debe incluir G + ψQ) o pasa weight_kn.")
            height = float(height_m) if height_m is not None else rows[-1]["top"] - rows[0]["level"]
            if height <= 0:
                raise ToolError("La altura del edificio es 0: define pisos en Robot o pasa height_m.")
            table = _site_h(zone, site)
            a, i = ZONE_A[zone], GROUP_I[group]
            t_emp = CT[system] * height**0.75
            t_used = t_emp if period_s is None else min(float(period_s), 1.3 * t_emp)
            lam = 0.85 if (t_used <= 2 * table["T2"] and len(rows) > 2) else 1.0
            sa = _sad_h(t_used, a, i, table["S"], table["T1"], table["T2"], table["T3"], r, qf)
            v_static = lam * sa * w
            cases = []
            worst: dict[str, Any] | None = None
            for number in numbers:
                axis = direction.lower() if direction != "AUTO" else dominant_direction(rows, number)
                base = rows[0]["by_case"][number]
                v_dyn = base[f"v_{axis}"] * scales[number] / 1e3
                ratio = v_dyn / v_static if v_static > 0 else 0.0
                passes = ratio >= 0.80
                required = 0.80 * v_static / v_dyn if (not passes and v_dyn > 0) else 1.0
                entry = {
                    "case": number,
                    "scale": scales[number],
                    "direction": axis.upper(),
                    "V_dynamic_kn": num(v_dyn, 2),
                    "ratio": num(ratio, 4),
                    "passes": passes,
                    "required_scale": num(required, 3),
                }
                cases.append(entry)
                if worst is None or ratio < worst["ratio"]:
                    worst = entry
            assert worst is not None
            summary = {
                "min_ratio": worst["ratio"],
                "governing_case": worst["case"],
                "direction": worst["direction"],
                "passes": all(item["passes"] for item in cases),
                "required_scale": num(max(item["required_scale"] for item in cases), 3),
            }
            out: dict[str, Any] = {
                "results_status": status,
                "standard": "RPA 2024",
                "formula": "V = λ·Sad(To)/g·W ; el cortante dinámico no debe ser inferior a 0.80·V (§4.3.5)",
                "parameters": {
                    "W_kn": num(w, 2),
                    "hN_m": num(height, 3),
                    "To_empirical_s": num(t_emp, 4),
                    "To_used_s": num(t_used, 4),
                    "period_capped": period_s is not None and float(period_s) > 1.3 * t_emp,
                    "lambda": lam,
                    "Sa_g": num(sa, 5),
                    "V_static_kn": num(v_static, 2),
                    "minimum_dynamic_kn": num(0.80 * v_static, 2),
                },
                "cases": cases,
                "summary": summary,
            }
            if not summary["passes"]:
                out["next_step"] = (
                    f"Aumenta todas las respuestas sísmicas por {summary['required_scale']:.3f} "
                    "(o carga el espectro con ese factor) y recalcula; luego repite la verificación."
                )
            return out

        return await on_robot(work)

    @mcp.tool(name="robot_check_rpa24_drifts", title="Dérives inter-étages RPA 2024 (§5.10)", annotations=READ_ONLY)
    async def robot_check_rpa24_drifts(
        seismic_cases: ScaledCases,
        r: Annotated[float, Field(gt=0, le=10, description="Coefficient de comportement R (Tableau 3.18).")],
        qf: Annotated[float, Field(ge=1, le=2, description="Facteur de qualité QF (Tableau 3.19).")] = 1.0,
        structure_type: Annotated[
            Literal["STEEL", "RC", "PAF", "TIMBER", "MASONRY"],
            Field(description="Type de structure pour la limite de non-effondrement (Tableau 5.2)."),
        ] = "RC",
        nonstructural: Annotated[
            Literal["FRAGILE", "DUCTILE"],
            Field(description="Éléments non structuraux pour la limitation des dommages (§5.10.2)."),
        ] = "FRAGILE",
        damage: Annotated[bool, Field(description="Vérifier aussi la limitation des dommages (δ = 0.5·(R/QF)·Δe).")] = True,
        limit_override: Annotated[
            float | None,
            Field(gt=0, le=0.05, description="Limite de non-effondrement personnalisée (× hk). Omitir = Tableau 5.2."),
        ] = None,
    ) -> dict[str, Any]:
        """Dérives inter-étages du RPA 2024 : non-effondrement et limitation des dommages.

        La dérive élastique de Robot (dr/h) est multipliée par 'scale' du cas, puis par R/QF
        (§5.10.1). Pour la limitation des dommages, on applique en plus ψA = 0.5, soit
        0.5·(R/QF)·Δe (§5.10.2). Limites : acier 0.0200·hk, béton armé 0.0150·hk, PAF 0.0100·hk,
        bois 0.0150·hk, maçonnerie chaînée 0.0100·hk ; éléments fragiles 0.0050·hk, ductiles
        0.0075·hk. Vérifie aussi chaque ligne de portique parallèle à la direction dominante,
        car sans diafragma rigide c'est elle qui gouverne. Devuelve {results_status, formula,
        parameters, cases, storeys:[…], summary:{noncollapse, damage}}. Usa los casos sísmicos
        (no las combinaciones con gravedad). R, QF y scale son datos del proyecto.
        """
        numbers = [item.case for item in seismic_cases]
        if len(set(numbers)) != len(numbers):
            raise ToolError("Hay casos sísmicos repetidos en la petición.")
        scales = {item.case: item.scale for item in seismic_cases}

        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, " ".join(str(number) for number in numbers), strict=True)
            analysis = {item["number"]: item["analysis"] for item in case_list}
            rows = storey_raw(s, res, numbers)
            by_level = level_nodes(s, rows)
            lines: dict[int, tuple[str, list[list[tuple[float, float]] | None]]] = {}
            reads = 0
            for number in numbers:
                axis = dominant_direction(rows, number)
                level_lines, reads = frame_lines(res, by_level, number, axis, reads)
                lines[number] = (axis, level_lines)
            factor = r / qf
            damage_factor = 0.5 * factor
            limit = float(limit_override) if limit_override is not None else DRIFT_LIMITS[structure_type]
            damage_limit = DAMAGE_LIMITS[nonstructural]
            worst_nc: dict[str, Any] = {"value": 0.0}
            worst_dmg: dict[str, Any] = {"value": 0.0}
            storeys = []
            for index, row in enumerate(rows):
                by_case = []
                for number in numbers:
                    data = row["by_case"][number]
                    elastic = {axis: data[f"dr_{axis}"] / row["height"] * scales[number] for axis in ("x", "y")}
                    design = {axis: factor * value for axis, value in elastic.items()}
                    damaged = {axis: damage_factor * value for axis, value in elastic.items()}
                    gov_axis = max(design, key=design.get)
                    entry: dict[str, Any] = {
                        "case": number,
                        "scale": scales[number],
                        "elastic_x": num(elastic["x"], 6),
                        "elastic_y": num(elastic["y"], 6),
                        "design_x": num(design["x"], 5),
                        "design_y": num(design["y"], 5),
                        "noncollapse_limit": num(limit, 5),
                        "noncollapse_passes": design[gov_axis] <= limit,
                        "governing": num(design[gov_axis], 5),
                        "direction": gov_axis.upper(),
                    }
                    if damage:
                        dmg_axis = max(damaged, key=damaged.get)
                        entry.update(
                            {
                                "damage_x": num(damaged["x"], 5),
                                "damage_y": num(damaged["y"], 5),
                                "damage_limit": num(damage_limit, 5),
                                "damage_passes": damaged[dmg_axis] <= damage_limit,
                                "damage_governing": num(damaged[dmg_axis], 5),
                                "damage_direction": dmg_axis.upper(),
                            }
                        )
                    _worse(
                        worst_nc,
                        design[gov_axis],
                        storey=row["name"],
                        case=number,
                        direction=gov_axis.upper(),
                        source="storey_table",
                    )
                    if damage:
                        _worse(
                            worst_dmg,
                            damaged[dmg_axis],
                            storey=row["name"],
                            case=number,
                            direction=dmg_axis.upper(),
                            source="storey_table",
                        )
                    line_axis, level_lines = lines[number]
                    below, here = level_lines[index], level_lines[index + 1]
                    lower = dict(below or [])
                    drifts = [(coord, abs(value - lower[coord])) for coord, value in (here or []) if coord in lower]
                    if drifts:
                        coordinate, drift = max(drifts, key=lambda item: item[1])
                        line_design = factor * drift / row["height"] * scales[number]
                        line_damage = damage_factor * drift / row["height"] * scales[number]
                        entry["frame_line"] = {
                            "direction": line_axis.upper(),
                            "coordinate_m": num(coordinate, 2),
                            "drift_mm": num(drift * 1e3, 3),
                            "design": num(line_design, 5),
                            "noncollapse_passes": line_design <= limit,
                            "approximate": analysis[number] != "STATIC_LINEAR",
                        }
                        if damage:
                            entry["frame_line"]["damage"] = num(line_damage, 5)
                            entry["frame_line"]["damage_passes"] = line_damage <= damage_limit
                        _worse(
                            worst_nc,
                            line_design,
                            storey=row["name"],
                            case=number,
                            direction=line_axis.upper(),
                            source="frame_line",
                            coordinate_m=num(coordinate, 2),
                        )
                        if damage:
                            _worse(
                                worst_dmg,
                                line_damage,
                                storey=row["name"],
                                case=number,
                                direction=line_axis.upper(),
                                source="frame_line",
                                coordinate_m=num(coordinate, 2),
                            )
                    by_case.append(entry)
                storeys.append({"storey": row["name"], "height_m": num(row["height"], 3), "by_case": by_case})
            return {
                "results_status": status,
                "standard": "RPA 2024",
                "formula": "Δd = (R/QF)·Δe (§5.10.1) ; dommages : δ = 0.5·(R/QF)·Δe (§5.10.2)",
                "parameters": {
                    "r": r,
                    "qf": qf,
                    "amplification": num(factor, 4),
                    "damage_amplification": num(damage_factor, 4),
                    "structure_type": structure_type,
                    "noncollapse_limit": num(limit, 5),
                    "nonstructural": nonstructural,
                    "damage_limit": num(damage_limit, 5),
                },
                "cases": case_list,
                "storeys": storeys,
                "summary": {
                    "noncollapse": _summary(worst_nc, limit, "max_design"),
                    "damage": _summary(worst_dmg, damage_limit, "max_damage") if damage else None,
                },
            }

        return await on_robot(work)

    @mcp.tool(name="robot_check_rpa24_irregularities", title="Irrégularités RPA 2024 (§3.7/3.8)", annotations=READ_ONLY)
    async def robot_check_rpa24_irregularities(
        zone: ZoneParam = "III",
        qf_category: Annotated[
            Literal["a", "b", "c"],
            Field(description="Catégorie du Tableau 3.19 : (a) ossatures/portiques, (b) voiles/noyaux, (c) QF=1."),
        ] = "a",
        e0x_m: Annotated[float | None, Field(ge=0, description="Excentricité structurale e0x (m), centre de masse − centre de rigidité.")] = None,
        e0y_m: Annotated[float | None, Field(ge=0, description="Excentricité structurale e0y (m).")] = None,
        rx_m: Annotated[float | None, Field(gt=0, description="Rayon de torsion rx (m).")] = None,
        ry_m: Annotated[float | None, Field(gt=0, description="Rayon de torsion ry (m).")] = None,
        length_x_m: Annotated[float | None, Field(gt=0, description="Dimension en plan Lx (m). Omitir = enveloppe des nœuds.")] = None,
        width_y_m: Annotated[float | None, Field(gt=0, description="Dimension en plan Ly (m). Omitir = enveloppe des nœuds.")] = None,
        recess_ratio: Annotated[float | None, Field(ge=0, le=1, description="Partie rentrante maximale / dimension totale (0-1).")] = None,
        opening_ratio: Annotated[float | None, Field(ge=0, le=1, description="Ouverture de plancher maximale / surface du plancher (0-1).")] = None,
        mass_variation_ratio: Annotated[float | None, Field(ge=0, le=1, description="Variation relative de masse entre étages (0-1).")] = None,
        stiffness_variation_ratio: Annotated[float | None, Field(ge=0, le=1, description="Variation relative de rigidité entre étages (0-1).")] = None,
        bays_x: Annotated[int | None, Field(ge=1, le=50, description="Nombre de travées dans la direction X.")] = None,
        bays_y: Annotated[int | None, Field(ge=1, le=50, description="Nombre de travées dans la direction Y.")] = None,
        mass_case: Annotated[
            int | None,
            Field(ge=1, description="Caso/combinación de masa (G+ψQ) para estimar la variación de masa entre pisos. Omitir = no leer."),
        ] = None,
    ) -> dict[str, Any]:
        """Contrôle des critères de régularité du RPA 2024 (§3.7) et calcul indicatif de QF (§3.8).

        Evalúa: torsion e0x ≤ 0.2·rx y e0y ≤ 0.2·ry, rapport L/B ≤ 4, parties rentrantes ≤ 25 %,
        ouvertures de plancher ≤ 15 %, variation de masse ou de rigidité entre étages ≤ 25 %,
        minimum 2 niveaux et 2 travées dans chaque direction. La géométrie en plan peut venir de
        l'enveloppe des nœuds si length_x_m/width_y_m sont omis. La variation de masse peut être
        calculée depuis les masses des pisos si mass_case est fourni. Devuelve {standard, zone,
        qf_category, model, criteria:[{id, label, value, limit, status, qf_weight, note}],
        groups, qf_suggested_min, qf_suggested_max}. Les critères UNKNOWN ne sont pas comptés :
        fournis les données manquantes pour conclure. QF n'est pas calculé à la place de
        l'ingénieur : qf_suggested_min/max encadrent la valeur selon les critères satisfaits.
        """
        def work(s: Session) -> dict[str, Any]:
            storeys = int(s.structure.Storeys.Count)
            bbox, node_count = _plan_bbox(s)
            if mass_variation_ratio is None and mass_case is not None:
                res, _ = _results(s)
                rows = storey_raw(s, res, [mass_case], mass_case=mass_case)
                masses = [row["mass"] for row in rows]
                ratios = [
                    abs(masses[i] - masses[i - 1]) / masses[i - 1]
                    for i in range(1, len(masses))
                    if masses[i - 1] > 0
                ]
                if ratios:
                    mass_variation_ratio = max(ratios)
            length_x = float(length_x_m) if length_x_m is not None else (bbox["length_x_m"] if bbox else None)
            width_y = float(width_y_m) if width_y_m is not None else (bbox["width_y_m"] if bbox else None)
            weights = QF_WEIGHTS[qf_category]
            plan_weight = weights.get("plan", 0.0)
            elevation_weight = weights.get("elevation", 0.0)
            criteria: list[dict[str, Any]] = []

            for axis, e0, radius in (("x", e0x_m, rx_m), ("y", e0y_m, ry_m)):
                if e0 is not None and radius is not None and radius > 0:
                    criteria.append(
                        _criterion(
                            f"torsion_{axis}",
                            f"Excentricité structurale e0{axis} ≤ 0.2·r{axis}",
                            num(abs(e0), 3),
                            num(0.2 * radius, 3),
                            abs(e0) <= 0.2 * radius,
                            plan_weight,
                            "La torsion accidentelle de 5 % (§4.2.7) reste à vérifier séparément.",
                        )
                    )
                else:
                    criteria.append(
                        _criterion(
                            f"torsion_{axis}",
                            f"Excentricité structurale e0{axis} ≤ 0.2·r{axis}",
                            None,
                            None,
                            None,
                            plan_weight,
                            f"Fournir e0{axis}_m et r{axis}_m pour conclure.",
                        )
                    )

            if length_x and width_y and length_x > 0 and width_y > 0:
                aspect = max(length_x / width_y, width_y / length_x)
                criteria.append(
                    _criterion(
                        "plan_aspect",
                        "Rapport en plan L/B ≤ 4",
                        num(aspect, 3),
                        4.0,
                        aspect <= 4.0,
                        plan_weight,
                        "Dimensions estimées par l'enveloppe des nœuds." if length_x_m is None or width_y_m is None else None,
                    )
                )
            else:
                criteria.append(
                    _criterion("plan_aspect", "Rapport en plan L/B ≤ 4", None, 4.0, None, plan_weight, "Fournir length_x_m et width_y_m.")
                )

            for cid, label, value, limit, note in (
                ("recess", "Parties rentrantes ≤ 25 %", recess_ratio, 0.25, None),
                ("opening", "Ouvertures de plancher ≤ 15 %", opening_ratio, 0.15, None),
                ("mass_variation", "Variation de masse entre étages ≤ 25 %", mass_variation_ratio, 0.25, None),
                ("stiffness_variation", "Variation de rigidité entre étages ≤ 25 %", stiffness_variation_ratio, 0.25, None),
            ):
                weight = plan_weight if cid in ("recess", "opening") else elevation_weight
                if value is None:
                    criteria.append(_criterion(cid, label, None, limit, None, weight, "Donnée non fournie."))
                else:
                    criteria.append(_criterion(cid, label, num(float(value), 4), limit, float(value) <= limit, weight, note))

            criteria.append(
                _criterion(
                    "storeys",
                    "Au moins 2 niveaux",
                    storeys,
                    2,
                    storeys >= 2 if storeys > 0 else None,
                    weights.get("storeys", 0.0),
                    "Le modèle n'a pas de pisos définis." if storeys == 0 else None,
                )
            )
            for axis, bays in (("x", bays_x), ("y", bays_y)):
                criteria.append(
                    _criterion(
                        f"bays_{axis}",
                        f"Au moins 2 travées en {axis.upper()}",
                        bays,
                        2,
                        bays >= 2 if bays is not None else None,
                        weights.get("bays", weights.get("redundancy", 0.0)),
                        "Donnée non fournie." if bays is None else None,
                    )
                )

            if qf_category == "a":
                groups = {
                    "plan": _group_status(criteria, {"torsion_x", "torsion_y", "plan_aspect", "recess", "opening"}),
                    "elevation": _group_status(criteria, {"mass_variation", "stiffness_variation"}),
                    "storeys": _group_status(criteria, {"storeys"}),
                    "bays": _group_status(criteria, {"bays_x", "bays_y"}),
                }
            elif qf_category == "b":
                groups = {
                    "plan": _group_status(criteria, {"torsion_x", "torsion_y", "plan_aspect", "recess", "opening"}),
                    "elevation": _group_status(criteria, {"mass_variation", "stiffness_variation"}),
                    "redundancy": _group_status(criteria, {"bays_x", "bays_y"}),
                }
            else:
                groups = {}
            qf_min = 1.0 + sum(weight for group, weight in weights.items() if groups.get(group) == "FAIL")
            qf_max = 1.0 + sum(weight for group, weight in weights.items() if groups.get(group) in ("FAIL", "UNKNOWN"))
            return {
                "standard": "RPA 2024",
                "zone": zone,
                "qf_category": qf_category,
                "model": {"storeys": storeys, "nodes": node_count, "bbox": bbox},
                "criteria": criteria,
                "groups": groups,
                "qf_suggested_min": num(qf_min, 3),
                "qf_suggested_max": num(qf_max, 3),
                "notes": [
                    "Les critères UNKNOWN ne sont ni satisfaits ni pénalisés : qf_suggested_max les compte tous comme non satisfaits.",
                    "La catégorie (c) du Tableau 3.19 impose QF = 1.",
                    "La régularité en élévation inclut aussi les variations de masse et de rigidité ; vérifier les reprises de voiles et les étages souples.",
                ],
            }

        return await on_robot(work)

    @mcp.tool(name="robot_check_rpa24_p_delta", title="Effets P-Δ RPA 2024 (§5.9)", annotations=READ_ONLY)
    async def robot_check_rpa24_p_delta(
        seismic_cases: Cases,
        gravity_case: Annotated[
            int,
            Field(ge=1, description="Caso o combinación con G + ψQ sin mayorar (peso acumulado Pk)."),
        ],
        direction: Annotated[Literal["AUTO", "X", "Y"], Field(description="Direction de calcul. AUTO = direction dominante du cas.")] = "AUTO",
    ) -> dict[str, Any]:
        """Indice de stabilité P-Δ par étage, θk = Pk·Δk/(Vk·hk), selon le RPA 2024 §5.9.

        Pk est le poids cumulé au-dessus du niveau k (G + ψQ), Δk la dérive du niveau k et Vk
        l'effort tranchant d'étage du cas sismique. Comme Δk et Vk proviennent du même cas, θk
        ne dépend pas du facteur d'échelle du séisme. Interprétation : θ < 0.10 négligeable ;
        0.10 ≤ θ < 0.20 amplifier les effets sismiques par 1/(1−θ) ; θ ≥ 0.20 structure
        potentiellement instable, à redimensionner. Devuelve {results_status, formula, cases,
        storeys:[{storey, height_m, P_above_kn, by_case:[{case, direction, delta_mm, V_kn,
        theta, classification, amplification, passes}]}], summary}. gravity_case doit être
        G + ψQ, sans majoration.
        """
        def work(s: Session) -> dict[str, Any]:
            res, status = _results(s)
            case_list = _cases(s, seismic_cases, strict=True)
            numbers = [item["number"] for item in case_list]
            rows = storey_raw(s, res, numbers, mass_case=gravity_case)
            weights = [row["mass"] * 9.81 / 1e3 for row in rows]
            storeys = []
            worst: dict[str, Any] = {"value": -1.0}
            for index, row in enumerate(rows):
                p_above = sum(weights[index:])
                by_case = []
                for number in numbers:
                    axis = direction.lower() if direction != "AUTO" else dominant_direction(rows, number)
                    data = row["by_case"][number]
                    delta = data[f"dr_{axis}"]
                    shear = data[f"v_{axis}"] / 1e3
                    height = row["height"]
                    theta = p_above * delta / (shear * height) if shear > 0 and height > 0 else None
                    if theta is None:
                        classification, amplification, passes = "INDETERMINADO", None, None
                    elif theta < 0.10:
                        classification, amplification, passes = "DESPRECIABLE", 1.0, True
                    elif theta < 0.20:
                        classification, amplification, passes = "AMPLIFICAR", 1.0 / (1.0 - theta), True
                    else:
                        classification, amplification, passes = "INESTABLE", None, False
                    by_case.append(
                        {
                            "case": number,
                            "direction": axis.upper(),
                            "delta_mm": num(delta * 1e3, 3),
                            "V_kn": num(shear, 2),
                            "theta": num(theta, 4) if theta is not None else None,
                            "classification": classification,
                            "amplification": num(amplification, 3) if amplification is not None else None,
                            "passes": passes,
                        }
                    )
                    if theta is not None and theta > worst["value"]:
                        worst.clear()
                        worst.update(
                            {
                                "value": theta,
                                "storey": row["name"],
                                "case": number,
                                "direction": axis.upper(),
                                "classification": classification,
                                "passes": passes,
                            }
                        )
                storeys.append(
                    {
                        "storey": row["name"],
                        "height_m": num(row["height"], 3),
                        "P_above_kn": num(p_above, 2),
                        "by_case": by_case,
                    }
                )
            value = max(worst.get("value", 0.0), 0.0)
            rest = {key: val for key, val in worst.items() if key != "value"}
            summary = {
                "max_theta": num(value, 4),
                **rest,
                "passes": value < 0.20,
                "needs_amplification": 0.10 <= value < 0.20,
                "amplification": num(1.0 / (1.0 - value), 3) if 0.10 <= value < 0.20 else 1.0,
            }
            return {
                "results_status": status,
                "standard": "RPA 2024",
                "formula": "θk = Pk·Δk/(Vk·hk) ; θ < 0.10 négligeable, 0.10-0.20 amplification 1/(1−θ), θ ≥ 0.20 instable",
                "cases": case_list,
                "storeys": storeys,
                "summary": summary,
            }

        return await on_robot(work)

    @mcp.tool(name="robot_check_rpa24_rc_members", title="Ferraillage BAEL 91 / CBA 93 (RPA 2024)", annotations=READ_ONLY)
    async def robot_check_rpa24_rc_members(
        cases: Cases,
        fc28_mpa: Annotated[float, Field(ge=20, le=60, description="Résistance caractéristique fc28 (MPa), dato del proyecto.")],
        fe_mpa: Annotated[float, Field(ge=400, le=600, description="Limite élastique des aciers fe (MPa), p. ex. 400 ou 500.")] = 400.0,
        zone: ZoneParam = "III",
        cover_cm: Annotated[float, Field(gt=2, le=10, description="Distance de la face au centre des armatures longitudinales (cm).")] = 5.0,
        bars_per_face: Annotated[int, Field(ge=2, le=8, description="Barres par face en poteaux (3 = 8 barres au total).")] = 3,
        selection: Selection = None,
        section: SectionFilter = None,
        storey: StoreyFilter = None,
        structural_type: StructuralTypeFilter = None,
        limit: Limit = 20,
        offset: Offset = 0,
    ) -> dict[str, Any]:
        """Armatures longitudinales et transversales BAEL 91 / CBA 93, avec les dispositions RPA 2024.

        Pasa las combinaciones últimas o accidentales (por ejemplo las de
        robot_create_rpa24_combinations); un caso espectral suelto no sirve porque sus esfuerzos
        no tienen signo. Materiales: fcd = 0.85·fc28/1.5, fyd = fe/1.15, ft28 = 0.6 + 0.06·fc28.
        Vigas: flexion simple μ = Mu/(b·d²·fcd), α = 1.25·(1−√(1−2μ)), z = d·(1−0.4α),
        As = Mu/(z·fyd), As min = 0.23·b·d·ft28/fe, ρ ≤ 4 %. Cortante: τu = Vu/(b·d),
        τu lim = min(0.2·fc28/1.5, 5 MPa), At/st ≥ (τu − 0.3·ft28)/(0.9·fyd).
        Poteaux: diagramme d'interaction par compatibilité de déformations (bloc 0.8·c, εcu=0.0035),
        ρmin selon la zone (0.8 % en I-II, 0.9 % en III, 1.0 % en IV-VI), ρmax 4 %. Se señalan
        poteaux courts (λg = lc1/min(b,h) < 5, §7.4.3) y dimensiones mínimas. Devuelve
        {results_status, standard, cases, materials, summary:{checked, failing, skipped}, total,
        count, offset, has_more, next_offset?, items:[…]}. No hace diseño por capacidad,
        confinamiento detallado ni esbeltez: son pasos del ingeniero sobre estos resultados.
        """
        fcd, fyd, ft28 = _bael_materials(fc28_mpa, fe_mpa)
        mu_lim = _mu_lim(fe_mpa)
        rho_min = RHO_MIN[zone]
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
            for index in indexes:
                bar = om.IRobotBar(col.Get(index))
                kind = enum_name(bar.StructuralType)
                guessed = kind not in ("BEAM", "COLUMN")
                start = om.IRobotNode(s.structure.Nodes.Get(bar.StartNode))
                end = om.IRobotNode(s.structure.Nodes.Get(bar.EndNode))
                if guessed:  # barra sin tipo estructural: vertical = columna
                    kind = "COLUMN" if abs(start.Z - end.Z) > math.hypot(start.X - end.X, start.Y - end.Y) else "BEAM"
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
                for case in case_list:
                    for x in stations:
                        f = forces.Value(bar.Number, case["number"], x)
                        rows.append((case["number"], x, f.FX, f.FY, f.FZ, f.MY * 1e3, f.MZ * 1e3))
                item: dict[str, Any] = {
                    "bar": bar.Number,
                    "kind": kind,
                    "section": name,
                    "b_cm": info["b_cm"],
                    "h_cm": info["h_cm"],
                }
                if guessed:
                    item["kind_from_geometry"] = True
                problems: list[str] = []
                if kind == "BEAM":
                    mu_pos = max(max(row[5] for row in rows), 0.0)
                    mu_neg = max(-min(row[5] for row in rows), 0.0)
                    as_min = max(0.23 * b * d * ft28 / fe_mpa, 0.001 * b * d)
                    flexure: dict[str, Any] = {
                        "mu_neg_knm": num(mu_neg / 1e6, 2),
                        "mu_pos_knm": num(mu_pos / 1e6, 2),
                        "as_min_cm2": num(as_min / 100, 2),
                        "mu_lim": num(mu_lim, 3),
                    }
                    for face, mu in (("top", mu_neg), ("bottom", mu_pos)):
                        steel, mu_red, _alpha = _beam_steel(mu, b, d, fcd, fyd, mu_lim)
                        flexure[f"mu_{face}"] = num(mu_red, 4)
                        if steel is None:
                            problems.append(f"flexion {face}: μ = {mu_red:.3f} > μlim = {mu_lim:.3f}, augmente le canto")
                            flexure[f"as_{face}_cm2"] = None
                            flexure[f"rho_{face}"] = None
                            continue
                        steel = max(steel, as_min)
                        rho = steel / (b * d)
                        flexure[f"as_{face}_cm2"] = num(steel / 100, 2)
                        flexure[f"rho_{face}"] = num(rho, 4)
                        if rho > 0.04:
                            problems.append(f"ρ {face} = {rho:.2%} > 4 % (BAEL A.4.2.1)")
                    if max(abs(row[2]) for row in rows) > 0.1 * fc28_mpa * b * h:
                        problems.append("effort axial > 0.1·fc28·B : dimensionner comme un poteau")
                    worst = max(rows, key=lambda row: abs(row[4]))
                    shear = _bael_shear(abs(worst[4]), b, d, fc28_mpa, fe_mpa, ft28)
                    item["d_cm"] = num(d / 10, 2)
                    item["flexure"] = flexure
                    item["shear"] = {
                        "vu_kn": num(abs(worst[4]) / 1e3, 2),
                        "case": worst[0],
                        "tau_u_mpa": num(float(shear["tau_u_mpa"]), 3),
                        "tau_lim_mpa": num(float(shear["tau_lim_mpa"]), 3),
                        "av_s_cm2_m": num(float(shear["av_s_mm2_mm"]) * 10, 3),
                        "av_s_min_cm2_m": num(float(shear["av_s_min_mm2_mm"]) * 10, 3),
                    }
                    if not shear["ok"]:
                        problems.append("cortante: τu > τu lim, augmente la section ou les armatures transversales")
                else:
                    demands = [(row[2], row[5], row[6]) for row in rows]
                    rho = _required_rho(demands, b, h, cover, bars_per_face, fcd, fyd, rho_min, RHO_MAX_COLUMN)
                    use, index = _column_utilization(
                        demands, b, h, cover, rho or RHO_MAX_COLUMN, bars_per_face, fcd, fyd
                    )
                    governs = rows[index]
                    item["governs"] = {
                        "case": governs[0],
                        "x": governs[1],
                        "nu_kn": num(governs[2] / 1e3, 2),
                        "muy_knm": num(governs[5] / 1e6, 2),
                        "muz_knm": num(governs[6] / 1e6, 2),
                    }
                    item["rho_min"] = rho_min
                    item["rho_required"] = num(rho, 4) if rho else None
                    item["as_required_cm2"] = num(rho * b * h / 100, 2) if rho else None
                    if rho is None:
                        problems.append(f"ni avec ρ = 4 % ne résiste (utilisation {use:.2f}) : augmente la section")
                    length_mm = math.dist((start.X, start.Y, start.Z), (end.X, end.Y, end.Z)) * 1000
                    lambda_g = length_mm / min(b, h) if min(b, h) > 0 else None
                    item["lambda_g"] = num(lambda_g, 2) if lambda_g is not None else None
                    item["short_column"] = bool(lambda_g is not None and lambda_g < 5)
                    if item["short_column"]:
                        problems.append(f"poteau court (λg = {lambda_g:.2f} < 5) : dispositions RPA24 §7.4.3")
                    min_dim = 250 if zone in ("I", "II", "III") else 300
                    if min(b, h) < min_dim:
                        problems.append(f"dimension minimale {min(b, h) / 10:.1f} cm < {min_dim / 10:.0f} cm (RPA24 §7.4.1)")
                    if not (0.25 <= b / h <= 4):
                        problems.append("rapport b/h hors de l'intervalle 1/4 à 4 (RPA24 §7.4.1)")
                    for axis, column, depth, width in (("shear_z", 4, h, b), ("shear_y", 3, b, h)):
                        worst = max(rows, key=lambda row: abs(row[column]))
                        shear = _bael_shear(abs(worst[column]), width, depth - cover, fc28_mpa, fe_mpa, ft28)
                        item[axis] = {
                            "vu_kn": num(abs(worst[column]) / 1e3, 2),
                            "case": worst[0],
                            "tau_u_mpa": num(float(shear["tau_u_mpa"]), 3),
                            "tau_lim_mpa": num(float(shear["tau_lim_mpa"]), 3),
                            "av_s_cm2_m": num(float(shear["av_s_mm2_mm"]) * 10, 3),
                            "av_s_min_cm2_m": num(float(shear["av_s_min_mm2_mm"]) * 10, 3),
                        }
                        if not shear["ok"]:
                            problems.append(f"cortante ({axis[-1]}): τu > τu lim, augmente la section ou les armatures")
                item["ok"] = not problems
                item["problems"] = problems
                failing += bool(problems)
                items.append(item)
            out = page(total, offset, limit, items)
            materials = {
                "fc28_mpa": fc28_mpa,
                "fe_mpa": fe_mpa,
                "fcd_mpa": num(fcd, 2),
                "fyd_mpa": num(fyd, 2),
                "ft28_mpa": num(ft28, 2),
                "mu_lim": num(mu_lim, 3),
                "zone": zone,
                "rho_min": rho_min,
                "rho_max": RHO_MAX_COLUMN,
                "cover_cm": cover_cm,
                "bars_per_face": bars_per_face,
            }
            return {
                "results_status": status,
                "standard": "RPA 2024 / CBA 93 / BAEL 91",
                "cases": case_list,
                "materials": materials,
                "summary": {"checked": len(items), "failing": failing, "skipped": skipped},
                **out,
            }

        return await on_robot(work)

    if not READ_ONLY_MODE:
        @mcp.tool(
            name="robot_create_rpa24_combinations",
            title="Combinaisons sismiques RPA 2024 (§5.2)",
            annotations=WRITE,
        )
        async def robot_create_rpa24_combinations(
            dead: Annotated[list[int], Field(min_length=1, max_length=20, description="Casos de charges permanentes G.")],
            live: Annotated[list[int] | None, Field(description="Casos de charges d'exploitation Q (sans majoration).")] = None,
            seismic_x: Annotated[list[int] | None, Field(description="Casos sísmicos Ex.")] = None,
            seismic_y: Annotated[list[int] | None, Field(description="Casos sísmicos Ey (mismo número que seismic_x).")] = None,
            vertical_z: Annotated[list[int] | None, Field(description="Caso sísmico vertical Ez (un solo caso, si vertical=true).")] = None,
            occupancy: Annotated[
                Literal["HABITATION", "ERP", "ENTREPOT", "ARCHIVES", "AUTRES"],
                Field(description="Type d'ouvrage pour ψ (Tableau 4.2) si psi est omis."),
            ] = "HABITATION",
            psi: Annotated[
                float | None,
                Field(ge=0, le=1, description="Coefficient d'accompagnement ψ. Omitir = Tableau 4.2 selon occupancy."),
            ] = None,
            both_signs: Annotated[bool, Field(description="Générer les deux signes (±).")] = True,
            foundation: Annotated[bool, Field(description="Ajouter les combinaisons de fondations G + ψQ ± 1.4·E1/E2.")] = False,
            vertical: Annotated[bool, Field(description="Ajouter les combinaisons verticales E3, E4 et E5.")] = False,
            prefix: Annotated[str, Field(min_length=1, max_length=10, description="Préfixe des noms de combinaisons.")] = "RPA24",
            dry_run: DryRun = True,
        ) -> dict[str, Any]:
            """Crée les combinaisons sismiques du RPA 2024 §5.2 (essai par défaut, dry_run=true).

            Génère G + ψQ + E1 et G + ψQ + E2 avec E1 = ±Ex ± 0.3Ey et E2 = ±0.3Ex ± Ey.
            Avec foundation=true, ajoute G + ψQ ± 1.4·E1 et ± 1.4·E2 (fondations communes).
            Avec vertical=true, ajoute E3 = ±Ex ± 0.3Ey ± 0.3Ez, E4 = ±0.3Ex ± Ey ± 0.3Ez et
            E5 = ±0.3Ex ± 0.3Ey ± Ez. Les cas Ex et Ey sont appariés par position dans les listes.
            Devuelve {dry_run, applied, results_status_before, invalidates_results, to_create:[…],
            next_step? | created, results_status_after}. Ne modifie rien avec dry_run=true. Les
            noms sont préfixés et tronqués à 60 caractères. Falla si un caso no existe, si un nom
            existe déjà, ou si la liste génère plus de 60 combinaisons.
            """
            live = live or []
            seismic_x = seismic_x or []
            seismic_y = seismic_y or []
            vertical_z = vertical_z or []
            if not seismic_x or not seismic_y:
                raise ToolError("Indica al menos un caso sísmico en X y otro en Y.")
            if len(seismic_x) != len(seismic_y):
                raise ToolError("seismic_x y seismic_y deben tener el mismo número de casos (se emparejan por posición).")
            if vertical and len(vertical_z) != 1:
                raise ToolError("Con vertical=true, vertical_z debe contener exactamente un caso Ez.")
            if vertical and not vertical_z:
                raise ToolError("Con vertical=true, indica el caso vertical Ez.")
            psi_value = float(psi) if psi is not None else PSI[occupancy]
            groups = {"G": dead, "Q": live, "Ex": seismic_x, "Ey": seismic_y, "Ez": vertical_z}
            seen: dict[int, str] = {}
            for key, cases in groups.items():
                for case in cases:
                    if case in seen:
                        raise ToolError(f"El caso {case} está a la vez en {seen[case]} y en {key}; cada caso va en una sola categoría.")
                    seen[case] = key
            signs = (1.0, -1.0) if both_signs else (1.0,)
            combos: list[CombinationIn] = []

            def add(terms: list[tuple[float, str, list[int]]]) -> None:
                factors: dict[int, float] = {}
                label = ""
                for index, (factor, symbol, cases) in enumerate(terms):
                    if not cases or factor == 0:
                        continue
                    sign = "-" if factor < 0 else ("" if index == 0 else "+")
                    label += f"{sign}{'' if abs(factor) == 1 else f'{abs(factor):g}'}{symbol}"
                    for case in cases:
                        factors[case] = round(factors.get(case, 0.0) + factor, 4)
                name = f"{prefix}{len(combos) + 1:02d} {label}"[:60]
                combos.append(
                    CombinationIn(
                        name=name,
                        type="ALS",
                        nature="SEISMIC",
                        factors=[FactorIn(case=case, factor=factor) for case, factor in factors.items() if factor],
                    )
                )

            gravity = [(1.0, "G", dead), (psi_value, "Q", live)]
            for ex, ey in zip(seismic_x, seismic_y):
                for sx in signs:
                    for sy in signs:
                        add([*gravity, (sx, f"Ex{ex}", [ex]), (0.3 * sy, f"Ey{ey}", [ey])])
                        add([*gravity, (0.3 * sx, f"Ex{ex}", [ex]), (sy, f"Ey{ey}", [ey])])
                        if foundation:
                            add([*gravity, (1.4 * sx, f"Ex{ex}", [ex]), (0.42 * sy, f"Ey{ey}", [ey])])
                            add([*gravity, (0.42 * sx, f"Ex{ex}", [ex]), (1.4 * sy, f"Ey{ey}", [ey])])
                if vertical:
                    z = vertical_z[0]
                    for sx in signs:
                        for sy in signs:
                            for sz in signs:
                                add([*gravity, (sx, f"Ex{ex}", [ex]), (0.3 * sy, f"Ey{ey}", [ey]), (0.3 * sz, f"Ez{z}", [z])])
                                add([*gravity, (0.3 * sx, f"Ex{ex}", [ex]), (sy, f"Ey{ey}", [ey]), (0.3 * sz, f"Ez{z}", [z])])
                                add([*gravity, (0.3 * sx, f"Ex{ex}", [ex]), (0.3 * sy, f"Ey{ey}", [ey]), (sz, f"Ez{z}", [z])])
            if len(combos) > 60:
                raise ToolError(
                    f"Se generarían {len(combos)} combinaciones; reduce el número de casos o desactiva foundation/vertical."
                )
            return await on_robot(lambda s: _combinations(s, combos, dry_run))
