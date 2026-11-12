"""Conexión COM con Autodesk Robot Structural Analysis (RobotOM).

Robot es un servidor COM fuera de proceso. Todo acceso pasa por UN solo hilo
(`on_robot`), porque los objetos COM no se pueden compartir entre hilos sin
marshalling y el SDK de MCP ejecuta las herramientas en un pool.
"""

from __future__ import annotations

import asyncio
import glob
import logging
import os
import re
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from typing import Any, Callable, Iterator, TypeVar

from mcp.server.mcpserver.exceptions import ToolError

log = logging.getLogger("robot_mcp")
T = TypeVar("T")

INTEROP_GLOB = r"C:\Program Files\Autodesk\Robot Structural Analysis Professional *\Exe\Interop.RobotOM.dll"
AUTOSTART = os.environ.get("ROBOT_MCP_AUTOSTART", "").strip().lower() in ("1", "true", "yes", "si", "sí")
READ_ONLY = os.environ.get("ROBOT_MCP_READ_ONLY", "").strip().lower() in ("1", "true", "yes", "si", "sí")
CALL_TIMEOUT_S = float(os.environ.get("ROBOT_MCP_TIMEOUT", "120"))
MAX_EXPLICIT_NUMBERS = 20000
STARTUP_GRACE_S = float(os.environ.get("ROBOT_MCP_STARTUP_GRACE", "40"))  # ver robot_starting()
STARTING_MESSAGE = "Robot todavía está arrancando. Espera a que termine de abrirse y reintenta en unos segundos."

_com_thread = ThreadPoolExecutor(max_workers=1, thread_name_prefix="robot-com")
_om: Any = None  # módulo RobotOM (pythonnet)
_app: Any = None  # RobotApplication


def interop_path() -> str:
    """Ruta de Interop.RobotOM.dll: ROBOT_INTEROP_DLL o la versión instalada más reciente."""
    explicit = os.environ.get("ROBOT_INTEROP_DLL")
    if explicit:
        if not os.path.isfile(explicit):
            raise ToolError(f"ROBOT_INTEROP_DLL apunta a un archivo que no existe: {explicit}")
        return explicit
    found = sorted(glob.glob(INTEROP_GLOB))
    if not found:
        raise ToolError(
            "No se encontró Interop.RobotOM.dll. Instala Robot Structural Analysis o define la variable "
            "de entorno ROBOT_INTEROP_DLL con la ruta completa de la DLL."
        )
    return found[-1]


def _load_om() -> Any:
    global _om
    if _om is None:
        import clr  # pythonnet

        path = interop_path()
        clr.AddReference(path)
        import RobotOM  # type: ignore[import-not-found]

        _om = RobotOM
        log.info("RobotOM cargado desde %s", path)
    return _om


def robot_process_count() -> int:
    import clr  # noqa: F401
    from System.Diagnostics import Process  # type: ignore[import-not-found]

    return len(Process.GetProcessesByName("robot"))


def robot_pids() -> set[int]:
    import clr  # noqa: F401
    from System.Diagnostics import Process  # type: ignore[import-not-found]

    return {int(process.Id) for process in Process.GetProcessesByName("robot")}


def robot_starting() -> bool:
    """True si algún Robot lleva abierto menos de STARTUP_GRACE_S.

    Mientras arranca, Robot aún no ha registrado su servidor COM: pedir Robot.Application en
    ese momento hace que Windows abra OTRA instancia, oculta, que se queda viva (medido: ocurre
    a los 2, 5 y ~15 s del arranque; a los 45 s ya no). No hay forma de consultar ese registro
    sin provocar el arranque, así que se usa la edad del proceso.
    """
    import clr  # noqa: F401
    from System import DateTime  # type: ignore[import-not-found]
    from System.Diagnostics import Process  # type: ignore[import-not-found]

    for process in Process.GetProcessesByName("robot"):
        try:
            if (DateTime.Now - process.StartTime).TotalSeconds < STARTUP_GRACE_S:
                return True
        except Exception:  # el proceso terminó entre la lista y la consulta
            continue
    return False


def _kill_spawned(pids: set[int]) -> None:
    """Cierra instancias ocultas de Robot que nuestra propia petición COM acaba de provocar.

    Solo procesos nacidos durante la llamada y sin título de ventana: no pueden tener un modelo.
    """
    from System.Diagnostics import Process  # type: ignore[import-not-found]

    for pid in pids:
        try:
            process = Process.GetProcessById(pid)
            if not process.MainWindowTitle:
                process.Kill()
                log.warning("Cerrada la instancia oculta de Robot %d abierta por la activación COM", pid)
        except Exception:
            log.exception("No se pudo cerrar la instancia oculta %d", pid)


def create_application(om: Any) -> Any:
    """Instancia Robot.Application por ProgID (se engancha al Robot abierto).

    No usar om.RobotApplicationClass(): pythonnet 3 no puede activar coclases COM
    y el fallo es una excepción .NET no controlada que mata el proceso.
    """
    from System import Activator, Type  # type: ignore[import-not-found]

    com_type = Type.GetTypeFromProgID("Robot.Application")
    if com_type is None:
        raise ToolError("La API COM de Robot (Robot.Application) no está registrada. Reinstala o repara Robot.")
    return om.IRobotApplication(Activator.CreateInstance(com_type))


# ---------- selecciones ----------

_WORD = r"[^\W\d_]+"  # palabra clave de rango/paso en cualquier idioma: to, A, By, CA…
_SEL_TOKEN = re.compile(rf"^(\d+)(?:{_WORD}(\d+)(?:{_WORD}(\d+))?)?$")
_SEL_SPACED = re.compile(rf"(\d)\s+({_WORD})\s+(?=\d)")
_RANGE_IN_TEXT = re.compile(rf"\d({_WORD})\d")
_RANGE_ANY = re.compile(rf"(\d+){_WORD}(\d+)(?:{_WORD}(\d+))?")


def parse_selection(text: str) -> list[tuple[int, int, int]]:
    """'1to10By2 15' -> [(1, 10, 2), (15, 15, 1)]. Acepta la palabra clave de cualquier idioma."""
    ranges = []
    for token in re.split(r"[\s,;]+", _SEL_SPACED.sub(r"\1\2", text.strip())):
        if not token:
            continue
        match = _SEL_TOKEN.match(token)
        first = int(match.group(1)) if match else 0
        last = int(match.group(2) or first) if match else 0
        step = int(match.group(3) or 1) if match else 0
        if not match or last < first or step < 1:
            raise ToolError(
                f"Selección no válida: '{token}'. Formatos admitidos: '5', '1to10', '1to20By2', "
                "separados por espacios. Para todos los elementos, omite la selección."
            )
        ranges.append((first, last, step))
    return ranges


def canonical_selection(text: str) -> str:
    """Texto de selección de Robot (en su idioma) -> forma estable '1to4 9to12', '1to7By2'."""

    def fix(match: re.Match[str]) -> str:
        return f"{match.group(1)}to{match.group(2)}" + (f"By{match.group(3)}" if match.group(3) else "")

    return " ".join(_RANGE_ANY.sub(fix, text).split())


class Session:
    """Lo que recibe cada función que se ejecuta en el hilo COM."""

    def __init__(self, om: Any, app: Any) -> None:
        self.om = om
        self.app = app

    @property
    def project(self) -> Any:
        return self.app.Project

    @property
    def structure(self) -> Any:
        return self.app.Project.Structure

    def require_project(self) -> None:
        if not self.app.Project.IsActive:
            raise ToolError("Robot está abierto pero no hay ningún proyecto activo. Abre o crea un modelo y reintenta.")

    def select(
        self,
        obj_type: Any,
        text: str | None = None,
        *,
        storey: str | None = None,
        label: tuple[Any, str] | None = None,
    ) -> Any:
        """Selección de Robot: texto ('1to10 15'), piso y/o etiqueta, combinados con AND."""
        factory = self.structure.Selections
        if text and text.strip():
            sel = self._from_text(obj_type, text)
        else:
            sel = factory.CreateFull(obj_type)
        if storey:
            names = self.storey_names()
            if storey not in names:
                raise ToolError(f"No existe el piso '{storey}'. Pisos definidos: {names or 'ninguno'}.")
            sel.And(factory.CreateByStorey(obj_type, storey))
        if label:
            label_type, label_name = label
            if not self.structure.Labels.Exist(label_type, label_name):
                raise ToolError(f"No existe la etiqueta '{label_name}' en el proyecto. Lista las disponibles primero.")
            sel.And(factory.CreateByLabel(obj_type, label_type, label_name))
        return sel

    def _from_text(self, obj_type: Any, text: str) -> Any:
        """Traduce '1to10By2' a la sintaxis del idioma de Robot antes de FromText.

        La palabra clave de rango depende del idioma ('to' en inglés, 'A' en español) y un
        texto que Robot no entiende puede seleccionarlo TODO ('1to4' en español = todos), así
        que la palabra clave se aprende del propio Robot y los rangos con paso se expanden.
        """
        ranges = parse_selection(text)
        factory = self.structure.Selections
        found = _RANGE_IN_TEXT.search(str(factory.CreateFull(obj_type).ToText()))
        keyword = found.group(1) if found else None
        parts: list[str] = []
        requested = 0
        for first, last, step in ranges:
            size = (last - first) // step + 1
            requested += size
            if first == last:
                parts.append(str(first))
            elif step == 1 and keyword:
                parts.append(f"{first}{keyword}{last}")
            else:
                if size > MAX_EXPLICIT_NUMBERS:
                    raise ToolError(f"El rango {first}-{last} es demasiado grande para expandirlo; divídelo en tramos.")
                parts.extend(str(n) for n in range(first, last + 1, step))
        sel = factory.Create(obj_type)
        sel.FromText(" ".join(parts))
        if sel.Count > requested:  # Robot interpretó otra cosa: nunca devolver de más
            raise ToolError(f"Robot no interpretó la selección '{text}' como se esperaba. Usa números sueltos: '1 2 3'.")
        return sel

    def storey_names(self) -> list[str]:
        mngr = self.structure.Storeys
        return [str(mngr.Get(i).Name) for i in range(1, mngr.Count + 1)]


def project_labels(s: Session, label_type: Any, *, only_used: bool) -> list[Any]:
    """Etiquetas del proyecto de un tipo.

    Labels.GetMany no las devuelve todas (omite, p. ej., espesores ortótropos y su material),
    así que se unen sus nombres con GetAvailableNames y se filtra con Exist / IsUsed.
    """
    labels = s.structure.Labels
    names: dict[str, None] = {}
    col = labels.GetMany(label_type)
    for i in range(1, col.Count + 1):
        names[str(s.om.IRobotLabel(col.Get(i)).Name)] = None
    available = labels.GetAvailableNames(label_type)
    for i in range(1, available.Count + 1):
        names[str(available.Get(i))] = None
    out = []
    for name in names:
        if labels.Exist(label_type, name) and (not only_used or labels.IsUsed(label_type, name)):
            out.append(labels.Get(label_type, name))
    return out


def generated_records(om: Any, records: Any) -> set[int]:
    """Índices de los registros de carga que Robot genera solo (reparto de paneles a barras)."""
    return {i for i in range(1, records.Count + 1) if om.IRobotLoadRecordCommon(records.Get(i)).IsAutoGenerated}


def bar_selection(
    s: Session,
    selection: str | None = None,
    section: str | None = None,
    storey: str | None = None,
    structural_type: str | None = None,
) -> Any:
    """Selección de barras combinando números, sección, piso y tipo estructural (AND).

    Robot no selecciona por tipo estructural, así que ese filtro recorre las barras que
    pasan los otros filtros (una lectura por barra).
    """
    om = s.om
    bar_type = om.IRobotObjectType.I_OT_BAR
    label = (om.IRobotLabelType.I_LT_BAR_SECTION, section) if section else None
    chosen = s.select(bar_type, selection, storey=storey, label=label)
    if not structural_type:
        return chosen
    server = s.structure.Bars
    kept = s.structure.Selections.Create(bar_type)
    for i in range(1, chosen.Count + 1):
        number = chosen.Get(i)
        if enum_name(server.GetStructuralType(number)) == structural_type:
            kept.AddOne(number)
    return kept


def section_item(s: Session, label: Any) -> dict[str, Any]:
    """Propiedades de una etiqueta de sección de barra (lo que devuelve robot_list_sections)."""
    om = s.om
    lt = om.IRobotLabelType.I_LT_BAR_SECTION
    dv = om.IRobotBarSectionDataValue
    cv = om.IRobotBarSectionConcreteDataValue
    name = str(label.Name)
    data = om.IRobotBarSectionData(label.Data)
    item: dict[str, Any] = {
        "name": name,
        "shape": enum_name(data.ShapeType),
        "material": str(data.MaterialName) or None,
        "is_concrete": bool(data.IsConcrete),
        "bars_count": s.structure.Selections.CreateByLabel(om.IRobotObjectType.I_OT_BAR, lt, name).Count,
    }
    b = h = 0.0
    if data.IsConcrete:
        concrete = data.Concrete
        b = concrete.GetValue(cv.I_BSCDV_BEAM_B)
        h = concrete.GetValue(cv.I_BSCDV_BEAM_H)
        item["b_cm"] = num(b * 100, 2)
        item["h_cm"] = num(h * 100, 2)
        reduced, ix, iy, iz = concrete.GetReduction(0.0, 0.0, 0.0)
        if reduced:
            item["inertia_reduction"] = {"ix": num(ix, 4), "iy": num(iy, 4), "iz": num(iz, 4)}
    stored_iy = data.GetValue(dv.I_BSDV_IY)
    stored_iz = data.GetValue(dv.I_BSDV_IZ)
    item["area_cm2"] = num(data.GetValue(dv.I_BSDV_AX) * 1e4, 3)
    item["ix_cm4"] = num(data.GetValue(dv.I_BSDV_IX) * 1e8, 2)
    item["iy_cm4"] = num(stored_iy * 1e8, 2)
    item["iz_cm4"] = num(stored_iz * 1e8, 2)
    if item["shape"] in ("CONCR_COL_R", "CONCR_BEAM_RECT") and b > 0 and h > 0:
        item["iy_stored_over_gross"] = num(stored_iy / (b * h**3 / 12), 3)
        item["iz_stored_over_gross"] = num(stored_iz / (h * b**3 / 12), 3)
        # OJO: esta relación NO dice si el análisis aplica la reducción (ver README, "Inercia
        # agrietada"): un edificio real con relación 1.0 analiza con la inercia reducida.
    return item


# ---------- modo no interactivo (escrituras y cálculo) ----------

_INTERACTIVE_MARKER = os.path.join(tempfile.gettempdir(), "robot_mcp_non_interactive.marker")


@contextmanager
def non_interactive(app: Any) -> Iterator[None]:
    """Mientras dura el bloque, Robot no abre diálogos de pregunta.

    Con Interactive = 1, modificar por API un modelo calculado abre «Los resultados de cálculos
    pueden desactualizarse. ¿Desea continuar?» y la llamada COM queda bloqueada hasta que alguien
    responde. Con Interactive = 0 el usuario tampoco puede usar la ventana, así que se restaura
    siempre al salir. El archivo marcador permite repararlo si el servidor muere a mitad, y
    también avisa de que otro servidor (otro chat) está escribiendo: entonces se rechaza.
    """
    if _busy_owner():
        raise ToolError("Otro chat está escribiendo o calculando en Robot ahora mismo. Espera a que termine y reintenta.")
    previous = app.Interactive
    if previous:
        with open(_INTERACTIVE_MARKER, "w", encoding="utf-8") as marker:
            marker.write(str(os.getpid()))
        app.Interactive = 0
    try:
        yield
    finally:
        if previous:
            app.Interactive = previous
            _remove_marker()


def _remove_marker() -> None:
    try:
        os.remove(_INTERACTIVE_MARKER)
    except OSError:
        pass


def _alive(pid: int) -> bool:
    import clr  # noqa: F401
    from System.Diagnostics import Process  # type: ignore[import-not-found]

    try:
        return not Process.GetProcessById(pid).HasExited
    except Exception:  # ArgumentException: no hay proceso con ese número
        return False


def _busy_owner() -> int:
    """PID de OTRO servidor vivo que tiene el marcador (está escribiendo ahora), o 0."""
    try:
        with open(_INTERACTIVE_MARKER, encoding="utf-8") as marker:
            owner = int(marker.read().strip() or 0)
    except (OSError, ValueError):
        return 0
    return owner if owner and owner != os.getpid() and _alive(owner) else 0


def _repair_interactive(app: Any) -> None:
    """Si un servidor anterior murió dejando Robot sin interacción, se la devuelve."""
    if os.path.exists(_INTERACTIVE_MARKER):
        if _busy_owner():
            return  # otro servidor (otro chat) está escribiendo ahora mismo: no tocar
        if not app.Interactive:
            app.Interactive = 1
            log.warning("Robot había quedado no interactivo tras un fallo anterior; restaurado")
        _remove_marker()


# ---------- conexión ----------


def _connect() -> Any:
    """Devuelve RobotApplication enganchado al Robot abierto (o lo arranca si AUTOSTART)."""
    global _app
    om = _load_om()
    before = robot_pids()
    running = bool(before)
    if not running:
        _app = None
        if not AUTOSTART:
            raise ToolError(
                "Robot Structural Analysis no está abierto. Ábrelo, carga un modelo y vuelve a llamar. "
                "(Con ROBOT_MCP_AUTOSTART=1 el servidor lo arranca solo.)"
            )
    if _app is not None:
        try:
            _ = _app.Visible  # ping: falla si Robot se cerró y se volvió a abrir
            _repair_interactive(_app)
            return _app
        except Exception:
            log.info("Conexión COM caducada; reconectando")
            _app = None
    if running and robot_starting():
        raise ToolError(STARTING_MESSAGE)
    app = create_application(om)
    if not running:
        app.Visible = 1
        app.Interactive = 1
    else:
        # Red de seguridad por si el margen de arranque no bastó: si la petición COM abrió
        # una instancia nueva, se cierra y se pide reintentar en vez de quedarse con ella.
        time.sleep(0.5)
        spawned = robot_pids() - before
        if spawned:
            del app
            _kill_spawned(spawned)
            raise ToolError(STARTING_MESSAGE)
        _repair_interactive(app)
    _app = app
    return _app


def _translate(exc: Exception) -> ToolError:
    hresult = getattr(exc, "HResult", None)
    code = f" (HRESULT 0x{hresult & 0xFFFFFFFF:08X})" if isinstance(hresult, int) else ""
    message = str(getattr(exc, "Message", None) or exc)
    return ToolError(
        f"Robot rechazó la operación{code}: {message}. "
        "Comprueba que no haya un diálogo abierto en Robot ni un cálculo en curso."
    )


def _run(fn: Callable[[Session], T], need_project: bool) -> T:
    try:
        session = Session(_load_om(), _connect())
        if need_project:
            session.require_project()
        return fn(session)
    except ToolError:
        raise
    except Exception as exc:  # COMException y demás errores .NET llegan como excepciones Python
        log.exception("Fallo en llamada a Robot")
        raise _translate(exc) from exc


_overdue: Any = None  # llamada que superó su tiempo y sigue ocupando el hilo COM


async def on_robot(fn: Callable[[Session], T], *, need_project: bool = True, timeout: float | None = None) -> T:
    """Ejecuta `fn(session)` en el hilo COM y espera el resultado."""
    global _overdue
    if _overdue is not None and not _overdue.done():
        raise ToolError(
            "Robot sigue ocupado con una llamada anterior que superó el tiempo de espera (p. ej. un cálculo largo "
            "o un diálogo abierto en Robot). Espera a que termine y reintenta."
        )
    limit = timeout or CALL_TIMEOUT_S
    future = asyncio.get_running_loop().run_in_executor(_com_thread, _run, fn, need_project)
    try:
        # shield: al vencer el plazo el hilo sigue trabajando; el futuro debe reflejarlo, no quedar cancelado
        return await asyncio.wait_for(asyncio.shield(future), limit)
    except asyncio.TimeoutError:
        _overdue = future
        raise ToolError(
            f"Robot no respondió en {limit:.0f} s y la llamada sigue en curso. Suele ser un cálculo largo o un "
            "diálogo modal abierto en Robot: deja que termine o ciérralo, y reintenta."
        ) from None


# ---------- utilidades de formato ----------

_ENUM_PREFIX = re.compile(r"^I_[A-Z]+_")
# Robot define varios nombres para el mismo valor; se devuelve siempre el más claro.
_ENUM_ALIASES = {
    "EFF": "ULS",
    "DEP": "SLS",
    "ACC": "ALS",
    "EXPLOATATION": "LIVE",
    "STATIC_FLAMBEMENT": "STATIC_BUCKLING",
    "STATIC_NONLINEAR_FLAMBEMENT": "STATIC_NONLINEAR_BUCKLING",
    "TEMPORAR": "TIME_HISTORY",
}


def enum_name(value: Any) -> str:
    """'I_CN_PERMANENT' -> 'PERMANENT', resolviendo alias. Valores sin nombre devuelven el número."""
    name = _ENUM_PREFIX.sub("", str(value.ToString()))
    return _ENUM_ALIASES.get(name, name)


def num(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def page(total: int, offset: int, limit: int, items: list[Any]) -> dict[str, Any]:
    end = offset + len(items)
    out: dict[str, Any] = {"total": total, "count": len(items), "offset": offset, "has_more": end < total}
    if end < total:
        out["next_offset"] = end
    out["items"] = items
    return out


def window(total: int, offset: int, limit: int) -> range:
    """Índices 1-based de la página pedida dentro de una colección de Robot."""
    return range(offset + 1, min(offset + limit, total) + 1)
