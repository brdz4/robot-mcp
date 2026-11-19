"""Tipos de parámetro y anotaciones compartidos por las herramientas."""

from __future__ import annotations

from typing import Annotated, Literal

from mcp_types import ToolAnnotations
from pydantic import Field

READ_ONLY = ToolAnnotations(read_only_hint=True, destructive_hint=False, idempotent_hint=True, open_world_hint=False)

WRITE = ToolAnnotations(read_only_hint=False, destructive_hint=False, idempotent_hint=False, open_world_hint=False)
OVERWRITE = ToolAnnotations(read_only_hint=False, destructive_hint=True, idempotent_hint=True, open_world_hint=False)

DryRun = Annotated[
    bool,
    Field(
        description="true (por defecto) = solo valida y describe lo que haría, sin modificar nada. "
        "false = aplica el cambio. Haz siempre primero el ensayo y muestra el plan al usuario."
    ),
]
DiscardResults = Annotated[
    bool,
    Field(
        description="Obligatorio true para aplicar un cambio que invalida resultados vigentes del modelo "
        "(habrá que recalcular). Ponlo solo si el usuario lo aceptó."
    ),
]

Selection = Annotated[
    str | None,
    Field(description="Números de elemento: '1to10', '3 7 12', '1to50By5'. Omitir = todos."),
]
Limit = Annotated[int, Field(ge=1, le=500, description="Máximo de elementos a devolver (1-500).")]
Offset = Annotated[int, Field(ge=0, description="Elementos a saltar; usa next_offset de la respuesta anterior.")]
Cases = Annotated[
    str,
    Field(
        description="Números de caso o combinación: '5', '1 2 3', '101to108' (ver robot_list_load_cases). "
        "Al leer resultados se ignoran los que no existan; en las verificaciones robot_check_* es un error."
    ),
]
SectionFilter = Annotated[
    str | None, Field(description="Nombre exacto de una sección (ver robot_list_sections) para filtrar barras.")
]
StoreyFilter = Annotated[
    str | None,
    Field(
        description="Nombre exacto de un piso (ver robot_list_storeys). Incluye las columnas que "
        "nacen en ese piso y las vigas de su nivel superior; las vigas de cubierta son las del último piso."
    ),
]
StructuralTypeFilter = Annotated[
    Literal["COLUMN", "BEAM"] | None,
    Field(description="Filtra por tipo estructural de la barra: COLUMN (columnas) o BEAM (vigas)."),
]
