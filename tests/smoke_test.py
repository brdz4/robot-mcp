"""Prueba de humo: arranca el servidor por stdio, lista las herramientas y llama a cada una.

Uso:  .venv\\Scripts\\python.exe tests\\smoke_test.py [--full]
Con Robot cerrado comprueba el arranque y los mensajes de error; con un modelo
abierto muestra los datos reales. --full imprime las respuestas completas.
"""

from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

from mcp import Client, StdioServerParameters

ROOT = Path(__file__).resolve().parent.parent
FULL = "--full" in sys.argv

CALLS: list[tuple[str, dict]] = [
    ("robot_health", {}),
    ("robot_get_project_info", {}),
    ("robot_list_storeys", {}),
    ("robot_list_nodes", {"limit": 5}),
    ("robot_list_nodes", {"selection": "1to4", "limit": 2, "offset": 2}),
    ("robot_list_nodes", {"selection": "1to12By4 6", "limit": 10}),
    ("robot_list_nodes", {"selection": "todo"}),
    ("robot_list_bars", {"limit": 3}),
    ("robot_list_bars", {"section": "V 30x50", "storey": "Piso 2"}),
    ("robot_list_bars", {"selection": "1to10", "storey": "Piso 1", "limit": 2}),
    ("robot_list_panels", {"limit": 5}),
    ("robot_list_thicknesses", {}),
    ("robot_list_sections", {}),
    ("robot_list_materials", {}),
    ("robot_list_supports", {}),
    ("robot_list_load_cases", {}),
    ("robot_list_combinations", {}),
    # --- resultados (los números de caso son los del edificio de referencia; en otro modelo
    #     las llamadas con casos inexistentes devuelven un error anticipado)
    ("robot_get_reactions", {"cases": "1 5 7", "limit": 3}),
    ("robot_get_node_displacements", {"cases": "5 7", "selection": "2 120"}),
    ("robot_get_bar_forces", {"cases": "5", "limit": 2}),
    ("robot_get_bar_forces", {"cases": "1to200", "limit": 500, "points": 11}),
    ("robot_get_result_extremes", {"quantity": "bar_forces", "cases": "101to108", "section": "V40x60", "components": ["MY", "FZ"]}),
    ("robot_get_result_extremes", {"quantity": "bar_forces", "cases": "101to108", "structural_type": "COLUMN", "components": ["FX"]}),
    ("robot_list_bars", {"storey": "Planta 4", "structural_type": "BEAM", "limit": 1}),
    ("robot_list_bars", {"structural_type": "COLUMN", "limit": 1}),
    ("robot_get_bar_forces", {"cases": "1", "structural_type": "COLUMN", "storey": "Planta 1", "limit": 1, "points": 2}),
    ("robot_get_result_extremes", {"quantity": "reactions", "cases": "101to108", "components": ["FZ"]}),
    ("robot_get_result_extremes", {"quantity": "node_displacements", "cases": "7 10", "components": ["UX", "UY"]}),
    ("robot_get_result_extremes", {"quantity": "reactions", "cases": "5", "components": ["UX"]}),
    ("robot_get_modal_results", {"max_modes": 6}),
    ("robot_get_storey_results", {"cases": "7 10"}),
    ("robot_get_reactions", {"cases": "999"}),
    ("robot_list_bars", {"section": "__no_existe__"}),
    ("robot_list_nodes", {"limit": 0}),
]


async def main() -> int:
    params = StdioServerParameters(command=sys.executable, args=[str(ROOT / "server.py")], cwd=str(ROOT))
    failures = 0
    async with Client(params) as client:
        tools = (await client.list_tools()).tools
        print(f"== {len(tools)} herramientas")
        for tool in tools:
            ann = tool.annotations
            print(f"   {tool.name:28s} readOnly={getattr(ann, 'read_only_hint', None)}  {tool.title}")
        for name, args in CALLS:
            result = await client.call_tool(name, args)
            text = "".join(getattr(block, "text", "") for block in result.content)
            status = "ERROR" if result.is_error else "ok"
            try:
                text = json.dumps(json.loads(text), ensure_ascii=False, indent=2 if FULL else None)
            except ValueError:
                pass
            if not FULL and len(text) > 900:
                text = text[:900] + " …"
            print(f"\n== {name} {json.dumps(args, ensure_ascii=False)} -> {status}")
            print(text)
            if result.is_error and text.strip() == f"Error executing tool {name}":
                failures += 1  # sin mensaje = excepción no anticipada: bug del servidor
    print(f"\nfallos no anticipados: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
