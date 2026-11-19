# robot_mcp

Servidor MCP para Autodesk Robot Structural Analysis. Se conecta por COM (RobotOM) al Robot
que tengas abierto y deja que Claude lea el modelo activo y sus resultados, y que lo modifique
con ensayo previo (crear, asignar y, con permiso expreso, borrar).

> **Estado: en pruebas, pero funciona.** Lo he probado en copias de proyectos reales con NEC-15
> y ACI 318-19 y pasan todas sus pruebas (162 comprobaciones, 156 de ellas contra Robot), pero
> aún no se ha usado fuera de mi equipo. Trabaja siempre sobre una copia del modelo, revisa el ensayo (`dry_run`) antes de
> aplicar y comprueba los resultados de diseño a mano. Se agradecen issues y PR.

Probado con Robot 2026 (39.0.1) en español, Python 3.12, `mcp` 2.3.0 y `pythonnet` 3.2.1,
contra un pórtico de prueba y la copia de un edificio real.

*English:* MCP server for Autodesk Robot Structural Analysis (Windows, COM API). It lets Claude
read the open model and its results, run Ecuadorian seismic code checks (NEC-SE-DS), size
reinforcement per ACI 318-19 and make guarded edits (every write is a dry run first). Status:
in testing but working. Tool descriptions and messages are in Spanish. MIT license.

## Instalación

```bash
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Registro en Claude Code (`.mcp.json` en la carpeta del proyecto):

```json
{
  "mcpServers": {
    "robot": {
      "command": "C:/ruta/robot_mcp/.venv/Scripts/python.exe",
      "args": ["C:/ruta/robot_mcp/server.py"]
    }
  }
}
```

## Herramientas

| Herramienta | Qué devuelve |
|---|---|
| `robot_health` | Si Robot está abierto, versión, proyecto activo, estado de resultados |
| `robot_get_project_info` | Nombre, archivo, tipo de estructura y conteos |
| `robot_list_nodes` | Nudos: coordenadas (m) y apoyo |
| `robot_list_bars` | Barras: nudos, longitud, sección, material. Filtros: selección, sección, piso, columna o viga |
| `robot_list_panels` | Paneles: espesor, material y modelo de cálculo (elementos finitos o solo reparto de cargas, diafragma) |
| `robot_list_storeys` | Pisos: niveles y alturas |
| `robot_list_sections` | Secciones: b, h, área, inercias, coeficientes de inercia agrietada |
| `robot_list_materials` | Materiales: E, ν, peso unitario, resistencia y dónde se usan |
| `robot_list_thicknesses` | Espesores de panel: homogéneos (cm) u ortótropos (tipo y dimensiones) |
| `robot_list_supports` | Tipos de apoyo, grados de libertad fijos y nudos asignados |
| `robot_list_load_cases` | Casos: naturaleza y tipo de análisis |
| `robot_list_combinations` | Combinaciones manuales con sus factores |
| `robot_list_loads` | Cargas de un caso simple: tipo, elementos y valores (kN, kN/m, kN/m²), para auditarlo |

Resultados (`results.py`; requieren el modelo calculado):

| Herramienta | Qué devuelve |
|---|---|
| `robot_get_reactions` | Reacciones por nudo apoyado y caso, y suma por caso (peso, cortante basal) |
| `robot_get_node_displacements` | Desplazamientos y giros de nudos por caso |
| `robot_get_bar_forces` | Esfuerzos N, V, M, T por barra, caso y estación |
| `robot_get_result_extremes` | Máximo y mínimo de esfuerzos, reacciones o desplazamientos, con ubicación (barra, piso, sección, caso) |
| `robot_get_modal_results` | Periodos, masas participantes, acumulados y modo en que se alcanza el 90 % |
| `robot_get_storey_results` | Tabla de pisos: desplazamientos, derivas, cortantes, masa y centros |
| `robot_get_bar_deflections` | Flecha máxima de cada barra respecto a su cuerda, L/flecha y, si se pide, si cumple L/n |
| `robot_get_panel_forces` | Máximo y mínimo de MXX, MYY, MXY, NXX… en paneles con elementos finitos, con su nudo |

Verificaciones NEC-SE-DS 2015 (`checks.py`; solo lectura, también en modo `ROBOT_MCP_READ_ONLY`):

| Herramienta | Qué verifica |
|---|---|
| `robot_check_drifts` | Deriva inelástica ΔM = 0.75·R·ΔE por piso frente al límite (§6.3.9), con la tabla de pisos y con la peor línea de pórtico (la que gobierna sin diafragma rígido) |
| `robot_check_p_delta` | Índice de estabilidad Qi = Pi·Δi/(Vi·hi) por piso (§6.3.8) |
| `robot_check_base_shear` | Cortante dinámico ≥ 80 % u 85 % del estático, y factor de ajuste (§6.2.2) |
| `robot_check_irregularities` | Irregularidad torsional, piso flexible y masa (§5.2) |
| `robot_check_rc_members` | Cuantías ACI 318-19 de vigas y columnas rectangulares de hormigón (`design.py`): As superior e inferior, ρ de columna por flexocompresión biaxial y Av/s de estribos |

En `robot_check_rc_members` se pasan las combinaciones últimas, y f'c y fy son datos del
proyecto. Las columnas se resuelven por compatibilidad de deformaciones con las barras
repartidas en el perímetro y la flexión biaxial por el contorno lineal Mx/φMnx + My/φMny ≤ 1,
que es conservador. Los momentos se toman en los nudos, no a la cara del apoyo. No incluye
diseño por capacidad (cortante por momentos probables, columna fuerte-viga débil) ni esbeltez.
Las secciones que no son rectangulares de hormigón, como las dobles T prefabricadas, se omiten.

R, los factores de escala de cada caso sísmico y el cortante estático (o su coeficiente) son
datos del proyecto que aporta quien llama: las herramientas aplican la fórmula, no eligen esos
valores. No sustituyen la revisión de un ingeniero responsable.

Escritura (`writes.py`):

| Herramienta | Qué hace |
|---|---|
| `robot_create_nodes` | Crea nudos por coordenadas; no duplica los coincidentes |
| `robot_create_bars` | Crea barras entre nudos existentes, con sección opcional |
| `robot_define_concrete_section` | Crea o redefine una sección rectangular de hormigón con inercia agrietada |
| `robot_assign_section` | Asigna una sección a barras |
| `robot_define_support` | Crea o redefine un tipo de apoyo rígido |
| `robot_assign_support` | Asigna un apoyo a nudos |
| `robot_create_load_case` | Crea un caso simple estático |
| `robot_add_loads` | Añade peso propio, fuerzas en nudos, cargas repartidas en barras o cargas superficiales en paneles |
| `robot_create_combinations` | Crea combinaciones manuales con sus factores |
| `robot_create_nec_combinations` | Genera y crea las combinaciones LRFD de la NEC-SE-CG §3.4.3 (y D+L de servicio) a partir de los casos D, L, Lr, W, Ex y Ey; `seismic_scale_x/y` aplica la escala del espectro cuando va en las combinaciones (p. ej. I·g/R) |
| `robot_create_panel` | Crea un panel plano por su contorno, con espesor y modelo de cálculo existentes |
| `robot_delete_elements` | Borra barras, nudos, paneles o casos y combinaciones |
| `robot_delete_loads` | Borra cargas de un caso por su índice en `robot_list_loads` |
| `robot_run_calculation` | Lanza el cálculo y espera |
| `robot_save_project` | Guarda o «guarda como» |

Reglas de toda escritura:

1. `dry_run=true` por defecto: valida y devuelve el plan sin tocar el modelo. Se aplica con
   `dry_run=false`.
2. Si el modelo tiene resultados vigentes y el cambio los invalida, hace falta además
   `discard_results=true`. Las combinaciones no invalidan resultados; todo lo demás sí.
3. Se valida antes de escribir y se relee lo escrito. Si algo no cuadra, la herramienta falla.
4. No hay deshacer: guarda antes de cambios grandes. El borrado se niega a dejar el modelo
   incoherente: no borra nudos que usan barras, casos que usa una combinación que se queda, ni
   las cargas que Robot genera al repartir paneles.

No incluye: casos modales o espectrales (el espectro se define en Robot), espesores ni
modelos de cálculo de panel nuevos, ni secciones que no sean rectangulares de hormigón. Eso se
hace en Robot.

Unidades: m, cm, cm², cm⁴, MPa, kN/m³; resultados y cargas en kN, kN/m, kN·m, mm, rad y t.
Las listas largas están paginadas (`limit`, `offset`).

## Variables de entorno

| Variable | Efecto |
|---|---|
| `ROBOT_INTEROP_DLL` | Ruta de `Interop.RobotOM.dll` si no está en la instalación estándar |
| `ROBOT_MCP_AUTOSTART=1` | El servidor arranca Robot si no está abierto (por defecto no lo hace) |
| `ROBOT_MCP_READ_ONLY=1` | No registra las herramientas de escritura, cálculo ni guardado (quedan 26) |
| `ROBOT_MCP_TIMEOUT` | Segundos de espera por llamada (120) |
| `ROBOT_MCP_STARTUP_GRACE` | Segundos que debe llevar abierto Robot antes de conectar (40) |

## Pruebas

```bash
.venv\Scripts\python.exe tests\smoke_test.py
```

Con Robot cerrado comprueba el arranque y los errores; con un modelo abierto muestra datos reales
(solo lectura).

```bash
.venv\Scripts\python.exe tests\state_test.py
```

Prueba de estados, con Robot **cerrado**: abre su propio Robot, crea un pórtico de 2 pisos, lo
calcula y contrasta reacciones con el peso calculado a mano, lo desactualiza, cierra y reabre
Robot con el servidor en marcha, y al final lo cierra sin guardar (31 comprobaciones, ~3 min).
No requiere tocar nada: verifica que Robot no haga ninguna pregunta. Se niega a empezar si ya hay
un Robot abierto.

```bash
.venv\Scripts\python.exe tests\write_test.py
```

Prueba de escritura, también con Robot **cerrado**: construye el mismo pórtico solo con las
herramientas MCP, lo carga, combina y calcula, contrasta con la cuenta a mano y comprueba los
rechazos (duplicados, números existentes, resultados vigentes) y que Robot no pregunte nada
(58 comprobaciones, ~2 min).

```bash
.venv\Scripts\python.exe tests\checks_test.py
```

Prueba de las verificaciones NEC, con Robot **cerrado**: añade al pórtico dos casos laterales
de valor conocido (uno asimétrico, que provoca torsión) y compara cada verificación con la
misma cuenta hecha a partir de las herramientas de lectura (26 comprobaciones, ~1 min).

`tests\make_fixture.py` crea ese pórtico en el Robot abierto, pero **reemplaza el proyecto
activo**: no lo ejecutes con un modelo real sin guardar.

```bash
.venv\Scripts\python.exe tests\offline_test.py
```

Prueba sin Robot del marcador de modo no interactivo (incluida la escritura simultánea desde
otro chat), del hilo COM ocupado tras un plazo vencido, de la validación estricta de casos, del
filtro de cargas generadas, del generador de combinaciones NEC y de las fórmulas ACI contra
cálculos a mano (6 pruebas, segundos).

```bash
.venv\Scripts\python.exe tests\panel_test.py
```

Prueba de paneles, con Robot **cerrado**: convierte la losa del pórtico en panel de reparto,
le pone 2 kN/m², calcula y comprueba el modelo de cálculo del panel, la lectura de la carga
superficial, las 4 cargas trapezoidales que Robot genera en las vigas (suman los 40 kN de la
losa) y la reacción total de 400 kN (8 comprobaciones, ~1 min).

```bash
.venv\Scripts\python.exe tests\design_test.py
```

Prueba de las herramientas añadidas el 2026-11-12, con Robot **cerrado**, sobre el pórtico con
su losa de elementos finitos. Crea las combinaciones NEC y las relee. Comprueba:

- La flecha de D+L es la suma de las de D y L.
- Los momentos de la losa salen con ambos signos.
- El As de la viga coincide con el cálculo a mano a partir de `robot_get_bar_forces`.
- La columna queda con la cuantía mínima.
- Un panel nuevo con 3 kN/m² eleva la reacción a 420 kN.
- Cada borrado y cada rechazo funcionan (33 comprobaciones, ~2 min).

## Evaluación

El 2026-11-10 un subagente (Sonnet) resolvió a ciegas 10 preguntas de solo lectura sobre una copia
de un edificio real: 10 de 10 con 20 llamadas. De sus críticas salieron el filtro
`structural_type` y el contexto de barra en los extremos. Ese juego de preguntas y el guion de
la skill `mcp-builder` se retiraron el 2026-11-12 porque el modelo cambió y sus respuestas ya no
valían; si se quiere repetir, hay que generar preguntas nuevas sobre el estado actual del modelo.

## Cosas que no son obvias

- **Selecciones e idioma.** La sintaxis de selección de Robot depende del idioma: `1to4` en inglés
  es `1A4` en español, y en español `1to4` selecciona *todo*. El servidor acepta `1to10`, `1A10`,
  `1to20By2` y los traduce (`Session._from_text`). No llames a `FromText` directamente.
- **Conexión.** `RobotApplicationClass()` tumba el proceso con pythonnet 3. Se instancia por
  ProgID (`create_application`).
- **Un solo hilo.** Toda llamada COM pasa por `on_robot`; el SDK ejecuta las herramientas en un pool.
- **stdout es del protocolo.** Los logs van a stderr; no uses `print` en el servidor.
- **Inercia agrietada: la inercia que reporta la API no dice si la reducción se aplica.**
  Hay dos mediciones que no encajan entre sí y el mecanismo interno de Robot no está aclarado:
  - *Secciones creadas por API* (experimento del 2026-11-11, ya retirado: voladizos contrastados con
    PL³/3EI, en proyectos FRAME_3D y BUILDING, barras tipadas o no): el análisis sigue el valor
    de `GetValue(I_BSDV_IY)`. Coeficiente 0.5 con ese valor reducido da el doble de flecha; el
    mismo coeficiente con ese valor forzado a la inercia bruta da la flecha sin reducir.
  - *Edificio real definido en la interfaz* (una sección de viga con coeficiente 0.5 y
    `GetValue` igual a la inercia bruta): el análisis SÍ usa la inercia reducida. Comprobado tres
    veces sobre una copia: recalcular tal cual y redefinirla con `robot_define_concrete_section`
    y 0.5 dan exactamente el mismo T1; quitarle el coeficiente lo baja un 17 %.
  Consecuencias: `i*_stored_over_gross` es solo informativo y no debe usarse para afirmar que
  falta la reducción; `robot_define_concrete_section` deja la sección en un estado que analiza
  con la reducción aplicada una sola vez en ambos casos; y la única verificación fiable es
  comparar períodos o flechas con y sin coeficiente, en una copia.
- **Etiquetas.** `Labels.GetMany` no devuelve todas (omite espesores ortótropos y su material).
  Usa `_labels()`, que une `GetMany` con `GetAvailableNames` y filtra con `Exist` / `IsUsed`.
- **Pisos.** `CreateByStorey` es fiable para barras; para nudos devuelve números sin sentido.
- **Varias instancias de Robot.** COM se engancha a una sola; trabaja con una abierta.
- **Resultados: unidades y signos.** La API devuelve N, N·m, m, rad y kg. `FX > 0` en barras es
  compresión. En casos espectrales el signo no es físico.
- **Resultados: velocidad.** Cada lectura (`Forces.Value`, `Reactions.Value`…) cuesta unos 10 ms
  por COM, así que cada llamada está limitada a 600 lecturas (`MAX_READS`). Para envolventes se usa
  `Results.Extremes`, que busca dentro de Robot (~50 ms por componente) y coincide con la lectura
  valor a valor.
- **Derivas.** `robot_get_storey_results` devuelve el `dr` de la tabla de pisos de Robot, sin
  amplificar; los factores de norma (0.75·R en NEC) y el de reducción del espectro los aplica quien
  consulta.
- **Resultados desactualizados.** Con estado `OUT_OF_DATE` la API no entrega resultados
  (`Results.Available` es falso); las herramientas lo dicen y piden recalcular.
- **Arranque de Robot.** Si se pide `Robot.Application` mientras Robot arranca (medido a los 2, 5 y
  ~15 s; a los 45 s ya no), Windows abre otra instancia oculta que se queda viva. Por eso el
  servidor no conecta con un Robot abierto hace menos de `ROBOT_MCP_STARTUP_GRACE` segundos
  (`robot_health` responde `ready: false`). Como red de seguridad, si una conexión hace aparecer
  un proceso `robot.exe` nuevo y sin título, lo cierra y pide reintentar.
- **Diálogos al modificar.** Con `Interactive = 1` (lo normal cuando el usuario trabaja en
  Robot), modificar por API un modelo calculado abre el diálogo «Los resultados de cálculos
  pueden desactualizarse. ¿Desea continuar?» y la llamada COM queda bloqueada hasta que alguien
  responde. Con `Interactive = 0` no pregunta y devuelve al instante. Por eso toda escritura y
  el cálculo pasan por `non_interactive()`, que lo restaura al salir y deja un archivo marcador
  en la carpeta temporal para repararlo si el servidor muere a mitad. Cerrar con
  `Quit(I_QO_DISCARD_CHANGES)` no pregunta. `tests\dialog_watch.py` registra estos diálogos.
- **La API de Robot no valida.** Medido: `Nodes.Create` con un número existente **cambia las
  coordenadas** de ese nudo; `Bars.Create` admite barras duplicadas e ignora en silencio las que
  usan nudos inexistentes; `CaseFactors.New` acepta casos que no existen. `Bars.FindWithNodes` y
  `Nodes.GetConnectedBars` devuelven vacío, así que los duplicados se detectan recorriendo las
  barras. Toda validación está en `writes.py`, antes de escribir.
- **Qué invalida resultados.** Nudos, barras, secciones o apoyos en uso, casos y cargas dejan el
  estado en `OUT_OF_DATE`. Las combinaciones y las etiquetas sin usar no. Tras calcular, Robot
  congela los resultados (`ResultsFreeze`), pero una escritura no interactiva los descongela sola.
- **Masa de piso.** `Results.Storeys.Values(piso, caso).Mass` depende del caso cuando el modelo
  no tiene masas dinámicas: en el pórtico de prueba da la masa del peso propio para ese caso y
  cero para una combinación. Con análisis modal es la misma para todos los casos. Por eso
  `robot_check_irregularities` lee la masa del caso modal, o del que se indique en `mass_case`, y
  `robot_get_storey_results` también la lee del caso modal si existe.
- **Torsión en casos espectrales.** La deriva de cada borde se aproxima como diferencia de
  desplazamientos ya combinados (CQC), no como combinación de derivas modales; la herramienta lo
  marca con `approximate`. Para una comprobación exacta hace falta un caso estático.
- **P-Δ.** Δ y V salen del mismo caso sísmico, así que el factor de escala no altera Qi.
- **Cargas generadas por Robot.** Con paneles de reparto (modelo de cálculo sin elementos
  finitos, `I_LT_PANEL_CALC_MODEL`), Robot crea al calcular un registro por viga con la carga del
  panel ya repartida (`BAR_UNIFORM` o `BAR_TRAPEZOIDALE`) y lo marca `IsAutoGenerated`
  (`IRobotLoadRecordCommon`). En un edificio real son casi todos los registros de cada caso. No salen en la
  tabla de cargas de Robot, y sumarlos a la carga del panel la cuenta dos veces;
  `robot_list_loads` y `robot_list_load_cases` los separan. Aparecen un momento después de
  `CalculateEx`: leídos en la misma sesión COM justo al terminar, aún no estaban.
- **Cargas trapezoidales.** `GetValue` devuelve 0 en `PX1..PZ2`; la forma se lee por puntos con
  `IRobotLoadRecordBarTrapezoidal.PointCount` y `GetPoint(i, px, py, pz, x)`.
- **Cargas de casos espectrales.** El único registro de un caso sísmico espectral es su espectro
  (tipo 48, sin nombre en la API, con el nombre del espectro en `Description`); pedirle
  `ObjectType` u `Objects` da `E_FAIL`. `robot_list_loads` lo lista con `objects: null`.
- **Paneles sin material.** Un espesor sin material deja pasar la creación del panel, pero una
  losa de elementos finitos así no supera la verificación del cálculo (`FAILED_VERIFICATION`).
  Los paneles de solo reparto sí calculan. `robot_create_panel` avisa en el ensayo.
- **Extremos de paneles.** `IRobotFeExtremeValue.Panel` devuelve 0; el nudo y el elemento sí
  vienen. Por eso `robot_get_panel_forces` no dice el panel.
- **Barras sin tipo estructural.** Las barras creadas por la API no son `BEAM` ni `COLUMN`;
  `robot_check_rc_members` las clasifica por geometría y los filtros `structural_type` no las
  encuentran. Los modelos que vienen de Revit sí traen el tipo.
- **Mensajes de cálculo.** Si el cálculo deja avisos, Robot abre la ventana «Mensajes de
  cálculo». No bloquea, pero queda abierta.
- **Sin verificar:** área de paneles (`CalcArea` devolvió 0 en la prueba, por eso no se expone);
  la red de seguridad del arranque y la reparación del modo no interactivo tras un fallo, que no
  llegaron a dispararse; un cálculo que falle o tarde mucho; y la escritura sobre un modelo
  grande (crear barras recorre todas las existentes, ~4 ms por barra). La escritura solo se ha
  probado sobre el pórtico de prueba, nunca sobre un modelo real.

## Siguientes fases

1. Resultados que faltan: tensiones; flecha diferida (ACI 24.2.4).
2. Escritura que falta: espectro y casos sísmicos, espesores y modelos de cálculo de panel.
3. Diseño: cortante por capacidad y nudo columna fuerte-viga débil (ACI 18), secciones
   prefabricadas (dobles T).
