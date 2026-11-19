# Cómo contribuir

Gracias por querer ayudar. Este servidor conecta Claude con Autodesk Robot Structural Analysis
por COM, así que casi todo cambio necesita Robot instalado (Windows) para probarlo.

*English:* Issues and pull requests in English are welcome too. You need Windows and a licensed
Robot Structural Analysis to run most tests; `tests/offline_test.py` runs without Robot.

## Preparar el entorno

```bash
git clone https://github.com/brdz4/robot-mcp.git
cd robot-mcp
python -m venv .venv
.venv\Scripts\python.exe -m pip install -e .
.venv\Scripts\python.exe tests\offline_test.py
```

Las demás pruebas (`tests\*_test.py`) abren su propio Robot, crean un pórtico de prueba y lo
cierran sin guardar: **cierra Robot antes de ejecutarlas** y no las lances con un modelo tuyo
abierto. El README explica cada una.

## Reglas del código

- **Nombres de la API de Robot verificados**, no de memoria: compruébalos por reflexión sobre
  `Interop.RobotOM.dll`. Lee antes la sección «Cosas que no son obvias» del README.
- **Un solo hilo COM**: todo acceso a Robot pasa por `on_robot` en `robot_com.py`.
- **Escritura segura**: toda herramienta que modifica el modelo tiene `dry_run=true` por
  defecto y, si invalida resultados vigentes, exige `discard_results=true`. Después de aplicar,
  relee de Robot lo que escribió y compáralo.
- **Lecturas acotadas**: respeta el presupuesto de lecturas (`MAX_READS`) y pagina las listas.
- **Normativa**: cita la sección (NEC-SE-DS §6.3.9, ACI 318-19 §22.5…) y añade una comprobación
  contra una cuenta a mano en las pruebas.
- **Sin datos de proyectos reales** en el código, las pruebas o los issues: usa modelos de
  prueba o valores inventados.

## Ideas para empezar

- Tensiones en barras y flecha diferida (ACI 24.2.4).
- Definir espectros y casos sísmicos, espesores y modelos de cálculo de panel.
- Diseño por capacidad: cortante por momentos probables y columna fuerte-viga débil (ACI 18).
- Secciones prefabricadas (dobles T) en el cálculo de cuantías.
- Otras normas (ASCE 7, Eurocódigo, NSR-10…) como verificaciones nuevas.
- Robot en inglés u otros idiomas: probar y corregir la sintaxis de selecciones.

Abre un issue antes de un cambio grande para acordar el enfoque.

## Pull requests

1. Una rama por cambio, con su prueba.
2. `tests\offline_test.py` debe pasar; si tocas lógica que habla con Robot, ejecuta también la
   prueba correspondiente e indica el resultado y la versión de Robot en el PR.
3. Actualiza la tabla de herramientas del README si añades o cambias una.

Al contribuir aceptas que tu aporte se publique con la licencia Apache-2.0 del proyecto.
