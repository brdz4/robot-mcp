"""Vigila los diálogos que Robot abre durante una prueba: los registra y, si nadie responde, los acepta.

Una llamada COM que provoca un diálogo modal en Robot queda bloqueada hasta que alguien lo
cierra. En pruebas desatendidas eso es un cuelgue, así que este vigilante anota título y texto
de cada diálogo y, pasados `answer_after` segundos, pulsa su botón por defecto (Intro).
Solo se usa en pruebas, contra el Robot que la propia prueba abrió.
"""

from __future__ import annotations

import ctypes
import threading
import time
from ctypes import wintypes

user32 = ctypes.windll.user32
ENUM_PROC = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
WM_COMMAND, WM_CLOSE, IDYES, IDOK = 0x0111, 0x0010, 6, 1


def _text(hwnd: int) -> str:
    buffer = ctypes.create_unicode_buffer(1024)
    user32.GetWindowTextW(hwnd, buffer, 1024)
    return buffer.value


def _class(hwnd: int) -> str:
    buffer = ctypes.create_unicode_buffer(256)
    user32.GetClassNameW(hwnd, buffer, 256)
    return buffer.value


def robot_dialogs(pids: set[int]) -> list[tuple[int, str, str, list[str]]]:
    """Diálogos visibles (clase #32770) de esos procesos: (hwnd, título, texto, botones)."""
    found: list[tuple[int, str, str, list[str]]] = []

    def on_window(hwnd: int, _: int) -> bool:
        pid = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if pid.value in pids and user32.IsWindowVisible(hwnd) and _class(hwnd) == "#32770":
            texts: list[str] = []
            buttons: list[str] = []

            def on_child(child: int, __: int) -> bool:
                label = _text(child)
                if label:
                    (buttons if _class(child) == "Button" else texts).append(label)
                return True

            user32.EnumChildWindows(hwnd, ENUM_PROC(on_child), 0)
            if buttons:  # la pantalla de inicio de Robot también es #32770, pero sin botones
                found.append((hwnd, _text(hwnd), " | ".join(texts), buttons))
        return True

    user32.EnumWindows(ENUM_PROC(on_window), 0)
    return found


class DialogWatch:
    """with DialogWatch(pids_fn) as watch: ...; watch.seen -> [(segundo, título, texto, botones, respuesta)]"""

    def __init__(self, pids_fn, answer_after: float = 4.0) -> None:
        self._pids_fn = pids_fn
        self._answer_after = answer_after
        self._stop = threading.Event()
        self._first_seen: dict[int, float] = {}
        self.seen: list[tuple[float, str, str, list[str], str]] = []
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self) -> "DialogWatch":
        self._thread.start()
        return self

    def __exit__(self, *_: object) -> None:
        self._stop.set()
        self._thread.join(timeout=3)

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.time()
            for hwnd, title, text, buttons in robot_dialogs(self._pids_fn()):
                if hwnd not in self._first_seen:
                    self._first_seen[hwnd] = now
                    self.seen.append((round(now - self._t0, 1), title, text, buttons, "pendiente"))
                    print(f"   [diálogo] t={now - self._t0:.1f}s  {title!r}: {text[:300]!r}  botones={buttons}")
                elif now - self._first_seen[hwnd] > self._answer_after and "&Sí" in buttons:
                    # Solo se responde a preguntas Sí/No; las ventanas informativas no bloquean.
                    user32.PostMessageW(hwnd, WM_COMMAND, IDYES, 0)
                    self._first_seen[hwnd] = now + 3600  # no insistir
                    print(f"   [diálogo] sin respuesta en {self._answer_after:.0f}s: se pulsa Sí")
            time.sleep(0.4)
