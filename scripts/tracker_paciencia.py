#!/usr/bin/env python3
"""
tracker_paciencia.py

- Lee la salida estándar de ARGoS canalizada por run_sim.py.
- Filtra específicamente los eventos de "[PACIENCIA]" y los "Timer=XXX".
- Dibuja una barra horizontal verde que se va agotando en tiempo real.
- No requiere dependencias explícitas de Lua para funcionar (lee logs puros).
"""

import sys
import re
import time
import threading
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =============================================================================
# ---------- CONFIGURACIÓN DE VARIABLES ----------
# =============================================================================
PACIENCIA_MAX_TICKS     = 600       # Valor máximo e inicial de la paciencia en Ticks
DECREMENTO_POR_TICK     = 1         # Cuánto baja la paciencia por cada evento 'Timer'
TICKS_POR_SEGUNDO       = 10        # Factor de conversión (ej: 60,0 seg -> 600 ticks)

LIVE_FPS                = 10        # Cuadros por segundo para actualizar la gráfica
TITULO_GRAFICA          = "Estado de Paciencia (Apprentice)"

END_SIMULATION_PATTERN  = "End_simulation"
AUTO_CLOSE_WINDOWS      = False     # Cerrar al terminar (False = Se queda abierta hasta que cierres)

# =============================================================================
# ---------- ESTADO COMPARTIDO ----------
# =============================================================================
estado = {
    'paciencia': PACIENCIA_MAX_TICKS,
    'timer_actual': 0,
    'terminado': False
}
lock = threading.Lock()

# =============================================================================
# ---------- LECTURA DE STDIN (HILO EN SEGUNDO PLANO) ----------
# =============================================================================
def stdin_reader(stop_event):
    """
    Lee la entrada estándar línea por línea.
    """
    global estado
    
    # Expresión regular para parsear "Actual: XX,X" 
    # Soporta enteros o decimales con coma (ej. 55,4 o 60)
    regex_actual = re.compile(r'Actual:\s*(\d+)(?:,(\d+))?', re.IGNORECASE)
    regex_timer = re.compile(r'timer\s*=\s*(\d+)', re.IGNORECASE)

    try:
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                break
            
            # 1. Condición de Finalización
            if END_SIMULATION_PATTERN in line:
                with lock:
                    estado['terminado'] = True
                stop_event.set()
                break
            
            # Si el experimento terminó, ya no procesamos actualizaciones
            with lock:
                if estado['terminado']:
                    continue
            
            # 2. Descuento constante por cada Tick
            match_timer = regex_timer.search(line)
            if match_timer:
                with lock:
                    estado['timer_actual'] = int(match_timer.group(1))
                    estado['paciencia'] = max(0, estado['paciencia'] - DECREMENTO_POR_TICK)
                continue

            # 3. Restauraciones y Agotamiento de Paciencia
            # (Se ignora el [t=xxx] automáticamente ya que solo buscamos [PACIENCIA])
            if '[PACIENCIA]' in line:
                with lock:
                    if 'Restaurada completamente' in line:
                        estado['paciencia'] = PACIENCIA_MAX_TICKS
                        
                    elif 'Restauración parcial' in line:
                        match_actual = regex_actual.search(line)
                        if match_actual:
                            enteros = match_actual.group(1)
                            decimales = match_actual.group(2) if match_actual.group(2) else '0'
                            valor_segundos = float(f"{enteros}.{decimales}")
                            # Convertimos los segundos parseados a Ticks
                            estado['paciencia'] = int(valor_segundos * TICKS_POR_SEGUNDO)
                            
                    elif 'AGOTADA' in line:
                        estado['paciencia'] = 0

    except Exception as e:
        print(f"[Tracker Paciencia] Error leyendo stdin: {e}")
    finally:
        stop_event.set()


# =============================================================================
# ---------- LÓGICA DE INTERFAZ GRÁFICA (MATPLOTLIB) ----------
# =============================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--live-fps', type=int, default=LIVE_FPS)
    args = parser.parse_args()

    stop_event = threading.Event()
    
    # Iniciar el hilo de lectura de la terminal
    reader_thread = threading.Thread(target=stdin_reader, args=(stop_event,), daemon=True)
    reader_thread.start()

    # Configuración de la Ventana
    fig, ax = plt.subplots(figsize=(7, 1.8))
    fig.canvas.manager.set_window_title("Tracker - Paciencia")

    def update(frame):
        # Evitar actualizaciones si se cerró la ventana
        if not plt.fignum_exists(fig.number):
            return

        with lock:
            pac = estado['paciencia']
            timer_val = estado['timer_actual']
            terminado = estado['terminado']

        ax.clear()
        
        # Límites visuales fijos para que la barra se mueva dentro de este espacio
        ax.set_xlim(0, PACIENCIA_MAX_TICKS)
        ax.set_ylim(-0.5, 0.5)
        
        # Ocultar ejes Y y ajustar apariencia de X
        ax.set_yticks([])
        ax.set_xlabel('Paciencia (Ticks)')
        
        # Color dinámico de la barra
        # Verde si > 300, Naranja si > 100, Rojo si se está agotando
        color_barra = 'green'
        if pac <= (PACIENCIA_MAX_TICKS * 0.25):   # Menos del 25%
            color_barra = 'red'
        elif pac <= (PACIENCIA_MAX_TICKS * 0.5):  # Menos del 50%
            color_barra = 'orange'

        # Dibujar la barra horizontal
        ax.barh(0, pac, height=0.6, color=color_barra, edgecolor='black', alpha=0.8)
        
        # Texto dentro/junto a la barra
        texto_estado = f"{pac} / {PACIENCIA_MAX_TICKS}"
        if pac == 0:
            texto_estado = "AGOTADA"

        # Colocar texto en el centro del eje para que siempre sea visible
        ax.text(PACIENCIA_MAX_TICKS / 2, 0, texto_estado, 
                ha='center', va='center', color='black', 
                fontweight='bold', fontsize=12,
                bbox=dict(facecolor='white', alpha=0.5, edgecolor='none', pad=2))

        # Actualizar título
        estado_txt = "(FINALIZADO)" if terminado else "En curso..."
        ax.set_title(f"{TITULO_GRAFICA} | Timer: {timer_val} | {estado_txt}")
        
        return []

    # Configurar la animación
    interval_ms = int(1000 / max(1, args.live_fps))
    ani = animation.FuncAnimation(fig, update, interval=interval_ms, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
        
        # Mantener el script vivo mientras la ventana exista
        while not stop_event.is_set() and plt.fignum_exists(fig.number):
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        stop_event.set()

    # Manejo del cierre
    if plt.fignum_exists(fig.number):
        if AUTO_CLOSE_WINDOWS:
            plt.close(fig)
        else:
            print("[Tracker Paciencia] Simulación terminada. Cierra la ventana para salir.")
            ani.event_source.stop()
            plt.show()

if __name__ == '__main__':
    main()