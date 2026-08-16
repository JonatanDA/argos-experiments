#!/usr/bin/env python3
"""
tracker_varianza.py

- controllers: Tr6 -> Qupa

- Maneja exclusivamente la recepción y gráfica de los datos de varianza de los maestros.
- Crea ventanas dinámicamente sin ventanas residuales.
- Cierre automático o manual de ventanas al detectar el patrón de fin.

NOTA: En el controlador Lua, se debe activar la opción que se necesite, solo una de las dos.
function Print_tracker_varianza(k, var, separacion, avance)
    --print( "Maestro"..k.." & " .. var)                -- Datos para el anális de varianza en los datos obtenidos por el observador. Script -> sun_sim.py
   -- print( "Otsu & " .. separacion.." , "..avance)    -- Datos para el anális del método de otsu a distintos valores de velocidad del demostrador: speed_dem  -> Script: master_evluator.py
end


"""

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# (La visualización estándar de Matplotlib se activa dinámicamente)

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
# --- Identificadores y Filtros ---
FILTRO_MAESTROS         = "None"            # IDs de los maestros a escuchar (ej. "None" o "1,2")

# --- Control de Finalización ---
DEFAULT_END_PATTERN     = "End_simulation"  # Frase en consola que finaliza la ejecución
AUTO_CLOSE_TIME         = None              # Segundos sin datos para forzar el cierre
AUTO_CLOSE_WINDOWS      = False             # True: Cierra automático al terminar / False: Cierre manual

# =============================================================================
# ---------- CONFIGURACIÓN DE PROCESAMIENTO Y LÓGICA ----------
# =============================================================================
TIEMPO_ESPERA_MAESTRO   = 1.0               # Tiempo mínimo entre actualizaciones gráficas por maestro

import sys
import argparse
import threading
import queue
import re
import time
from collections import defaultdict
import matplotlib.pyplot as plt

NUMBER_RE = re.compile(r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?')

def parse_line(line):
    if '&' not in line: return None
    parts = line.split('&', 1)
    name_raw = parts[0].strip()
    nums = NUMBER_RE.findall(parts[1])
    name_clean = re.sub(r'^\[.*?\]\s*', '', name_raw)
    
    if name_clean.lower().startswith("maestro"):
        if len(nums) >= 1:
            try:
                var = float(nums[0].replace(',', '.'))
                name = name_clean.lower().replace(" ", "")
                return 'maestro', name, var, None
            except ValueError:
                pass
    return None

def parse_maestros_arg(s):
    if not s or s.lower() == "none": return None
    parts = [p.strip() for p in s.split(',') if p.strip()]
    names = set()
    for p in parts:
        if p.isdigit(): names.add(f"maestro{int(p)}")
        else: names.add(p.lower().replace(" ", ""))
    return names

def stdin_reader(q, stop_event, end_pattern, last_time_holder):
    try:
        while True:
            raw = sys.stdin.readline()
            if raw == '': break
            line = raw.strip()
            if not line: continue
            if end_pattern and end_pattern in line:
                stop_event.set()
                break
            
            parsed = parse_line(line)
            if parsed:
                category, name, val1, val2 = parsed
                ts = time.time()
                q.put((category, name, val1, val2, ts))
                last_time_holder[0] = ts
    except Exception:
        pass
    finally:
        stop_event.set()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--end-pattern', type=str, default=DEFAULT_END_PATTERN)
    parser.add_argument('--inactivity-timeout', type=float, default=AUTO_CLOSE_TIME)
    args = parser.parse_args()

    maestros_filter = parse_maestros_arg(FILTRO_MAESTROS)

    q = queue.Queue()
    stop_event = threading.Event()
    last_time_holder = [time.time()]

    reader_thread = threading.Thread(target=stdin_reader, args=(q, stop_event, args.end_pattern, last_time_holder), daemon=True)
    reader_thread.start()

    data_maestros = defaultdict(lambda: {'vars': []})
    maestro_figs = {}             
    last_seen_maestro = {}        
    maestro_needs_plot = {}       

    # Activar modo interactivo para controlar las ventanas manualmente
    plt.ion() 

    try:
        while not stop_event.is_set():
            current_time = time.time()
            
            # 1. Procesar todos los datos nuevos
            while True:
                try:
                    category, name, val1, val2, ts = q.get_nowait()
                except queue.Empty:
                    break
                
                if maestros_filter and name not in maestros_filter:
                    continue
                
                # Crear ventana si el maestro es nuevo
                if name not in data_maestros:
                    fig_m, ax_m = plt.subplots(figsize=(5, 3))
                    fig_m.canvas.manager.set_window_title(f'Varianza - {name}')
                    maestro_figs[name] = (fig_m, ax_m)
                    plt.show(block=False)
                
                data_maestros[name]['vars'].append(val1)
                last_seen_maestro[name] = current_time
                maestro_needs_plot[name] = True

            # 2. Actualizar gráficos si pasó el tiempo de silencio
            for m_name, (fig_m, ax_m) in maestro_figs.items():
                if maestro_needs_plot.get(m_name, False):
                    tiempo_silencio = current_time - last_seen_maestro.get(m_name, current_time)
                    
                    if tiempo_silencio >= TIEMPO_ESPERA_MAESTRO:
                        ax_m.clear()
                        vars_data = data_maestros[m_name]['vars']
                        x_data = list(range(1, len(vars_data) + 1))
                        
                        ax_m.set_title(f'Varianza: {m_name}')
                        ax_m.set_xlabel('Número de dato')
                        ax_m.set_ylabel('Varianza')
                        ax_m.grid(True)
                        
                        if x_data and vars_data:
                            ax_m.plot(x_data, vars_data, lw=1.5, color='orange')
                            ax_m.scatter([x_data[-1]], [vars_data[-1]], s=25, color='red')
                        
                        fig_m.canvas.draw_idle()
                        maestro_needs_plot[m_name] = False
            
            # 3. Timeout general
            if args.inactivity_timeout is not None:
                if (time.time() - last_time_holder[0]) > args.inactivity_timeout:
                    stop_event.set()
                    break

            # Esto mantiene viva la interfaz gráfica
            plt.pause(0.1) 

    except KeyboardInterrupt:
        stop_event.set()
    
    # --- LIMPIEZA Y GRÁFICA FINAL ---
    # Forzar el dibujado de los maestros que se quedaron con datos pendientes al cerrarse ARGoS
    for m_name, (fig_m, ax_m) in maestro_figs.items():
        if maestro_needs_plot.get(m_name, False):
            ax_m.clear()
            vars_data = data_maestros[m_name]['vars']
            x_data = list(range(1, len(vars_data) + 1))
            ax_m.set_title(f'Varianza: {m_name}')
            ax_m.set_xlabel('Número de dato')
            ax_m.set_ylabel('Varianza')
            ax_m.grid(True)
            if x_data and vars_data:
                ax_m.plot(x_data, vars_data, lw=1.5, color='orange')
                ax_m.scatter([x_data[-1]], [vars_data[-1]], s=25, color='red')
            fig_m.canvas.draw_idle()

    # --- LÓGICA DE CIERRE SEGÚN VARIABLE AUTO_CLOSE_WINDOWS ---
    if maestro_figs:
        if AUTO_CLOSE_WINDOWS:
            print("Simulación terminada. Cerrando ventanas de varianza automáticamente...")
            plt.close('all')
        else:
            print("Simulación terminada. Cierra las ventanas de varianza manualmente para salir.")
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()