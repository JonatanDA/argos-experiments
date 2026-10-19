#!/usr/bin/env python3
"""
tracker_trayectoria.py

- controllers compatibles: Ks1 (Corregir para las demás versiones que aún no se sincronizan con el timer global de la simulación en argos.)

DEPENDENCIAS LUA REQUERIDAS:
- Función: `Print_tracker_trayectory_data(k, x, y, logging)` actualizada con soporte 'trajectory' y 'Active_learning'.
- Variables: `displayed_data`, `PTTD`

CARACTERÍSTICAS PRINCIPALES:
1. AGRUPACIÓN EXPLÍCITA POR TICK: Busca `Timer=XXX` para reconstrucción perfecta del tiempo.
2. OVERLAYS DE ACTIVE LEARNING: Soporta visualización de Centroides y Fuerzas Virtuales por sobre los robots.
3. RENDERIZADO DUAL: Soporta renderizado de mapa de calor superpuesto o lado a lado.
"""

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================

# --- Ventana Principal y Gráficas ---
TITULO_GRAFICA          = "Datos observados por el Robot Aprendiz (Ra)" # Título de la gráfica
MOSTRAR_TIEMPO_TITULO   = True                                          # Muestra el tiempo en el título: True/False
UNIDAD_TIEMPO_TITULO    = "segundos"                                    # Unidad del tiempo: "segundos" o "ticks"

MODO                = 1         # Modo de visualización: 1 = trayectoria completa, 2 = ventana de tiempo
WINDOW              = 300       # Cantidad de ticks mostrados en modo 2

DEFAULT_XMIN        = -1.5      # Límite mínimo del eje X
DEFAULT_XMAX        = 1.5       # Límite máximo del eje X
DEFAULT_YMIN        = -1.5      # Límite mínimo del eje Y
DEFAULT_YMAX        = 1.5       # Límite máximo del eje Y

DEAD_TIME_LIMIT     = 1.0         # Tiempo de inactividad permitido en segundos de simulación
DELETE_ON_TIMEOUT   = True     # Elimina la trayectoria tras inactividad: True/False

# --- CONTROL DE VELOCIDADES Y TIEMPO (FPS) ---
SIM_TICKS_PER_SEC   = 4         # Ticks de simulación por segundo
LIVE_FPS            = 10        # Frecuencia de actualización de la gráfica en vivo
VIDEO_FRAMES        = 2         # Cuadros por segundo usados para generar el video
VIDEO_SPEED         = 8.0       # Multiplicador de velocidad (1.0 = normal, 2.0 = cámara rápida, 0.5 = cámara lenta)

# --- Etiquetas y Nombres ---
MODO_ETIQUETAS                  = 1     # Etiquetas: 0 = leyenda, 3 = fijas iniciales, otro = posición actual
PREFIX_ETIQUETAS                = "Rm"  # Prefijo para normalizar nombres de robots
NOMBRE_OBSERVADOR               = "Ra"  # Nombre mostrado para el robot observador
MAX_ETIQUETAS_FIJAS             = 7     # Máximo número de etiquetas fijas; None = sin límite
ELIMINAR_SUPERPUESTAS           = False # Evita etiquetas superpuestas: True/False
MAX_HISTORICAS_TRANSPARENTES    = 3     # Cantidad máxima de posiciones históricas etiquetadas

# Listas y reglas de nombres configurables
PALABRAS_IGNORADAS       = []          # Palabras al inicio del nombre que hacen que se ignore el robot
PREFIJOS_NORMALIZABLES   = ["master"]    # Prefijos cuyos nombres con números serán normalizados
TRAYECTORIAS_PERMANENTES = ["Patrón1", "Patrón2", "Patrón3", "Patrón4", "Patrón5", "Patrón6"] # Nombres cuya trayectoria NUNCA se borra ni se recorta
OBSERVADOR_ID            = "Apprentice"  # Identificador interno del robot observador
PRESERVAR_NOMBRES_LIBRES = True          # Conserva nombres sin número: True/False

# --- Representación Física y Trayectorias ---
DIAMETRO_ROBOT            = 0.15    # Diámetro físico del robot (ej. Qupa = 0.15 metros)
OBSERVADOR_USA_DIAMETRO   = True    # El observador dibuja su trayectoria y mapa de calor con el grosor real del robot
MAESTROS_USAN_DIAMETRO    = False   # Los maestros dibujan su trayectoria con el grosor real del robot
MODO_TRAYECTORIA_MAESTROS = 1       # 1 = Línea continua para maestros, 0 = Solo puntos sueltos (scatter)

# --- Representación del Observador (Cámara) 
MODO_VISUAL_OBSERVADOR  = 1         # Observador: 0 = oculto, 1 = punto actual, 2 = trayectoria
MODO_CIRCULO            = 2         # Círculo de visión: 0 = oculto, 1 = circulo visible , 2 = relleno semitransparente
ANGULO_APERTURA         = 72.1      # Ángulo de apertura usado para calcular el radio de visión (Usar 70,711)
ALTURA_CAMARA           = 0.25      # Altura usada para calcular el radio de visión

# --- Mapa de Calor (Heatmap) ---
MOSTRAR_MAPA_CALOR      = False      # Muestra el mapa de calor en vivo para el robot observador: True/False
RENDERIZAR_MAPA_CALOR   = True      # Muestra el mapa de calor en el video exportado: True/False
DISPOSICION_MAPA        = 2         # 1 = Superpuesto en trayectoria, 2 = Lado a lado (Izquierda Mapa, Derecha Trayectorias)
RESOLUCION_MAPA         = 400       # Resolución de la cuadrícula del mapa (ej. 100 = 100x100 celdas)
SENSIBILIDAD_MAPA       = 20.0      # 10 = Normal. < 10 = Más difícil llegar a rojo. > 10 = Más fácil llegar a rojo.
TRANSPARENCIA_MAPA      = 1.0       # 0.0 = Invisible, 1.0 = Estándar, 2.0 = Sólido marcado

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
# --- Rutas de Directorios ---
VIDEO_DIR       = "Resultados/Robots/Qupa/Ks3/CamposVectoriales/octagono/Trajectory"       # Carpeta donde se guarda el video
CSV_DIR         = "Resultados/Robots/Qupa/Ks3/CamposVectoriales/octagono/Trajectory/Data"  # Carpeta donde se guardan los CSV

# --- Video ---
SAVE_VIDEO      = False                                            # Guarda el video: True/False
DEFAULT_OUTFILE = "Ks3.mp4"                                     # Nombre del archivo de video de salida

# --- CSV ---
SAVE_CSV        = False                                            # Guarda los datos en CSV: True/False

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================

FILTRO              = False             # Filtra los robots mostrados: True/False
ROBOTS              = "7,10,11"         # Robots incluidos cuando FILTRO=True

DEFAULT_END_PATTERN = "End_simulation"  # Texto que indica el final de la simulación
AUTO_CLOSE_WINDOWS  = False             # Cierra automáticamente la gráfica al terminar: True/False
AUTO_CLOSE_TIME     = 3                 # Tiempo real de inactividad para detener el programa; None = desactivado                   

import os
import sys
import argparse
import threading
import queue
import re
import time
import math
import subprocess
import gc
from collections import defaultdict
import csv
import copy
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

# Configuración del Colormap para el mapa de calor (Transparente para 0, Azul a Rojo para el resto)
heat_cmap = copy.copy(plt.get_cmap('jet'))
heat_cmap.set_under('none')

# Variables globales para el manejo de offset automático
current_offset_x = 0.0
current_offset_y = 0.0
current_offset_theta = 0.0

RADIO_VISION = ALTURA_CAMARA * math.tan(math.radians(ANGULO_APERTURA))
NUM_PATTERN = r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?'
FULL_LINE_PATTERN = re.compile(rf'^(.+?)\s*&\s*({NUM_PATTERN})\s*,\s*({NUM_PATTERN})\s*$')

def parse_line(line):
    global current_offset_x, current_offset_y, current_offset_theta
    line_str = line.strip()
    if not line_str: return None
    
    # Procesar línea de calibración de offset dinámico
    if line_str.startswith("[trajectory];[offset];"):
        parts = line_str.split(';')
        if len(parts) >= 4:
            try:
                current_offset_x = float(parts[2].replace(',', '.'))
                current_offset_y = float(parts[3].replace(',', '.'))
            except ValueError: pass
        if len(parts) >= 5:
            try:
                current_offset_theta = float(parts[4].replace(',', '.'))
            except ValueError: pass
        return None
        
    if line_str.startswith("[active_learning]"):
        parts = line_str.split(';')
        if len(parts) >= 5:
            try:
                mode = int(parts[1])
                name = parts[2]
                x = float(parts[3].replace(',', '.'))
                y = float(parts[4].replace(',', '.'))
                extras = [float(p.replace(',', '.')) for p in parts[5:] if p]
                
                # --- CORRECCIÓN DE ORIENTACIÓN AQUÍ ---
                if current_offset_theta != 0.0:
                    cos_t = math.cos(current_offset_theta)
                    sin_t = math.sin(current_offset_theta)
                    
                    # 1. ROTAR VECTORES (fx,fy o cx,cy): 
                    # Se aplica a TODOS los modos (incluido el 4) y a TODOS los robots (incluido el Observador).
                    # Esto asegura que la flecha apunte hacia donde realmente mira el robot globalmente.
                    if len(extras) >= 2:
                        nx_ext = extras[0] * cos_t - extras[1] * sin_t
                        ny_ext = extras[0] * sin_t + extras[1] * cos_t
                        extras[0], extras[1] = nx_ext, ny_ext
                    
                    # 2. ROTAR POSICIÓN (x, y):
                    # Se aplica SOLO a los maestros. El Observador (Ra/Apprentice) 
                    # ya envía su (x, y) de forma global en Lua, por lo que no se debe rotar.
                    if name != OBSERVADOR_ID and name not in TRAYECTORIAS_PERMANENTES:
                        nx = x * cos_t - y * sin_t
                        ny = x * sin_t + y * cos_t
                        x, y = nx, ny
                        
                # 3. TRASLADAR POSICIÓN (x, y):
                # De nuevo, solo a los maestros. El Observador ya está en sus coordenadas globales reales.
                if name != OBSERVADOR_ID and name not in TRAYECTORIAS_PERMANENTES:
                    x += current_offset_x
                    y += current_offset_y
                    
                return 'al_data', name, x, y, {'mode': mode, 'extras': extras}
            except ValueError: pass
        return None
        
    if line_str.startswith("[trajectory]"):
        parts = line_str.split(';')
        if len(parts) >= 4:
            try:
                name = parts[1]
                name_clean = re.sub(r'^\[.*?\]\s*', '', name)
                x = float(parts[2].replace(',', '.'))
                y = float(parts[3].replace(',', '.'))
                
                if name_clean != OBSERVADOR_ID and name_clean not in TRAYECTORIAS_PERMANENTES:
                    if current_offset_theta != 0.0:
                        cos_t = math.cos(current_offset_theta)
                        sin_t = math.sin(current_offset_theta)
                        nx = x * cos_t - y * sin_t
                        ny = x * sin_t + y * cos_t
                        x, y = nx, ny
                        
                    x += current_offset_x
                    y += current_offset_y
                    
                return 'robot', name_clean, x, y, None
            except ValueError: pass
        return None

    match = FULL_LINE_PATTERN.match(line_str)
    if match:
        name_raw = match.group(1).strip()
        name_clean = re.sub(r'^\[.*?\]\s*', '', name_raw)
        
        name_lower = name_clean.lower()
        if PALABRAS_IGNORADAS and any(name_lower.startswith(p.lower()) for p in PALABRAS_IGNORADAS): 
            return None
            
        try:
            x = float(match.group(2).replace(',', '.'))
            y = float(match.group(3).replace(',', '.'))
            
            if name_clean != OBSERVADOR_ID and name_clean not in TRAYECTORIAS_PERMANENTES:
                if current_offset_theta != 0.0:
                    cos_t = math.cos(current_offset_theta)
                    sin_t = math.sin(current_offset_theta)
                    nx = x * cos_t - y * sin_t
                    ny = x * sin_t + y * cos_t
                    x, y = nx, ny
                    
                x += current_offset_x
                y += current_offset_y
                
            return 'robot', name_clean, x, y, None
        except ValueError:
            pass
            
    return None

def parse_robots_arg(s):
    if not s: return None
    parts = [p.strip() for p in s.split(',') if p.strip()]
    names = set()
    for p in parts:
        if p.isdigit(): names.add(f"Master{int(p)}") 
        else: names.add(p)
    return names

def get_formatted_title(base_title, sim_time, ticks_per_sec):
    if not MOSTRAR_TIEMPO_TITULO: return base_title
    if UNIDAD_TIEMPO_TITULO.lower() == "ticks":
        ticks = int(round(sim_time * ticks_per_sec))
        return f"{base_title} | Ticks: {ticks}"
    else:
        return f"{base_title} | Tiempo: {sim_time:.2f}s"

def stdin_reader(q, timeline, stop_event, end_pattern, last_time_holder):
    current_tick = []
    first_timer_seen = False
    tick_counter = 0

    try:
        while True:
            raw = sys.stdin.readline()
            if raw == '': break
            line = raw.strip()
            if not line: continue

            if end_pattern and end_pattern in line:
                if first_timer_seen: timeline.append(current_tick)
                stop_event.set()
                break

            if line.lower().startswith("timer="):
                if first_timer_seen:
                    timeline.append(current_tick)
                    current_tick = []
                first_timer_seen = True
                match_val = re.search(r'(\d+)', line)
                if match_val: tick_counter = int(match_val.group(1))
                else: tick_counter += 1
                continue
            
            match_timer = re.match(r'timer\s*=\s*(\d+)', line, re.IGNORECASE)
            if match_timer:
                if first_timer_seen:
                    timeline.append(current_tick)
                    current_tick = []
                first_timer_seen = True
                tick_counter = int(match_timer.group(1))
                continue

            parsed = parse_line(line)
            if parsed:
                category, name, val1, val2, extra_data = parsed
                real_ts = time.time()
                sim_pseudo_ts = tick_counter / float(SIM_TICKS_PER_SEC)
                
                q.put((category, name, val1, val2, sim_pseudo_ts, extra_data))

                if first_timer_seen: current_tick.append((category, name, val1, val2, sim_pseudo_ts, extra_data))
                else: timeline.append([(category, name, val1, val2, sim_pseudo_ts, extra_data)])

                last_time_holder[0] = real_ts

    except Exception: pass
    finally:
        if first_timer_seen and not stop_event.is_set(): timeline.append(current_tick)
        stop_event.set()

def get_color(i, cmap='tab20'):
    return plt.get_cmap(cmap)(i % plt.get_cmap(cmap).N)

def iso(ts):
    return datetime.fromtimestamp(ts).isoformat(sep=' ', timespec='milliseconds')

def get_display_name(name):
    if name == OBSERVADOR_ID and NOMBRE_OBSERVADOR != "": return NOMBRE_OBSERVADOR
    if PREFIJOS_NORMALIZABLES:
        match_norm = re.match(r'^([a-zA-Z]+)(\d+)$', name)
        if match_norm:
            prefix_found = match_norm.group(1).lower()
            num_found = match_norm.group(2)
            if any(prefix_found == p.lower() for p in PREFIJOS_NORMALIZABLES):
                if PREFIX_ETIQUETAS != "": return f"{PREFIX_ETIQUETAS}{num_found}"
                return name
    return name

def load_timeline_from_csv(csv_dir):
    events = []
    if not os.path.exists(csv_dir): return []

    for file in os.listdir(csv_dir):
        if file.endswith('.csv'):
            name = file.replace('.csv', '').replace('_', ' ')
            with open(os.path.join(csv_dir, file), 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    events.append(('robot', name, float(row['x']), float(row['y']), float(row['timestamp']), None))

    events.sort(key=lambda x: x[4])
    timeline, current_tick, current_ts = [], [], -1
    for ev in events:
        if ev[4] != current_ts:
            if current_tick: timeline.append(current_tick)
            current_tick = [ev]
            current_ts = ev[4]
        else:
            current_tick.append(ev)
    if current_tick: timeline.append(current_tick)
    return timeline

def add_heat(x, y, hmap, xmin, xmax, ymin, ymax, size, use_diameter, diameter):
    """Añade calor a la cuadrícula considerando el diámetro si está activado."""
    if x is None or y is None: return
    col = int((x - xmin) / (xmax - xmin) * size)
    row = int((y - ymin) / (ymax - ymin) * size)
    
    if not (0 <= col < size and 0 <= row < size): return
    
    if use_diameter:
        cell_size = (xmax - xmin) / size
        r_cells = max(1, int((diameter / 2.0) / cell_size))
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if dr**2 + dc**2 <= r_cells**2:
                    r_idx, c_idx = row + dr, col + dc
                    if 0 <= r_idx < size and 0 <= c_idx < size:
                        dist = math.sqrt(dr**2 + dc**2)
                        factor = 1.0 - (dist / (r_cells + 1))
                        hmap[r_idx, c_idx] += factor
    else:
        hmap[row, col] += 1.0
        for dr, dc, val in [(-1,0,0.5), (1,0,0.5), (0,-1,0.5), (0,1,0.5),
                            (-1,-1,0.25), (-1,1,0.25), (1,-1,0.25), (1,1,0.25)]:
            if 0 <= row+dr < size and 0 <= col+dc < size:
                hmap[row+dr, col+dc] += val

def get_rgba_buffer(fig):
    """Extrae el buffer RGBA de la figura de forma segura garantizando el backend Agg."""
    if not hasattr(fig, 'canvas') or not isinstance(fig.canvas, FigureCanvas):
        FigureCanvas(fig)
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba()).tobytes()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xmin', type=float, default=DEFAULT_XMIN)
    parser.add_argument('--xmax', type=float, default=DEFAULT_XMAX)
    parser.add_argument('--ymin', type=float, default=DEFAULT_YMIN)
    parser.add_argument('--ymax', type=float, default=DEFAULT_YMAX)
    parser.add_argument('--live-fps', type=int, default=LIVE_FPS)
    parser.add_argument('--video-fps', type=int, default=VIDEO_FRAMES)
    parser.add_argument('--video-speed', type=float, default=VIDEO_SPEED)
    parser.add_argument('--outfile', type=str, default=DEFAULT_OUTFILE)
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--mode', type=int, choices=[1,2], default=MODO)
    parser.add_argument('--window', type=int, default=WINDOW)
    parser.add_argument('--robots', type=str, default=None)
    parser.add_argument('--end-pattern', type=str, default=DEFAULT_END_PATTERN)
    parser.add_argument('--inactivity-timeout', type=float, default=AUTO_CLOSE_TIME)
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--no-csv', action='store_true')
    parser.add_argument('--from-csv', action='store_true')
    args = parser.parse_args()

    inactivity_timeout = args.inactivity_timeout if AUTO_CLOSE_WINDOWS else None

    robots_filter = parse_robots_arg(args.robots) if args.robots else (parse_robots_arg(ROBOTS) if FILTRO else None)
    use_filter_robots = True if robots_filter else False

    save_video = False if args.no_video else SAVE_VIDEO
    save_csv = False if args.no_csv else SAVE_CSV
    timeline = []

    if args.from_csv:
        print("\n[Tracker Trayectoria] [+] MODO OFFLINE ACTIVADO: Generando video desde CSVs...")
        timeline = load_timeline_from_csv(CSV_DIR)
        if not timeline:
            print(f"[Tracker Trayectoria] [!] No se encontraron datos CSV en: {CSV_DIR}")
            return
        save_csv = False 
        save_video = True
    else:
        q = queue.Queue()
        stop_event = threading.Event()
        last_time_holder = [time.time()]
        start_time = time.time()

        reader_thread = threading.Thread(target=stdin_reader, args=(q, timeline, stop_event, args.end_pattern, last_time_holder), daemon=True)
        reader_thread.start()

        data_robots = defaultdict(lambda: {'xs': [], 'ys': []})
        name_to_color, last_seen_map, first_seen_positions = {}, {}, {}
        current_al_data = {}  
        next_index = 0
        latest_sim_time = 0.0
        
        distancia_total_observador = 0.0
        last_obs_pos = None
        total_maestros_set = set()
        heatmap_live = np.zeros((RESOLUCION_MAPA, RESOLUCION_MAPA))

        # --- LÓGICA DE INTERFAZ GRÁFICA EN VIVO ---
        if MOSTRAR_MAPA_CALOR:
            if DISPOSICION_MAPA == 2:
                fig_main, axs = plt.subplots(1, 3, figsize=(15, 7), gridspec_kw={'width_ratios': [5, 5, 2]})
                ax_map = axs[0]
                ax_main = axs[1]
                ax_info = axs[2]
            else:
                fig_main, axs = plt.subplots(1, 2, figsize=(10, 7), gridspec_kw={'width_ratios': [7, 3]})
                ax_main = axs[0]
                ax_map = ax_main
                ax_info = axs[1]
        else:
            fig_main, ax_main = plt.subplots(figsize=(7, 7))
            ax_info = None
            ax_map = None
            
        fig_main.canvas.manager.set_window_title(TITULO_GRAFICA)

        def draw_labels_clamped(ax, data_dict, xmin, xmax, ymin, ymax, is_video=False, first_pos_dict=None):
            if not data_dict: return
            if MODO_ETIQUETAS == 0:
                ax.legend(loc='upper right', fontsize='small'); return

            sorted_names = sorted(data_dict.keys())
            x_margin, y_margin = (xmax - xmin) * 0.02, (ymax - ymin) * 0.02

            if MODO_ETIQUETAS == 3:
                if not first_pos_dict: return
                for i, name in enumerate(sorted_names):
                    if name not in first_pos_dict: continue
                    if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                    if MAX_ETIQUETAS_FIJAS is not None and name != OBSERVADOR_ID:
                        match = re.search(r'(\d+)$', name)
                        if match and int(match.group(1)) > MAX_ETIQUETAS_FIJAS: continue
                    x_val, y_val = first_pos_dict[name]
                    display_name, color = get_display_name(name), get_color(i) if is_video else name_to_color.get(name, get_color(i))
                    tx_clamped = max(xmin + x_margin, min(x_val + x_margin, xmax - (x_margin * 6)))
                    ty_clamped = max(ymin + y_margin, min(y_val + y_margin, ymax - (y_margin * 2)))
                    ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=9, fontweight='bold', alpha=1.0, zorder=5, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
                return

            x_threshold, y_threshold = (xmax - xmin) * 0.06, (ymax - ymin) * 0.04  
            drawn_positions = []  

            for i, name in enumerate(sorted_names):
                if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                xs, ys = data_dict[name]['xs'], data_dict[name]['ys']
                display_name, color = get_display_name(name), get_color(i) if is_video else name_to_color.get(name, get_color(i))
                if xs and ys and xs[-1] is not None:
                    tx_clamped = max(xmin + x_margin, min(xs[-1] + x_margin, xmax - (x_margin * 6)))
                    ty_clamped = max(ymin + y_margin, min(ys[-1] + y_margin, ymax - (y_margin * 2)))
                    overlap = False
                    for dx, dy in drawn_positions:
                        if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold: overlap = True; break
                    if overlap and ELIMINAR_SUPERPUESTAS: continue  
                    drawn_positions.append((tx_clamped, ty_clamped))
                    ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=9, fontweight='bold', zorder=5, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))

        def update(frame):
            nonlocal next_index, latest_sim_time, last_obs_pos, distancia_total_observador, total_maestros_set

            while True:
                try: category, name, val1, val2, sim_ts, extra = q.get_nowait()
                except queue.Empty: break
                latest_sim_time = max(latest_sim_time, sim_ts)
                
                if category == 'al_data':
                    current_al_data[name] = {'ts': sim_ts, 'data': extra, 'x': val1, 'y': val2}
                    continue 

                if use_filter_robots and name not in robots_filter: continue
                if name not in first_seen_positions: first_seen_positions[name] = (val1, val2)
                if name not in data_robots:
                    data_robots[name]['xs'] = []; data_robots[name]['ys'] = []
                    name_to_color[name] = get_color(next_index)
                    next_index += 1

                prev_sim_ts = last_seen_map.get(name, sim_ts)
                if name not in TRAYECTORIAS_PERMANENTES:
                    if (sim_ts - prev_sim_ts) > DEAD_TIME_LIMIT:
                        if DELETE_ON_TIMEOUT: data_robots[name]['xs'], data_robots[name]['ys'] = [], []
                        else: data_robots[name]['xs'].append(None); data_robots[name]['ys'].append(None)
                        
                    if name != OBSERVADOR_ID and val1 is not None:
                        total_maestros_set.add(name)

                if name == OBSERVADOR_ID:
                    if MOSTRAR_MAPA_CALOR:
                        add_heat(val1, val2, heatmap_live, args.xmin, args.xmax, args.ymin, args.ymax, RESOLUCION_MAPA, OBSERVADOR_USA_DIAMETRO, DIAMETRO_ROBOT)
                        
                    if last_obs_pos is not None and val1 is not None and val2 is not None:
                        dist = math.hypot(val1 - last_obs_pos[0], val2 - last_obs_pos[1])
                        if dist < 1.0: 
                            distancia_total_observador += dist
                    
                    if val1 is not None and val2 is not None:
                        last_obs_pos = (val1, val2)

                data_robots[name]['xs'].append(val1)  
                data_robots[name]['ys'].append(val2)  
                last_seen_map[name] = sim_ts 

            if DELETE_ON_TIMEOUT:
                for name in list(data_robots.keys()):
                    if name not in TRAYECTORIAS_PERMANENTES:
                        if name in last_seen_map and (latest_sim_time - last_seen_map[name]) > DEAD_TIME_LIMIT:
                            if data_robots[name]['xs']: data_robots[name]['xs'], data_robots[name]['ys'] = [], []

            if args.mode == 2:
                N = max(1, int(args.window))
                for name in list(data_robots.keys()):
                    if name not in TRAYECTORIAS_PERMANENTES and len(data_robots[name]['xs']) > N:
                        data_robots[name]['xs'] = data_robots[name]['xs'][-N:]
                        data_robots[name]['ys'] = data_robots[name]['ys'][-N:]

            ax_main.clear()
            ax_main.set_aspect('equal', adjustable='box')
            ax_main.set_xlim(args.xmin, args.xmax); ax_main.set_ylim(args.ymin, args.ymax)
            ax_main.set_xlabel('X'); ax_main.set_ylabel('Y')
            ax_main.set_title(get_formatted_title(TITULO_GRAFICA, latest_sim_time, SIM_TICKS_PER_SEC))
            ax_main.grid(True)
            
            if ax_map is not None and ax_map != ax_main:
                ax_map.clear()
                ax_map.set_aspect('equal', adjustable='box')
                ax_map.set_xlim(args.xmin, args.xmax); ax_map.set_ylim(args.ymin, args.ymax)
                ax_map.set_xlabel('X'); ax_map.set_ylabel('Y')
                ax_map.set_title(get_formatted_title("Mapa de Calor", latest_sim_time, SIM_TICKS_PER_SEC))
                ax_map.grid(True)

            pts_per_unit = (ax_main.get_window_extent().width * 72 / fig_main.dpi) / (args.xmax - args.xmin)
            lw_diam = max(1.5, DIAMETRO_ROBOT * pts_per_unit) 

            # --- DIBUJAR MAPA DE CALOR ---
            if MOSTRAR_MAPA_CALOR and OBSERVADOR_ID in data_robots:
                max_heat = np.max(heatmap_live)
                if max_heat > 0:
                    vmax_calc = max_heat * (10.0 / max(0.1, SENSIBILIDAD_MAPA))
                    vmax_calc = max(vmax_calc, 0.1) 
                    alpha_map_calc = min(1.0, max(0.0, 0.5 * TRANSPARENCIA_MAPA)) if DISPOSICION_MAPA == 1 else min(1.0, max(0.0, TRANSPARENCIA_MAPA))
                    ax_map.imshow(heatmap_live, extent=(args.xmin, args.xmax, args.ymin, args.ymax),
                                   origin='lower', cmap=heat_cmap, vmin=0.1, vmax=vmax_calc, alpha=alpha_map_calc,
                                   interpolation='gaussian', zorder=1)
                    if ax_map != ax_main:
                        xs_obs, ys_obs = data_robots[OBSERVADOR_ID]['xs'], data_robots[OBSERVADOR_ID]['ys']
                        if xs_obs and ys_obs and xs_obs[-1] is not None:
                            ax_map.scatter([xs_obs[-1]], [ys_obs[-1]], s=25, color='white', edgecolor='black', zorder=5)

            # --- ACTUALIZAR PANEL LATERAL ---
            if ax_info is not None:
                ax_info.clear()
                ax_info.axis('off')
                maestros_activos = sum(1 for r_name, r_data in data_robots.items() if r_name != OBSERVADOR_ID and r_name not in TRAYECTORIAS_PERMANENTES and len(r_data['xs']) > 0 and r_data['xs'][-1] is not None)
                info_text = (f"ESTADÍSTICAS\n{'-'*20}\n\n"
                             f"Tiempo Transcurrido:\n   {latest_sim_time:.2f} s\n\n"
                             f"Maestros Activos:\n   {maestros_activos}\n\n"
                             f"Total Maestros Vistos:\n   {len(total_maestros_set)}\n\n"
                             f"Distancia Recorrida:\n   {distancia_total_observador:.2f} m\n")
                ax_info.text(0.0, 0.95, info_text, transform=ax_info.transAxes, fontsize=11, va='top', ha='left', fontweight='bold', color='#333333')

            for i, name in enumerate(sorted(data_robots.keys())):
                xs, ys = data_robots[name]['xs'], data_robots[name]['ys']
                display_name, color = get_display_name(name), name_to_color.get(name, get_color(i))
                if not xs or not ys: continue
                if name == OBSERVADOR_ID:
                    lw_current = lw_diam if OBSERVADOR_USA_DIAMETRO else 1.5
                    s_current  = lw_diam**2 if OBSERVADOR_USA_DIAMETRO else 25
                else:
                    lw_current = lw_diam if MAESTROS_USAN_DIAMETRO else 1.5
                    s_current  = lw_diam**2 if MAESTROS_USAN_DIAMETRO else 25

                if name == OBSERVADOR_ID:
                    if MODO_VISUAL_OBSERVADOR == 0: continue 
                    elif MODO_VISUAL_OBSERVADOR == 1:
                        if xs[-1] is not None: ax_main.scatter([xs[-1]], [ys[-1]], s=s_current, color=color, label=display_name, zorder=6)
                    elif MODO_VISUAL_OBSERVADOR == 2:
                        ax_main.plot(xs, ys, lw=lw_current, label=display_name, color=color, zorder=6, solid_capstyle='round')
                        if xs[-1] is not None: ax_main.scatter([xs[-1]], [ys[-1]], s=s_current, color=color, zorder=6)
                else:
                    if MODO_TRAYECTORIA_MAESTROS == 1:
                        ax_main.plot(xs, ys, lw=lw_current, label=display_name, color=color, zorder=6, solid_capstyle='round')
                        if xs[-1] is not None: ax_main.scatter([xs[-1]], [ys[-1]], s=s_current, color=color, zorder=6)
                    else:
                        xs_clean = [x for x in xs if x is not None]
                        ys_clean = [y for y in ys if y is not None]
                        if xs_clean: ax_main.scatter(xs_clean, ys_clean, s=s_current, color=color, label=display_name, zorder=6)

            if MODO_VISUAL_OBSERVADOR > 0 and MODO_CIRCULO > 0 and OBSERVADOR_ID in data_robots:
                xs_obs, ys_obs = data_robots[OBSERVADOR_ID]['xs'], data_robots[OBSERVADOR_ID]['ys']
                if xs_obs and ys_obs and xs_obs[-1] is not None:
                    cx, cy = xs_obs[-1], ys_obs[-1]
                    if MODO_CIRCULO == 1: ax_main.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='--', linewidth=1.5, alpha=0.7, zorder=5))
                    elif MODO_CIRCULO == 2:
                        ax_main.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=True, alpha=0.15, zorder=5))
                        ax_main.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='-', linewidth=1.5, alpha=0.5, zorder=5))

            for n in list(current_al_data.keys()): 
                if latest_sim_time - current_al_data[n]['ts'] > DEAD_TIME_LIMIT: del current_al_data[n]

            # =========================================================================
            # RENDERIZADO MODULAR DE ACTIVE LEARNING (ENVIVO)
            # =========================================================================
            for n, al_info in current_al_data.items():
                x_rob, y_rob = al_info['x'], al_info['y']
                mode = al_info['data']['mode']
                extras = al_info['data']['extras']
                
                if mode == 1:
                    # Modo 1: Centroide Básico
                    if len(extras) >= 2:
                        cx, cy = extras[0], extras[1]
                        ax_main.arrow(x_rob, y_rob, cx, cy, head_width=0.04, head_length=0.06, fc='orange', ec='orange', alpha=0.8, length_includes_head=True, zorder=7)
                        ax_main.plot(x_rob + cx, y_rob + cy, marker='X', markersize=6, color='orange', markeredgewidth=1, zorder=7)
                
                elif mode == 2:
                    # Modo 2: Centroide Threshold
                    if len(extras) >= 2:
                        cx, cy = extras[0], extras[1]
                        ax_main.arrow(x_rob, y_rob, cx, cy, head_width=0.04, head_length=0.06, fc='orange', ec='orange', alpha=0.8, length_includes_head=True, zorder=7)
                        ax_main.plot(x_rob + cx, y_rob + cy, marker='X', markersize=6, color='orange', markeredgewidth=1, zorder=7)
                
                elif mode == 3:
                    # Modo 3: Centroide PID
                    if len(extras) >= 2:
                        cx, cy = extras[0], extras[1]
                        ax_main.arrow(x_rob, y_rob, cx, cy, head_width=0.04, head_length=0.06, fc='orange', ec='orange', alpha=0.8, length_includes_head=True, zorder=7)
                        ax_main.plot(x_rob + cx, y_rob + cy, marker='X', markersize=6, color='orange', markeredgewidth=1, zorder=7)
                
                elif mode == 4:
                    # Modo 4: Campos Vectoriales
                    if len(extras) >= 2:
                        fx, fy = extras[0], extras[1]
                        ax_main.arrow(x_rob, y_rob, fx, fy, head_width=0.04, head_length=0.06, fc='red', ec='red', alpha=0.9, linewidth=2, length_includes_head=True, zorder=7)

            draw_labels_clamped(ax_main, data_robots, args.xmin, args.xmax, args.ymin, args.ymax, first_pos_dict=first_seen_positions)
            return []

        interval_ms = int(1000 / max(1, int(args.live_fps)))
        ani = animation.FuncAnimation(fig_main, update, interval=interval_ms, blit=False, cache_frame_data=False)

        printed_stats = False
        try:
            plt.ion()
            plt.show()
            while plt.fignum_exists(fig_main.number):
                plt.pause(0.1)
                if args.duration and (time.time() - start_time) > args.duration: stop_event.set()
                if inactivity_timeout is not None:
                    if (time.time() - last_time_holder[0]) > inactivity_timeout: stop_event.set()
                
                if stop_event.is_set() and q.empty():
                    if not printed_stats:
                        print(f"\n[Tracker trajectory] Total_distancia_apprentice={distancia_total_observador:.4f}&Total_masters={len(total_maestros_set)}")
                        printed_stats = True
                        ani.event_source.stop()  
                    if AUTO_CLOSE_WINDOWS: break
                        
        except KeyboardInterrupt: 
            stop_event.set()
            if not printed_stats:
                print(f"\n[Tracker trajectory] Total_distancia_apprentice={distancia_total_observador:.4f}&Total_masters={len(total_maestros_set)}")
                printed_stats = True

        if plt.fignum_exists(fig_main.number):
            if AUTO_CLOSE_WINDOWS:
                if args.inactivity_timeout is not None and args.inactivity_timeout > 0:
                    time_since_last = time.time() - last_time_holder[0]
                    remaining = args.inactivity_timeout - time_since_last
                    if remaining > 0:
                        t_end = time.time() + remaining
                        while time.time() < t_end and plt.fignum_exists(fig_main.number): plt.pause(0.1)  
                if plt.fignum_exists(fig_main.number): plt.close(fig_main)
            else:
                plt.ioff()
                plt.show()

        if not printed_stats:
            print(f"\n[Tracker trajectory] Total_distancia_apprentice={distancia_total_observador:.4f}&Total_masters={len(total_maestros_set)}")
            printed_stats = True

    if not timeline: return
    names_all = set()
    for frame_events in timeline:
        for (cat, n, x, y, ts, extra) in frame_events: 
            if cat == 'robot': names_all.add(n)

    names_to_plot = sorted([n for n in names_all if n in robots_filter]) if use_filter_robots else sorted(names_all)

    # =========================================================================
    # GUARDADO DE CSV
    # =========================================================================
    if save_csv:
        csv_dir_path = os.path.abspath(CSV_DIR)
        print(f"\n[Tracker Trayectoria] [+] Guardando archivos CSV...")
        os.makedirs(csv_dir_path, exist_ok=True)
        samples = {n: [] for n in names_all}

        for frame_events in timeline:
            for (cat, n, x, y, ts, extra) in frame_events: 
                if cat == 'robot': samples[n].append((ts, x, y))

        for n, rows in samples.items():
            if not rows: continue
            safe_name = n.replace(" ", "_")
            csv_path = os.path.join(csv_dir_path, f"{safe_name}.csv")
            try:
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time_iso', 'timestamp', 'x', 'y'])
                    for ts, x, y in rows: writer.writerow([iso(ts), f"{ts:.6f}", f"{x:.6f}", f"{y:.6f}"])
            except Exception as e: print(f"[Tracker Trayectoria] [!] Error guardando CSV: {e}")

    # =========================================================================
    # GUARDADO DE VIDEO (Piping a RAM)
    # =========================================================================
    if save_video:
        video_dir_path = os.path.abspath(VIDEO_DIR)
        os.makedirs(video_dir_path, exist_ok=True)
        video_file_path = os.path.join(video_dir_path, args.outfile)

        sim_time_per_tick = 1.0 / SIM_TICKS_PER_SEC
        video_frame_interval = 1.0 / args.video_fps
        next_frame_sim_time, current_sim_time = 0.0, 0.0
        visual_frames, current_visual_frame = [], []

        for tick_events in timeline:
            current_visual_frame.append(tick_events)
            current_sim_time += sim_time_per_tick
            if current_sim_time + 1e-6 >= next_frame_sim_time:
                visual_frames.append(current_visual_frame)
                current_visual_frame = []
                while next_frame_sim_time <= current_sim_time + 1e-6: next_frame_sim_time += video_frame_interval
        if current_visual_frame: visual_frames.append(current_visual_frame)

        # La tasa de fotogramas de salida es FPS extraídos multiplicados por la velocidad deseada.
        playback_fps = args.video_fps * args.video_speed 

        try:
            accum = {n: {'xs': [], 'ys': []} for n in names_to_plot}
            vid_last_seen, vid_first_seen_positions = {}, {}
            fallback_ts = 0.0
            heatmap_video = np.zeros((RESOLUCION_MAPA, RESOLUCION_MAPA))

            if RENDERIZAR_MAPA_CALOR and DISPOSICION_MAPA == 2:
                fig_save = Figure(figsize=(14,7), dpi=100) 
                FigureCanvas(fig_save)  # Asignación forzada de backend Agg
                ax_save_map = fig_save.add_subplot(121)
                ax_save = fig_save.add_subplot(122)
            else:
                fig_save = Figure(figsize=(7,7), dpi=100)
                FigureCanvas(fig_save)  # Asignación forzada de backend Agg
                ax_save = fig_save.add_subplot(111)
                ax_save_map = ax_save if RENDERIZAR_MAPA_CALOR else None

            fig_save.canvas.draw()
            width, height = fig_save.canvas.get_width_height()

            ffmpeg_cmd = [
                "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo", "-s", f"{width}x{height}",
                "-pix_fmt", "rgba", "-r", str(playback_fps), "-i", "-", "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", video_file_path
            ]
            ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            def ffmpeg_monitor():
                for line in ffmpeg_proc.stdout:
                    pass # Silenciamos FFMPEG para usar nuestro propio contador limpio
            
            monitor_thread = threading.Thread(target=ffmpeg_monitor, daemon=True)
            monitor_thread.start()

            vid_current_al_data = {}
            total_frames = len(visual_frames) # Total de frames a renderizar

            for idx, v_frame in enumerate(visual_frames):
                current_tick_ts = fallback_ts
                for tick_list in v_frame:
                    for ev in tick_list:
                        if ev[4] > current_tick_ts: current_tick_ts = ev[4]
                fallback_ts = current_tick_ts

                if DELETE_ON_TIMEOUT:
                    for n in list(accum.keys()):
                        if n not in TRAYECTORIAS_PERMANENTES and n in vid_last_seen and (current_tick_ts - vid_last_seen[n]) > DEAD_TIME_LIMIT:
                            accum[n]['xs'], accum[n]['ys'] = [], []

                for tick_events in v_frame:
                    for (cat, n, x, y, ts, extra) in tick_events:
                        if cat == 'al_data':
                            vid_current_al_data[n] = {'ts': ts, 'data': extra, 'x': x, 'y': y}
                        elif cat == 'robot' and n in accum:
                            if n not in vid_first_seen_positions: vid_first_seen_positions[n] = (x, y)
                            if n not in TRAYECTORIAS_PERMANENTES and (ts - vid_last_seen.get(n, ts)) > DEAD_TIME_LIMIT:
                                if DELETE_ON_TIMEOUT: accum[n]['xs'], accum[n]['ys'] = [], []
                                else: accum[n]['xs'].append(None); accum[n]['ys'].append(None)
                                
                            if n == OBSERVADOR_ID and RENDERIZAR_MAPA_CALOR:
                                add_heat(x, y, heatmap_video, args.xmin, args.xmax, args.ymin, args.ymax, RESOLUCION_MAPA, OBSERVADOR_USA_DIAMETRO, DIAMETRO_ROBOT)
                                
                            accum[n]['xs'].append(x); accum[n]['ys'].append(y)
                            vid_last_seen[n] = ts

                if args.mode == 2:
                    N = max(1, int(args.window))
                    for n in accum:
                        if n not in TRAYECTORIAS_PERMANENTES and len(accum[n]['xs']) > N:
                            accum[n]['xs'], accum[n]['ys'] = accum[n]['xs'][-N:], accum[n]['ys'][-N:]

                ax_save.clear()
                ax_save.set_aspect('equal', adjustable='box') 
                ax_save.set_xlim(args.xmin, args.xmax); ax_save.set_ylim(args.ymin, args.ymax)
                ax_save.set_xlabel('X'); ax_save.set_ylabel('Y')
                ax_save.set_title(get_formatted_title(TITULO_GRAFICA, current_tick_ts, SIM_TICKS_PER_SEC))
                ax_save.grid(True)
                
                if ax_save_map is not None and ax_save_map != ax_save:
                    ax_save_map.clear()
                    ax_save_map.set_aspect('equal', adjustable='box') 
                    ax_save_map.set_xlim(args.xmin, args.xmax); ax_save_map.set_ylim(args.ymin, args.ymax)
                    ax_save_map.set_xlabel('X'); ax_save_map.set_ylabel('Y')
                    ax_save_map.set_title(get_formatted_title("Mapa de Calor", current_tick_ts, SIM_TICKS_PER_SEC))
                    ax_save_map.grid(True)

                pts_per_unit_vid = (ax_save.get_window_extent().width * 72 / fig_save.dpi) / (args.xmax - args.xmin)
                lw_diam_vid = max(1.5, DIAMETRO_ROBOT * pts_per_unit_vid)

                if RENDERIZAR_MAPA_CALOR and OBSERVADOR_ID in accum:
                    max_heat_vid = np.max(heatmap_video)
                    if max_heat_vid > 0:
                        vmax_calc_vid = max_heat_vid * (10.0 / max(0.1, SENSIBILIDAD_MAPA))
                        vmax_calc_vid = max(vmax_calc_vid, 0.1)
                        alpha_map_calc_vid = min(1.0, max(0.0, 0.5 * TRANSPARENCIA_MAPA)) if DISPOSICION_MAPA == 1 else min(1.0, max(0.0, TRANSPARENCIA_MAPA))
                        ax_save_map.imshow(heatmap_video, extent=(args.xmin, args.xmax, args.ymin, args.ymax),
                                       origin='lower', cmap=heat_cmap, vmin=0.1, vmax=vmax_calc_vid, alpha=alpha_map_calc_vid,
                                       interpolation='gaussian', zorder=1)
                        if ax_save_map != ax_save:
                            xs_obs, ys_obs = accum[OBSERVADOR_ID]['xs'], accum[OBSERVADOR_ID]['ys']
                            if xs_obs and ys_obs and xs_obs[-1] is not None:
                                ax_save_map.scatter([xs_obs[-1]], [ys_obs[-1]], s=25, color='white', edgecolor='black', zorder=5)

                for i, n in enumerate(names_to_plot):
                    xs, ys = accum[n]['xs'], accum[n]['ys']
                    display_name, color = get_display_name(n), get_color(i)
                    if not xs or not ys: continue
                    
                    if n == OBSERVADOR_ID:
                        lw_current = lw_diam_vid if OBSERVADOR_USA_DIAMETRO else 1.5
                        s_current  = lw_diam_vid**2 if OBSERVADOR_USA_DIAMETRO else 25
                    else:
                        lw_current = lw_diam_vid if MAESTROS_USAN_DIAMETRO else 1.5
                        s_current  = lw_diam_vid**2 if MAESTROS_USAN_DIAMETRO else 25
                    
                    if n == OBSERVADOR_ID:
                        if MODO_VISUAL_OBSERVADOR == 0: continue 
                        elif MODO_VISUAL_OBSERVADOR == 1:
                            if xs[-1] is not None: ax_save.scatter([xs[-1]], [ys[-1]], s=s_current, color=color, label=display_name, zorder=6)
                        elif MODO_VISUAL_OBSERVADOR == 2:
                            ax_save.plot(xs, ys, lw=lw_current, label=display_name, color=color, zorder=6, solid_capstyle='round')
                            if xs[-1] is not None: ax_save.scatter([xs[-1]], [ys[-1]], s=s_current, color=color, zorder=6)
                    else:
                        if MODO_TRAYECTORIA_MAESTROS == 1:
                            ax_save.plot(xs, ys, lw=lw_current, label=display_name, color=color, zorder=6, solid_capstyle='round')
                            if xs[-1] is not None: ax_save.scatter([xs[-1]], [ys[-1]], s=s_current, color=color, zorder=6)
                        else:
                            xs_clean = [x for x in xs if x is not None]
                            ys_clean = [y for y in ys if y is not None]
                            if xs_clean: ax_save.scatter(xs_clean, ys_clean, s=s_current, color=color, label=display_name, zorder=6)

                if MODO_VISUAL_OBSERVADOR > 0 and MODO_CIRCULO > 0 and OBSERVADOR_ID in accum:
                    xs_obs, ys_obs = accum[OBSERVADOR_ID]['xs'], accum[OBSERVADOR_ID]['ys']
                    if xs_obs and ys_obs and xs_obs[-1] is not None:
                        cx, cy = xs_obs[-1], ys_obs[-1]
                        if MODO_CIRCULO == 1: ax_save.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='--', linewidth=1.5, alpha=0.7, zorder=5))
                        elif MODO_CIRCULO == 2:
                            ax_save.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=True, alpha=0.15, zorder=5))
                            ax_save.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='-', linewidth=1.5, alpha=0.5, zorder=5))

                for n in list(vid_current_al_data.keys()):
                    if current_tick_ts - vid_current_al_data[n]['ts'] > DEAD_TIME_LIMIT: del vid_current_al_data[n]

                # =========================================================================
                # RENDERIZADO MODULAR DE ACTIVE LEARNING (VIDEO OFFLINE)
                # =========================================================================
                for n, al_info in vid_current_al_data.items():
                    x_rob, y_rob = al_info['x'], al_info['y']
                    mode = al_info['data']['mode']
                    extras = al_info['data']['extras']
                    
                    if mode == 1:
                        if len(extras) >= 2:
                            cx, cy = extras[0], extras[1]
                            ax_save.arrow(x_rob, y_rob, cx, cy, head_width=0.04, head_length=0.06, fc='orange', ec='orange', alpha=0.8, length_includes_head=True, zorder=7)
                            ax_save.plot(x_rob + cx, y_rob + cy, marker='X', markersize=6, color='orange', markeredgewidth=1, zorder=7)
                    elif mode == 2:
                        if len(extras) >= 2:
                            cx, cy = extras[0], extras[1]
                            ax_save.arrow(x_rob, y_rob, cx, cy, head_width=0.04, head_length=0.06, fc='orange', ec='orange', alpha=0.8, length_includes_head=True, zorder=7)
                            ax_save.plot(x_rob + cx, y_rob + cy, marker='X', markersize=6, color='orange', markeredgewidth=1, zorder=7)
                    elif mode == 3:
                        if len(extras) >= 2:
                            cx, cy = extras[0], extras[1]
                            ax_save.arrow(x_rob, y_rob, cx, cy, head_width=0.04, head_length=0.06, fc='orange', ec='orange', alpha=0.8, length_includes_head=True, zorder=7)
                            ax_save.plot(x_rob + cx, y_rob + cy, marker='X', markersize=6, color='orange', markeredgewidth=1, zorder=7)
                    elif mode == 4:
                        if len(extras) >= 2:
                            fx, fy = extras[0], extras[1]
                            ax_save.arrow(x_rob, y_rob, fx, fy, head_width=0.04, head_length=0.06, fc='red', ec='red', alpha=0.9, linewidth=2, length_includes_head=True, zorder=7)

                sorted_names = sorted(accum.keys())
                x_margin, y_margin = (args.xmax - args.xmin) * 0.02, (args.ymax - args.ymin) * 0.02
                if MODO_ETIQUETAS == 3:
                    for i, name in enumerate(sorted_names):
                        if name not in vid_first_seen_positions: continue
                        if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                        x_val, y_val = vid_first_seen_positions[name]
                        tx_clamped = max(args.xmin + x_margin, min(x_val + x_margin, args.xmax - (x_margin * 6)))
                        ty_clamped = max(args.ymin + y_margin, min(y_val + y_margin, args.ymax - (y_margin * 2)))
                        ax_save.text(tx_clamped, ty_clamped, get_display_name(name), color=get_color(i), fontsize=9, fontweight='bold', zorder=8, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
                else:
                    x_threshold, y_threshold = (args.xmax - args.xmin) * 0.06, (args.ymax - args.ymin) * 0.04  
                    drawn_positions = []
                    for i, name in enumerate(sorted_names):
                        if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                        xs, ys = accum[name]['xs'], accum[name]['ys']
                        if xs and ys and xs[-1] is not None:
                            tx_clamped = max(args.xmin + x_margin, min(xs[-1] + x_margin, args.xmax - (x_margin * 6)))
                            ty_clamped = max(args.ymin + y_margin, min(ys[-1] + y_margin, args.ymax - (y_margin * 2)))
                            overlap = False
                            for dx, dy in drawn_positions:
                                if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold: overlap = True; break
                            if overlap and ELIMINAR_SUPERPUESTAS: continue  
                            drawn_positions.append((tx_clamped, ty_clamped))
                            ax_save.text(tx_clamped, ty_clamped, get_display_name(name), color=get_color(i), fontsize=9, fontweight='bold', zorder=8, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))

                # Extracción y escritura del buffer usando get_rgba_buffer directamente
                try: ffmpeg_proc.stdin.write(get_rgba_buffer(fig_save))
                except BrokenPipeError: break
                
                # --- PROGRESO EN PANTALLA ---
                sys.stdout.write(f"\r[Tracker Trayectoria] Renderizando frame {idx + 1} de {total_frames} ({(idx + 1)/total_frames*100:.1f}%)")
                sys.stdout.flush()
                # ----------------------------

                if (idx + 1) % 50 == 0: gc.collect()

            ffmpeg_proc.stdin.close(); ffmpeg_proc.wait(); monitor_thread.join()
            print("\n[Tracker Trayectoria] [+] ¡Video guardado exitosamente!")

        except Exception as e: print(f"\n[Tracker Trayectoria] [!] ERROR CRÍTICO VIDEO:\n {e}")

if __name__ == '__main__':
    main()