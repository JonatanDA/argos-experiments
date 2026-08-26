#!/usr/bin/env python3
"""
tracker_trayectoria.py

- controllers -> Ks1 (Corregir para las demás versiones que aún no se sincronizan con el timer global de la simulación en argos.)

1. AGRUPACIÓN EXPLÍCITA POR TICK:
   El script busca la línea `Timer=XXX` (impreso por Lua) para saber 
   exactamente dónde empieza y termina un frame de la simulación. Esto garantiza 
   una reconstrucción perfecta del tiempo.

2. CONTROL INDEPENDIENTE DE VELOCIDADES (FPS):
   - SIM_TICKS_PER_SEC  : Los ticks configurados en tu archivo .argos (ej. 10).
   - LIVE_FPS           : Frecuencia de actualización de la gráfica en vivo (ej. 2 FPS).
   - VIDEO_FPS          : Cuántos cuadros por segundo de simulación se van a extraer y renderizar.

3. CONTROL DE DURACIÓN DEL VIDEO (SYNC_VIDEO_DURATION):
   - True : Conserva la duración original. (Ej. Extrae 2 cuadros/seg y el video se reproduce a 2 FPS).
   - False: Acelera/Ralentiza el video. (Ej. Extrae 2 cuadros/seg pero se reproduce a los 10 FPS 
            originales de ARGoS, creando una cámara rápida 5x).

4. VIDEO ULTRA-RÁPIDO (PIPING):
   No se guardan imágenes PNG temporales. Los pixeles viajan directo de la 
   RAM a FFmpeg por una tubería, reduciendo el desgaste del disco y el tiempo.
"""

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Ventana Principal y Gráficas ---
TITULO_GRAFICA                  = "Datos observados"
MOSTRAR_TIEMPO_TITULO           = True              # True: Agregar tiempo/ticks transcurridos al título
UNIDAD_TIEMPO_TITULO            = "segundos"        # "segundos" o "ticks"
MODO                            = 1                 # 1: Toda la trayectoria, 2: Ventana de muestras (estela)    
WINDOW                          = 300               
DEFAULT_XMIN                    = -1.5              
DEFAULT_XMAX                    =  1.5
DEFAULT_YMIN                    = -1.5
DEFAULT_YMAX                    =  1.5
DEAD_TIME_LIMIT                 = 1                 
DELETE_ON_TIMEOUT               = True              

# --- CONTROL DE VELOCIDADES Y TIEMPO (FPS) ---
SIM_TICKS_PER_SEC               = 10                # Ticks de ARGoS por segundo de simulación
LIVE_FPS                        = 10                # FPS en vivo (Gráfica de Matplotlib)
VIDEO_FPS                       = 2                 # Cuadros a extraer por segundo de simulación
SYNC_VIDEO_DURATION             = False             # True: Duración real. False: Cámara rápida/lenta

# --- Etiquetas y Nombres ---
MODO_ETIQUETAS                  = 1                 
PREFIX_ETIQUETAS                = "Rm"              
NOMBRE_OBSERVADOR               = "Ra"              
MAX_ETIQUETAS_FIJAS             = 7                 
ELIMINAR_SUPERPUESTAS           = False             
MAX_HISTORICAS_TRANSPARENTES    = 3                 

# Listas y reglas de nombres configurables
PALABRAS_IGNORADAS              = ["maestro"]  # Lista de palabras clave para descartar líneas
PREFIJOS_NORMALIZABLES          = ["master"]     # Prefijos que se traducen a PREFIX_ETIQUETAS (ej. obj1 -> Rm1)
PRESERVAR_NOMBRES_LIBRES        = True                  # True: Nombres libres sin número o no listados pasan tal cual

# --- Representación del Observador (Cámara) ---
MODO_VISUAL_OBSERVADOR          = 1                 
MODO_CIRCULO                    = 2                 
ANGULO_APERTURA                 = 72.800            
ALTURA_CAMARA                   = 0.3               

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
# --- Rutas de Directorios ---
VIDEO_DIR                       = "Resultados/Robots/Qupa/Ks1/Trajectory"
CSV_DIR                         = "Resultados/Robots/Qupa/Ks1/Trajectory/Data"

# --- Video ---
SAVE_VIDEO                      = True                      
DEFAULT_OUTFILE                 = "Ks1.mp4"                 

# --- CSV ---
SAVE_CSV                        = False              # True: Para tener un backup siempre.

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
OBSERVADOR_ID                   = "Apprentice"         
FILTRO                          = False             # True: Mostrar solo los listados en ROBOTS
ROBOTS                          = "7,10,11"         # Lista de robots a mostrar (si FILTRO=True)

DEFAULT_END_PATTERN             = "End_simulation"  
AUTO_CLOSE_WINDOWS              = False             
AUTO_CLOSE_TIME                 = None              

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
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.animation as animation
# Importamos el motor de renderizado puro (sin interfaz gráfica) para el video
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

# --- CÁLCULO DEL RADIO DE VISIÓN ---
RADIO_VISION = ALTURA_CAMARA * math.tan(math.radians(ANGULO_APERTURA))
NUM_PATTERN = r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?'
STRICT_PAIR_RE = re.compile(rf'^\s*({NUM_PATTERN})\s*[,]?\s*({NUM_PATTERN})\s*$')

def parse_line(line):
    if '&' not in line: return None
    parts = line.split('&', 1)
    name_raw = parts[0].strip()
    match = STRICT_PAIR_RE.match(parts[1])
    if not match: return None
    name_clean = re.sub(r'^\[.*?\]\s*', '', name_raw)
    
    # Validar si el nombre empieza con alguna de las palabras ignoradas configuradas
    name_lower = name_clean.lower()
    if PALABRAS_IGNORADAS and any(name_lower.startswith(p.lower()) for p in PALABRAS_IGNORADAS): 
        return None
        
    nums = match.groups()
    try:
        x = float(nums[0].replace(',', '.'))
        y = float(nums[1].replace(',', '.'))
        return 'robot', name_clean, x, y
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
    if not MOSTRAR_TIEMPO_TITULO:
        return base_title
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

            # Condición de salida
            if end_pattern and end_pattern in line:
                if first_timer_seen: timeline.append(current_tick)
                stop_event.set()
                break

            # Delimitador explícito de Tick
            if line.lower().startswith("timer="):
                if first_timer_seen:
                    timeline.append(current_tick)
                    current_tick = []
                first_timer_seen = True
                match_val = re.search(r'(\d+)', line)
                if match_val:
                    tick_counter = int(match_val.group(1))
                else:
                    tick_counter += 1
                continue
            
            # Extraer número de tick si viene explícito (ej. Timer=26) o usar el contador
            match_timer = re.match(r'timer\s*=\s*(\d+)', line, re.IGNORECASE)
            if match_timer:
                if first_timer_seen:
                    timeline.append(current_tick)
                    current_tick = []
                first_timer_seen = True
                tick_counter = int(match_timer.group(1))
                continue

            # Lectura de coordenadas
            parsed = parse_line(line)
            if parsed:
                category, name, val1, val2 = parsed
                real_ts = time.time()
                # Usamos el tick_counter como una marca de tiempo basada en simulación (1 tick = 1/SIM_TICKS_PER_SEC segundos)
                sim_pseudo_ts = tick_counter / float(SIM_TICKS_PER_SEC)
                
                q.put((category, name, val1, val2, sim_pseudo_ts))

                # Agrupar si vimos el Timer, si no, lo guardamos como ticks individuales (Fallback)
                if first_timer_seen:
                    current_tick.append((name, val1, val2, sim_pseudo_ts))
                else:
                    timeline.append([(name, val1, val2, sim_pseudo_ts)])

                last_time_holder[0] = real_ts

    except Exception: pass
    finally:
        if first_timer_seen and not stop_event.is_set():
            timeline.append(current_tick)
        stop_event.set()

def get_color(i, cmap='tab20'):
    return plt.get_cmap(cmap)(i % plt.get_cmap(cmap).N)

def iso(ts):
    return datetime.fromtimestamp(ts).isoformat(sep=' ', timespec='milliseconds')

def get_display_name(name):
    if name == OBSERVADOR_ID and NOMBRE_OBSERVADOR != "": return NOMBRE_OBSERVADOR
    
    # 1. Verificar si coincide con los prefijos normalizables (ej. obj1 -> Rm1)
    if PREFIJOS_NORMALIZABLES:
        match_norm = re.match(r'^([a-zA-Z]+)(\d+)$', name)
        if match_norm:
            prefix_found = match_norm.group(1).lower()
            num_found = match_norm.group(2)
            if any(prefix_found == p.lower() for p in PREFIJOS_NORMALIZABLES):
                if PREFIX_ETIQUETAS != "":
                    return f"{PREFIX_ETIQUETAS}{num_found}"
                return name

    # 2. Si es un nombre libre sin número y se preservan nombres libres
    if PRESERVAR_NOMBRES_LIBRES:
        match_num = re.search(r'(\d+)$', name)
        if not match_num:
            return name

    # 3. Comportamiento estándar con prefijo
    if PREFIX_ETIQUETAS != "":
        match = re.search(r'(\d+)$', name)
        if match: return f"{PREFIX_ETIQUETAS}{match.group(1)}"
        else: return f"{PREFIX_ETIQUETAS}_{name}"
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
                    events.append((name, float(row['x']), float(row['y']), float(row['timestamp'])))

    events.sort(key=lambda x: x[3])
    timeline = []
    current_ts = -1
    current_tick = []
    # Agrupamos artificialmente por timestamp para emular el 'Timer=' en el offline
    for ev in events:
        if ev[3] != current_ts:
            if current_tick: timeline.append(current_tick)
            current_tick = [ev]
            current_ts = ev[3]
        else:
            current_tick.append(ev)
    if current_tick: timeline.append(current_tick)

    return timeline

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xmin', type=float, default=DEFAULT_XMIN)
    parser.add_argument('--xmax', type=float, default=DEFAULT_XMAX)
    parser.add_argument('--ymin', type=float, default=DEFAULT_YMIN)
    parser.add_argument('--ymax', type=float, default=DEFAULT_YMAX)
    parser.add_argument('--live-fps', type=int, default=LIVE_FPS)
    parser.add_argument('--video-fps', type=int, default=VIDEO_FPS)
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

    robots_filter = parse_robots_arg(args.robots) if args.robots else (parse_robots_arg(ROBOTS) if FILTRO else None)
    use_filter_robots = True if robots_filter else False

    save_video = False if args.no_video else SAVE_VIDEO
    save_csv = False if args.no_csv else SAVE_CSV

    timeline = []

    # =========================================================================
    # EJECUCIÓN (NORMAL vs OFFLINE)
    # =========================================================================
    if args.from_csv:
        print("\n[+] MODO OFFLINE ACTIVADO: Generando video desde CSVs...")
        timeline = load_timeline_from_csv(CSV_DIR)
        if not timeline:
            print(f"[!] No se encontraron datos CSV en: {CSV_DIR}")
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
        name_to_color = {}
        last_seen_map = {} 
        first_seen_positions = {}  
        next_index = 0
        latest_sim_time = 0.0

        fig_main, ax_main = plt.subplots(figsize=(7,7))
        fig_main.canvas.manager.set_window_title(TITULO_GRAFICA)

        def draw_labels_clamped(ax, data_dict, xmin, xmax, ymin, ymax, is_video=False, first_pos_dict=None):
            if not data_dict: return
            if MODO_ETIQUETAS == 0:
                ax.legend(loc='upper right', fontsize='small')
                return

            sorted_names = sorted(data_dict.keys())
            x_margin = (xmax - xmin) * 0.02
            y_margin = (ymax - ymin) * 0.02

            if MODO_ETIQUETAS == 3:
                if not first_pos_dict: return
                for i, name in enumerate(sorted_names):
                    if name not in first_pos_dict: continue
                    if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                    if MAX_ETIQUETAS_FIJAS is not None and name != OBSERVADOR_ID:
                        match = re.search(r'(\d+)$', name)
                        if match and int(match.group(1)) > MAX_ETIQUETAS_FIJAS: continue

                    x_val, y_val = first_pos_dict[name]
                    display_name = get_display_name(name)
                    color = get_color(i) if is_video else name_to_color.get(name, get_color(i))

                    tx_clamped = max(xmin + x_margin, min(x_val + x_margin, xmax - (x_margin * 6)))
                    ty_clamped = max(ymin + y_margin, min(y_val + y_margin, ymax - (y_margin * 2)))

                    ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=9, fontweight='bold', alpha=1.0, zorder=5,
                            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
                return

            x_threshold, y_threshold = (xmax - xmin) * 0.06, (ymax - ymin) * 0.04  
            drawn_positions = []  

            for i, name in enumerate(sorted_names):
                if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                xs, ys = data_dict[name]['xs'], data_dict[name]['ys']
                display_name = get_display_name(name)
                color = get_color(i) if is_video else name_to_color.get(name, get_color(i))

                if xs and ys and xs[-1] is not None:
                    tx_clamped = max(xmin + x_margin, min(xs[-1] + x_margin, xmax - (x_margin * 6)))
                    ty_clamped = max(ymin + y_margin, min(ys[-1] + y_margin, ymax - (y_margin * 2)))

                    overlap = False
                    for dx, dy in drawn_positions:
                        if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold:
                            overlap = True; break

                    if overlap and ELIMINAR_SUPERPUESTAS: continue  
                    drawn_positions.append((tx_clamped, ty_clamped))
                    ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=9, fontweight='bold', zorder=5,
                            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))

            if not ELIMINAR_SUPERPUESTAS:
                for i, name in enumerate(sorted_names):
                    if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR in [0, 1]: continue
                    xs, ys = data_dict[name]['xs'], data_dict[name]['ys']
                    display_name = get_display_name(name)
                    color = get_color(i) if is_video else name_to_color.get(name, get_color(i))
                    if not xs or not ys: continue

                    historical_indices = []
                    count = 0
                    for idx in range(len(xs) - 2, -1, -1):  
                        if xs[idx] is not None:
                            historical_indices.append(idx)
                            count += 1
                            if count >= MAX_HISTORICAS_TRANSPARENTES: break

                    for s_idx, pos in enumerate(historical_indices):
                        tx_clamped = max(xmin + x_margin, min(xs[pos] + x_margin, xmax - (x_margin * 6)))
                        ty_clamped = max(ymin + y_margin, min(ys[pos] + y_margin, ymax - (y_margin * 2)))

                        overlap = False
                        for dx, dy in drawn_positions:
                            if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold:
                                overlap = True; break
                        if overlap: continue  

                        distance_factor = s_idx + 1
                        alpha_val = max(0.12, 0.8 - (distance_factor * (0.7 / MAX_HISTORICAS_TRANSPARENTES)))
                        drawn_positions.append((tx_clamped, ty_clamped))
                        ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=8, alpha=alpha_val, zorder=4,
                                bbox=dict(facecolor='white', alpha=0.6 * alpha_val, edgecolor='none', pad=1))

        def update(frame):
            nonlocal next_index, latest_sim_time

            while True:
                try: category, name, val1, val2, sim_ts = q.get_nowait()
                except queue.Empty: break
                latest_sim_time = max(latest_sim_time, sim_ts)
                if use_filter_robots and name not in robots_filter: continue
                if name not in first_seen_positions: first_seen_positions[name] = (val1, val2)
                if name not in data_robots:
                    data_robots[name]['xs'] = []; data_robots[name]['ys'] = []
                    name_to_color[name] = get_color(next_index)
                    next_index += 1

                prev_sim_ts = last_seen_map.get(name, sim_ts)
                if (sim_ts - prev_sim_ts) > DEAD_TIME_LIMIT:
                    if DELETE_ON_TIMEOUT:
                        data_robots[name]['xs'] = []
                        data_robots[name]['ys'] = []
                    else:
                        data_robots[name]['xs'].append(None)
                        data_robots[name]['ys'].append(None)

                data_robots[name]['xs'].append(val1)  
                data_robots[name]['ys'].append(val2)  
                last_seen_map[name] = sim_ts 

            if DELETE_ON_TIMEOUT:
                for name in list(data_robots.keys()):
                    if name in last_seen_map and (latest_sim_time - last_seen_map[name]) > DEAD_TIME_LIMIT:
                        if data_robots[name]['xs']: 
                            data_robots[name]['xs'] = []
                            data_robots[name]['ys'] = []

            if args.mode == 2:
                N = max(1, int(args.window))
                for name in list(data_robots.keys()):
                    if len(data_robots[name]['xs']) > N:
                        data_robots[name]['xs'] = data_robots[name]['xs'][-N:]
                        data_robots[name]['ys'] = data_robots[name]['ys'][-N:]

            ax_main.clear()
            ax_main.set_xlim(args.xmin, args.xmax); ax_main.set_ylim(args.ymin, args.ymax)
            ax_main.set_xlabel('X'); ax_main.set_ylabel('Y')
            current_title = get_formatted_title(TITULO_GRAFICA, latest_sim_time, SIM_TICKS_PER_SEC)
            ax_main.set_title(current_title)
            ax_main.grid(True)

            for i, name in enumerate(sorted(data_robots.keys())):
                xs, ys = data_robots[name]['xs'], data_robots[name]['ys']
                display_name, color = get_display_name(name), name_to_color.get(name, get_color(i))
                if not xs or not ys: continue

                if name == OBSERVADOR_ID:
                    if MODO_VISUAL_OBSERVADOR == 0: continue 
                    elif MODO_VISUAL_OBSERVADOR == 1:
                        if xs[-1] is not None: ax_main.scatter([xs[-1]], [ys[-1]], s=25, color=color, label=display_name)
                    elif MODO_VISUAL_OBSERVADOR == 2:
                        ax_main.plot(xs, ys, lw=1.5, label=display_name, color=color)
                        if xs[-1] is not None: ax_main.scatter([xs[-1]], [ys[-1]], s=25, color=color)
                else:
                    ax_main.plot(xs, ys, lw=1.5, label=display_name, color=color)
                    if xs[-1] is not None: ax_main.scatter([xs[-1]], [ys[-1]], s=25, color=color)

            if MODO_VISUAL_OBSERVADOR > 0 and MODO_CIRCULO > 0 and OBSERVADOR_ID in data_robots:
                xs_obs, ys_obs = data_robots[OBSERVADOR_ID]['xs'], data_robots[OBSERVADOR_ID]['ys']
                if xs_obs and ys_obs and xs_obs[-1] is not None:
                    cx, cy = xs_obs[-1], ys_obs[-1]
                    if MODO_CIRCULO == 1:
                        circle = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='--', linewidth=1.5, alpha=0.7)
                        ax_main.add_patch(circle)
                    elif MODO_CIRCULO == 2:
                        ax_main.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=True, alpha=0.15))
                        ax_main.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='-', linewidth=1.5, alpha=0.5))

            draw_labels_clamped(ax_main, data_robots, args.xmin, args.xmax, args.ymin, args.ymax, first_pos_dict=first_seen_positions)
            return []

        # El intervalo en vivo es independiente y dictado por args.live_fps
        interval_ms = int(1000 / max(1, int(args.live_fps)))
        ani = animation.FuncAnimation(fig_main, update, interval=interval_ms, blit=False, cache_frame_data=False)

        try:
            plt.ion()
            plt.show()
            while not stop_event.is_set() and plt.fignum_exists(fig_main.number):
                plt.pause(0.1)
                if args.duration and (time.time() - start_time) > args.duration: stop_event.set(); break
                if args.inactivity_timeout is not None:
                    if (time.time() - last_time_holder[0]) > args.inactivity_timeout: stop_event.set(); break
        except KeyboardInterrupt:
            stop_event.set()

        if plt.fignum_exists(fig_main.number):
            if AUTO_CLOSE_WINDOWS:
                print("Simulación terminada. Cerrando ventana automáticamente...")
                plt.close(fig_main)
            else:
                print("Simulación terminada. Cierra la ventana manualmente para continuar.")
                ani.event_source.stop()
                plt.ioff()
                plt.show()

    if not timeline: return
    names_all = set()
    for frame_events in timeline:
        for (n, x, y, ts) in frame_events: names_all.add(n)

    names_to_plot = sorted([n for n in names_all if n in robots_filter]) if use_filter_robots else sorted(names_all)

    # =========================================================================
    # GUARDADO DE CSV
    # =========================================================================
    if save_csv:
        csv_dir_path = os.path.abspath(CSV_DIR)
        print(f"\n[+] Guardando archivos CSV...")
        print(f"    - Directorio: {csv_dir_path}")
        os.makedirs(csv_dir_path, exist_ok=True)
        samples = {n: [] for n in names_all}

        for frame_events in timeline:
            for (n, x, y, ts) in frame_events: samples[n].append((ts, x, y))

        for n, rows in samples.items():
            if not rows: continue
            safe_name = n.replace(" ", "_")
            csv_path = os.path.join(csv_dir_path, f"{safe_name}.csv")
            try:
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time_iso', 'timestamp', 'x', 'y'])
                    for ts, x, y in rows:
                        writer.writerow([iso(ts), f"{ts:.6f}", f"{x:.6f}", f"{y:.6f}"])
            except Exception as e: 
                print(f" [!] Error guardando CSV para {n}: {e}")
        print(f"    -> CSVs guardados exitosamente.", flush=True)

    # =========================================================================
    # GUARDADO DE VIDEO (Piping a RAM con sincronización Inteligente)
    # =========================================================================
    if save_video:
        video_dir_path = os.path.abspath(VIDEO_DIR)
        os.makedirs(video_dir_path, exist_ok=True)
        video_file_path = os.path.join(video_dir_path, args.outfile)

        total_sim_ticks = len(timeline)

        # -- SIEMPRE extraemos la cantidad de frames dictada por VIDEO_FPS --
        sim_time_per_tick = 1.0 / SIM_TICKS_PER_SEC
        video_frame_interval = 1.0 / args.video_fps

        next_frame_sim_time = 0.0
        current_sim_time = 0.0

        visual_frames = []
        current_visual_frame = []

        for tick_events in timeline:
            current_visual_frame.append(tick_events)
            current_sim_time += sim_time_per_tick

            # Se usa una tolerancia pequeña (+1e-6) por la precisión de coma flotante de Python
            if current_sim_time + 1e-6 >= next_frame_sim_time:
                visual_frames.append(current_visual_frame)
                current_visual_frame = []
                while next_frame_sim_time <= current_sim_time + 1e-6:
                    next_frame_sim_time += video_frame_interval

        if current_visual_frame:
            visual_frames.append(current_visual_frame)

        total_visual_frames = len(visual_frames)

        # --- LÓGICA DE VELOCIDAD DE REPRODUCCIÓN (El corazón de la magia) ---
        if SYNC_VIDEO_DURATION:
            playback_fps = args.video_fps 
        else:
            playback_fps = SIM_TICKS_PER_SEC 

        print(f"\n[+] Iniciando renderizado de Video Ultra-Rápido (Piping a Memoria)...")
        print(f"    - Tiempo base simulación: {SIM_TICKS_PER_SEC} Ticks/seg")
        print(f"    - Cuadros extraídos:      {args.video_fps} FPS (De los datos originales)")

        if SYNC_VIDEO_DURATION:
            print(f"    - Modo de reproducción:   Tiempo Real (Video a {playback_fps} FPS)")
        else:
            if args.video_fps < SIM_TICKS_PER_SEC:
                speed_factor = SIM_TICKS_PER_SEC / args.video_fps
                print(f"    - Modo de reproducción:   Cámara Rápida x{speed_factor:.1f} (Video a {playback_fps} FPS)")
            else:
                speed_factor = args.video_fps / SIM_TICKS_PER_SEC
                print(f"    - Modo de reproducción:   Cámara Lenta x{speed_factor:.1f} (Video a {playback_fps} FPS)")

        print(f"    - Total Ticks detectados: {total_sim_ticks}")
        print(f"    - Imágenes a renderizar:  {total_visual_frames}")
        print(f"    - Archivo destino:        {video_file_path}\n", flush=True)

        try:
            accum = {n: {'xs': [], 'ys': []} for n in names_to_plot}
            vid_last_seen = {} 
            vid_first_seen_positions = {}  
            fallback_ts = 0.0

            # 1. Configuración de la figura en Memoria Pura
            fig_save = Figure(figsize=(7,7), dpi=100)
            canvas = FigureCanvas(fig_save)
            ax_save = fig_save.add_subplot(111)

            fig_save.canvas.draw()
            width, height = fig_save.canvas.get_width_height()

            # 2. Configurar FFmpeg
            ffmpeg_cmd = [
                "ffmpeg", 
                "-y", 
                "-f", "rawvideo",
                "-vcodec", "rawvideo",
                "-s", f"{width}x{height}",
                "-pix_fmt", "rgba",
                "-r", str(playback_fps), # Aquí inyectamos el FPS calculado
                "-i", "-", 
                "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-c:v", "libx264",
                "-pix_fmt", "yuv420p",
                "-crf", "18",
                video_file_path
            ]

            ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            def ffmpeg_monitor():
                for line in ffmpeg_proc.stdout:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if "frame=" in line_str or "fps=" in line_str or "size=" in line_str:
                        sys.stdout.write(f"\r      -> {line_str}")
                        sys.stdout.flush()

            monitor_thread = threading.Thread(target=ffmpeg_monitor, daemon=True)
            monitor_thread.start()

            # 3. Bucle de Renderizado sobre los bloques visuales calculados
            for idx, v_frame in enumerate(visual_frames):
                
                # Buscar el timestamp sim-basado más reciente en este bloque para sincronizar exactamente el timeout con el en vivo
                current_tick_ts = fallback_ts
                for tick_list in v_frame:
                    for ev in tick_list:
                        if ev[3] > current_tick_ts:
                            current_tick_ts = ev[3]
                fallback_ts = current_tick_ts

                # --- IGUALDAD LÓGICA 1: Purgar muertos por inactividad ---
                if DELETE_ON_TIMEOUT:
                    for n in list(accum.keys()):
                        if n in vid_last_seen and (current_tick_ts - vid_last_seen[n]) > DEAD_TIME_LIMIT:
                            accum[n]['xs'] = []
                            accum[n]['ys'] = []

                # --- IGUALDAD LÓGICA 2: Procesamos la matemática interna de los ticks agrupados ---
                for tick_events in v_frame:
                    for (n, x, y, ts) in tick_events:
                        if n in accum:
                            if n not in vid_first_seen_positions:
                                vid_first_seen_positions[n] = (x, y)

                            prev_ts = vid_last_seen.get(n, ts)
                            if (ts - prev_ts) > DEAD_TIME_LIMIT:
                                if DELETE_ON_TIMEOUT:
                                    accum[n]['xs'] = []
                                    accum[n]['ys'] = []
                                else:
                                    accum[n]['xs'].append(None)
                                    accum[n]['ys'].append(None)

                            accum[n]['xs'].append(x)
                            accum[n]['ys'].append(y)
                            vid_last_seen[n] = ts

                # --- IGUALDAD LÓGICA 3: Ventana de tiempo ---
                if args.mode == 2:
                    N = max(1, int(args.window))
                    for n in accum:
                        if len(accum[n]['xs']) > N:
                            accum[n]['xs'] = accum[n]['xs'][-N:]
                            accum[n]['ys'] = accum[n]['ys'][-N:]

                # Limpiar gráfica y preparar el lienzo
                ax_save.clear()
                ax_save.set_xlim(args.xmin, args.xmax)
                ax_save.set_ylim(args.ymin, args.ymax)
                ax_save.set_xlabel('X'); ax_save.set_ylabel('Y')
                current_title = get_formatted_title(TITULO_GRAFICA, current_tick_ts, SIM_TICKS_PER_SEC)
                ax_save.set_title(current_title)
                ax_save.grid(True)

                # Dibujar Robots
                for i, n in enumerate(names_to_plot):
                    xs, ys = accum[n]['xs'], accum[n]['ys']
                    display_name, color = get_display_name(n), get_color(i)
                    if not xs or not ys: continue

                    if n == OBSERVADOR_ID:
                        if MODO_VISUAL_OBSERVADOR == 0: continue 
                        elif MODO_VISUAL_OBSERVADOR == 1:
                            if xs[-1] is not None: ax_save.scatter([xs[-1]], [ys[-1]], s=25, color=color, label=display_name)
                        elif MODO_VISUAL_OBSERVADOR == 2:
                            ax_save.plot(xs, ys, lw=1.5, label=display_name, color=color)
                            if xs[-1] is not None: ax_save.scatter([xs[-1]], [ys[-1]], s=25, color=color)
                    else:
                        ax_save.plot(xs, ys, lw=1.5, label=display_name, color=color)
                        if xs[-1] is not None: ax_save.scatter([xs[-1]], [ys[-1]], s=25, color=color)

                # Dibujar Observador (Círculos)
                if MODO_VISUAL_OBSERVADOR > 0 and MODO_CIRCULO > 0 and OBSERVADOR_ID in accum:
                    xs_obs, ys_obs = accum[OBSERVADOR_ID]['xs'], accum[OBSERVADOR_ID]['ys']
                    if xs_obs and ys_obs and xs_obs[-1] is not None:
                        cx, cy = xs_obs[-1], ys_obs[-1]
                        if MODO_CIRCULO == 1:
                            circle = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='--', linewidth=1.5, alpha=0.7)
                            ax_save.add_patch(circle)
                        elif MODO_CIRCULO == 2:
                            ax_save.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=True, alpha=0.15))
                            ax_save.add_patch(plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='-', linewidth=1.5, alpha=0.5))

                # Dibujar Etiquetas
                sorted_names = sorted(accum.keys())
                x_margin, y_margin = (args.xmax - args.xmin) * 0.02, (args.ymax - args.ymin) * 0.02
                if MODO_ETIQUETAS == 3:
                    for i, name in enumerate(sorted_names):
                        if name not in vid_first_seen_positions: continue
                        if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR == 0: continue
                        if MAX_ETIQUETAS_FIJAS is not None and name != OBSERVADOR_ID:
                            match = re.search(r'(\d+)$', name)
                            if match and int(match.group(1)) > MAX_ETIQUETAS_FIJAS: continue

                        x_val, y_val = vid_first_seen_positions[name]
                        tx_clamped = max(args.xmin + x_margin, min(x_val + x_margin, args.xmax - (x_margin * 6)))
                        ty_clamped = max(args.ymin + y_margin, min(y_val + y_margin, args.ymax - (y_margin * 2)))
                        ax_save.text(tx_clamped, ty_clamped, get_display_name(name), color=get_color(i), fontsize=9, fontweight='bold', zorder=5, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
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
                            ax_save.text(tx_clamped, ty_clamped, get_display_name(name), color=get_color(i), fontsize=9, fontweight='bold', zorder=5, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))

                    if not ELIMINAR_SUPERPUESTAS:
                        for i, name in enumerate(sorted_names):
                            if name == OBSERVADOR_ID and MODO_VISUAL_OBSERVADOR in [0, 1]: continue
                            xs, ys = accum[name]['xs'], accum[name]['ys']
                            if not xs or not ys: continue
                            historical_indices, count = [], 0
                            for hist_idx in range(len(xs) - 2, -1, -1):  
                                if xs[hist_idx] is not None:
                                    historical_indices.append(hist_idx); count += 1
                                    if count >= MAX_HISTORICAS_TRANSPARENTES: break
                            for s_idx, pos in enumerate(historical_indices):
                                tx_clamped = max(args.xmin + x_margin, min(xs[pos] + x_margin, args.xmax - (x_margin * 6)))
                                ty_clamped = max(args.ymin + y_margin, min(ys[pos] + y_margin, args.ymax - (y_margin * 2)))
                                overlap = False
                                for dx, dy in drawn_positions:
                                    if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold: overlap = True; break
                                if overlap: continue  
                                alpha_val = max(0.12, 0.8 - ((s_idx + 1) * (0.7 / MAX_HISTORICAS_TRANSPARENTES)))
                                drawn_positions.append((tx_clamped, ty_clamped))
                                ax_save.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=8, alpha=alpha_val, zorder=4,
                                        bbox=dict(facecolor='white', alpha=0.6 * alpha_val, edgecolor='none', pad=1))

                # --- MAGIA PIPING: Enviar a FFmpeg directo ---
                fig_save.canvas.draw()
                frame_bytes = fig_save.canvas.buffer_rgba()

                try:
                    ffmpeg_proc.stdin.write(frame_bytes)
                except BrokenPipeError:
                    print("\n[!] FFmpeg cerró la conexión inesperadamente.")
                    break

                # Heartbeat
                if (idx + 1) % 5 == 0 or (idx + 1) == total_visual_frames:
                    porcentaje = ((idx + 1) / total_visual_frames) * 100
                    sys.stdout.write(f"\r      -> Renderizando frame visual {idx + 1}/{total_visual_frames} ({porcentaje:.1f}%)")
                    sys.stdout.flush() 

                if (idx + 1) % 50 == 0:
                    gc.collect()

            print() 
            print("    -> Generación terminada. Finalizando archivo de video...", flush=True)

            ffmpeg_proc.stdin.close() 
            ffmpeg_proc.wait()        
            monitor_thread.join()     

            del ax_save
            del canvas
            del fig_save
            gc.collect()

            print(f"\n[+] ¡Video guardado exitosamente!", flush=True)

        except Exception as e:
            print(f"\n[!] ERROR CRÍTICO AL GUARDAR EL VIDEO:\n    {e}", flush=True)

if __name__ == '__main__':
    main()