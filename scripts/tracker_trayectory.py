#!/usr/bin/env python3
"""
tracker_trayectoria.py

- controllers -> all

- Maneja exclusivamente la visualización de trayectorias de los robots.
- Ignora los datos de varianza o de 'maestros'.
- Cierre automático o manual de ventanas al detectar el patrón de fin.
"""

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Ventana Principal y Gráficas ---
TITULO_GRAFICA                  = "Datos observados" # Título de la ventana y de la gráfica
MODO                            = 1                 # 1: Mostrar toda la trayectoria, 2: Mostrar solo las últimas N muestras    
WINDOW                          = 300               # (Aplica si MODO=2) Número de muestras a visualizar
DEFAULT_XMIN                    = -1.5              # Límites espaciales de la gráfica
DEFAULT_XMAX                    =  1.5
DEFAULT_YMIN                    = -1.5
DEFAULT_YMAX                    =  1.5
DEAD_TIME_LIMIT                 = 0.5               # Tiempo (s) para considerar que no hay actualización de datos de un determinado robot (inactividad). 
DELETE_ON_TIMEOUT               = False             # True: Borra trayectoria completa tras inactividad. False: Inserta None (discontinuidad).
DEFAULT_FPS                     = 60                # Tasa de refresco de la animación

# --- Etiquetas y Nombres ---
MODO_ETIQUETAS                  = 3                 # 0: Leyenda, 1: Flotantes junto al robot, 3: Fijas en punto de inicio
PREFIX_ETIQUETAS                = "Rm"              # Prefijo para los robots (ej: "Rm" -> Rm1). Si es "", usa original.
NOMBRE_OBSERVADOR               = "Ra"              # Nombre especial para la etiqueta del observador.
MAX_ETIQUETAS_FIJAS             = 7                 # (Aplica si MODO_ETIQUETAS=3) Límite numérico: Ver Rm1 hasta Rm7. Usa None para sin límite.
ELIMINAR_SUPERPUESTAS           = True              # (Aplica si MODO_ETIQUETAS=1) Elimina etiquetas encimadas. 
MAX_HISTORICAS_TRANSPARENTES    = 5                 # (Aplica si MODO_ETIQUETAS=1 y ELIMINAR=False) Cantidad de etiquetas viejas transparentes.

# --- Representación del Observador (Cámara) ---
MODO_CIRCULO                    = 0                 # 0: Ninguno, 1: Contorno, 2: Relleno semitransparente
ANGULO_APERTURA                 = 73.935            # (Aplica si MODO_CIRCULO>0) Grados (omnidirectional_camera_aperture)
ALTURA_CAMARA                   = 0.3               # (Aplica si MODO_CIRCULO>0) Altura típica de la cámara (en metros)


# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
DATA_DIR                        ="Resultados/Trayectory"# Directorio base para el guardado
# --- Video ---
SAVE_VIDEO  = False                                 # True: Guarda la animación en formato de video
DEFAULT_OUTFILE                 = "trayectoria.mp4" # (Aplica si SAVE_VIDEO=True) Nombre del archivo de salida
# --- CSV ---
SAVE_CSV                        = False             # True: Guarda las coordenadas en un archivo tabular

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
# --- Identificadores y Filtros ---
OBSERVADOR_ID                   = "Qupa400"         # ID del robot observador (Ajusta según tu parser)
FILTRO                          = False             # True: Muestra solo los robots especificados abajo
ROBOTS                          = "1"               # (Aplica si FILTRO=True) IDs a visualizar

# --- Control de Finalización ---
DEFAULT_END_PATTERN             = "End_simulation"  # Frase en consola que finaliza la ejecución
AUTO_CLOSE_WINDOWS              = False             # True: Cierra la gráfica automáticamente al terminar
AUTO_CLOSE_TIME                 = None              # Segundos sin datos para forzar el cierre


import os
import sys
import argparse
import threading
import queue
import re
import time
import math
from collections import defaultdict
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- CÁLCULO DEL RADIO DE VISIÓN ---
RADIO_VISION = ALTURA_CAMARA * math.tan(math.radians(ANGULO_APERTURA))

# --- EXPRESIONES REGULARES ESTRICTAS ---
NUM_PATTERN = r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?'
STRICT_PAIR_RE = re.compile(rf'^\s*({NUM_PATTERN})\s*[,]?\s*({NUM_PATTERN})\s*$')

def parse_line(line):
    if '&' not in line: return None
    
    parts = line.split('&', 1)
    name_raw = parts[0].strip()
    
    match = STRICT_PAIR_RE.match(parts[1])
    if not match:
        return None
        
    name_clean = re.sub(r'^\[.*?\]\s*', '', name_raw)
    if name_clean.lower().startswith("maestro"):
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
        if p.isdigit(): names.add(f"Qupa{int(p)}") # CORREGIDO: Ahora usa Qupa en lugar de Footbot
        else: names.add(p)
    return names

def stdin_reader(q, timeline, stop_event, end_pattern, last_time_holder):
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
                timeline.append([(name, val1, val2, ts)])
                last_time_holder[0] = ts
    except Exception:
        pass
    finally:
        stop_event.set()

def get_color(i, cmap='tab20'):
    return plt.get_cmap(cmap)(i % plt.get_cmap(cmap).N)

def iso(ts):
    return datetime.fromtimestamp(ts).isoformat(sep=' ', timespec='milliseconds')

def get_display_name(name):
    """Retorna el nombre personalizado según la configuración de etiquetas."""
    if name == OBSERVADOR_ID and NOMBRE_OBSERVADOR != "":
        return NOMBRE_OBSERVADOR
        
    if PREFIX_ETIQUETAS != "":
        # Extraemos solo el número al final del nombre original (ej: Qupa14 -> 14)
        match = re.search(r'(\d+)$', name)
        if match:
            return f"{PREFIX_ETIQUETAS}{match.group(1)}"
        else:
            return f"{PREFIX_ETIQUETAS}_{name}"
            
    return name

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xmin', type=float, default=DEFAULT_XMIN)
    parser.add_argument('--xmax', type=float, default=DEFAULT_XMAX)
    parser.add_argument('--ymin', type=float, default=DEFAULT_YMIN)
    parser.add_argument('--ymax', type=float, default=DEFAULT_YMAX)
    parser.add_argument('--fps', type=int, default=DEFAULT_FPS)
    parser.add_argument('--outfile', type=str, default=DEFAULT_OUTFILE)
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--mode', type=int, choices=[1,2], default=MODO)
    parser.add_argument('--window', type=int, default=WINDOW)
    parser.add_argument('--robots', type=str, default=None)
    parser.add_argument('--end-pattern', type=str, default=DEFAULT_END_PATTERN)
    parser.add_argument('--inactivity-timeout', type=float, default=AUTO_CLOSE_TIME)
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--no-csv', action='store_true')
    args = parser.parse_args()

    robots_filter = parse_robots_arg(args.robots) if args.robots else (parse_robots_arg(ROBOTS) if FILTRO else None)
    use_filter_robots = True if robots_filter else False
    
    save_video = False if args.no_video else SAVE_VIDEO
    save_csv = False if args.no_csv else SAVE_CSV

    q = queue.Queue()
    timeline = []
    stop_event = threading.Event()
    last_time_holder = [time.time()]
    start_time = time.time()

    reader_thread = threading.Thread(target=stdin_reader, args=(q, timeline, stop_event, args.end_pattern, last_time_holder), daemon=True)
    reader_thread.start()

    data_robots = defaultdict(lambda: {'xs': [], 'ys': []})
    name_to_color = {}
    last_seen_map = {} 
    first_seen_positions = {}  # Guarda la coordenada de origen de cada robot
    next_index = 0
    
    fig_main, ax_main = plt.subplots(figsize=(7,7))
    mode_str = "Eliminar tras DeadTime" if DELETE_ON_TIMEOUT else "Romper línea tras DeadTime"
    fig_main.canvas.manager.set_window_title(TITULO_GRAFICA) # CORREGIDO: Título de ventana dinámico

    def draw_labels_clamped(ax, data_dict, xmin, xmax, ymin, ymax, is_video=False, first_pos_dict=None):
        """Dibuja etiquetas flotantes, gestiona personalización de nombres y modo de inicio fijo."""
        if not data_dict: return
        
        if MODO_ETIQUETAS == 0:
            ax.legend(loc='upper right', fontsize='small')
            return

        sorted_names = sorted(data_dict.keys())
        x_margin = (xmax - xmin) * 0.02
        y_margin = (ymax - ymin) * 0.02

        # =====================================================================
        # MODO 3: ETIQUETAS FIJAS EN EL PUNTO DE INICIO
        # =====================================================================
        if MODO_ETIQUETAS == 3:
            if not first_pos_dict: return
            
            for i, name in enumerate(sorted_names):
                if name not in first_pos_dict: continue
                
                # --- NUEVA LÓGICA: FILTRO MATEMÁTICO ESTRICTO POR NÚMERO ---
                if MAX_ETIQUETAS_FIJAS is not None and name != OBSERVADOR_ID:
                    match = re.search(r'(\d+)$', name)
                    if match:
                        num_robot = int(match.group(1))
                        # Si el número (ej: 12) supera el límite (ej: 7), ignoramos dibujar esta etiqueta
                        if num_robot > MAX_ETIQUETAS_FIJAS:
                            continue
                
                x_val, y_val = first_pos_dict[name]
                display_name = get_display_name(name)
                color = get_color(i) if is_video else name_to_color.get(name, get_color(i))

                tx = x_val + x_margin
                ty = y_val + y_margin
                
                tx_clamped = max(xmin + x_margin, min(tx, xmax - (x_margin * 6)))
                ty_clamped = max(ymin + y_margin, min(ty, ymax - (y_margin * 2)))

                # Las dibujamos estáticas (solo aquellas que pasaron el filtro)
                ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=9, fontweight='bold', alpha=1.0, zorder=5,
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
            return

        # =====================================================================
        # MODO 1: ETIQUETAS FLOTANTES JUNTO AL ROBOT (ACTUALIZADAS)
        # =====================================================================
        x_threshold = (xmax - xmin) * 0.06  
        y_threshold = (ymax - ymin) * 0.04  
        drawn_positions = []  

        # PASO 1: Dibujar PRIMERO las etiquetas de los últimos datos
        for i, name in enumerate(sorted_names):
            xs = data_dict[name]['xs']
            ys = data_dict[name]['ys']
            display_name = get_display_name(name)
            color = get_color(i) if is_video else name_to_color.get(name, get_color(i))

            if xs and ys and xs[-1] is not None:
                x_val, y_val = xs[-1], ys[-1]
                
                tx = x_val + x_margin
                ty = y_val + y_margin
                
                tx_clamped = max(xmin + x_margin, min(tx, xmax - (x_margin * 6)))
                ty_clamped = max(ymin + y_margin, min(ty, ymax - (y_margin * 2)))

                overlap = False
                for dx, dy in drawn_positions:
                    if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold:
                        overlap = True
                        break
                
                if overlap and ELIMINAR_SUPERPUESTAS:
                    continue  

                drawn_positions.append((tx_clamped, ty_clamped))

                ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=9, fontweight='bold', alpha=1.0, zorder=5,
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))

        # PASO 2: Dibujar las etiquetas históricas (Rastro)
        if not ELIMINAR_SUPERPUESTAS:
            for i, name in enumerate(sorted_names):
                xs = data_dict[name]['xs']
                ys = data_dict[name]['ys']
                display_name = get_display_name(name)
                color = get_color(i) if is_video else name_to_color.get(name, get_color(i))

                if not xs or not ys: continue

                historical_indices = []
                count = 0
                for idx in range(len(xs) - 2, -1, -1):  
                    if xs[idx] is not None:
                        historical_indices.append(idx)
                        count += 1
                        if count >= MAX_HISTORICAS_TRANSPARENTES:
                            break

                for s_idx, pos in enumerate(historical_indices):
                    x_val, y_val = xs[pos], ys[pos]
                    
                    tx = x_val + x_margin
                    ty = y_val + y_margin
                    
                    tx_clamped = max(xmin + x_margin, min(tx, xmax - (x_margin * 6)))
                    ty_clamped = max(ymin + y_margin, min(ty, ymax - (y_margin * 2)))

                    overlap = False
                    for dx, dy in drawn_positions:
                        if abs(tx_clamped - dx) < x_threshold and abs(ty_clamped - dy) < y_threshold:
                            overlap = True
                            break

                    if overlap:
                        continue  

                    distance_factor = s_idx + 1
                    alpha_val = max(0.12, 0.8 - (distance_factor * (0.7 / MAX_HISTORICAS_TRANSPARENTES)))

                    drawn_positions.append((tx_clamped, ty_clamped))

                    ax.text(tx_clamped, ty_clamped, display_name, color=color, fontsize=8, fontweight='normal', alpha=alpha_val, zorder=4,
                            bbox=dict(facecolor='white', alpha=0.6 * alpha_val, edgecolor='none', pad=1))

    def update(frame):
        nonlocal next_index
        current_time = time.time()

        if DELETE_ON_TIMEOUT:
            for name in list(data_robots.keys()):
                if name in last_seen_map:
                    if (current_time - last_seen_map[name]) > DEAD_TIME_LIMIT:
                        if data_robots[name]['xs']: 
                            data_robots[name]['xs'] = []
                            data_robots[name]['ys'] = []

        while True:
            try:
                category, name, val1, val2, ts = q.get_nowait()
            except queue.Empty:
                break
            
            if use_filter_robots and name not in robots_filter:
                continue

            # Registramos la posición inicial absoluta si el robot es nuevo
            if name not in first_seen_positions:
                first_seen_positions[name] = (val1, val2)

            if name not in data_robots:
                data_robots[name]['xs'] = []
                data_robots[name]['ys'] = []
                name_to_color[name] = get_color(next_index)
                next_index += 1
                last_seen_map[name] = ts 
            
            prev_ts = last_seen_map.get(name, ts)
            dt = ts - prev_ts

            if dt > DEAD_TIME_LIMIT:
                if DELETE_ON_TIMEOUT:
                    data_robots[name]['xs'] = []
                    data_robots[name]['ys'] = []
                else:
                    data_robots[name]['xs'].append(None)
                    data_robots[name]['ys'].append(None)

            data_robots[name]['xs'].append(val1)  
            data_robots[name]['ys'].append(val2)  
            last_seen_map[name] = ts 

        if args.mode == 2:
            N = max(1, int(args.window))
            for name in list(data_robots.keys()):
                if len(data_robots[name]['xs']) > N:
                    data_robots[name]['xs'] = data_robots[name]['xs'][-N:]
                    data_robots[name]['ys'] = data_robots[name]['ys'][-N:]

        ax_main.clear()
        ax_main.set_xlim(args.xmin, args.xmax)
        ax_main.set_ylim(args.ymin, args.ymax)
        ax_main.set_xlabel('X')
        ax_main.set_ylabel('Y')
        ax_main.set_title(TITULO_GRAFICA) # CORREGIDO: Título principal de la gráfica dinámica
        ax_main.grid(True)

        for i, name in enumerate(sorted(data_robots.keys())):
            xs = data_robots[name]['xs']
            ys = data_robots[name]['ys']
            display_name = get_display_name(name)
            color = name_to_color.get(name, get_color(i))

            if xs and ys:
                ax_main.plot(xs, ys, lw=1.5, label=display_name, color=color)
                if xs[-1] is not None:
                    ax_main.scatter([xs[-1]], [ys[-1]], s=25, color=color)
        
        # --- LÓGICA DEL CIRCULO DEL OBSERVADOR ---
        if MODO_CIRCULO > 0 and OBSERVADOR_ID in data_robots:
            xs_obs = data_robots[OBSERVADOR_ID]['xs']
            ys_obs = data_robots[OBSERVADOR_ID]['ys']
            if xs_obs and ys_obs and xs_obs[-1] is not None:
                cx, cy = xs_obs[-1], ys_obs[-1]
                
                if MODO_CIRCULO == 1:
                    circle = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='--', linewidth=1.5, alpha=0.7)
                    ax_main.add_patch(circle)
                elif MODO_CIRCULO == 2:
                    circle_fill = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=True, alpha=0.15)
                    circle_line = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='-', linewidth=1.5, alpha=0.5)
                    ax_main.add_patch(circle_fill)
                    ax_main.add_patch(circle_line)
                
        # Llamamos a nuestra función pasando las posiciones iniciales
        draw_labels_clamped(ax_main, data_robots, args.xmin, args.xmax, args.ymin, args.ymax, first_pos_dict=first_seen_positions)

        return []

    interval_ms = int(1000 / max(1, int(args.fps)))
    ani = animation.FuncAnimation(fig_main, update, interval=interval_ms, blit=False, cache_frame_data=False)

    try:
        plt.ion()
        plt.show()
        while not stop_event.is_set() and plt.fignum_exists(fig_main.number):
            plt.pause(0.1)
            if args.duration and (time.time() - start_time) > args.duration:
                stop_event.set()
                break
            if args.inactivity_timeout is not None:
                if (time.time() - last_time_holder[0]) > args.inactivity_timeout:
                    stop_event.set()
                    break
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
        for (n, x, y, ts) in frame_events:
            names_all.add(n)

    if use_filter_robots:
        names_to_plot = sorted([n for n in names_all if n in robots_filter])
    else:
        names_to_plot = sorted(names_all)

    # Los CSV seguirán guardándose con sus nombres limpios originales para no alterar la data
    if save_csv:
        print("\nGuardando CSV...")
        data_dir = os.path.join(os.getcwd(), DATA_DIR)
        os.makedirs(data_dir, exist_ok=True)
        samples = {n: [] for n in names_all}
        for frame_events in timeline:
            for (n, x, y, ts) in frame_events:
                samples[n].append((ts, x, y))
        for n, rows in samples.items():
            if not rows: continue
            safe_name = n.replace(" ", "_")
            csv_path = os.path.join(data_dir, f"{safe_name}.csv")
            try:
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time_iso', 'timestamp', 'x', 'y'])
                    for ts, x, y in rows:
                        writer.writerow([iso(ts), f"{ts:.6f}", f"{x:.6f}", f"{y:.6f}"])
            except Exception: pass

    if save_video:
        print("Guardando Video...")
        fig_save, ax_save = plt.subplots(figsize=(7,7))
        Writer = animation.FFMpegWriter
        writer = Writer(fps=args.fps, metadata=dict(artist='tracker'), bitrate=2000)
        try:
            with writer.saving(fig_save, args.outfile, dpi=100):
                accum = {n: {'xs': [], 'ys': []} for n in names_to_plot}
                vid_last_seen = {} 
                vid_first_seen_positions = {}  
                
                for frame_events in timeline:
                    for (n, x, y, ts) in frame_events:
                        if n in accum:
                            # Registramos coordenada cero para el export de video
                            if n not in vid_first_seen_positions:
                                vid_first_seen_positions[n] = (x, y)

                            if not DELETE_ON_TIMEOUT:
                                prev_ts = vid_last_seen.get(n, ts)
                                if (ts - prev_ts) > DEAD_TIME_LIMIT:
                                    accum[n]['xs'].append(None)
                                    accum[n]['ys'].append(None)
                            accum[n]['xs'].append(x)
                            accum[n]['ys'].append(y)
                            vid_last_seen[n] = ts

                    if args.mode == 2:
                        N = max(1, int(args.window))
                        for n in accum:
                            if len(accum[n]['xs']) > N:
                                accum[n]['xs'] = accum[n]['xs'][-N:]
                                accum[n]['ys'] = accum[n]['ys'][-N:]

                    ax_save.clear()
                    ax_save.set_xlim(args.xmin, args.xmax)
                    ax_save.set_ylim(args.ymin, args.ymax)
                    ax_save.grid(True)
                    for i, n in enumerate(names_to_plot):
                        xs = accum[n]['xs']
                        ys = accum[n]['ys']
                        display_name = get_display_name(n)
                        color = get_color(i)
                        
                        if xs and ys:
                            ax_save.plot(xs, ys, lw=1.5, label=display_name, color=color)
                            if xs[-1] is not None:
                                ax_save.scatter([xs[-1]], [ys[-1]], s=25, color=color)
                    
                    # --- LÓGICA DEL CÍRCULO EN EL VIDEO ---
                    if MODO_CIRCULO > 0 and OBSERVADOR_ID in accum:
                        xs_obs = accum[OBSERVADOR_ID]['xs']
                        ys_obs = accum[OBSERVADOR_ID]['ys']
                        if xs_obs and ys_obs and xs_obs[-1] is not None:
                            cx, cy = xs_obs[-1], ys_obs[-1]
                            if MODO_CIRCULO == 1:
                                circle = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='--', linewidth=1.5, alpha=0.7)
                                ax_save.add_patch(circle)
                            elif MODO_CIRCULO == 2:
                                circle_fill = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=True, alpha=0.15)
                                circle_line = plt.Circle((cx, cy), RADIO_VISION, color='blue', fill=False, linestyle='-', linewidth=1.5, alpha=0.5)
                                ax_save.add_patch(circle_fill)
                                ax_save.add_patch(circle_line)

                    # Exportando texto al video
                    draw_labels_clamped(ax_save, accum, args.xmin, args.xmax, args.ymin, args.ymax, is_video=True, first_pos_dict=vid_first_seen_positions)

                    writer.grab_frame()
        except Exception:
            pass
        finally:
            plt.close(fig_save)
            print("Video guardado exitosamente.")

if __name__ == '__main__':
    main()