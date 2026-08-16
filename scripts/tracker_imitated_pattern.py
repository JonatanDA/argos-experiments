#!/usr/bin/env python3
"""
tracker_imitated_pattern.py

- controllers: St10 -> Qupa - Footbot

- Visualiza secuencias de patrones de movimiento (aprendizaje social/imitación).
- Mantiene el PRIMER patrón global como "Original".
- Gestiona una cola con límite para los intentos de imitación de todos los robots.
- Muestra un contador de imitaciones en el título.
- Guarda los datos en CSV y renderiza un video MP4 al finalizar.
"""

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Gráfica de Imitación Histórica ---
HISTORY_LIMIT = "All"           # Número de imitaciones GLOBALES a mostrar o "ALL"
LINE_WIDTH    = 2.0             # Grosor de la línea (menor = más delgada)
SHOW_MARKERS  = False           # True: Muestra puntos en las esquinas e inicio. False: Solo líneas.

# --- Límites y Animación ---
DEFAULT_XMIN = -5               # Límites espaciales de la gráfica
DEFAULT_XMAX =  5
DEFAULT_YMIN = -5
DEFAULT_YMAX =  5
DEFAULT_FPS  = 60               # Tasa de refresco de la animación

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
# --- Video ---
SAVE_VIDEO    = True
VIDEO_DIR     = "Resultados/videos"
VIDEO_NAME    = "patrones_imitacion.mp4"
# --- CSV ---
SAVE_CSV      = True
CSV_DIR       = "Resultados/Data"
CSV_NAME      = "patrones_imitacion.csv"

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
DEFAULT_END_PATTERN = "End_simulation" # Frase en consola que finaliza la ejecución
DEFAULT_INACTIVITY_TIMEOUT = None      # Segundos sin datos para forzar el cierre
AUTO_CLOSE_WINDOWS = False             # True: Cierra las ventanas automáticamente al terminar

import os
import sys
import argparse
import threading
import queue
import re
import time
import csv
from datetime import datetime
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib.animation as animation

NUMBER_RE = re.compile(r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?')

def parse_line(line):
    if '&' not in line: 
        return None
        
    parts = line.split('&', 1)
    name_raw = parts[0].strip()
    name_clean = re.sub(r'^\[.*?\]\s*', '', name_raw)
    
    if name_clean.lower().startswith("maestro"):
        return None
        
    tuples_raw = re.findall(r'\((.*?)\)', parts[1])
    points = []
    for t_raw in tuples_raw:
        nums = NUMBER_RE.findall(t_raw)
        if len(nums) >= 2:
            try:
                x = float(nums[0].replace(',', '.'))
                y = float(nums[1].replace(',', '.'))
                points.append((x, y))
            except ValueError:
                continue
                
    if points:
        return name_clean, points
    return None

def stdin_reader(q, timeline, stop_event, end_pattern, last_time_holder):
    try:
        while True:
            raw = sys.stdin.readline()
            if raw == '': 
                break
            
            line = raw.strip()
            if not line: 
                continue
                
            if end_pattern and end_pattern in line:
                stop_event.set()
                break
            
            parsed = parse_line(line)
            if parsed:
                name, points = parsed
                ts = time.time()
                q.put((name, points, ts))
                timeline.append((name, points, ts)) 
                last_time_holder[0] = ts
    except Exception:
        pass
    finally:
        stop_event.set()

def get_color(i, cmap='tab10'):
    return plt.get_cmap(cmap)(i % plt.get_cmap(cmap).N)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xmin', type=float, default=DEFAULT_XMIN)
    parser.add_argument('--xmax', type=float, default=DEFAULT_XMAX)
    parser.add_argument('--ymin', type=float, default=DEFAULT_YMIN)
    parser.add_argument('--ymax', type=float, default=DEFAULT_YMAX)
    parser.add_argument('--fps', type=int, default=DEFAULT_FPS)
    parser.add_argument('--end-pattern', type=str, default=DEFAULT_END_PATTERN)
    parser.add_argument('--inactivity-timeout', type=float, default=DEFAULT_INACTIVITY_TIMEOUT)
    args = parser.parse_args()

    q = queue.Queue()
    timeline = []
    stop_event = threading.Event()
    last_time_holder = [time.time()]

    reader_thread = threading.Thread(
        target=stdin_reader, 
        args=(q, timeline, stop_event, args.end_pattern, last_time_holder), 
        daemon=True
    )
    reader_thread.start()

    # Variables para controlar el flujo global de patrones
    global_original = None
    imitations_history = [] 
    name_to_color = {}
    next_index = 0
    
    fig_main, ax_main = plt.subplots(figsize=(7,7))
    fig_main.canvas.manager.set_window_title('Patrones Imitados (Aprendizaje Social)')

    def update(frame):
        nonlocal next_index, global_original
        
        while True:
            try:
                name, points, ts = q.get_nowait()
            except queue.Empty:
                break
                
            if name not in name_to_color:
                name_to_color[name] = get_color(next_index)
                next_index += 1
                
            # Lógica central: El primero es el original, el resto son imitaciones
            if global_original is None:
                global_original = (name, points)
            else:
                imitations_history.append((name, points))

        ax_main.clear()
        ax_main.set_xlim(args.xmin, args.xmax)
        ax_main.set_ylim(args.ymin, args.ymax)
        ax_main.set_xlabel('X')
        ax_main.set_ylabel('Y')
        
        # --- MODIFICACIÓN: Contador en el título (Vivo) ---
        total_imitaciones = len(imitations_history)
        ax_main.set_title(f'Patrones de Movimiento (Imitaciones: {total_imitaciones} | Límite: {HISTORY_LIMIT})')
        
        ax_main.grid(True)
        
        m_style = 'o' if SHOW_MARKERS else ''

        # 1. GRAFICAR EL PATRÓN ORIGINAL (El primero absoluto)
        if global_original is not None:
            orig_name, orig_points = global_original
            c_orig = name_to_color[orig_name]
            xs_orig = [p[0] for p in orig_points]
            ys_orig = [p[1] for p in orig_points]
            
            ax_main.plot(xs_orig, ys_orig, marker=m_style, linestyle='-', lw=LINE_WIDTH * 2, 
                         label=f'Original ({orig_name})', color=c_orig, alpha=1.0, markersize=4)
            if SHOW_MARKERS:
                ax_main.scatter([xs_orig[0]], [ys_orig[0]], s=80, facecolors='none', edgecolors=c_orig, linewidth=2)

        # 2. GRAFICAR LAS IMITACIONES (Aplicando el límite global)
        if str(HISTORY_LIMIT).upper() == "ALL":
            recent_imitations = imitations_history
        else:
            try:
                limit = int(HISTORY_LIMIT)
                recent_imitations = imitations_history[-limit:] if imitations_history else []
            except ValueError:
                recent_imitations = imitations_history

        # Diccionario temporal para etiquetar solo la última imitación de cada robot en la leyenda
        seen_in_legend = set()
        
        for name, points in recent_imitations:
            c = name_to_color[name]
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            
            lbl = None
            if name not in seen_in_legend:
                lbl = f'Imitación ({name})'
                seen_in_legend.add(name)
                
            ax_main.plot(xs, ys, marker=m_style, linestyle='-', lw=LINE_WIDTH, 
                         color=c, alpha=0.4, label=lbl, markersize=3)
            if SHOW_MARKERS:
                ax_main.scatter([xs[0]], [ys[0]], s=30, facecolors='none', edgecolors=c, linewidth=1, alpha=0.4)

        if global_original is not None:
            # Reorganizar leyenda para que no se repitan nombres innecesariamente
            handles, labels = ax_main.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax_main.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize='small')

        return []

    interval_ms = int(1000 / max(1, int(args.fps)))
    ani = animation.FuncAnimation(fig_main, update, interval=interval_ms, blit=False, cache_frame_data=False)

    try:
        plt.ion()
        plt.show()
        while not stop_event.is_set() and plt.fignum_exists(fig_main.number):
            plt.pause(0.1)
            if args.inactivity_timeout is not None:
                if (time.time() - last_time_holder[0]) > args.inactivity_timeout:
                    stop_event.set()
                    break
    except KeyboardInterrupt:
        stop_event.set()
    
    if plt.fignum_exists(fig_main.number):
        if AUTO_CLOSE_WINDOWS:
            plt.close(fig_main)
        else:
            ani.event_source.stop()
            plt.ioff()
            plt.show()

    # ================= GUARDADO DE DATOS AL FINALIZAR =================
    if not timeline:
        print("No se registraron datos para guardar.")
        return

    # GUARDAR CSV
    if SAVE_CSV:
        print("\n[Gestor de Datos] Guardando CSV...")
        os.makedirs(CSV_DIR, exist_ok=True)
        csv_path = os.path.join(CSV_DIR, CSV_NAME)
        
        try:
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['time_iso', 'timestamp', 'robot', 'tipo', 'intento_num', 'punto_index', 'x', 'y'])
                
                attempts_tracker = defaultdict(int)
                for idx, (name, points, ts) in enumerate(timeline):
                    attempts_tracker[name] += 1
                    intento_num = attempts_tracker[name]
                    
                    # El índice 0 de la línea de tiempo es el maestro absoluto
                    tipo = "Original" if idx == 0 else "Imitacion"
                    iso_time = datetime.fromtimestamp(ts).isoformat(sep=' ', timespec='milliseconds')
                    
                    for p_idx, (x, y) in enumerate(points):
                        writer.writerow([iso_time, f"{ts:.6f}", name, tipo, intento_num, p_idx, f"{x:.6f}", f"{y:.6f}"])
            print(f"-> CSV guardado exitosamente en: {csv_path}")
        except Exception as e:
            print(f"Error guardando CSV: {e}")

    # GUARDAR VIDEO
    if SAVE_VIDEO:
        print("\n[Gestor de Datos] Renderizando Video (MP4)...")
        os.makedirs(VIDEO_DIR, exist_ok=True)
        vid_path = os.path.join(VIDEO_DIR, VIDEO_NAME)
        
        fig_save, ax_save = plt.subplots(figsize=(7,7))
        Writer = animation.FFMpegWriter
        writer = Writer(fps=args.fps, metadata=dict(artist='ARGoS_Tracker'), bitrate=2000)
        
        try:
            with writer.saving(fig_save, vid_path, dpi=100):
                vid_global_orig = None
                vid_imitations = []
                
                for idx, (name, points, ts) in enumerate(timeline):
                    if idx == 0:
                        vid_global_orig = (name, points)
                    else:
                        vid_imitations.append((name, points))
                        
                    ax_save.clear()
                    ax_save.set_xlim(args.xmin, args.xmax)
                    ax_save.set_ylim(args.ymin, args.ymax)
                    ax_save.grid(True)
                    
                    # --- MODIFICACIÓN: Contador en el título (Video) ---
                    total_imitaciones_vid = len(vid_imitations)
                    ax_save.set_title(f'Historial de Imitación (Imitaciones: {total_imitaciones_vid})')
                    
                    m_style = 'o' if SHOW_MARKERS else ''
                    
                    # Graficar el Original en el frame actual
                    if vid_global_orig:
                        o_name, o_points = vid_global_orig
                        c_orig = name_to_color.get(o_name, 'blue')
                        xs_o = [p[0] for p in o_points]
                        ys_o = [p[1] for p in o_points]
                        ax_save.plot(xs_o, ys_o, marker=m_style, linestyle='-', lw=LINE_WIDTH*2, label=f'Orig ({o_name})', color=c_orig)

                    # Graficar imitaciones en el frame actual (respetando el límite)
                    if str(HISTORY_LIMIT).upper() == "ALL":
                        rec_vid = vid_imitations
                    else:
                        try:
                            limit = int(HISTORY_LIMIT)
                            rec_vid = vid_imitations[-limit:] if vid_imitations else []
                        except ValueError:
                            rec_vid = vid_imitations

                    for im_name, im_points in rec_vid:
                        c_im = name_to_color.get(im_name, 'blue')
                        xs_i = [p[0] for p in im_points]
                        ys_i = [p[1] for p in im_points]
                        ax_save.plot(xs_i, ys_i, marker=m_style, linestyle='-', lw=LINE_WIDTH, color=c_im, alpha=0.4)

                    writer.grab_frame()
            print(f"-> Video guardado exitosamente en: {vid_path}")
        except Exception as e:
            print(f"Error guardando Video: {e}\n(Asegúrate de tener FFmpeg instalado en tu sistema)")
        finally:
            plt.close(fig_save)

if __name__ == '__main__':
    main()