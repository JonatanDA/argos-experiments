#!/usr/bin/env python3
"""
tracker_AL_PID.py

Este script grafica en tiempo real las señales del controlador PID.
Soporta múltiples señales de control enviadas desde Lua con el formato:
[PID];target;actual;var1=valor1;var2=valor2;...
"""

import os
import sys
import threading
import queue
import re
import time
import subprocess
import gc
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

# =============================================================================
# ---------- CONFIGURACIÓN GENERAL ----------
# =============================================================================

TITULO_GRAFICA      = "Análisis del Controlador PID - Active Learning"
DEFAULT_END_PATTERN = "End_simulation"
SIM_TICKS_PER_SEC   = 10
LIVE_FPS            = 10

# Visibilidad de señales
VER_ENTRADA = True  # Target / Referencia
VER_SALIDA  = True  # Actual / Variable de Proceso
VER_CONTROL = True  # Output / Señales de Control dinámicas

# Colores fijos (Los controles dinámicos usarán la paleta por defecto de matplotlib)
COLOR_ENTRADA = 'green'     
COLOR_SALIDA  = 'blue'      

# Guardado
DIR_BASE       = "Resultados/Robots/Qupa/Ks4/PID"
GUARDAR_VIDEO  = False
VIDEO_DIR      = f"{DIR_BASE}/Video"
OUTFILE_VIDEO  = "PID_Ks4.mp4"
VIDEO_FPS      = 10

GUARDAR_IMAGEN = True
IMAGEN_DIR     = f"{DIR_BASE}/Imagen"
OUTFILE_IMAGEN = "Grafica_PID_Ks4.png"

GUARDAR_CSV    = True
CSV_DIR        = f"{DIR_BASE}/Data"
OUTFILE_CSV    = "Datos_PID_Ks4.csv"

# =============================================================================

def parse_pid_line(line):
    """ Extrae los datos con soporte para N variables dinámicas. """
    line_str = line.strip()
    if line_str.startswith("[PID];"):
        parts = line_str.split(';')
        if len(parts) >= 3:
            try:
                target = float(parts[1].replace(',', '.'))
                actual = float(parts[2].replace(',', '.'))
                
                # Extraemos las variables dinámicas de control
                control_signals = {}
                for ctrl_part in parts[3:]:
                    if '=' in ctrl_part:
                        name, val_str = ctrl_part.split('=')
                        control_signals[name.strip()] = float(val_str.replace(',', '.'))
                        
                return target, actual, control_signals
            except ValueError:
                pass
    return None

def stdin_reader(q, timeline, stop_event, end_pattern):
    tick_counter = 0
    try:
        while True:
            raw = sys.stdin.readline()
            if raw == '': break
            line = raw.strip()
            if not line: continue

            if end_pattern and end_pattern in line:
                stop_event.set()
                break

            # Sincronización del tiempo
            if line.lower().startswith("timer="):
                match_val = re.search(r'(\d+)', line)
                if match_val: tick_counter = int(match_val.group(1))
                else: tick_counter += 1
                continue
            
            match_timer = re.match(r'timer\s*=\s*(\d+)', line, re.IGNORECASE)
            if match_timer:
                tick_counter = int(match_timer.group(1))
                continue

            parsed = parse_pid_line(line)
            if parsed:
                target, actual, control_signals = parsed
                sim_time = tick_counter / float(SIM_TICKS_PER_SEC)
                q.put((sim_time, target, actual, control_signals))
                timeline.append((sim_time, target, actual, control_signals))

    except Exception as e:
        print(f"[Tracker PID] Error de lectura: {e}")
    finally:
        stop_event.set()

def main():
    timeline = []
    q = queue.Queue()
    stop_event = threading.Event()

    t_data = []
    target_data = []
    actual_data = []
    control_data = {} # Diccionario para almacenar las listas de control dinámicas

    print("[Tracker PID] [+] Iniciando monitoreo de señales...")
    reader_thread = threading.Thread(target=stdin_reader, args=(q, timeline, stop_event, DEFAULT_END_PATTERN), daemon=True)
    reader_thread.start()

    fig_main, ax_main = plt.subplots(figsize=(10, 5))
    fig_main.canvas.manager.set_window_title(TITULO_GRAFICA)

    def update(frame):
        updated = False
        while True:
            try:
                sim_time, t, a, c_dict = q.get_nowait()
                t_data.append(sim_time)
                target_data.append(t)
                actual_data.append(a)
                
                # Actualizar arreglos de control dinámico
                for name, val in c_dict.items():
                    if name not in control_data:
                        # Rellenar con None hacia atrás si aparece una variable nueva
                        control_data[name] = [None] * (len(t_data) - 1)
                    control_data[name].append(val)
                
                # Rellenar con None si falta alguna variable en este tick
                for name in control_data:
                    if name not in c_dict:
                        control_data[name].append(None)

                updated = True
            except queue.Empty:
                break
        
        if updated:
            ax_main.clear()
            ax_main.set_title(f"{TITULO_GRAFICA} | Tiempo: {t_data[-1]:.2f}s")
            ax_main.set_xlabel("Tiempo (s)")
            ax_main.set_ylabel("Magnitud")
            ax_main.grid(True, linestyle='--', alpha=0.7)

            if VER_ENTRADA:
                ax_main.plot(t_data, target_data, label='Entrada (Referencia)', color=COLOR_ENTRADA, linestyle='--', linewidth=2)
            if VER_SALIDA:
                ax_main.plot(t_data, actual_data, label='Salida (Distancia Actual)', color=COLOR_SALIDA, linewidth=2)
            if VER_CONTROL:
                for name, data_list in control_data.items():
                    ax_main.plot(t_data, data_list, label=f'Control: {name}', linewidth=1.5, alpha=0.8)
            
            if VER_ENTRADA or VER_SALIDA or VER_CONTROL:
                ax_main.legend(loc='upper right')

        return []

    interval_ms = int(1000 / max(1, int(LIVE_FPS)))
    ani = animation.FuncAnimation(fig_main, update, interval=interval_ms, blit=False, cache_frame_data=False)

    try:
        plt.ion()
        plt.show()
        while not stop_event.is_set() and plt.fignum_exists(fig_main.number):
            plt.pause(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    if plt.fignum_exists(fig_main.number): plt.close(fig_main)
    reader_thread.join()
    print("\n[Tracker PID] Simulación terminada. Procesando archivos finales...")

    if not timeline:
        print("[Tracker PID] [!] No se registraron datos del PID durante la simulación.")
        return

    # Extraer todos los nombres de variables de control encontradas para la exportación
    control_names = set()
    for _, _, _, c_dict in timeline:
        control_names.update(c_dict.keys())
    control_names = sorted(list(control_names))

    # =========================================================================
    # GUARDADO DE CSV
    # =========================================================================
    if GUARDAR_CSV:
        os.makedirs(CSV_DIR, exist_ok=True)
        csv_path = os.path.join(CSV_DIR, OUTFILE_CSV)
        try:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                header = ['time_iso', 'sim_time', 'target', 'actual'] + control_names
                writer.writerow(header)
                
                for ts, tg, ac, c_dict in timeline:
                    real_time_iso = datetime.now().isoformat(sep=' ', timespec='milliseconds')
                    row = [real_time_iso, f"{ts:.6f}", f"{tg:.6f}", f"{ac:.6f}"]
                    for name in control_names:
                        val = c_dict.get(name, 0.0)
                        row.append(f"{val:.6f}")
                    writer.writerow(row)
            print(f"[Tracker PID] [+] CSV guardado en: {csv_path}")
        except Exception as e:
            print(f"[Tracker PID] [!] Error al guardar CSV: {e}")

    # =========================================================================
    # GUARDADO DE IMAGEN FINAL
    # =========================================================================
    if GUARDAR_IMAGEN:
        os.makedirs(IMAGEN_DIR, exist_ok=True)
        img_path = os.path.join(IMAGEN_DIR, OUTFILE_IMAGEN)
        
        full_t = [row[0] for row in timeline]
        full_target = [row[1] for row in timeline]
        full_actual = [row[2] for row in timeline]

        fig_img, ax_img = plt.subplots(figsize=(10, 5))
        ax_img.set_title(f"{TITULO_GRAFICA} (Resultado Final)")
        ax_img.set_xlabel("Tiempo (s)")
        ax_img.set_ylabel("Magnitud")
        ax_img.grid(True, linestyle='--', alpha=0.7)

        if VER_ENTRADA:
            ax_img.plot(full_t, full_target, label='Entrada (Referencia)', color=COLOR_ENTRADA, linestyle='--', linewidth=2)
        if VER_SALIDA:
            ax_img.plot(full_t, full_actual, label='Salida (Distancia Actual)', color=COLOR_SALIDA, linewidth=2)
        if VER_CONTROL:
            for name in control_names:
                c_data = [row[3].get(name, None) for row in timeline]
                ax_img.plot(full_t, c_data, label=f'Control: {name}', linewidth=1.5, alpha=0.8)
        
        if VER_ENTRADA or VER_SALIDA or VER_CONTROL:
            ax_img.legend(loc='upper right')

        plt.tight_layout()
        try:
            fig_img.savefig(img_path, dpi=300)
            print(f"[Tracker PID] [+] Imagen guardada en: {img_path}")
        except Exception as e:
            print(f"[Tracker PID] [!] Error al guardar Imagen: {e}")
        finally:
            plt.close(fig_img)

    # =========================================================================
    # GUARDADO DE VIDEO
    # =========================================================================
    if GUARDAR_VIDEO:
        os.makedirs(VIDEO_DIR, exist_ok=True)
        video_path = os.path.join(VIDEO_DIR, OUTFILE_VIDEO)
        print("[Tracker PID] [+] Renderizando Video de la gráfica PID...")
        
        try:
            fig_vid = Figure(figsize=(10, 5), dpi=100)
            canvas = FigureCanvas(fig_vid)
            ax_vid = fig_vid.add_subplot(111)
            fig_vid.canvas.draw()
            width, height = fig_vid.canvas.get_width_height()

            ffmpeg_cmd = [
                "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo", "-s", f"{width}x{height}",
                "-pix_fmt", "rgba", "-r", str(VIDEO_FPS), "-i", "-", "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", video_path
            ]
            
            ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            total_frames = len(timeline)

            v_t, v_target, v_actual = [], [], []
            v_control = {name: [] for name in control_names}

            for idx, data_point in enumerate(timeline):
                ts, tg, ac, c_dict = data_point
                v_t.append(ts)
                v_target.append(tg)
                v_actual.append(ac)
                
                for name in control_names:
                    v_control[name].append(c_dict.get(name, None))

                ax_vid.clear()
                ax_vid.set_title(f"{TITULO_GRAFICA} | Tiempo: {ts:.2f}s")
                ax_vid.set_xlim(timeline[0][0], timeline[-1][0])
                
                # Ajuste dinámico del eje Y
                all_vals = v_target + v_actual
                for c_list in v_control.values():
                    all_vals.extend([v for v in c_list if v is not None])
                
                if all_vals:
                    ax_vid.set_ylim(min(all_vals) - 5, max(all_vals) + 5)
                
                ax_vid.set_xlabel("Tiempo (s)")
                ax_vid.set_ylabel("Magnitud")
                ax_vid.grid(True, linestyle='--', alpha=0.7)

                if VER_ENTRADA: ax_vid.plot(v_t, v_target, label='Entrada (Ref)', color=COLOR_ENTRADA, linestyle='--')
                if VER_SALIDA:  ax_vid.plot(v_t, v_actual, label='Salida (Dist)', color=COLOR_SALIDA)
                if VER_CONTROL: 
                    for name in control_names:
                        ax_vid.plot(v_t, v_control[name], label=f'Control: {name}')
                        
                if VER_ENTRADA or VER_SALIDA or VER_CONTROL: ax_vid.legend(loc='upper right')

                fig_vid.canvas.draw()
                ffmpeg_proc.stdin.write(fig_vid.canvas.buffer_rgba())

                if (idx + 1) % 10 == 0:
                    porcentaje = ((idx + 1) / total_frames) * 100
                    sys.stdout.write(f"\r[Tracker PID] Renderizando frame {idx + 1}/{total_frames} ({porcentaje:.1f}%)")
                    sys.stdout.flush()

            ffmpeg_proc.stdin.close()
            ffmpeg_proc.wait()
            del ax_vid, canvas, fig_vid
            gc.collect()
            print(f"\n[Tracker PID] [+] Video guardado en: {video_path}")
            
        except Exception as e:
            print(f"\n[Tracker PID] [!] Error crítico al guardar video: {e}")

    print("[Tracker PID] Operación Finalizada con éxito.")

if __name__ == '__main__':
    main()