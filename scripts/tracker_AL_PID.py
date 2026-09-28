#!/usr/bin/env python3
"""
tracker_AL_PID.py

Este script grafica en tiempo real las señales del controlador PID del Active Learning (Modo 3).
Permite evaluar el desempeño del control al visualizar:
1. Señal de Entrada (Setpoint / Referencia)
2. Señal de Salida  (Variable de Proceso / Distancia actual al centroide)
3. Señal de Control (Esfuerzo del controlador / Salida PID)

Depende de que el controlador Lua imprima líneas con el formato:
[PID];target;actual;output
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
DEFAULT_END_PATTERN = "End_simulation"  # Palabra que finaliza el tracker
SIM_TICKS_PER_SEC   = 10                # Frecuencia de la simulación en ARGoS
LIVE_FPS            = 10                # FPS de la gráfica en vivo

# =============================================================================
# ---------- CONFIGURACIÓN DE VISIBILIDAD DE SEÑALES ----------
# =============================================================================
# Cambia a False las señales que no quieras visualizar.
VER_ENTRADA = True  # Target / Referencia (La distancia deseada al centroide)
VER_SALIDA  = True  # Actual / Variable de Proceso (La distancia real al centroide)
VER_CONTROL = True  # Output / Señal de Control (La magnitud de movimiento calculada)

# Colores y estilos
COLOR_ENTRADA = 'green'     # Verde (línea punteada)
COLOR_SALIDA  = 'blue'      # Azul (línea continua)
COLOR_CONTROL = 'red'       # Rojo (línea continua delgada)

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO (RESULTADOS) ----------
# =============================================================================
# Directorio base
DIR_BASE = "Resultados/Robots/Qupa/Ks4/PID"

# --- 1. Video ---
GUARDAR_VIDEO = False
VIDEO_DIR     = f"{DIR_BASE}/Video"
OUTFILE_VIDEO = "PID_Ks4.mp4"
VIDEO_FPS     = 10

# --- 2. Imagen Final ---
GUARDAR_IMAGEN = True
IMAGEN_DIR     = f"{DIR_BASE}/Imagen"
OUTFILE_IMAGEN = "Grafica_PID_Ks4.png"

# --- 3. Datos en CSV ---
GUARDAR_CSV = True
CSV_DIR     = f"{DIR_BASE}/Data"
OUTFILE_CSV = "Datos_PID_Ks4.csv"

# =============================================================================

def parse_pid_line(line):
    """
    Extrae los datos de la línea del PID soportando decimales con punto o coma.
    Formato esperado: [PID];target;actual;output
    """
    line_str = line.strip()
    if line_str.startswith("[PID];"):
        parts = line_str.split(';')
        if len(parts) >= 4:
            try:
                target = float(parts[1].replace(',', '.'))
                actual = float(parts[2].replace(',', '.'))
                output = float(parts[3].replace(',', '.'))
                return target, actual, output
            except ValueError:
                pass
    return None

def stdin_reader(q, timeline, stop_event, end_pattern):
    """ Hilo de lectura de la terminal. """
    tick_counter = 0
    first_timer_seen = False

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
                first_timer_seen = True
                match_val = re.search(r'(\d+)', line)
                if match_val:
                    tick_counter = int(match_val.group(1))
                else:
                    tick_counter += 1
                continue
            
            match_timer = re.match(r'timer\s*=\s*(\d+)', line, re.IGNORECASE)
            if match_timer:
                first_timer_seen = True
                tick_counter = int(match_timer.group(1))
                continue

            # Lectura del PID
            parsed = parse_pid_line(line)
            if parsed:
                target, actual, output = parsed
                sim_time = tick_counter / float(SIM_TICKS_PER_SEC)
                
                # Enviamos a la gráfica en vivo
                q.put((sim_time, target, actual, output))
                # Guardamos en la línea de tiempo global
                timeline.append((sim_time, target, actual, output))

    except Exception as e:
        print(f"[Tracker PID] Error de lectura: {e}")
    finally:
        stop_event.set()

def main():
    timeline = []
    q = queue.Queue()
    stop_event = threading.Event()

    # Listas de datos para la gráfica
    t_data = []
    target_data = []
    actual_data = []
    control_data = []

    print("[Tracker PID] [+] Iniciando monitoreo de señales...")
    
    # Iniciar hilo de lectura
    reader_thread = threading.Thread(target=stdin_reader, args=(q, timeline, stop_event, DEFAULT_END_PATTERN), daemon=True)
    reader_thread.start()

    # Preparar Gráfica en vivo
    fig_main, ax_main = plt.subplots(figsize=(10, 5))
    fig_main.canvas.manager.set_window_title(TITULO_GRAFICA)

    # Posicionar ventana a la izquierda (Opcional, para no pisar el tracker de trayectoria si abres ambos)
    try:
        mngr = fig_main.canvas.manager
        if hasattr(mngr.window, 'wm_geometry'): mngr.window.wm_geometry("+50+50")
        elif hasattr(mngr.window, 'move'): mngr.window.move(50, 50)
    except Exception: pass

    def update(frame):
        updated = False
        while True:
            try:
                sim_time, t, a, c = q.get_nowait()
                t_data.append(sim_time)
                target_data.append(t)
                actual_data.append(a)
                control_data.append(c)
                updated = True
            except queue.Empty:
                break
        
        if updated:
            ax_main.clear()
            ax_main.set_title(f"{TITULO_GRAFICA} | Tiempo: {t_data[-1]:.2f}s")
            ax_main.set_xlabel("Tiempo (s)")
            ax_main.set_ylabel("Magnitud (Distancia / PID Out)")
            ax_main.grid(True, linestyle='--', alpha=0.7)

            if VER_ENTRADA:
                ax_main.plot(t_data, target_data, label='Entrada (Referencia)', color=COLOR_ENTRADA, linestyle='--', linewidth=2)
            if VER_SALIDA:
                ax_main.plot(t_data, actual_data, label='Salida (Distancia Actual)', color=COLOR_SALIDA, linewidth=2)
            if VER_CONTROL:
                ax_main.plot(t_data, control_data, label='Control (Salida PID)', color=COLOR_CONTROL, linewidth=1.5, alpha=0.8)
            
            # Solo dibujamos la leyenda si hay al menos una línea
            if VER_ENTRADA or VER_SALIDA or VER_CONTROL:
                ax_main.legend(loc='upper right')

        return []

    interval_ms = int(1000 / max(1, int(LIVE_FPS)))
    ani = animation.FuncAnimation(fig_main, update, interval=interval_ms, blit=False, cache_frame_data=False)

    # Bucle principal de la gráfica
    try:
        plt.ion()
        plt.show()
        while not stop_event.is_set() and plt.fignum_exists(fig_main.number):
            plt.pause(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    if plt.fignum_exists(fig_main.number):
        plt.close(fig_main)

    reader_thread.join()
    print("\n[Tracker PID] Simulación terminada. Procesando archivos finales...")

    if not timeline:
        print("[Tracker PID] [!] No se registraron datos del PID durante la simulación.")
        return

    # =========================================================================
    # GUARDADO DE CSV
    # =========================================================================
    if GUARDAR_CSV:
        os.makedirs(CSV_DIR, exist_ok=True)
        csv_path = os.path.join(CSV_DIR, OUTFILE_CSV)
        try:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_iso', 'sim_time', 'target', 'actual', 'control_output'])
                for ts, tg, ac, co in timeline:
                    real_time_iso = datetime.now().isoformat(sep=' ', timespec='milliseconds')
                    writer.writerow([real_time_iso, f"{ts:.6f}", f"{tg:.6f}", f"{ac:.6f}", f"{co:.6f}"])
            print(f"[Tracker PID] [+] CSV guardado en: {csv_path}")
        except Exception as e:
            print(f"[Tracker PID] [!] Error al guardar CSV: {e}")

    # =========================================================================
    # GUARDADO DE IMAGEN FINAL
    # =========================================================================
    if GUARDAR_IMAGEN:
        os.makedirs(IMAGEN_DIR, exist_ok=True)
        img_path = os.path.join(IMAGEN_DIR, OUTFILE_IMAGEN)
        
        # Extraemos los datos completos del timeline
        full_t = [row[0] for row in timeline]
        full_target = [row[1] for row in timeline]
        full_actual = [row[2] for row in timeline]
        full_control = [row[3] for row in timeline]

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
            ax_img.plot(full_t, full_control, label='Control (Salida PID)', color=COLOR_CONTROL, linewidth=1.5, alpha=0.8)
        
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
    # GUARDADO DE VIDEO (Piping a Memoria)
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
            
            # Variables de progreso
            total_frames = len(timeline)
            v_t, v_target, v_actual, v_control = [], [], [], []

            for idx, data_point in enumerate(timeline):
                ts, tg, ac, co = data_point
                v_t.append(ts)
                v_target.append(tg)
                v_actual.append(ac)
                v_control.append(co)

                ax_vid.clear()
                ax_vid.set_title(f"{TITULO_GRAFICA} | Tiempo: {ts:.2f}s")
                ax_vid.set_xlim(timeline[0][0], timeline[-1][0]) # Eje X fijo total para ver cómo avanza
                
                # Eje Y dinámico que se adapta al min/max actual
                min_y = min(min(v_target), min(v_actual), min(v_control)) - 5
                max_y = max(max(v_target), max(v_actual), max(v_control)) + 5
                ax_vid.set_ylim(min_y, max_y)
                
                ax_vid.set_xlabel("Tiempo (s)")
                ax_vid.set_ylabel("Magnitud")
                ax_vid.grid(True, linestyle='--', alpha=0.7)

                if VER_ENTRADA: ax_vid.plot(v_t, v_target, label='Entrada (Ref)', color=COLOR_ENTRADA, linestyle='--')
                if VER_SALIDA:  ax_vid.plot(v_t, v_actual, label='Salida (Dist)', color=COLOR_SALIDA)
                if VER_CONTROL: ax_vid.plot(v_t, v_control, label='Control (PID)', color=COLOR_CONTROL)
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