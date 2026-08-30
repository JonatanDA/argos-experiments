#!/usr/bin/env python3
"""
tracker_varianza.py

- controllers compatibles: Tr6, Ks1 -> Qupa

DEPENDENCIAS LUA REQUERIDAS:
- Función: `Print_tracker_varianza(k, var)`

CARACTERÍSTICAS PRINCIPALES:
- Maneja exclusivamente la recepción y gráfica de los datos de varianza de los maestros.
- NO grafica en vivo la trayectoria continua. Acumula las posiciones y cuando recibe 
  los datos de varianza, abre la ventana graficando ambos (Varianza + Trayectoria completada).
- Guardado independiente de la visualización (puede trabajar en modo headless).
- Gestión dinámica del límite de ventanas (desliza y cierra las más antiguas).
"""

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Control de Gráficas y Modos ---
MOSTRAR_TRAYECTORIA             = 2                 # 0: Sin gráfica visual, 1: Solo varianza, 2: Varianza + Trayectoria
MODO_GRAFICA                    = 0                 # 0: Todos los maestros, 1: Solo los del filtro
VENTANA_N                       = 10                # Límite máximo de ventanas emergentes simultáneas (cierra la más antigua).
GRAFICAR_VARIANZA_COMO_LINEA    = True              # True: Grafica varianza como línea continua, False: Como puntos discretos
GRAFICAR_TRAYECTORIA_COMO_LINEA = False             # True: Grafica trayectoria como línea continua, False: Como puntos discretos

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
GUARDAR_IMAGEN          = 0                 # 0: No guardar, 1: Guardar solo Varianza, 2: Guardar Varianza + Trayectoria
DIRECTORIO_GUARDADO     = "Resultados/Robots/Qupa/Ks1/Varianza" # Directorio donde se almacenarán las capturas
NOMBRE_BASE             = "Qa"              # Prefijo de la imagen guardada. (Ej: 'ma' genera 'ma1.png', 'ma2.png')

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
TIEMPO_ESPERA_MAESTRO   = 1.0               # Tiempo mínimo entre actualizaciones gráficas por maestro en vivo

import sys
import os
import argparse
import threading
import queue
import re
import time
from collections import defaultdict
import matplotlib.pyplot as plt

NUMBER_RE = re.compile(r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?')

def parse_line(line):
    """
    Parsea la línea detectando si se envían 1 dato (varianza) o >=2 datos (coordenadas de trayectoria).
    Normaliza MasterX a MaestroX para que coincidan en el mismo diccionario.
    """
    if '&' not in line: return None
    parts = line.split('&', 1)
    name_raw = parts[0].strip()
    nums = NUMBER_RE.findall(parts[1])
    
    # Limpieza de nombre
    name_clean = re.sub(r'^\[.*?\]\s*', '', name_raw).lower().replace(" ", "")
    
    # Normalización: "master1" se convierte en "maestro1" para vincular los datos
    if name_clean.startswith("master"):
        name_clean = name_clean.replace("master", "maestro")
        
    if name_clean.startswith("maestro"):
        name = name_clean
        
        # Si tiene 1 solo número -> Es un dato de Varianza
        if len(nums) == 1:
            try:
                var = float(nums[0].replace(',', '.'))
                return 'maestro', name, 'var', var, None
            except ValueError:
                pass
        
        # Si tiene 2 (o más) números -> Es un dato de Trayectoria (X, Y)
        elif len(nums) >= 2:
            try:
                x = float(nums[0].replace(',', '.'))
                y = float(nums[1].replace(',', '.'))
                return 'maestro', name, 'traj', x, y
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
                category, name, dtype, val1, val2 = parsed
                ts = time.time()
                q.put((category, name, dtype, val1, val2, ts))
                last_time_holder[0] = ts
    except Exception:
        pass
    finally:
        stop_event.set()

def dibujar_canvas(m_name, fig, ax_m, ax_t, data_maestro, mode_config):
    """
    Función centralizada para dibujar. Se usa tanto para la GUI interactiva como para renderizar 
    las imágenes de guardado en background de forma independiente.
    """
    vars_data = data_maestro['vars']
    xs_data = data_maestro['xs']
    ys_data = data_maestro['ys']

    # --- 1. Gráfica de Varianza ---
    ax_m.clear()
    x_data_var = list(range(1, len(vars_data) + 1))
    
    sin_trayectoria = (mode_config == 2) and not xs_data
    titulo_var = f'Varianza: {m_name}'
    if sin_trayectoria:
        titulo_var += '\n(Sin datos de trayectoria)'
        
    ax_m.set_title(titulo_var)
    ax_m.set_xlabel('Número de dato')
    ax_m.set_ylabel('Varianza')
    ax_m.grid(True)
    if vars_data:
        if GRAFICAR_VARIANZA_COMO_LINEA:
            ax_m.plot(x_data_var, vars_data, lw=1.5, color='orange')
        else:
            ax_m.scatter(x_data_var, vars_data, s=25, color='orange')
        ax_m.scatter([x_data_var[-1]], [vars_data[-1]], s=25, color='red')

    # --- 2. Gráfica de Trayectoria (Opcional) ---
    if mode_config == 2 and ax_t is not None:
        ax_t.clear()
        if xs_data and ys_data:
            ax_t.set_title(f'Trayectoria: {m_name}')
            ax_t.set_xlabel('X')
            ax_t.set_ylabel('Y')
            ax_t.grid(True)
            if GRAFICAR_TRAYECTORIA_COMO_LINEA:
                ax_t.plot(xs_data, ys_data, lw=1.5, color='blue', alpha=0.7)
            else:
                ax_t.scatter(xs_data, ys_data, s=25, color='blue', alpha=0.7)
            ax_t.scatter([xs_data[-1]], [ys_data[-1]], s=25, color='green', zorder=5)
            # Marcar el punto de inicio con otro color para contexto visual
            ax_t.scatter([xs_data[0]], [xs_data[0]], s=20, color='black', marker='x', label='Inicio')
            ax_t.legend()
        else:
            ax_t.set_title('Sin datos de trayectoria')
            ax_t.text(0.5, 0.5, 'No se registraron posiciones', 
                      horizontalalignment='center', verticalalignment='center', 
                      transform=ax_t.transAxes, color='gray')
            ax_t.axis('off')

    fig.canvas.draw_idle()


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

    # Se agregan 'xs' e 'ys' al diccionario de cada maestro para la trayectoria
    data_maestros = defaultdict(lambda: {'vars': [], 'xs': [], 'ys': []})
    
    # Control de la GUI en vivo
    maestro_figs = {}            
    last_seen_maestro = {}       
    maestro_needs_plot = {}      
    historial_ventanas = [] # Usado para controlar el límite de ventanas

    if MOSTRAR_TRAYECTORIA > 0:
        plt.ion() 

    try:
        while not stop_event.is_set():
            current_time = time.time()
            
            # 1. Procesar todos los datos nuevos
            while True:
                try:
                    category, name, dtype, val1, val2, ts = q.get_nowait()
                except queue.Empty:
                    break
                
                # Control por Modo de Gráfica (1: Aplica el filtro, 0: Lo ignora)
                if MODO_GRAFICA == 1 and maestros_filter and name not in maestros_filter:
                    continue
                
                # --- ACUMULACIÓN DE DATOS SEGÚN SU TIPO ---
                if dtype == 'traj':
                    data_maestros[name]['xs'].append(val1)
                    data_maestros[name]['ys'].append(val2)
                
                elif dtype == 'var':
                    # Solo nos preocupamos de las ventanas si MOSTRAR_TRAYECTORIA > 0
                    if MOSTRAR_TRAYECTORIA > 0 and name not in maestro_figs:
                        
                        # Lógica de cierre para limitar el número de ventanas emergentes (VENTANA_N)
                        if VENTANA_N > 0 and len(historial_ventanas) >= VENTANA_N:
                            oldest_m = historial_ventanas.pop(0)
                            if oldest_m in maestro_figs:
                                fig_old, _, _ = maestro_figs[oldest_m]
                                plt.close(fig_old)
                                del maestro_figs[oldest_m]
                                
                        historial_ventanas.append(name)

                        # Creación de la ventana
                        if MOSTRAR_TRAYECTORIA == 2:
                            fig_m, axes = plt.subplots(1, 2, figsize=(10, 4))
                            ax_m, ax_t = axes[0], axes[1]
                            maestro_figs[name] = (fig_m, ax_m, ax_t)
                        else: # MOSTRAR_TRAYECTORIA == 1
                            fig_m, ax_m = plt.subplots(figsize=(5, 3))
                            maestro_figs[name] = (fig_m, ax_m, None)
                        
                        fig_m.canvas.manager.set_window_title(f'Resultados - {name}')
                        plt.show(block=False)
                    
                    data_maestros[name]['vars'].append(val1)
                    last_seen_maestro[name] = current_time
                    
                    if MOSTRAR_TRAYECTORIA > 0:
                        maestro_needs_plot[name] = True

            # 2. Actualizar gráficos en vivo (Si la visualización está encendida)
            if MOSTRAR_TRAYECTORIA > 0:
                for m_name, figs in maestro_figs.items():
                    if maestro_needs_plot.get(m_name, False):
                        tiempo_silencio = current_time - last_seen_maestro.get(m_name, current_time)
                        if tiempo_silencio >= TIEMPO_ESPERA_MAESTRO:
                            fig_m, ax_m, ax_t = figs
                            dibujar_canvas(m_name, fig_m, ax_m, ax_t, data_maestros[m_name], MOSTRAR_TRAYECTORIA)
                            maestro_needs_plot[m_name] = False
                
                plt.pause(0.1) # Mantiene viva la interfaz
            else:
                time.sleep(0.1) # Pausa ligera para no saturar CPU en modo headless

            # 3. Timeout general
            if args.inactivity_timeout is not None:
                if (time.time() - last_time_holder[0]) > args.inactivity_timeout:
                    stop_event.set()
                    break

    except KeyboardInterrupt:
        stop_event.set()
    
    # --- LIMPIEZA Y GRÁFICA FINAL (GUI en vivo) ---
    if MOSTRAR_TRAYECTORIA > 0:
        for m_name, figs in maestro_figs.items():
            if maestro_needs_plot.get(m_name, False):
                fig_m, ax_m, ax_t = figs
                dibujar_canvas(m_name, fig_m, ax_m, ax_t, data_maestros[m_name], MOSTRAR_TRAYECTORIA)

    # --- GUARDADO INDEPENDIENTE DE IMÁGENES ---
    if GUARDAR_IMAGEN > 0:
        os.makedirs(DIRECTORIO_GUARDADO, exist_ok=True)
        print(f"\n[Tracker Varianza] [+] Generando y guardando imágenes en background en: {DIRECTORIO_GUARDADO}")
        
        for m_name, data_m in data_maestros.items():
            if not data_m['vars']: # Solo guardamos si el maestro registró datos de varianza
                continue
                
            # Construir un lienzo exclusivo para el guardado (no se muestra al usuario)
            if GUARDAR_IMAGEN == 2:
                fig_save, axes = plt.subplots(1, 2, figsize=(10, 4))
                ax_m_save, ax_t_save = axes[0], axes[1]
            else: # GUARDAR_IMAGEN == 1
                fig_save, ax_m_save = plt.subplots(figsize=(5, 3))
                ax_t_save = None

            # Pintar el canvas y guardarlo
            dibujar_canvas(m_name, fig_save, ax_m_save, ax_t_save, data_m, GUARDAR_IMAGEN)
            
            match_num = re.search(r'\d+', m_name)
            sufijo = match_num.group(0) if match_num else m_name
            ruta_imagen = os.path.join(DIRECTORIO_GUARDADO, f"{NOMBRE_BASE}{sufijo}.png")
            
            fig_save.savefig(ruta_imagen, bbox_inches='tight', dpi=150)
            plt.close(fig_save) # Liberamos la memoria de la figura guardada
            print(f"    -> Imagen guardada: {ruta_imagen}")
        print()

    # --- LÓGICA DE CIERRE GUI ---
    if maestro_figs and MOSTRAR_TRAYECTORIA > 0:
        if AUTO_CLOSE_WINDOWS:
            print("[Tracker Varianza] Simulación terminada. Cerrando ventanas automáticamente...")
            plt.close('all')
        else:
            print("[Tracker Varianza] Simulación terminada. Cierra las ventanas manualmente para salir.")
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()