#!/usr/bin/env python3
"""
tracker_calidad_imitacion.py

- controllers compatibles: Tr6, Tr7, Ks1 -> Qupa

DEPENDENCIAS LUA REQUERIDAS:
- Necesita que se llame a la función `Print_tracker_calidad_imitacion_data(linea_resumen, is_master)`
  desde el script del robot en ARGoS para alimentarse de los datos.

CARACTERÍSTICAS PRINCIPALES:
    Evaluador Geométrico:
    - Realiza Alineamiento Circular con Recuperación Geométrica (Heal Pattern) del patrón imitado.
    - Evalúa la métrica general Qi (Calidad de Imitación) basándose en longitud, ángulos y segmentos.

"""

import sys
import re
import math
import queue
import threading
import os
import matplotlib.pyplot as plt

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Ventanas de Comparación ---
SHOW_COMPARISON_WINDOWS = True                                     # True: Abre pop-ups evaluando la calidad de imitación
MAX_WINDOWS             = 5                                        # Límite de ventanas abiertas simultáneamente

# --- Ventana de Diccionario Original ---
SHOW_ALL_ORIGINALS      = False                                    # True: Muestra gráfica con todos los patrones originales de referencia

# --- Modo de Gráfica ---
PLOT_SUPERPUESTAS       = False                                    # (Aplica para SHOW_COMPARISON y SHOW_ALL_ORIGINALS) True: Gráficas superpuesta. False: Gráficas separadas.

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
SAVE_IMAGES             = False                                    # True: Guarda capturas de las gráficas de comparación automáticamente
IMAGE_DIR               = "Resultados/Robots/Qupa/Tr7/Pentagono/Calidad_Imitacion"    # (Aplica si SAVE_IMAGES=True) Directorio de guardado

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
END_PATTERN             = "End_simulation"                 # Frase clave para dejar de escuchar el tracker
AUTO_CLOSE_WINDOWS      = True                                    # True: Cierra las gráficas automáticamente al terminar
AUTO_CLOSE_TIME         = 10.0                                     # (Aplica si AUTO_CLOSE_WINDOWS=True) Segundos de espera antes de cerrar

# =============================================================================
# ---------- CONFIGURACIÓN DE PROCESAMIENTO Y LÓGICA ----------
# =============================================================================
# --- Control Geométrico ---
USE_FIRST_ANGLE         = False                                    # True: Usa el primer ángulo del patrón, False: Lo fija en 0.0 direccional
ASSUME_CLOSED_POLYGONS  = True                                     # True: Fuerza el cierre geométrico del patrón calculando el giro faltante (Heal Pattern)

# --- Pesos de Ecuación de Calidad de Imitación (Qi) ---
WEIGHT_L                = 1.0                                     # Peso para la métrica de Longitud (Ql)
WEIGHT_A                = 1.0                                     # Peso para la métrica de Ángulos (Qa)
WEIGHT_S                = 1.0                                     # Peso para la métrica de Segmentos (Qs)

reference_patterns = {} 
active_figs = []
fig_originals = None

def parse_line(line):
    if '&' not in line: return None
    parts = line.split('&')
    if len(parts) < 3: return None
       
    robot_id = parts[0].strip()
    role = parts[1].strip()
    data_str = parts[2].strip()
    
    # Limpiar colores bash ANSI si los hubiera
    data_str = re.sub(r'\x1b\[[0-9;]*m', '', data_str)
    
    patterns_found = {}
    current_name = "default"
    
    matches = re.finditer(r'([\w\s]+)?\(([\-\d\.\,]+)\)', data_str)
    
    for m in matches:
        p_name = m.group(1)
        if p_name and p_name.strip():
            current_name = p_name.strip()
            if current_name not in patterns_found:
                 patterns_found[current_name] = []
           
        raw_nums = m.group(2)
        nums_split = raw_nums.split(',')
       
        try:
            if len(nums_split) == 4:
                dist = float(f"{nums_split[0]}.{nums_split[1]}")
                ang = float(f"{nums_split[2]}.{nums_split[3]}")
            elif len(nums_split) == 2:
                dist = float(nums_split[0])
                ang = float(nums_split[1])
            else:
                continue
               
            if current_name not in patterns_found:
                 patterns_found[current_name] = []
            patterns_found[current_name].append((dist, ang))
           
        except ValueError:
            continue
           
    if not patterns_found or (len(patterns_found) == 1 and not patterns_found.get(current_name, [])):
        return None
       
    return {'id': robot_id, 'role': role, 'patterns': patterns_found}

def clean_vectors(vectors, is_original=False):
    """
    Filtra segmentos con distancia == 0.0 (utilizados para control de tiempo).
    Acumula los ángulos de los segmentos de relleno iniciales y finales en el primer segmento válido.
    """
    if not vectors: return vectors
    vecs = list(vectors)
    
    # Separar segmentos iniciales con distancia 0.0
    leading_angle_sum = 0.0
    idx = 0
    while idx < len(vecs) and vecs[idx][0] == 0.0:
        leading_angle_sum += vecs[idx][1]
        idx += 1
        
    if idx == len(vecs):
        return []
        
    # Separar segmentos finales con distancia 0.0
    trailing_angle_sum = 0.0
    end_idx = len(vecs) - 1
    while end_idx >= idx and vecs[end_idx][0] == 0.0:
        trailing_angle_sum += vecs[end_idx][1]
        end_idx -= 1
        
    middle_vecs = vecs[idx:end_idx+1]
    if not middle_vecs:
        return []
        
    cleaned = list(middle_vecs)
    first_dist, first_ang = cleaned[0]
    cleaned[0] = (first_dist, first_ang + leading_angle_sum + trailing_angle_sum)
    
    return cleaned

def heal_pattern(vectors):
    if not ASSUME_CLOSED_POLYGONS or len(vectors) <= 2:
        return list(vectors)
       
    sum_angles = sum(a for d, a in vectors[1:])
    target_sum = 360.0 if sum_angles >= 0 else -360.0
    missing_turn = target_sum - sum_angles
       
    healed = list(vectors)
    healed[0] = (healed[0][0], missing_turn)
    return healed

def calc_coords(vectors):
    x, y = 0.0, 0.0
    current_angle = 0.0
    xs, ys = [x], [y]
    for dist, ang in vectors:
        current_angle += ang
        rad = math.radians(current_angle)
        x += dist * math.cos(rad)
        y += dist * math.sin(rad)
        xs.append(x)
        ys.append(y)
    return xs, ys

def evaluate_against_reference(orig, imit):
    No = len(orig)
    Nc = len(imit)
    if No == 0 or Nc == 0: return imit, (0, 0, 0, 0)
   
    orig_healed = heal_pattern(orig)
    imit_healed = heal_pattern(imit)
   
    Nmin = min(No, Nc)
    best_qi = -float('inf')
    best_metrics = (0, 0, 0, 0)
    best_aligned_imit = imit
   
    Qs = 1.0 - (abs(Nc - No) / No)
   
    for k in range(Nc):
        shifted_imit = imit_healed[k:] + imit_healed[:k]
       
        eval_orig = list(orig_healed)
        if not USE_FIRST_ANGLE: eval_orig[0] = (eval_orig[0][0], 0.0)
       
        eval_imit = list(shifted_imit)
        if not USE_FIRST_ANGLE: eval_imit[0] = (eval_imit[0][0], 0.0)
       
        sum_l_o = sum(abs(eval_orig[m][0]) for m in range(Nmin))
        sum_diff_l = sum(abs(eval_orig[m][0] - eval_imit[m][0]) for m in range(Nmin))
        Ql = 1.0 - (sum_diff_l / sum_l_o) if sum_l_o > 0 else (1.0 if sum_diff_l == 0 else 0.0)
       
        start_idx = 0 if USE_FIRST_ANGLE else 1
        if Nmin > start_idx:
            valid_m = range(start_idx, Nmin)
            sum_a_o = sum(abs(eval_orig[m][1]) for m in valid_m)
            sum_diff_a = sum(abs(eval_orig[m][1] - eval_imit[m][1]) for m in valid_m)
            Qa = 1.0 - (sum_diff_a / sum_a_o) if sum_a_o > 0 else (1.0 if sum_diff_a == 0 else 0.0)
        else:
            Qa = 1.0
           
        Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
       
        if Qi > best_qi:
            best_qi = Qi
            best_metrics = (Ql, Qa, Qs, Qi)
            best_aligned_imit = eval_imit
           
    return best_aligned_imit, best_metrics

def update_originals_window():
    global fig_originals
    if not SHOW_ALL_ORIGINALS: return
   
    num_patterns = len(reference_patterns)
    if num_patterns == 0: return

    if fig_originals is None:
        fig_originals = plt.figure(figsize=(10, 6))
        fig_originals.canvas.manager.set_window_title("Diccionario de Patrones Originales")
    else:
        fig_originals.clf() 

    if PLOT_SUPERPUESTAS:
        ax = fig_originals.add_subplot(111)
        ax.set_title("Todos los Patrones Originales (Referencia)")
        for name, vectors in reference_patterns.items():
            plot_vecs = list(vectors)
            if not USE_FIRST_ANGLE and len(plot_vecs) > 0:
                plot_vecs[0] = (plot_vecs[0][0], 0.0)
            xs, ys = calc_coords(plot_vecs)
            ax.plot(xs, ys, '.-', label=f"{name} (N={len(plot_vecs)})")
        ax.grid(True)
        ax.axis('equal')
        ax.legend()
    else:
        cols = math.ceil(math.sqrt(num_patterns))
        rows = math.ceil(num_patterns / cols)
        fig_originals.suptitle("Diccionario de Patrones Originales (Separados)", fontsize=12)
        for i, (name, vectors) in enumerate(reference_patterns.items(), 1):
            ax = fig_originals.add_subplot(rows, cols, i)
            plot_vecs = list(vectors)
            if not USE_FIRST_ANGLE and len(plot_vecs) > 0:
                plot_vecs[0] = (plot_vecs[0][0], 0.0)
            xs, ys = calc_coords(plot_vecs)
            ax.plot(xs, ys, '.-', label=f"{name}")
            ax.set_title(f"{name} (N={len(plot_vecs)})", fontsize=10)
            ax.grid(True)
            ax.axis('equal')
           
    fig_originals.tight_layout()
    if not PLOT_SUPERPUESTAS:
        fig_originals.subplots_adjust(top=0.90) 
    fig_originals.canvas.draw_idle()

def create_dashboard(robot_id, imit_name, imit_vectors, orig_name, orig_vectors, metrics):
    if not SHOW_COMPARISON_WINDOWS: return
   
    Ql, Qa, Qs, Qi = metrics
    fig = plt.figure(figsize=(14, 5))
    window_title = f"Evaluación de Imitación | {robot_id} ({imit_name}) vs {orig_name}"
    fig.canvas.manager.set_window_title(window_title)
   
    plot_orig = list(orig_vectors)
    if not USE_FIRST_ANGLE and len(plot_orig) > 0: plot_orig[0] = (plot_orig[0][0], 0.0)
    orig_xs, orig_ys = calc_coords(plot_orig)
    imit_xs, imit_ys = calc_coords(imit_vectors)
   
    # --- CONSTRUCCIÓN DEL BLOQUE DE TEXTO DETALLADO ---
    text_info = f"--- MÉTRICAS DE CALIDAD ---\n"
    text_info += f"Q_l (Longitud):  {Ql:.4f}\n"
    text_info += f"Q_a (Ángulos):   {Qa:.4f}\n"
    text_info += f"Q_s (Segmentos): {Qs:.4f}\n"
    text_info += f"Q_i (Global):    {Qi:.4f}\n\n"
   
    text_info += f"--- PATRÓN ORIGINAL ---\n"
    text_info += f"Ref: {orig_name} (N={len(orig_vectors)})\n"
    for i, (d, a) in enumerate(orig_vectors, 1):
        if i == 1 and not USE_FIRST_ANGLE:
            ang_str = "0.00° (Ángulo direccional)"
        else:
            ang_str = f"{a:.2f}°"
        text_info += f" S{i}: D={d:.2f}, A={ang_str}\n"
       
    text_info += f"\n--- PATRÓN IMITADO (ALINEADO) ---\n"
    text_info += f"Robot: {robot_id} - {imit_name} (N={len(imit_vectors)})\n"
    for i, (d, a) in enumerate(imit_vectors, 1):
        if i == 1 and not USE_FIRST_ANGLE:
            ang_str = "0.00° (Ángulo direccional)"
        else:
            ang_str = f"{a:.2f}°"
        text_info += f" S{i}: D={d:.2f}, A={ang_str}\n"

    # --- DISEÑO ESPACIAL CORREGIDO ---
    if PLOT_SUPERPUESTAS:
        # Ventana estructurada exactamente en 2 partes
        ax1 = plt.subplot(121)
        ax_txt = plt.subplot(122)
       
        ax1.plot(orig_xs, orig_ys, 'b.-', label=f"Original ({orig_name})")
        ax1.plot(imit_xs, imit_ys, 'r.--', label=f"Imitado ({imit_name})")
        ax1.set_title("Comparativa Superpuesta")
        ax1.legend(); ax1.grid(True); ax1.axis('equal')
    else:
        # Ventana estructurada exactamente en 3 partes
        ax1 = plt.subplot(131)
        ax2 = plt.subplot(132)
        ax_txt = plt.subplot(133)
       
        ax1.plot(orig_xs, orig_ys, 'b.-', label="Original")
        ax1.set_title(f"Original: {orig_name}")
        ax1.grid(True); ax1.axis('equal'); ax1.legend()
       
        ax2.plot(imit_xs, imit_ys, 'r.-', label="Imitado")
        ax2.set_title(f"Aprendido por {robot_id}")
        ax2.grid(True); ax2.axis('equal'); ax2.legend()
       
    ax_txt.axis('off')
    ax_txt.text(0.0, 0.95, text_info, transform=ax_txt.transAxes, 
                fontsize=9, va='top', family='monospace', 
                bbox=dict(boxstyle='round', facecolor='whitesmoke', alpha=0.9))
               
    plt.tight_layout()
    if SAVE_IMAGES:
        filename = os.path.join(IMAGE_DIR, f"QI_{robot_id}_{imit_name}_vs_{orig_name}.png")
        plt.savefig(filename)
       
    active_figs.append(fig)
    if len(active_figs) > MAX_WINDOWS:
        old = active_figs.pop(0)
        plt.close(old)
    fig.canvas.draw_idle()

def stdin_reader(q, stop_event):
    try:
        while True:
            raw = sys.stdin.readline()
            if raw == '': break
            line = raw.strip()
            if not line: continue
            if END_PATTERN in line:
                stop_event.set()
                break
            parsed = parse_line(line)
            if parsed: q.put(parsed)
    except Exception: pass
    finally: stop_event.set()

def main():
    if SAVE_IMAGES: os.makedirs(IMAGE_DIR, exist_ok=True)
    q = queue.Queue()
    stop_event = threading.Event()
   
    reader_thread = threading.Thread(target=stdin_reader, args=(q, stop_event), daemon=True)
    reader_thread.start()
   
    plt.ion()
    print("[Análisis Calidad] Esperando patrones...", flush=True)
   
    while not stop_event.is_set() or not q.empty():
        try:
            while True:
                data = q.get_nowait()
                role = data['role']
               
                if role == "Original":
                    for p_name, vectors in data['patterns'].items():
                        if p_name != "default" and p_name not in reference_patterns:
                            cleaned_vecs = clean_vectors(vectors, is_original=True)
                            reference_patterns[p_name] = cleaned_vecs
                            print(f"[Análisis Calidad] [+] Nuevo patrón Original guardado: {p_name} ({len(cleaned_vecs)} segmentos válidos)", flush=True)
                            update_originals_window()
                           
                elif role == "Imitado":
                    if not reference_patterns: continue
                       
                    for imit_name, imit_vectors in data['patterns'].items():
                        cleaned_imit = clean_vectors(imit_vectors, is_original=False)
                        best_qi = -float('inf')
                        best_orig_name, best_metrics = None, None
                        final_aligned_imit = None
                       
                        for orig_name, orig_vectors in reference_patterns.items():
                            aligned_imit, metrics = evaluate_against_reference(orig_vectors, cleaned_imit)
                            if metrics[3] > best_qi:
                                best_qi = metrics[3]
                                best_orig_name = orig_name
                                best_metrics = metrics
                                final_aligned_imit = aligned_imit
                                
                        if best_orig_name:
                            print(f"[Análisis Calidad] [*] {data['id']} ({imit_name}) imita mejor a '{best_orig_name}' con Qi = {best_qi:.4f}", flush=True)
                            print(f"[Análisis Calidad] [DATA_METRICS] Ql={best_metrics[0]:.4f} Qa={best_metrics[1]:.4f} Qs={best_metrics[2]:.4f} Qi={best_metrics[3]:.4f}", flush=True)
                           
                            if SHOW_COMPARISON_WINDOWS:
                                create_dashboard(data['id'], imit_name, final_aligned_imit, best_orig_name, reference_patterns[best_orig_name], best_metrics)
                                
        except queue.Empty: 
            pass
       
        if not stop_event.is_set():
            plt.pause(0.1)

    plt.ioff()
    print("\n[Análisis Calidad] Simulación finalizada.", flush=True)
   
    if SHOW_COMPARISON_WINDOWS or SHOW_ALL_ORIGINALS:
        if AUTO_CLOSE_WINDOWS:
            print(f"[Análisis Calidad] Las ventanas se cerrarán automáticamente en {AUTO_CLOSE_TIME} segundos...", flush=True)
            plt.pause(AUTO_CLOSE_TIME)
        else:
            print("[Análisis Calidad] Esperando cierre manual de las ventanas de gráficas...", flush=True)
            plt.show(block=True)

if __name__ == '__main__':
    main()