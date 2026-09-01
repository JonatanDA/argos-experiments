#!/usr/bin/env python3
"""
evaluator_qi_speed_noise.py

controllers compatibles: Tr6, Tr7, Ks1

Evaluador pasivo en 2D (Ruido vs Velocidad).
- Escucha la salida de multi_run_sim.py.
- Procesa combinaciones de velocidad (ej. speed_dem) y ruido (ej. NOISE_LEVEL_DIST).
- Dibuja simultáneamente una Gráfica de Líneas y un Mapa de Calor (Heatmap) de la Calidad (Qi).
- Ajusta el formato automáticamente si recibe una variable de ruido o un par.

DEPENDENCIAS LUA REQUERIDAS:
- Print_tracker_calidad_imitacion_data
TRACKERS REQUERIDOS EN RUN_SIM:
- tracker_calidad_imitacion.py
"""

import sys
import re
import json
import math
import statistics
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# ---------- 1. VARIABLES DE CONTROL ----------
# =============================================================================
# Opciones de Renderizado (Se pueden activar ambas, dividirá la pantalla)
MOSTRAR_LINEAS_VIVO      = True
MOSTRAR_HEATMAP_VIVO     = True

GUARDAR_RESULTADOS       = True  
CARPETA_RESULTADOS       = "Resultados/Robots/Qupa/Tr6/Evaluator_Speed_Noise"

# Patrón base contra el cual contrastará la Calidad (Qi) 
PATRON_OBJETIVO          = "triangulo"
PATRON_2         = "Pentágono_irregular"

CSV_OUTPUT       = f"{CARPETA_RESULTADOS}/analisis_2d_qi.csv"
GRAFICA_OUTPUT   = f"{CARPETA_RESULTADOS}/grafica_2d_qi.png"

# =============================================================================
# ---------- 2. EXPRESIONES REGULARES ----------
# =============================================================================
ANSI_ESCAPE    = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
REGEX_QI       = re.compile(r"\*.*imita mejor a '([^']+)' con Qi = ([-+]?[0-9]*\.?[0-9]+)")

# =============================================================================
# ---------- 3. LOGICA DE PROCESAMIENTO DE PARÁMETROS ----------
# =============================================================================
def parse_parameters(params):
    """
    Identifica cuál es la variable de velocidad y cuál(es) la de ruido.
    Devuelve:
    - speed_val: Valor numérico de la velocidad
    - primary_noise_val: Valor numérico para usar en el eje X de la gráfica de líneas
    - primary_noise_key: Nombre de la variable principal de ruido
    - paired_noise_str: String formateado (ej. "0.010 / 0.050") ideal para el Heatmap
    """
    speed_val = 0.0
    noises = {}
    
    for k, v in params.items():
        if "speed" in k.lower() or "velocidad" in k.lower():
            speed_val = v
        else:
            noises[k] = v
            
    # Determinamos la variable primaria (Para el eje X continuo de la gráfica de líneas)
    primary_noise_val = list(noises.values())[0] if noises else 0.0
    primary_noise_key = list(noises.keys())[0] if noises else "N/A"
    
    # Creamos el string emparejado para el Heatmap
    if len(noises) > 1:
        vals = list(noises.values())
        paired_noise_str = f"{vals[0]:.3f}\n{vals[1]:.3f}"
    elif len(noises) == 1:
        paired_noise_str = f"{list(noises.values())[0]:.3f}"
    else:
        paired_noise_str = "0.000"
        
    return speed_val, primary_noise_val, primary_noise_key, paired_noise_str

# =============================================================================
# ---------- 4. INTERFAZ GRÁFICA EN VIVO ----------
# =============================================================================
def actualizar_graficas(fig, ax_lines, ax_heat, resultados, noise_key):
    """ Redibuja el gráfico de líneas y el heatmap basándose en los datos acumulados """
    
    # Extraer las velocidades y ruidos únicos registrados hasta el momento
    speeds = sorted(list(set(r['speed'] for r in resultados)))
    noises_num = sorted(list(set(r['noise_num'] for r in resultados)))
    noises_str = sorted(list(set(r['noise_str'] for r in resultados)), key=lambda x: float(x.split('\n')[0]))
    
    # 1. GRÁFICA DE LÍNEAS MÚLTIPLES
    if ax_lines:
        ax_lines.clear()
        
        # Color map para diferenciar fácilmente las líneas
        cmap = plt.get_cmap('tab10')
        
        for idx, s in enumerate(speeds):
            data_s = [r for r in resultados if r['speed'] == s]
            data_s.sort(key=lambda x: x['noise_num'])
            
            x_vals = [r['noise_num'] for r in data_s]
            y_vals = [r['qi_mean'] for r in data_s]
            y_errs = [math.sqrt(r['qi_var']) for r in data_s]
            
            ax_lines.errorbar(x_vals, y_vals, yerr=y_errs, marker='o', 
                              label=f'Speed: {s:.1f}', capsize=4, 
                              color=cmap(idx % 10), lw=2)
                              
        ax_lines.set_title('Robustez ante Ruido según Velocidad', fontweight='bold')
        ax_lines.set_xlabel(f'Ruido Base ({noise_key})')
        ax_lines.set_ylabel(f'Calidad (Qi - {PATRON_OBJETIVO})')
        ax_lines.set_ylim(-0.05, 1.05)
        ax_lines.grid(True, linestyle='--', alpha=0.7)
        if speeds:
            ax_lines.legend()

    # 2. MAPA DE CALOR (HEATMAP)
    if ax_heat:
        ax_heat.clear()
        if len(speeds) > 0 and len(noises_num) > 0:
            Z = np.zeros((len(speeds), len(noises_num)))
            Z[:] = np.nan # Llenar con NaN lo que aún no se ha explorado
            
            for r in resultados:
                s_idx = speeds.index(r['speed'])
                n_idx = noises_num.index(r['noise_num'])
                Z[s_idx, n_idx] = r['qi_mean']
                
            cax = ax_heat.imshow(Z, aspect='auto', origin='lower', cmap='RdYlGn', vmin=0, vmax=1)
            
            ax_heat.set_title('Mapa de Calor: Límites Operativos', fontweight='bold')
            ax_heat.set_xticks(range(len(noises_num)))
            ax_heat.set_xticklabels(noises_str)
            ax_heat.set_yticks(range(len(speeds)))
            ax_heat.set_yticklabels([f"{s:.1f}" for s in speeds])
            ax_heat.set_xlabel('Pares de Ruido (Dist / Ángulo)' if '\n' in noises_str[0] else f'Ruido ({noise_key})')
            ax_heat.set_ylabel('Velocidad (speed_dem)')
            
            # Anotaciones numéricas dentro de las celdas
            for i in range(len(speeds)):
                for j in range(len(noises_num)):
                    if not np.isnan(Z[i, j]):
                        # Texto negro en colores claros (medio), blanco en extremos oscuros
                        text_color = "black" if 0.3 < Z[i, j] < 0.8 else "white"
                        ax_heat.text(j, i, f"{Z[i, j]:.2f}", ha="center", va="center", color=text_color, fontweight='bold')

    # Flush UI
    fig.canvas.draw_idle()
    plt.pause(0.05)

# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    if MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO:
        plt.ion()
        cols = (1 if MOSTRAR_LINEAS_VIVO else 0) + (1 if MOSTRAR_HEATMAP_VIVO else 0)
        fig, axes = plt.subplots(1, cols, figsize=(7 * cols, 6))
        fig.canvas.manager.set_window_title('Evaluador 2D - Ruido vs Velocidad')
        
        if cols == 2:
            ax_lines, ax_heat = axes[0], axes[1]
        elif MOSTRAR_LINEAS_VIVO:
            ax_lines, ax_heat = axes, None
        else:
            ax_lines, ax_heat = None, axes
            
        plt.tight_layout(pad=3.0)
        plt.show(block=False)

    resultados_totales = []
    cfg_actual = None
    
    # Acumuladores de simulación
    cfg_qi = []
    sim_max_qi = None
    noise_key_global = "Ruido"
    line_counter = 0

    for linea_bruta in sys.stdin:
        line_counter += 1
        linea = ANSI_ESCAPE.sub('', linea_bruta.strip())
        
        # 1. CONTROL DE ORQUESTADOR
        if linea.startswith("[MULTI_CONFIG]") or linea.startswith("[MULTI_END_ALL]"):
            
            if cfg_actual and len(cfg_qi) > 0:
                speed_v, noise_num, noise_key, noise_str = parse_parameters(cfg_actual['params'])
                noise_key_global = noise_key
                
                res = {
                    'id': cfg_actual['id'],
                    'speed': speed_v,
                    'noise_num': noise_num,
                    'noise_str': noise_str,
                    'qi_mean': statistics.mean(cfg_qi),
                    'qi_var': statistics.variance(cfg_qi) if len(cfg_qi) > 1 else 0.0
                }
                res.update(cfg_actual['params'])
                resultados_totales.append(res)
                
                if MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO:
                    actualizar_graficas(fig, ax_lines, ax_heat, resultados_totales, noise_key_global)

            if linea.startswith("[MULTI_END_ALL]"):
                break
                
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_qi = []
            
        elif linea.startswith("[SIM_START]"):
            sim_max_qi = None
            
        elif linea.startswith("[SIM_END]"):
            if sim_max_qi is not None:
                cfg_qi.append(sim_max_qi)

        # 2. EXTRACCIÓN DE MÉTRICA (Qi)
        else:
            match_qi = REGEX_QI.search(linea)
            if match_qi:
                patron = match_qi.group(1).strip()
                qi_val = float(match_qi.group(2))
                if patron == PATRON_OBJETIVO:
                    sim_max_qi = max(sim_max_qi or 0, qi_val)

        # Mantiene la UI viva a nivel SO sin lag artificial
        if (MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO) and line_counter % 1000 == 0:
            fig.canvas.flush_events()

    # =========================================================================
    # ---------- GUARDADO FINAL (CSV Y GRÁFICAS) ----------
    # =========================================================================
    if resultados_totales and GUARDAR_RESULTADOS:
        print("\n" + "="*70)
        print("[Evaluador 2D] Finalizando y exportando reportes...")
        os.makedirs(CARPETA_RESULTADOS, exist_ok=True)
        
        # 1. Guardar CSV
        with open(CSV_OUTPUT, mode='w', newline='') as f:
            writer = csv.writer(f)
            
            # Obtener nombres dinámicos de las variables originales
            vars_keys = [k for k in resultados_totales[0].keys() if k not in ['id', 'speed', 'noise_num', 'noise_str', 'qi_mean', 'qi_var']]
            headers = ['Config_ID'] + vars_keys + [f'Qi_Mean_{PATRON_OBJETIVO}', f'Qi_Var_{PATRON_OBJETIVO}']
            writer.writerow(headers)
            
            for r in resultados_totales:
                fila = [r['id']] + [r[k] for k in vars_keys] + [r['qi_mean'], r['qi_var']]
                writer.writerow(fila)
        print(f"[Evaluador 2D] CSV guardado en: {CSV_OUTPUT}")
        
        # 2. Guardar Imagen Completa
        if MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO:
            plt.figure(fig.number) 
            plt.savefig(GRAFICA_OUTPUT, dpi=200, bbox_inches='tight')
            print(f"[Evaluador 2D] Gráfica combinada guardada en: {GRAFICA_OUTPUT}")
            
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()