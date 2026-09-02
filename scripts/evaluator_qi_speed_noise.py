#!/usr/bin/env python3
"""
evaluator_qi_speed_noise.py

controllers compatibles: Tr6, Tr7, Ks1

Evaluador pasivo en 2D (Ruido vs Velocidad).
- Escucha la salida de multi_run_sim.py.
- Procesa combinaciones de velocidad (ej. speed_dem) y ruido (ej. NOISE_LEVEL_DIST).
- Actualiza las gráficas al concluir cada ciclo completo del experimento (N/N).
- Ajusta el formato automáticamente si recibe una variable de ruido o un par.
- Permite omitir configuraciones sin datos rellenando con NaN y visualizando de forma punteada.
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
MOSTRAR_LINEAS_VIVO      = True
MOSTRAR_HEATMAP_VIVO     = True

GUARDAR_RESULTADOS       = True  
CARPETA_RESULTADOS       = "Resultados/Robots/Qupa/Tr7/Pentagono/Evaluator_Speed_Noise"

#PATRON_OBJETIVO          = "triangulo"
PATRON_OBJETIVO          = "Pentágono_irregular"

# PATRON_2                 = "Pentágono_irregular"

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
    speed_val = 0.0
    noises = {}
    
    for k, v in params.items():
        if "speed" in k.lower() or "velocidad" in k.lower():
            speed_val = v
        else:
            noises[k] = v
            
    primary_noise_val = list(noises.values())[0] if noises else 0.0
    primary_noise_key = list(noises.keys())[0] if noises else "N/A"
    
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
    speeds = sorted(list(set(r['speed'] for r in resultados)))
    noises_num = sorted(list(set(r['noise_num'] for r in resultados)))
    noises_str = sorted(list(set(r['noise_str'] for r in resultados)), key=lambda x: float(x.split('\n')[0]))
    
    if ax_lines:
        ax_lines.clear()
        
        # Limpieza segura para el eje twin de varianza sin duplicar
        for ax in fig.axes:
            if ax != ax_lines and ax != ax_heat:
                ax.remove()
                
        ax_var = ax_lines.twinx()
        cmap = plt.get_cmap('tab10')
        max_var = 0.01
        
        for idx, s in enumerate(speeds):
            data_s = [r for r in resultados if r['speed'] == s]
            data_s.sort(key=lambda x: x['noise_num'])
            
            x_vals = np.array([r['noise_num'] for r in data_s], dtype=float)
            y_vals = np.array([r['qi_mean'] for r in data_s], dtype=float)
            var_vals = np.array([r['qi_var'] for r in data_s], dtype=float)
            
            valid_vars = var_vals[~np.isnan(var_vals)]
            if len(valid_vars) > 0:
                max_var = max(max_var, np.nanmax(valid_vars))
                
            y_errs = np.array([math.sqrt(v) if not np.isnan(v) else 0.0 for v in var_vals])
            color = cmap(idx % 10)
            
            # 1. Dibujamos línea punteada que une los datos saltándose los NaN
            valid_mask = ~np.isnan(y_vals)
            if np.sum(valid_mask) > 1:
                ax_lines.plot(x_vals[valid_mask], y_vals[valid_mask], linestyle=':', color=color, alpha=0.7)
            
            # 2. Dibujamos los puntos y líneas reales (Se romperán naturalmente si hay un NaN)
            ax_lines.errorbar(x_vals, y_vals, yerr=y_errs, fmt='o-', label=f'Speed: {s:.1f}', capsize=4, color=color, lw=2)
            
            # 3. Dibujamos la varianza en el segundo eje (Derecha)
            if len(valid_vars) > 0:
                ax_var.plot(x_vals, var_vals, linestyle='--', marker='x', alpha=0.4, color=color)

        ax_lines.set_title('Robustez ante Ruido según Velocidad', fontweight='bold')
        ax_lines.set_xlabel(f'Ruido Base ({noise_key})')
        ax_lines.set_ylabel(f'Calidad (Qi - {PATRON_OBJETIVO})')
        ax_lines.set_ylim(-0.05, 1.05)
        ax_lines.grid(True, linestyle='-', alpha=0.3)
        if speeds:
            ax_lines.legend(loc='upper left')
            
        ax_var.set_ylabel('Varianza (Qi)', color='gray')
        ax_var.set_ylim(0, max_var * 1.2) 
        ax_var.tick_params(axis='y', labelcolor='gray')

    if ax_heat:
        ax_heat.clear()
        if len(speeds) > 0 and len(noises_num) > 0:
            Z = np.zeros((len(speeds), len(noises_num)))
            Z[:] = np.nan 
            
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
            
            for i in range(len(speeds)):
                for j in range(len(noises_num)):
                    if not np.isnan(Z[i, j]):
                        text_color = "black" if 0.3 < Z[i, j] < 0.8 else "white"
                        ax_heat.text(j, i, f"{Z[i, j]:.2f}", ha="center", va="center", color=text_color, fontweight='bold')
                    else:
                        ax_heat.text(j, i, "-", ha="center", va="center", color="gray", fontweight='bold')

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
        
        if cols == 2: ax_lines, ax_heat = axes[0], axes[1]
        elif MOSTRAR_LINEAS_VIVO: ax_lines, ax_heat = axes, None
        else: ax_lines, ax_heat = None, axes
            
        plt.tight_layout(pad=3.0)
        plt.show(block=False)

    resultados_totales = []
    cfg_actual = None
    
    cfg_qi = []
    sim_max_qi = None
    noise_key_global = "Ruido"
    line_counter = 0

    for linea_bruta in sys.stdin:
        line_counter += 1
        linea = ANSI_ESCAPE.sub('', linea_bruta.strip())
        
        if linea.startswith("[MULTI_CONFIG]"):
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_qi = []
            
        elif linea.startswith("[MULTI_END_ALL]"):
            break
            
        elif linea.startswith("[SIM_START]"):
            sim_max_qi = None
            
        elif linea.startswith("[SIM_END]"):
            if sim_max_qi is not None:
                cfg_qi.append(sim_max_qi)
            
            # Formato esperado: [SIM_END] 3/3
            match = re.search(r'\[SIM_END\]\s+(\d+)/(\d+)', linea)
            if match:
                j = int(match.group(1))
                n = int(match.group(2))
                
                # Renderiza al terminar todas las simulaciones de la configuración actual
                if j == n and cfg_actual:
                    speed_v, noise_num, noise_key, noise_str = parse_parameters(cfg_actual['params'])
                    noise_key_global = noise_key
                    
                    # Si no hay datos, guarda np.nan (se creará la casilla en blanco y puntuado en gráfica)
                    qi_mean = statistics.mean(cfg_qi) if len(cfg_qi) > 0 else np.nan
                    qi_var = statistics.variance(cfg_qi) if len(cfg_qi) > 1 else (0.0 if len(cfg_qi) == 1 else np.nan)
                    
                    res = {
                        'id': cfg_actual['id'],
                        'speed': speed_v,
                        'noise_num': noise_num,
                        'noise_str': noise_str,
                        'qi_mean': qi_mean,
                        'qi_var': qi_var
                    }
                    res.update(cfg_actual['params'])
                    resultados_totales.append(res)
                    
                    if MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO:
                        actualizar_graficas(fig, ax_lines, ax_heat, resultados_totales, noise_key_global)

        else:
            match_qi = REGEX_QI.search(linea)
            if match_qi:
                patron = match_qi.group(1).strip()
                qi_val = float(match_qi.group(2))
                if patron == PATRON_OBJETIVO:
                    sim_max_qi = max(sim_max_qi or 0, qi_val)

        if (MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO) and line_counter % 1000 == 0:
            fig.canvas.flush_events()

    if resultados_totales and GUARDAR_RESULTADOS:
        print("\n" + "="*70)
        print("[Evaluador 2D] Finalizando y exportando reportes...")
        os.makedirs(CARPETA_RESULTADOS, exist_ok=True)
        
        with open(CSV_OUTPUT, mode='w', newline='') as f:
            writer = csv.writer(f)
            vars_keys = [k for k in resultados_totales[0].keys() if k not in ['id', 'speed', 'noise_num', 'noise_str', 'qi_mean', 'qi_var']]
            # Se verá la Variación (Qi_Var) añadida en la tabla CSV
            headers = ['Config_ID'] + vars_keys + [f'Qi_Mean_{PATRON_OBJETIVO}', f'Qi_Var_{PATRON_OBJETIVO}']
            writer.writerow(headers)
            
            for r in resultados_totales:
                fila = [r['id']] + [r[k] for k in vars_keys] + [r['qi_mean'], r['qi_var']]
                writer.writerow(fila)
        print(f"[Evaluador 2D] CSV guardado en: {CSV_OUTPUT}")
        
        if MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO:
            plt.figure(fig.number) 
            plt.savefig(GRAFICA_OUTPUT, dpi=200, bbox_inches='tight')
            print(f"[Evaluador 2D] Gráfica combinada guardada en: {GRAFICA_OUTPUT}")
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()