#!/usr/bin/env python3
"""
evaluator_otsu.py

Evaluador pasivo que extrae los valores de Otsu y avance.
Genera tablas y actualiza las gráficas EN VIVO según la variable barrida.

DEPENDENCIAS LUA REQUERIDAS:
- Función: `evaluator_otsu(separacion, avance)`
"""

import sys
import os
import re
import json
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# ---------- CONFIGURACIÓN ----------
# =============================================================================
METRIC_OTSU_PATTERN     = "Otsu &"
PLOTS_OUTPUT_DIR        = "Resultados/Robots/Qupa/Ks1/Otsu_Evaluation"
SAVE_TABLE_ENABLED      = True
SHOW_VARIANCE_IN_PLOTS  = True

# Regex para limpieza
ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
NUMBER_RE   = re.compile(r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?')

def generar_tabla_estadisticas(labels_x, m_otsu, s_otsu, m_avance, s_avance, output_dir):
    fig_table, ax_table = plt.subplots(figsize=(8, (len(labels_x) * 0.3) + 1.5))
    ax_table.axis('off')
    ax_table.axis('tight')
    
    table_data = []
    for i in range(len(labels_x)):
        table_data.append([
            f"{labels_x[i]:.2f}", f"{m_otsu[i]:.4f}", f"{s_otsu[i]:.4f}",
            f"{m_avance[i]:.2f}", f"{s_avance[i]:.2f}"
        ])
    
    cols = ['Variable Eval', 'Promedio Otsu', 'Desv. Std Otsu', 'Promedio Avance', 'Desv. Std Avance']
    tabla = ax_table.table(cellText=table_data, colLabels=cols, loc='center', cellLoc='center')
    tabla.scale(1.2, 1.5)
    
    plt.title("Resumen Estadístico - Otsu", pad=20, fontsize=14, fontweight='bold')
    
    if SAVE_TABLE_ENABLED:
        os.makedirs(output_dir, exist_ok=True)
        path = os.path.join(output_dir, "tabla_resumen_estadisticas.png")
        plt.savefig(path, dpi=200, bbox_inches='tight')
        print(f"\n[Evaluador Otsu] Tabla guardada en: {path}")

def actualizar_grafica_en_vivo(ax, fig, titulo, label_x, x_vals, y_mean, y_std, color, fmt):
    """ Función auxiliar para redibujar la gráfica interactiva """
    ax.clear()
    ax.set_title(titulo, fontweight='bold')
    ax.set_xlabel(label_x)
    ax.grid(True, linestyle='--', alpha=0.7)
    if SHOW_VARIANCE_IN_PLOTS:
        ax.errorbar(x_vals, y_mean, yerr=y_std, fmt=fmt, color=color, capsize=5, ecolor='red', lw=2)
    else:
        ax.plot(x_vals, y_mean, fmt, color=color, lw=2)
    fig.canvas.draw_idle()
    plt.pause(0.05) # Pausa necesaria para que Matplotlib renderice la UI

def main():
    # Inicializar modo interactivo de Matplotlib
    plt.ion()
    
    fig_otsu, ax_otsu = plt.subplots(figsize=(7,5))
    fig_otsu.canvas.manager.set_window_title('Promedio - Separación de Otsu')
    
    fig_avance, ax_avance = plt.subplots(figsize=(7,5))
    fig_avance.canvas.manager.set_window_title('Promedio - Mejor Avance')
    
    plt.show(block=False)

    cfg_actual = None
    
    # Acumuladores globales para las gráficas
    x_vals, y_o_mean, y_o_std, y_a_mean, y_a_std = [], [], [], [], []
    
    # Acumuladores de la configuración actual
    cfg_otsu = []
    cfg_avance = []
    
    # Acumuladores temporales de una sola simulación
    sim_otsu = []
    sim_avance = []
    
    nombre_var_eje_x = "Variable"
    
    # Contador para mantener la UI viva sin lag
    line_counter = 0

    for linea in sys.stdin:
        linea = linea.strip()
        line_counter += 1
        
        # 1. PROTOCOLO ORQUESTADOR
        if linea.startswith("[MULTI_CONFIG]"):
            # Si ya había datos procesados de una configuración anterior, los graficamos
            if cfg_actual and len(cfg_otsu) > 0:
                nombre_var_eje_x = list(cfg_actual['params'].keys())[0]
                x_vals.append(cfg_actual['params'][nombre_var_eje_x])
                
                y_o_mean.append(np.mean(cfg_otsu))
                y_o_std.append(np.std(cfg_otsu))
                y_a_mean.append(np.mean(cfg_avance))
                y_a_std.append(np.std(cfg_avance))

                # ACTUALIZAR GRÁFICAS EN VIVO SOLO AL CAMBIAR CONFIGURACIÓN
                actualizar_grafica_en_vivo(ax_otsu, fig_otsu, f'Promedio Otsu vs {nombre_var_eje_x}', nombre_var_eje_x, x_vals, y_o_mean, y_o_std, 'blue', '-o')
                actualizar_grafica_en_vivo(ax_avance, fig_avance, f'Mejor Avance vs {nombre_var_eje_x}', nombre_var_eje_x, x_vals, y_a_mean, y_a_std, 'green', '-s')

            # Preparamos la nueva configuración
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_otsu, cfg_avance = [], []
            
        elif linea.startswith("[SIM_START]"):
            sim_otsu, sim_avance = [], []
            
        elif linea.startswith("[SIM_END]"):
            # Agrupar las métricas de la simulación recién terminada
            if sim_otsu: cfg_otsu.extend(sim_otsu)
            if sim_avance: cfg_avance.extend(sim_avance)

        elif linea.startswith("[MULTI_END_ALL]"):
            # Finalizamos todo, procesamos y graficamos la última configuración pendiente
            if cfg_actual and len(cfg_otsu) > 0:
                nombre_var_eje_x = list(cfg_actual['params'].keys())[0]
                x_vals.append(cfg_actual['params'][nombre_var_eje_x])
                
                y_o_mean.append(np.mean(cfg_otsu))
                y_o_std.append(np.std(cfg_otsu))
                y_a_mean.append(np.mean(cfg_avance))
                y_a_std.append(np.std(cfg_avance))
                
                actualizar_grafica_en_vivo(ax_otsu, fig_otsu, f'Promedio Otsu vs {nombre_var_eje_x}', nombre_var_eje_x, x_vals, y_o_mean, y_o_std, 'blue', '-o')
                actualizar_grafica_en_vivo(ax_avance, fig_avance, f'Mejor Avance vs {nombre_var_eje_x}', nombre_var_eje_x, x_vals, y_a_mean, y_a_std, 'green', '-s')
            break

        # 2. EXTRACCIÓN DE DATOS NORMAL
        else:
            clean_line = ANSI_ESCAPE.sub('', linea)
            if METRIC_OTSU_PATTERN in clean_line:
                parts = clean_line.split('&', 1)
                if len(parts) > 1:
                    nums = NUMBER_RE.findall(parts[1])
                    if len(nums) >= 2:
                        try:
                            sim_otsu.append(float(nums[0].replace(',', '.')))
                            sim_avance.append(float(nums[1].replace(',', '.')))
                        except ValueError:
                            pass
                            
        # Permitir que la UI no se congele a nivel de OS (Costo computacional casi 0, comparado con plt.pause)
        if line_counter % 1000 == 0:
            fig_otsu.canvas.flush_events()
            fig_avance.canvas.flush_events()

    # =========================================================================
    # ---------- GUARDADO Y CIERRE FINAL ----------
    # =========================================================================
    if len(x_vals) > 0:
        print("\n[Evaluador Otsu] Generando y guardando archivos finales...")
        os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)
        
        generar_tabla_estadisticas(x_vals, y_o_mean, y_o_std, y_a_mean, y_a_std, PLOTS_OUTPUT_DIR)

        fig_otsu.savefig(os.path.join(PLOTS_OUTPUT_DIR, "otsu_vs_var.png"), dpi=200)
        fig_avance.savefig(os.path.join(PLOTS_OUTPUT_DIR, "avance_vs_var.png"), dpi=200)
        
        print(f"[Evaluador Otsu] Gráficas guardadas en: {PLOTS_OUTPUT_DIR}")
        
        # Detener modo interactivo pero mantener ventanas abiertas
        plt.ioff()
        plt.show() 

if __name__ == '__main__':
    main()