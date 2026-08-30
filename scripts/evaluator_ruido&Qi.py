#!/usr/bin/env python3
"""
evaluator_ruido&Qi.py

Evaluador pasivo (Requiere ser lanzado por multi_run_sim.py para recibir datos por stdin).
- Escucha la salida de multi_run_sim.
- Limpia caracteres ANSI (colores de terminal) para leer correctamente el EMP/RMSE y Qi.
- Muestra una interfaz combinada EN VIVO (Gráfica de barras + Tabla).
- Calcula la relación de error cuadrático por inyección de ruido estadístico.

DEPENDENCIAS LUA REQUERIDAS:
- evaluator_ruido
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
import matplotlib.pyplot as plt

# =============================================================================
# ---------- 1. VARIABLES DE CONTROL ----------
# =============================================================================
# ¿Quieres ver cómo se dibujan los gráficos y la tabla en tiempo real?
MOSTRAR_EN_VIVO          = True  

# ¿Quieres que se guarden los resultados (Imagen combinada y CSV) al terminar todas las configs?
GUARDAR_RESULTADOS       = True  

# ¿Dónde quieres guardar los archivos resultantes del experimento de ruido?
CARPETA_RESULTADOS       = "Resultados/Ruido_Estadistico"

# Patrones geométricos base contra los cuales contrastará la Calidad (Qi) 
# Deben existir exactamente con este nombre en la salida del tracker.
PATRON_1         = "triangulo"
PATRON_2         = "Pentágono_irregular"

# Rutas dinámicas donde se guardarán los resultados si GUARDAR_RESULTADOS = True
CSV_OUTPUT       = f"{CARPETA_RESULTADOS}/analisis_ruido.csv"
GRAFICA_OUTPUT   = f"{CARPETA_RESULTADOS}/grafica_y_tabla_ruido.png"

# =============================================================================
# ---------- 2. EXPRESIONES REGULARES ----------
# =============================================================================
# Filtro vital para limpiar los colores de la terminal bash que arruinan la lectura regex
ANSI_ESCAPE    = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

# Atrapamos la métrica consolidada Qi que produce tracker_calidad_imitacion.py
REGEX_QI       = re.compile(r"\*.*imita mejor a '([^']+)' con Qi = ([-+]?[0-9]*\.?[0-9]+)")

# Atrapamos los reportes puros de ruido enviados directamente desde ARGoS LUA
REGEX_ORIGINAL = re.compile(r"Blob detectado:\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+)")
REGEX_RUIDOSO  = re.compile(r"Blob ruidoso detectado:\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+)")

# =============================================================================
# ---------- 3. INTERFAZ GRÁFICA EN VIVO ----------
# =============================================================================
def actualizar_interfaz_vivo(fig, ax_bar, ax_table, resultados, var_name):
    """ Dibuja el gráfico de barras y la tabla de resumen en la misma ventana de manera concurrente """
    
    # 1. ACTUALIZAR GRÁFICA DE BARRAS (PANEL SUPERIOR)
    ax_bar.clear()
    labels_x  = [f"{r[var_name]:.3f}" for r in resultados]
    x_pos     = range(len(labels_x))
    width     = 0.35  
    
    medias_p1 = [r['p1_media'] for r in resultados]
    std_p1    = [math.sqrt(r['p1_var']) for r in resultados]
    medias_p2 = [r['p2_media'] for r in resultados]
    std_p2    = [math.sqrt(r['p2_var']) for r in resultados]
    
    ax_bar.bar([p - width/2 for p in x_pos], medias_p1, width, yerr=std_p1, label=PATRON_1, capsize=5, color='royalblue', alpha=0.8, edgecolor='black')
    ax_bar.bar([p + width/2 for p in x_pos], medias_p2, width, yerr=std_p2, label=PATRON_2, capsize=5, color='crimson', alpha=0.8, edgecolor='black')
    
    ax_bar.set_ylabel('Calidad de Imitación (Qi)')
    ax_bar.set_title('Robustez del Algoritmo ante Ruido - Resultados en Vivo', fontweight='bold')
    ax_bar.set_xticks(list(x_pos))
    ax_bar.set_xticklabels(labels_x)
    ax_bar.set_xlabel(f"Variable Modificada: {var_name}")
    ax_bar.set_ylim(-0.05, 1.05)
    ax_bar.legend(loc='upper right')
    ax_bar.grid(axis='y', linestyle='--', alpha=0.7)

    # 2. ACTUALIZAR TABLA DE DATOS (PANEL INFERIOR)
    ax_table.clear()
    ax_table.axis('off')
    ax_table.axis('tight')
    
    cols = ['Exp', f'{var_name}', 'EMP (cm)', 'RMSE (cm)', f'Qi {PATRON_1[:4]}', f'Qi {PATRON_2[:4]}']
    table_data = []
    
    for r in resultados:
        table_data.append([
            r['id'], 
            f"{r[var_name]:.4f}", 
            f"{r['emp_media']:.2f}", 
            f"{r['rmse_media']:.2f}",
            f"{r['p1_media']:.4f}", 
            f"{r['p2_media']:.4f}"
        ])
        
    if table_data:
        tabla = ax_table.table(cellText=table_data, colLabels=cols, loc='center', cellLoc='center')
        tabla.auto_set_font_size(False)
        tabla.set_fontsize(10)
        tabla.scale(1.0, 1.5)

    # Forzar el redibujado de la ventana sin congelar el código principal (modo interactivo)
    fig.canvas.draw_idle()
    plt.pause(0.05)

# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    # Inicialización de la ventana (Figura con 2 filas: 1 Gráfica superior, 2 Tabla inferior)
    if MOSTRAR_EN_VIVO:
        plt.ion()
        fig, (ax_bar, ax_table) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [2.5, 1]})
        fig.canvas.manager.set_window_title('Evaluador de Ruido - Tiempo Real')
        plt.show(block=False)

    resultados_totales = []
    cfg_actual = None
    
    # Acumuladores 
    cfg_qi_p1, cfg_qi_p2 = [], []
    cfg_emp, cfg_mse, cfg_rmse = [], [], []

    sim_max_qi_p1, sim_max_qi_p2 = None, None
    sim_err_lineal, sim_err_cuad = [], []
    last_orig_ang, last_orig_dist = None, None

    # Lectura entubada por stdin (Recibiendo datos desde multi_run_sim.py)
    for linea_bruta in sys.stdin:
        # ¡Punto Clave! Quitamos los colores ANSI para que la Regex funcione bien
        linea = ANSI_ESCAPE.sub('', linea_bruta.strip())
        
        # 1. CONTROL DE ORQUESTADOR
        if linea.startswith("[MULTI_CONFIG]") or linea.startswith("[MULTI_END_ALL]"):
            
            # Guardamos y graficamos la configuración que acaba de terminar
            if cfg_actual and len(cfg_emp) > 0:
                res = {
                    'id': cfg_actual['id'],
                    'emp_media': statistics.mean(cfg_emp),
                    'mse_media': statistics.mean(cfg_mse),
                    'rmse_media': statistics.mean(cfg_rmse),
                    'p1_media': statistics.mean(cfg_qi_p1) if cfg_qi_p1 else 0.0,
                    'p1_var': statistics.variance(cfg_qi_p1) if len(cfg_qi_p1) > 1 else 0.0,
                    'p2_media': statistics.mean(cfg_qi_p2) if cfg_qi_p2 else 0.0,
                    'p2_var': statistics.variance(cfg_qi_p2) if len(cfg_qi_p2) > 1 else 0.0
                }
                res.update(cfg_actual['params'])
                resultados_totales.append(res)
                
                var_name = list(cfg_actual['params'].keys())[0]
                
                if MOSTRAR_EN_VIVO:
                    actualizar_interfaz_vivo(fig, ax_bar, ax_table, resultados_totales, var_name)

            if linea.startswith("[MULTI_END_ALL]"):
                break
                
            # Preparamos la siguiente
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_qi_p1, cfg_qi_p2 = [], []
            cfg_emp, cfg_mse, cfg_rmse = [], [], []
            
        elif linea.startswith("[SIM_START]"):
            sim_max_qi_p1, sim_max_qi_p2 = None, None
            sim_err_lineal, sim_err_cuad = [], []
            last_orig_ang, last_orig_dist = None, None
            
        elif linea.startswith("[SIM_END]"):
            if sim_max_qi_p1 is not None: cfg_qi_p1.append(sim_max_qi_p1)
            if sim_max_qi_p2 is not None: cfg_qi_p2.append(sim_max_qi_p2)
            
            s_emp = statistics.mean(sim_err_lineal) if sim_err_lineal else 0.0
            s_mse = statistics.mean(sim_err_cuad) if sim_err_cuad else 0.0
            cfg_emp.append(s_emp)
            cfg_mse.append(s_mse)
            cfg_rmse.append(math.sqrt(s_mse))
            
            if MOSTRAR_EN_VIVO:
                plt.pause(0.001) 

        # 2. EXTRACCIÓN DE MÉTRICAS VIA REGEX
        else:
            match_orig = REGEX_ORIGINAL.search(linea)
            if match_orig:
                last_orig_ang = float(match_orig.group(1))
                last_orig_dist = float(match_orig.group(2))
                continue
            
            match_ruido = REGEX_RUIDOSO.search(linea)
            if match_ruido and last_orig_ang is not None:
                rad_orig = math.radians(last_orig_ang)
                rad_noisy = math.radians(float(match_ruido.group(1)))
                d_noisy = float(match_ruido.group(2))
                delta_th = rad_noisy - rad_orig
                
                e_cuad = (last_orig_dist**2) + (d_noisy**2) - (2 * last_orig_dist * d_noisy * math.cos(delta_th))
                e_cuad = max(0.0, e_cuad)
                
                sim_err_lineal.append(math.sqrt(e_cuad))
                sim_err_cuad.append(e_cuad)
                last_orig_ang = None
                continue

            match_qi = REGEX_QI.search(linea)
            if match_qi:
                patron = match_qi.group(1).strip()
                qi_val = float(match_qi.group(2))
                if patron == PATRON_1:
                    sim_max_qi_p1 = max(sim_max_qi_p1 or 0, qi_val)
                elif patron == PATRON_2:
                    sim_max_qi_p2 = max(sim_max_qi_p2 or 0, qi_val)

    # =========================================================================
    # ---------- GUARDADO FINAL (CSV Y GRÁFICAS) ----------
    # =========================================================================
    if resultados_totales and GUARDAR_RESULTADOS:
        print("\n" + "="*70)
        print("[Evaluador Ruido] Finalizando y exportando reportes...")
        os.makedirs(CARPETA_RESULTADOS, exist_ok=True)
        var_name = list(resultados_totales[0].keys())[-1] 
        
        # 1. Guardar CSV
        with open(CSV_OUTPUT, mode='w', newline='') as f:
            writer = csv.writer(f)
            headers = ['Config_ID', var_name, 'EMP_Promedio', 'MSE_Prom', 'RMSE_Prom',
                       f'Media_{PATRON_1}', f'Var_{PATRON_1}', f'Media_{PATRON_2}', f'Var_{PATRON_2}']
            writer.writerow(headers)
            
            for r in resultados_totales:
                fila = [r['id'], r[var_name], r['emp_media'], r['mse_media'], r['rmse_media'],
                        r['p1_media'], r['p1_var'], r['p2_media'], r['p2_var']]
                writer.writerow(fila)
        print(f"[Evaluador Ruido] CSV guardado en: {CSV_OUTPUT}")
        
        # 2. Guardar Imagen Completa (Gráfica + Tabla)
        if MOSTRAR_EN_VIVO:
            plt.figure(fig.number) 
            plt.savefig(GRAFICA_OUTPUT, dpi=200, bbox_inches='tight')
            print(f"[Evaluador Ruido] Imagen combinada guardada en: {GRAFICA_OUTPUT}")
            
            # Mantener la ventana abierta para que el usuario pueda observarla antes de cerrar
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()