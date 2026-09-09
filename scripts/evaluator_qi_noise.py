#!/usr/bin/env python3
"""
evaluator_qi_noise.py

controllers compatibles: Ks1

Evaluador pasivo (Requiere ser lanzado por multi_run_sim.py para recibir datos por stdin).
- Escucha la salida de multi_run_sim.
- Limpia caracteres ANSI (colores de terminal) para leer correctamente el EMP/RMSE y Qi.
- Muestra una interfaz en vivo modular (Gráfica de barras, Gráfica de líneas, Tabla).
- Soporta variables de ruido emparejadas (Modo 1 de multi_run_sim).
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
# ¿Qué ventanas quieres ver en tiempo real durante la simulación?
MOSTRAR_BARRAS           = True
MOSTRAR_LINEAS           = True
MOSTRAR_TABLA            = True

# ¿Qué archivos quieres que se guarden al finalizar todos los experimentos?
GUARDAR_BARRAS           = True
GUARDAR_LINEAS           = True
GUARDAR_TABLA            = True
GUARDAR_CSV              = True

# ¿Dónde quieres guardar los archivos resultantes del experimento de ruido?
CARPETA_RESULTADOS       = "Resultados/Robots/Qupa/Ks1/Evaluator_Ruido"

# Patrones geométricos base contra los cuales contrastará la Calidad (Qi) 
PATRON_1         = "triangulo"
PATRON_2         = "Pentágono_irregular"

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
# ---------- 3. FUNCIONES DE APOYO Y DIBUJADO ----------
# =============================================================================
def parse_variables(params_dict):
    """ Convierte un diccionario de variables en cadenas de texto legibles para ejes y tablas """
    keys = list(params_dict.keys())
    vals = list(params_dict.values())
    
    if len(keys) > 1:
        keys_str = " / ".join(keys)
        vals_str = " / ".join([f"{v:.3f}" for v in vals])
    elif len(keys) == 1:
        keys_str = keys[0]
        vals_str = f"{vals[0]:.3f}"
    else:
        keys_str = "Variable"
        vals_str = "0.000"
        
    return keys_str, vals_str

def dibujar_graficas(resultados, var_keys_str, ax_bar=None, ax_line=None, ax_table=None):
    """ Dibuja dinámicamente en los ejes que se le pasen (sirve para vista en vivo y para guardado) """
    
    labels_x  = [r['val_str'] for r in resultados]
    x_pos     = range(len(labels_x))
    
    medias_p1 = [r['p1_media'] for r in resultados]
    std_p1    = [math.sqrt(r['p1_var']) for r in resultados]
    medias_p2 = [r['p2_media'] for r in resultados]
    std_p2    = [math.sqrt(r['p2_var']) for r in resultados]

    # 1. GRÁFICA DE BARRAS
    if ax_bar is not None:
        ax_bar.clear()
        width = 0.35  
        ax_bar.bar([p - width/2 for p in x_pos], medias_p1, width, yerr=std_p1, label=PATRON_1, capsize=5, color='royalblue', alpha=0.8, edgecolor='black')
        ax_bar.bar([p + width/2 for p in x_pos], medias_p2, width, yerr=std_p2, label=PATRON_2, capsize=5, color='crimson', alpha=0.8, edgecolor='black')
        ax_bar.set_ylabel('Calidad de Imitación (Qi)')
        ax_bar.set_title('Robustez ante Ruido (Barras)', fontweight='bold')
        ax_bar.set_xticks(list(x_pos))
        ax_bar.set_xticklabels(labels_x, rotation=15 if len(labels_x[0]) > 8 else 0)
        ax_bar.set_xlabel(f"Variables: {var_keys_str}")
        ax_bar.set_ylim(-0.05, 1.05)
        ax_bar.legend(loc='upper right')
        ax_bar.grid(axis='y', linestyle='--', alpha=0.7)

    # 2. GRÁFICA DE LÍNEAS
    if ax_line is not None:
        ax_line.clear()
        ax_line.errorbar(x_pos, medias_p1, yerr=std_p1, label=PATRON_1, fmt='-o', color='royalblue', capsize=5, lw=2)
        ax_line.errorbar(x_pos, medias_p2, yerr=std_p2, label=PATRON_2, fmt='-s', color='crimson', capsize=5, lw=2)
        ax_line.set_ylabel('Calidad de Imitación (Qi)')
        ax_line.set_title('Robustez ante Ruido (Líneas)', fontweight='bold')
        ax_line.set_xticks(list(x_pos))
        ax_line.set_xticklabels(labels_x, rotation=15 if len(labels_x[0]) > 8 else 0)
        ax_line.set_xlabel(f"Variables: {var_keys_str}")
        ax_line.set_ylim(-0.05, 1.05)
        ax_line.legend(loc='upper right')
        ax_line.grid(True, linestyle='--', alpha=0.7)

    # 3. TABLA DE DATOS
    if ax_table is not None:
        ax_table.clear()
        ax_table.axis('off')
        ax_table.axis('tight')
        
        # Formatear el header para que encaje bien
        header_var = var_keys_str.replace(" / ", "\n") 
        cols = ['Exp', header_var, 'EMP (cm)', 'RMSE (cm)', f'Qi {PATRON_1[:4]}', f'Qi {PATRON_2[:4]}']
        table_data = []
        
        for r in resultados:
            table_data.append([
                r['id'], 
                r['val_str'].replace(" / ", "\n"), 
                f"{r['emp_media']:.2f}", 
                f"{r['rmse_media']:.2f}",
                f"{r['p1_media']:.4f}", 
                f"{r['p2_media']:.4f}"
            ])
            
        if table_data:
            tabla = ax_table.table(cellText=table_data, colLabels=cols, loc='center', cellLoc='center')
            tabla.auto_set_font_size(False)
            tabla.set_fontsize(10)
            tabla.scale(1.0, 2.0)
            ax_table.set_title('Resumen Tabular de Resultados', fontweight='bold', pad=20)


# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    # Inicialización condicional de ventanas en vivo
    fig_bar, ax_bar = None, None
    fig_line, ax_line = None, None
    fig_table, ax_table = None, None
    
    hay_ventanas_vivas = MOSTRAR_BARRAS or MOSTRAR_LINEAS or MOSTRAR_TABLA

    if hay_ventanas_vivas:
        plt.ion()
        if MOSTRAR_BARRAS:
            fig_bar, ax_bar = plt.subplots(figsize=(8, 6))
            fig_bar.canvas.manager.set_window_title('Evaluador Ruido - Barras')
            fig_bar.show()
        if MOSTRAR_LINEAS:
            fig_line, ax_line = plt.subplots(figsize=(8, 6))
            fig_line.canvas.manager.set_window_title('Evaluador Ruido - Líneas')
            fig_line.show()
        if MOSTRAR_TABLA:
            fig_table, ax_table = plt.subplots(figsize=(10, 4))
            fig_table.canvas.manager.set_window_title('Evaluador Ruido - Tabla')
            fig_table.show()

    resultados_totales = []
    cfg_actual = None
    var_keys_global = "Variable"
    
    # Acumuladores 
    cfg_qi_p1, cfg_qi_p2 = [], []
    cfg_emp, cfg_mse, cfg_rmse = [], [], []

    sim_max_qi_p1, sim_max_qi_p2 = None, None
    sim_err_lineal, sim_err_cuad = [], []
    last_orig_ang, last_orig_dist = None, None
    line_counter = 0

    # Lectura entubada por stdin (Recibiendo datos desde multi_run_sim.py)
    for linea_bruta in sys.stdin:
        line_counter += 1
        linea = ANSI_ESCAPE.sub('', linea_bruta.strip())
        
        # 1. CONTROL DE ORQUESTADOR
        if linea.startswith("[MULTI_CONFIG]") or linea.startswith("[MULTI_END_ALL]"):
            
            # Guardamos y graficamos la configuración que acaba de terminar
            if cfg_actual and len(cfg_emp) > 0:
                keys_str, vals_str = parse_variables(cfg_actual['params'])
                var_keys_global = keys_str
                
                res = {
                    'id': cfg_actual['id'],
                    'params_dict': cfg_actual['params'],
                    'val_str': vals_str,
                    'emp_media': statistics.mean(cfg_emp),
                    'mse_media': statistics.mean(cfg_mse),
                    'rmse_media': statistics.mean(cfg_rmse),
                    'p1_media': statistics.mean(cfg_qi_p1) if cfg_qi_p1 else 0.0,
                    'p1_var': statistics.variance(cfg_qi_p1) if len(cfg_qi_p1) > 1 else 0.0,
                    'p2_media': statistics.mean(cfg_qi_p2) if cfg_qi_p2 else 0.0,
                    'p2_var': statistics.variance(cfg_qi_p2) if len(cfg_qi_p2) > 1 else 0.0
                }
                resultados_totales.append(res)
                
                if hay_ventanas_vivas:
                    dibujar_graficas(resultados_totales, var_keys_global, ax_bar, ax_line, ax_table)
                    if fig_bar: fig_bar.canvas.draw_idle()
                    if fig_line: fig_line.canvas.draw_idle()
                    if fig_table: fig_table.canvas.draw_idle()
                    plt.pause(0.05)

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

        # Flush ligero para evitar el lag a nivel de SO
        if hay_ventanas_vivas and line_counter % 1000 == 0:
            if fig_bar: fig_bar.canvas.flush_events()
            if fig_line: fig_line.canvas.flush_events()
            if fig_table: fig_table.canvas.flush_events()


    # =========================================================================
    # ---------- GUARDADO FINAL (CSV Y GRÁFICAS) ----------
    # =========================================================================
    if resultados_totales and (GUARDAR_BARRAS or GUARDAR_LINEAS or GUARDAR_TABLA or GUARDAR_CSV):
        print("\n" + "="*70)
        print("[Evaluador Ruido] Finalizando y exportando reportes...")
        os.makedirs(CARPETA_RESULTADOS, exist_ok=True)
        
        # --- 1. GUARDAR CSV ---
        if GUARDAR_CSV:
            csv_path = f"{CARPETA_RESULTADOS}/analisis_ruido.csv"
            with open(csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                vars_keys = list(resultados_totales[0]['params_dict'].keys())
                
                headers = ['Config_ID'] + vars_keys + ['EMP_Promedio', 'MSE_Prom', 'RMSE_Prom',
                           f'Media_{PATRON_1}', f'Var_{PATRON_1}', f'Media_{PATRON_2}', f'Var_{PATRON_2}']
                writer.writerow(headers)
                
                for r in resultados_totales:
                    fila = [r['id']] + [r['params_dict'][k] for k in vars_keys] + [
                            r['emp_media'], r['mse_media'], r['rmse_media'],
                            r['p1_media'], r['p1_var'], r['p2_media'], r['p2_var']]
                    writer.writerow(fila)
            print(f"[Evaluador Ruido] CSV guardado en: {csv_path}")

        # --- 2. GUARDAR GRÁFICAS ---
        # Si elegiste guardar algo que NO estabas mostrando en vivo, lo creamos de fondo
        if GUARDAR_BARRAS:
            if not fig_bar:
                fig_bar, ax_bar = plt.subplots(figsize=(8, 6))
                dibujar_graficas(resultados_totales, var_keys_global, ax_bar=ax_bar)
            path = f"{CARPETA_RESULTADOS}/grafica_barras.png"
            fig_bar.savefig(path, dpi=200, bbox_inches='tight')
            print(f"[Evaluador Ruido] Barras guardadas en: {path}")

        if GUARDAR_LINEAS:
            if not fig_line:
                fig_line, ax_line = plt.subplots(figsize=(8, 6))
                dibujar_graficas(resultados_totales, var_keys_global, ax_line=ax_line)
            path = f"{CARPETA_RESULTADOS}/grafica_lineas.png"
            fig_line.savefig(path, dpi=200, bbox_inches='tight')
            print(f"[Evaluador Ruido] Líneas guardadas en: {path}")

        if GUARDAR_TABLA:
            if not fig_table:
                fig_table, ax_table = plt.subplots(figsize=(10, 4))
                dibujar_graficas(resultados_totales, var_keys_global, ax_table=ax_table)
            path = f"{CARPETA_RESULTADOS}/tabla_datos.png"
            fig_table.savefig(path, dpi=200, bbox_inches='tight')
            print(f"[Evaluador Ruido] Tabla guardada en: {path}")

        # Mantener las ventanas abiertas si estabas en modo en vivo
        if hay_ventanas_vivas:
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()