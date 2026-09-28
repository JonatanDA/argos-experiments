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
import csv
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# ---------- CONFIGURACIÓN ----------
# =============================================================================
METRIC_OTSU_PATTERN     = "Otsu &"
PLOTS_OUTPUT_DIR        = "Resultados/Robots/Qupa/Ks1/vel=5/Evaluator_otsu/Pr4-av=40"  # Carpeta donde se guardan las gráficas y tablas

# NOMBRES DE LOS ARCHIVOS GENERADOS
NAME_FILE_OTSU_PLOT     = "otsu_vs_std.png"
NAME_FILE_AVANCE_PLOT   = "avance_vs_std.png"
NAME_FILE_TABLE         = "tabla_resumen_estadisticas.png"
NAME_FILE_CSV           = "analisis_otsu.csv"


# Variables para guardar o no las gráficas y CSV
SAVE_TABLE_ENABLED      = True
SAVE_OTSU_PLOT          = True
SAVE_AVANCE_PLOT        = True
GUARDAR_CSV             = True

# Variables para decidir si ver o no las gráficas en vivo
SHOW_STD_IN_PLOTS       = True  # Muestra las barras de Desviación Estándar (Std) en las gráficas
SHOW_OTSU_PLOT          = False
SHOW_AVANCE_PLOT        = False

# NUEVO: Control de métricas y agregaciones
METRICA_LINEAS          = "ambas"    # Opciones: "media", "mediana", "ambas"

# AGREGACION_INTRA_SIM: Cómo reducir los N valores impresos de 1 solo experimento a 1 valor.
# - "media": Promedia los valores del experimento.
# - "mediana": Saca la mediana de los valores del experimento.
# - "ambas": Usa la Media del experimento para calcular la Media del grupo de experimentos, 
#            y usa la Mediana del experimento para calcular la Mediana del grupo.
AGREGACION_INTRA_SIM    = "ambas"

# Regex para limpieza
ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
NUMBER_RE   = re.compile(r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?')

def parse_variables(params_dict):
    """ Convierte el diccionario de parámetros en strings para labels """
    keys = list(params_dict.keys())
    vals = list(params_dict.values())
    if len(keys) > 1:
        keys_str = " / ".join(keys)
        vals_str = " / ".join([f"{v:.3f}" if isinstance(v, float) else str(v) for v in vals])
    elif len(keys) == 1:
        keys_str = keys[0]
        vals_str = f"{vals[0]:.3f}" if isinstance(vals[0], float) else str(vals[0])
    else:
        keys_str = "Variable"
        vals_str = "0.000"
    return keys_str, vals_str

def generar_tabla_estadisticas(resultados, output_dir, var_keys_str):
    fig_table, ax_table = plt.subplots(figsize=(11, (len(resultados) * 0.3) + 1.5))
    ax_table.axis('off')
    ax_table.axis('tight')
    
    # Se integra el nombre de la variable debajo del header "Variable Eval"
    header_variable = f"Variable Eval\n({var_keys_str})"
    cols = ['Exp', header_variable, 'Media Otsu', 'Mediana Otsu', 'Std Otsu', 'Media Avance', 'Mediana Avance', 'Std Avance']
    table_data = []
    
    for r in resultados:
        table_data.append([
            r['id'],
            r['val_str'],
            f"{r['otsu']['media']:.4f}", f"{r['otsu']['mediana']:.4f}", f"{r['otsu']['std']:.4f}",
            f"{r['avance']['media']:.2f}", f"{r['avance']['mediana']:.2f}", f"{r['avance']['std']:.2f}"
        ])
    
    tabla = ax_table.table(cellText=table_data, colLabels=cols, loc='center', cellLoc='center')
    tabla.auto_set_font_size(False)
    tabla.set_fontsize(9)
    tabla.scale(1.2, 1.5)
    
    plt.title("Resumen Estadístico - Otsu y Avance", pad=20, fontsize=14, fontweight='bold')
    
    if SAVE_TABLE_ENABLED:
        os.makedirs(output_dir, exist_ok=True)
        path = os.path.join(output_dir, NAME_FILE_TABLE)
        plt.savefig(path, dpi=200, bbox_inches='tight')
        print(f"\n[Evaluador Otsu] Tabla guardada en: {path}")

def actualizar_grafica_en_vivo(ax, fig, titulo, label_x, resultados, key_metrica, color, is_live=True):
    """ Función auxiliar para redibujar la gráfica interactiva con media, mediana y scatter """
    ax.clear()
    ax.set_title(titulo, fontweight='bold')
    ax.set_xlabel(label_x)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    x_pos = range(len(resultados))
    labels_x = [r['val_str'] for r in resultados]
    
    medias = [r[key_metrica]['media'] for r in resultados]
    medianas = [r[key_metrica]['mediana'] for r in resultados]
    stds = [r[key_metrica]['std'] for r in resultados]
    
    # Graficar según la métrica deseada
    if METRICA_LINEAS == "media":
        if SHOW_STD_IN_PLOTS:
            ax.errorbar(x_pos, medias, yerr=stds, fmt='-o', color=color, capsize=5, ecolor='red', lw=2, label='Media Global')
        else:
            ax.plot(x_pos, medias, '-o', color=color, lw=2, label='Media Global')
            
    elif METRICA_LINEAS == "mediana":
        ax.plot(x_pos, medianas, '-s', color=color, lw=2, label='Mediana Global')
        
    elif METRICA_LINEAS == "ambas":
        # Media: Punteada, con desviación estándar (si está activa) y más fina
        if SHOW_STD_IN_PLOTS:
            ax.errorbar(x_pos, medias, yerr=stds, fmt='--o', color=color, alpha=0.5, capsize=3, lw=1.5, ecolor='red', label='Media Global')
        else:
            ax.plot(x_pos, medias, '--o', color=color, alpha=0.5, lw=1.5, label='Media Global')
        # Mediana: Línea continua, gruesa y notoria
        ax.plot(x_pos, medianas, '-s', color=color, lw=3, label='Mediana Global')
        
    # Scatter superpuesto con los puntos de cada experimento según AGREGACION_INTRA_SIM
    for i, r in enumerate(resultados):
        x_vals = [i] * len(r[key_metrica]['sims_media']) # Mismo tamaño para media o mediana
        
        # Etiquetar solo en la primera iteración para evitar duplicados en la leyenda
        lbl_media = 'Medias (Exp)' if i == 0 else ""
        lbl_mediana = 'Medianas (Exp)' if i == 0 else ""

        if AGREGACION_INTRA_SIM == "media":
            y_vals = r[key_metrica]['sims_media']
            ax.scatter(x_vals, y_vals, color='black', alpha=0.6, s=20, zorder=4, label=lbl_media)
            
        elif AGREGACION_INTRA_SIM == "mediana":
            y_vals = r[key_metrica]['sims_mediana']
            ax.scatter(x_vals, y_vals, color='black', alpha=0.6, s=20, zorder=4, label=lbl_mediana)
            
        elif AGREGACION_INTRA_SIM == "ambas":
            y_medias = r[key_metrica]['sims_media']
            y_medianas = r[key_metrica]['sims_mediana']
            # Dibujamos las medias en rojo y las medianas en negro
            ax.scatter(x_vals, y_medias, color='red', alpha=0.6, s=20, zorder=4, label=lbl_media)
            ax.scatter(x_vals, y_medianas, color='black', alpha=0.6, s=20, zorder=5, label=lbl_mediana)
            
    ax.set_xticks(list(x_pos))
    ax.set_xticklabels(labels_x, rotation=15 if len(labels_x)>0 and len(labels_x[0])>8 else 0)
    ax.legend(loc='best')
    
    if is_live:
        fig.canvas.draw_idle()
        plt.pause(0.05) # Pausa necesaria para que Matplotlib renderice la UI

def main():
    # Inicializar modo interactivo de Matplotlib
    plt.ion()
    
    fig_otsu, ax_otsu = None, None
    if SHOW_OTSU_PLOT:
        fig_otsu, ax_otsu = plt.subplots(figsize=(8,6))
        fig_otsu.canvas.manager.set_window_title('Evaluador - Separación de Otsu')
    
    fig_avance, ax_avance = None, None
    if SHOW_AVANCE_PLOT:
        fig_avance, ax_avance = plt.subplots(figsize=(8,6))
        fig_avance.canvas.manager.set_window_title('Evaluador - Avance')
    
    if SHOW_OTSU_PLOT or SHOW_AVANCE_PLOT:
        plt.show(block=False)

    resultados_totales = []
    cfg_actual = None
    var_keys_global = "Variable"
    
    # Acumuladores de los N experimentos para la configuración actual (separados en media y mediana)
    cfg_otsu_sims_media = []
    cfg_otsu_sims_mediana = []
    cfg_avance_sims_media = []
    cfg_avance_sims_mediana = []
    
    # Acumuladores temporales de todos los prints de UNA sola simulación
    sim_otsu = []
    sim_avance = []
    
    # Contador para mantener la UI viva sin lag
    line_counter = 0

    for linea in sys.stdin:
        linea = linea.strip()
        line_counter += 1
        
        # 1. PROTOCOLO ORQUESTADOR
        if linea.startswith("[MULTI_CONFIG]") or linea.startswith("[MULTI_END_ALL]"):
            
            # Si ya había datos procesados de una configuración anterior, los empaquetamos
            if cfg_actual and (len(cfg_otsu_sims_media) > 0 or len(cfg_otsu_sims_mediana) > 0):
                keys_str, vals_str = parse_variables(cfg_actual['params'])
                var_keys_global = keys_str
                
                # Seleccionar los arrays a utilizar según AGREGACION_INTRA_SIM
                if AGREGACION_INTRA_SIM == "media":
                    arr_o_media, arr_o_mediana = cfg_otsu_sims_media, cfg_otsu_sims_media
                    arr_a_media, arr_a_mediana = cfg_avance_sims_media, cfg_avance_sims_media
                elif AGREGACION_INTRA_SIM == "mediana":
                    arr_o_media, arr_o_mediana = cfg_otsu_sims_mediana, cfg_otsu_sims_mediana
                    arr_a_media, arr_a_mediana = cfg_avance_sims_mediana, cfg_avance_sims_mediana
                else: # "ambas"
                    arr_o_media, arr_o_mediana = cfg_otsu_sims_media, cfg_otsu_sims_mediana
                    arr_a_media, arr_a_mediana = cfg_avance_sims_media, cfg_avance_sims_mediana
                
                # La Desviación Estándar (Std) se calcula sobre el arreglo inter-simulación (los N valores de las repeticiones)
                res = {
                    'id': cfg_actual.get('id', len(resultados_totales) + 1),
                    'params_dict': cfg_actual['params'],
                    'val_str': vals_str,
                    'otsu': {
                        'media': np.mean(arr_o_media),
                        'mediana': np.median(arr_o_mediana),
                        'std': np.std(arr_o_media),
                        'sims_media': list(arr_o_media),
                        'sims_mediana': list(arr_o_mediana)
                    },
                    'avance': {
                        'media': np.mean(arr_a_media),
                        'mediana': np.median(arr_a_mediana),
                        'std': np.std(arr_a_media),
                        'sims_media': list(arr_a_media),
                        'sims_mediana': list(arr_a_mediana)
                    }
                }
                resultados_totales.append(res)

                # ACTUALIZAR GRÁFICAS EN VIVO SOLO AL CAMBIAR CONFIGURACIÓN
                if SHOW_OTSU_PLOT:
                    actualizar_grafica_en_vivo(ax_otsu, fig_otsu, f'Otsu vs {var_keys_global}', var_keys_global, resultados_totales, 'otsu', 'blue')
                if SHOW_AVANCE_PLOT:
                    actualizar_grafica_en_vivo(ax_avance, fig_avance, f'Avance vs {var_keys_global}', var_keys_global, resultados_totales, 'avance', 'green')

            if linea.startswith("[MULTI_END_ALL]"):
                break

            # Preparamos la nueva configuración
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_otsu_sims_media, cfg_otsu_sims_mediana = [], []
            cfg_avance_sims_media, cfg_avance_sims_mediana = [], []
            
        elif linea.startswith("[SIM_START]"):
            sim_otsu, sim_avance = [], []
            
        elif linea.startswith("[SIM_END]"):
            # Si en el experimento no se recibieron datos, se asumen como 0
            if not sim_otsu: sim_otsu.append(0.0)
            if not sim_avance: sim_avance.append(0.0)

            # Extraemos tanto la media como la mediana intra-simulación
            val_otsu_media = np.mean(sim_otsu)
            val_otsu_mediana = np.median(sim_otsu)
            val_avance_media = np.mean(sim_avance)
            val_avance_mediana = np.median(sim_avance)
                
            cfg_otsu_sims_media.append(val_otsu_media)
            cfg_otsu_sims_mediana.append(val_otsu_mediana)
            cfg_avance_sims_media.append(val_avance_media)
            cfg_avance_sims_mediana.append(val_avance_mediana)

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
            if SHOW_OTSU_PLOT: fig_otsu.canvas.flush_events()
            if SHOW_AVANCE_PLOT: fig_avance.canvas.flush_events()

    # =========================================================================
    # ---------- GUARDADO Y CIERRE FINAL ----------
    # =========================================================================
    if len(resultados_totales) > 0:
        print("\n[Evaluador Otsu] Generando y guardando archivos finales...")
        os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)
        
        # 1. TABLA FINAL
        generar_tabla_estadisticas(resultados_totales, PLOTS_OUTPUT_DIR, var_keys_global)
        
        # 2. GUARDAR CSV
        if GUARDAR_CSV:
            csv_path = os.path.join(PLOTS_OUTPUT_DIR, NAME_FILE_CSV)
            try:
                with open(csv_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    vars_keys = list(resultados_totales[0]['params_dict'].keys())
                    
                    # Calcular el maximo de repeticiones para poner las cabeceras correctamente
                    max_sims = max(len(r['otsu']['sims_mediana']) for r in resultados_totales)
                    
                    headers = ['Exp_ID'] + vars_keys + [
                        'Media_Otsu', 'Mediana_Otsu', 'Std_Otsu',
                        'Media_Avance', 'Mediana_Avance', 'Std_Avance'
                    ]
                    
                    # Agregamos cabeceras para los valores individuales de cada repetición
                    for i in range(max_sims):
                        if AGREGACION_INTRA_SIM == "ambas":
                            headers.extend([
                                f'Otsu_Media_Sim_{i+1}', f'Otsu_Mediana_Sim_{i+1}', 
                                f'Avance_Media_Sim_{i+1}', f'Avance_Mediana_Sim_{i+1}'
                            ])
                        else:
                            headers.extend([f'Otsu_Sim_{i+1}', f'Avance_Sim_{i+1}'])
                        
                    writer.writerow(headers)
                    
                    for r in resultados_totales:
                        fila = [r['id']] + [r['params_dict'][k] for k in vars_keys] + [
                            r['otsu']['media'], r['otsu']['mediana'], r['otsu']['std'],
                            r['avance']['media'], r['avance']['mediana'], r['avance']['std']
                        ]
                        
                        sims_o_mediana = r['otsu']['sims_mediana']
                        sims_o_media = r['otsu']['sims_media']
                        sims_a_mediana = r['avance']['sims_mediana']
                        sims_a_media = r['avance']['sims_media']
                        
                        for i in range(max_sims):
                            v_o_med = sims_o_mediana[i] if i < len(sims_o_mediana) else ""
                            v_o_mea = sims_o_media[i] if i < len(sims_o_media) else ""
                            v_a_med = sims_a_mediana[i] if i < len(sims_a_mediana) else ""
                            v_a_mea = sims_a_media[i] if i < len(sims_a_media) else ""
                            
                            if AGREGACION_INTRA_SIM == "ambas":
                                fila.extend([v_o_mea, v_o_med, v_a_mea, v_a_med])
                            else:
                                fila.extend([v_o_med, v_a_med]) # Mediana o media, ya que son idénticos si no es "ambas"
                            
                        writer.writerow(fila)
                print(f"[Evaluador Otsu] CSV guardado en: {csv_path}")
            except Exception as e:
                print(f"[Evaluador Otsu] Error al guardar CSV: {e}")

        # 3. GUARDAR GRÁFICAS
        if SAVE_OTSU_PLOT:
            if SHOW_OTSU_PLOT:
                fig_otsu.savefig(os.path.join(PLOTS_OUTPUT_DIR, NAME_FILE_OTSU_PLOT), dpi=200, bbox_inches='tight')
            else:
                # Crear y guardar la gráfica en segundo plano si no se está visualizando
                fig_otsu_temp, ax_otsu_temp = plt.subplots(figsize=(8,6))
                actualizar_grafica_en_vivo(ax_otsu_temp, fig_otsu_temp, f'Otsu vs {var_keys_global}', var_keys_global, resultados_totales, 'otsu', 'blue', is_live=False)
                fig_otsu_temp.savefig(os.path.join(PLOTS_OUTPUT_DIR, NAME_FILE_OTSU_PLOT), dpi=200, bbox_inches='tight')
                plt.close(fig_otsu_temp)

        if SAVE_AVANCE_PLOT:
            if SHOW_AVANCE_PLOT:
                fig_avance.savefig(os.path.join(PLOTS_OUTPUT_DIR, NAME_FILE_AVANCE_PLOT), dpi=200, bbox_inches='tight')
            else:
                # Crear y guardar la gráfica en segundo plano si no se está visualizando
                fig_avance_temp, ax_avance_temp = plt.subplots(figsize=(8,6))
                actualizar_grafica_en_vivo(ax_avance_temp, fig_avance_temp, f'Avance vs {var_keys_global}', var_keys_global, resultados_totales, 'avance', 'green', is_live=False)
                fig_avance_temp.savefig(os.path.join(PLOTS_OUTPUT_DIR, NAME_FILE_AVANCE_PLOT), dpi=200, bbox_inches='tight')
                plt.close(fig_avance_temp)
        
        print(f"[Evaluador Otsu] Gráficas guardadas en: {PLOTS_OUTPUT_DIR}")
        
        # Detener modo interactivo pero mantener ventanas abiertas
        plt.ioff()
        if SHOW_OTSU_PLOT or SHOW_AVANCE_PLOT:
            plt.show() 

if __name__ == '__main__':
    main()