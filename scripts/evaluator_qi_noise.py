#!/usr/bin/env python3
"""
evaluator_qi_noise.py

controllers compatibles: Ks1

Evaluador pasivo (Requiere ser lanzado por multi_run_sim.py para recibir datos por stdin).
- Escucha la salida de multi_run_sim.
- Muestra una interfaz en vivo modular (Gráfica de barras, Gráfica de líneas, Tabla).
- Calcula Media, Mediana, Tasa de Éxito y superpone dispersión de datos para análisis bimodal.
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
MOSTRAR_BARRAS           = False
MOSTRAR_LINEAS           = False
MOSTRAR_TABLA            = False   # Cambiado a False en vivo (se creará solo al final para evitar problemas de tamaño)

GUARDAR_BARRAS           = True
GUARDAR_LINEAS           = True
GUARDAR_TABLA            = True
GUARDAR_CSV              = True

CARPETA_RESULTADOS       = "Resultados/Robots/Qupa/Ks1/vel=5/Evaluator_ruido/Pr4=40"  # Carpeta donde se guardan las gráficas y tablas

# NOMBRES DE LOS ARCHIVOS GENERADOS
NAME_FILE_BARRAS         = "grafica_barras.png"
NAME_FILE_LINEAS         = "grafica_lineas.png"
NAME_FILE_TABLA          = "tabla_datos.png"
NAME_FILE_CSV            = "analisis_ruido.csv"

# Qué métrica quieres ver en la gráfica de líneas: "media", "mediana" o "ambas"
METRICA_LINEAS           = "ambas"

# Patrones geométricos base
PATRON_1         = "triangulo"
# PATRON_2         = "Pentágono_irregular"

# Cualquier simulación con Qi menor o igual a este umbral se considera "Fallo" para la Tasa de Éxito
UMBRAL_EXITO = 0.85 

PATRONES = [val for key, val in list(globals().items()) if key.startswith("PATRON_") and isinstance(val, str)]
if not PATRONES:
    PATRONES = ["Desconocido"]

# =============================================================================
# ---------- 2. EXPRESIONES REGULARES ----------
# =============================================================================
ANSI_ESCAPE    = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
REGEX_QI       = re.compile(r"\*.*imita mejor a '([^']+)' con Qi = ([-+]?[0-9]*\.?[0-9]+)")

# Actualizado para soportar "Blob perfecto" y notación científica (ej: 3.277e-11)
REGEX_ORIGINAL = re.compile(r"Blob (?:detectado|perfecto):\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)")
REGEX_RUIDOSO  = re.compile(r"Blob ruidoso(?: detectado)?\s*:\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)")

# =============================================================================
# ---------- 3. FUNCIONES DE APOYO Y DIBUJADO ----------
# =============================================================================
def parse_variables(params_dict):
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
    labels_x  = [r['val_str'] for r in resultados]
    x_pos     = range(len(labels_x))
    
    # --- LÓGICA PARA EVITAR SUPERPOSICIÓN DE TEXTOS EN EJE X ---
    max_ticks = 10
    if len(x_pos) > max_ticks:
        step = math.ceil(len(x_pos) / max_ticks)
        ticks_show = list(x_pos)[::step]
        labels_show = [labels_x[i] for i in ticks_show]
    else:
        ticks_show = list(x_pos)
        labels_show = labels_x

    colores = ['royalblue', 'crimson', 'forestgreen', 'darkorange', 'purple', 'teal']
    marcas  = ['-o', '-s', '-^', '-D', '-v', '-*']
    num_patrones = len(PATRONES)

    # 1. GRÁFICA DE BARRAS (Éxito + Superposición de Media, Varianza y Dispersión Qi)
    if ax_bar is not None:
        ax_bar.clear()
        width = 0.8 / num_patrones if num_patrones > 0 else 0.35  
        
        for idx, p in enumerate(PATRONES):
            tasa_exito = [r['qi_stats'][p]['exito'] / 100.0 for r in resultados]
            offset = (idx - num_patrones / 2.0 + 0.5) * width
            posiciones = [pos + offset for pos in x_pos]
            
            ax_bar.bar(posiciones, tasa_exito, width, label=f"{p} (Tasa Éxito)", 
                       color=colores[idx % len(colores)], alpha=0.5, edgecolor='black', zorder=1)
            
            medias_qi = [r['qi_stats'][p]['media'] for r in resultados]
            stds_qi   = [math.sqrt(r['qi_stats'][p]['var']) for r in resultados]
            ax_bar.errorbar(posiciones, medias_qi, yerr=stds_qi, fmt='none', 
                            ecolor='black', capsize=5, lw=1.5, zorder=2)
            
            for i, r in enumerate(resultados):
                y_vals = r['qi_stats'][p]['raw']
                x_vals = [posiciones[i]] * len(y_vals)
                ax_bar.scatter(x_vals, y_vals, color='black', alpha=0.6, s=15, zorder=3)
            
        ax_bar.set_ylabel('Tasa Éxito (Barras) / Qi (Puntos)')
        ax_bar.set_title('Robustez ante Ruido: Éxito vs Calidad', fontweight='bold')
        ax_bar.set_xticks(ticks_show)
        ax_bar.set_xticklabels(labels_show, rotation=15 if len(labels_show) > 0 and len(labels_show[0]) > 8 else 0)
        ax_bar.set_xlabel(f"Variables: {var_keys_str}")
        ax_bar.set_ylim(-0.05, 1.05)
        ax_bar.legend(loc='upper right')
        ax_bar.grid(axis='y', linestyle='--', alpha=0.7)

    # 2. GRÁFICA DE LÍNEAS (Media, Mediana o Ambas)
    if ax_line is not None:
        ax_line.clear()
        
        for idx, p in enumerate(PATRONES):
            medias = [r['qi_stats'][p]['media'] for r in resultados]
            medianas = [r['qi_stats'][p]['mediana'] for r in resultados]
            stds   = [math.sqrt(r['qi_stats'][p]['var']) for r in resultados]
            
            color_actual = colores[idx % len(colores)]
            marca_char = marcas[idx % len(marcas)][1] 
            
            if METRICA_LINEAS == "media":
                ax_line.errorbar(x_pos, medias, yerr=stds, label=p, fmt=marca_char, linestyle='-',
                                 color=color_actual, capsize=5, lw=2, zorder=2)
            
            elif METRICA_LINEAS == "mediana":
                ax_line.plot(x_pos, medianas, marker=marca_char, linestyle='-',
                             color=color_actual, lw=2, label=p, zorder=2)
            
            elif METRICA_LINEAS == "ambas":
                ax_line.errorbar(x_pos, medias, yerr=stds, label=f"{p} (Media)", fmt=marca_char, linestyle='--',
                                 color=color_actual, capsize=3, lw=1.5, alpha=0.5, zorder=2)
                ax_line.plot(x_pos, medianas, marker=marca_char, linestyle='-',
                             color=color_actual, lw=3, label=f"{p} (Mediana)", alpha=1.0, zorder=3)
            
            for i, r in enumerate(resultados):
                y_vals = r['qi_stats'][p]['raw']
                x_vals = [i] * len(y_vals)
                ax_line.scatter(x_vals, y_vals, color=color_actual, alpha=0.4, s=20, zorder=4)
            
        titulo_lineas = 'Robustez ante Ruido (Líneas + Dispersión)'
        if METRICA_LINEAS == "ambas":
            titulo_lineas = 'Robustez ante Ruido (Mediana vs Media)'
            
        ax_line.set_ylabel('Calidad de Imitación (Qi)')
        ax_line.set_title(titulo_lineas, fontweight='bold')
        ax_line.set_xticks(ticks_show)
        ax_line.set_xticklabels(labels_show, rotation=15 if len(labels_show) > 0 and len(labels_show[0]) > 8 else 0)
        ax_line.set_xlabel(f"Variables: {var_keys_str}")
        ax_line.set_ylim(-0.05, 1.05)
        ax_line.legend(loc='lower left' if METRICA_LINEAS == "ambas" else 'upper right')
        ax_line.grid(True, linestyle='--', alpha=0.7)

    # 3. TABLA DE DATOS (Solo se invocará al final de la simulación)
    if ax_table is not None:
        ax_table.clear()
        ax_table.axis('off')
        ax_table.axis('tight')
        
        header_var = var_keys_str.replace(" / ", "\n") 
        # Añadimos las nuevas columnas de ruido
        cols = ['Exp', header_var, 'Ruido Media\n(cm)', 'Ruido Std\n(cm)', 'Ruido Mediana\n(cm)']
        for p in PATRONES:
            cols.extend([f'Qi Mediana {p[:4]}', f'Tasa Éxito {p[:4]}', f'Qi Media {p[:4]}'])
            
        table_data = []
        for r in resultados:
            fila = [
                r['id'], 
                r['val_str'].replace(" / ", "\n"),
                f"{r['ruido_stats']['media']:.2f}",
                f"{r['ruido_stats']['std']:.2f}",
                f"{r['ruido_stats']['mediana']:.2f}"
            ]
            for p in PATRONES:
                mediana_val = r['qi_stats'][p]['mediana']
                exito_val = r['qi_stats'][p]['exito']
                media_val = r['qi_stats'][p]['media']
                fila.extend([f"{mediana_val:.4f}", f"{exito_val:.0f}%", f"{media_val:.4f}"])
            table_data.append(fila)
            
        if table_data:
            tabla = ax_table.table(cellText=table_data, colLabels=cols, loc='center', cellLoc='center')
            tabla.auto_set_font_size(False)
            tabla.set_fontsize(9)
            tabla.scale(1.0, 1.8)
            ax_table.set_title('Resumen Tabular: Ruido y Calidad de Imitación', fontweight='bold', pad=20)


# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    fig_bar, ax_bar = None, None
    fig_line, ax_line = None, None
    
    hay_ventanas_vivas = MOSTRAR_BARRAS or MOSTRAR_LINEAS

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

    resultados_totales = []
    cfg_actual = None
    var_keys_global = "Variable"
    
    cfg_qi = {p: [] for p in PATRONES}
    cfg_err_media = []
    cfg_err_mediana = []

    sim_max_qi = {p: None for p in PATRONES}
    sim_err_lineal = []
    last_orig_ang, last_orig_dist = None, None
    line_counter = 0

    for linea_bruta in sys.stdin:
        line_counter += 1
        linea = ANSI_ESCAPE.sub('', linea_bruta.strip())
        
        if linea.startswith("[MULTI_CONFIG]") or linea.startswith("[MULTI_END_ALL]"):
            
            if cfg_actual and len(cfg_err_media) > 0:
                keys_str, vals_str = parse_variables(cfg_actual['params'])
                var_keys_global = keys_str
                
                res = {
                    'id': cfg_actual['id'],
                    'params_dict': cfg_actual['params'],
                    'val_str': vals_str,
                    'ruido_stats': {
                        'media': statistics.mean(cfg_err_media) if cfg_err_media else 0.0,
                        'std': statistics.stdev(cfg_err_media) if len(cfg_err_media) > 1 else 0.0,
                        'mediana': statistics.median(cfg_err_mediana) if cfg_err_mediana else 0.0
                    },
                    'qi_stats': {}
                }
                
                for p in PATRONES:
                    datos = cfg_qi[p]
                    if datos:
                        exitos = sum(1 for v in datos if v > UMBRAL_EXITO)
                        pct_exito = (exitos / len(datos)) * 100
                    else:
                        pct_exito = 0.0

                    res['qi_stats'][p] = {
                        'media': statistics.mean(datos) if datos else 0.0,
                        'mediana': statistics.median(datos) if datos else 0.0,
                        'var': statistics.variance(datos) if len(datos) > 1 else 0.0,
                        'raw': list(datos),
                        'exito': pct_exito
                    }
                resultados_totales.append(res)
                
                if hay_ventanas_vivas:
                    try:
                        dibujar_graficas(resultados_totales, var_keys_global, ax_bar, ax_line, ax_table=None)
                        if fig_bar: fig_bar.canvas.draw_idle()
                        if fig_line: fig_line.canvas.draw_idle()
                        plt.pause(0.05)
                    except Exception:
                        hay_ventanas_vivas = False

            if linea.startswith("[MULTI_END_ALL]"):
                break
                
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_qi = {p: [] for p in PATRONES}
            cfg_err_media, cfg_err_mediana = [], []
            
        elif linea.startswith("[SIM_START]"):
            sim_max_qi = {p: None for p in PATRONES}
            sim_err_lineal = []
            last_orig_ang, last_orig_dist = None, None
            
        elif linea.startswith("[SIM_END]"):
            for p in PATRONES:
                cfg_qi[p].append(sim_max_qi[p] if sim_max_qi[p] is not None else 0.0)
            
            # Se extrae el promedio y la mediana del error de ruido por simulación
            s_err_media = statistics.mean(sim_err_lineal) if sim_err_lineal else 0.0
            s_err_mediana = statistics.median(sim_err_lineal) if sim_err_lineal else 0.0
            cfg_err_media.append(s_err_media)
            cfg_err_mediana.append(s_err_mediana)

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
                
                # Fórmula de distancia euclidiana vía ley de cosenos para coordenadas polares
                e_cuad = (last_orig_dist**2) + (d_noisy**2) - (2 * last_orig_dist * d_noisy * math.cos(delta_th))
                e_cuad = max(0.0, e_cuad)
                
                sim_err_lineal.append(math.sqrt(e_cuad))
                last_orig_ang = None
                continue

            match_qi = REGEX_QI.search(linea)
            if match_qi:
                patron = match_qi.group(1).strip()
                qi_val = float(match_qi.group(2))
                if patron in PATRONES:
                    sim_max_qi[patron] = max(sim_max_qi[patron] or 0, qi_val)

        if hay_ventanas_vivas and line_counter % 1000 == 0:
            try:
                if fig_bar: fig_bar.canvas.flush_events()
                if fig_line: fig_line.canvas.flush_events()
            except Exception:
                hay_ventanas_vivas = False


    # =========================================================================
    # ---------- GUARDADO FINAL (CSV Y GRÁFICAS) ----------
    # =========================================================================
    if resultados_totales and (GUARDAR_BARRAS or GUARDAR_LINEAS or GUARDAR_TABLA or GUARDAR_CSV):
        print("\n" + "="*70)
        print("[Evaluador Ruido] Finalizando y exportando reportes...")
        os.makedirs(CARPETA_RESULTADOS, exist_ok=True)
        
        # --- 1. GUARDAR CSV ---
        if GUARDAR_CSV:
            csv_path = os.path.join(CARPETA_RESULTADOS, NAME_FILE_CSV)
            try:
                with open(csv_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    vars_keys = list(resultados_totales[0]['params_dict'].keys())
                    
                    headers = ['Config_ID'] + vars_keys + ['Ruido_Media', 'Ruido_Std', 'Ruido_Mediana']
                    for p in PATRONES:
                        headers.extend([f'Media_{p}', f'Mediana_{p}', f'Tasa_Exito_{p}%', f'Var_{p}'])
                    writer.writerow(headers)
                    
                    for r in resultados_totales:
                        fila = [r['id']] + [r['params_dict'][k] for k in vars_keys] + [
                                r['ruido_stats']['media'], r['ruido_stats']['std'], r['ruido_stats']['mediana']]
                        for p in PATRONES:
                            fila.extend([r['qi_stats'][p]['media'], r['qi_stats'][p]['mediana'], 
                                         r['qi_stats'][p]['exito'], r['qi_stats'][p]['var']])
                        writer.writerow(fila)
                print(f"[Evaluador Ruido] CSV guardado en: {csv_path}")
            except Exception as e:
                print(f"[Evaluador Ruido] Error al guardar CSV: {e}")

        # --- 2. GUARDAR GRÁFICAS ---
        if GUARDAR_BARRAS:
            try:
                if not fig_bar or not plt.fignum_exists(fig_bar.number):
                    fig_bar, ax_bar = plt.subplots(figsize=(8, 6))
                    dibujar_graficas(resultados_totales, var_keys_global, ax_bar=ax_bar)
                path = os.path.join(CARPETA_RESULTADOS, NAME_FILE_BARRAS)
                fig_bar.savefig(path, dpi=200, bbox_inches='tight')
                print(f"[Evaluador Ruido] Barras guardadas en: {path}")
            except Exception as e:
                print(f"[Evaluador Ruido] Error al guardar barras: {e}")

        if GUARDAR_LINEAS:
            try:
                if not fig_line or not plt.fignum_exists(fig_line.number):
                    fig_line, ax_line = plt.subplots(figsize=(8, 6))
                    dibujar_graficas(resultados_totales, var_keys_global, ax_line=ax_line)
                path = os.path.join(CARPETA_RESULTADOS, NAME_FILE_LINEAS)
                fig_line.savefig(path, dpi=200, bbox_inches='tight')
                print(f"[Evaluador Ruido] Líneas guardadas en: {path}")
            except Exception as e:
                print(f"[Evaluador Ruido] Error al guardar líneas: {e}")

        if GUARDAR_TABLA:
            try:
                # Se aumentó el ancho del figsize a 14 para acomodar las nuevas columnas
                alto_dinamico = max(5, (len(resultados_totales) * 0.3) + 2)
                fig_table, ax_table = plt.subplots(figsize=(14, alto_dinamico)) 
                dibujar_graficas(resultados_totales, var_keys_global, ax_table=ax_table)
                path = os.path.join(CARPETA_RESULTADOS, NAME_FILE_TABLA)
                fig_table.savefig(path, dpi=200, bbox_inches='tight')
                print(f"[Evaluador Ruido] Tabla guardada en: {path}")
            except Exception as e:
                print(f"[Evaluador Ruido] Error al guardar tabla: {e}")

        if hay_ventanas_vivas:
            try:
                plt.ioff()
                plt.show()
            except Exception:
                pass

if __name__ == '__main__':
    main()