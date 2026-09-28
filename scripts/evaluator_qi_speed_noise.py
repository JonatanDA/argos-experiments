#!/usr/bin/env python3
"""
evaluator_qi_speed_noise.py

controllers compatibles: Tr6, Tr7, Ks1

Evaluador pasivo en 2D (Variables dinámicas).
- Escucha la salida de multi_run_sim.py.
- Maneja dinámicamente múltiples patrones (PATRON_1, PATRON_2, etc.).
- Cada patrón genera su propia ventana de Heatmap (Mediana) y Líneas (configurable: media, mediana, o ambas con dispersión).
- Exporta Gráficas y Tabla de resultados al terminar.
- Calcula estadísticas del error de ruido (Media, Std, Mediana) leyendo las coordenadas polares originales y ruidosas.
- Asigna automáticamente los ejes X y Y dependiendo de las variables que reciba de multi_run_sim.
- Optimizado para evitar congelamientos durante ejecuciones largas.

FORMATO ESPERADO EN LA SALIDA ESTÁNDAR (STDOUT DEL SIMULADOR):
Para que el evaluador procese correctamente los datos, el simulador debe imprimir las siguientes líneas:
- Inicio de configuración:   [MULTI_CONFIG] {"id": 1, "params": {"Velocidad": 5.0, "NOISE_FACTOR": 2.0}}
- Inicio de simulación:      [SIM_START]
- Reporte de Blob Original:  Blob perfecto: angulo = 45.0, distancia = 100.0
- Reporte de Blob Ruidoso:   Blob ruidoso: angulo = 46.5, distancia = 102.3
- Reporte de Qi:             * El enjambre imita mejor a 'triangulo' con Qi = 0.85
- Fin de simulación (x/N):   [SIM_END] 1/3
- Fin de todos los ciclos:   [MULTI_END_ALL]
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
MOSTRAR_TABLA_VIVO       = False   # Se genera solo al final por cuestiones de espacio

GUARDAR_RESULTADOS       = True  
CARPETA_RESULTADOS       = "Resultados/Robots/Qupa/Ks1/Pentagono/Evaluator_Speed_Noise.1"

# NOMBRES DE ARCHIVOS A GUARDAR (El {patron} se reemplazará automáticamente por el nombre del patrón)
NAME_FILE_HEATMAP        = "heatmap_qi_{patron}.png"
NAME_FILE_LINEAS         = "lineas_qi_{patron}.png"
NAME_FILE_TABLA          = "tabla_datos.png"
NAME_FILE_CSV            = "analisis_2d_qi.csv"

# Qué métrica quieres ver en la gráfica de líneas: "media", "mediana" o "ambas"
METRICA_LINEAS           = "ambas"

# Patrones geométricos a evaluar (puedes agregar PATRON_3, PATRON_4, etc.)
# PATRON_1                 = "triangulo"
PATRON_2                 = "Pentágono_irregular"

# Detección automática de patrones declarados
PATRONES = [val for key, val in list(globals().items()) if key.startswith("PATRON_") and isinstance(val, str)]
if not PATRONES:
    PATRONES = ["Desconocido"]

# =============================================================================
# ---------- 2. EXPRESIONES REGULARES ----------
# =============================================================================
ANSI_ESCAPE    = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
REGEX_QI       = re.compile(r"\*.*imita mejor a '([^']+)' con Qi = ([-+]?[0-9]*\.?[0-9]+)")

REGEX_ORIGINAL = re.compile(r"Blob (?:detectado|perfecto):\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)")
REGEX_RUIDOSO  = re.compile(r"Blob ruidoso(?: detectado)?\s*:\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)")

# =============================================================================
# ---------- 3. LOGICA DE PROCESAMIENTO DE PARÁMETROS DINÁMICOS ----------
# =============================================================================
def parse_parameters(params):
    keys = list(params.keys())
    
    if len(keys) == 0:
        return 0.0, "N/A", 0.0, "N/A", "0.000"
        
    if len(keys) == 1:
        # Solo se está barriendo 1 variable
        var_y_name = "N/A"
        var_y_val = 0.0
        var_x_name = keys[0]
        var_x_val = params[keys[0]]
        var_x_str = f"{var_x_val:.3f}"
        
    elif len(keys) == 2:
        # Se están barriendo 2 variables
        var_y_name = keys[0]
        var_y_val = params[keys[0]]
        var_x_name = keys[1]
        var_x_val = params[keys[1]]
        var_x_str = f"{var_x_val:.3f}"
        
    else:
        # Si hay más de 2 variables, se toma la primera para Y, y el resto para X
        var_y_name = keys[0]
        var_y_val = params[keys[0]]
        var_x_name = " / ".join(keys[1:])
        var_x_val = params[keys[1]] 
        var_x_str = "\n".join([f"{params[k]:.3f}" for k in keys[1:]])
        
    return var_y_val, var_y_name, var_x_val, var_x_name, var_x_str

# =============================================================================
# ---------- 4. FUNCIONES DE DIBUJADO ----------
# =============================================================================
def dibujar_grafica_patron(p, resultados, ax_line=None, ax_heat=None):
    if not resultados:
        return
        
    var_y_name = resultados[0]['var_y_name']
    var_x_name = resultados[0]['var_x_name']
        
    vars_y_vals = sorted(list(set(r['var_y_val'] for r in resultados)))
    vars_x_vals = sorted(list(set(r['var_x_val'] for r in resultados)))
    vars_x_strs = sorted(list(set(r['var_x_str'] for r in resultados)), key=lambda x: float(x.split('\n')[0]))
    
    cmap = plt.get_cmap('tab10')
    
    # ---------- GRÁFICA DE LÍNEAS ----------
    if ax_line is not None:
        ax_line.clear()
        
        for idx, y_val in enumerate(vars_y_vals):
            data_y = [r for r in resultados if r['var_y_val'] == y_val]
            data_y.sort(key=lambda x: x['var_x_val'])
            
            x_vals = np.array([r['var_x_val'] for r in data_y], dtype=float)
            medias = np.array([r['qi_stats'][p]['media'] for r in data_y], dtype=float)
            medianas = np.array([r['qi_stats'][p]['mediana'] for r in data_y], dtype=float)
            
            # Desviación estándar (para la media)
            stds = np.array([math.sqrt(r['qi_stats'][p]['var']) if not np.isnan(r['qi_stats'][p]['var']) else 0.0 for r in data_y], dtype=float)
            color = cmap(idx % 10)
            
            lbl_media = f'{var_y_name}: {y_val:.1f} (Media)' if var_y_name != "N/A" else 'Media'
            lbl_mediana = f'{var_y_name}: {y_val:.1f} (Mediana)' if var_y_name != "N/A" else 'Mediana'
            
            # 1. Graficar Media (Con barras de error / desviación estándar)
            if METRICA_LINEAS in ["media", "ambas"]:
                alpha_line = 0.5 if METRICA_LINEAS == "ambas" else 1.0
                lw_line = 1.5 if METRICA_LINEAS == "ambas" else 2.0
                ls_line = '--' if METRICA_LINEAS == "ambas" else '-'
                
                valid_mask = ~np.isnan(medias)
                if np.sum(valid_mask) > 1:
                    ax_line.plot(x_vals[valid_mask], medias[valid_mask], linestyle=':', color=color, alpha=0.4)
                
                ax_line.errorbar(x_vals, medias, yerr=stds, fmt='o', linestyle=ls_line, 
                                 label=lbl_media, capsize=4, color=color, lw=lw_line, alpha=alpha_line, zorder=2)
            
            # 2. Graficar Mediana (Destacada si es 'ambas')
            if METRICA_LINEAS in ["mediana", "ambas"]:
                lw_med = 3.0 if METRICA_LINEAS == "ambas" else 2.0
                
                valid_mask = ~np.isnan(medianas)
                if np.sum(valid_mask) > 1:
                    ax_line.plot(x_vals[valid_mask], medianas[valid_mask], linestyle=':', color=color, alpha=0.4)
                
                ax_line.plot(x_vals, medianas, marker='s', linestyle='-', 
                             label=lbl_mediana if METRICA_LINEAS == "ambas" else (f'{var_y_name}: {y_val:.1f}' if var_y_name != "N/A" else 'Mediana'), 
                             color=color, lw=lw_med, zorder=3)
            
            # 3. Dispersión (Scatter de valores brutos)
            for d in data_y:
                x_val_actual = d['var_x_val']
                y_raws = d['qi_stats'][p]['raw']
                if y_raws:
                    ax_line.scatter([x_val_actual] * len(y_raws), y_raws, color=color, alpha=0.6, s=15, zorder=4)

        titulo_lineas = f'Robustez ante {var_x_name}' + (f' y {var_y_name}' if var_y_name != "N/A" else '') + f' ({p})'
        ax_line.set_title(titulo_lineas, fontweight='bold')
        ax_line.set_xlabel(f'{var_x_name}')
        ax_line.set_ylabel(f'Calidad de Imitación (Qi)')
        ax_line.set_ylim(-0.05, 1.05)
        ax_line.grid(True, linestyle='-', alpha=0.3)
        
        if vars_y_vals:
            ax_line.legend(loc='upper right', fontsize=8)
            
        ax_line.tick_params(axis='x', rotation=45)

    # ---------- MAPA DE CALOR (Basado en Mediana) ----------
    if ax_heat is not None:
        ax_heat.clear()
        if len(vars_y_vals) > 0 and len(vars_x_vals) > 0:
            Z = np.zeros((len(vars_y_vals), len(vars_x_vals)))
            Z[:] = np.nan 
            
            for r in resultados:
                y_idx = vars_y_vals.index(r['var_y_val'])
                x_idx = vars_x_vals.index(r['var_x_val'])
                Z[y_idx, x_idx] = r['qi_stats'][p]['mediana']
                
            cax = ax_heat.imshow(Z, aspect='auto', origin='lower', cmap='RdYlGn', vmin=0, vmax=1)
            
            ax_heat.set_title(f'Mapa de Calor: Límites Operativos ({p})\n[Valores = Mediana]', fontweight='bold')
            ax_heat.set_xticks(range(len(vars_x_vals)))
            ax_heat.set_xticklabels(vars_x_strs)
            ax_heat.set_yticks(range(len(vars_y_vals)))
            ax_heat.set_yticklabels([f"{y:.2f}" for y in vars_y_vals] if var_y_name != "N/A" else ["-"])
            
            ax_heat.set_xlabel(f'{var_x_name}')
            ax_heat.set_ylabel(f'{var_y_name}' if var_y_name != "N/A" else "Fija")
            
            for i in range(len(vars_y_vals)):
                for j in range(len(vars_x_vals)):
                    if not np.isnan(Z[i, j]):
                        text_color = "black" if 0.3 < Z[i, j] < 0.8 else "white"
                        ax_heat.text(j, i, f"{Z[i, j]:.2f}", ha="center", va="center", color=text_color, fontweight='bold')
                    else:
                        ax_heat.text(j, i, "-", ha="center", va="center", color="gray", fontweight='bold')
                        
            ax_heat.tick_params(axis='x', rotation=45)

def dibujar_tabla(resultados, ax_table):
    if not resultados: return
    ax_table.clear()
    ax_table.axis('off')
    ax_table.axis('tight')
    
    var_y_name = resultados[0]['var_y_name']
    var_x_name = resultados[0]['var_x_name']
    
    cols = ['ID', var_y_name if var_y_name != "N/A" else "Var_Y", var_x_name, 'Ruido Media\n(cm)', 'Ruido Std\n(cm)', 'Ruido Mediana\n(cm)']
    for p in PATRONES:
        cols.extend([f'Media\n{p[:4]}', f'Mediana\n{p[:4]}', f'Std\n{p[:4]}'])
        
    table_data = []
    for r in resultados:
        fila = [
            r['id'],
            f"{r['var_y_val']:.2f}" if r['var_y_name'] != "N/A" else "-",
            r['var_x_str'].replace("\n", " / "),
            f"{r['ruido_stats']['media']:.2f}",
            f"{r['ruido_stats']['std']:.2f}",
            f"{r['ruido_stats']['mediana']:.2f}"
        ]
        for p in PATRONES:
            media = r['qi_stats'][p]['media']
            mediana = r['qi_stats'][p]['mediana']
            var = r['qi_stats'][p]['var']
            std = math.sqrt(var) if not np.isnan(var) else 0.0
            
            fila.extend([
                f"{media:.4f}" if not np.isnan(media) else "-",
                f"{mediana:.4f}" if not np.isnan(mediana) else "-",
                f"{std:.4f}" if not np.isnan(std) else "-"
            ])
        table_data.append(fila)
        
    if table_data:
        tabla = ax_table.table(cellText=table_data, colLabels=cols, loc='center', cellLoc='center')
        tabla.auto_set_font_size(False)
        tabla.set_fontsize(9)
        tabla.scale(1.0, 1.6)
        ax_table.set_title(f'Resumen Tabular: {var_x_name}' + (f' y {var_y_name}' if var_y_name != "N/A" else '') + ' vs Calidad (Qi)', fontweight='bold', pad=20)


# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    figs_lines = {}
    axs_lines = {}
    figs_heats = {}
    axs_heats = {}
    
    hay_ventanas_vivas = MOSTRAR_LINEAS_VIVO or MOSTRAR_HEATMAP_VIVO

    if hay_ventanas_vivas:
        plt.ion()
        for p in PATRONES:
            if MOSTRAR_LINEAS_VIVO:
                fl, al = plt.subplots(figsize=(8, 6))
                fl.canvas.manager.set_window_title(f'Evaluador 2D - Líneas [{p}]')
                figs_lines[p] = (fl, al)
                axs_lines[p] = al 
                fl.show()
                
            if MOSTRAR_HEATMAP_VIVO:
                fh, ah = plt.subplots(figsize=(7, 6))
                fh.canvas.manager.set_window_title(f'Evaluador 2D - Heatmap [{p}]')
                figs_heats[p] = (fh, ah)
                axs_heats[p] = ah 
                fh.show()

    resultados_totales = []
    cfg_actual = None
    
    cfg_qi = {p: [] for p in PATRONES}
    sim_max_qi = {p: -float('inf') for p in PATRONES}  # Inicializado a -infinito
    
    cfg_err_media = []
    cfg_err_mediana = []
    sim_err_lineal = []
    last_orig_ang, last_orig_dist = None, None
    
    line_counter = 0

    for linea_bruta in sys.stdin:
        line_counter += 1
        linea = ANSI_ESCAPE.sub('', linea_bruta.strip())
        
        if linea.startswith("[MULTI_CONFIG]"):
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            cfg_qi = {p: [] for p in PATRONES}
            cfg_err_media, cfg_err_mediana = [], []
            
        elif linea.startswith("[MULTI_END_ALL]"):
            break
            
        elif linea.startswith("[SIM_START]"):
            sim_max_qi = {p: -float('inf') for p in PATRONES}  # Reiniciar a -infinito para cada nueva simulación
            sim_err_lineal = []
            last_orig_ang, last_orig_dist = None, None
            
        elif linea.startswith("[SIM_END]"):
            for p in PATRONES:
                # Si no se encontró ningún Qi válido (sigue siendo -inf), guardamos 0.0
                val_to_append = sim_max_qi[p] if sim_max_qi[p] != -float('inf') else 0.0
                cfg_qi[p].append(val_to_append)
            
            # Se extrae el promedio y la mediana del error de ruido por simulación
            s_err_media = statistics.mean(sim_err_lineal) if sim_err_lineal else 0.0
            s_err_mediana = statistics.median(sim_err_lineal) if sim_err_lineal else 0.0
            cfg_err_media.append(s_err_media)
            cfg_err_mediana.append(s_err_mediana)
            
            match = re.search(r'\[SIM_END\]\s+(\d+)/(\d+)', linea)
            if match:
                j = int(match.group(1))
                n = int(match.group(2))
                
                # Renderiza al terminar todas las simulaciones de la config actual
                if j == n and cfg_actual:
                    var_y_val, var_y_name, var_x_val, var_x_name, var_x_str = parse_parameters(cfg_actual['params'])
                    
                    res = {
                        'id': cfg_actual['id'],
                        'var_y_val': var_y_val,
                        'var_y_name': var_y_name,
                        'var_x_val': var_x_val,
                        'var_x_name': var_x_name,
                        'var_x_str': var_x_str,
                        'params_dict': cfg_actual['params'],
                        'ruido_stats': {
                            'media': statistics.mean(cfg_err_media) if cfg_err_media else 0.0,
                            'std': statistics.stdev(cfg_err_media) if len(cfg_err_media) > 1 else 0.0,
                            'mediana': statistics.median(cfg_err_mediana) if cfg_err_mediana else 0.0
                        },
                        'qi_stats': {}
                    }
                    
                    for p in PATRONES:
                        datos = cfg_qi[p]
                        res['qi_stats'][p] = {
                            'media': statistics.mean(datos) if len(datos) > 0 else np.nan,
                            'mediana': statistics.median(datos) if len(datos) > 0 else np.nan,
                            'var': statistics.variance(datos) if len(datos) > 1 else (0.0 if len(datos) == 1 else np.nan),
                            'raw': list(datos)
                        }
                    
                    resultados_totales.append(res)
                    
                    if hay_ventanas_vivas:
                        try:
                            for p in PATRONES:
                                ax_l = axs_lines.get(p)
                                ax_h = axs_heats.get(p)
                                
                                dibujar_grafica_patron(p, resultados_totales, ax_line=ax_l, ax_heat=ax_h)
                                
                                if p in figs_lines:
                                    figs_lines[p][0].tight_layout(pad=2.0)
                                    figs_lines[p][0].canvas.draw_idle()
                                if p in figs_heats:
                                    figs_heats[p][0].tight_layout(pad=2.0)
                                    figs_heats[p][0].canvas.draw_idle()
                                    
                            plt.pause(0.05)
                        except Exception:
                            hay_ventanas_vivas = False

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
                    # Se asigna el valor máximo comparando el valor actual y el nuevo Qi encontrado
                    sim_max_qi[patron] = max(sim_max_qi[patron], qi_val)

        # Prevenir congelamientos asegurando eventos iterativos sin bloquear
        if hay_ventanas_vivas and line_counter % 1000 == 0:
            try:
                for p in PATRONES:
                    if p in figs_lines: figs_lines[p][0].canvas.flush_events()
                    if p in figs_heats: figs_heats[p][0].canvas.flush_events()
            except Exception:
                hay_ventanas_vivas = False

    # =========================================================================
    # ---------- GUARDADO FINAL ----------
    # =========================================================================
    if resultados_totales and GUARDAR_RESULTADOS:
        print("\n" + "="*70)
        print("[Evaluador 2D] Finalizando y exportando reportes...")
        os.makedirs(CARPETA_RESULTADOS, exist_ok=True)
        
        # 1. Guardar CSV
        csv_path = os.path.join(CARPETA_RESULTADOS, NAME_FILE_CSV)
        try:
            with open(csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                vars_keys = list(resultados_totales[0]['params_dict'].keys())
                
                headers = ['Config_ID'] + vars_keys + ['Ruido_Media', 'Ruido_Std', 'Ruido_Mediana']
                for p in PATRONES:
                    headers.extend([f'Media_{p}', f'Mediana_{p}', f'Var_{p}'])
                writer.writerow(headers)
                
                for r in resultados_totales:
                    fila = [r['id']] + [r['params_dict'][k] for k in vars_keys] + [
                           r['ruido_stats']['media'], r['ruido_stats']['std'], r['ruido_stats']['mediana']]
                    for p in PATRONES:
                        fila.extend([r['qi_stats'][p]['media'], r['qi_stats'][p]['mediana'], r['qi_stats'][p]['var']])
                    writer.writerow(fila)
            print(f"[Evaluador 2D] CSV guardado en: {csv_path}")
        except Exception as e:
            print(f"[Evaluador 2D] Error al guardar CSV: {e}")
            
        # 2. Guardar Gráficas para cada patrón (Líneas y Heatmap)
        for p in PATRONES:
            # --- Guardar Líneas ---
            try:
                fl, al = figs_lines.get(p, (None, None))
                if not fl or not plt.fignum_exists(fl.number):
                    fl, al = plt.subplots(figsize=(8, 6))
                
                dibujar_grafica_patron(p, resultados_totales, ax_line=al)
                
                path_l = os.path.join(CARPETA_RESULTADOS, NAME_FILE_LINEAS.format(patron=p))
                fl.savefig(path_l, dpi=200, bbox_inches='tight')
                print(f"[Evaluador 2D] Líneas ({p}) guardadas en: {path_l}")
            except Exception as e:
                print(f"[Evaluador 2D] Error al guardar líneas de {p}: {e}")

            # --- Guardar Heatmap ---
            try:
                fh, ah = figs_heats.get(p, (None, None))
                if not fh or not plt.fignum_exists(fh.number):
                    fh, ah = plt.subplots(figsize=(7, 6))
                
                dibujar_grafica_patron(p, resultados_totales, ax_heat=ah)
                
                path_h = os.path.join(CARPETA_RESULTADOS, NAME_FILE_HEATMAP.format(patron=p))
                fh.savefig(path_h, dpi=200, bbox_inches='tight')
                print(f"[Evaluador 2D] Heatmap ({p}) guardado en: {path_h}")
            except Exception as e:
                print(f"[Evaluador 2D] Error al guardar heatmap de {p}: {e}")

        # 3. Crear y Guardar Tabla Final
        try:
            alto_dinamico = max(5, (len(resultados_totales) * 0.3) + 2)
            fig_table, ax_table = plt.subplots(figsize=(14, alto_dinamico))
            dibujar_tabla(resultados_totales, ax_table)
            
            path_t = os.path.join(CARPETA_RESULTADOS, NAME_FILE_TABLA)
            fig_table.savefig(path_t, dpi=200, bbox_inches='tight')
            print(f"[Evaluador 2D] Tabla de resumen guardada en: {path_t}")
        except Exception as e:
            print(f"[Evaluador 2D] Error al guardar tabla: {e}")

        if hay_ventanas_vivas:
            try:
                plt.ioff()
                plt.show()
            except Exception:
                pass

if __name__ == '__main__':
    main()