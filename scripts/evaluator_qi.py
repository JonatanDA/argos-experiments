#!/usr/bin/env python3
"""
evaluator_qi.py

Evaluador pasivo que extrae la Calidad de Imitación (Qi) desde la salida de 
tracker_calidad_imitacion.py.
Genera una gráfica EN VIVO de Qi con su varianza vs la variable barrida.

DEPENDENCIAS REQUERIDAS:
- Necesita que 'tracker_calidad_imitacion.py' esté activo en run_sim.py
"""

import sys
import os
import re
import json
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# ---------- CONFIGURACIÓN PRINCIPAL ----------
# =============================================================================
# Nombre del patrón de comportamiento al que se le hará seguimiento
TARGET_PATTERN          = "triangulo"  

# Configuración de guardado
SAVE_PLOT_ENABLED       = True
PLOTS_OUTPUT_DIR        = "Resultados/Robots/Qupa/Tr7/Pentagono/Evaluator_Qi"

# Configuración visual
SHOW_VARIANCE_IN_PLOTS  = True

# =============================================================================
# ---------- EXPRESIONES REGULARES ----------
# =============================================================================
# Regex para limpiar colores bash ANSI de la terminal
ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

# Regex para extraer el patrón imitado y el valor de Qi. 
# Busca coincidir con: [Análisis Calidad] [*] Qupa4 (p1) imita mejor a 'triangulo' con Qi = 0.9635
QI_REGEX = re.compile(r"imita mejor a '([^']+)' con Qi = ([\d\.]+)")

# =============================================================================
# ---------- FUNCIONES ----------
# =============================================================================
def actualizar_grafica_en_vivo(ax, fig, titulo, label_x, x_vals, y_mean, y_std, color, fmt):
    """ Función auxiliar para redibujar la gráfica interactiva """
    ax.clear()
    ax.set_title(titulo, fontweight='bold', fontsize=12)
    ax.set_xlabel(label_x, fontsize=10)
    ax.set_ylabel("Calidad de Imitación (Qi)", fontsize=10)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # Fijamos el límite de Y de 0 a 1 ya que Qi es un porcentaje métrico (0.0 a 1.0)
    ax.set_ylim(-0.05, 1.05) 
    
    if SHOW_VARIANCE_IN_PLOTS and len(x_vals) > 0:
        ax.errorbar(x_vals, y_mean, yerr=y_std, fmt=fmt, color=color, capsize=5, ecolor='red', lw=2)
    elif len(x_vals) > 0:
        ax.plot(x_vals, y_mean, fmt, color=color, lw=2)
        
    fig.canvas.draw_idle()
    plt.pause(0.01) # Pausa ultra-corta solo para renderizar UI

def main():
    # Inicializar modo interactivo de Matplotlib
    plt.ion()
    
    fig_qi, ax_qi = plt.subplots(figsize=(8, 5))
    fig_qi.canvas.manager.set_window_title(f'Calidad de Imitación - {TARGET_PATTERN}')
    plt.show(block=False)

    # Acumuladores globales para configuraciones terminadas
    x_vals, y_mean, y_std = [], [], []
    
    cfg_actual = None
    current_x_val = 0.0
    nombre_var_eje_x = "Variable"
    
    # Acumulador para las repeticiones (simulaciones) de la configuración ACTUAL
    current_cfg_qi_list = []
    
    # Bandera para saber si en la simulación actual ya recibimos al menos un dato
    qi_received_in_current_sim = False

    line_counter = 0

    print(f"[Evaluador Qi] Escuchando datos... Filtrando por patrón: '{TARGET_PATTERN}'\n")

    for linea in sys.stdin:
        linea = linea.strip()
        if not linea: continue
        line_counter += 1
        
        # 1. PROTOCOLO ORQUESTADOR (MULTI_RUN)
        if linea.startswith("[MULTI_CONFIG]"):
            # Si hay datos de la config anterior, los consolidamos en los históricos
            if cfg_actual and len(current_cfg_qi_list) > 0:
                x_vals.append(current_x_val)
                y_mean.append(np.mean(current_cfg_qi_list))
                y_std.append(np.std(current_cfg_qi_list))

            # Preparamos la nueva configuración
            data_str = linea.replace("[MULTI_CONFIG]", "").strip()
            cfg_actual = json.loads(data_str)
            
            # Extraer nombre y valor de la variable de barrido (ej: NOISE_BASE_RADIUS)
            nombre_var_eje_x = list(cfg_actual['params'].keys())[0]
            current_x_val = cfg_actual['params'][nombre_var_eje_x]
            
            # Reiniciar acumuladores para la nueva variable
            current_cfg_qi_list = []
            
        elif linea.startswith("[SIM_START]"):
            qi_received_in_current_sim = False
            
        elif linea.startswith("[SIM_END]"):
            # Si acabó la simulación y el tracker nunca imprimió el valor, asumimos Qi = 0
            if not qi_received_in_current_sim:
                current_cfg_qi_list.append(0.0)
                
                # Proyectar listas temporales para graficar en vivo el punto en cero
                px = list(x_vals) + [current_x_val]
                py = list(y_mean) + [np.mean(current_cfg_qi_list)]
                pe = list(y_std) + [np.std(current_cfg_qi_list)]
                
                titulo = f'Promedio Qi vs {nombre_var_eje_x} (Patrón: {TARGET_PATTERN})'
                actualizar_grafica_en_vivo(ax_qi, fig_qi, titulo, nombre_var_eje_x, px, py, pe, 'purple', '-o')

        elif linea.startswith("[MULTI_END_ALL]"):
            # Consolidamos la última configuración procesada
            if cfg_actual and len(current_cfg_qi_list) > 0:
                x_vals.append(current_x_val)
                y_mean.append(np.mean(current_cfg_qi_list))
                y_std.append(np.std(current_cfg_qi_list))
                
                titulo = f'Promedio Qi vs {nombre_var_eje_x} (Patrón: {TARGET_PATTERN})'
                actualizar_grafica_en_vivo(ax_qi, fig_qi, titulo, nombre_var_eje_x, x_vals, y_mean, y_std, 'purple', '-o')
            break

        # 2. EXTRACCIÓN DE DATOS DESDE EL TRACKER
        else:
            clean_line = ANSI_ESCAPE.sub('', linea)
            match = QI_REGEX.search(clean_line)
            
            if match:
                patron_leido = match.group(1).strip()
                
                # Solo procesar si es el comportamiento objetivo
                if patron_leido == TARGET_PATTERN:
                    qi_val = float(match.group(2))
                    
                    if not qi_received_in_current_sim:
                        # Es el primer dato de esta simulación
                        current_cfg_qi_list.append(qi_val)
                        qi_received_in_current_sim = True
                    else:
                        # Si el tracker imprime varias veces en la misma simulación, sobreescribimos 
                        # con el valor más reciente para quedarnos con el resultado final/mejor.
                        current_cfg_qi_list[-1] = qi_val 

                    # ACTUALIZAR GRÁFICA EN VIVO SOLAMENTE CUANDO LLEGA UN DATO
                    # Proyectamos las listas combinando el histórico + estado actual
                    px = list(x_vals) + [current_x_val]
                    py = list(y_mean) + [np.mean(current_cfg_qi_list)]
                    pe = list(y_std) + [np.std(current_cfg_qi_list)]
                    
                    titulo = f'Promedio Qi vs {nombre_var_eje_x} (Patrón: {TARGET_PATTERN})'
                    actualizar_grafica_en_vivo(ax_qi, fig_qi, titulo, nombre_var_eje_x, px, py, pe, 'purple', '-o')
                            
        # Permitir que la interfaz del OS no se congele 
        if line_counter % 100 == 0:
            fig_qi.canvas.flush_events()

    # =========================================================================
    # ---------- GUARDADO Y CIERRE FINAL ----------
    # =========================================================================
    if len(x_vals) > 0 and SAVE_PLOT_ENABLED:
        print("\n[Evaluador Qi] Generando y guardando archivos finales...")
        os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)
        
        # Nombre de archivo dinámico basado en el patrón analizado
        path_img = os.path.join(PLOTS_OUTPUT_DIR, f"Qi_vs_var_{TARGET_PATTERN}.png")
        fig_qi.savefig(path_img, dpi=200, bbox_inches='tight')
        
        print(f"[Evaluador Qi] Gráfica guardada en: {path_img}")
        
    if len(x_vals) > 0:
        # Detener modo interactivo pero mantener la ventana abierta para inspección
        plt.ioff()
        plt.show() 

if __name__ == '__main__':
    main()