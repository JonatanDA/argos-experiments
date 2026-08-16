#!/usr/bin/env python3
"""
master_estadistico_ruido.py

- controllers -> Tr6.lua (Qupa)

- scripts: "scripts/tracker_calidad_imitacion.py",

- Ejecuta N pasos de ruido.
- Realiza M experimentos por cada paso.
- Calcula dinámicamente el EMP, MSE y RMSE de los blobs detectados.
- Extrae el Qi máximo para dos patrones específicos por experimento.
- Genera una tabla final, exporta a CSV y crea una gráfica estadística basada en RMSE.
"""

import os
import sys
import subprocess
import re
import shutil
import queue
import threading
import statistics
import csv
import math
import time
import matplotlib.pyplot as plt

# ================= CONFIGURACIÓN DEL EXPERIMENTO =================
NUM_PASOS_N      = 5  
NUM_EXP_M        = 5  

# Rangos de Ruido (Min, Max)
RANGO_RUIDO_DIST = (0.02, 0.08)
RANGO_RUIDO_ANG  = (0.6, 1.4)

# Bandera para habilitar o deshabilitar la visualización de todos los logs en terminal
MOSTRAR_TODOS_LOS_LOGS = True

# Nombres exactos de los patrones originales
PATRON_1         = "triangulo"
PATRON_2         = "Pentágono_irregular"

# Rutas
LUA_SCRIPT       = "experiments/Qupa/Controller/Tr6.lua"
RUN_SIM_SCRIPT   = "scripts/run_sim.py"

# Archivo de salida para los datos
CSV_OUTPUT       = "Resultados/analisis_ruido_qi.csv"
GRAFICA_OUTPUT   = "Resultados/grafica_ruido_qi.png"
# ==============================================================

REGEX_QI       = re.compile(r"\*.*imita mejor a '([^']+)' con Qi = ([-+]?[0-9]*\.?[0-9]+)")
REGEX_ORIGINAL = re.compile(r"Blob detectado:\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+)")
REGEX_RUIDOSO  = re.compile(r"Blob ruidoso detectado:\s*angulo\s*=\s*([-+]?[0-9]*\.?[0-9]+),\s*distancia\s*=\s*([-+]?[0-9]*\.?[0-9]+)")

def calcular_pasos(rango, num_pasos):
    if num_pasos <= 1:
        return [rango[0]]
    paso = (rango[1] - rango[0]) / (num_pasos - 1)
    return [rango[0] + (i * paso) for i in range(num_pasos)]

def modify_lua_noise(backup_path, target_path, dist_val, ang_val):
    with open(backup_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    content = re.sub(r'(NOISE_LEVEL_DIST\s*=\s*)[0-9]*\.?[0-9]+', f'\\g<1>{dist_val:.4f}', content)
    content = re.sub(r'(NOISE_LEVEL_ANGLE\s*=\s*)[0-9]*\.?[0-9]+', f'\\g<1>{ang_val:.4f}', content)
    
    with open(target_path, 'w', encoding='utf-8') as f:
        f.write(content)

def lector_salida(pipe, cola_buffer):
    try:
        for linea in iter(pipe.readline, ''):
            if linea: cola_buffer.put(linea)
    except Exception: pass
    finally: pipe.close()

def graficar_resultados(resultados):
    if not resultados: return
    
    labels_x = [f"EMP: {r['emp_media']:.1f}cm\nRMSE: {r['rmse_media']:.1f}cm" for r in resultados]
    x_pos = range(len(labels_x))
    width = 0.35  
    
    medias_p1 = [r['p1_media'] for r in resultados]
    std_p1 = [math.sqrt(r['p1_var']) for r in resultados]
    
    medias_p2 = [r['p2_media'] for r in resultados]
    std_p2 = [math.sqrt(r['p2_var']) for r in resultados]
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    ax.bar([p - width/2 for p in x_pos], medias_p1, width, yerr=std_p1, 
           label=PATRON_1, capsize=5, color='royalblue', alpha=0.8, edgecolor='black')
    
    ax.bar([p + width/2 for p in x_pos], medias_p2, width, yerr=std_p2, 
           label=PATRON_2, capsize=5, color='crimson', alpha=0.8, edgecolor='black')
    
    ax.set_ylabel('Calidad de Imitación (Qi Promedio)', fontsize=11)
    ax.set_title('Robustez del Algoritmo de Aprendizaje Social ante Ruido\n(Barras de error = ±1 Desviación Estándar)', fontsize=13)
    ax.set_xticks(list(x_pos))
    ax.set_xticklabels(labels_x, fontsize=9)
    ax.set_xlabel('Error Promedio Proyectado (Físico en cm)', fontsize=11)
    ax.set_ylim(-0.05, 1.05)
    
    ax.legend(loc='upper right')
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    os.makedirs(os.path.dirname(GRAFICA_OUTPUT), exist_ok=True)
    plt.savefig(GRAFICA_OUTPUT, dpi=150)
    print(f"\n[+] Gráfica estadística guardada en: {GRAFICA_OUTPUT}")

def main():
    print("="*70)
    print(" INICIANDO EXPERIMENTO ESTADÍSTICO DE SUSCEPTIBILIDAD AL RUIDO")
    print("="*70)
    print(f"Pasos de ruido (N) : {NUM_PASOS_N}")
    print(f"Experimentos/paso (M): {NUM_EXP_M}")
    print(f"Total simulaciones   : {NUM_PASOS_N * NUM_EXP_M}")
    print(f"Modo Debug de Logs   : {'ACTIVADO' if MOSTRAR_TODOS_LOS_LOGS is True else 'DESACTIVADO'}")
    print("="*70)
    
    if not os.path.exists(LUA_SCRIPT):
        print(f"ERROR: No se encontró {LUA_SCRIPT}")
        return

    backup_lua = LUA_SCRIPT + ".bak"
    shutil.copy(LUA_SCRIPT, backup_lua)
    
    valores_dist = calcular_pasos(RANGO_RUIDO_DIST, NUM_PASOS_N)
    valores_ang  = calcular_pasos(RANGO_RUIDO_ANG, NUM_PASOS_N)
    resultados_totales = []

    try:
        for i in range(NUM_PASOS_N):
            dist_actual = valores_dist[i]
            ang_actual = valores_ang[i]
            
            print(f"\n[PASO {i+1}/{NUM_PASOS_N}] Configurando Ruido -> Dist: {dist_actual:.4f}, Ang: {ang_actual:.4f}")
            modify_lua_noise(backup_lua, LUA_SCRIPT, dist_actual, ang_actual)
            
            qi_p1_experimentos = []
            qi_p2_experimentos = []
            emp_experimentos   = []
            mse_experimentos   = []
            rmse_experimentos  = []

            for j in range(NUM_EXP_M):
                num_sim = (i * NUM_EXP_M) + (j + 1)
                print(f"  > Ejecutando exp {j+1}/{NUM_EXP_M} (Sim {num_sim}/{NUM_PASOS_N * NUM_EXP_M})... ", end="", flush=True)
                
                cmd = [sys.executable, RUN_SIM_SCRIPT]
                proceso = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
                
                cola_datos = queue.Queue()
                hilo_lector = threading.Thread(target=lector_salida, args=(proceso.stdout, cola_datos), daemon=True)
                hilo_lector.start()
                
                simulacion_activa = True
                max_qi_p1 = None
                max_qi_p2 = None
                
                last_orig_ang = None
                last_orig_dist = None
                errores_lineales_sim = []
                errores_cuadraticos_sim = []
                
                tiempo_inicio = time.time()
                TIMEOUT_SIMULACION = 90  

                while simulacion_activa or not cola_datos.empty():
                    if time.time() - tiempo_inicio > TIMEOUT_SIMULACION:
                        print("\n[!] ADVERTENCIA: Timeout alcanzado en esta simulación. Forzando cierre...")
                        proceso.terminate()
                        break

                    try:
                        linea = cola_datos.get(timeout=0.05)
                        
                        if MOSTRAR_TODOS_LOS_LOGS is True:
                            sys.stdout.write(linea)
                            sys.stdout.flush()
                        
                        # Búsqueda de métricas Qi
                        match_qi = REGEX_QI.search(linea)
                        if match_qi:
                            patron_evaluado = match_qi.group(1).strip()
                            qi_val = float(match_qi.group(2))
                            
                            if patron_evaluado == PATRON_1:
                                if max_qi_p1 is None or qi_val > max_qi_p1:
                                    max_qi_p1 = qi_val
                            elif patron_evaluado == PATRON_2:
                                if max_qi_p2 is None or qi_val > max_qi_p2:
                                    max_qi_p2 = qi_val
                                    
                        # Lógica de extracción de distancias para EMP, MSE y RMSE
                        match_orig = REGEX_ORIGINAL.search(linea)
                        if match_orig:
                            last_orig_ang = float(match_orig.group(1))
                            last_orig_dist = float(match_orig.group(2))
                            continue
                        
                        match_ruido = REGEX_RUIDOSO.search(linea)
                        if match_ruido and last_orig_ang is not None:
                            noisy_ang = float(match_ruido.group(1))
                            noisy_dist = float(match_ruido.group(2))
                            
                            # Cálculo de error (Ley de los Cosenos)
                            rad_orig = math.radians(last_orig_ang)
                            rad_noisy = math.radians(noisy_ang)
                            delta_theta = rad_noisy - rad_orig
                            
                            e_cuadrado = (last_orig_dist**2) + (noisy_dist**2) - (2 * last_orig_dist * noisy_dist * math.cos(delta_theta))
                            if e_cuadrado < 0: e_cuadrado = 0.0 # Control de imprecisión flotante
                            
                            e_lineal = math.sqrt(e_cuadrado) # Este es el error absoluto de posición (EMP)
                            
                            errores_lineales_sim.append(e_lineal)
                            errores_cuadraticos_sim.append(e_cuadrado)
                            last_orig_ang = None
                            
                    except queue.Empty:
                        if proceso.poll() is not None:
                            simulacion_activa = False
                
                proceso.wait()
                hilo_lector.join()
                
                # Consolidar datos de la simulación
                if max_qi_p1 is not None: qi_p1_experimentos.append(max_qi_p1)
                if max_qi_p2 is not None: qi_p2_experimentos.append(max_qi_p2)
                
                sim_emp = statistics.mean(errores_lineales_sim) if errores_lineales_sim else 0.0
                sim_mse = statistics.mean(errores_cuadraticos_sim) if errores_cuadraticos_sim else 0.0
                sim_rmse = math.sqrt(sim_mse)
                
                emp_experimentos.append(sim_emp)
                mse_experimentos.append(sim_mse)
                rmse_experimentos.append(sim_rmse)
                
                print(f"Listo. [EMP: {sim_emp:.2f}cm] [RMSE: {sim_rmse:.2f}cm] [Qi {PATRON_1}: {max_qi_p1 if max_qi_p1 else 'N/A'}] [Qi {PATRON_2}: {max_qi_p2 if max_qi_p2 else 'N/A'}]")
            
            # --- CÁLCULO ESTADÍSTICO DEL PASO ---
            media_p1 = statistics.mean(qi_p1_experimentos) if qi_p1_experimentos else 0.0
            var_p1   = statistics.variance(qi_p1_experimentos) if len(qi_p1_experimentos) > 1 else 0.0
            
            media_p2 = statistics.mean(qi_p2_experimentos) if qi_p2_experimentos else 0.0
            var_p2   = statistics.variance(qi_p2_experimentos) if len(qi_p2_experimentos) > 1 else 0.0
            
            media_emp = statistics.mean(emp_experimentos) if emp_experimentos else 0.0
            media_mse = statistics.mean(mse_experimentos) if mse_experimentos else 0.0
            media_rmse = statistics.mean(rmse_experimentos) if rmse_experimentos else 0.0
            
            resultados_totales.append({
                'paso': i + 1,
                'ruido_dist': dist_actual,
                'ruido_ang': ang_actual,
                'emp_media': media_emp,
                'mse_media': media_mse,
                'rmse_media': media_rmse,
                'p1_media': media_p1,
                'p1_var': var_p1,
                'p2_media': media_p2,
                'p2_var': var_p2
            })
            
    except KeyboardInterrupt:
        print("\n[!] Experimento interrumpido por el usuario.")
        
    finally:
        if os.path.exists(backup_lua):
            shutil.move(backup_lua, LUA_SCRIPT)
            print("\nControlador Lua restaurado.")

        if resultados_totales:
            print("\n" + "="*125)
            print(f" ANÁLISIS FINAL ({NUM_EXP_M} experimentos por paso)")
            print("="*125)
            print(f"{'Paso':<4} | {'EMP(cm)':<7} | {'MSE':<7} | {'RMSE(cm)':<8} | "
                  f"Prom {PATRON_1[:4]}: {'Media':<7} {'Var':<7} | "
                  f"Prom {PATRON_2[:4]}: {'Media':<7} {'Var':<7}")
            print("-" * 125)
            
            for res in resultados_totales:
                print(f"{res['paso']:<4} | {res['emp_media']:<7.2f} | {res['mse_media']:<7.2f} | {res['rmse_media']:<8.2f} | "
                      f"      {res['p1_media']:<7.4f} {res['p1_var']:<7.4f} | "
                      f"      {res['p2_media']:<7.4f} {res['p2_var']:<7.4f}")
            print("="*125)

            os.makedirs(os.path.dirname(CSV_OUTPUT), exist_ok=True)
            with open(CSV_OUTPUT, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Paso', 'Ruido_Dist_Config', 'Ruido_Ang_Config', 
                                 'EMP_Promedio_cm', 'MSE_Promedio', 'RMSE_Promedio_cm',
                                 f'Media_Qi_{PATRON_1}', f'Varianza_Qi_{PATRON_1}', 
                                 f'Media_Qi_{PATRON_2}', f'Varianza_Qi_{PATRON_2}'])
                for res in resultados_totales:
                    writer.writerow([res['paso'], res['ruido_dist'], res['ruido_ang'], 
                                     res['emp_media'], res['mse_media'], res['rmse_media'],
                                     res['p1_media'], res['p1_var'], 
                                     res['p2_media'], res['p2_var']])
            print(f"Datos estadísticos exportados a: {CSV_OUTPUT}")
            
            graficar_resultados(resultados_totales)

if __name__ == '__main__':
    main()