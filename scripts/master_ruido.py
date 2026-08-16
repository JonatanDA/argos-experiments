#!/usr/bin/env python3

"""
master_ruido.py
- Ejecuta múltiples simulaciones variando gradualmente el ruido de distancia y ángulo.
- Modifica dinámicamente NOISE_LEVEL_DIST y NOISE_LEVEL_ANGLE en el controlador Lua.
- Captura los índices de calidad (Ql, Qa, Qs, Qi) del tracker.
- Grafica los resultados según las preferencias de visualización.
"""

import os
import sys
import subprocess
import re
import shutil
import threading
import queue
import matplotlib.pyplot as plt

# ================= CONFIGURACIÓN DEL EXPERIMENTO =================
NUM_EXPERIMENTOS = 6

# Rangos de Ruido (Min, Max)
RANGO_RUIDO_DIST = (0.0, 0.05)
RANGO_RUIDO_ANG  = (0.0, 0.25)

# Rutas
LUA_SCRIPT = "experiments/Qupa/Controller/Tr6.lua"
RUN_SIM_SCRIPT = "scripts/run_sim.py"

# Opciones de Gráfica
GRAFICAR_TODOS_LOS_INDICES = True  # True: Grafica Ql, Qa, Qs y Qi. False: Solo Qi
SEMITRANSPARENTES          = True  # True: Ql, Qa y Qs tendrán opacidad reducida. False: Misma intensidad.

# Cadenas de detección
END_PATTERN = "End_simulation"
METRICS_PATTERN = "[DATA_METRICS]"

# Opciones de salida
AUTO_CLOSE_WINDOWS = False  
SAVE_PLOTS = True          
PLOTS_DIR = "Resultados"   
# ==============================================================

# Regex para extraer métricas del string: [DATA_METRICS] Ql=0.9 Qa=0.8 Qs=1.0 Qi=0.9
METRICS_RE = re.compile(r'Ql=([0-9.]+) Qa=([0-9.]+) Qs=([0-9.]+) Qi=([0-9.]+)')

def calcular_pasos(rango, num_exp):
    """Calcula los valores interpolados para los experimentos."""
    if num_exp <= 1:
        return [rango[0]]
    paso = (rango[1] - rango[0]) / (num_exp - 1)
    return [rango[0] + (i * paso) for i in range(num_exp)]

def modify_lua_noise(backup_path, target_path, dist_val, ang_val):
    """Sustituye los valores de ruido en el archivo Lua."""
    with open(backup_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Reemplaza NOISE_LEVEL_DIST y NOISE_LEVEL_ANGLE preservando el formato
    content = re.sub(r'(NOISE_LEVEL_DIST\s*=\s*)[0-9]*\.?[0-9]+', f'\\g<1>{dist_val:.4f}', content)
    content = re.sub(r'(NOISE_LEVEL_ANGLE\s*=\s*)[0-9]*\.?[0-9]+', f'\\g<1>{ang_val:.4f}', content)
    
    with open(target_path, 'w', encoding='utf-8') as f:
        f.write(content)

def parse_metrics_line(line):
    """Busca y extrae Ql, Qa, Qs, Qi de la línea del tracker."""
    if METRICS_PATTERN in line:
        match = METRICS_RE.search(line)
        if match:
            return float(match.group(1)), float(match.group(2)), float(match.group(3)), float(match.group(4))
    return None, None, None, None

def lector_salida(pipe, cola_buffer):
    try:
        for linea in iter(pipe.readline, ''):
            if linea: cola_buffer.put(linea)
    except Exception: pass
    finally: pipe.close()

def main():
    print("Iniciando orquestador de Evaluación de Ruido...")
    
    if not os.path.exists(LUA_SCRIPT):
        print(f"ERROR: No se encontró el archivo Lua: {LUA_SCRIPT}")
        return

    backup_lua = LUA_SCRIPT + ".bak"
    shutil.copy(LUA_SCRIPT, backup_lua)
    print(f"Copia de seguridad del Lua creada en: {backup_lua}")
    
    # Listas para almacenar datos del eje X e Y
    x_experimento = []
    eje_x_labels = []
    
    y_ql, y_qa, y_qs, y_qi = [], [], [], []
    
    # Calcular los valores que tomarán las variables de ruido
    valores_dist = calcular_pasos(RANGO_RUIDO_DIST, NUM_EXPERIMENTOS)
    valores_ang = calcular_pasos(RANGO_RUIDO_ANG, NUM_EXPERIMENTOS)

    plt.ion() 
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.canvas.manager.set_window_title('Evolución de Calidad vs Ruido')
    plt.show(block=False)

    try:
        for i in range(NUM_EXPERIMENTOS):
            dist_actual = valores_dist[i]
            ang_actual = valores_ang[i]
            num_exp = i + 1
            
            print(f"\n[{'='*60}]")
            print(f"--- Experimento {num_exp}/{NUM_EXPERIMENTOS} | Ruido Dist: {dist_actual:.4f} | Ruido Ang: {ang_actual:.4f} ---")
            
            modify_lua_noise(backup_lua, LUA_SCRIPT, dist_actual, ang_actual)
                
            cmd = [sys.executable, RUN_SIM_SCRIPT]
            proceso = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            
            cola_datos = queue.Queue()
            hilo_lector = threading.Thread(target=lector_salida, args=(proceso.stdout, cola_datos), daemon=True)
            hilo_lector.start()
            
            # Variables temporales para el experimento actual
            cur_ql, cur_qa, cur_qs, cur_qi = None, None, None, None
            simulacion_activa = True
            
            while simulacion_activa or not cola_datos.empty():
                try:
                    linea = cola_datos.get(timeout=0.05)
                    sys.stdout.write(linea)
                    sys.stdout.flush()
                    
                    # Extraer métricas
                    ql, qa, qs, qi = parse_metrics_line(linea)
                    if qi is not None:
                        cur_ql, cur_qa, cur_qs, cur_qi = ql, qa, qs, qi
                        
                    # --- ELIMINA O COMENTA ESTAS LÍNEAS ---
                    # if END_PATTERN in linea:
                    #     simulacion_activa = False
                    # --------------------------------------
                        
                except queue.Empty:
                    plt.pause(0.01)
                    # El master solo se detendrá cuando run_sim.py termine completamente
                    if proceso.poll() is not None:
                        simulacion_activa = False
            
            proceso.wait() 
            hilo_lector.join()
            
            # --- ACTUALIZACIÓN GRÁFICA ---
            if cur_qi is not None:
                x_experimento.append(num_exp)
                # Formatear la etiqueta del eje X (Número de experimento + ruidos)
                eje_x_labels.append(f"Exp {num_exp}\n(D:{dist_actual:.3f}, A:{ang_actual:.2f})")
                
                y_ql.append(cur_ql)
                y_qa.append(cur_qa)
                y_qs.append(cur_qs)
                y_qi.append(cur_qi)
                
                ax.clear()
                ax.set_title('Calidad de Imitación vs Incremento de Ruido', fontsize=14)
                ax.set_xlabel('Experimento (Ruido Distancia, Ruido Ángulo)', fontsize=11)
                ax.set_ylabel('Índices de Calidad', fontsize=11)
                ax.set_ylim(-0.05, 1.05) # Los índices van de 0 a 1
                ax.grid(True, linestyle='--', alpha=0.7)
                
                # Configurar estilo de líneas basado en banderas
                alpha_sub = 0.3 if SEMITRANSPARENTES else 1.0
                
                if GRAFICAR_TODOS_LOS_INDICES:
                    ax.plot(x_experimento, y_ql, lw=2, color='orange', marker='^', alpha=alpha_sub, label='Ql (Longitud)')
                    ax.plot(x_experimento, y_qa, lw=2, color='green', marker='v', alpha=alpha_sub, label='Qa (Ángulo)')
                    ax.plot(x_experimento, y_qs, lw=2, color='purple', marker='s', alpha=alpha_sub, label='Qs (Segmentos)')
                
                # El general (Qi) siempre se grafica resaltado
                ax.plot(x_experimento, y_qi, lw=3, color='blue', marker='o', label='Qi (General)')
                ax.scatter([x_experimento[-1]], [y_qi[-1]], s=80, color='red', zorder=5) # Resalta el último punto
                
                # Reemplazar los números del eje X con nuestras etiquetas compuestas
                ax.set_xticks(x_experimento)
                ax.set_xticklabels(eje_x_labels, rotation=0, fontsize=9)
                
                ax.legend(loc='lower left')
                fig.canvas.draw_idle()
                plt.pause(0.1)
            else:
                print(f"[!] Advertencia: No se extrajeron métricas en el Experimento {num_exp}")

    except KeyboardInterrupt:
        print("\n[!] Orquestador interrumpido manualmente.")
        
    finally:
        print(f"\n[{'='*60}]")
        print("Restaurando controlador Lua original...")
        if os.path.exists(backup_lua):
            shutil.move(backup_lua, LUA_SCRIPT)
            print("Controlador restaurado con éxito.")

        if SAVE_PLOTS and x_experimento:
            os.makedirs(PLOTS_DIR, exist_ok=True)
            try:
                ruta_img = os.path.join(PLOTS_DIR, "calidad_vs_ruido.png")
                fig.savefig(ruta_img, dpi=150, bbox_inches='tight')
                print(f"Gráfica final guardada en: '{ruta_img}'.")
            except Exception as e:
                print(f"Error al guardar: {e}")

        if AUTO_CLOSE_WINDOWS:
            plt.close('all')
        else:
            print("Proceso finalizado. Cierra las ventanas de gráficas para terminar.")
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()