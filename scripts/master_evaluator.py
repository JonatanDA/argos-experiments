# !/usr/bin/env python3

"""
master_evaluator.py
- Ejecuta múltiples simulaciones del gestor principal (run_sim.py).
- Crea una copia temporal del script Lua.
- Utiliza un Hilo (Thread) y un Buffer (Queue) para leer la terminal sin perder datos.
- Grafica los resultados dinámicamente.
- Permite auto-cierre y guardado automático de gráficas al finalizar.
"""

import os
import sys
import subprocess
import re
import shutil
import threading
import queue
import matplotlib.pyplot as plt

# ================= CONFIGURACIÓN DE VARIABLES =================
MIN_SPEED = 5
MAX_SPEED = 30
STEP_SPEED = 1

# Cadenas de detección
END_PATTERN = "End_simulation"
OTSU_PATTERN = "Otsu &"

# Rutas a tus archivos
LUA_SCRIPT = "experiments/Qupa/Controller/Tr6.lua"
RUN_SIM_SCRIPT = "scripts/run_sim.py"

# Opciones de finalización
AUTO_CLOSE_WINDOWS = False  
SAVE_PLOTS = True          
PLOTS_DIR = "Resultados"   
# ==============================================================

ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
NUMBER_RE = re.compile(r'[-]?\d+[.,]?\d*(?:[eE][-+]?\d+)?')

def create_temp_lua(backup_path, target_path, speed_val):
    with open(backup_path, 'r', encoding='utf-8') as f:
        content = f.read()
    new_content = re.sub(r'(speed_dem\s*=\s*)\d+(\.?\d*)', f'\\g<1>{speed_val}', content)
    with open(target_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

def parse_otsu_line(line):
    clean_line = ANSI_ESCAPE.sub('', line)
    if OTSU_PATTERN in clean_line:
        parts = clean_line.split('&', 1)
        if len(parts) > 1:
            nums = NUMBER_RE.findall(parts[1])
            if len(nums) >= 2:
                try:
                    otsu_val = float(nums[0].replace(',', '.'))
                    avance_val = float(nums[1].replace(',', '.'))
                    return otsu_val, avance_val
                except ValueError:
                    pass
    return None, None

def lector_salida(pipe, cola_buffer):
    """
    Se ejecuta en un hilo secundario. Lee la terminal lo más rápido posible 
    y guarda las líneas en la cola para no perder datos.
    """
    try:
        for linea in iter(pipe.readline, ''):
            if linea:
                cola_buffer.put(linea)
    except Exception:
        pass
    finally:
        pipe.close()

def main():
    print("Iniciando orquestador de simulaciones ARGoS...")
    
    if not os.path.exists(LUA_SCRIPT):
        print(f"ERROR: No se encontró el archivo Lua: {LUA_SCRIPT}")
        return

    backup_lua = LUA_SCRIPT + ".bak"
    print(f"Creando copia de seguridad del controlador Lua en: {backup_lua}")
    shutil.copy(LUA_SCRIPT, backup_lua)
    
    x_velocidades = []
    y_otsu = []
    y_avance = []
    
    plt.ion() 
    fig_otsu, ax_otsu = plt.subplots(figsize=(6, 4))
    fig_otsu.canvas.manager.set_window_title('Métrica - Separación de Otsu')
    
    fig_avance, ax_avance = plt.subplots(figsize=(6, 4))
    fig_avance.canvas.manager.set_window_title('Métrica - Mejor Avance')
    
    plt.show(block=False)

    try:
        rango_velocidades = range(MIN_SPEED, MAX_SPEED + 1, STEP_SPEED)
        
        for velocidad in rango_velocidades:
            print(f"\n[{'='*50}]")
            print(f"--- Ejecutando simulación con speed_dem = {velocidad} ---")
            
            create_temp_lua(backup_lua, LUA_SCRIPT, velocidad)
                
            cmd = [sys.executable, RUN_SIM_SCRIPT]
            proceso = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            
            # --- INICIO DEL BUFFER (HILO LECTOR) ---
            cola_datos = queue.Queue()
            hilo_lector = threading.Thread(target=lector_salida, args=(proceso.stdout, cola_datos), daemon=True)
            hilo_lector.start()
            
            otsu_iter = None
            avance_iter = None
            simulacion_activa = True
            
            # --- PROCESAMIENTO DEL BUFFER ---
            while simulacion_activa or not cola_datos.empty():
                try:
                    # Sacamos la línea del buffer (liberando datos sin bloquear)
                    linea = cola_datos.get(timeout=0.05)
                    
                    sys.stdout.write(linea)
                    sys.stdout.flush()
                    
                    o_val, a_val = parse_otsu_line(linea)
                    if o_val is not None and a_val is not None:
                        otsu_iter = o_val
                        avance_iter = a_val
                        print(f"\n[DATO EXTRAÍDO LÍMPIO] Velocidad: {velocidad} | Otsu: {otsu_iter} | Avance: {avance_iter}\n")
                            
                    if END_PATTERN in linea:
                        simulacion_activa = False
                        
                except queue.Empty:
                    # Si el buffer está temporalmente vacío, mantenemos viva la interfaz gráfica
                    plt.pause(0.01)
                    # Revisamos si el proceso de ARGoS se cerró por error
                    if proceso.poll() is not None:
                        simulacion_activa = False
            
            proceso.wait() 
            hilo_lector.join()
            
            # --- ACTUALIZACIÓN GRÁFICA AL FINALIZAR LA VELOCIDAD ---
            if otsu_iter is not None and avance_iter is not None:
                x_velocidades.append(velocidad)
                y_otsu.append(otsu_iter)
                y_avance.append(avance_iter)
                
                ax_otsu.clear()
                ax_otsu.set_title('Separación de Otsu vs Velocidad')
                ax_otsu.set_xlabel('Velocidad de Demostrador')
                ax_otsu.set_ylabel('Valor Otsu')
                ax_otsu.grid(True)
                if x_velocidades:
                    ax_otsu.plot(x_velocidades, y_otsu, lw=2, color='blue', marker='o')
                    ax_otsu.scatter([x_velocidades[-1]], [y_otsu[-1]], s=50, color='red')
                fig_otsu.canvas.draw_idle()
                
                ax_avance.clear()
                ax_avance.set_title('Mejor Avance vs Velocidad')
                ax_avance.set_xlabel('Velocidad de Demostrador')
                ax_avance.set_ylabel('Mejor Avance')
                ax_avance.grid(True)
                if x_velocidades:
                    ax_avance.plot(x_velocidades, y_avance, lw=2, color='green', marker='s')
                    ax_avance.scatter([x_velocidades[-1]], [y_avance[-1]], s=50, color='red')
                fig_avance.canvas.draw_idle()
                
                # Se actualiza la pantalla de golpe
                plt.pause(0.1)
            else:
                print(f"Advertencia: No se detectaron métricas válidas en la iteración {velocidad}")

    except KeyboardInterrupt:
        print("\n[!] Automatización interrumpida por el usuario.")
        
    finally:
        print(f"\n[{'='*50}]")
        print("Restaurando controlador Lua original...")
        if os.path.exists(backup_lua):
            shutil.move(backup_lua, LUA_SCRIPT)
            print("Limpieza completada.")

        if SAVE_PLOTS and x_velocidades:
            print("Guardando gráficas finales...")
            os.makedirs(PLOTS_DIR, exist_ok=True)
            try:
                fig_otsu.savefig(os.path.join(PLOTS_DIR, "otsu_vs_speed.png"), dpi=150)
                fig_avance.savefig(os.path.join(PLOTS_DIR, "avance_vs_speed.png"), dpi=150)
                print(f"Gráficas guardadas en la carpeta '{PLOTS_DIR}'.")
            except Exception as e:
                print(f"Error al guardar las gráficas: {e}")

        if AUTO_CLOSE_WINDOWS:
            print("Cerrando ventanas automáticamente...")
            plt.close('all')
        else:
            print("Proceso finalizado. Cierra las ventanas manualmente para salir.")
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()