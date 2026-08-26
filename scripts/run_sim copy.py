"""
run_sim.py (Modular)
- Prepara temp_run.argos (cambia script lua y configura visualización).
- Lanza argos3 y múltiples scripts de seguimiento.
- Clona la salida de ARGoS y la envía a cada script.
- Detiene ARGoS automáticamente al detectar el END_PATTERN (opcional).
- Guarda el historial de la consola en un archivo de texto filtrado.
"""

# ---------------- CONFIGURACIÓN (modifica aquí valores por defecto) ----------------

# ARGOS_FILE          = "experiments/Footbot/Argos/Winfield.argos"   
# LUA_SCRIPT          = "experiments/Footbot/Controller/J_exp1.lua" 

# ARGOS_FILE          = "experiments/Qupa/Argos/St.argos"   
# LUA_SCRIPT          = "experiments/Qupa/Controller/St10.lua" 

# ARGOS_FILE          = "experiments/Qupa/Argos/Tr.argos"   
# LUA_SCRIPT          = "experiments/Qupa/Controller/Tr7/Tr7.lua" 

ARGOS_FILE          = "experiments/Qupa/Argos/Ks.argos"   
LUA_SCRIPT          = "experiments/Qupa/Controller/Ks1/Ks1.lua" 

USE_STDBUF          = True   
ARGOS_CMD           = ["argos3", "-c", "temp_run.argos"]  

USE_VISUALIZATION   = True # Controla si se lanza la interfaz gráfica de ARGoS o no.

# <-- VARIABLES: Control de pantalla virtual de alta resolución
USE_XVFB            = False           
XVFB_RESOLUTION     = "3840x2160x24" 

# <-- VARIABLE: Controla el idioma del sistema operativo para ARGoS
USE_DOT_DECIMALS    = False  

SAVE_OUTPUT_MODE    = "None" 
DATA_DIR            = "Resultados/Historial_run_sim" 
OUTPUT_FILE_NAME    = "j1.txt"

END_PATTERN         = "End_simulation" 
AUTO_STOP_ON_END    = True 

# Agrega aquí todos los scripts que necesiten leer la salida de ARGoS


TRACKERS = [    
    {
        "script": "scripts/tracker_trayectory.py",
        "args": []
    },

    # {
    #     "script": "scripts/tracker_calidad_imitacion.py",
    #     "args": []
    # },

    # {
    #     "script": "scripts/tracker_varianza.py",
    #     "args": []
    # },

    # {
    #     "script": "scripts/tracker_imitation_chain.py",
    #     "args": []
    # },

    # {
    #     "script": "scripts/tracker_imitated_pattern.py",
    #     "args": []
    # },
    
    # {
    #     "script": "scripts/tracker_clusters.py",
    #     "args": []
    # }

    
]

# ----------------------------------------------------------------------------------

import os
import sys
import subprocess
import xml.etree.ElementTree as ET
import threading

def build_argos_command():
    base = ARGOS_CMD.copy()
    if USE_STDBUF:
        return ["stdbuf", "-oL"] + base
    return base

def broadcast_output(argos_stdout, tracker_procesos, argos_proc, out_file, mode, auto_stop):
    argos_tags = ["[INFO]", "[WARNING]", "[ERR]", "[FATAL]"]

    for line in argos_stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        
        if out_file and mode:
            is_log = any(tag in line for tag in argos_tags)
            if mode == "ALL":
                out_file.write(line)
            elif mode == "LOG" and is_log:
                out_file.write(line)
            elif mode == "PRINT" and not is_log:
                out_file.write(line)
            out_file.flush() 
        
        for p in tracker_procesos:
            if p.stdin and p.poll() is None: 
                try:
                    p.stdin.write(line)
                    p.stdin.flush()
                except OSError:
                    pass
        
        if auto_stop and END_PATTERN in line:
            print(f"\n[Gestor] Patrón de fin '{END_PATTERN}' detectado. Cerrando ARGoS...")
            argos_proc.terminate()
            break 

    for p in tracker_procesos:
        if p.stdin:
            try:
                p.stdin.close()
            except Exception:
                pass

def main():
    cwd = os.getcwd()
    print(f"Directorio de trabajo: {cwd}")

    try:
        tree = ET.parse(ARGOS_FILE)
    except FileNotFoundError:
        print(f"ERROR: No se encontró {ARGOS_FILE}")
        return
    root = tree.getroot()
    
    found_lua = False
    for controller in root.findall('controllers/lua_controller'):
        params = controller.find('params')
        if params is not None:
            print(f"Cambiando controlador a: {LUA_SCRIPT}")
            params.set('script', LUA_SCRIPT)
            found_lua = True
            
    if not found_lua:
        print("Error: No se encontró <lua_controller> o <params> en el XML.")
        return

    vis_node = root.find('visualization')
    if not USE_VISUALIZATION:
        if vis_node is not None:
            root.remove(vis_node)
            print("Visualización DESACTIVADA. Ejecutando en modo sin interfaz (headless).")
        else:
            print("Visualización DESACTIVADA. (No se encontró el nodo en el XML base).")
    else:
        if USE_XVFB:
            print(f"Visualización ACTIVADA. Conectando a la pantalla virtual :99.")
        else:
            print("Visualización ACTIVADA. ARGoS abrirá la interfaz gráfica nativa.")

    temp_argos_file = "temp_run.argos"
    tree.write(temp_argos_file, encoding='utf-8', xml_declaration=True)

    output_file_handle = None
    full_output_path = ""
    
    if SAVE_OUTPUT_MODE and SAVE_OUTPUT_MODE != "None":
        os.makedirs(DATA_DIR, exist_ok=True)
        full_output_path = os.path.join(DATA_DIR, OUTPUT_FILE_NAME)
        output_file_handle = open(full_output_path, "w", encoding="utf-8")

    # --- AQUÍ ESTÁ LA MAGIA QUE CONECTA CON TU VNC ---
    run_env = os.environ.copy()
    if USE_DOT_DECIMALS:
        run_env["LC_ALL"] = "C"
    
    if USE_VISUALIZATION and USE_XVFB:
        run_env["DISPLAY"] = ":99"  # Esto envía ARGoS a la pantalla negra que ya tienes abierta
    # -------------------------------------------------

    argos_cmd = build_argos_command()
    try:
        print("Lanzando ARGoS...")
        argos_proc = subprocess.Popen(
            argos_cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.STDOUT, 
            text=True, 
            bufsize=1,
            env=run_env 
        )
    except FileNotFoundError as e:
        print("Error al lanzar ARGoS.", e)
        if os.path.exists(temp_argos_file):
            os.remove(temp_argos_file)
        if output_file_handle:
            output_file_handle.close()
        return

    tracker_procs = []
    for t_conf in TRACKERS:
        cmd = [sys.executable, t_conf["script"]] + t_conf["args"]
        try:
            print(f"Lanzando tracker: {t_conf['script']}...")
            p = subprocess.Popen(
                cmd, 
                stdin=subprocess.PIPE, 
                text=True, 
                bufsize=1,
                env=run_env
            )
            tracker_procs.append(p)
        except FileNotFoundError as e:
            print(f"Error al lanzar {t_conf['script']}:", e)

    broadcaster = threading.Thread(
        target=broadcast_output, 
        args=(argos_proc.stdout, tracker_procs, argos_proc, output_file_handle, SAVE_OUTPUT_MODE, AUTO_STOP_ON_END)
    )
    broadcaster.start()

    try:
        if USE_VISUALIZATION and USE_XVFB:
            print("ARGoS conectado. Revisa tu cliente VNC para ver la simulación.")
            
        ret = argos_proc.wait()
        print(f"\nARGoS finalizó con código {ret}.")
        
        broadcaster.join()
        
        for p in tracker_procs:
            try:
                p.wait(timeout=60) 
            except subprocess.TimeoutExpired:
                p.terminate()
                p.wait(timeout=5)
                
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario (Ctrl+C), terminando procesos...")
        for p in tracker_procs:
            try:
                p.terminate()
            except Exception:
                pass
        try:
            argos_proc.terminate()
        except Exception:
            pass
    finally:
        if output_file_handle:
            output_file_handle.close()
            
        if os.path.exists(temp_argos_file):
            os.remove(temp_argos_file)

if __name__ == '__main__':
    main()