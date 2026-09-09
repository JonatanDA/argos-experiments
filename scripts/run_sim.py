#!/usr/bin/env python3

"""
run_sim.py (Modular y Auto-Configurable)
- Prepara temp_run_<PID>.argos y temp_run_<PID>.lua dinámicos (evita colisiones).
- Auto-descomenta funciones y variables Lua basándose en los evaluadores pasados y trackers activos.
- Inyecta el valor de las variables solicitadas por multi_run directamente en la copia de lua.
- Lanza argos3 y múltiples scripts de seguimiento (trackers).
- Clona la salida de ARGoS y la envía a cada script evitando colisiones en terminal.
- Detiene ARGoS automáticamente al detectar el END_SIMULATION_PATTERN.
- Guarda el historial de la consola en un archivo de texto filtrado.
"""

import os
import sys
import subprocess
import xml.etree.ElementTree as ET
import threading
import time
import argparse
import re
import json

# =============================================================================
# ---------- 1. DICCIONARIO DE EVALUADORES Y DEPENDENCIAS ----------
# =============================================================================
EVALUATOR_DEPENDENCIES = {
    "scripts/evaluator_qi_noise.py": {
        "lua_functions": [
            "evaluator_ruido",
            "Print_tracker_calidad_imitacion_data"
        ],
        "lua_variables": [],
        "run_sim_trackers": [
            "scripts/tracker_calidad_imitacion.py"
        ]
    },
    "scripts/evaluator_otsu.py": {
        "lua_functions": [
            "evaluator_otsu"
        ],
        "lua_variables": [],
        "run_sim_trackers": []
    },
    "scripts/evaluator_qi_speed_noise.py": {
        "lua_functions": [],
        "lua_variables": [],
        "run_sim_trackers": [
            "scripts/tracker_calidad_imitacion.py"
        ]
    }
}

# =============================================================================
# ---------- 2. DICCIONARIO DE TRACKERS Y DEPENDENCIAS LUA ----------
# =============================================================================
TRACKER_DEPENDENCIES = {
    "tracker_trayectory.py": {
        "descripcion": "Rastrea y grafica la trayectoria 2D (x,y) global del master y/o aprendiz.",
        "funciones_lua": ["Print_tracker_trayectory_data"],
        "variables_lua": ["displayed_data", "PTTD"]
    },
    "tracker_calidad_imitacion.py": {
        "descripcion": "Evalúa y compara la calidad geométrica del patrón dibujado frente al original (Heal Pattern, EMP, Qi).",
        "funciones_lua": ["Print_tracker_calidad_imitacion_data"],
        "variables_lua": []
    },
    "tracker_varianza.py": {
        "descripcion": "Analiza la varianza y convergencia en los datos obtenidos por el observador.",
        "funciones_lua": ["Print_tracker_varianza"],
        "variables_lua": []
    },
    "evaluator_otsu.py": {
        "descripcion": "Evalúa el método de Otsu a distintos valores de velocidad.",
        "funciones_lua": ["evaluator_otsu"],
        "variables_lua": []
    },
    "evaluator_qi_noise.py": {
        "descripcion": "Evalúa el comportamiento al inyectar ruido estadístico (ángulo y distancia).",
        "funciones_lua": ["evaluator_ruido"],
        "variables_lua": []
    },
    "evaluator_qi_speed_noise.py": {
        "descripcion": "Evalúa el comportamiento al inyectar ruido estadístico (ángulo y distancia).",
        "funciones_lua": [],
        "variables_lua": []
    },
}


# =============================================================================
# ---------- CONFIGURACIÓN DE RUTAS Y ARCHIVOS DE SIMULACIÓN ----------
# =============================================================================
# Preconfiguraciones guardadas (descomentar según el experimento):
# DEFAULT_ARGOS_FILE              = "experiments/Footbot/Argos/Winfield.argos"   
# DEFAULT_LUA_SCRIPT              = "experiments/Footbot/Controller/J_exp1.lua" 

# DEFAULT_ARGOS_FILE              = "experiments/Qupa/Argos/St.argos"   
# DEFAULT_LUA_SCRIPT              = "experiments/Qupa/Controller/St10.lua" 

# DEFAULT_ARGOS_FILE              = "experiments/Qupa/Argos/Tr.argos"   
# DEFAULT_LUA_SCRIPT              = "experiments/Qupa/Controller/Tr6.lua" 

# DEFAULT_ARGOS_FILE              = "experiments/Qupa/Argos/Tr.argos"   
# DEFAULT_LUA_SCRIPT              = "experiments/Qupa/Controller/Tr7/Tr7.lua" 

DEFAULT_ARGOS_FILE              = "experiments/Qupa/Argos/Ks.argos"   
DEFAULT_LUA_SCRIPT              = "experiments/Qupa/Controller/Ks1/Ks1.lua" 

# DEFAULT_ARGOS_FILE              = "experiments/Qupa/Argos/Prueba1.argos"   
# DEFAULT_LUA_SCRIPT              = "experiments/Qupa/Controller/Pruebas/Move.lua" 

# =============================================================================
# ---------- CONFIGURACIÓN DE TRACKERS ACTIVOS ----------
# =============================================================================
TRACKERS = [    
    {
        "script": "scripts/tracker_trayectory.py",
        "args": []
    },
    {
        "script": "scripts/tracker_paciencia.py",
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

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
ENABLE_VISUALIZATION            = True              # True: Muestra la ventana de ARGoS, False: Headless (sin GUI)             

# --- Control de Pantalla Virtual (Xvfb) ---
USE_XVFB_VIRTUAL_DISPLAY        = False             
XVFB_DISPLAY_RESOLUTION         = "3840x2160x24" 

# =============================================================================
# ---------- CONFIGURACIÓN DE EJECUCIÓN Y MOTOR DE ARGOS ----------
# =============================================================================
USE_STDBUF_BUFFERING            = True                                
ARGOS_BASE_COMMAND              = ["argos3", "-c"] # Se añadirá el archivo dinámico luego

# =============================================================================
# ---------- CONFIGURACIÓN DE HISTORIAL Y ARCHIVOS DE SALIDA ----------
# =============================================================================
SAVE_OUTPUT_MODE                = "None"            
OUTPUT_DATA_DIR                 = "Resultados/Historial_run_sim" 
OUTPUT_LOG_FILENAME             = "j1.txt"

# =============================================================================
# ---------- CONTROL DE PROCESO Y FINALIZACIÓN ----------
# =============================================================================
END_SIMULATION_PATTERN          = "End_simulation"  
AUTO_STOP_ON_END_PATTERN        = True              
MAX_IDLE_TIMEOUT_SEC            = 120               

# =============================================================================
# ---------- CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
# 0: Silencioso total (solo info inicial y conteo de exps). Todo pasa oculto. Ideal para pruebas masivas.
# 1: Recomendado (Muestra eventos vitales: inicios, cierres, guardados de evaluadores).
# 2: Detallado (Modo 1 + Prints específicos etiquetados de los Trackers, sin mezclar salidas ruidosas).
# 3: Todo (Modo 2 + Datos puros de ARGoS como variables continuas, timers, alertas internas, etc).
NIVEL_LOG = 3

tracker_activity = {}
print_lock = threading.Lock()

def print_log(mensaje, nivel_minimo=1, actual_nivel_log=1, end="\n"):
    if actual_nivel_log >= nivel_minimo:
        with print_lock:
            sys.stdout.write(mensaje + end)
            sys.stdout.flush()

def auto_uncomment_lua_and_inject_vars(filepath, dest_path, func_names, var_names, vars_dict):
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    out_lines = []
    in_target_func = False
    depth = 0

    for line in lines:
        is_var_line = False
        if not in_target_func:
            for vname in var_names:
                if re.match(rf"^[ \t]*--\s*{vname}\s*=", line):
                    uncommented = re.sub(r"^([ \t]*)--\s?", r"\1", line)
                    out_lines.append(uncommented)
                    is_var_line = True
                    break
        
        if is_var_line:
            continue

        if not in_target_func:
            for fname in func_names:
                if re.match(rf"^[ \t]*--\s*function\s+{fname}\b", line):
                    in_target_func = True
                    depth = 0
                    break

        if in_target_func:
            uncommented = re.sub(r"^([ \t]*)--\s?", r"\1", line)
            out_lines.append(uncommented)
            clean_line = re.sub(r"--.*", "", uncommented)
            clean_line = re.sub(r'".*?"', '""', clean_line)
            words = re.findall(r"\b\w+\b", clean_line)
            for w in words:
                if w in ['function', 'if', 'for', 'while', 'repeat']:
                    depth += 1
                elif w in ['end', 'until']:
                    depth -= 1
            if depth == 0:
                in_target_func = False
        else:
            out_lines.append(line)

    # Inyección de variables sobre el contenido final
    content = "".join(out_lines)
    for var_name, var_val in vars_dict.items():
        patron = r'(?P<antes>\b' + re.escape(var_name) + r'\s*=\s*)[-+]?[0-9]*\.?[0-9]+'
        content = re.sub(patron, rf'\g<antes>{var_val:.4f}', content)

    with open(dest_path, 'w', encoding='utf-8') as f:
        f.write(content)

def build_argos_command(temp_argos_file):
    base = ARGOS_BASE_COMMAND.copy() + [temp_argos_file]
    if USE_STDBUF_BUFFERING:
        return ["stdbuf", "-oL"] + base
    return base

def monitor_tracker_stdout(p, p_pid, actual_nivel_log):
    buffer_line = ""
    while True:
        char = p.stdout.read(1)
        if not char:
            if buffer_line and actual_nivel_log >= 2:
                with print_lock:
                    sys.stdout.write(buffer_line)
                    sys.stdout.flush()
            break
            
        buffer_line += char
        if char == '\n' or char == '\r':
            if actual_nivel_log >= 2:
                with print_lock:
                    sys.stdout.write(buffer_line)
                    sys.stdout.flush()
            buffer_line = ""
            
        tracker_activity[p_pid] = time.time() 

def broadcast_output(argos_stdout, tracker_procesos, argos_proc, out_file, mode, auto_stop, actual_nivel_log):
    for line in argos_stdout:
        if actual_nivel_log >= 3:
            with print_lock:
                sys.stdout.write(line)
                sys.stdout.flush()
        
        if out_file and mode:
            is_log = "[INFO]" in line or "[WARNING]" in line or "[ERR]" in line or "[FATAL]" in line
            if mode == "ALL" or (mode == "LOG" and is_log) or (mode == "PRINT" and not is_log):
                out_file.write(line)
            out_file.flush() 
        
        for p in tracker_procesos:
            if p.poll() is None: 
                try:
                    p.stdin.write(line)
                    p.stdin.flush()
                except OSError:
                    pass
        
        if auto_stop and END_SIMULATION_PATTERN in line:
            print_log(f"\n[Gestor] Patrón de fin '{END_SIMULATION_PATTERN}' detectado. Cerrando ARGoS...", 1, actual_nivel_log)
            argos_proc.terminate()
            break 

    for p in tracker_procesos:
        if p.stdin:
            try:
                p.stdin.close()
            except Exception:
                pass

def translate_return_code(code):
    if code == 0: return "Éxito/Normal"
    if code == -15 or code == 143: return "SIGTERM (Forzado a cerrarse - Esperado por Gestor)"
    if code == -9 or code == 137: return "SIGKILL (Asesinado abruptamente)"
    return f"Desconocido"

def main():
    global ENABLE_VISUALIZATION
    
    parser = argparse.ArgumentParser(description="Orquestador de ARGoS (run_sim.py)")
    parser.add_argument("--argos", type=str, help="Ruta al archivo .argos", default=None)
    parser.add_argument("--lua", type=str, help="Ruta al script .lua", default=None)
    parser.add_argument("--nivel_log", type=int, help="Sobrescribe NIVEL_LOG internamente", default=None)
    parser.add_argument("--vars", type=str, help="JSON con variables a inyectar", default="{}")
    parser.add_argument("--evaluators", type=str, help="Lista JSON de evaluadores a usar", default="[]")
    parser.add_argument("--visualize", type=str, help="True o False, de multi_run", default=None)
    args, _ = parser.parse_known_args()

    actual_argos_file = args.argos if args.argos else DEFAULT_ARGOS_FILE
    actual_lua_script = args.lua if args.lua else DEFAULT_LUA_SCRIPT
    actual_nivel_log = args.nivel_log if args.nivel_log is not None else NIVEL_LOG

    try: evaluators_list = json.loads(args.evaluators)
    except: evaluators_list = []
    
    try: vars_dict = json.loads(args.vars)
    except: vars_dict = {}

    if args.visualize is not None:
        ENABLE_VISUALIZATION = (args.visualize.lower() == "true")

    cwd = os.getcwd()
    print_log(f"[Gestor] Directorio de trabajo: {cwd}", 1, actual_nivel_log)

    pid = os.getpid()
    temp_argos_file = f"temp_run_{pid}.argos"
    
    dir_lua_original = os.path.dirname(actual_lua_script)
    if not dir_lua_original: dir_lua_original = "."
    nombre_base_lua = os.path.basename(actual_lua_script)
    temp_lua_file = os.path.join(dir_lua_original, f"temp_{pid}_{nombre_base_lua}")

    # --- AUTO-DESCOMENTADO DE DEPENDENCIAS LUA ---
    req_lua_funcs = []
    req_lua_vars = []
    
    # Evaluar dependencias de Evaluadores inyectados
    for ev_script in evaluators_list:
        if ev_script in EVALUATOR_DEPENDENCIES:
            req_lua_funcs.extend(EVALUATOR_DEPENDENCIES[ev_script].get("lua_functions", []))
            req_lua_vars.extend(EVALUATOR_DEPENDENCIES[ev_script].get("lua_variables", []))
            
            # Inyectar trackers dinámicos en la lista de TRACKERS
            req_trackers = EVALUATOR_DEPENDENCIES[ev_script].get("run_sim_trackers", [])
            for rt in req_trackers:
                if not any(t["script"] == rt for t in TRACKERS):
                    TRACKERS.append({"script": rt, "args": []})

    # Evaluar dependencias de todos los Trackers (manuales + inyectados)
    for t_conf in TRACKERS:
        script_key = os.path.basename(t_conf["script"])
        if script_key in TRACKER_DEPENDENCIES:
            req_lua_funcs.extend(TRACKER_DEPENDENCIES[script_key].get("funciones_lua", []))
            req_lua_vars.extend(TRACKER_DEPENDENCIES[script_key].get("variables_lua", []))

    req_lua_funcs = list(set(req_lua_funcs))
    req_lua_vars = list(set(req_lua_vars))

    if req_lua_funcs or req_lua_vars or vars_dict:
        print_log(f"[Gestor] Auto-configurando e inyectando dependencias Lua en copia temporal...", 1, actual_nivel_log)
    
    try:
        auto_uncomment_lua_and_inject_vars(actual_lua_script, temp_lua_file, req_lua_funcs, req_lua_vars, vars_dict)
    except FileNotFoundError:
        print_log(f"[Gestor] ERROR: No se encontró {actual_lua_script}", 0, actual_nivel_log)
        return

    # --- MODIFICACIÓN DEL ARBOL XML ---
    try:
        tree = ET.parse(actual_argos_file)
    except FileNotFoundError:
        print_log(f"[Gestor] ERROR: No se encontró {actual_argos_file}", 0, actual_nivel_log)
        if os.path.exists(temp_lua_file): os.remove(temp_lua_file)
        return
        
    root = tree.getroot()
    
    found_lua = False
    for controller in root.findall('controllers/lua_controller'):
        params = controller.find('params')
        if params is not None:
            print_log(f"[Gestor] Cambiando controlador XML a copia temporal: {temp_lua_file}", 1, actual_nivel_log)
            params.set('script', temp_lua_file)
            found_lua = True
            
    if not found_lua:
        print_log("[Gestor] Error: No se encontró <lua_controller> o <params> en el XML.", 0, actual_nivel_log)
        if os.path.exists(temp_lua_file): os.remove(temp_lua_file)
        return

    vis_node = root.find('visualization')
    if not ENABLE_VISUALIZATION:
        if vis_node is not None:
            root.remove(vis_node)
            print_log("[Gestor] Visualización DESACTIVADA. Ejecutando en headless.", 1, actual_nivel_log)
    else:
        if USE_XVFB_VIRTUAL_DISPLAY:
            print_log(f"[Gestor] Visualización ACTIVADA. (Pantalla virtual :99).", 1, actual_nivel_log)

    tree.write(temp_argos_file, encoding='utf-8', xml_declaration=True)

    output_file_handle = None
    if SAVE_OUTPUT_MODE and SAVE_OUTPUT_MODE != "None":
        os.makedirs(OUTPUT_DATA_DIR, exist_ok=True)
        full_output_path = os.path.join(OUTPUT_DATA_DIR, OUTPUT_LOG_FILENAME)
        output_file_handle = open(full_output_path, "w", encoding="utf-8")

    run_env = os.environ.copy()
    if ENABLE_VISUALIZATION and USE_XVFB_VIRTUAL_DISPLAY: run_env["DISPLAY"] = ":99" 

    argos_cmd = build_argos_command(temp_argos_file)
    try:
        print_log("[Gestor] Lanzando ARGoS...", 1, actual_nivel_log)
        argos_proc = subprocess.Popen(
            argos_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
            text=True, bufsize=1, env=run_env 
        )
    except FileNotFoundError as e:
        print_log(f"[Gestor] Error al lanzar ARGoS: {e}", 0, actual_nivel_log)
        if os.path.exists(temp_argos_file): os.remove(temp_argos_file)
        if os.path.exists(temp_lua_file): os.remove(temp_lua_file)
        if output_file_handle: output_file_handle.close()
        return

    tracker_procs = []
    for t_conf in TRACKERS:
        cmd = [sys.executable, t_conf["script"]] + t_conf["args"]
        try:
            print_log(f"[Gestor] Lanzando tracker: {t_conf['script']}...", 1, actual_nivel_log)
            p = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT, text=True, bufsize=1, env=run_env
            )
            tracker_procs.append(p)
            tracker_activity[p.pid] = time.time()
            
            t = threading.Thread(target=monitor_tracker_stdout, args=(p, p.pid, actual_nivel_log), daemon=True)
            t.start()
        except FileNotFoundError as e:
            print_log(f"[Gestor] Error al lanzar {t_conf['script']}: {e}", 0, actual_nivel_log)

    broadcaster = threading.Thread(
        target=broadcast_output, 
        args=(argos_proc.stdout, tracker_procs, argos_proc, output_file_handle, SAVE_OUTPUT_MODE, AUTO_STOP_ON_END_PATTERN, actual_nivel_log)
    )
    broadcaster.start()

    try:
        ret = argos_proc.wait()
        desc_codigo = translate_return_code(ret)
        print_log(f"\n[Gestor] ARGoS finalizó con código {ret} -> {desc_codigo}.", 1, actual_nivel_log)
        
        broadcaster.join()
        
        print_log(f"\n[+] Esperando a que los trackers finalicen sus tareas (Timeout: {MAX_IDLE_TIMEOUT_SEC}s)...", 1, actual_nivel_log)
        
        while tracker_procs:
            for p in list(tracker_procs):
                if p.poll() is not None:
                    tracker_procs.remove(p)
                    continue
                
                last_active = tracker_activity.get(p.pid, time.time())
                if AUTO_STOP_ON_END_PATTERN and (time.time() - last_active) > MAX_IDLE_TIMEOUT_SEC:
                    print_log(f"\n[!] ERROR: Tracker PID:{p.pid} congelado. Matando...", 0, actual_nivel_log)
                    p.terminate()
                    try: p.wait(timeout=5)
                    except: p.kill()
                    tracker_procs.remove(p)
                    
            time.sleep(1) 
            
        print_log("\n[+] Todos los procesos y trackers han finalizado.", 1, actual_nivel_log)
                
    except KeyboardInterrupt:
        print_log("\n[Gestor] Interrumpido por usuario (Ctrl+C), terminando procesos...", 0, actual_nivel_log)
        for p in tracker_procs:
            try: p.terminate()
            except: pass
        try: argos_proc.terminate()
        except: pass
    finally:
        if output_file_handle:
            output_file_handle.close()
        if os.path.exists(temp_argos_file):
            os.remove(temp_argos_file)
        if os.path.exists(temp_lua_file):
            os.remove(temp_lua_file)

if __name__ == '__main__':
    main()