#!/usr/bin/env python3
"""
multi_run_sim.py

Orquestador Maestro de Experimentos (Auto-Configurable).
- Descomenta automáticamente funciones de LUA y Trackers en run_sim.py basándose en los evaluadores activos.
- Genera las combinaciones de variables a evaluar.
- Modifica el script Lua para cada configuración.
- Ejecuta run_sim.py repetidas veces y le canaliza los resultados a los evaluadores.

Dependencias necesarias (Automáticas):
- Requiere que run_sim.py y los evaluadores (ej. evaluator_ruido&Qi.py) estén en sus rutas correctas.
"""

import os
import sys
import subprocess
import shutil
import itertools
import time
import json
import re
import threading

# =============================================================================
# ---------- 1. DICCIONARIO DE DEPENDENCIAS (AUTO-DESCOMENTADO) ----------
# =============================================================================
# Define qué evaluador activa qué funciones LUA y qué trackers en run_sim.py
EVALUATOR_DEPENDENCIES = {
    "scripts/evaluator_ruido&Qi.py": {
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
    }
}

# =============================================================================
# ---------- 2. CONFIGURACIÓN DEL EXPERIMENTO (VARIABLES A BARRER) ----------
# =============================================================================
NUM_PASOS_N      = 5   
NUM_EXP_M        = 3   

# 1: Simultáneo (Avanza las variables al mismo tiempo). 2: Combinatorio (Todas las mezclas posibles)
MODO_EVALUACION  = 1

# Diccionario de variables a inyectar en el código Lua con sus rangos (Mínimo, Máximo)
VARIABLES_A_EVALUAR = {
    "NOISE_LEVEL_DIST"  : (0.0, 0.05), # -> evaluator_ruido&Qi -> controllers compatibles: Qupa -> Tr6, Tr7
    "NOISE_LEVEL_ANGLE" : (0.0, 0.25),

    # "NOISE_BASE_RADIUS": (0.00, 0.5),  # -> evaluator_ruido&Qi -> controllers compatibles: Qupa ->  Ks1

    # "speed_dem": (5, 30)               # -> evaluator_otsu     -> controllers compatibles: Qupa -> Tr6, Tr7, Ks1
}

# =============================================================================
# ---------- 3. CONFIGURACIÓN DE MULTI EVALUATORS ----------
# =============================================================================
MULTI_EVALUATORS = [
    {
        "script": "scripts/evaluator_ruido&Qi.py",
        "args": []
    },
    # {
    #     "script": "scripts/evaluator_otsu.py",
    #     "args": []
    # }
]
# =============================================================================
# ---------- 4. CONFIGURACIÓN DE ARCHIVOS Y RUTAS ----------
# # =============================================================================
# ARGOS_SCRIPT     = "experiments/Qupa/Argos/Tr.argos"         
# LUA_SCRIPT       = "experiments/Qupa/Controller/Tr6.lua" 

ARGOS_SCRIPT     = "experiments/Qupa/Argos/Tr.argos"         
LUA_SCRIPT       = "experiments/Qupa/Controller/Tr7/Tr7.lua" 

# ARGOS_SCRIPT     = "experiments/Qupa/Argos/Ks.argos"         
# LUA_SCRIPT       = "experiments/Qupa/Controller/Ks1/Ks1.lua" 

RUN_SIM_SCRIPT   = "scripts/run_sim.py"                      
TIMEOUT_SIMULACION = 90                                      

# =============================================================================
# ---------- CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
# 0: Silencioso total (solo info inicial y conteo de exps). Todo pasa oculto. Ideal para pruebas masivas.
# 1: Recomendado (Muestra eventos vitales: inicios, cierres, guardados de evaluadores).
# 2: Detallado (Modo 1 + Prints específicos etiquetados de los Trackers, sin mezclar salidas ruidosas).
# 3: Todo (Modo 2 + Datos puros de ARGoS como variables continuas, timers, alertas internas, etc).
NIVEL_LOG = 3

def print_log(mensaje, nivel_minimo=0, end="\n"):
    if NIVEL_LOG >= nivel_minimo:
        sys.stdout.write(mensaje + end)
        sys.stdout.flush()

# =============================================================================
# ---------- MOTOR SINTÁCTICO: LUA Y PYTHON UNCOMMENTERS ----------
# =============================================================================
def get_uncommented_lua(content, func_names, var_names):
    if not func_names and not var_names: return content
    lines = content.split('\n')
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
                    
        if is_var_line: continue

        if not in_target_func:
            for fname in func_names:
                if re.match(rf"^[ \t]*--\s*function\s+{fname}\b", line):
                    in_target_func = True
                    depth = 0
                    break
                    
        if in_target_func:
            uncommented_line = re.sub(r"^([ \t]*)--\s?", r"\1", line)
            out_lines.append(uncommented_line)
            clean_line = re.sub(r"--.*", "", uncommented_line) 
            clean_line = re.sub(r'".*?"', '""', clean_line)    
            words = re.findall(r"\b\w+\b", clean_line)
            for w in words:
                if w in ['function', 'if', 'for', 'while', 'repeat']: depth += 1
                elif w in ['end', 'until']: depth -= 1
            if depth == 0: in_target_func = False 
        else:
            out_lines.append(line)
    return '\n'.join(out_lines)

def uncomment_python_trackers(file_path, tracker_scripts):
    if not tracker_scripts: return
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    out_lines = []
    in_target = False
    brace_depth = 0
    buffer_lines = []
    for line in lines:
        if not in_target:
            if re.match(r"^[ \t]*#\s*\{", line):
                in_target = True
                brace_depth = 1
                buffer_lines = [line]
            else:
                out_lines.append(line)
        else:
            buffer_lines.append(line)
            uncommented = re.sub(r"^([ \t]*)#\s?", r"\1", line)
            clean_line = re.sub(r'".*?"', '""', uncommented)
            brace_depth += clean_line.count("{")
            brace_depth -= clean_line.count("}")
            if brace_depth == 0: 
                block_str = "".join(buffer_lines)
                if any(ts in block_str for ts in tracker_scripts):
                    for bl in buffer_lines:
                        out_lines.append(re.sub(r"^([ \t]*)#\s?", r"\1", bl))
                else:
                    out_lines.extend(buffer_lines)
                in_target = False
                buffer_lines = []
    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(out_lines)

def generar_configuraciones(variables, pasos, modo):
    valores_por_var = {}
    for var, rango in variables.items():
        if pasos <= 1:
            valores_por_var[var] = [rango[0]]
        else:
            paso_val = (rango[1] - rango[0]) / (pasos - 1)
            valores_por_var[var] = [rango[0] + (i * paso_val) for i in range(pasos)]
    configuraciones = []
    keys = list(valores_por_var.keys())
    if modo == 1:
        for i in range(pasos):
            configuraciones.append({k: valores_por_var[k][i] for k in keys})
    elif modo == 2:
        for combo in itertools.product(*[valores_por_var[k] for k in keys]):
            configuraciones.append({keys[i]: combo[i] for i in range(len(keys))})
    return configuraciones

def modificar_lua(base_content, target_path, config_dict):
    content = base_content
    for var_name, var_val in config_dict.items():
        patron = r'(?P<antes>' + re.escape(var_name) + r'\s*=\s*)[-+]?[0-9]*\.?[0-9]+'
        content = re.sub(patron, rf'\g<antes>{var_val:.4f}', content)
    with open(target_path, 'w', encoding='utf-8') as f:
        f.write(content)

def enviar_a_evaluadores(procesos, mensaje):
    for p in procesos:
        if p.poll() is None:
            p.stdin.write(mensaje + "\n")
            p.stdin.flush()

def monitor_evaluator_stdout(proc, script_name):
    for linea in iter(proc.stdout.readline, ''):
        if not linea: break
        print_log(f"[EVALUADOR: {script_name}] {linea}", 1, end="")

# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    print_log("="*75, 0)
    print_log(" INICIANDO ORQUESTADOR DE EXPERIMENTOS (AUTO-CONFIGURABLE)", 0)
    print_log("="*75, 0)

    configuraciones = generar_configuraciones(VARIABLES_A_EVALUAR, NUM_PASOS_N, MODO_EVALUACION)
    total_configs = len(configuraciones)
    total_sims = total_configs * NUM_EXP_M

    print_log(f"[INFO] Modo de Evaluación   : {'Simultáneo' if MODO_EVALUACION == 1 else 'Combinatorio'}", 0)
    print_log(f"[INFO] Variables detectadas : {list(VARIABLES_A_EVALUAR.keys())}", 0)

    req_lua_funcs = set()
    req_lua_vars = set()
    req_trackers = set()
    for ev in MULTI_EVALUATORS:
        script_name = ev["script"]
        if script_name in EVALUATOR_DEPENDENCIES:
            req_lua_funcs.update(EVALUATOR_DEPENDENCIES[script_name].get("lua_functions", []))
            req_lua_vars.update(EVALUATOR_DEPENDENCIES[script_name].get("lua_variables", []))
            req_trackers.update(EVALUATOR_DEPENDENCIES[script_name].get("run_sim_trackers", []))

    print_log(f"[INFO] Func. Lua Eval Req.  : {list(req_lua_funcs) if req_lua_funcs else 'Ninguna'}", 0)
    print_log(f"[INFO] Trackers Requeridos  : {[os.path.basename(t) for t in req_trackers] if req_trackers else 'Ninguno'}", 0)
    print_log("-" * 75, 0)
    print_log(f"[>] Configuraciones totales : {total_configs}", 0)
    print_log(f"[>] Repeticiones por config : {NUM_EXP_M}", 0)
    print_log(f"[!] TOTAL DE SIMULACIONES   : {total_sims}", 0)
    print_log("-" * 75, 0)

    while True:
        resp = input(f"¿Desea iniciar la ejecución de {total_sims} simulaciones? (Enter para continuar / 'n' para cancelar): ").strip().lower()
        if resp == 'n':
            print("Ejecución cancelada por el usuario.")
            return
        elif resp == '':
            break

    eval_procesos = []
    eval_threads = []
    print_log("\n[+] Levantando Evaluadores...", 1)

    for ev in MULTI_EVALUATORS:
        cmd = [sys.executable, ev["script"]] + ev["args"]
        nombre_script = os.path.basename(ev["script"])
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        eval_procesos.append(p)
        print_log(f"    - Iniciado: {ev['script']} (PID: {p.pid})", 1)

        t = threading.Thread(target=monitor_evaluator_stdout, args=(p, nombre_script), daemon=True)
        t.start()
        eval_threads.append(t)

    backup_lua = LUA_SCRIPT + ".bak"
    backup_run_sim = RUN_SIM_SCRIPT + ".bak"

    shutil.copy(LUA_SCRIPT, backup_lua)
    shutil.copy(RUN_SIM_SCRIPT, backup_run_sim)

    print_log("\n[+] Modificando dependencias automáticamente...", 1)
    uncomment_python_trackers(RUN_SIM_SCRIPT, list(req_trackers))

    with open(backup_lua, 'r', encoding='utf-8') as f:
        base_lua_raw = f.read()
    base_lua_content = get_uncommented_lua(base_lua_raw, list(req_lua_funcs), list(req_lua_vars))

    try:
        for i, config_actual in enumerate(configuraciones):
            str_vars = ", ".join([f"{k}: {v:.4f}" for k, v in config_actual.items()])
            print_log(f"\n[{'='*70}]", 1)
            print_log(f"[CONFIG {i+1}/{total_configs}] Aplicando -> {str_vars}", 1)

            modificar_lua(base_lua_content, LUA_SCRIPT, config_actual)

            cfg_data = {"id": i+1, "params": config_actual}
            enviar_a_evaluadores(eval_procesos, f"[MULTI_CONFIG] {json.dumps(cfg_data)}")

            for j in range(NUM_EXP_M):
                num_sim = (i * NUM_EXP_M) + (j + 1)
                print_log(f"  > Ejecutando exp {j+1}/{NUM_EXP_M} (Simulación Global {num_sim}/{total_sims})", 0)

                enviar_a_evaluadores(eval_procesos, f"[SIM_START] {j+1}")

                cmd = [
                    sys.executable, RUN_SIM_SCRIPT,
                    "--argos", ARGOS_SCRIPT,
                    "--lua", LUA_SCRIPT,
                    "--nivel_log", "3"
                ]
                proc_sim = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

                tiempo_inicio = time.time()
                while True:
                    if time.time() - tiempo_inicio > TIMEOUT_SIMULACION:
                        print_log("    [!] TIMEOUT de simulación. Forzando cierre...", 1)
                        proc_sim.terminate()
                        break

                    linea = proc_sim.stdout.readline()
                    if not linea and proc_sim.poll() is not None:
                        break

                    if linea:
                        enviar_a_evaluadores(eval_procesos, linea.strip())

                        if NIVEL_LOG == 3:
                            sys.stdout.write(linea) 
                        elif NIVEL_LOG == 2:
                            # Optimización: Direct string matching instead of comprehesion
                            if "[Gestor]" in linea or "[+]" in linea or "[Análisis Calidad]" in linea or "ARGoS finalizó" in linea:
                                sys.stdout.write(linea)
                        elif NIVEL_LOG == 1:
                            if "[Gestor]" in linea or "[+] Esperando" in linea or "[+] Todos los procesos" in linea or "ARGoS finalizó" in linea:
                                sys.stdout.write(linea)

                proc_sim.wait()
                enviar_a_evaluadores(eval_procesos, f"[SIM_END] {j+1}")
                print_log(f"  < Fin exp {j+1}/{NUM_EXP_M}", 0)

    except KeyboardInterrupt:
        print_log("\n[!] Ejecución interrumpida por el usuario.", 0)

    finally:
        print_log("\n[+] Restaurando scripts originales desde las copias de seguridad...", 1)
        if os.path.exists(backup_lua):
            shutil.move(backup_lua, LUA_SCRIPT)
        if os.path.exists(backup_run_sim):
            shutil.move(backup_run_sim, RUN_SIM_SCRIPT)

        print_log("[+] Enviando señal de finalización a los evaluadores...", 1)
        enviar_a_evaluadores(eval_procesos, "[MULTI_END_ALL]")

        for p in eval_procesos:
            if p.stdin:
                p.stdin.close()
            p.wait()

        for t in eval_threads:
            t.join()

        print_log("\n" + "="*75, 0)
        print_log(" EXPERIMENTACIÓN FINALIZADA ", 0)
        print_log("="*75, 0)

if __name__ == '__main__':
    main()