#!/usr/bin/env python3
"""
multi_run_sim.py

Orquestador Maestro de Experimentos (Auto-Configurable).
- Genera las combinaciones de variables a evaluar por grupos.
- Ejecuta run_sim.py enviándole rutas originales, variables modificadas y evaluadores a usar.
- run_sim.py se encarga de crear copias temporales y aplicar modificaciones sin tocar los originales.
"""

import os
import sys
import subprocess
import itertools
import time
import json
import threading

# =============================================================================
# ---------- 1. CONFIGURACIÓN DEL EXPERIMENTO (VARIABLES A BARRER) ----------
# =============================================================================

NUM_EXP_M        = 1   # Repeticiones por cada configuración (Simulaciones idénticas)

# Definición de grupos de variables.
# Formato de cada grupo: ( {variables}, NUM_PASOS_N, MODO_EVALUACION )
# MODO: 1 -> Simultáneo (Avanza las variables al mismo tiempo)
#       2 -> Combinatorio (Todas las mezclas posibles internamente en el grupo)
VARIABLES_A_EVALUAR = [
    (
        { 
            "NOISE_BASE_RADIUS"  : (0.0, 0.05), 
        }, 
        5, # NUMERO DE PASOS
        1   # MODO A EVALUAR
    ),

    # (
    #     { # Grupo 1
    #         "NOISE_LEVEL_DIST"  : (0.0, 0.05), 
    #         "NOISE_LEVEL_ANGLE" : (0.0, 0.25),
    #     }, 
    #     5, # NUMERO DE PASOS
    #     1   # MODO A EVALUAR
    # ),
    # (
    #     { # Grupo 2
    #         "speed_dem": (5, 30)        
    #     }, 
    #     3, # NUMERO DE PASOS
    #     1  # MODO A EVALUAR
    # )
]

# =============================================================================
# ---------- 2. CONFIGURACIÓN DE MULTI EVALUATORS ----------
# =============================================================================
MULTI_EVALUATORS = [
    {
        "script": "scripts/evaluator_qi_noise.py",
        "args": []
    }
    # {
    #     "script": "scripts/evaluator_otsu.py",
    #     "args": []
    # }
    # {
    #     "script": "scripts/evaluator_qi_speed_noise.py",
    #     "args": []
    # },
]

# =============================================================================
# ---------- 3. CONFIGURACIÓN DE ARCHIVOS Y RUTAS ----------
# =============================================================================
# ARGOS_SCRIPT     = "experiments/Qupa/Argos/Tr.argos"         
# LUA_SCRIPT       = "experiments/Qupa/Controller/Tr6.lua" 

# ARGOS_SCRIPT     = "experiments/Qupa/Argos/Tr.argos"        
# LUA_SCRIPT       = "experiments/Qupa/Controller/Tr7/Tr7.lua"

# ARGOS_SCRIPT     = "experiments/Qupa/Argos/Ks.argos"         
# LUA_SCRIPT       = "experiments/Qupa/Controller/Ks1/Ks1.lua" 

ARGOS_SCRIPT     = "experiments/Qupa/Argos/Ks.argos"         
LUA_SCRIPT       = "experiments/Qupa/Controller/Pruebas/Ks1.lua" 

RUN_SIM_SCRIPT   = "scripts/run_sim.py"                      
TIMEOUT_SIMULACION = 90                                      

# =============================================================================
# ---------- CONFIGURACIÓN VISUAL Y VERBOSIDAD ----------
# =============================================================================
ENABLE_VISUALIZATION = True # Se enviará a run_sim.py para sobrescribir su valor por defecto

# =============================================================================
# ---------- CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
# 0: Silencioso total (solo info inicial y conteo de exps). Todo pasa oculto. Ideal para pruebas masivas.
# 1: Recomendado (Muestra eventos vitales: inicios, cierres, guardados de evaluadores).
# 2: Detallado (Modo 1 + Prints específicos etiquetados de los Trackers, sin mezclar salidas ruidosas).
# 3: Todo (Modo 2 + Datos puros de ARGoS como variables continuas, timers, alertas internas, etc).
NIVEL_LOG = 2

def print_log(mensaje, nivel_minimo=0, end="\n"):
    if NIVEL_LOG >= nivel_minimo:
        sys.stdout.write(mensaje + end)
        sys.stdout.flush()

# =============================================================================
# ---------- GENERADOR DE COMBINACIONES ----------
# =============================================================================
def generar_configuraciones_grupo(variables, pasos, modo):
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

def generar_configuraciones(grupos):
    listas_configs = []
    for g in grupos:
        listas_configs.append(generar_configuraciones_grupo(g[0], g[1], g[2]))
        
    # Invertimos el orden para que el último grupo en la lista varie más lento (loop externo),
    # y el primer grupo varíe más rápido (loop interno), tal cual lo solicitado.
    listas_configs.reverse()
    
    configs_finales = []
    for combos in itertools.product(*listas_configs):
        merged = {}
        # Revertimos nuevamente para añadir las variables en el orden original
        for c in reversed(combos):
            merged.update(c)
        configs_finales.append(merged)
        
    return configs_finales

# =============================================================================
# ---------- FUNCIONES DE EVALUADOR ----------
# =============================================================================
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

    configuraciones = generar_configuraciones(VARIABLES_A_EVALUAR)
    total_configs = len(configuraciones)
    total_sims = total_configs * NUM_EXP_M

    print_log(f"[INFO] Grupos de evaluación : {len(VARIABLES_A_EVALUAR)}", 0)
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

    # Lista plana de scripts evaluadores para enviar a run_sim
    evaluators_list = [ev["script"] for ev in MULTI_EVALUATORS]

    try:
        for i, config_actual in enumerate(configuraciones):
            str_vars = ", ".join([f"{k}: {v:.4f}" for k, v in config_actual.items()])
            print_log(f"\n[{'='*70}]", 1)
            print_log(f"[CONFIG {i+1}/{total_configs}] Aplicando -> {str_vars}", 1)

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
                    # FORZADO A NIVEL 3. Multi_run capturará todo y filtrará su propia consola.
                    "--nivel_log", "3", 
                    "--vars", json.dumps(config_actual),
                    "--evaluators", json.dumps(evaluators_list),
                    "--visualize", str(ENABLE_VISUALIZATION)
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
                        # Mandamos al evaluador TODO sin importar el NIVEL_LOG del multi_run
                        enviar_a_evaluadores(eval_procesos, linea.strip())

                        # Filtrado interno del terminal para el usuario
                        if NIVEL_LOG == 3:
                            sys.stdout.write(linea) 
                        elif NIVEL_LOG == 2:
                            if "[Gestor]" in linea or "[+]" in linea or "[Análisis Calidad]" in linea or "ARGoS finalizó" in linea:
                                sys.stdout.write(linea)
                        elif NIVEL_LOG == 1:
                            if "[Gestor]" in linea or "[+] Esperando" in linea or "[+] Todos los procesos" in linea or "ARGoS finalizó" in linea:
                                sys.stdout.write(linea)

                proc_sim.wait()
                # Informamos el progreso j / NUM_EXP_M al evaluador para que actualice la gráfica
                enviar_a_evaluadores(eval_procesos, f"[SIM_END] {j+1}/{NUM_EXP_M}")
                print_log(f"  < Fin exp {j+1}/{NUM_EXP_M}", 0)

    except KeyboardInterrupt:
        print_log("\n[!] Ejecución interrumpida por el usuario.", 0)

    finally:
        print_log("\n[+] Enviando señal de finalización a los evaluadores...", 1)
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