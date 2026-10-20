# ============================================================================
# Archivo: scripts/tracker_python_gestor.py
# ============================================================================

import math
import re
import sys
from Observer import math_helper
import Observer.ImitationQuality as ImitationQuality
from Observer.Global_Trajectory_Reconstructor import GlobalTrajectoryReconstructor
from Observer.Parameter_Storage import ParameterStorage
from Observer.Point_Cloud_Segmenter import PointCloudSegmenter
from Observer.Segment_Parameterizer import SegmentParameterizer

# ============================================================================
# PARÁMETROS GLOBALES Y CONFIGURACIÓN (Espejo de Ks5.lua)
# ============================================================================
ticks_per_second = 4
factor_vel = 1

# Métricas del modelo de observación
distance_treshold = 20

# Métricas del modelo de paciencia
MAX_PATIENCE = 160 * ticks_per_second  # Tiempo máximo de espera sin recibir datos útiles en ticks
BASE_RESTORE = 20 * ticks_per_second   # Paciencia base a restaurar con datos refinados/actualizados en ticks

# Métricas del módulo: Point_Cloud_Segmenter y Segment_Parameterizer
seg_cfg = {
    'distance_treshold': 15,
    'limite_maximo_ventana': 30 / factor_vel,
    'tamano_subsampling': 15,
    'use_subsampling': False,
}

# Métricas del módulo de reconstrucción: Global_Trajectory_Reconstructor y Parameter_Storage
CONFIG = {
    'TOL_DIST': 15,
    'TOL_ANG': 15,
}

# Banderas de loggeo (igual que en Ks5.lua)
SRP_LOG_MODE = 0  # 0: Sin logs, 1: Parámetros obtenidos tras finalizar observación, 2: Paso a paso, 3: Todo
UP_LOG_MODE = 2   # 0: Sin logs, 1: Delimitador de ciclos de aprendizaje, 2: Información sobre la paciencia

cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE, 'use_subsampling': False}

# ============================================================================
# [SECCIÓN 2] VARIABLES GLOBALES DE ESTADO Y MEMORIA
# ============================================================================
N_MAX_INTENTOS = 3
intentos_redundantes = N_MAX_INTENTOS
K_UMBRAL_SIMILITUD = 0.85

current_patience = MAX_PATIENCE
current_timer = 0
timer_agotado_python = None
timer_pendiente_decremento = False
en_ciclo_aprendizaje = False
esperando_limpieza_rw = False
AP_1 = 1

master_tracking = {}    # Memoria detallada de cada maestro observado
srt_anterior_data = {}  # Registro temporal para enlazar blobs entre ticks (tracking)
all_params = []         # Almacenará todos los patrones descubiertos
current_winning_ids = {}

param_store = ParameterStorage()

# ============================================================================
# [SECCIÓN 3] GESTIÓN DE PACIENCIA Y RECONSTRUCCIÓN
# ============================================================================
DECAY_RATES = {
    1: 0.15,  # 15% de pérdida por cada vez que ESE fragmento hace Nivel 1
    2: 0.10,  # 10% de pérdida por cada vez que ESE fragmento hace Nivel 2
    3: 0.0,   # 0% de pérdida para Nivel 3 (Totalmente nuevo, restaura full)
}


def log_py(msg):
    print(f"[PYTHON GESTOR] {msg}", flush=True)


def fmt_f(val, decimals=2):
    return f"{val:.{decimals}f}".replace('.', ',')


def soft_reset_observer():
    """Equivalente a soft_reset_observer() en Lua tras finalizar el Random Walk (c4 == 2)."""
    global current_patience, timer_agotado_python, timer_pendiente_decremento
    global master_tracking, srt_anterior_data, AP_1
    current_patience = MAX_PATIENCE
    timer_agotado_python = None
    timer_pendiente_decremento = False
    master_tracking.clear()
    srt_anterior_data.clear()
    AP_1 = 1


def ejecutar_limpieza_ciclo():
    """Ejecuta el bloque de limpieza selectiva del estado c4 == 2 de Ks5.lua."""
    global current_winning_ids, esperando_limpieza_rw
    if not esperando_limpieza_rw:
        return
    current_cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE}
    param_store.clear(current_cfg, current_winning_ids)
    current_winning_ids = {}
    soft_reset_observer()
    esperando_limpieza_rw = False


def decrement_python_patience():
    """Descuenta 1 tick de paciencia en Python sincronizado con Timer=xx."""
    global current_patience, timer_agotado_python
    if param_store.is_done():
        return
    current_patience -= 1
    if current_patience <= 0 and timer_agotado_python is None:
        timer_agotado_python = current_timer


def update_patience(restore_level, data_count, fragment_history_count):
    global current_patience, timer_agotado_python
    if restore_level == 0:
        return  # Nivel 0: Sin restauración

    if restore_level == 3:
        # Nivel 3: Datos totalmente nuevos (Full). Ahora no decae (0%)
        decay_factor = 1.0
        restore_amount = math.floor(MAX_PATIENCE * decay_factor)

        current_patience = min(MAX_PATIENCE, current_patience + restore_amount)
        if current_patience > 0:
            timer_agotado_python = None

        if UP_LOG_MODE == 2 or UP_LOG_MODE == 3:
            log_py(
                f"[PACIENCIA] Restaurada (NIVEL 3). Eficacia 100%. "
                f"+{fmt_f(restore_amount / ticks_per_second, 2)} seg. "
                f"Actual: {fmt_f(current_patience / ticks_per_second, 2)} seg."
            )

    elif restore_level == 1 or restore_level == 2:
        decay_factor = max(0.0, 1.0 - (fragment_history_count * DECAY_RATES[restore_level]))
        base_efectiva = BASE_RESTORE * decay_factor
        restore_amount = math.floor(base_efectiva)

        if restore_level == 2:
            restore_amount = restore_amount * 2

        current_patience = min(MAX_PATIENCE, current_patience + restore_amount)
        if current_patience > 0:
            timer_agotado_python = None

        if UP_LOG_MODE == 2 or UP_LOG_MODE == 3:
            log_py(
                f"[PACIENCIA] Rest. Parcial (NIVEL {restore_level}). "
                f"Eficacia {decay_factor * 100:.0f}%. "
                f"+{fmt_f(restore_amount / ticks_per_second, 2)} seg. "
                f"Actual: {fmt_f(current_patience / ticks_per_second, 2)} seg."
            )


def trigger_reconstruction():
    """Se ejecuta cuando la simulación informa que la paciencia se agotó en Lua."""
    global all_params, intentos_redundantes, current_winning_ids
    global en_ciclo_aprendizaje, esperando_limpieza_rw, timer_pendiente_decremento

    if param_store.is_done():
        return

    # Si el Timer del tick actual aún no se había descontado, lo descontamos antes de comparar
    if en_ciclo_aprendizaje and timer_pendiente_decremento:
        decrement_python_patience()
        timer_pendiente_decremento = False

    # Línea informativa de comparación de paciencia entre Python y Lua
    paciencia_py_seg = current_patience / ticks_per_second
    if current_patience == 0:
        log_py(
            f"[SINCRONIZACIÓN PACIENCIA] (Timer={current_timer}) ¡COINCIDENCIA EXACTA! "
            f"Paciencia en Python: 0,00 seg (0 ticks) vs Lua: 0,00 seg."
        )
    elif current_patience < 0:
        diff_seg = abs(paciencia_py_seg)
        diff_ticks = abs(current_patience)
        t_agotado_str = f"Timer={timer_agotado_python}" if timer_agotado_python is not None else "N/A"
        log_py(
            f"[SINCRONIZACIÓN PACIENCIA] (Timer={current_timer}) DESFASE: "
            f"En Python la paciencia ya se había agotado hace {fmt_f(diff_seg, 2)} seg "
            f"({diff_ticks} ticks, en {t_agotado_str}). "
            f"Paciencia actual Python: {fmt_f(paciencia_py_seg, 2)} seg."
        )
    else:
        log_py(
            f"[SINCRONIZACIÓN PACIENCIA] (Timer={current_timer}) DESFASE: "
            f"En Python aún sobraba paciencia: +{fmt_f(paciencia_py_seg, 2)} seg "
            f"(+{current_patience} ticks restantes) cuando en Lua llegó a 0,00 seg."
        )

    datos_guardados = param_store.get_stored_data()
    current_cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE}

    exito, linea_resumen, nuevos_comportamientos, winner_ids = GlobalTrajectoryReconstructor.run(
        "Qupa4", datos_guardados, current_cfg, len(all_params)
    )

    hay_comportamiento_nuevo = False

    if exito:
        param_store.set_done()
        current_winning_ids = winner_ids if winner_ids else {}

        # LÓGICA DE SIMILITUD Y REDUNDANCIA
        if len(all_params) == 0:
            hay_comportamiento_nuevo = True
            for comportamiento in nuevos_comportamientos:
                all_params.append(comportamiento)
        else:
            for nuevo_comportamiento in nuevos_comportamientos:
                es_conocido = False
                for comportamiento_conocido in all_params:
                    qi = ImitationQuality.evaluate(comportamiento_conocido, nuevo_comportamiento)
                    if qi >= K_UMBRAL_SIMILITUD:
                        es_conocido = True
                        if UP_LOG_MODE == 2:
                            log_py(f" -> Comportamiento redundante detectado. Similitud Qi = {fmt_f(qi * 100, 2)}%")

                        # PROMEDIAR PARÁMETROS DEL COMPORTAMIENTO
                        min_len = min(len(comportamiento_conocido), len(nuevo_comportamiento))
                        for i in range(min_len):
                            comportamiento_conocido[i][0] = (comportamiento_conocido[i][0] + nuevo_comportamiento[i][0]) / 2.0
                            comportamiento_conocido[i][1] = (comportamiento_conocido[i][1] + nuevo_comportamiento[i][1]) / 2.0

                        if len(nuevo_comportamiento) > len(comportamiento_conocido):
                            for i in range(min_len, len(nuevo_comportamiento)):
                                comportamiento_conocido.append([nuevo_comportamiento[i][0], nuevo_comportamiento[i][1]])
                        break

                if not es_conocido:
                    hay_comportamiento_nuevo = True
                    all_params.append(nuevo_comportamiento)
                    if UP_LOG_MODE == 2:
                        log_py(" >>> COMPORTAMIENTO NUEVO AÑADIDO AL DICCIONARIO <<<")
    else:
        current_winning_ids = {}

    # LÓGICA DE REDUCCIÓN DE INTENTOS Y FIN DE CICLO
    if hay_comportamiento_nuevo:
        intentos_redundantes = N_MAX_INTENTOS
        if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
            log_py("\n===========================================================")
            log_py(" [FIN CICLO DE APRENDIZAJE] NUEVO COMPORTAMIENTO GUARDADO")
            log_py(" Iniciando Random Walk para explorar nuevas áreas.")
            log_py("===========================================================\n\n\n")
        esperando_limpieza_rw = True
    else:
        intentos_redundantes = intentos_redundantes - 1
        if intentos_redundantes > 0:
            if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
                log_py("\n===========================================================")
                log_py(" [FIN CICLO DE APRENDIZAJE] REDUNDANCIA O FALLO")
                if exito:
                    log_py(" -> Todo era redundante (Ya conocido). Intentos restantes N = " + str(intentos_redundantes))
                else:
                    log_py(" -> Reconstrucción fallida (Pura basura). Intentos restantes N = " + str(intentos_redundantes))
                log_py(" Iniciando Random Walk para buscar mejor perspectiva.")
                log_py("===========================================================\n\n\n")
            esperando_limpieza_rw = True
        else:
            if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
                log_py("\n\n\n===========================================================")
                log_py(" [FIN FASE DE APRENDIZAJE] INTENTOS AGOTADOS (N=0)")
                log_py("===========================================================")

            log_py("\n===========================================================")
            log_py(" RESUMEN GLOBAL: COMPORTAMIENTOS APRENDIDOS (DICCIONARIO)  ")
            log_py("===========================================================")
            for idx, comportamiento in enumerate(all_params):
                log_py(f"\n--> Comportamiento [{idx + 1}]:")
                if isinstance(comportamiento, list):
                    params_str = [f"{{D:{fmt_f(p[0], 2)}, A:{fmt_f(p[1], 2)}}}" for p in comportamiento]
                    log_py("    Parámetros: " + ", ".join(params_str))
                else:
                    log_py("    Parámetros: [Formato desconocido]")
            log_py("===========================================================\n")

            intentos_redundantes = N_MAX_INTENTOS
            esperando_limpieza_rw = False

    en_ciclo_aprendizaje = False


# ============================================================================
# [SECCIÓN 4] FUNCIONES AUXILIARES DEL OBSERVADOR (TRACKING CRUDO)
# ============================================================================
def initialize_master_tracking(k, angle_deg, distance, ty, obs_ang_offset_global, x_mark, y_mark):
    angle_rad = angle_deg * (math.pi / 180.0)
    ang_total = angle_rad + obs_ang_offset_global
    x_rel, y_rel = math_helper.polarACartesianoPAR(distance, ang_total)

    x = x_rel + x_mark
    y = y_rel + y_mark

    master_tracking[k] = {
        'raw_x': [x],
        'raw_y': [y],
        'raw_type': [ty],
        'Pattern_start_detected': (ty == 1),
        'End_of_pattern_detected': (ty == 3),
        'processed': False,
        'segmentos': {'movement': [], 'type': [], 'valid': []},
        'params': [],
    }


def process_master_segmentation(k, track, x_original, y_original, ty):
    track['raw_x'].append(x_original)
    track['raw_y'].append(y_original)
    track['raw_type'].append(ty)

    if ty == 1:
        track['Pattern_start_detected'] = True
    if ty == 3:
        track['End_of_pattern_detected'] = True


# ============================================================================
# [SECCIÓN 5] PIPELINE DE PROCESAMIENTO (OBSERVACIÓN INDIVIDUAL)
# ============================================================================
def process_single_observation(k, track):
    if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3:
        log_py(" ........................................................ ")
        log_py(f" PROCESANDO OBSERVACIÓN DEL MAESTRO {k} (Datos totales: {len(track['raw_x'])})")
        log_py(" ........................................................ ")

    clean_mov, clean_type, clean_valid, p_start, p_end = PointCloudSegmenter.process(
        k, track['raw_x'], track['raw_y'], track['raw_type'], seg_cfg
    )

    track['segmentos']['movement'] = clean_mov
    track['segmentos']['type'] = clean_type
    track['segmentos']['valid'] = clean_valid

    if p_start:
        track['Pattern_start_detected'] = True
    if p_end:
        track['End_of_pattern_detected'] = True

    if track['segmentos']['movement'] and len(track['segmentos']['movement']) > 1:
        track['params'] = SegmentParameterizer.process_segments(
            track['segmentos']['movement'], track['segmentos']['valid']
        )

        current_cfg = {'CONFIG': CONFIG}

        # Capturamos el tercer valor: history_decay_level
        r_level, r_count, history_decay_level = param_store.package_and_add(
            track['params'], track['segmentos']['valid'], current_cfg
        )

        # Y lo enviamos a update_patience para que lo evalúe de forma aislada
        if r_level and r_level > 0:
            update_patience(r_level, r_count, history_decay_level)
    else:
        if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3:
            log_py("No hay segmentos suficientes para procesar tras la limpieza.")


# ============================================================================
# [SECCIÓN 6] BUCLE PRINCIPAL DEL OBSERVADOR (TICK A TICK)
# ============================================================================
def procesar_datos_logicos_y_reconstruccion(datos_fisicos):
    global AP_1
    if param_store.is_done():
        return

    if AP_1 == 1:
        if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
            log_py("\n\n\n===========================================================")
            log_py(" [INICIO CICLO DE APRENDIZAJE] ")
            log_py("===========================================================")
        AP_1 = 0

    yellow_blobs = datos_fisicos['yellow_blobs']
    green_blobs = datos_fisicos['green_blobs']
    obs_ang_offset_global = datos_fisicos['obs_ang_offset_global']
    x_mark = datos_fisicos['x_mark']
    y_mark = datos_fisicos['y_mark']

    updated_anterior_indices = {}
    matched_new_indices = {}
    fallos_maestros = {}
    count = 0

    # =========================================================================
    # FASE B: ACTUALIZACIÓN DE MAESTROS EXISTENTES
    # =========================================================================
    candidatos = []
    for j, new_data in enumerate(yellow_blobs):
        ang_total_blob = (new_data['angle'] * (math.pi / 180.0)) + obs_ang_offset_global
        bx_rel, by_rel = math_helper.polarACartesianoPAR(new_data['distance'], ang_total_blob)
        bx_global = bx_rel + x_mark
        by_global = by_rel + y_mark

        for k, anterior_data in srt_anterior_data.items():
            if anterior_data.get('angle') is not None and k in master_tracking and len(master_tracking[k]['raw_x']) > 0:
                mx_global = master_tracking[k]['raw_x'][-1]
                my_global = master_tracking[k]['raw_y'][-1]
                error_dist = math.sqrt((bx_global - mx_global) ** 2 + (by_global - my_global) ** 2)
                if error_dist <= distance_treshold:
                    candidatos.append({'blob_idx': j, 'master_idx': k, 'error': error_dist})

    candidatos.sort(key=lambda a: a['error'])

    for cand in candidatos:
        j = cand['blob_idx']
        k = cand['master_idx']
        if j not in matched_new_indices and k not in updated_anterior_indices:
            srt_anterior_data[k]['angle'] = yellow_blobs[j]['angle']
            srt_anterior_data[k]['distance'] = yellow_blobs[j]['distance']
            srt_anterior_data[k]['type'] = yellow_blobs[j]['type']
            fallos_maestros.pop(k, None)
            updated_anterior_indices[k] = True
            matched_new_indices[j] = True
            count = count + 1

    for k, anterior_data in srt_anterior_data.items():
        if anterior_data.get('angle') is not None and k not in updated_anterior_indices and k in master_tracking and len(master_tracking[k]['raw_x']) > 0:
            mx_global = master_tracking[k]['raw_x'][-1]
            my_global = master_tracking[k]['raw_y'][-1]
            distancias_sobrantes = []
            for j, new_data in enumerate(yellow_blobs):
                if j not in matched_new_indices:
                    ang_total_blob = (new_data['angle'] * (math.pi / 180.0)) + obs_ang_offset_global
                    bx_rel, by_rel = math_helper.polarACartesianoPAR(new_data['distance'], ang_total_blob)
                    bx_global = bx_rel + x_mark
                    by_global = by_rel + y_mark
                    error_dist = math.sqrt((bx_global - mx_global) ** 2 + (by_global - my_global) ** 2)
                    distancias_sobrantes.append(f"{fmt_f(error_dist, 2)} cm")

            msg_razon = f"Ningún blob válido a menos de {fmt_f(distance_treshold, 2)} cm."
            if len(distancias_sobrantes) > 0:
                msg_razon = msg_razon + " Blobs sobrantes a: " + " | ".join(distancias_sobrantes)
            else:
                msg_razon = msg_razon + " No se detectaron blobs sobrantes."
            fallos_maestros[k] = msg_razon

    # =========================================================================
    # FASE C: CREAR NUEVOS MAESTROS
    # =========================================================================
    for j, new_data in enumerate(yellow_blobs):
        if j not in matched_new_indices:
            new_k = len(srt_anterior_data) + 1
            srt_anterior_data[new_k] = {
                'angle': new_data['angle'],
                'distance': new_data['distance'],
                'type': new_data.get('type') or 2,
            }
            updated_anterior_indices[new_k] = True
            initialize_master_tracking(
                new_k,
                new_data['angle'],
                new_data['distance'],
                srt_anterior_data[new_k]['type'],
                obs_ang_offset_global,
                x_mark,
                y_mark,
            )

    # =========================================================================
    # FASE D: PROCESAR BLOBS VERDES
    # =========================================================================
    for green_blob in green_blobs:
        ang_total_green = (green_blob['angle'] * (math.pi / 180.0)) + obs_ang_offset_global
        gbx_rel, gby_rel = math_helper.polarACartesianoPAR(green_blob['distance'], ang_total_green)
        gbx = gbx_rel + x_mark
        gby = gby_rel + y_mark

        for k, master_data in srt_anterior_data.items():
            if master_data.get('angle') is not None and k not in updated_anterior_indices and k in master_tracking and len(master_tracking[k]['raw_x']) > 0:
                mx = master_tracking[k]['raw_x'][-1]
                my = master_tracking[k]['raw_y'][-1]
                dist_error = math.sqrt((gbx - mx) ** 2 + (gby - my) ** 2)
                if dist_error <= distance_treshold:
                    srt_anterior_data[k]['type'] = 3
                    updated_anterior_indices[k] = True
                    master_tracking[k]['End_of_pattern_detected'] = True
                    break

    # =========================================================================
    # FASE G: Gestión de Pérdidas y Procesamiento de Rutas
    # =========================================================================
    for k in range(1, len(srt_anterior_data) + 1):
        track = master_tracking.get(k)
        is_lost = (k in srt_anterior_data and srt_anterior_data[k].get('angle') is not None and k not in updated_anterior_indices)
        is_finished = track and track['End_of_pattern_detected']
        if is_lost:
            if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3:
                razon = fallos_maestros.get(k) or "Sin blobs amarillos cerca."
                log_py(f"\n[!] Maestro {k} PERDIDO. Razón: {razon}")
        if (is_lost or is_finished) and track and not track['processed']:
            process_single_observation(k, track)
            track['processed'] = True
            srt_anterior_data[k]['angle'] = None
            srt_anterior_data[k]['distance'] = None
            srt_anterior_data[k]['type'] = None

    # =========================================================================
    # FASE H: Grabación Continua
    # =========================================================================
    for k in updated_anterior_indices:
        master_data = srt_anterior_data.get(k)
        track = master_tracking.get(k)
        if master_data and track and not track['processed']:
            current_distance = master_data['distance']
            current_angle_rad = master_data['angle'] * (math.pi / 180.0)
            ty = master_data['type']

            ang_total = current_angle_rad + obs_ang_offset_global
            x_new, y_new = math_helper.polarACartesianoPAR(current_distance, ang_total)
            x_original = x_new + x_mark
            y_original = y_new + y_mark

            process_master_segmentation(k, track, x_original, y_original, ty)


def parse_observation_line(line):
    # Formato: [python];observation;obs_ang_offset_global;x_mark;y_mark;yb_str;gb_str
    parts = line.strip().split(';')
    idx = parts.index('[python]')
    obs_ang_offset_global = float(parts[idx + 2].replace(',', '.'))
    x_mark = float(parts[idx + 3].replace(',', '.'))
    y_mark = float(parts[idx + 4].replace(',', '.'))

    yellow_blobs = []
    if len(parts) > idx + 5 and parts[idx + 5]:
        for b_str in parts[idx + 5].split('|'):
            if b_str:
                ang_s, dist_s, type_s = b_str.split(':')
                yellow_blobs.append({
                    'angle': float(ang_s.replace(',', '.')),
                    'distance': float(dist_s.replace(',', '.')),
                    'type': int(type_s),
                })

    green_blobs = []
    if len(parts) > idx + 6 and parts[idx + 6]:
        for b_str in parts[idx + 6].split('|'):
            if b_str:
                ang_s, dist_s = b_str.split(':')
                green_blobs.append({
                    'angle': float(ang_s.replace(',', '.')),
                    'distance': float(dist_s.replace(',', '.')),
                })

    return {
        'obs_ang_offset_global': obs_ang_offset_global,
        'x_mark': x_mark,
        'y_mark': y_mark,
        'yellow_blobs': yellow_blobs,
        'green_blobs': green_blobs,
    }


def main():
    global current_timer, timer_pendiente_decremento, en_ciclo_aprendizaje

    log_py("Iniciado. Escuchando datos de ARGoS en tiempo real...")
    pending_observation = None

    def flush_pending_observation():
        nonlocal pending_observation
        if pending_observation is not None:
            obs = pending_observation
            pending_observation = None
            procesar_datos_logicos_y_reconstruccion(obs)

    for line in sys.stdin:
        # 1. Lectura de Timer=xx para sincronizar el reloj y la paciencia
        if "Timer=" in line:
            flush_pending_observation()
            m_timer = re.search(r"Timer=(\d+)", line)
            if m_timer:
                current_timer = int(m_timer.group(1))
                if en_ciclo_aprendizaje and not param_store.is_done():
                    timer_pendiente_decremento = True
            continue

        # 2. Detección de transferencia a memoria global en Lua (estado c4 == 2 al terminar Random Walk)
        if "[DEPURACIÓN] INICIANDO TRANSFERENCIA A MEMORIA GLOBAL" in line or (
            esperando_limpieza_rw and "[trajectory];[offset];" in line
        ):
            pending_observation = None
            ejecutar_limpieza_ciclo()
            continue

        # 3. Detección de fin de paciencia por señal explícita o por línea de [PACIENCIA](0)
        if "[python];patience_timeout" in line:
            pending_observation = None
            trigger_reconstruction()
            continue

        if "[PACIENCIA](" in line and "segundos" in line:
            m_pac = re.search(r"\[PACIENCIA\]\(([-+]?[0-9]+(?:[.,][0-9]+)?)\)\s*segundos", line)
            if m_pac:
                pac_lua_val = float(m_pac.group(1).replace(',', '.'))
                if pac_lua_val <= 0.0:
                    pending_observation = None
                    trigger_reconstruction()
                else:
                    flush_pending_observation()
                continue

        # 4. Recepción de datos de observación del tick actual
        if "[python];observation;" in line:
            flush_pending_observation()
            # Si veníamos de un Random Walk y no se había disparado aún la limpieza c4 == 2, la ejecutamos ahora
            if esperando_limpieza_rw:
                ejecutar_limpieza_ciclo()

            if not en_ciclo_aprendizaje:
                en_ciclo_aprendizaje = True
                timer_pendiente_decremento = True

            if timer_pendiente_decremento:
                decrement_python_patience()
                timer_pendiente_decremento = False

            try:
                pending_observation = parse_observation_line(line)
            except Exception as e:
                log_py(f"[ERROR] Fallo procesando observación: {e}")
            continue

        # 5. Cualquier otra línea de ese tick confirma que no hubo timeout inmediato tras la observación
        if pending_observation is not None:
            flush_pending_observation()

        if "End_simulation" in line:
            break

    flush_pending_observation()


if __name__ == '__main__':
    main()