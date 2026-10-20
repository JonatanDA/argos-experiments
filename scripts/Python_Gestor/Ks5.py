# Archivo: Python_Gestor/Ks5.py

# =============================================
#   MEJORAS :  -- Migración de funciones de procesamiento de Lua a python.
# =============================================
# -----------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------
#                                                   VARIABLES GLOBALES                                                             --
# -----------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------

# ================== ACTIVAR PARA QUPA ==================
# -----------------------------
# NOTAS:
#     Este archivo es el espejo en Python del controlador global de los Qupa (Ks5.lua).
#     Contiene la lógica de comportamiento del observador, aprendizaje, paciencia y reconstrucción de patrones.

import math

from Python_Gestor.Functions import math_helper
from Python_Gestor.Functions.Observer.Global_Trajectory_Reconstructor import GlobalTrajectoryReconstructor
import Python_Gestor.Functions.Observer.ImitationQuality as ImitationQuality
from Python_Gestor.Functions.Observer.Parameter_Storage import ParameterStorage
from Python_Gestor.Functions.Observer.Point_Cloud_Segmenter import PointCloudSegmenter
from Python_Gestor.Functions.Observer.Segment_Parameterizer import SegmentParameterizer

# -----------------------------
# Parámetro de simulación externos

ticks_per_second                    = 2                         # Número de ticks que representa un segundo en la simulación (ajusta según la configuración de .argos)
max_time                            = 5000 * ticks_per_second   # Duración máxima de la simulación en ticks
Distribucion_sensores_de_proximidad = [0, -45, 45, -90, 90, 180]
Num_cantidad_leds                   = 2
FOOTBOT_INTERWHEEL_DISTANCE         = 14                        # Distancia entre ruedas del Qupa (en metros)

# ------------------------------
# Parámetro de simulación internos

factor_vel  = 1                   # Factor para ajustar la velocidad del master al ejecutar el patrón
speed_dem   = 10 * factor_vel     #
speed_dem_g = (speed_dem / 2)     #

d_speed     = 30                  # velocidad en linea recta del apprentice
g_speed     = 80                  # Velocidad de giro del apprentice

# ------------------------------
# Modelos

# Métricas del modelo de observación
distance_treshold       = 20         # Sensibilidad del apprentice a cambios en los puntos obtenidos del maestro durante la etapa de observación.

# Métricas del modelo de paciencia
MAX_PATIENCE            = 160 * ticks_per_second      # Tiempo máximo de espera sin recibir datos útiles en segundos (ticks)
BASE_RESTORE            = 20  * ticks_per_second      # Paciencia base a restaurar con datos refinados/actualizados en segundos (ticks)

# Métricas del modelo de ActiveLearning
back_threshold          = 10      # Distancia que retrocede el apprentice para evitar colisión. cm.
front_threshold         = 30      # Limite de distancia minima entre un objeto detectado y el apprentice para entrar a evadirlo
max_tracking_distance   = 10      # Máxima distancia a la que se puede alejar el centroide antes de que el apprentice se acerque para no perder de vista. cm
porcentaje              = 0.75    # 1 < porcentaje < 0.33 , indica que tanto el apprentice busca alejarse del maestro cuando este cruza el max_tracking_distance.

# ------------------------------
# Módulos

# Métricas del módulo de ruido en camara de blobs del aprendiz: noise_manager.lua
noise_cfg = {
    'ENABLE_NOISE'           : True,    # Si es false, el robot ve perfecto (Ground Truth)
    'USE_DISTANCE_DEPENDENT' : True,    # El ruido aumenta si el objeto está lejos
    'USE_OUTLIERS'           : False,   # Activar lecturas "fantasma" o fallos graves
    'USE_BIAS'               : True,    # Error sistemático (ej. sensor mal calibrado +2cm siempre)
    'NOISE_BASE_RADIUS'      : 0.000,   # Ruido constante base (ej. 1 cm de error mínimo)
    'NOISE_SIGMAS'           : 2.0,     # 2 sigmas = 95% de probabilidad de caer dentro del 'Radio Actual'
    'NOISE_FACTOR'           : 0.1,     # Crece un 10% respecto a la distancia
    'OUTLIER_PROB'           : 0.01,    # % de probabilidad de fallo catastrófico
    'BIAS_OFFSET'            : 0.0,     # Cm extra que agrega el sensor por defecto (si USE_BIAS es true)
}

# Métricas del módulo: Point_Cloud_Segmenter y Segment_Parameterizer
seg_cfg = {
    'distance_treshold'     : 15,                 # Menor que la longitud de lado más pequeña del/los comportamiento a aprender (cm)
    'limite_maximo_ventana' : (30) / factor_vel,  #
    'tamano_subsampling'    : 15,                 #
    'use_subsampling'       : False,              #
}

# Métricas del módulo de reconstrucción de comportamientos de movimiento: Global_Trajectory_Reconstructor y Parameter_Storage
# Esta tabla centraliza los hiperparámetros de tolerancia geométrica, pesos de evaluación, filtrado de ruido y protecciones de rendimiento en tiempo real para el sistema de reconstrucción.
CONFIG = {
    # ------------------------------------------------------------------------
    # 1. TOLERANCIAS GEOMÉTRICAS Y SOLAPES

    # Margen de error permitido en distancia (en centímetros).
    # Implicación: Define cuán estrictos somos para considerar que dos distancias detectadas corresponden al mismo segmento.
    # Si es muy bajo, el ruido real romperá los datos verdaderos; si es muy alto, unirá cosas dispares.
    'TOL_DIST': 15,

    # Margen de error permitido en ángulos (en grados).
    # Implicación: Tolerancia angular aceptada para determinar si dos rectas, tramos son colineales.
    'TOL_ANG': 15,
}

# ------------------------------
# Banderas de loggeo
SRP_LOG_MODE        = 0  # 0: Sin logs, 1: Parámetros obtenidos tras finalizar la observación de un maestro. 2: Paso a paso de la actualización de maestros, 3: Imprime todo.
UP_LOG_MODE         = 2  # 0: Sin logs, 1: Delimitador de ciclos de aprendizaje, 2: Información sobre la paciencia (Tiempo que el apprentice actualiza para continuar o detener su aprendizaje)
AL_LOG_MODE         = 0  # 0: Sin logs, 1: Información sobre el Active Learning, 2: Información detallada del Active Learning, 3: Estados del active learning
DEBUG_EVASION       = 0  # 0: Nada, 1: Cambios de estado, 2: Sensores y estados continuos

cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE, 'use_subsampling': False}


# -----------------------------------------------------------------------------------------------------------------------------------
# Funciones auxiliares de impresión y formato (Equivalentes a log() y string.format() de Lua)
# -----------------------------------------------------------------------------------------------------------------------------------

def log_py(msg):
    print(f"[PYTHON GESTOR] {msg}", flush=True)


def fmt_f(val, decimals=2):
    return f"{val:.{decimals}f}".replace('.', ',')


# -----------------------------------------------------------------------------------------------------------------------------------
# Variables de rol y estados del Apprentice (Espejo de Ks5.lua)
# -----------------------------------------------------------------------------------------------------------------------------------

all_params              = []    # Almacenará todos los patrones descubiertos
parametros_elegidos     = []    # El patrón específico seleccionado para ejecutar

c4      = 0
AP_1    = 1


# -----------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------
# MODULO DEL OBSERVADOR
# -----------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------

# ============================================================================
# [SECCIÓN 1] INSTANCIACIÓN DE SUBMÓDULOS DEL OBSERVADOR
# ============================================================================
param_store = ParameterStorage()

# ============================================================================
# [SECCIÓN 2] VARIABLES GLOBALES DE ESTADO Y MEMORIA
# ============================================================================

# Variables para lógica de exploración y redundancia (N)
N_MAX_INTENTOS          = 3
intentos_redundantes    = N_MAX_INTENTOS
TIEMPO_RW_TICKS         = 60 * ticks_per_second  # 60 segundos de Random Walk
rw_timer                = 0
K_UMBRAL_SIMILITUD      = 0.85   # 85% de similitud de Qi (0.85) para considerarlo un patrón "conocido"

current_patience  = MAX_PATIENCE
master_tracking   = {}  # Memoria detallada de cada maestro observado
srt_anterior_data = {}  # Registro temporal para enlazar blobs entre ticks (tracking)

# Variables de sincronización de reloj con la terminal de ARGoS
current_timer              = 0
timer_agotado_python       = None
timer_pendiente_decremento = False
en_ciclo_aprendizaje       = False
esperando_limpieza_rw      = False


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
    """Equivalente al estado elseif c4 == 2 de apprentice() en Ks5.lua."""
    global current_winning_ids, esperando_limpieza_rw, c4
    if not esperando_limpieza_rw:
        return
    # Limpieza selectiva de memoria: Manda los IDs ganadores para que el global buffer descarte los inútiles
    current_cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE}
    param_store.clear(current_cfg, current_winning_ids)
    current_winning_ids = {}  # Vaciamos tras usar

    soft_reset_observer()

    esperando_limpieza_rw = False
    c4 = 0  # Regresa a observar


def registrar_timer(nuevo_timer):
    """Sincroniza el contador de ticks Timer=xx leído desde la terminal."""
    global current_timer, timer_pendiente_decremento
    current_timer = nuevo_timer
    if en_ciclo_aprendizaje and not param_store.is_done():
        timer_pendiente_decremento = True


def preparar_ciclo_observacion():
    """Prepara el estado del ciclo cuando llega una línea [python];observation; desde terminal."""
    global en_ciclo_aprendizaje, timer_pendiente_decremento
    if esperando_limpieza_rw:
        ejecutar_limpieza_ciclo()

    if not en_ciclo_aprendizaje:
        en_ciclo_aprendizaje = True
        timer_pendiente_decremento = True


# ============================================================================
# [SECCIÓN 3] GESTIÓN DE PACIENCIA Y RECONSTRUCCIÓN
# ============================================================================

# Decaimientos diferenciados según la calidad de la información
DECAY_RATES = {
    1: 0.15,  # 15% de pérdida por cada vez que ESE fragmento hace Nivel 1
    2: 0.10,  # 10% de pérdida por cada vez que ESE fragmento hace Nivel 2
    3: 0.0,   # 0% de pérdida para Nivel 3 (Totalmente nuevo, restaura full)
}

# Para almacenar los IDs ganadores de la última reconstrucción exitosa
current_winning_ids = {}


# Fíjate que ahora recibe "fragment_history_count"
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
        # ¡CORRECCIÓN! Eliminamos completamente la división por la cantidad de datos.
        # Ahora la cantidad a restaurar depende única y exclusivamente de la Eficacia del historial.

        decay_factor = max(0.0, 1.0 - (fragment_history_count * DECAY_RATES[restore_level]))

        # Base efectiva = Maximo a restaurar para Nivel 1 (ej: 40seg) * Eficacia
        base_efectiva = BASE_RESTORE * decay_factor

        restore_amount = math.floor(base_efectiva)

        # Si es Nivel 2, el máximo a restaurar es el doble (ej: 80seg), manteniendo la proporción
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


def check_patience_and_reconstruct(timeout_from_lua=False):
    global current_patience, timer_agotado_python, timer_pendiente_decremento
    global all_params, intentos_redundantes, current_winning_ids
    global en_ciclo_aprendizaje, esperando_limpieza_rw, c4

    if param_store.is_done():
        return

    # El contador se reduce constantemente con el paso del tiempo (sincronizado 1 vez por Timer=xx)
    if en_ciclo_aprendizaje and timer_pendiente_decremento:
        current_patience = current_patience - 1
        timer_pendiente_decremento = False
        if current_patience <= 0 and timer_agotado_python is None:
            timer_agotado_python = current_timer

    # La reconstrucción se ejecuta cuando se agota la paciencia en la simulación
    if not timeout_from_lua:
        return

    if not en_ciclo_aprendizaje and current_patience == MAX_PATIENCE:
        return

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
        if winner_ids:
            current_winning_ids = winner_ids
        else:
            current_winning_ids = {}

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
        c4 = 1
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
            c4 = 1
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
            c4 = 3
            esperando_limpieza_rw = False

    en_ciclo_aprendizaje = False


# ============================================================================
# [SECCIÓN 4] FUNCIONES AUXILIARES DEL OBSERVADOR (TRACKING CRUDO)
# ============================================================================

def initialize_master_tracking(k, angle_deg, distance, ty, obs_ang_offset_global, x_mark, y_mark):
    angle_rad = angle_deg * (math.pi / 180.0)  # Blob viene en grados, se pasa a rad

    # COMPENSACIÓN: offset global del observador (Ya están en Radianes)
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


# Función para normalizar ángulos entre -PI y PI y evitar que srt_ang_mark_ant crezca infinitamente tras muchas correcciones
def normalize_angle_rad(ang):
    while ang > math.pi:
        ang = ang - 2 * math.pi
    while ang <= -math.pi:
        ang = ang + 2 * math.pi
    return ang


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
    track['segmentos']['type']     = clean_type
    track['segmentos']['valid']    = clean_valid

    if p_start:
        track['Pattern_start_detected']  = True
    if p_end:
        track['End_of_pattern_detected'] = True

    if track['segmentos']['movement'] and len(track['segmentos']['movement']) > 1:
        track['params'] = SegmentParameterizer.process_segments(
            track['segmentos']['movement'], track['segmentos']['valid']
        )

        current_cfg = {'CONFIG': CONFIG}

        # ¡ATENCIÓN! Capturamos el tercer valor: history_decay_level
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

# PROCESAMIENTO LÓGICO Y DE RECONSTRUCCIÓN
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

    # 1. Extraer los datos de la tabla recibida
    yellow_blobs          = datos_fisicos['yellow_blobs']
    green_blobs           = datos_fisicos['green_blobs']
    obs_ang_offset_global = datos_fisicos['obs_ang_offset_global']
    x_mark                = datos_fisicos['x_mark']
    y_mark                = datos_fisicos['y_mark']

    # 2. Leer la paciencia y actualizar el contador del tick (solicitado)
    check_patience_and_reconstruct(timeout_from_lua=False)

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
                num_pts = len(master_tracking[k]['raw_x'])
                mx_global = master_tracking[k]['raw_x'][num_pts - 1]
                my_global = master_tracking[k]['raw_y'][num_pts - 1]
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
            num_pts = len(master_tracking[k]['raw_x'])
            mx_global = master_tracking[k]['raw_x'][num_pts - 1]
            my_global = master_tracking[k]['raw_y'][num_pts - 1]
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
                num_pts = len(master_tracking[k]['raw_x'])
                mx = master_tracking[k]['raw_x'][num_pts - 1]
                my = master_tracking[k]['raw_y'][num_pts - 1]
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