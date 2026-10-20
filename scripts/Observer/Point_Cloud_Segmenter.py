# =============================================================================
# MÓDULO: Point_Cloud_Segmenter.py
# =============================================================================
# DESCRIPCIÓN:
# Este módulo procesa las coordenadas Pi = (xi, yi) que un robot "aprendiz" registra
# mientras observa a un "maestro". Su objetivo es segmentar esa nube de puntos
# continua en trazos rectos (lados) y esquinas (vértices).
#
# EJEMPLO DE FUNCIONAMIENTO (VENTANAS DESLIZANTES):
# Supongamos que el maestro traza una trayectoria en forma de "L" y el aprendiz registra una
# Secuencia de 15 puntos: [P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15]
# Estado Físico real:     |----- MOVIMIENTO -----| |--- ESQUINA ---| |--- MOVIMIENTO ---|
#
# 1. Exploración y Ventanas: Si el algoritmo elige un tamaño de ventana de 3,
#    calcula la varianza espacial agrupando de a 3 puntos de forma deslizante:
#    - V1 = [P1, P2, P3] -> Varianza Alta (Movimiento)
#    - V2 = [P2, P3, P4] -> Varianza Alta (Movimiento)
#    ...
#    - V8 = [P8, P9, P10] -> Varianza Baja (Entrando a esquina)
#
# 2. Otsu: Calcula matemáticamente el umbral exacto que separa las Varianzas Altas
#    de las Bajas a lo largo de todas las ventanas de la ruta.
#
# 3. Histéresis (Look-Ahead): Si en V4 = [P4, P5, P6] el maestro tropieza y la varianza
#    cae por debajo del umbral de Otsu, el algoritmo NO corta el segmento de inmediato.
#    Revisa las siguientes 'paquetes_lookahead_ventana' ventanas (V5, V6, V7, V8, V9).
#    Como el maestro recuperó la velocidad, V5, V6 y V7 tendrán varianza alta de nuevo.
#    Al no cumplirse el porcentaje mínimo de confirmación, ignora el tropiezo y
#    mantiene el segmento unido.
# =============================================================================

import math
from Observer import math_helper


class PointCloudSegmenter:
    # =============================================================================
    # CONFIGURACIÓN LOCAL DEL MÓDULO (Parámetros de ajuste general)
    # =============================================================================

    # --- FASE 1: Búsqueda del tamaño de ventana óptimo ---
    paso_exploracion_ventana          = 1
    # limite_maximo_ventana             = 60

    # --- FASE 2 y 3: Clasificación Otsu y Filtro Look-Ahead ---
    factor_suficiencia_otsu           = 1.0
    paquetes_lookahead_ventana        = 5
    porcentaje_confirmacion_lookahead = 0.8

    # Nivel estricto para validación de 2 segmentos (0, 1 o 2)
    nivel_estricto_dos_segmentos      = 0
    log_imprimir_sin_segmentos        = False

    # --- CONFIGURACIÓN DE LOGS (Depuración en terminal) ---

    # Maestros cuyos logs se desean mostrar.
    # [] = muestra los logs de TODOS los maestros.
    # ["Master1"] = muestra únicamente los logs de Master1.
    # ["Master1", "Master3"] = muestra únicamente los logs de Master1 y Master3.
    maestros_a_loggear                = []

    # Control del log específico de Otsu.
    # 0 = No muestra información de Otsu.
    # 1 = Muestra información de Otsu.
    log_otsu                          = 0

    # Nivel de detalle de los logs de detección/segmentación.
    # 0 = No muestra logs de detección.
    # 1 = Muestra todos los logs: exploración, configuración ganadora,
    #     paquetes/varianzas y resumen de segmentos.
    # 2 = Muestra los resúmenes de segmentos y la configuración general,
    #     pero no muestra el detalle de cada paquete/ventana.
    # 3 = Muestra el detalle de los paquetes/ventanas y los resúmenes,
    #     pero no la exploración completa de Otsu.
    log_mode_detection                = 0

    # Controla si se imprimen todos los puntos de cada segmento.
    # False = Solo muestra inicio, fin y cantidad de puntos.
    # True  = Además muestra toda la trayectoria de puntos de cada segmento.
    mostrar_todos_los_puntos          = True

    # Nivel de detalle de los logs del filtrado y fusión de segmentos.
    # 0 = No muestra logs del filtrado.
    # 1 = Muestra información del proceso de filtrado y el resumen final.
    # 2 = Muestra el resumen final del filtrado.
    # 3 = Muestra el resumen final del filtrado.
    log_mode_filtering                = 0

    # =============================================================================
    # FUNCIONES AUXILIARES (Locales y de Buffering)
    # =============================================================================

    log_buffer = []

    @classmethod
    def log(cls, msg):
        print(f"[PYTHON GESTOR] {msg}", flush=True)

    @classmethod
    def fmt_f(cls, val, decimals=2):
        return f"{val:.{decimals}f}".replace('.', ',')

    @classmethod
    def custom_log(cls, msg):
        cls.log_buffer.append(msg)

    @classmethod
    def flush_logs(cls, should_print):
        if should_print:
            for msg in cls.log_buffer:
                cls.log(msg)
        cls.log_buffer = []  # Limpiar buffer

    @classmethod
    def should_log(cls, master_id):
        if len(cls.maestros_a_loggear) == 0:
            return True
        for m_id in cls.maestros_a_loggear:
            if m_id == master_id or str(m_id) == str(master_id):
                return True
        return False

    @classmethod
    def imprimir_resumen_segmentos(cls, titulo, lista_segmentos, lista_tipos, lista_validos, cfg):
        cls.custom_log("\n\n= ====================================================== ")
        cls.custom_log(" " + str(titulo))
        cls.custom_log(" ======================================================= ")

        if len(lista_segmentos) == 0:
            cls.custom_log(" >> No hay segmentos para mostrar.")

        for idx, points in enumerate(lista_segmentos):
            num_puntos = len(points)
            types_list = lista_tipos[idx] if (lista_tipos and idx < len(lista_tipos)) else None

            etiqueta_validez = ""
            if lista_validos is not None and idx < len(lista_validos) and lista_validos[idx] is not None:
                if lista_validos[idx]:
                    etiqueta_validez = " [VALIDO]"
                else:
                    etiqueta_validez = " [INVALIDO]"

            if num_puntos > 0:
                start_pt = points[0]
                end_pt = points[num_puntos - 1]
                start_type = types_list[0] if (types_list and len(types_list) > 0) else -1
                end_type = types_list[num_puntos - 1] if (types_list and len(types_list) >= num_puntos) else -1

                cls.custom_log(f"SEGMENTO #{idx + 1}{etiqueta_validez}")
                cls.custom_log(f" > Total Puntos: {num_puntos}")
                cls.custom_log(f" > Inicio: ({cls.fmt_f(start_pt[0], 3)}, {cls.fmt_f(start_pt[1], 3)}) [Type: {start_type}]")
                cls.custom_log(f" > Fin:    ({cls.fmt_f(end_pt[0], 3)}, {cls.fmt_f(end_pt[1], 3)}) [Type: {end_type}]")

                if cls.mostrar_todos_los_puntos:
                    buffer_linea = " > TRAYECTORIA: "
                    for p_idx, p in enumerate(points):
                        p_idx_1 = p_idx + 1
                        buffer_linea = buffer_linea + f"({cls.fmt_f(p[0], 2)}, {cls.fmt_f(p[1], 2)}) "
                        if p_idx_1 % 5 == 0 and p_idx_1 < num_puntos:
                            cls.custom_log(buffer_linea)
                            buffer_linea = "                "
                    cls.custom_log(buffer_linea)
                cls.custom_log("--------------------------------------------------------")
        cls.custom_log(" ======================================================= \n")

    @classmethod
    def get_segment_dist(cls, seg):
        if not seg or len(seg) < 1:
            return 0
        p1 = seg[0]
        p2 = seg[-1]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    # =============================================================================
    # FASE 1: DETECCIÓN DE SEGMENTOS RAW
    # =============================================================================
    @classmethod
    def detect_seg(cls, master_id, xx, yy, ttypes, cfg):
        total_puntos = len(xx)
        if total_puntos < 3:
            return [], [], None

        if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_otsu == 1):
            cls.custom_log("\n>>> INICIANDO FASE 1: EXPLORACIÓN AUTOMÁTICA DE PARÁMETROS - MAESTRO " + str(master_id))

        max_avance_permitido = math.floor(total_puntos / 3)
        limite_max_ventana = cfg.get('limite_maximo_ventana', 60)
        if max_avance_permitido > limite_max_ventana:
            max_avance_permitido = limite_max_ventana
        if max_avance_permitido < cls.paso_exploracion_ventana:
            max_avance_permitido = cls.paso_exploracion_ventana

        resultados_exploracion = []
        max_separacion_global = -1

        # Búsqueda de la ventana óptima
        for av in range(cls.paso_exploracion_ventana, int(max_avance_permitido) + 1, cls.paso_exploracion_ventana):
            temp_vars, temp_cx, temp_cy = [], [], []

            for i in range(total_puntos - av):
                std_dev, cen_x, cen_y = math_helper.get_window_stats(i, av, xx, yy)
                temp_vars.append(std_dev)
                temp_cx.append(cen_x)
                temp_cy.append(cen_y)

            umbral, separacion = math_helper.otsu_1d(temp_vars)

            if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_otsu == 1):
                cls.custom_log(f" Probando Avance={av} | Umbral={cls.fmt_f(umbral, 3)} | Separación Otsu={cls.fmt_f(separacion, 3)}")

            resultados_exploracion.append({
                'av': av, 'umbral': umbral, 'separacion': separacion,
                'vars': temp_vars, 'cx': temp_cx, 'cy': temp_cy
            })

            if separacion > max_separacion_global:
                max_separacion_global = separacion

        # DETECCIÓN DE COMPORTAMIENTO EN "U" (BAJA Y LUEGO SUBE)
        is_u_shape = False
        first_valley_idx = 0
        decreased = False

        for i in range(1, len(resultados_exploracion)):
            if resultados_exploracion[i]['separacion'] < resultados_exploracion[i - 1]['separacion']:
                decreased = True
            elif decreased and resultados_exploracion[i]['separacion'] > resultados_exploracion[i - 1]['separacion']:
                is_u_shape = True
                first_valley_idx = i - 1
                break

        umbral_aceptacion = 0
        start_search_idx = 0

        if is_u_shape:
            start_search_idx = first_valley_idx
            max_after_valley = -1
            # Solo consideramos los picos generados DESPUÉS del valle de ruido
            for i in range(start_search_idx, len(resultados_exploracion)):
                if resultados_exploracion[i]['separacion'] > max_after_valley:
                    max_after_valley = resultados_exploracion[i]['separacion']
            umbral_aceptacion = max_after_valley * cls.factor_suficiencia_otsu

            if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_otsu == 1):
                cls.custom_log(f" > [!] Comportamiento en 'U' detectado. Ignorando ruido inicial (valle detectado en Avance={resultados_exploracion[first_valley_idx]['av']}).")
        else:
            umbral_aceptacion = max_separacion_global * cls.factor_suficiencia_otsu

        # Selección del avance que cumple el criterio de suficiencia
        mejor_avance = resultados_exploracion[start_search_idx]['av']
        mejor_umbral = resultados_exploracion[start_search_idx]['umbral']
        mejor_separacion_seleccionada = resultados_exploracion[start_search_idx]['separacion']
        mejores_varianzas = resultados_exploracion[start_search_idx]['vars']
        mejores_centroides_x = resultados_exploracion[start_search_idx]['cx']
        mejores_centroides_y = resultados_exploracion[start_search_idx]['cy']

        for i in range(start_search_idx, len(resultados_exploracion)):
            res = resultados_exploracion[i]
            if res['separacion'] >= (umbral_aceptacion - 0.0001):
                mejor_avance = res['av']
                mejor_umbral = res['umbral']
                mejor_separacion_seleccionada = res['separacion']
                mejores_varianzas = res['vars']
                mejores_centroides_x = res['cx']
                mejores_centroides_y = res['cy']

                if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_otsu == 1):
                    cls.custom_log(
                        f" -> SELECCIONADO: Avance={res['av']} ({cls.fmt_f(res['separacion'], 3)} supera el umbral del {int(cls.factor_suficiencia_otsu * 100)}%: {cls.fmt_f(umbral_aceptacion, 3)})"
                    )
                break

        if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_otsu == 1):
            cls.custom_log("\n>>> FASE 2: CONFIGURACIÓN GANADORA")
            cls.custom_log(f" GANADOR -> Avance: {mejor_avance} | Umbral Dinámico: {cls.fmt_f(mejor_umbral, 3)}")
            cls.custom_log("\n>>> FASE 3: SEGMENTACIÓN CON HISTÉRESIS (LOOK-AHEAD)")

        extra_data_callbacks = {
            'mejor_separacion': mejor_separacion_seleccionada,
            'mejor_avance': mejor_avance,
            'varianzas': mejores_varianzas
        }

        segmentos, segmentos_type = [[]], [[]]
        k = 0

        ESTADO_MOVIMIENTO, ESTADO_BUSCANDO_VALLE = 1, 2
        estado_actual = ESTADO_MOVIMIENTO

        valle_min_var = 999999
        valle_best_idx = -1
        valle_cen_x, valle_cen_y = 0, 0
        packet_counter = 1

        for i in range(len(mejores_varianzas)):
            std_dev = mejores_varianzas[i]
            x, y = xx[i], yy[i]
            ty = ttypes[i]
            cx, cy = mejores_centroides_x[i], mejores_centroides_y[i]

            if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_mode_detection == 3):
                end_idx = i + mejor_avance - 1
                end_x, end_y = xx[end_idx], yy[end_idx]
                cls.custom_log(
                    f"Pkt {packet_counter} | Idx[{i + 1}-{end_idx + 1}] Pos({cls.fmt_f(x, 2)}, {cls.fmt_f(y, 2)})->({cls.fmt_f(end_x, 2)}, {cls.fmt_f(end_y, 2)}) | Var={cls.fmt_f(std_dev, 4)}"
                )

            packet_counter = packet_counter + 1

            if estado_actual == ESTADO_MOVIMIENTO:
                if std_dev > mejor_umbral:
                    segmentos[k].append([x, y])
                    segmentos_type[k].append(ty)
                else:
                    count_bajo_umbral = 0
                    limite_busqueda = min(i + cls.paquetes_lookahead_ventana, len(mejores_varianzas))
                    puntos_a_revisar = limite_busqueda - i

                    for j in range(i, limite_busqueda):
                        if mejores_varianzas[j] <= mejor_umbral:
                            count_bajo_umbral = count_bajo_umbral + 1

                    porcentaje_real = count_bajo_umbral / puntos_a_revisar

                    if porcentaje_real >= cls.porcentaje_confirmacion_lookahead:
                        estado_actual = ESTADO_BUSCANDO_VALLE
                        valle_min_var = std_dev
                        valle_best_idx = i
                        valle_cen_x, valle_cen_y = cx, cy
                    else:
                        segmentos[k].append([x, y])
                        segmentos_type[k].append(ty)

            elif estado_actual == ESTADO_BUSCANDO_VALLE:
                if std_dev <= mejor_umbral:
                    if std_dev < valle_min_var:
                        valle_min_var = std_dev
                        valle_best_idx = i
                        valle_cen_x, valle_cen_y = cx, cy
                else:
                    count_sobre_umbral = 0
                    limite_busqueda = min(i + cls.paquetes_lookahead_ventana, len(mejores_varianzas))
                    puntos_a_revisar = limite_busqueda - i

                    for j in range(i, limite_busqueda):
                        if mejores_varianzas[j] > mejor_umbral:
                            count_sobre_umbral = count_sobre_umbral + 1

                    porcentaje_real = count_sobre_umbral / puntos_a_revisar

                    if porcentaje_real >= cls.porcentaje_confirmacion_lookahead:
                        pivot_point = [valle_cen_x, valle_cen_y]
                        pivot_type = ttypes[valle_best_idx]

                        segmentos[k].append(pivot_point)
                        segmentos_type[k].append(pivot_type)

                        k = k + 1
                        segmentos.append([])
                        segmentos_type.append([])
                        segmentos[k].append(list(pivot_point))
                        segmentos_type[k].append(pivot_type)
                        segmentos[k].append([x, y])
                        segmentos_type[k].append(ty)

                        estado_actual = ESTADO_MOVIMIENTO

        if estado_actual == ESTADO_BUSCANDO_VALLE:
            pivot_point = [valle_cen_x, valle_cen_y]
            pivot_type = ttypes[valle_best_idx]
            segmentos[k].append(pivot_point)
            segmentos_type[k].append(pivot_type)

            k = k + 1
            segmentos.append([])
            segmentos_type.append([])
            segmentos[k].append(list(pivot_point))
            segmentos_type[k].append(pivot_type)

        for i in range(len(mejores_varianzas), total_puntos):
            segmentos[k].append([xx[i], yy[i]])
            segmentos_type[k].append(ttypes[i])

        if cls.should_log(master_id) and (cls.log_mode_detection == 1 or cls.log_mode_detection == 3 or cls.log_mode_detection == 2):
            cls.imprimir_resumen_segmentos("RESUMEN RAW (DETECT SEG) - MAESTRO " + str(master_id), segmentos, segmentos_type, None, cfg)

        return segmentos, segmentos_type, extra_data_callbacks

    # =============================================================================
    # FASE 2: FILTRADO Y FUSIÓN DE SEGMENTOS
    # =============================================================================
    @classmethod
    def filter_and_validate_segments(cls, master_id, raw_seg_mov, raw_seg_type, cfg):
        n = len(raw_seg_mov)
        if n == 0:
            return [], [], []

        if cls.should_log(master_id) and cls.log_mode_filtering == 1:
            cls.custom_log("\n>>> INICIANDO PROCESO DE FILTRADO Y FUSIÓN - MAESTRO " + str(master_id))

        raw_start_is_static = False
        raw_end_is_static = False

        dist_thresh = cfg.get('distance_treshold', 15)
        if cls.get_segment_dist(raw_seg_mov[0]) < dist_thresh:
            raw_start_is_static = True
        if cls.get_segment_dist(raw_seg_mov[n - 1]) < dist_thresh:
            raw_end_is_static = True

        final_mov, final_type, final_valid = [], [], []
        accum_x, accum_y, accum_count = 0, 0, 0

        for i in range(n):
            seg = raw_seg_mov[i]
            types = raw_seg_type[i]
            dist = cls.get_segment_dist(seg)

            if dist < dist_thresh:
                p_start = seg[0]
                p_end = seg[-1]
                accum_x = accum_x + p_start[0] + p_end[0]
                accum_y = accum_y + p_start[1] + p_end[1]
                accum_count = accum_count + 2
            else:
                if accum_count > 0:
                    avg_x = accum_x / accum_count
                    avg_y = accum_y / accum_count
                    seg[0] = [avg_x, avg_y]
                    accum_x, accum_y, accum_count = 0, 0, 0
                final_mov.append(seg)
                final_type.append(types)
                final_valid.append(True)

        final_n = len(final_mov)
        if final_n > 0:
            if not raw_start_is_static:
                final_valid[0] = False
            if not raw_end_is_static:
                final_valid[final_n - 1] = False

        # =========================================================================
        # NUEVA LÓGICA: SEGUNDA FASE DE FILTRADO (Validación de cantidad)
        # =========================================================================
        descartar_todo = False

        if final_n == 1:
            # Regla 1: Si hay exactamente 1 segmento (válido o no), se elimina.
            descartar_todo = True
        elif final_n == 2:
            # Regla 2: Comportamiento según 'nivel_estricto_dos_segmentos'
            if cls.nivel_estricto_dos_segmentos == 0:
                # Mantiene 2 segmentos sin importar validez.
                descartar_todo = False
            elif cls.nivel_estricto_dos_segmentos == 1:
                # Mantiene 2 segmentos si mínimo 1 es válido (Elimina si AMBOS son inválidos)
                if not final_valid[0] and not final_valid[1]:
                    descartar_todo = True
            elif cls.nivel_estricto_dos_segmentos == 2:
                # Mantiene 2 segmentos SOLO si los dos son válidos (Elimina si ALGUNO es inválido)
                if not final_valid[0] or not final_valid[1]:
                    descartar_todo = True

        if descartar_todo:
            if cls.should_log(master_id) and cls.log_mode_filtering >= 1:
                cls.custom_log(" > [!] Segunda Fase de Filtrado: Se descartaron los segmentos resultantes (No cumplen criterios de validez/cantidad mínima).")
            # Vaciamos las tablas para forzar a 0 el conteo
            final_mov, final_type, final_valid = [], [], []
        # =========================================================================

        if cls.should_log(master_id) and (cls.log_mode_filtering == 1 or cls.log_mode_filtering == 2 or cls.log_mode_filtering == 3):
            cls.imprimir_resumen_segmentos("RESUMEN FILTRADO (FINAL) - MAESTRO " + str(master_id), final_mov, final_type, final_valid, cfg)

        return final_mov, final_type, final_valid

    # =============================================================================
    # FUNCIÓN EXPORTADA: Ejecuta segmentación, filtrado y detección de banderas
    # =============================================================================
    @classmethod
    def process(cls, master_id, raw_x, raw_y, raw_type, cfg):
        cls.log_buffer = []  # Iniciar buffer limpio por si acaso

        # =========================================================================
        # NUEVA LÓGICA: PRE-PROCESAMIENTO (SUB-SAMPLING)
        # =========================================================================
        procesado_x, procesado_y, procesado_type = [], [], []

        # Verificamos si la configuración pide usar sub-sampling
        if cfg.get('use_subsampling') and cfg.get('tamano_subsampling') and cfg['tamano_subsampling'] > 1:
            step = cfg['tamano_subsampling']

            for i in range(0, len(raw_x), step):
                procesado_x.append(raw_x[i])
                procesado_y.append(raw_y[i])
                procesado_type.append(raw_type[i])

            if cls.should_log(master_id):
                cls.custom_log(f">>> SUB-SAMPLING APLICADO: Step = {step} | Puntos reducidos de {len(raw_x)} a {len(procesado_x)}")
        else:
            # Si está apagado o no se configuró, pasamos los datos originales
            procesado_x = raw_x
            procesado_y = raw_y
            procesado_type = raw_type

        # 1. Extraemos los segmentos crudos y los datos de la fase 1 (Usando los datos procesados)
        raw_mov, r_type, extra_data = cls.detect_seg(master_id, procesado_x, procesado_y, procesado_type, cfg)

        # 2. Filtramos el ruido, validamos espacialmente y aplicamos la Segunda Fase
        clean_mov, clean_type, clean_valid = cls.filter_and_validate_segments(master_id, raw_mov, r_type, cfg)

        # 3. Verificación de segmentos (Ahora limpio, len(clean_mov) será 0 si la 2da Fase los eliminó)
        sin_segmentos_despues_del_filtro = (len(clean_mov) <= 1)

        # =========================================================================
        # A) CONTROL DE LOGS
        # =========================================================================
        if cls.log_imprimir_sin_segmentos or not sin_segmentos_despues_del_filtro:
            cls.flush_logs(True)   # Imprime en terminal
        else:
            cls.flush_logs(False)  # Silencia y descarta todo

        # 4. Detección de Patrón Visual (Banderas de Inicio y Fin)
        pattern_start = False
        pattern_end = False

        if len(clean_type) > 0:
            first_seg_types = clean_type[0]
            for t in first_seg_types:
                if t == 1:
                    pattern_start = True
                    break
            last_seg_types = clean_type[-1]
            for t in last_seg_types:
                if t == 3:
                    pattern_end = True
                    break

        return clean_mov, clean_type, clean_valid, pattern_start, pattern_end