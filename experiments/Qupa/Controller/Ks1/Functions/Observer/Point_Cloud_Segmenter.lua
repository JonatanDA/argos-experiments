-- =============================================================================
-- MÓDULO: Point_Cloud_Segmenter.lua
-- =============================================================================
-- DESCRIPCIÓN:
-- Este módulo procesa las coordenadas Pi = (xi, yi) que un robot "aprendiz" registra
-- mientras observa a un "maestro". Su objetivo es segmentar esa nube de puntos
-- continua en trazos rectos (lados) y esquinas (vértices).
--
-- EJEMPLO DE FUNCIONAMIENTO (VENTANAS DESLIZANTES):
-- Supongamos que el maestro traza una trayectoria en forma de "L" y el aprendiz registra una 
-- Secuencia de 15 puntos: [P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15]
-- Estado Físico real:     |----- MOVIMIENTO -----| |--- ESQUINA ---| |--- MOVIMIENTO ---|
--
-- 1. Exploración y Ventanas: Si el algoritmo elige un tamaño de ventana de 3,
--    calcula la varianza espacial agrupando de a 3 puntos de forma deslizante:
--    - V1 = [P1, P2, P3] -> Varianza Alta (Movimiento)
--    - V2 = [P2, P3, P4] -> Varianza Alta (Movimiento)
--    ...
--    - V8 = [P8, P9, P10] -> Varianza Baja (Entrando a esquina)
--
-- 2. Otsu: Calcula matemáticamente el umbral exacto que separa las Varianzas Altas 
--    de las Bajas a lo largo de todas las ventanas de la ruta.
--
-- 3. Histéresis (Look-Ahead): Si en V4 = [P4, P5, P6] el maestro tropieza y la varianza
--    cae por debajo del umbral de Otsu, el algoritmo NO corta el segmento de inmediato. 
--    Revisa las siguientes 'paquetes_lookahead_ventana' ventanas (V5, V6, V7, V8, V9). 
--    Como el maestro recuperó la velocidad, V5, V6 y V7 tendrán varianza alta de nuevo. 
--    Al no cumplirse el porcentaje mínimo de confirmación, ignora el tropiezo y 
--    mantiene el segmento unido.
-- =============================================================================

local PointCloudSegmenter = {}

-- Importar dependencias matemáticas
local math_helper = require("Functions.math_helper")

-- =============================================================================
-- CONFIGURACIÓN LOCAL DEL MÓDULO (Parámetros de ajuste general)
-- =============================================================================

-- --- FASE 1: Búsqueda del tamaño de ventana óptimo ---
-- Para calcular la varianza espacial, el script agrupa los puntos de la ruta.
local paso_exploracion_ventana          = 1     -- Incrementos para probar el tamaño del grupo (ej. prueba tamaños de ventana de 1, 2, 3... puntos).
local limite_maximo_ventana             = 30    -- El tamaño máximo absoluto del grupo. Nunca evaluará ventanas mayores a 30 puntos.

-- --- FASE 2 y 3: Clasificación Otsu y Filtro Look-Ahead ---
local factor_suficiencia_otsu           = 0.6   -- Si la mejor separación de Otsu posible en la ruta es 1000, un factor de 0.6 acepta cualquier ventana que logre al menos 600. Se elige la ventana más pequeña que cumpla esto para reducir tiempo de procesamiento.
local paquetes_lookahead_ventana        = 5     -- Al detectar una caída de varianza, inspecciona el resultado de las próximas 5 ventanas deslizantes hacia adelante.
local porcentaje_confirmacion_lookahead = 0.8   -- De esas 5 ventanas futuras, al menos el 80% (4 ventanas) debe mantenerse con varianza baja para confirmar que es una esquina real y no un tropiezo del maestro.

-- --- CONFIGURACIÓN DE LOGS (Depuración en terminal) ---
local log_mode_detection                = 0     -- Fase de detección. 0: Sin logs, 1: Detallado, 2: Resumen, 3: Resumen+Puntos, 4: Solo Otsu.
local mostrar_todos_los_puntos          = false -- true: Imprime todas las coordenadas (x,y) del segmento en la terminal, false: Solo el inicio y fin.

local log_mode_filtering                = 0     -- Fase de filtrado espacial. 0: Sin logs, 1: Detallado, 2: Resumen final, 3: Detalles de fusión.

-- =============================================================================
-- FUNCIONES AUXILIARES (Locales)
-- =============================================================================
local function imprimir_resumen_segmentos(titulo, lista_segmentos, lista_tipos, lista_validos, cfg)
    log("\n\n= ====================================================== ")
    log(" " .. titulo)
    log(" ======================================================= ")
    
    if #lista_segmentos == 0 then
        log(" >> No hay segmentos para mostrar.")
    end

    for idx, points in ipairs(lista_segmentos) do
        local num_puntos = #points
        local types_list = lista_tipos[idx]
        
        local etiqueta_validez = ""
        if lista_validos and lista_validos[idx] ~= nil then
            if lista_validos[idx] then etiqueta_validez = " [VALIDO]" else etiqueta_validez = " [INVALIDO]" end
        end

        if num_puntos > 0 then
            local start_pt = points[1]
            local end_pt = points[num_puntos]
            local start_type = (types_list and types_list[1]) or -1
            local end_type = (types_list and types_list[num_puntos]) or -1

            log(string.format("SEGMENTO #%d%s", idx, etiqueta_validez))
            log(string.format(" > Total Puntos: %d", num_puntos))
            log(string.format(" > Inicio: (%.3f, %.3f) [Type: %d]", start_pt[1], start_pt[2], start_type))
            log(string.format(" > Fin:    (%.3f, %.3f) [Type: %d]", end_pt[1], end_pt[2], end_type))
            
            if mostrar_todos_los_puntos then
                local buffer_linea = " > TRAYECTORIA: "
                for p_idx, p in ipairs(points) do
                    buffer_linea = buffer_linea .. string.format("(%.2f, %.2f) ", p[1], p[2])
                    if p_idx % 5 == 0 and p_idx < num_puntos then
                        log(buffer_linea)
                        buffer_linea = "                "
                    end
                end
                log(buffer_linea)
            end
            log("--------------------------------------------------------")
        end
    end
    log(" ======================================================= \n")
end

local function get_segment_dist(seg)
    if not seg or #seg < 1 then return 0 end
    local p1 = seg[1]
    local p2 = seg[#seg]
    local dx = p2[1] - p1[1]
    local dy = p2[2] - p1[2]
    return math.sqrt(dx*dx + dy*dy)
end

-- =============================================================================
-- FASE 1: DETECCIÓN DE SEGMENTOS RAW
-- =============================================================================
local function detect_seg(master_id, xx, yy, ttypes, cfg) 
    local total_puntos = #xx
    if total_puntos < 3 then return {}, {} end 

    if log_mode_detection == 1 or log_mode_detection == 4 then
        log("\n>>> INICIANDO FASE 1: EXPLORACIÓN AUTOMÁTICA DE PARÁMETROS")
    end

    -- Dinámicamente limitamos el tamaño máximo de ventana basado en la cantidad de datos reales
    local max_avance_permitido = math.floor(total_puntos / 3)
    if max_avance_permitido > limite_maximo_ventana then max_avance_permitido = limite_maximo_ventana end
    if max_avance_permitido < paso_exploracion_ventana then max_avance_permitido = paso_exploracion_ventana end

    local resultados_exploracion = {}
    local max_separacion_global = -1

    -- Búsqueda de la ventana óptima
    for av = paso_exploracion_ventana, max_avance_permitido, paso_exploracion_ventana do
        local temp_vars, temp_cx, temp_cy = {}, {}, {}
        
        for i = 1, total_puntos - av do
            local std_dev, cen_x, cen_y = math_helper.get_window_stats(i, av, xx, yy)
            table.insert(temp_vars, std_dev)
            table.insert(temp_cx, cen_x)
            table.insert(temp_cy, cen_y)
        end
        
        local umbral, separacion = math_helper.otsu_1d(temp_vars)
        
        if log_mode_detection == 1 or log_mode_detection == 4 then
            log(string.format(" Probando Avance=%d | Umbral=%.3f | Separación Otsu=%.3f", av, umbral, separacion))
        end

        table.insert(resultados_exploracion, {
            av = av, umbral = umbral, separacion = separacion,
            vars = temp_vars, cx = temp_cx, cy = temp_cy
        })

        if separacion > max_separacion_global then 
            max_separacion_global = separacion 
        end
    end

    local umbral_aceptacion = max_separacion_global * factor_suficiencia_otsu
    local mejor_avance = 5
    local mejor_umbral = 0
    local mejor_separacion_seleccionada = 0
    local mejores_varianzas = {}
    local mejores_centroides_x = {}
    local mejores_centroides_y = {}

    -- Selección del avance más pequeño que cumple el criterio de suficiencia
    for _, res in ipairs(resultados_exploracion) do
        if res.separacion >= umbral_aceptacion then
            mejor_avance = res.av
            mejor_umbral = res.umbral
            mejor_separacion_seleccionada = res.separacion
            mejores_varianzas = res.vars
            mejores_centroides_x = res.cx
            mejores_centroides_y = res.cy
            
            if log_mode_detection == 1 or log_mode_detection == 4 then
                log(string.format(" -> SELECCIONADO: Avance=%d (%.3f supera el umbral del %d%%: %.3f)", 
                    res.av, res.separacion, factor_suficiencia_otsu*100, umbral_aceptacion))
            end
            break 
        end
    end

    if log_mode_detection == 1 or log_mode_detection == 4 then
        log("\n>>> FASE 2: CONFIGURACIÓN GANADORA")
        log(string.format(" GANADOR -> Avance: %d | Umbral Dinámico: %.3f", mejor_avance, mejor_umbral))
        log("\n>>> FASE 3: SEGMENTACIÓN CON HISTÉRESIS (LOOK-AHEAD)")
    end

    -- Exportar los datos ganadores al evaluador externo de Python
    if evaluator_otsu then evaluator_otsu(mejor_separacion_seleccionada, mejor_avance) end

    local segmentos, segmentos_type = {}, {} 
    local k = 1       
    segmentos[k] = {}; segmentos_type[k] = {} 

    local ESTADO_MOVIMIENTO, ESTADO_BUSCANDO_VALLE = 1, 2
    local estado_actual = ESTADO_MOVIMIENTO

    local valle_min_var = 999999
    local valle_best_idx = -1
    local valle_cen_x, valle_cen_y = 0, 0
    local packet_counter = 1 

    -- Recorrido de los datos usando el umbral ganador y aplicando Look-Ahead
    for i = 1, #mejores_varianzas do
        local std_dev = mejores_varianzas[i]
        local x, y = xx[i], yy[i]
        local ty = ttypes[i]
        local cx, cy = mejores_centroides_x[i], mejores_centroides_y[i]

        if log_mode_detection == 1 or log_mode_detection == 3 then
            local end_idx = i + mejor_avance - 1 
            local end_x, end_y = xx[end_idx], yy[end_idx] 
            log(string.format("Pkt %d | Idx[%d-%d] Pos(%.2f, %.2f)->(%.2f, %.2f) | Var=%.4f", 
                packet_counter, i, end_idx, x, y, end_x, end_y, std_dev))
        end
        
        if Print_tracker_varianza then Print_tracker_varianza(master_id, std_dev) end
        packet_counter = packet_counter + 1

        if estado_actual == ESTADO_MOVIMIENTO then
            if std_dev > mejor_umbral then
                table.insert(segmentos[k], { x, y }); table.insert(segmentos_type[k], ty)
            else
                local count_bajo_umbral = 0
                local limite_busqueda = math.min(i + paquetes_lookahead_ventana - 1, #mejores_varianzas)
                local puntos_a_revisar = limite_busqueda - i + 1

                for j = i, limite_busqueda do
                    if mejores_varianzas[j] <= mejor_umbral then count_bajo_umbral = count_bajo_umbral + 1 end
                end

                local porcentaje_real = count_bajo_umbral / puntos_a_revisar

                if porcentaje_real >= porcentaje_confirmacion_lookahead then
                    estado_actual = ESTADO_BUSCANDO_VALLE
                    valle_min_var = std_dev; valle_best_idx = i
                    valle_cen_x, valle_cen_y = cx, cy
                else
                    table.insert(segmentos[k], { x, y }); table.insert(segmentos_type[k], ty)
                end
            end

        elseif estado_actual == ESTADO_BUSCANDO_VALLE then
            if std_dev <= mejor_umbral then
                if std_dev < valle_min_var then
                    valle_min_var = std_dev; valle_best_idx = i
                    valle_cen_x, valle_cen_y = cx, cy
                end
            else
                local count_sobre_umbral = 0
                local limite_busqueda = math.min(i + paquetes_lookahead_ventana - 1, #mejores_varianzas)
                local puntos_a_revisar = limite_busqueda - i + 1

                for j = i, limite_busqueda do
                    if mejores_varianzas[j] > mejor_umbral then count_sobre_umbral = count_sobre_umbral + 1 end
                end

                local porcentaje_real = count_sobre_umbral / puntos_a_revisar

                if porcentaje_real >= porcentaje_confirmacion_lookahead then
                    local pivot_point = { valle_cen_x, valle_cen_y }
                    local pivot_type = ttypes[valle_best_idx]

                    table.insert(segmentos[k], pivot_point); table.insert(segmentos_type[k], pivot_type)
                    
                    k = k + 1
                    segmentos[k] = {}; segmentos_type[k] = {}
                    table.insert(segmentos[k], pivot_point); table.insert(segmentos_type[k], pivot_type)
                    table.insert(segmentos[k], { x, y }); table.insert(segmentos_type[k], ty)
                    
                    estado_actual = ESTADO_MOVIMIENTO
                end
            end
        end
    end 
     
    -- Forzar cierre si la simulación terminó mientras se buscaba un valle
    if estado_actual == ESTADO_BUSCANDO_VALLE then
        local pivot_point = { valle_cen_x, valle_cen_y }
        local pivot_type = ttypes[valle_best_idx]
        table.insert(segmentos[k], pivot_point); table.insert(segmentos_type[k], pivot_type)

        k = k + 1
        segmentos[k] = {}; segmentos_type[k] = {}
        table.insert(segmentos[k], pivot_point); table.insert(segmentos_type[k], pivot_type)
    end

    -- Insertar el resto de los puntos finales
    for i = #mejores_varianzas + 1, total_puntos do 
        table.insert(segmentos[k], { xx[i], yy[i] }); table.insert(segmentos_type[k], ttypes[i]) 
    end 

    if log_mode_detection == 1 or log_mode_detection == 3 or log_mode_detection == 2 then
        imprimir_resumen_segmentos("RESUMEN RAW (DETECT SEG)", segmentos, segmentos_type, nil, cfg)
    end

    return segmentos, segmentos_type 
end

-- =============================================================================
-- FASE 2: FILTRADO Y FUSIÓN DE SEGMENTOS
-- =============================================================================
local function filter_and_validate_segments(raw_seg_mov, raw_seg_type, cfg)
    local n = #raw_seg_mov
    if n == 0 then return {}, {}, {} end

    if log_mode_filtering == 1 then log("\n>>> INICIANDO PROCESO DE FILTRADO Y FUSIÓN") end

    local raw_start_is_static = false
    local raw_end_is_static = false

    if get_segment_dist(raw_seg_mov[1]) < cfg.distance_treshold then raw_start_is_static = true end
    if get_segment_dist(raw_seg_mov[n]) < cfg.distance_treshold then raw_end_is_static = true end

    local final_mov, final_type, final_valid = {}, {}, {} 
    local accum_x, accum_y, accum_count = 0, 0, 0
    
    for i = 1, n do
        local seg = raw_seg_mov[i]
        local types = raw_seg_type[i]
        local dist = get_segment_dist(seg)

        if dist < cfg.distance_treshold then
            local p_start = seg[1]
            local p_end = seg[#seg]
            accum_x = accum_x + p_start[1] + p_end[1]
            accum_y = accum_y + p_start[2] + p_end[2]
            accum_count = accum_count + 2
        else
            if accum_count > 0 then
                local avg_x = accum_x / accum_count
                local avg_y = accum_y / accum_count
                seg[1] = { avg_x, avg_y } 
                accum_x, accum_y, accum_count = 0, 0, 0
            end
            table.insert(final_mov, seg); table.insert(final_type, types); table.insert(final_valid, true) 
        end
    end

    local final_n = #final_mov
    if final_n > 0 then
        if not raw_start_is_static then final_valid[1] = false end
        if not raw_end_is_static then final_valid[final_n] = false end
    end

    if log_mode_filtering == 1 or log_mode_filtering == 2 or log_mode_filtering == 3 then 
        imprimir_resumen_segmentos("RESUMEN FILTRADO (FINAL)", final_mov, final_type, final_valid, cfg) 
    end

    return final_mov, final_type, final_valid
end

-- =============================================================================
-- FUNCIÓN EXPORTADA: Ejecuta segmentación, filtrado y detección de banderas
-- =============================================================================
function PointCloudSegmenter.process(master_id, raw_x, raw_y, raw_type, cfg)
    -- 1. Extraemos los segmentos crudos
    local raw_mov, r_type = detect_seg(master_id, raw_x, raw_y, raw_type, cfg)
    -- 2. Filtramos el ruido y validamos espacialmente
    local clean_mov, clean_type, clean_valid = filter_and_validate_segments(raw_mov, r_type, cfg)
    
    -- 3. Detección de Patrón Visual (Banderas de Inicio y Fin)
    local pattern_start = false
    local pattern_end = false

    if #clean_type > 0 then
        -- Revisar primer segmento limpio
        local first_seg_types = clean_type[1]
        for _, t in ipairs(first_seg_types) do
            if t == 1 then pattern_start = true; break end
        end
        -- Revisar último segmento limpio
        local last_seg_types = clean_type[#clean_type]
        for _, t in ipairs(last_seg_types) do
            if t == 3 then pattern_end = true; break end
        end
    end
    
    -- Retornamos la geometría limpia Y las banderas visuales detectadas
    return clean_mov, clean_type, clean_valid, pattern_start, pattern_end
end

return PointCloudSegmenter