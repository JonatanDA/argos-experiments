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
local paso_exploracion_ventana          = 1     
-- local limite_maximo_ventana             = 60    

-- --- FASE 2 y 3: Clasificación Otsu y Filtro Look-Ahead ---
local factor_suficiencia_otsu           = 1.0   
local paquetes_lookahead_ventana        = 5     
local porcentaje_confirmacion_lookahead = 0.8   

-- --- CONFIGURACIÓN DE LOGS (Depuración en terminal) ---

-- Maestros cuyos logs se desean mostrar.
-- {} = muestra los logs de TODOS los maestros.
-- {"Master1"} = muestra únicamente los logs de Master1.
-- {"Master1", "Master3"} = muestra únicamente los logs de Master1 y Master3.
local maestros_a_loggear                = {}    


-- Control del log específico de Otsu.
-- 0 = No muestra información de Otsu.
-- 1 = Muestra información de Otsu.
local log_otsu                          = 1     


-- Nivel de detalle de los logs de detección/segmentación.
-- 0 = No muestra logs de detección.
-- 1 = Muestra todos los logs: exploración, configuración ganadora,
--     paquetes/varianzas y resumen de segmentos.
-- 2 = Muestra los resúmenes de segmentos y la configuración general,
--     pero no muestra el detalle de cada paquete/ventana.
-- 3 = Muestra el detalle de los paquetes/ventanas y los resúmenes,
--     pero no la exploración completa de Otsu.
local log_mode_detection                = 2     


-- Controla si se imprimen todos los puntos de cada segmento.
-- false = Solo muestra inicio, fin y cantidad de puntos.
-- true  = Además muestra toda la trayectoria de puntos de cada segmento.
local mostrar_todos_los_puntos          = true 


-- Nivel de detalle de los logs del filtrado y fusión de segmentos.
-- 0 = No muestra logs del filtrado.
-- 1 = Muestra información del proceso de filtrado y el resumen final.
-- 2 = Muestra el resumen final del filtrado.
-- 3 = Muestra el resumen final del filtrado.
local log_mode_filtering                = 1
-- =============================================================================
-- FUNCIONES AUXILIARES (Locales y de Buffering)
-- =============================================================================

local log_buffer = {}

local function custom_log(msg)
    table.insert(log_buffer, msg)
end

local function flush_logs(should_print)
    if should_print then
        for _, msg in ipairs(log_buffer) do
            log(msg)
        end
    end
    log_buffer = {} -- Limpiar buffer
end

local function should_log(master_id)
    if #maestros_a_loggear == 0 then return true end
    for _, id in ipairs(maestros_a_loggear) do
        if id == master_id then return true end
    end
    return false
end

local function imprimir_resumen_segmentos(titulo, lista_segmentos, lista_tipos, lista_validos, cfg)
    custom_log("\n\n= ====================================================== ")
    custom_log(" " .. titulo)
    custom_log(" ======================================================= ")
    
    if #lista_segmentos == 0 then
        custom_log(" >> No hay segmentos para mostrar.")
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

            custom_log(string.format("SEGMENTO #%d%s", idx, etiqueta_validez))
            custom_log(string.format(" > Total Puntos: %d", num_puntos))
            custom_log(string.format(" > Inicio: (%.3f, %.3f) [Type: %d]", start_pt[1], start_pt[2], start_type))
            custom_log(string.format(" > Fin:    (%.3f, %.3f) [Type: %d]", end_pt[1], end_pt[2], end_type))
            
            if mostrar_todos_los_puntos then
                local buffer_linea = " > TRAYECTORIA: "
                for p_idx, p in ipairs(points) do
                    buffer_linea = buffer_linea .. string.format("(%.2f, %.2f) ", p[1], p[2])
                    if p_idx % 5 == 0 and p_idx < num_puntos then
                        custom_log(buffer_linea)
                        buffer_linea = "                "
                    end
                end
                custom_log(buffer_linea)
            end
            custom_log("--------------------------------------------------------")
        end
    end
    custom_log(" ======================================================= \n")
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
    if total_puntos < 3 then return {}, {}, nil end 

    if should_log(master_id) and (log_mode_detection == 1 or log_otsu == 1) then
        custom_log("\n>>> INICIANDO FASE 1: EXPLORACIÓN AUTOMÁTICA DE PARÁMETROS - MAESTRO " .. tostring(master_id))
    end

    local max_avance_permitido = math.floor(total_puntos / 3)
    if max_avance_permitido > cfg.limite_maximo_ventana then max_avance_permitido = cfg.limite_maximo_ventana end
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
        
        if should_log(master_id) and (log_mode_detection == 1 or log_otsu == 1) then
            custom_log(string.format(" Probando Avance=%d | Umbral=%.3f | Separación Otsu=%.3f", av, umbral, separacion))
        end

        table.insert(resultados_exploracion, {
            av = av, umbral = umbral, separacion = separacion,
            vars = temp_vars, cx = temp_cx, cy = temp_cy
        })

        if separacion > max_separacion_global then 
            max_separacion_global = separacion 
        end
    end

    -- DETECCIÓN DE COMPORTAMIENTO EN "U" (BAJA Y LUEGO SUBE)
    local is_u_shape = false
    local first_valley_idx = 1
    local decreased = false
    
    for i = 2, #resultados_exploracion do
        if resultados_exploracion[i].separacion < resultados_exploracion[i-1].separacion then
            decreased = true
        elseif decreased and resultados_exploracion[i].separacion > resultados_exploracion[i-1].separacion then
            is_u_shape = true
            first_valley_idx = i - 1
            break
        end
    end

    local umbral_aceptacion = 0
    local start_search_idx = 1

    if is_u_shape then
        start_search_idx = first_valley_idx
        local max_after_valley = -1
        -- Solo consideramos los picos generados DESPUÉS del valle de ruido
        for i = start_search_idx, #resultados_exploracion do
            if resultados_exploracion[i].separacion > max_after_valley then
                max_after_valley = resultados_exploracion[i].separacion
            end
        end
        umbral_aceptacion = max_after_valley * factor_suficiencia_otsu
        
        if should_log(master_id) and (log_mode_detection == 1 or log_otsu == 1) then
            custom_log(" > [!] Comportamiento en 'U' detectado. Ignorando ruido inicial (valle detectado en Avance=" .. resultados_exploracion[first_valley_idx].av .. ").")
        end
    else
        umbral_aceptacion = max_separacion_global * factor_suficiencia_otsu
    end

    -- Selección del avance que cumple el criterio de suficiencia
    local mejor_avance = resultados_exploracion[start_search_idx].av
    local mejor_umbral = resultados_exploracion[start_search_idx].umbral
    local mejor_separacion_seleccionada = resultados_exploracion[start_search_idx].separacion
    local mejores_varianzas = resultados_exploracion[start_search_idx].vars
    local mejores_centroides_x = resultados_exploracion[start_search_idx].cx
    local mejores_centroides_y = resultados_exploracion[start_search_idx].cy

    for i = start_search_idx, #resultados_exploracion do
        local res = resultados_exploracion[i]
        if res.separacion >= (umbral_aceptacion - 0.0001) then
            mejor_avance = res.av
            mejor_umbral = res.umbral
            mejor_separacion_seleccionada = res.separacion
            mejores_varianzas = res.vars
            mejores_centroides_x = res.cx
            mejores_centroides_y = res.cy
            
            if should_log(master_id) and (log_mode_detection == 1 or log_otsu == 1) then
                custom_log(string.format(" -> SELECCIONADO: Avance=%d (%.3f supera el umbral del %d%%: %.3f)", 
                    res.av, res.separacion, factor_suficiencia_otsu*100, umbral_aceptacion))
            end
            break 
        end
    end

    if should_log(master_id) and (log_mode_detection == 1 or log_otsu == 1) then
        custom_log("\n>>> FASE 2: CONFIGURACIÓN GANADORA")
        custom_log(string.format(" GANADOR -> Avance: %d | Umbral Dinámico: %.3f", mejor_avance, mejor_umbral))
        custom_log("\n>>> FASE 3: SEGMENTACIÓN CON HISTÉRESIS (LOOK-AHEAD)")
    end

    local extra_data_callbacks = {
        mejor_separacion = mejor_separacion_seleccionada,
        mejor_avance = mejor_avance,
        varianzas = mejores_varianzas
    }

    local segmentos, segmentos_type = {}, {} 
    local k = 1       
    segmentos[k] = {}; segmentos_type[k] = {} 

    local ESTADO_MOVIMIENTO, ESTADO_BUSCANDO_VALLE = 1, 2
    local estado_actual = ESTADO_MOVIMIENTO

    local valle_min_var = 999999
    local valle_best_idx = -1
    local valle_cen_x, valle_cen_y = 0, 0
    local packet_counter = 1 

    for i = 1, #mejores_varianzas do
        local std_dev = mejores_varianzas[i]
        local x, y = xx[i], yy[i]
        local ty = ttypes[i]
        local cx, cy = mejores_centroides_x[i], mejores_centroides_y[i]

        if should_log(master_id) and (log_mode_detection == 1 or log_mode_detection == 3) then
            local end_idx = i + mejor_avance - 1 
            local end_x, end_y = xx[end_idx], yy[end_idx] 
            custom_log(string.format("Pkt %d | Idx[%d-%d] Pos(%.2f, %.2f)->(%.2f, %.2f) | Var=%.4f", 
                packet_counter, i, end_idx, x, y, end_x, end_y, std_dev))
        end
        
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
     
    if estado_actual == ESTADO_BUSCANDO_VALLE then
        local pivot_point = { valle_cen_x, valle_cen_y }
        local pivot_type = ttypes[valle_best_idx]
        table.insert(segmentos[k], pivot_point); table.insert(segmentos_type[k], pivot_type)

        k = k + 1
        segmentos[k] = {}; segmentos_type[k] = {}
        table.insert(segmentos[k], pivot_point); table.insert(segmentos_type[k], pivot_type)
    end

    for i = #mejores_varianzas + 1, total_puntos do 
        table.insert(segmentos[k], { xx[i], yy[i] }); table.insert(segmentos_type[k], ttypes[i]) 
    end 

    if should_log(master_id) and (log_mode_detection == 1 or log_mode_detection == 3 or log_mode_detection == 2) then
        imprimir_resumen_segmentos("RESUMEN RAW (DETECT SEG) - MAESTRO " .. tostring(master_id), segmentos, segmentos_type, nil, cfg)
    end

    return segmentos, segmentos_type, extra_data_callbacks
end

-- =============================================================================
-- FASE 2: FILTRADO Y FUSIÓN DE SEGMENTOS
-- =============================================================================
local function filter_and_validate_segments(master_id, raw_seg_mov, raw_seg_type, cfg)
    local n = #raw_seg_mov
    if n == 0 then return {}, {}, {} end

    if should_log(master_id) and log_mode_filtering == 1 then custom_log("\n>>> INICIANDO PROCESO DE FILTRADO Y FUSIÓN - MAESTRO " .. tostring(master_id)) end

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

    -- =========================================================================
    -- NUEVA LÓGICA: SEGUNDA FASE DE FILTRADO (Validación de cantidad)
    -- =========================================================================
    local descartar_todo = false

    if final_n == 1 then
        -- Regla 1: Si hay exactamente 1 segmento (válido o no), se elimina.
        descartar_todo = true
    elseif final_n == 2 then
        -- Regla 2: Comportamiento según 'nivel_estricto_dos_segmentos'
        if nivel_estricto_dos_segmentos == 0 then
            -- Mantiene 2 segmentos sin importar validez.
            descartar_todo = false
        elseif nivel_estricto_dos_segmentos == 1 then
            -- Mantiene 2 segmentos si mínimo 1 es válido (Elimina si AMBOS son inválidos)
            if not final_valid[1] and not final_valid[2] then
                descartar_todo = true
            end
        elseif nivel_estricto_dos_segmentos == 2 then
            -- Mantiene 2 segmentos SOLO si los dos son válidos (Elimina si ALGUNO es inválido)
            if not final_valid[1] or not final_valid[2] then
                descartar_todo = true
            end
        end
    end

    if descartar_todo then
        if should_log(master_id) and log_mode_filtering >= 1 then
            custom_log(" > [!] Segunda Fase de Filtrado: Se descartaron los segmentos resultantes (No cumplen criterios de validez/cantidad mínima).")
        end
        -- Vaciamos las tablas para forzar a 0 el conteo
        final_mov, final_type, final_valid = {}, {}, {}
    end
    -- =========================================================================

    if should_log(master_id) and (log_mode_filtering == 1 or log_mode_filtering == 2 or log_mode_filtering == 3) then 
        imprimir_resumen_segmentos("RESUMEN FILTRADO (FINAL) - MAESTRO " .. tostring(master_id), final_mov, final_type, final_valid, cfg) 
    end

    return final_mov, final_type, final_valid
end

-- =============================================================================
-- FUNCIÓN EXPORTADA: Ejecuta segmentación, filtrado y detección de banderas
-- =============================================================================
function PointCloudSegmenter.process(master_id, raw_x, raw_y, raw_type, cfg)
    log_buffer = {} -- Iniciar buffer limpio por si acaso
    
    -- =========================================================================
    -- NUEVA LÓGICA: PRE-PROCESAMIENTO (SUB-SAMPLING)
    -- =========================================================================
    local procesado_x, procesado_y, procesado_type = {}, {}, {}
    
    -- Verificamos si la configuración pide usar sub-sampling
    if cfg.use_subsampling and cfg.tamano_subsampling and cfg.tamano_subsampling > 1 then
        local step = cfg.tamano_subsampling
        
        for i = 1, #raw_x, step do
            table.insert(procesado_x, raw_x[i])
            table.insert(procesado_y, raw_y[i])
            table.insert(procesado_type, raw_type[i])
        end
        
        if should_log(master_id) then
            custom_log(string.format(">>> SUB-SAMPLING APLICADO: Step = %d | Puntos reducidos de %d a %d", step, #raw_x, #procesado_x))
        end
    else
        -- Si está apagado o no se configuró, pasamos los datos originales
        procesado_x = raw_x
        procesado_y = raw_y
        procesado_type = raw_type
    end
    
    -- 1. Extraemos los segmentos crudos y los datos de la fase 1 (Usando los datos procesados)
    local raw_mov, r_type, extra_data = detect_seg(master_id, procesado_x, procesado_y, procesado_type, cfg)
    
    -- 2. Filtramos el ruido, validamos espacialmente y aplicamos la Segunda Fase
    local clean_mov, clean_type, clean_valid = filter_and_validate_segments(master_id, raw_mov, r_type, cfg)
    
    -- 3. Verificación de segmentos (Ahora limpio, #clean_mov será 0 si la 2da Fase los eliminó)
    local sin_segmentos_despues_del_filtro = (#clean_mov <= 1)

    -- =========================================================================
    -- A) CONTROL DE LOGS
    -- =========================================================================
    if log_imprimir_sin_segmentos or not sin_segmentos_despues_del_filtro then
        flush_logs(true) -- Imprime en terminal
    else
        flush_logs(false) -- Silencia y descarta todo
    end

    -- =========================================================================
    -- B) CONTROL DE FUNCIONES EXTERNAS
    -- =========================================================================
    if not sin_segmentos_despues_del_filtro then
        if evaluator_otsu and extra_data then 
            evaluator_otsu(extra_data.mejor_separacion, extra_data.mejor_avance) 
        end
        if Print_tracker_varianza and extra_data and extra_data.varianzas then
            for _, std_dev in ipairs(extra_data.varianzas) do
                Print_tracker_varianza(master_id, std_dev)
            end
        end
    end

    -- 4. Detección de Patrón Visual (Banderas de Inicio y Fin)
    local pattern_start = false
    local pattern_end = false

    if #clean_type > 0 then
        local first_seg_types = clean_type[1]
        for _, t in ipairs(first_seg_types) do
            if t == 1 then pattern_start = true; break end
        end
        local last_seg_types = clean_type[#clean_type]
        for _, t in ipairs(last_seg_types) do
            if t == 3 then pattern_end = true; break end
        end
    end
    
    return clean_mov, clean_type, clean_valid, pattern_start, pattern_end
end

return PointCloudSegmenter