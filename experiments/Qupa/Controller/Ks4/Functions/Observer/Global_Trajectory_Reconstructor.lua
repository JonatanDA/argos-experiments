-- Archivo: Functions/Observer/Global_Trajectory_Reconstructor.lua

local GlobalReconstructor = {}

-- ============================================================================
-- VARIABLES DE CONTROL Y LOGGEO
-- ============================================================================
local LOG_TABLA_FRAGMENTOS  = true  -- 1. true/false: Imprimir o no la tabla de fragmentos recolectados
local LOG_TERMOSTATO        = true  -- 2. true/false: Imprimir o no la auto-calibración y el filtro de ruido
local LOG_ARBITRO           = 2     -- 3. 0: Nada, 1: Resumen ganador, 2: Detalle de puntajes y caminos propuestos
local LOG_TRADUCCION        = true  -- 4. true/false: Imprimir o no la traducción física de los patrones finales
local LOG_RECURSION_DEBUG   = false -- 5. true/false: Activa impresiones para evitar congelamientos en M2
local ENABLE_DYNAMIC_CLOSURE= true  -- 6. true/false: Activa el cierre inteligente buscando el segmento más cercano al origen
local FORCE_CLOSE_IRREGULAR = false -- 7. true/false: Activa el cierre forzado añadiendo un segmento final hacia el origen (M1/M2).
local LOG_MAPEO_IDS         = true  -- 8. true/false: Imprimir qué IDs locales fueron absorbidos por cada comportamiento ganador.

-- ============================================================================
-- HIPERPARÁMETROS LOCALES DEL SISTEMA
-- ============================================================================

-- ------------------------------------------------------------------------
-- 1. SOLAPES Y TAMAÑO DE ANÁLISIS

-- Tamaño del N-Grama (Cantidad de transiciones consecutivas evaluadas como un bloque).
-- 2 = Pares (Distancia, Ángulo). 3 = Tríos (Distancia, Ángulo, Distancia).
-- Si el sistema de captura descarta fragmentos con menos de 3 datos, subir este valor evita problemas de filtrado.
local NGRAM_SIZE = 3

-- Solape mínimo independiente para el método rápido (Greedy / M1).
local MIN_OVERLAP_M1 = 3

-- Solape mínimo base para el método exhaustivo (Backtracking / M2).
local MIN_OVERLAP_M2 = 3

-- ------------------------------------------------------------------------
-- 2. SEGURIDAD Y RENDIMIENTO (PROTECCIÓN CONTRA CONGELAMIENTOS)

-- Límite máximo de llamadas recursivas permitidas en M2.
local MAX_RECURSIONS_M2 = 5000

-- Interruptor booleano (true/false) para imprimir en consola las rutas exploradas por M2.
local PRINT_RECURSIVE_PATHS = true

-- Límite máximo de fragmentos independientes que M2 está autorizado a procesar.
local MAX_FRAGMENTS_M2 = 9

-- ------------------------------------------------------------------------
-- 3. PONDERACIÓN Y SELECCIÓN DE PATRONES

-- Peso de la frecuencia (Premia patrones repetidos consistentemente).
local W_FREQ = 1

-- Peso de la longitud de la trayectoria.
-- Valores positivos favorecen rutas cortas (restan puntos por tamaño); negativos favorecen rutas largas.
local W_LEN = 0

-- Peso de la calidad de los solapes (Premia uniones robustas y limpias).
local W_OV_QUAL = 2

-- Tolerancia matemática interna utilizada en las comprobaciones geométricas del Geómetra (M3).
-- Mide los grados de error aceptados para que una figura cierre perfectamente en 360°.
local MATH_TOLERANCE = 15

-- ------------------------------------------------------------------------
-- 4. FILTRADO DE RUIDO (TOLERANCIA A LA BASURA DEL SENSOR)

-- Piso mínimo absoluto de filtrado (0.2 equivalentes al 20%).
local RUIDO_BASE = 0.05

-- Margen de ajuste adicional para el ruido (0.15 equivalentes al +15%).
local RUIDO_PENALIZACION_MAX = 0.15

-- ------------------------------------------------------------------------
-- 5. EL GEÓMETRA M3 Y EL ÁRBITRO (LÓGICA DE DECISIÓN)

-- Multiplicador base neutral para las geometrías regulares (polígonos cerrados).
local BONO_M3_BASE = 1.0

-- Premio máximo adicional en la puntuación por simetría perfecta.
local BONO_M3_MAX_EXTRA = 0.5

-- Cobertura mínima absoluta requerida para que el árbitro acepte un patrón (0.25 = 25%).
local SCORE_MIN_BASE = 0.25

-- Blindaje estadístico dinámico (Eleva exigencia en escenarios caóticos).
local SCORE_PENALIZACION_MAX = 0.10

-- Constante de penalización estructural aplicada a trayectorias demasiado largas (Navaja de Ockham).
local PESO_COMPLEJIDAD = 0.05

local ARBITER_CONFIG = {}
local ngram_db = {}
local raw_solutions_found = {} 

local function fmt_struct(s)
    if not s then return "nil" end
    local vals = {}
    for _, v in ipairs(s.value) do table.insert(vals, string.format("%.2f", v)) end
    local t_str = (s.type == 1) and "Ang" or "Dist"
    local count_str = (s.count and s.count > 1) and string.format(" [x%d]", s.count) or ""
    return string.format("(%s)%s {%s}", t_str, count_str, table.concat(vals, ", "))
end

local function get_val_type(struct_type, index)
    local is_odd = (index % 2 ~= 0)
    if struct_type == 1 then return is_odd and "A" or "D"
    else return is_odd and "D" or "A" end
end

local function format_ngram(ngr)
    local parts = {}
    for k, v in ipairs(ngr.values) do
        local is_dist = (ngr.start_type == 0) and (k%2 ~= 0) or (ngr.start_type == 1) and (k%2 == 0)
        local lbl = is_dist and "Dist" or "Ang"
        table.insert(parts, string.format("%s: %.2f", lbl, v))
    end
    return "(" .. table.concat(parts, ", ") .. ")"
end

local function ngrams_match(ng1, ng2, cfg)
    if ng1.start_type ~= ng2.start_type then return false end
    if #ng1.values ~= #ng2.values then return false end
    for k = 1, #ng1.values do
        local is_dist = (ng1.start_type == 0) and (k%2 ~= 0) or (ng1.start_type == 1) and (k%2 == 0)
        local tol = is_dist and cfg.CONFIG.TOL_DIST or cfg.CONFIG.TOL_ANG
        if math.abs(ng1.values[k] - ng2.values[k]) > tol then return false end
    end
    return true
end

local function extract_ngrams_from_fragment(struct, cfg)
    local ngrams = {}
    local is_dist_first = (struct.type == 0)
    
    if #struct.value < NGRAM_SIZE then return {}, false end
    
    local is_pure = false
    if #struct.value >= 3 then
        is_pure = true
        for i = 1, #struct.value - 2 do
            local v1 = struct.value[i]
            local v2 = struct.value[i+2]
            local v_type = get_val_type(struct.type, i)
            local tol = (v_type == "A") and cfg.CONFIG.TOL_ANG or cfg.CONFIG.TOL_DIST
            if math.abs(v1 - v2) > tol then
                is_pure = false
                break
            end
        end
    end
    
    for i = 1, #struct.value - NGRAM_SIZE + 1 do
        local ngr_vals = {}
        for k = 0, NGRAM_SIZE - 1 do
            table.insert(ngr_vals, struct.value[i + k])
        end
        
        local is_v1_dist = is_dist_first and (i%2 ~= 0) or (not is_dist_first and i%2 == 0)
        local p_type = is_v1_dist and 0 or 1
        
        table.insert(ngrams, {values = ngr_vals, start_type = p_type, is_pure = is_pure})
    end
    
    return ngrams, is_pure
end

local function build_global_ngram_db(pool, cfg)
    ngram_db = {}
    for _, struct in ipairs(pool) do
        local fragment_ngrams, is_pure = extract_ngrams_from_fragment(struct, cfg)
        for _, ngr in ipairs(fragment_ngrams) do
            local found = false
            for _, db_ngr in ipairs(ngram_db) do
                if ngrams_match(db_ngr, ngr, cfg) then
                    for k = 1, NGRAM_SIZE do
                        db_ngr.values[k] = db_ngr.values[k] + (ngr.values[k] - db_ngr.values[k]) / (db_ngr.merges + 1)
                    end
                    db_ngr.count = db_ngr.count + (struct.count or 1)
                    db_ngr.merges = db_ngr.merges + 1
                    db_ngr.is_pure = db_ngr.is_pure or is_pure 
                    found = true
                    break
                end
            end
            if not found then 
                local vals_copy = {}
                for _, v in ipairs(ngr.values) do table.insert(vals_copy, v) end
                table.insert(ngram_db, {values = vals_copy, start_type = ngr.start_type, count = (struct.count or 1), merges = 1, is_pure = is_pure}) 
            end
        end
    end
end

local function calibrar_hiperparametros(freq_array, current_ngram_db, cfg)
    if #freq_array == 0 then return end

    local max_frecuencia = 0
    local suma_frecuencias = 0
    local pureza_total = 0
    local ngrams_dominantes = {}

    for _, ngr in ipairs(current_ngram_db) do
        if ngr.count > max_frecuencia then
            max_frecuencia = ngr.count
            ngrams_dominantes = { ngr }
        elseif ngr.count == max_frecuencia then
            table.insert(ngrams_dominantes, ngr)
        end
    end

    for _, f in ipairs(freq_array) do
        suma_frecuencias = suma_frecuencias + f
    end
    
    for _, ngr in ipairs(current_ngram_db) do
        if ngr.is_pure then pureza_total = pureza_total + ngr.count end
    end

    local prom_frecuencia = suma_frecuencias / #freq_array
    
    local indice_orden = 1.0
    if suma_frecuencias > 0 then
        indice_orden = 1.0 - (#freq_array / suma_frecuencias)
        if indice_orden < 0.0 then indice_orden = 0.0 end
    end
    
    ARBITER_CONFIG.INDICE_ORDEN = indice_orden 
    
    local ratio_pureza = 0
    if suma_frecuencias > 0 then
        ratio_pureza = pureza_total / suma_frecuencias
    end

    local nivel_caos = 1.0 - indice_orden
    local ruido_extra = nivel_caos * RUIDO_PENALIZACION_MAX
    ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO = RUIDO_BASE + ruido_extra

    local bono_extra = ratio_pureza * BONO_M3_MAX_EXTRA * indice_orden
    ARBITER_CONFIG.BONO_REGULARIDAD = BONO_M3_BASE + bono_extra

    local penalizacion_caos = nivel_caos * SCORE_PENALIZACION_MAX
    ARBITER_CONFIG.MIN_SCORE = SCORE_MIN_BASE + penalizacion_caos

    local repeticiones_minimas = max_frecuencia * ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO

    if LOG_TERMOSTATO then
        log("\n=========================================")
        log(" [TERMOSTATO] AUTO-CALIBRACIÓN DEL ENJAMBRE")
        log("=========================================")
        log("  [1] Extracción de Métricas Base:")
        log(string.format("Frecuencia Máxima Dominante: %d", max_frecuencia))
        for _, ngr in ipairs(ngrams_dominantes) do
            log(string.format("        -> Corresponde al N-Grama: %s", format_ngram(ngr)))
        end
        log(string.format("Frecuencia Promedio Global: %.2f", prom_frecuencia))
        log(string.format("Volumen Total (Cantidad de N-Gramas): %d", suma_frecuencias))
        log(string.format("N-Gramas Únicos: %d", #freq_array))
        log(string.format("N-Gramas con Simetría (Periodicos): %d", pureza_total))
        
        log("\n  [2] Diagnóstico del Entorno (Fórmulas):")
        log("Índice de Orden: Mide qué tan repetitivos son los datos (1.0 = patrón dominante claro, 0 = caos).")
        log(string.format("        -> 1.0 - (Únicos [%d] / Volumen [%d]) = %.2f", #freq_array, suma_frecuencias, indice_orden))
        log("Ratio de Pureza: Mide la proporción de geometrías perfectas (como polígonos cerrados).")
        log(string.format("        -> Periodicos [%d] / Volumen [%d] = %.2f", pureza_total, suma_frecuencias, ratio_pureza))

        log("\n  [3] Fórmulas y Resultados Aplicados:")
        log("Umbral de Ruido: Porcentaje mínimo de repetición para que un dato NO sea descartado.")
        log(string.format("        > FÓRMULA UMBRAL : Base(%.2f) + [Caos(%.2f) * Max_Pen(%.2f)] = %.1f%%.", 
            RUIDO_BASE, nivel_caos, RUIDO_PENALIZACION_MAX, ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO * 100))
        log(string.format("        > LÍMITE ABSOLUTO: %.1f%% de %d repeticiones (frecuencia dominante).", ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO * 100, max_frecuencia))
        log(string.format("        > RESULTADO      : Cualquier fragmento con MENOS de %.2f repeticiones es RUIDO.", repeticiones_minimas))
        log("=========================================")
    end
end

local function calcular_limite_ruido(frecuencias)
    if #frecuencias == 0 then return 0 end
    local max_freq = 0
    for _, f in ipairs(frecuencias) do
        if f > max_freq then max_freq = f end
    end
    return max_freq * ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO
end

local function check_subset_match(long_val, short_val, start_idx, long_type, short_type, cfg)
    if get_val_type(long_type, start_idx) ~= get_val_type(short_type, 1) then return false end
    for k = 1, #short_val do
        local l_idx = start_idx + k - 1
        local v_type = get_val_type(short_type, k)
        local diff = math.abs(long_val[l_idx] - short_val[k])
        local tol = (v_type == "A") and cfg.CONFIG.TOL_ANG or cfg.CONFIG.TOL_DIST
        if diff > tol then return false end
    end
    return true
end

local function reduce_fragments_pool(pool, cfg)
    local reduced_pool = {}
    local group_logs = {}
    local skip = {}
    
    table.sort(pool, function(a,b) return #a.value > #b.value end)
    
    for i = 1, #pool do
        if not skip[i] then
            local current_target = pool[i]
            local absorbed_ids = { current_target.id }
            local merged_any = false
            
            for j = i + 1, #pool do
                if not skip[j] then
                    local candidate = pool[j]
                    
                    if #current_target.value >= #candidate.value then
                        for start_idx = 1, #current_target.value - #candidate.value + 1 do
                            if check_subset_match(current_target.value, candidate.value, start_idx, current_target.type, candidate.type, cfg) then
                                local c1 = current_target.count or 1
                                local c2 = candidate.count or 1
                                local divisor = c1 + c2
                                
                                for k = 1, #candidate.value do
                                    local t_idx = start_idx + k - 1
                                    local sum = (current_target.value[t_idx] * c1) + (candidate.value[k] * c2)
                                    current_target.value[t_idx] = sum / divisor
                                end
                                
                                current_target.count = divisor
                                for cid, _ in pairs(candidate.consumed_ids or {}) do
                                    current_target.consumed_ids[cid] = true
                                end
                                
                                table.insert(absorbed_ids, candidate.id)
                                merged_any = true
                                skip[j] = true
                                break 
                            end
                        end
                    end
                end
            end
            
            if merged_any then
                table.insert(group_logs, {
                    ids = absorbed_ids,
                    result_str = fmt_struct(current_target)
                })
            end
            
            table.insert(reduced_pool, current_target)
        end
    end
    
    return reduced_pool, group_logs
end

local function calculate_overlap(s1, s2, min_ov, cfg)
    local len1, len2 = #s1.value, #s2.value
    local max_ov = math.min(len1, len2)
    for ov = max_ov, min_ov, -1 do
        local match = true
        for k = 0, ov-1 do
            local idx1, idx2 = len1-ov+1+k, 1+k
            local t1 = get_val_type(s1.type, idx1)
            local t2 = get_val_type(s2.type, idx2)
            if t1 ~= t2 then match=false; break end
            local diff = math.abs(s1.value[idx1] - s2.value[idx2])
            local tol = (t1=="A") and cfg.CONFIG.TOL_ANG or cfg.CONFIG.TOL_DIST
            if diff > tol then match=false; break end
        end
        if match then return ov end
    end
    return 0
end

local function merge_fragments(s1, s2, overlap)
    local new_vals = {}
    for k=1, #s1.value - overlap do table.insert(new_vals, s1.value[k]) end
    
    for k=0, overlap - 1 do 
        local v1 = s1.value[#s1.value - overlap + 1 + k]
        local v2 = s2.value[1 + k]
        table.insert(new_vals, (v1+v2)/2) 
    end
    
    for k=overlap + 1, #s2.value do table.insert(new_vals, s2.value[k]) end
    
    local combined_id = s1.id .. "+" .. s2.id
    local new_count = (s1.count or 1) + (s2.count or 1)
    local new_total_ov = (s1.total_ov or 0) + (s2.total_ov or 0) + overlap
    
    local new_consumed = {}
    for k, v in pairs(s1.consumed_ids or {}) do new_consumed[k] = true end
    for k, v in pairs(s2.consumed_ids or {}) do new_consumed[k] = true end

    return { value = new_vals, type = s1.type, id = combined_id, count = new_count, total_ov = new_total_ov, consumed_ids = new_consumed }
end

local function clone_pool(pool)
    local new_pool = {}
    for i, v in ipairs(pool) do
        local nv = {}
        for _, val in ipairs(v.value) do table.insert(nv, val) end
        local c_ids = {}
        for id_k, id_v in pairs(v.consumed_ids or {}) do c_ids[id_k] = id_v end
        table.insert(new_pool, { value = nv, type = v.type, id = v.id, count = v.count, total_ov = v.total_ov or 0, consumed_ids = c_ids })
    end
    return new_pool
end

local function solve_greedy(initial_pool, cfg)
    local pool = clone_pool(initial_pool)
    while #pool > 1 do
        local best = { i=-1, j=-1, ov=-1, struct=nil }
        for i=1, #pool do
            for j=1, #pool do
                if i ~= j then
                    local ov = calculate_overlap(pool[i], pool[j], MIN_OVERLAP_M1, cfg)
                    if ov > best.ov then best = { i=i, j=j, ov=ov, struct=merge_fragments(pool[i], pool[j], ov) } end
                end
            end
        end
        if best.ov > 0 then
            pool[best.i] = best.struct; table.remove(pool, best.j)
        else break end
    end
    return pool
end

local debug_recursions = 0
local m2_aborted = false

local function recursive_finder(current_pool, history_log, cfg, dynamic_min_overlap)
    if m2_aborted then return end 
    
    debug_recursions = debug_recursions + 1
    
    if debug_recursions > MAX_RECURSIONS_M2 then
        if LOG_RECURSION_DEBUG then
            print(string.format("[ALERTA CRÍTICA M2] Límite de %d iteraciones superado. Abortando árbol de búsqueda para evitar congelamiento...", MAX_RECURSIONS_M2))
        end
        m2_aborted = true
        return
    end

    if LOG_RECURSION_DEBUG and (debug_recursions % 500 == 0) then 
        print("[ALERTA RECURSIVIDAD] recursive_finder iteración: " .. debug_recursions .. " | Tamaño del pool: " .. #current_pool) 
    end

    local made_merge = false
    local candidates = {}
    for i=1, #current_pool do
        for j=1, #current_pool do
            if i ~= j then
                local ov = calculate_overlap(current_pool[i], current_pool[j], dynamic_min_overlap, cfg)
                if ov > 0 then table.insert(candidates, {i=i, j=j, ov=ov}) end
            end
        end
    end

    if #candidates > 0 then
        made_merge = true
        table.sort(candidates, function(a,b) return a.ov > b.ov end)
        for _, cand in ipairs(candidates) do
            if m2_aborted then break end 
            
            local next_pool = {}
            local merged = merge_fragments(current_pool[cand.i], current_pool[cand.j], cand.ov)
            table.insert(next_pool, merged)
            for k=1, #current_pool do
                if k ~= cand.i and k ~= cand.j then table.insert(next_pool, current_pool[k]) end
            end
            local next_history = {} 
            for _, h in ipairs(history_log) do table.insert(next_history, h) end
            table.insert(next_history, string.format("Merge %s+%s (Ov:%d)", current_pool[cand.i].id, current_pool[cand.j].id, cand.ov))
            
            recursive_finder(next_pool, next_history, cfg, dynamic_min_overlap)
        end
    end

    if not made_merge and not m2_aborted then 
        table.insert(raw_solutions_found, { pool_result = current_pool, history = history_log }) 
        
        if PRINT_RECURSIVE_PATHS then
            local path_str = table.concat(history_log, " -> ")
            if path_str == "" then path_str = "Directo sin merges" end
            print("[M2 RUTA ENCONTRADA] Pasos: " .. path_str)
        end
    end
end

local function select_best_solution_m2(solutions, cfg)
    if #solutions == 0 then return nil end
    local unique_groups = {}
    for _, sol in ipairs(solutions) do
        local hashes = {}
        local total_len = 0; local total_ov = 0
        for _, s in ipairs(sol.pool_result) do 
            table.insert(hashes, fmt_struct(s))
            total_len = total_len + #s.value; total_ov = total_ov + (s.total_ov or 0)
        end
        table.sort(hashes)
        local key = table.concat(hashes, " || ")
        if not unique_groups[key] then 
            unique_groups[key] = { pool_result = sol.pool_result, count = 1, history = sol.history, total_len = total_len, total_ov = total_ov }
        else unique_groups[key].count = unique_groups[key].count + 1 end
    end
    local unique_list = {}
    for key, data in pairs(unique_groups) do
        local score = (data.count * W_FREQ) + (data.total_ov * W_OV_QUAL) - (data.total_len * W_LEN)
        data.score = score; table.insert(unique_list, data)
    end
    table.sort(unique_list, function(a, b)
        if a.score == b.score then return a.count > b.count end
        return a.score > b.score
    end)
    return unique_list[1]
end

local function is_fragment_closed(struct, cfg)
    local values = struct.value
    local current_type = struct.type
    
    local temp_dist = 0; local temp_ang = 0
    local expecting_dist = (current_type == 0)
    local params = {}
    
    for i, val in ipairs(values) do
        if expecting_dist then
            temp_dist = val
            table.insert(params, { temp_dist, temp_ang })
            temp_ang = 0
            expecting_dist = false
        else
            temp_ang = val
            expecting_dist = true
        end
    end
    
    if #params < 3 then return false end
    
    local x, y = 0, 0
    local theta_deg = 0
    for i = 1, #params do
        local d = params[i][1]
        local a = params[i][2]
        theta_deg = theta_deg + a
        local theta_rad = math.rad(theta_deg)
        x = x + d * math.cos(theta_rad)
        y = y + d * math.sin(theta_rad)
    end
    
    local dist_to_origin = math.sqrt(x^2 + y^2)
    local tol = (cfg and cfg.CONFIG and cfg.CONFIG.TOL_DIST) and cfg.CONFIG.TOL_DIST or 15.0
    return dist_to_origin <= tol
end

local function force_close_fragment(struct, cfg)
    local values = struct.value
    local current_type = struct.type
    
    local temp_dist = 0; local temp_ang = 0
    local expecting_dist = (current_type == 0)
    local params = {}
    
    for i, val in ipairs(values) do
        if expecting_dist then
            temp_dist = val
            table.insert(params, { temp_dist, temp_ang })
            temp_ang = 0
            expecting_dist = false
        else
            temp_ang = val
            expecting_dist = true
        end
    end
    
    local x, y = 0, 0
    local theta_deg = 0
    for i = 1, #params do
        local d = params[i][1]
        local a = params[i][2]
        theta_deg = theta_deg + a
        local theta_rad = math.rad(theta_deg)
        x = x + d * math.cos(theta_rad)
        y = y + d * math.sin(theta_rad)
    end
    
    local dist_to_origin = math.sqrt(x^2 + y^2)
    local tol = (cfg and cfg.CONFIG and cfg.CONFIG.TOL_DIST) and cfg.CONFIG.TOL_DIST or 15.0
    if dist_to_origin <= tol then return end 
    
    local target_angle_rad = math.atan2(-y, -x)
    local target_angle_deg = math.deg(target_angle_rad)
    
    local turn_angle = target_angle_deg - theta_deg
    turn_angle = (turn_angle + 180) % 360 - 180
    
    if expecting_dist then
        values[#values] = turn_angle
        table.insert(values, dist_to_origin)
    else
        table.insert(values, turn_angle)
        table.insert(values, dist_to_origin)
    end
end

local function solve_smart_backtracking_all(initial_pool, cfg, dynamic_min_overlap)
    if LOG_RECURSION_DEBUG then
        print(string.format("[DEBUG M2] -> Iniciando Backtracking M2. Pool: %d | Solape Requerido: %d", #initial_pool, dynamic_min_overlap))
    end

    raw_solutions_found = {} 
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) return #a.value > #b.value end)
    
    debug_recursions = 0
    m2_aborted = false
    
    recursive_finder(sorted_pool, {}, cfg, dynamic_min_overlap)
    
    if m2_aborted then 
        return nil 
    end
    
    if LOG_RECURSION_DEBUG then
        print("[DEBUG M2] -> Finalizado con éxito. Llamadas recursivas totales: " .. debug_recursions)
    end
    
    return select_best_solution_m2(raw_solutions_found, cfg)
end

local function evaluar_hipotesis_m3(clean_pool, current_ngram_db, total_data_volume, iteration, cfg)
    local m3_score = -1
    local m3_hypothesis = nil
    local m3_covered_ids = {} 
    
    local best_pure_ngram = nil
    for _, ngr in ipairs(current_ngram_db) do
        if ngr.is_pure then
            if not best_pure_ngram or ngr.count > best_pure_ngram.count then best_pure_ngram = ngr end
        end
    end

    if best_pure_ngram then
        local coverage_count = 0
        local source_fragments_list = {}

        local best_d, best_a = 0, 0
        if best_pure_ngram.start_type == 0 then
            best_d = best_pure_ngram.values[1]
            best_a = best_pure_ngram.values[2]
        else
            best_a = best_pure_ngram.values[1]
            best_d = best_pure_ngram.values[2]
        end
        
        if not best_a or not best_d then return -1, nil, {} end

        for _, struct in ipairs(clean_pool) do
            local matches_perfectly = true
            local expecting_dist = (struct.type == 0)
            local vals = struct.value
            
            for k = 1, #vals do
                local tol = expecting_dist and cfg.CONFIG.TOL_DIST or cfg.CONFIG.TOL_ANG
                local ref_val = expecting_dist and best_d or best_a
                
                if math.abs(vals[k] - ref_val) > tol then
                    matches_perfectly = false
                    break
                end
                expecting_dist = not expecting_dist
            end
            
            if matches_perfectly then
                coverage_count = coverage_count + struct.count
                for cid, _ in pairs(struct.consumed_ids or {}) do
                    m3_covered_ids[cid] = true
                end
                table.insert(source_fragments_list, struct.id)
            end
        end
       
        local coverage_norm_m3 = coverage_count / total_data_volume
        local lados = 4 
        if best_a ~= 0 then lados = math.floor((360 / math.abs(best_a)) + 0.5) end
       
        local error_cierre = math.abs((lados * math.abs(best_a)) - 360)
        if error_cierre > MATH_TOLERANCE then
            return -1, nil, {}
        end

        local parametros_m3 = lados * 2
        local complexity_norm_m3 = parametros_m3 / total_data_volume
        m3_score = (coverage_norm_m3 * ARBITER_CONFIG.BONO_REGULARIDAD) - (complexity_norm_m3 * PESO_COMPLEJIDAD)
       
        local synthetic_vals = {}
        for i=1, lados do table.insert(synthetic_vals, best_d); table.insert(synthetic_vals, best_a) end
        m3_hypothesis = { type = 0, value = synthetic_vals, id = "Poligono_Iter"..iteration, count = coverage_count }
    end

    return m3_score, m3_hypothesis, m3_covered_ids
end

local function process_numeric_result(result_struct, patron_id, cfg)
    local current_pattern_params = {} 
    local values = result_struct.value
    local current_type = result_struct.type
    
    local temp_dist = 0; local temp_ang = 0
    local expecting_dist = (current_type == 0)
    
    for i, val in ipairs(values) do
        if expecting_dist then
            temp_dist = val
            table.insert(current_pattern_params, { temp_dist, temp_ang })
            temp_ang = 0
            expecting_dist = false
        else
            temp_ang = val
            expecting_dist = true
        end
    end
    
    local dropped_segment_flag = false
    local removed_count = 0
    
    if ENABLE_DYNAMIC_CLOSURE and #current_pattern_params >= 3 then
        local x, y = 0, 0
        local theta_deg = 0
        local min_dist = math.huge
        local best_closing_index = -1
        
        local tol_cierre = cfg.CONFIG.TOL_DIST or 5.0
        
        for i = 1, #current_pattern_params do
            local d = current_pattern_params[i][1]
            local a = current_pattern_params[i][2]
            theta_deg = theta_deg + a
            local theta_rad = math.rad(theta_deg)
            x = x + d * math.cos(theta_rad)
            y = y + d * math.sin(theta_rad)
            
            local dist = math.sqrt(x^2 + y^2)
            
            if i >= 3 then
                if dist < min_dist then
                    min_dist = dist
                    best_closing_index = i
                end
            end
        end
        
        if best_closing_index ~= -1 and min_dist <= tol_cierre and best_closing_index < #current_pattern_params then
            removed_count = #current_pattern_params - best_closing_index
            for _ = 1, removed_count do
                table.remove(current_pattern_params)
            end
            dropped_segment_flag = true
        end
    end

    if LOG_TRADUCCION then
        log(string.format("\n[TRADUCCIÓN FÍSICA - COMPORTAMIENTO %d]", patron_id))
        
        if dropped_segment_flag then
            local kept_vals = {}
            local elements_to_keep = #current_pattern_params * 2
            elements_to_keep = math.min(elements_to_keep, #values)
            
            for k = 1, elements_to_keep do
                table.insert(kept_vals, string.format("%.2f", values[k]))
            end
            
            local t_str = (current_type == 1) and "Ang" or "Dist"
            local count_str = (result_struct.count and result_struct.count > 1) and string.format(" [x%d]", result_struct.count) or ""
            local new_chain_str = string.format("(%s)%s {%s}", t_str, count_str, table.concat(kept_vals, ", "))
            
            log(string.format("   >> LÓGICA DE CIERRE INTELIGENTE: Se eliminaron los últimos %d segmento(s) porque el segmento %d dejaba al robot más cerca del origen.", removed_count, #current_pattern_params))
            log("   >> CADENA ACTUALIZADA: " .. new_chain_str)
        end
        
        for idx, seg in ipairs(current_pattern_params) do 
            log(string.format("   Seg %d: (Ang %.2f, Dist %.2f)", idx, seg[2], seg[1])) 
        end
    end

    return current_pattern_params
end

function GlobalReconstructor.run(robot_id, stored_data, cfg, offset_comportamientos)    
    if LOG_TABLA_FRAGMENTOS then
        log("\n=========================================")
        log(" TABLA DE FRAGMENTOS RECOLECTADOS")
        log("=========================================")
        for i, v in ipairs(stored_data) do 
            log(string.format(" [%d] %s", i, fmt_struct(v))) 
        end
    end
       
    local raw_pool = {}
    for i, v in ipairs(stored_data) do 
        local preserved_consumed_ids = {}
        if v.consumed_ids then
            for cid, _ in pairs(v.consumed_ids) do 
                preserved_consumed_ids[cid] = true 
            end
        else
            preserved_consumed_ids[tostring(i)] = true
        end

        table.insert(raw_pool, { 
            value = v.value, 
            type = v.type, 
            count = v.count, 
            id = v.id or tostring(i),
            consumed_ids = preserved_consumed_ids 
        }) 
    end
    
    if #raw_pool == 0 then return false, "", {} end

    local group_logs = {}
    local reduced_pool = {}
    reduced_pool, group_logs = reduce_fragments_pool(raw_pool, cfg)
    
    if LOG_TERMOSTATO then
        log("\n=========================================")
        log(" [REDUCCIÓN DE FRAGMENTOS (PRE-PROCESAMIENTO)]")
        if #group_logs > 0 then
            for num, g_log in ipairs(group_logs) do
                local ids_str = table.concat(g_log.ids, ", ")
                log(string.format(" promedio %d = fragmentos: %s = %s", num, ids_str, g_log.result_str))
            end
        else
            log(" >> No se encontraron sub-cadenas para reducir. Todos los fragmentos son únicos.")
        end
    end

    build_global_ngram_db(reduced_pool, cfg)
    
    local freq_array = {}
    for _, ngr in ipairs(ngram_db) do table.insert(freq_array, ngr.count) end
    
    calibrar_hiperparametros(freq_array, ngram_db, cfg)
    
    local limite_ruido = calcular_limite_ruido(freq_array)
    
    local clean_pool = {}
    local discarded_fragments = {} 
    
    for _, struct in ipairs(reduced_pool) do
        local fragment_ngrams, _ = extract_ngrams_from_fragment(struct, cfg)
        local is_noisy = false
        local failed_ngram_count = 0
        local failed_ngram_str = ""
        local ngrams_info = {} 
       
        for _, ngr in ipairs(fragment_ngrams) do
            local current_count = 0
            for _, db_ngr in ipairs(ngram_db) do
                if ngrams_match(db_ngr, ngr, cfg) then
                    current_count = db_ngr.count
                    break
                end
            end
           
            table.insert(ngrams_info, {str = format_ngram(ngr), count = current_count})

            if current_count < limite_ruido and not is_noisy then 
                is_noisy = true 
                failed_ngram_count = current_count
                failed_ngram_str = format_ngram(ngr)
            end
        end
       
        if not is_noisy then 
            table.insert(clean_pool, struct) 
        else
            table.insert(discarded_fragments, {
                id = struct.id,
                str = fmt_struct(struct),
                failed_count = failed_ngram_count,
                failed_str = failed_ngram_str,
                min = limite_ruido,
                ngrams = ngrams_info
            })
        end
    end

    if LOG_TERMOSTATO then
        log("\n=========================================")
        log(" [FILTRO RUIDO]")
       
        if #discarded_fragments > 0 then
            for _, info in ipairs(discarded_fragments) do
                log("")
                log(string.format(" Fragmento %s -> %s", info.id, info.str))
                log("   Frecuencia de sus secuencias (N-Gramas):")
                for _, ngr_info in ipairs(info.ngrams) do
                    log(string.format("   %s -> Frecuencia: %d", ngr_info.str, ngr_info.count))
                end
                log(string.format("   >> DESCARTADO: Debido a que la secuencia %s tenía %d repeticiones (frecuencia mínima debió ser %.2f).", 
                    info.failed_str, info.failed_count, info.min))
            end
        else
            log("\n >> Ningún fragmento fue eliminado. Todos superaron el umbral de ruido mínimo (" .. string.format("%.2f", limite_ruido) .. ").")
        end
        log("=========================================")
        
        log("\n=========================================")
        log(" [POOL FINAL POST-REDUCCIÓN Y FILTRO]")
        if #clean_pool > 0 then
            for i, v in ipairs(clean_pool) do 
                log(string.format(" [%d] %s", i, fmt_struct(v))) 
            end
        else
            log(" >> El pool quedó vacío.")
        end
        log("=========================================\n")
    end

    local all_final_results = {} 
    local iteration = 1

    while #clean_pool > 0 do
        local total_data_volume = 0
        for _, v in ipairs(clean_pool) do total_data_volume = total_data_volume + v.count end
        if total_data_volume == 0 then break end

        local winner_struct = nil
        local ids_to_remove = {}
        local is_m3_winner = false
        local is_single_closed = false

        if #clean_pool == 1 then
            is_single_closed = is_fragment_closed(clean_pool[1], cfg)
            if is_single_closed then
                winner_struct = clean_pool[1]
                ids_to_remove = clean_pool[1].consumed_ids
                if LOG_ARBITRO >= 1 then
                    log(string.format("\n [ÁRBITRO] >>> ¡Gana Único Fragmento (Cierre Natural)! -> %s", fmt_struct(winner_struct)))
                end
            end
        end

        if not is_single_closed then
            build_global_ngram_db(clean_pool, cfg)
            local m3_score, m3_hypothesis, m3_covered_ids = evaluar_hipotesis_m3(clean_pool, ngram_db, total_data_volume, iteration, cfg)

            local explorer_score = -1
            local explorer_hypothesis = nil
            local explorer_name = "M1"
            local explorer_pool_result = {}
            local m1_fallback_reason = ""
           
            local idx_orden = ARBITER_CONFIG.INDICE_ORDEN or 1.0
            local dynamic_min_overlap = MIN_OVERLAP_M2
            local dynamic_max_fragments = MAX_FRAGMENTS_M2
           
            if idx_orden <= 0.4 then
                dynamic_min_overlap = math.max(dynamic_min_overlap, 3)
                dynamic_max_fragments = math.min(dynamic_max_fragments, 5)
            elseif idx_orden <= 0.8 then
                dynamic_min_overlap = math.max(dynamic_min_overlap, 2)
                dynamic_max_fragments = math.min(dynamic_max_fragments, 7)
            end
           
            if LOG_RECURSION_DEBUG then
                print(string.format("\n[DEBUG ADAPTATIVO] Indice de Orden: %.2f | M2 ajustado a Solape >= %d, Límite Fragmentos <= %d", idx_orden, dynamic_min_overlap, dynamic_max_fragments))
            end

            if #clean_pool <= dynamic_max_fragments then
                local m2_solution = solve_smart_backtracking_all(clean_pool, cfg, dynamic_min_overlap)
               
                if m2_solution and #m2_solution.pool_result > 0 then
                    explorer_pool_result = m2_solution.pool_result
                    explorer_name = "M2"
                else
                    m1_fallback_reason = string.format(" [Forzado a M1: M2 abortó al superar %d llamadas recursivas]", MAX_RECURSIONS_M2)
                    explorer_pool_result = solve_greedy(clean_pool, cfg)
                end
            else
                m1_fallback_reason = string.format(" [Forzado a M1: %d fragmentos superan el límite dinámico de %d fragmentos]", #clean_pool, dynamic_max_fragments)
                explorer_pool_result = solve_greedy(clean_pool, cfg)
            end

            if #explorer_pool_result > 0 then
                table.sort(explorer_pool_result, function(a,b) return (a.count or 1) > (b.count or 1) end)
                explorer_hypothesis = explorer_pool_result[1]
                local coverage_norm_exp = (explorer_hypothesis.count or 1) / total_data_volume
                local complexity_norm_exp = #explorer_hypothesis.value / total_data_volume
                explorer_score = (coverage_norm_exp * 1.0) - (complexity_norm_exp * PESO_COMPLEJIDAD)
            end

            if m3_score > explorer_score and m3_score >= ARBITER_CONFIG.MIN_SCORE then

                m3_hypothesis.consumed_ids = m3_covered_ids
                winner_struct = m3_hypothesis
                ids_to_remove = m3_covered_ids
                is_m3_winner = true
               
                if LOG_ARBITRO == 2 then
                    log(string.format("\n [ÁRBITRO DETALLE] Evaluación de la Iteración %d", iteration))
                    log(string.format("Puntaje M3 (Regular): %.4f", m3_score))
                    log(string.format("Puntaje %s (Irregular): %.4f", explorer_name, explorer_score))
                    log(string.format("Mínimo Requerido: %.4f", ARBITER_CONFIG.MIN_SCORE))
                    log(string.format("Camino propuesto M3: %s", fmt_struct(m3_hypothesis)))
                    if explorer_hypothesis then
                        log(string.format("Camino propuesto %s: %s", explorer_name, fmt_struct(explorer_hypothesis)))
                    end
                end
               
                if LOG_ARBITRO >= 1 then
                    log(string.format("\n [ÁRBITRO] >>> ¡Gana M3 (Geometría Regular)! -> %s", fmt_struct(winner_struct)))
                end
               
            elseif explorer_score >= m3_score and explorer_score >= ARBITER_CONFIG.MIN_SCORE then
                winner_struct = explorer_hypothesis
                ids_to_remove = explorer_hypothesis.consumed_ids
               
                if LOG_ARBITRO == 2 then
                    log(string.format("\n [ÁRBITRO DETALLE] Evaluación de la Iteración %d", iteration))
                    log(string.format("Puntaje M3 (Regular): %.4f", m3_score))
                    log(string.format("Puntaje %s (Irregular): %.4f", explorer_name, explorer_score))
                    log(string.format("Mínimo Requerido: %.4f", ARBITER_CONFIG.MIN_SCORE))
                    if m3_hypothesis then
                        log(string.format("Camino propuesto M3: %s", fmt_struct(m3_hypothesis)))
                    end
                    log(string.format("Camino propuesto %s: %s", explorer_name, fmt_struct(explorer_hypothesis)))
                end
               
                if LOG_ARBITRO >= 1 then
                    local reason_str = (explorer_name == "M1") and m1_fallback_reason or ""
                    log(string.format("\n [ÁRBITRO] >>> ¡Gana %s (Ruta Irregular)%s! -> %s", explorer_name, reason_str, fmt_struct(winner_struct)))
                end
            else
                if LOG_ARBITRO == 2 then
                    log(string.format("\n [ÁRBITRO DETALLE] Fin - Ninguno superó el umbral mínimo (%.4f)", ARBITER_CONFIG.MIN_SCORE))
                    log(string.format("Puntaje M3: %.4f | Puntaje %s: %.4f", m3_score, explorer_name, explorer_score))
                end
                break
            end
        end

        if winner_struct and FORCE_CLOSE_IRREGULAR and not is_m3_winner and not is_single_closed then
            force_close_fragment(winner_struct, cfg)
            if LOG_ARBITRO >= 1 then
                log(string.format(" [CIERRE FORZADO] Ruta irregular actualizada (Regresa al origen) -> %s", fmt_struct(winner_struct)))
            end
        end

        table.insert(all_final_results, winner_struct)
        local next_pool = {}
        for _, struct in ipairs(clean_pool) do
            if not ids_to_remove[struct.id] then table.insert(next_pool, struct) end
        end
        clean_pool = next_pool
        iteration = iteration + 1
    end
    
    local linea_resumen = robot_id .. " & Imitado&"
    local nuevos_patrones = {}
    local winner_ids_map = {} 

    local offset_c = offset_comportamientos or 0

    if LOG_MAPEO_IDS then log("\n [MAPEO DE IDs AL FINALIZAR ÁRBITRO]") end

    for i, res in ipairs(all_final_results) do 
        if not res.consumed_ids then res.consumed_ids = { [res.id] = true } end

        local res_ids_str = ""
        for cid, _ in pairs(res.consumed_ids) do
            winner_ids_map[cid] = true
            res_ids_str = res_ids_str .. cid .. ", "
        end

        -- ¡CORRECCIÓN APLICADA AQUÍ! Ahora el mapeo muestra el mismo índice que la traducción
        if LOG_MAPEO_IDS then
            log(string.format("   >> Comportamiento %d absorbió los fragmentos locales con ID: {%s}", offset_c + i, res_ids_str))
        end

        local pattern_params = process_numeric_result(res, offset_c + i, cfg) 
        linea_resumen = linea_resumen .. "p" .. (offset_c + i)
        for idx, seg in ipairs(pattern_params) do
            if idx > 1 then linea_resumen = linea_resumen .. "," end
            linea_resumen = linea_resumen .. string.format("(%.2f,%.2f)", seg[2], seg[1])
        end
        table.insert(nuevos_patrones, pattern_params)
    end

    return true, linea_resumen, nuevos_patrones, winner_ids_map
end

return GlobalReconstructor