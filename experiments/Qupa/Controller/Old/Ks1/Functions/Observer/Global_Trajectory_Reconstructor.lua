-- Archivo: Functions/Observer/Global_Trajectory_Reconstructor.lua

--NOTAS: 
    -- Fragmento: Es una secuencia completa de datos detectada. Ej: {20,11, 120,57, 18,98, 117,85, 18,72}
    -- N-Grama: Es una transición única de (Distancia->Ángulo) o (Ángulo->Distancia) extraída de los fragmentos. 
        --Nos dice cuántas combinaciones hay en el enjambre.
    -- Periodicos: Fragmentos que tienen una simetría perfecta (repiten un patrón idéntico constantemente, como los lados de un triangulo). ej: (Ang: 119,21, Dist: 19,55) 

local GlobalReconstructor = {}

-- ============================================================================
-- VARIABLES DE CONTROL Y LOGGEO
-- ============================================================================
local LOG_TABLA_FRAGMENTOS = true  -- 1. true/false: Imprimir o no la tabla de fragmentos recolectados
local LOG_TERMOSTATO       = true  -- 2. true/false: Imprimir o no la auto-calibración y el filtro de ruido
local LOG_ARBITRO          = 1     -- 3. 0: Nada, 1: Resumen ganador, 2: Detalle de puntajes y caminos propuestos
local LOG_TRADUCCION       = true  -- 4. true/false: Imprimir o no la traducción física de los patrones finales

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

local function extract_ngrams_from_fragment(struct, cfg)
    local ngrams = {}
    local is_dist_first = (struct.type == 0)
    
    if #struct.value < 2 then return {}, false end
    
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
    
    for i = 1, #struct.value - 1 do
        local v1 = struct.value[i]
        local v2 = struct.value[i+1]
        local is_v1_dist = is_dist_first and (i%2 ~= 0) or (not is_dist_first and i%2 == 0)
        local d, a, p_type
        
        if is_v1_dist then d = v1; a = v2; p_type = 0 
        else a = v1; d = v2; p_type = 1 end
        
        table.insert(ngrams, {d = d, a = a, type = p_type, is_pure = is_pure})
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
                local values_match = (math.abs(db_ngr.d - ngr.d) <= cfg.CONFIG.TOL_DIST and math.abs(db_ngr.a - ngr.a) <= cfg.CONFIG.TOL_ANG)
                if values_match and db_ngr.type == ngr.type then
                    db_ngr.d = db_ngr.d + (ngr.d - db_ngr.d) / (db_ngr.merges + 1)
                    db_ngr.a = db_ngr.a + (ngr.a - db_ngr.a) / (db_ngr.merges + 1)
                    db_ngr.count = db_ngr.count + (struct.count or 1)
                    db_ngr.merges = db_ngr.merges + 1
                    db_ngr.is_pure = db_ngr.is_pure or is_pure 
                    found = true
                    break
                end
            end
            if not found then 
                table.insert(ngram_db, {d = ngr.d, a = ngr.a, type = ngr.type, count = (struct.count or 1), merges = 1, is_pure = is_pure}) 
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
    
    local ratio_pureza = 0
    if suma_frecuencias > 0 then
        ratio_pureza = pureza_total / suma_frecuencias
    end

    local nivel_caos = 1.0 - indice_orden
    local ruido_extra = nivel_caos * cfg.CONFIG.RUIDO_PENALIZACION_MAX
    ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO = cfg.CONFIG.RUIDO_BASE + ruido_extra

    local bono_extra = ratio_pureza * cfg.CONFIG.BONO_M3_MAX_EXTRA * indice_orden
    ARBITER_CONFIG.BONO_REGULARIDAD = cfg.CONFIG.BONO_M3_BASE + bono_extra

    local penalizacion_caos = nivel_caos * cfg.CONFIG.SCORE_PENALIZACION_MAX
    ARBITER_CONFIG.MIN_SCORE = cfg.CONFIG.SCORE_MIN_BASE + penalizacion_caos

    local repeticiones_minimas = max_frecuencia * ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO

    if LOG_TERMOSTATO then
        log("\n=========================================")
        log(" [TERMOSTATO] AUTO-CALIBRACIÓN DEL ENJAMBRE")
        log("=========================================")
        log("  [1] Extracción de Métricas Base:")
        log(string.format("      - Frecuencia Máxima Dominante: %d", max_frecuencia))
        for _, ngr in ipairs(ngrams_dominantes) do
            log(string.format("        -> Corresponde al N-Grama: (Ang: %.2f, Dist: %.2f)", ngr.a, ngr.d))
        end
        log(string.format("      - Frecuencia Promedio Global: %.2f", prom_frecuencia))
        log(string.format("      - Volumen Total (Cantidad de N-Gramas): %d", suma_frecuencias))
        log(string.format("      - N-Gramas Únicos: %d", #freq_array))
        log(string.format("      - N-Gramas con Simetría (Periodicos): %d", pureza_total))
        
        log("\n  [2] Diagnóstico del Entorno (Fórmulas):")
        log("      * Índice de Orden: Mide qué tan repetitivos son los datos (1.0 = patrón dominante claro, 0 = caos).")
        log(string.format("        -> 1.0 - (Únicos [%d] / Volumen [%d]) = %.2f", #freq_array, suma_frecuencias, indice_orden))
        log("      * Ratio de Pureza: Mide la proporción de geometrías perfectas (como polígonos cerrados).")
        log(string.format("        -> Periodicos [%d] / Volumen [%d] = %.2f", pureza_total, suma_frecuencias, ratio_pureza))

        log("\n  [3] Fórmulas y Resultados Aplicados:")
        log("      * Umbral de Ruido: Porcentaje mínimo de repetición para que un dato NO sea descartado.")
        log(string.format("        > FÓRMULA UMBRAL : Base(%.2f) + [Caos(%.2f) * Max_Pen(%.2f)] = %.1f%%.", 
            cfg.CONFIG.RUIDO_BASE, nivel_caos, cfg.CONFIG.RUIDO_PENALIZACION_MAX, ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO * 100))
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
                    local ov = calculate_overlap(pool[i], pool[j], cfg.CONFIG.MIN_OVERLAP_M1, cfg)
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

local function recursive_finder(current_pool, history_log, cfg)
    local made_merge = false
    local candidates = {}
    for i=1, #current_pool do
        for j=1, #current_pool do
            if i ~= j then
                local ov = calculate_overlap(current_pool[i], current_pool[j], cfg.CONFIG.MIN_OVERLAP_M2, cfg)
                if ov > 0 then table.insert(candidates, {i=i, j=j, ov=ov}) end
            end
        end
    end

    if #candidates > 0 then
        made_merge = true
        table.sort(candidates, function(a,b) return a.ov > b.ov end)
        for _, cand in ipairs(candidates) do
            local next_pool = {}
            local merged = merge_fragments(current_pool[cand.i], current_pool[cand.j], cand.ov)
            table.insert(next_pool, merged)
            for k=1, #current_pool do
                if k ~= cand.i and k ~= cand.j then table.insert(next_pool, current_pool[k]) end
            end
            local next_history = {} 
            for _, h in ipairs(history_log) do table.insert(next_history, h) end
            table.insert(next_history, string.format("   Merge %s+%s (Ov:%d)", current_pool[cand.i].id, current_pool[cand.j].id, cand.ov))
            recursive_finder(next_pool, next_history, cfg)
        end
    end

    if not made_merge then 
        table.insert(raw_solutions_found, { pool_result = current_pool, history = history_log }) 
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
        local score = (data.count * cfg.CONFIG.W_FREQ) + (data.total_ov * cfg.CONFIG.W_OV_QUAL)
        data.score = score; table.insert(unique_list, data)
    end
    table.sort(unique_list, function(a, b)
        if a.score == b.score then return a.count > b.count end
        return a.score > b.score
    end)
    return unique_list[1]
end

local function solve_smart_backtracking_all(initial_pool, cfg)
    raw_solutions_found = {} 
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) return #a.value > #b.value end)
    recursive_finder(sorted_pool, {}, cfg)
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

        for _, struct in ipairs(clean_pool) do
            local fragment_ngrams, _ = extract_ngrams_from_fragment(struct, cfg)
            local matches_perfectly = true
            for _, ngr in ipairs(fragment_ngrams) do
                if math.abs(ngr.d - best_pure_ngram.d) > cfg.CONFIG.TOL_DIST or math.abs(ngr.a - best_pure_ngram.a) > cfg.CONFIG.TOL_ANG then
                    matches_perfectly = false; break
                end
            end
            if matches_perfectly then
                coverage_count = coverage_count + struct.count
                m3_covered_ids[struct.id] = true
                table.insert(source_fragments_list, struct.id)
            end
        end
        
        local coverage_norm_m3 = coverage_count / total_data_volume
        local lados = 4 
        if best_pure_ngram.a ~= 0 then lados = math.floor((360 / math.abs(best_pure_ngram.a)) + 0.5) end
        
        local parametros_m3 = lados * 2
        local complexity_norm_m3 = parametros_m3 / total_data_volume
        m3_score = (coverage_norm_m3 * ARBITER_CONFIG.BONO_REGULARIDAD) - (complexity_norm_m3 * cfg.CONFIG.PESO_COMPLEJIDAD)
        
        local synthetic_vals = {}
        for i=1, lados do table.insert(synthetic_vals, best_pure_ngram.d); table.insert(synthetic_vals, best_pure_ngram.a) end
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
    
    if #current_pattern_params >= 3 then
        local first_seg = current_pattern_params[1]
        local last_seg = current_pattern_params[#current_pattern_params]

        local tol_dist = cfg.CONFIG.TOL_DIST or 5.0
        if math.abs(first_seg[1] - last_seg[1]) <= tol_dist then
            local x, y = 0, 0
            local theta_deg = 0
            
            for i = 1, #current_pattern_params - 1 do
                local d = current_pattern_params[i][1]
                local a = current_pattern_params[i][2]
                theta_deg = theta_deg + a
                local theta_rad = math.rad(theta_deg)
                x = x + d * math.cos(theta_rad)
                y = y + d * math.sin(theta_rad)
            end
            
            local dist_al_origen = math.sqrt(x^2 + y^2)
            local tol_cierre = tol_dist
            
            if dist_al_origen <= tol_cierre then
                table.remove(current_pattern_params, #current_pattern_params)
            end
        end
    end

    if LOG_TRADUCCION then
        log(string.format("\n[TRADUCCIÓN FÍSICA - PATRÓN %d]", patron_id))
        for idx, seg in ipairs(current_pattern_params) do 
            log(string.format("   Seg %d: (Ang %.2f, Dist %.2f)", idx, seg[2], seg[1])) 
        end
    end

    return current_pattern_params
end

-- =========================================================================
--  EJECUCIÓN PRINCIPAL
-- =========================================================================

function GlobalReconstructor.run(robot_id, stored_data, cfg)
    
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
        table.insert(raw_pool, { 
            value = v.value, 
            type = v.type, 
            count = v.count, 
            id = tostring(i),
            consumed_ids = { [tostring(i)] = true } 
        }) 
    end
    
    if #raw_pool == 0 then return false, "", {} end
    
    build_global_ngram_db(raw_pool, cfg)
    
    local freq_array = {}
    for _, ngr in ipairs(ngram_db) do table.insert(freq_array, ngr.count) end
    
    calibrar_hiperparametros(freq_array, ngram_db, cfg)
    
    local limite_ruido = calcular_limite_ruido(freq_array)
    
    local clean_pool = {}
    local discarded_fragments = {} -- Lista para agrupar los eliminados
    
    for _, struct in ipairs(raw_pool) do
        local fragment_ngrams, _ = extract_ngrams_from_fragment(struct, cfg)
        local is_noisy = false
        local failed_ngram_count = 0
        local failed_ngram_d = 0
        local failed_ngram_a = 0
        local ngrams_info = {} -- Para guardar la información de todos los pares
        
        for _, ngr in ipairs(fragment_ngrams) do
            local current_count = 0
            for _, db_ngr in ipairs(ngram_db) do
                if (math.abs(db_ngr.d - ngr.d) <= cfg.CONFIG.TOL_DIST and math.abs(db_ngr.a - ngr.a) <= cfg.CONFIG.TOL_ANG) then
                    current_count = db_ngr.count
                    break
                end
            end
            
            -- Guardamos la frecuencia de cada par encontrado
            table.insert(ngrams_info, {d = ngr.d, a = ngr.a, count = current_count})

            if current_count < limite_ruido and not is_noisy then 
                is_noisy = true 
                failed_ngram_count = current_count
                failed_ngram_d = ngr.d
                failed_ngram_a = ngr.a
                -- No hacemos 'break' aquí para poder registrar las frecuencias de los demás pares del fragmento
            end
        end
        
        if not is_noisy then 
            table.insert(clean_pool, struct) 
        else
            -- Guardamos el fragmento eliminado para imprimirlo luego con el formato exacto y detalles de sus pares
            table.insert(discarded_fragments, {
                id = struct.id,
                str = fmt_struct(struct),
                failed_count = failed_ngram_count,
                failed_d = failed_ngram_d,
                failed_a = failed_ngram_a,
                min = limite_ruido,
                ngrams = ngrams_info
            })
        end
    end

    -- Impresión del bloque de FILTRO RUIDO
    if LOG_TERMOSTATO then
        log("\n=========================================")
        log(" [FILTRO RUIDO]")
        
        if #discarded_fragments > 0 then
            for _, info in ipairs(discarded_fragments) do
                log("")
                log(string.format(" Fragmento %s -> %s", info.id, info.str))
                log("   Frecuencia de sus pares (N-Gramas):")
                for _, ngr_info in ipairs(info.ngrams) do
                    log(string.format("     - (Ang: %.2f, Dist: %.2f) -> Frecuencia: %d", ngr_info.a, ngr_info.d, ngr_info.count))
                end
                log(string.format("   >> DESCARTADO: Debido a que el par (Ang: %.2f, Dist: %.2f) tenía %d repeticiones (frecuencia mínima debió ser %.2f).", 
                    info.failed_a, info.failed_d, info.failed_count, info.min))
            end
        else
            log("\n >> Ningún fragmento fue eliminado. Todos los pares superaron el umbral de ruido mínimo (" .. string.format("%.2f", limite_ruido) .. ").")
        end
        log("=========================================")
    end

    local all_final_results = {} 
    local iteration = 1

    while #clean_pool > 0 do
        local total_data_volume = 0
        for _, v in ipairs(clean_pool) do total_data_volume = total_data_volume + v.count end
        if total_data_volume == 0 then break end

        build_global_ngram_db(clean_pool, cfg)
        local m3_score, m3_hypothesis, m3_covered_ids = evaluar_hipotesis_m3(clean_pool, ngram_db, total_data_volume, iteration, cfg)

        local explorer_score = -1
        local explorer_hypothesis = nil
        local explorer_name = "M1"
        local explorer_pool_result = {}
        
        if #clean_pool <= cfg.CONFIG.MAX_ISLANDS_M2 then
            local m2_solution = solve_smart_backtracking_all(clean_pool, cfg)
            if m2_solution and #m2_solution.pool_result > 0 then
                explorer_pool_result = m2_solution.pool_result
                explorer_name = "M2"
            else
                explorer_pool_result = solve_greedy(clean_pool, cfg)
            end
        else
            explorer_pool_result = solve_greedy(clean_pool, cfg)
        end

        if #explorer_pool_result > 0 then
            table.sort(explorer_pool_result, function(a,b) return (a.count or 1) > (b.count or 1) end)
            explorer_hypothesis = explorer_pool_result[1]
            local coverage_norm_exp = (explorer_hypothesis.count or 1) / total_data_volume
            local complexity_norm_exp = #explorer_hypothesis.value / total_data_volume
            explorer_score = (coverage_norm_exp * 1.0) - (complexity_norm_exp * cfg.CONFIG.PESO_COMPLEJIDAD)
        end

        local winner_struct = nil
        local ids_to_remove = {}

        if m3_score > explorer_score and m3_score >= ARBITER_CONFIG.MIN_SCORE then
            winner_struct = m3_hypothesis
            ids_to_remove = m3_covered_ids
            
            if LOG_ARBITRO == 2 then
                log(string.format("\n [ÁRBITRO DETALLE] Evaluación de la Iteración %d", iteration))
                log(string.format("   - Puntaje M3 (Regular): %.4f", m3_score))
                log(string.format("   - Puntaje %s (Irregular): %.4f", explorer_name, explorer_score))
                log(string.format("   - Mínimo Requerido: %.4f", ARBITER_CONFIG.MIN_SCORE))
                log(string.format("   - Camino propuesto M3: %s", fmt_struct(m3_hypothesis)))
                if explorer_hypothesis then
                    log(string.format("   - Camino propuesto %s: %s", explorer_name, fmt_struct(explorer_hypothesis)))
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
                log(string.format("   - Puntaje M3 (Regular): %.4f", m3_score))
                log(string.format("   - Puntaje %s (Irregular): %.4f", explorer_name, explorer_score))
                log(string.format("   - Mínimo Requerido: %.4f", ARBITER_CONFIG.MIN_SCORE))
                if m3_hypothesis then
                    log(string.format("   - Camino propuesto M3: %s", fmt_struct(m3_hypothesis)))
                end
                log(string.format("   - Camino propuesto %s: %s", explorer_name, fmt_struct(explorer_hypothesis)))
            end
            
            if LOG_ARBITRO >= 1 then
                log(string.format("\n [ÁRBITRO] >>> ¡Gana %s (Ruta Irregular)! -> %s", explorer_name, fmt_struct(winner_struct)))
            end
        else
            if LOG_ARBITRO == 2 then
                log(string.format("\n [ÁRBITRO DETALLE] Fin - Ninguno superó el umbral mínimo (%.4f)", ARBITER_CONFIG.MIN_SCORE))
                log(string.format("   - Puntaje M3: %.4f | Puntaje %s: %.4f", m3_score, explorer_name, explorer_score))
            end
            break
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
    
    for i, res in ipairs(all_final_results) do 
        local pattern_params = process_numeric_result(res, i, cfg) 
        linea_resumen = linea_resumen .. "p" .. i
        for idx, seg in ipairs(pattern_params) do
            if idx > 1 then linea_resumen = linea_resumen .. "," end
            linea_resumen = linea_resumen .. string.format("(%.2f,%.2f)", seg[1], seg[2])
        end
        table.insert(nuevos_patrones, pattern_params)
    end
    
    return true, linea_resumen, nuevos_patrones
end

return GlobalReconstructor