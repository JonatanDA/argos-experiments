-- Archivo: Functions/Observer/Global_Trajectory_Reconstructor.lua

local GlobalReconstructor = {}

local ARBITER_CONFIG = {}
local pairs_db = {}
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

local function extract_pairs_from_island(struct, cfg)
    local island_pairs = {}
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
        
        table.insert(island_pairs, {d = d, a = a, type = p_type, is_pure = is_pure})
    end
    
    return island_pairs, is_pure
end

local function build_global_pairs_db(pool, cfg)
    pairs_db = {}
    for _, struct in ipairs(pool) do
        local island_pairs, is_pure = extract_pairs_from_island(struct, cfg)
        for _, p in ipairs(island_pairs) do
            local found = false
            for _, db_p in ipairs(pairs_db) do
                local values_match = (math.abs(db_p.d - p.d) <= cfg.CONFIG.TOL_DIST and math.abs(db_p.a - p.a) <= cfg.CONFIG.TOL_ANG)
                if values_match and db_p.type == p.type then
                    db_p.d = db_p.d + (p.d - db_p.d) / (db_p.merges + 1)
                    db_p.a = db_p.a + (p.a - db_p.a) / (db_p.merges + 1)
                    db_p.count = db_p.count + (struct.count or 1)
                    db_p.merges = db_p.merges + 1
                    db_p.is_pure = db_p.is_pure or is_pure 
                    found = true
                    break
                end
            end
            if not found then 
                table.insert(pairs_db, {d = p.d, a = p.a, type = p.type, count = (struct.count or 1), merges = 1, is_pure = is_pure}) 
            end
        end
    end
end

local function calibrar_hiperparametros(freq_array, current_pairs_db, cfg)
    if #freq_array == 0 then return end

    local max_frecuencia = 0
    local suma_frecuencias = 0
    local pureza_total = 0

    for _, f in ipairs(freq_array) do
        if f > max_frecuencia then max_frecuencia = f end
        suma_frecuencias = suma_frecuencias + f
    end
    
    for _, p in ipairs(current_pairs_db) do
        if p.is_pure then pureza_total = pureza_total + p.count end
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

    log("\n=========================================")
    log("[TERMOSTATO] AUTO-CALIBRACIÓN DEL ENJAMBRE")
    log("=========================================")
    log("  [1] Extracción de Métricas Base:")
    log(string.format("      - Frecuencia Máxima Dominante: %d", max_frecuencia))
    log(string.format("      - Frecuencia Promedio Global: %.2f", prom_frecuencia))
    log(string.format("      - Total Fragmentos (Volumen): %d", suma_frecuencias))
    log(string.format("      - N-Gramas (Pares) Únicos: %d", #freq_array))
    log(string.format("      - Fragmentos con Simetría (Puros): %d", pureza_total))
    
    log("\n  [2] Diagnóstico del Entorno (Fórmulas):")
    log(string.format("      - Índice de Orden: 1.0 - (Únicos [%d] / Volumen [%d]) = %.2f", #freq_array, suma_frecuencias, indice_orden))
    log(string.format("      - Ratio de Pureza: Puros [%d] / Volumen [%d] = %.2f", pureza_total, suma_frecuencias, ratio_pureza))

    local nivel_caos = 1.0 - indice_orden
    local ruido_extra = nivel_caos * cfg.CONFIG.RUIDO_PENALIZACION_MAX
    ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO = cfg.CONFIG.RUIDO_BASE + ruido_extra

    local bono_extra = ratio_pureza * cfg.CONFIG.BONO_M3_MAX_EXTRA * indice_orden
    ARBITER_CONFIG.BONO_REGULARIDAD = cfg.CONFIG.BONO_M3_BASE + bono_extra

    local penalizacion_caos = nivel_caos * cfg.CONFIG.SCORE_PENALIZACION_MAX
    ARBITER_CONFIG.MIN_SCORE = cfg.CONFIG.SCORE_MIN_BASE + penalizacion_caos

    log("\n  [3] Fórmulas y Resultados Aplicados:")
    log(string.format("      > UMBRAL RUIDO: Base(%.2f) + [Caos(%.2f) * Max_Pen(%.2f)] = %.1f%%.", 
        cfg.CONFIG.RUIDO_BASE, nivel_caos, cfg.CONFIG.RUIDO_PENALIZACION_MAX, ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO * 100))
    log("=========================================")
end

local function calcular_limite_ruido(frecuencias)
    if #frecuencias == 0 then return 0 end
    local max_freq = 0
    for _, f in ipairs(frecuencias) do
        if f > max_freq then max_freq = f end
    end
    local limite = max_freq * ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO
    return limite
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

local function evaluar_hipotesis_m3(clean_pool, current_pairs_db, total_data_volume, iteration, cfg)
    local m3_score = -1
    local m3_hypothesis = nil
    local m3_covered_ids = {} 
    
    local best_pure_pair = nil
    for _, p in ipairs(current_pairs_db) do
        if p.is_pure then
            if not best_pure_pair or p.count > best_pure_pair.count then best_pure_pair = p end
        end
    end

    if best_pure_pair then
        local coverage_count = 0
        local source_islands_list = {}

        for _, struct in ipairs(clean_pool) do
            local island_pairs, _ = extract_pairs_from_island(struct, cfg)
            local matches_perfectly = true
            for _, p in ipairs(island_pairs) do
                if math.abs(p.d - best_pure_pair.d) > cfg.CONFIG.TOL_DIST or math.abs(p.a - best_pure_pair.a) > cfg.CONFIG.TOL_ANG then
                    matches_perfectly = false; break
                end
            end
            if matches_perfectly then
                coverage_count = coverage_count + struct.count
                m3_covered_ids[struct.id] = true
                table.insert(source_islands_list, struct.id)
            end
        end
        
        local coverage_norm_m3 = coverage_count / total_data_volume
        local lados = 4 
        if best_pure_pair.a ~= 0 then lados = math.floor((360 / math.abs(best_pure_pair.a)) + 0.5) end
        
        local parametros_m3 = lados * 2
        local complexity_norm_m3 = parametros_m3 / total_data_volume
        m3_score = (coverage_norm_m3 * ARBITER_CONFIG.BONO_REGULARIDAD) - (complexity_norm_m3 * cfg.CONFIG.PESO_COMPLEJIDAD)
        
        local synthetic_vals = {}
        for i=1, lados do table.insert(synthetic_vals, best_pure_pair.d); table.insert(synthetic_vals, best_pure_pair.a) end
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
            local tol_cierre = tol_dist * 2.0 
            
            if dist_al_origen <= tol_cierre then
                table.remove(current_pattern_params, #current_pattern_params)
            end
        end
    end

    log(string.format("\n[TRADUCCIÓN FÍSICA - PATRÓN %d]", patron_id))
    for idx, seg in ipairs(current_pattern_params) do 
        log(string.format("   Seg %d: Dist %.2f, Ang %.2f", idx, seg[1], seg[2])) 
    end

    return current_pattern_params
end

-- =========================================================================
--  EJECUCIÓN PRINCIPAL
-- =========================================================================

function GlobalReconstructor.run(robot_id, stored_data, cfg)
    log("\n=========================================")
    log("TABLA DE ISLAS (FRAGMENTOS RECOLECTADOS)")
    log("=========================================")
    for i, v in ipairs(stored_data) do 
        log(string.format("[%d] %s", i, fmt_struct(v))) 
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
    
    build_global_pairs_db(raw_pool, cfg)
    
    local freq_array = {}
    for _, p in ipairs(pairs_db) do table.insert(freq_array, p.count) end
    
    calibrar_hiperparametros(freq_array, pairs_db, cfg)
    
    local limite_ruido = calcular_limite_ruido(freq_array)
    
    local clean_pool = {}
    for _, struct in ipairs(raw_pool) do
        local island_pairs, _ = extract_pairs_from_island(struct, cfg)
        local is_noisy = false
        for _, p in ipairs(island_pairs) do
            for _, db_p in ipairs(pairs_db) do
                if (math.abs(db_p.d - p.d) <= cfg.CONFIG.TOL_DIST and math.abs(db_p.a - p.a) <= cfg.CONFIG.TOL_ANG) then
                    if db_p.count < limite_ruido then is_noisy = true end
                    break
                end
            end
            if is_noisy then break end
        end
        
        if not is_noisy then table.insert(clean_pool, struct) end
    end

    local all_final_results = {} 
    local iteration = 1

    while #clean_pool > 0 do
        local total_data_volume = 0
        for _, v in ipairs(clean_pool) do total_data_volume = total_data_volume + v.count end
        if total_data_volume == 0 then break end

        build_global_pairs_db(clean_pool, cfg)
        local m3_score, m3_hypothesis, m3_covered_ids = evaluar_hipotesis_m3(clean_pool, pairs_db, total_data_volume, iteration, cfg)

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
            log(string.format("[ÁRBITRO] >>> ¡Gana M3 (Geometría Regular)! -> %s", fmt_struct(winner_struct)))
            
        elseif explorer_score >= m3_score and explorer_score >= ARBITER_CONFIG.MIN_SCORE then
            winner_struct = explorer_hypothesis
            ids_to_remove = explorer_hypothesis.consumed_ids
            log(string.format("[ÁRBITRO] >>> ¡Gana %s (Ruta Irregular)! -> %s", explorer_name, fmt_struct(winner_struct)))
        else
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