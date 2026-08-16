-- ============================================================================
-- [ZONA 1] ENTORNO DE PRUEBA STANDALONE (Simulación de Entradas)
-- ============================================================================

function log(msg) print(msg) end
math.randomseed(os.time())

CONFIG = {
    TOL_DIST                = 3,                    -- (cm)
    TOL_ANG                 = 3,                    -- (°)
    MIN_OVERLAP_M1          = 1,                    -- Solape mínimo independiente para Greedy (M1)
    MIN_OVERLAP_M2          = 1,                    -- Solape mínimo independiente para Exhaustivo (M2)
    PRINT_RECURSIVE_PATHS   = true,                 -- Imprime los pasos de M2
    FORCE_M2_ALWAYS         = false,                -- Ejecutar M2 incluso si M1 fue perfecto
    MAX_ISLANDS_M2          = 9,                    -- Límite de islas para M2
    
    selection_mode          = 3,                    -- 1=Frecuente, 2=Larga, 3=Función de Ponderación (Score)
    
    W_FREQ                  = 1,                    -- Peso de la frecuencia
    W_LEN                   = 0,                    -- Positivo: Favorece Cortas. Negativo: Favorece Largas. 0: Ignorar.
    W_OV_QUAL               = 2,                    -- Peso de la calidad de solapes

    MATH_TOLERANCE          = 0.5                   -- Tolerancia matemática M3
}

-- Configuración del Árbitro y Filtro
local ARBITER_CONFIG = {
    BONO_REGULARIDAD = 1.3,  -- Multiplicador para dar ventaja a los polígonos
    MIN_SCORE = 5.0,         -- Score mínimo para aceptar una hipótesis
    RUIDO_UMBRAL_RELATIVO = 0.08 -- 8% de la frecuencia máxima (Filtro de Ruido Dinámico)
}


c4 = 0 

local base_patterns = {
    --Pentágono_irregular
    { value = { 53, 50, 74, 50},      type = 1, repeats = 25  },
    { value = { 50, 84, 58.3  },      type = 0, repeats = 30  },
    { value = { 90, 50, 53, 50},      type = 1, repeats = 30  },
    { value = { 50, 90, 50    },      type = 0, repeats = 25  },
    
    -- Triangulo irregular
    { value = { 80, 90, 60 },         type = 0, repeats = 40  },
    { value = { 90, 60, 127},         type = 1, repeats = 25  },
    { value = { 127, 100   },         type = 1, repeats = 40  },

    --Triangulo regular
    { value = { 78, 120, 78 },         type = 0, repeats = 1  },
    { value = { 78, 120     },         type = 0, repeats = 7  },

    --Octágono  
    { value = { -45, 30, -45    },    type = 1, repeats = 2   },
    { value = { -45, 30, -45, 30},    type = 1, repeats = 15  },
    { value = { 30 ,-45,        },    type = 0, repeats = 10  },
    { value = { 30 ,-45, 30, -45},    type = 0, repeats = 16  },

    --ruido
    { value = { 30 ,-45, -20    },    type = 0, repeats = 3  },
    { value = { 70 ,-21, 30,  45},    type = 0, repeats = 4  },


}

    -- Patron_Original{
    --         params = {
    --         { 50, 10},
    --         { 50, 90},
    --         { 50, 53},
    --         { 50, 74},
    --         { 58.3, 84},
    --         },
    --     name = "Pentágono_irregular"
    -- }

    -- Patron_Original{
    --         params = {
    --         { 80, 00},
    --         { 60, 90},
    --         { 100, 127},
    --         },
    --     name = "Triangulo irregular 1"
    -- }

    -- Patron_Original{
    --         params = {
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         { 30 ,-45 },
    --         },
    --     name = "octógono"
    -- }

local test_queue = {}
local total_fragmentos = 0

for _, pattern in ipairs(base_patterns) do
    for i = 1, pattern.repeats do
        local cloned_val = {}
        for _, v in ipairs(pattern.value) do 
            local noise = (math.random() * 3.0) - 1.5
            table.insert(cloned_val, v + noise) 
        end
        table.insert(test_queue, { value = cloned_val, type = pattern.type })
        total_fragmentos = total_fragmentos + 1
    end
end

for i = #test_queue, 2, -1 do
    local j = math.random(i)
    test_queue[i], test_queue[j] = test_queue[j], test_queue[i]
end

local current_patience = 30
local MAX_PATIENCE = 30

function update_patience(type_update, count)
    current_patience = MAX_PATIENCE
end

function check_patience_and_reconstruct()
    if Global_Reconstruction_Done then return true end
    current_patience = current_patience - 1
    
    if current_patience <= 0 then
        log("\n[!] PACIENCIA AGOTADA. Iniciando reconstrucción global...\n")
        run_global_reconstruction()
        return true
    end
    return false
end

-- ============================================================================
-- [ZONA 2] CÓDIGO DEL CONTROLADOR (Copia y Pega a ARGoS3)
-- ============================================================================

DataBuffer = { stored = {} }
Global_Reconstruction_Done = false 

-- ================= HERRAMIENTAS DE TEXTO Y UTILIDAD =================

local function fmt_struct(s)
    if not s then return "nil" end
    local vals = {}
    for _, v in ipairs(s.value) do table.insert(vals, string.format("%.2f", v)) end
    local t_str = (s.type == 1) and "Ang" or "Dist"
    local count_str = (s.count and s.count > 1) and string.format(" [x%d]", s.count) or ""
    return string.format("(%s)%s {%s}", t_str, count_str, table.concat(vals, ", "))
end

local function simple_fmt(val_table)
    local vals = {}
    for _, v in ipairs(val_table) do table.insert(vals, string.format("%.2f", v)) end
    return table.concat(vals, ",")
end

local function get_val_type(struct_type, index)
    local is_odd = (index % 2 ~= 0)
    if struct_type == 1 then return is_odd and "A" or "D"
    else return is_odd and "D" or "A" end
end

local function check_match(long_dat, short_dat, start_idx)
    if get_val_type(long_dat.type, start_idx) ~= get_val_type(short_dat.type, 1) then return false end
    for k = 1, #short_dat.value do
        local l_idx = start_idx + k - 1
        local v_type = get_val_type(short_dat.type, k)
        local diff = math.abs(long_dat.value[l_idx] - short_dat.value[k])
        local tol = (v_type == "A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
        if diff > tol then return false end
    end
    return true
end

-- ================= ALMACENAMIENTO DE ISLAS =================

function DataBuffer.add(new_dat)
    if Global_Reconstruction_Done then return end 
    local found = false

    for i, exist in ipairs(DataBuffer.stored) do
        if #exist.value == #new_dat.value and exist.type == new_dat.type then
            if check_match(exist, new_dat, 1) then
                found = true
                exist.count = (exist.count or 1) + 1
                for k = 1, #exist.value do 
                    exist.value[k] = exist.value[k] + (new_dat.value[k] - exist.value[k]) / exist.count 
                end
                table.insert(exist.same_size_merges, simple_fmt(new_dat.value))
                log(string.format("[INFO] Dato Refinado (Promedio de %d): %s", exist.count, fmt_struct(exist)))
                update_patience("partial", exist.count) 
                break
            end
        end

        if not found and #new_dat.value >= #exist.value + 2 then
            for start_idx = 2, #new_dat.value - #exist.value do
                if check_match(new_dat, exist, start_idx) then
                    found = true
                    exist.count = (exist.count or 1) + 1
                    local merged_values = {}
                    for k = 1, #new_dat.value do
                        if k >= start_idx and k < start_idx + #exist.value then
                            local e_idx = k - start_idx + 1
                            local avg_val = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / exist.count
                            table.insert(merged_values, avg_val)
                        else
                            table.insert(merged_values, new_dat.value[k])
                        end
                    end
                    exist.value = merged_values; exist.type = new_dat.type 
                    table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    log(string.format("[INFO] Dato Actualizado (Extendido a %d val, solape en medio): %s", #exist.value, fmt_struct(exist)))
                    update_patience("partial", exist.count)
                    break
                end
            end
            if found then break end
        end

        if not found and #exist.value >= #new_dat.value + 2 then
            for start_idx = 2, #exist.value - #new_dat.value do
                if check_match(exist, new_dat, start_idx) then
                    found = true
                    local divisor = (exist.count or 1) + 1
                    for k = 1, #new_dat.value do
                        local e_idx = start_idx + k - 1
                        exist.value[e_idx] = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / divisor
                    end
                    table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    log(string.format("[INFO] Dato Actualizado (Subcadena promediada - Frec INTACTA %d): %s", exist.count, fmt_struct(exist)))
                    update_patience("partial", exist.count)
                    break
                end
            end
            if found then break end
        end
    end

    if not found then
        local new_v = {}
        for _, v in ipairs(new_dat.value) do table.insert(new_v, v) end
        local entry = { value = new_v, type = new_dat.type, count = 1, same_size_merges = {}, diff_size_merges = {} }
        table.insert(DataBuffer.stored, entry)
        log(string.format("[INFO] Nuevo Dato Registrado: %s", fmt_struct(entry)))
        update_patience("full", 1)
    end
end

-- ================= FILTRO DE RUIDO RELATIVO =================

local function calcular_limite_ruido(frecuencias)
    if #frecuencias == 0 then return 0 end
    local max_freq = 0
    for _, f in ipairs(frecuencias) do
        if f > max_freq then max_freq = f end
    end
    return max_freq * ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO
end

-- ================= EXTRACCIÓN DE PARES Y PUREZA =================

local pairs_db = {}

local function extract_pairs_from_island(struct)
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
            local tol = (v_type == "A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
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

local function build_global_pairs_db(pool)
    pairs_db = {}
    for _, struct in ipairs(pool) do
        local island_pairs, is_pure = extract_pairs_from_island(struct)
        for _, p in ipairs(island_pairs) do
            local found = false
            for _, db_p in ipairs(pairs_db) do
                local values_match = (math.abs(db_p.d - p.d) <= CONFIG.TOL_DIST and math.abs(db_p.a - p.a) <= CONFIG.TOL_ANG)
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

-- ================= ENSAMBLADORES (M1 / M2) =================

local function calculate_overlap(s1, s2, min_ov)
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
            local tol = (t1=="A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
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

-- METODO 1: GREEDY
local function solve_greedy(initial_pool)
    local pool = clone_pool(initial_pool)
    while #pool > 1 do
        local best = { i=-1, j=-1, ov=-1, struct=nil }
        for i=1, #pool do
            for j=1, #pool do
                if i ~= j then
                    local ov = calculate_overlap(pool[i], pool[j], CONFIG.MIN_OVERLAP_M1)
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

-- METODO 2: EXHAUSTIVE BACKTRACKING
local raw_solutions_found = {} 

local function recursive_finder(current_pool, history_log)
    local made_merge = false
    local candidates = {}
    for i=1, #current_pool do
        for j=1, #current_pool do
            if i ~= j then
                local ov = calculate_overlap(current_pool[i], current_pool[j], CONFIG.MIN_OVERLAP_M2)
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
            recursive_finder(next_pool, next_history)
        end
    end

    if not made_merge then 
        table.insert(raw_solutions_found, { pool_result = current_pool, history = history_log }) 
    end
end

local function select_best_solution_m2(solutions)
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
        -- Score interno de M2 (Evalúa la calidad de todo el pool resultante)
        local score = (data.count * CONFIG.W_FREQ) + (data.total_ov * CONFIG.W_OV_QUAL)
        data.score = score; table.insert(unique_list, data)
    end

    table.sort(unique_list, function(a, b)
        if a.score == b.score then return a.count > b.count end
        return a.score > b.score
    end)

    return unique_list[1]
end

local function solve_smart_backtracking_all(initial_pool)
    raw_solutions_found = {} 
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) return #a.value > #b.value end)
    recursive_finder(sorted_pool, {})
    return select_best_solution_m2(raw_solutions_found)
end


-- =========================================================================
--  GESTOR PRINCIPAL: EL BUCLE ITERATIVO DE EXTRACCIÓN (ÁRBITRO)
-- =========================================================================
function run_global_reconstruction()
    if Global_Reconstruction_Done then return end
    
    -- LOG SOLICITADO: TABLA DE ISLAS
    log("\n=========================================")
    log("TABLA DE ISLAS (FRAGMENTOS RECOLECTADOS)")
    log("=========================================")
    for i, v in ipairs(DataBuffer.stored) do 
        log(string.format("[%d] %s", i, fmt_struct(v))) 
    end
        
    local raw_pool = {}
    for i, v in ipairs(DataBuffer.stored) do 
        table.insert(raw_pool, { 
            value = v.value, 
            type = v.type, 
            count = v.count, 
            id = tostring(i),
            consumed_ids = { [tostring(i)] = true } 
        }) 
    end
    if #raw_pool == 0 then return end
    
    -- FASE 1: LOGS DE PARES Y FILTRADO ESTADÍSTICO
    build_global_pairs_db(raw_pool)
    
    log("\n=========================================")
    log("FASE 1: EXTRACCIÓN Y CONTEO DE PARES (N-GRAMAS)")
    log("=========================================")
    local freq_array = {}
    for _, p in ipairs(pairs_db) do 
        table.insert(freq_array, p.count) 
        local t_str = (p.type == 1) and "A-D" or "D-A"
        log(string.format("[PAR %s] Dist %.2f, Ang %.2f | Frecuencia global: %d (Pureza global: %s)", t_str, p.d, p.a, p.count, tostring(p.is_pure)))
    end
    
    local limite_ruido = calcular_limite_ruido(freq_array)
    log(string.format("\n>> Límite de Ruido Dinámico Calculado: %.2f", limite_ruido))
    
    local clean_pool = {}
    local noise_removed_count = 0
    for _, struct in ipairs(raw_pool) do
        local island_pairs, _ = extract_pairs_from_island(struct)
        local is_noisy = false
        for _, p in ipairs(island_pairs) do
            for _, db_p in ipairs(pairs_db) do
                if (math.abs(db_p.d - p.d) <= CONFIG.TOL_DIST and math.abs(db_p.a - p.a) <= CONFIG.TOL_ANG) then
                    if db_p.count < limite_ruido then is_noisy = true end
                    break
                end
            end
            if is_noisy then break end
        end
        
        if is_noisy then
            noise_removed_count = noise_removed_count + 1
            log(string.format("  [-] Eliminado por ruido (Frecuencia baja): %s", fmt_struct(struct)))
        else
            table.insert(clean_pool, struct)
        end
    end

    local all_final_results = {} 
    local iteration = 1

    -- FASE 2: EL BUCLE DEL ÁRBITRO
    log("\n=========================================")
    log("FASE 2: EXTRACCIÓN ITERATIVA DE PATRONES")
    log("=========================================")

    while #clean_pool > 0 do
        log(string.format("\n--- ITERACIÓN %d | Fragmentos en buffer: %d ---", iteration, #clean_pool))
        
        local total_data_volume = 0
        for _, v in ipairs(clean_pool) do total_data_volume = total_data_volume + v.count end
        if total_data_volume == 0 then break end

        -- =======================================================
        -- HIPÓTESIS M3 (El Geómetra)
        -- =======================================================
        local m3_score = -1
        local m3_hypothesis = nil
        local m3_covered_ids = {} 
        
        build_global_pairs_db(clean_pool)
        local best_pure_pair = nil
        for _, p in ipairs(pairs_db) do
            if p.is_pure then
                if not best_pure_pair or p.count > best_pure_pair.count then best_pure_pair = p end
            end
        end

        if best_pure_pair then
            local coverage_count = 0
            local source_islands_list = {}

            for _, struct in ipairs(clean_pool) do
                local island_pairs, _ = extract_pairs_from_island(struct)
                local matches_perfectly = true
                for _, p in ipairs(island_pairs) do
                    if math.abs(p.d - best_pure_pair.d) > CONFIG.TOL_DIST or math.abs(p.a - best_pure_pair.a) > CONFIG.TOL_ANG then
                        matches_perfectly = false; break
                    end
                end
                if matches_perfectly then
                    coverage_count = coverage_count + struct.count
                    m3_covered_ids[struct.id] = true
                    table.insert(source_islands_list, struct.id)
                end
            end
            
            local coverage_percent = (coverage_count / total_data_volume) * 100
            m3_score = (coverage_percent * ARBITER_CONFIG.BONO_REGULARIDAD) - 2 
            
            local lados = 4 
            if best_pure_pair.a ~= 0 then lados = math.floor((360 / math.abs(best_pure_pair.a)) + 0.5) end
            local synthetic_vals = {}
            for i=1, lados do table.insert(synthetic_vals, best_pure_pair.d); table.insert(synthetic_vals, best_pure_pair.a) end
            
            m3_hypothesis = { type = 0, value = synthetic_vals, id = "Poligono_Iter"..iteration, count = coverage_count }
            
            local sources_str = table.concat(source_islands_list, ", ")
            log(string.format("[M3] Analizando Par {Dist: %.2f, Ang: %.2f} de Islas [%s]", best_pure_pair.d, best_pure_pair.a, sources_str))
            log(string.format("     -> Propone Polígono (%d lados). Cobertura: %.1f%% | Score: %.2f", lados, coverage_percent, m3_score))
        else
            log("[M3] No hay fragmentos puros. Se abstiene.")
        end

        -- =======================================================
        -- HIPÓTESIS DE LOS EXPLORADORES (M2 prioritario, M1 fallback)
        -- =======================================================
        local explorer_score = -1
        local explorer_hypothesis = nil
        local explorer_name = "M1"
        
        local explorer_pool_result = {}
        
        if #clean_pool <= CONFIG.MAX_ISLANDS_M2 then
            log("[EXPLORADORES] Activando M2 (Backtracking Exhaustivo)...")
            local m2_solution = solve_smart_backtracking_all(clean_pool)
            if m2_solution and #m2_solution.pool_result > 0 then
                explorer_pool_result = m2_solution.pool_result
                explorer_name = "M2"
            else
                log("[EXPLORADORES] M2 no encontró combinaciones, usando M1 Fallback.")
                explorer_pool_result = solve_greedy(clean_pool)
            end
        else
            log(string.format("[EXPLORADORES] Buffer excede %d islas. Usando M1 (Greedy Voraz)...", CONFIG.MAX_ISLANDS_M2))
            explorer_pool_result = solve_greedy(clean_pool)
        end

        -- De todas las cadenas que armó el explorador, extraemos la más dominante para competir
        if #explorer_pool_result > 0 then
            table.sort(explorer_pool_result, function(a,b) return (a.count or 1) > (b.count or 1) end)
            explorer_hypothesis = explorer_pool_result[1]
            
            local coverage_percent = ((explorer_hypothesis.count or 1) / total_data_volume) * 100
            local complexity = #explorer_hypothesis.value
            explorer_score = (coverage_percent * 1.0) - complexity
            
            log(string.format("[%s] Propone Cadena (Largo %d). Cobertura: %.1f%% | Score: %.2f", explorer_name, complexity, coverage_percent, explorer_score))
        end

        -- =======================================================
        -- EL ÁRBITRO DECIDE
        -- =======================================================
        local winner_struct = nil
        local ids_to_remove = {}

        if m3_score > explorer_score and m3_score >= ARBITER_CONFIG.MIN_SCORE then
            winner_struct = m3_hypothesis
            ids_to_remove = m3_covered_ids
            log(string.format("[ÁRBITRO] ¡Gana M3 (Patrón Regular)! -> %s", fmt_struct(winner_struct)))
            
        elseif explorer_score >= m3_score and explorer_score >= ARBITER_CONFIG.MIN_SCORE then
            winner_struct = explorer_hypothesis
            ids_to_remove = explorer_hypothesis.consumed_ids
            log(string.format("[ÁRBITRO] ¡Gana %s (Patrón Irregular)! -> %s", explorer_name, fmt_struct(winner_struct)))
            
        else
            log("[ÁRBITRO] Ninguna hipótesis supera el umbral de confianza. Fin del ciclo.")
            break
        end

        -- EXTRACCIÓN Y LIMPIEZA DEL BUFFER
        table.insert(all_final_results, winner_struct)
        local next_pool = {}
        local removed_count = 0
        for _, struct in ipairs(clean_pool) do
            if not ids_to_remove[struct.id] then table.insert(next_pool, struct)
            else removed_count = removed_count + 1 end
        end
        
        log(string.format("  > Se extrajeron %d fragmentos base de la arena.", removed_count))
        clean_pool = next_pool
        iteration = iteration + 1
    end

    log("\n---------------- RESULTADOS FINALES ----------------")
    if #all_final_results > 0 then
        log(string.format(">> Se descubrieron %d patrones distintos:", #all_final_results))
        for i, res in ipairs(all_final_results) do log(string.format("   Patrón %d: %s", i, fmt_struct(res))) end
    else
        log(">> ERROR CRÍTICO: No se pudo recuperar ninguna estructura válida.")
    end
    log("----------------------------------------------------\n")

    Global_Reconstruction_Done = true
    for i, res in ipairs(all_final_results) do process_numeric_result(res, i) end
end

function process_numeric_result(result_struct, patron_id)
    local full_pattern_params = {}
    local values = result_struct.value
    local current_type = result_struct.type
    
    local temp_dist = 0; local temp_ang = 0
    local expecting_dist = (current_type == 0)
    
    for i, val in ipairs(values) do
        if expecting_dist then
            temp_dist = val; table.insert(full_pattern_params, { temp_dist, temp_ang })
            temp_ang = 0; expecting_dist = false
        else
            temp_ang = val; expecting_dist = true
        end
    end
    
    log(string.format("\n[TRADUCCIÓN FÍSICA - PATRÓN %d]", patron_id))
    for idx, seg in ipairs(full_pattern_params) do 
        log(string.format("   Seg %d: Dist %.2f, Ang %.2f", idx, seg[1], seg[2])) 
    end
    c4 = 1 
end

-- ============================================================================
-- [ZONA 3] SIMULACION DE ENTRADAS Y EJECUCIÓN DEL CONTROLADOR
-- ============================================================================

-- ============================================================================
log("\n--- INICIANDO SIMULACIÓN STANDALONE ---")
local tick = 0
while true do
    tick = tick + 1
    if check_patience_and_reconstruct() then break end
    if #test_queue > 0 and math.random() > 0.6 then 
        local new_data = table.remove(test_queue)
        DataBuffer.add(new_data)
    end
    if tick > 1000 then break end
end