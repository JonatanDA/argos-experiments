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
    MAX_ISLANDS_M2          = 8,                    -- Límite de islas para M2
    
    selection_mode          = 3,                    -- 1=Frecuente, 2=Larga, 3=Función de Ponderación (Score)
    
    W_FREQ                  = 1,                    -- Peso de la frecuencia
    W_LEN                   = 0,                    -- Positivo: Favorece Cortas. Negativo: Favorece Largas. 0: Ignorar.
    W_OV_QUAL               = 2,                    -- Peso de la calidad de solapes
    THRESHOLD               = 0.2,                  -- 20% para M3
    MATH_TOLERANCE          = 0.5                   -- Tolerancia matemática M3
}

c4 = 0 

local base_patterns = {
    { value = { 53, 50, 74, 50},      type = 1, repeats = 5  },
    { value = { 50, 84, 58.3  },      type = 0, repeats = 3  },
    { value = { 90, 50, 53, 50},      type = 1, repeats = 3  },
    { value = { 50, 90, 50    },      type = 0, repeats = 5  },
    
    { value = { 80, 90, 60 },         type = 0, repeats = 4  },
    { value = { 90, 60, 127},         type = 1, repeats = 5  },
    { value = { 127, 100   },         type = 1, repeats = 4  },

    { value = { -45, 30, -45    },       type = 1, repeats = 2  },
    { value = { -45, 30, -45, 30},       type = 1, repeats = 3  },
    { value = { 30 ,-45,        },       type = 0, repeats = 4  },
    { value = { 30 ,-45, 30, -45},       type = 0, repeats = 3  },
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

function get_window_stats(start_idx, win_size, xx, yy)
    local sum_x, sum_y = 0, 0
    local n = 0
    for j = start_idx, start_idx + win_size - 1 do
        if xx[j] and yy[j] then
            sum_x = sum_x + xx[j]; sum_y = sum_y + yy[j]; n = n + 1
        end
    end
    if n == 0 then return 0, 0, 0 end
    local avg_x = sum_x / n; local avg_y = sum_y / n
    local sum_sq_dist = 0
    for j = start_idx, start_idx + win_size - 1 do
        if xx[j] and yy[j] then
            local dx = xx[j] - avg_x; local dy = yy[j] - avg_y
            sum_sq_dist = sum_sq_dist + (dx*dx + dy*dy)
        end
    end
    return math.sqrt(sum_sq_dist / n), avg_x, avg_y
end

function calcular_otsu_1d(valores)
    if #valores == 0 then return 0, 0 end
    local min_v, max_v = valores[1], valores[1]
    for i = 1, #valores do
        if valores[i] < min_v then min_v = valores[i] end
        if valores[i] > max_v then max_v = valores[i] end
    end
    if min_v == max_v then return min_v, 0 end
    
    local num_bins = 50
    local hist = {}
    for i = 1, num_bins do hist[i] = 0 end
    
    local bin_size = (max_v - min_v) / num_bins
    for i = 1, #valores do
        local bin = math.floor((valores[i] - min_v) / bin_size) + 1
        if bin > num_bins then bin = num_bins end
        hist[bin] = hist[bin] + 1
    end
    
    local total_datos = #valores
    local sum_total = 0
    for i = 1, num_bins do
        local bin_val = min_v + (i - 0.5) * bin_size
        sum_total = sum_total + (bin_val * hist[i])
    end
    
    local weight_bg, sum_bg, max_varianza_interclase, umbral_optimo = 0, 0, -1, 0
    for i = 1, num_bins do
        weight_bg = weight_bg + hist[i]
        if weight_bg > 0 then
            local weight_fg = total_datos - weight_bg
            if weight_fg == 0 then break end
            local bin_val = min_v + (i - 0.5) * bin_size
            sum_bg = sum_bg + (bin_val * hist[i])
            local mean_bg = sum_bg / weight_bg
            local mean_fg = (sum_total - sum_bg) / weight_fg
            local var_between = weight_bg * weight_fg * (mean_bg - mean_fg) * (mean_bg - mean_fg)
            if var_between > max_varianza_interclase then
                max_varianza_interclase = var_between
                umbral_optimo = bin_val
            end
        end
    end
    return umbral_optimo, max_varianza_interclase
end

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
                    exist.count = (exist.count or 1) + 1
                    for k = 1, #new_dat.value do
                        local e_idx = start_idx + k - 1
                        exist.value[e_idx] = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / exist.count
                    end
                    table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    log(string.format("[INFO] Dato Actualizado (Subcadena promediada en medio): %s", fmt_struct(exist)))
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

local function calculate_overlap(s1, s2, min_ov)
    local len1, len2 = #s1.value, #s2.value
    local max_ov = math.min(len1, len2)
    for ov = max_ov, min_ov, -1 do
        local match = true
        for k = 0, ov-1 do
            local idx1, idx2 = len1-ov+1+k, 1+k
            local odd1, odd2 = (idx1%2~=0), (idx2%2~=0)
            local t1 = (s1.type==1) and (odd1 and "A" or "D") or (odd1 and "D" or "A")
            local t2 = (s2.type==1) and (odd2 and "A" or "D") or (odd2 and "D" or "A")
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
    
    local combined_id = tostring(s1.id) .. "+" .. tostring(s2.id)
    local new_total_ov = (s1.total_ov or 0) + (s2.total_ov or 0) + overlap
    return { value = new_vals, type = s1.type, id = combined_id, total_ov = new_total_ov }
end

local function clone_pool(pool)
    local new_pool = {}
    for i, v in ipairs(pool) do
        local nv = {}; for _, val in ipairs(v.value) do table.insert(nv, val) end
        table.insert(new_pool, { value = nv, type = v.type, id = v.id or tostring(i), count = v.count, total_ov = v.total_ov or 0 })
    end
    return new_pool
end

local function solve_greedy(initial_pool)
    local pool = clone_pool(initial_pool)
    log("\n=========================================")
    log("FASE 3: ENSAMBLAJE VORAZ (M1 GREEDY)")
    log("=========================================")

    while #pool > 1 do
        local best = { i=-1, j=-1, ov=-1, struct=nil }
        for i=1, #pool do
            for j=1, #pool do
                if i ~= j then
                    local ov = calculate_overlap(pool[i], pool[j], CONFIG.MIN_OVERLAP_M1)
                    if ov > best.ov then
                        best = { i=i, j=j, ov=ov, struct=merge_fragments(pool[i], pool[j], ov) }
                    end
                end
            end
        end
        
        if best.ov > 0 then
            local name_A = pool[best.i].id and ("Frag " .. pool[best.i].id) or "Frag ?"
            local name_B = pool[best.j].id and ("Frag " .. pool[best.j].id) or "Frag ?"
            log(string.format("[MERGE] Uniendo Indices %d y %d (Solape: %d) -> %s", best.i, best.j, best.ov, name_A.."+"..name_B))
            pool[best.i] = best.struct
            table.remove(pool, best.j)
        else
            log("[INFO] No se encontraron más solapes posibles con MIN_OVERLAP_M1 = " .. CONFIG.MIN_OVERLAP_M1)
            break
        end
    end
    return pool
end

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
        if #history_log > 0 then 
            table.insert(raw_solutions_found, { pool_result = current_pool, history = history_log }) 
        end
    end
end

local function select_best_solution(solutions)
    if #solutions == 0 then return nil end
    local unique_groups = {}
    
    for _, sol in ipairs(solutions) do
        local hashes = {}
        local total_len = 0
        local total_ov = 0
        for _, s in ipairs(sol.pool_result) do 
            table.insert(hashes, fmt_struct(s))
            total_len = total_len + #s.value
            total_ov = total_ov + (s.total_ov or 0)
        end
        table.sort(hashes)
        local key = table.concat(hashes, " || ")
        
        if not unique_groups[key] then 
            unique_groups[key] = { 
                pool_result = sol.pool_result, 
                count = 1, 
                history = sol.history,
                total_len = total_len,
                total_ov = total_ov
            }
        else 
            unique_groups[key].count = unique_groups[key].count + 1 
        end
    end

    local unique_list = {}
    for key, data in pairs(unique_groups) do
        local score = (data.count * CONFIG.W_FREQ) + (data.total_ov * CONFIG.W_OV_QUAL)
        if CONFIG.W_LEN > 0 then score = score + ((100 / data.total_len) * CONFIG.W_LEN)
        elseif CONFIG.W_LEN < 0 then score = score + (data.total_len * math.abs(CONFIG.W_LEN)) end
        
        data.score = score
        table.insert(unique_list, data)
    end

    table.sort(unique_list, function(a, b)
        if a.score == b.score then return a.count > b.count end
        return a.score > b.score
    end)

    log(string.format("[GESTOR] M2 encontró %d caminos de término, colapsados en %d estados globales únicos.", #solutions, #unique_list))

    if CONFIG.PRINT_RECURSIVE_PATHS then
        for idx, sol in ipairs(unique_list) do
            log(string.format("\n  >> [OPCIÓN %d] Frecuencia: %d, Cadenas Desconectadas: %d | Score: %.2f", idx, sol.count, #sol.pool_result, sol.score))
            for c_idx, s in ipairs(sol.pool_result) do
                log(string.format("     Cadena %d: %s", c_idx, fmt_struct(s)))
            end
            log("     Historial hasta este estado:")
            for _, step in ipairs(sol.history) do log("     " .. step) end
        end
        log("")
    end

    local best_choice = unique_list[1]
    
    local certeza = 100
    if #unique_list > 1 then
        local sum_scores = 0
        for _, s in ipairs(unique_list) do if s.score > 0 then sum_scores = sum_scores + s.score end end
        if sum_scores > 0 and best_choice.score > 0 then certeza = (best_choice.score / sum_scores) * 100 else certeza = 0 end
    end
    
    log(string.format("[GESTOR] M2 DECISIÓN: Seleccionada la OPCIÓN 1 por Mejor Score Global (%.2f).", best_choice.score))
    log(string.format("[GESTOR] CERTEZA DE SOLUCIÓN M2: %.1f%%", certeza))
    
    return best_choice
end

local function solve_smart_backtracking_all(initial_pool)
    raw_solutions_found = {} 
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) return #a.value > #b.value end)
    recursive_finder(sorted_pool, {})
    return select_best_solution(raw_solutions_found)
end

-- =========================================================================
--  6.3: MÉTODO 3 ITERATIVO 
-- =========================================================================

local function is_struct_part_of_periodic(struct, d_pair)
    -- VERIFICACIÓN ESTRICTA: La isla no debe contener NINGÚN valor ajeno al par
    local is_dist_first = (struct.type == 0)
    for i, val in ipairs(struct.value) do
        local is_dist = is_dist_first and (i % 2 ~= 0) or (not is_dist_first and i % 2 == 0)
        local tol = is_dist and CONFIG.TOL_DIST or CONFIG.TOL_ANG
        local target = is_dist and d_pair.d or d_pair.a
        
        -- Si encontramos un número en la isla que se desvía del par, la descartamos
        if math.abs(val - target) > tol then
            return false
        end
    end
    return true
end

local function check_and_extract_regular_polygon(pool)
    local pairs_found = {}
    local total_pairs = 0
    
    for _, struct in ipairs(pool) do
        if #struct.value >= 2 then
            local is_dist_first = (struct.type == 0)
            local i = 1
            while i < #struct.value do
                local dist, ang
                if is_dist_first then dist = struct.value[i]; ang = struct.value[i+1]
                else ang = struct.value[i]; dist = struct.value[i+1] end

                if dist and ang then
                    total_pairs = total_pairs + (struct.count or 1)
                    local matched = false
                    for _, p in ipairs(pairs_found) do
                        if math.abs(p.d - dist) <= CONFIG.TOL_DIST and math.abs(p.a - ang) <= CONFIG.TOL_ANG then
                            p.d = p.d + (dist - p.d) / (p.count + 1); p.a = p.a + (ang - p.a) / (p.count + 1)
                            p.count = p.count + (struct.count or 1); matched = true
                            break
                        end
                    end
                    if not matched then table.insert(pairs_found, {d = dist, a = ang, count = (struct.count or 1)}) end
                end
                i = i + 2
            end
        end
    end

    if total_pairs == 0 then return nil, pool, false end

    local valid_pairs = {}
    for _, p in ipairs(pairs_found) do
        p.percentage = (p.count / total_pairs) * 100
        if (p.count / total_pairs) >= CONFIG.THRESHOLD then 
            table.insert(valid_pairs, p) 
        end
    end

    table.sort(valid_pairs, function(a,b) return a.count > b.count end)

    for _, dominant_pair in ipairs(valid_pairs) do
        local remaining_pool = {}
        local extracted_count = 0
        for _, struct in ipairs(pool) do
            if is_struct_part_of_periodic(struct, dominant_pair) then
                extracted_count = extracted_count + 1
            else
                table.insert(remaining_pool, struct)
            end
        end

        if extracted_count > 0 then
            log(string.format("[M3] Patrón periódico extraído: Dist %.2f, Ang %.2f | Repetición: %.1f%% (Limpiadas %d islas)", 
                dominant_pair.d, dominant_pair.a, dominant_pair.percentage, extracted_count))
                
            -- CÁLCULO CORRECTO DE LADOS
            local lados = 4 -- Valor por defecto seguro (línea recta)
            if dominant_pair.a ~= 0 then
                lados = math.floor((360 / math.abs(dominant_pair.a)) + 0.5)
            end
            
            local synthetic_vals = {}
            for i=1, lados do 
                table.insert(synthetic_vals, dominant_pair.d)
                table.insert(synthetic_vals, dominant_pair.a) 
            end
            local synthetic_struct = { type = 0, value = synthetic_vals, id = "M3_Periódico_"..math.floor(dominant_pair.d) }
            
            return synthetic_struct, remaining_pool, true
        end
    end

    return nil, pool, false
end


-- =========================================================================
--  6.4. GESTOR PRINCIPAL
-- =========================================================================
function run_global_reconstruction()
    if Global_Reconstruction_Done then return end
    
    log("\n=========================================")
    log("TABLA DE ISLAS (FRAGMENTOS RECOLECTADOS)")
    log("=========================================")
    for i, v in ipairs(DataBuffer.stored) do log(string.format("[%d] %s", i, fmt_struct(v))) end
    log("=========================================\n")
        
    local current_pool = {}
    for i, v in ipairs(DataBuffer.stored) do table.insert(current_pool, { value = v.value, type = v.type, count = v.count, id = tostring(i) }) end
    if #current_pool == 0 then return end
    
    local all_final_results = {} 

    -- FASE 1: M3 ITERATIVO
    log("=========================================")
    log("FASE 1: EXTRACCIÓN DE PERIODICIDAD (M3)")
    log("=========================================")
    while true do
        local periodic_struct, remaining, found = check_and_extract_regular_polygon(current_pool)
        if found then
            table.insert(all_final_results, periodic_struct)
            current_pool = remaining
            if #current_pool == 0 then break end
        else
            if #all_final_results == 0 then log("[M3] No se detectaron patrones periódicos fuertes.") end
            break
        end
    end

    -- FASE 2 y 3: OTSU Y M1/M2
    if #current_pool > 0 then
        local counts = {}
        for _, v in ipairs(current_pool) do table.insert(counts, v.count) end
        local min_f, max_f = counts[1], counts[1]
        for _, c in ipairs(counts) do
            if c < min_f then min_f = c end
            if c > max_f then max_f = c end
        end

        log("\n=========================================")
        log("FASE 2: FILTRO DE RUIDO ESPORÁDICO (OTSU)")
        log("=========================================")
        
        local clean_pool = {}
        if (max_f - min_f) > 2 then
            local umbral_otsu, _ = calcular_otsu_1d(counts)
            log(string.format("[OTSU] Umbral calculado: %.2f. Descartando ruido...", umbral_otsu))
            for _, v in ipairs(current_pool) do
                if v.count >= umbral_otsu then table.insert(clean_pool, v) end
            end
        else
            log("[OTSU] Muestra Unimodal. Pasan todas las islas restantes.")
            clean_pool = current_pool
        end

        if #clean_pool > 0 then
            local greedy_result_pool = solve_greedy(clean_pool)
            local m1_perfect = (#greedy_result_pool == 1)

            if m1_perfect then
                table.insert(all_final_results, greedy_result_pool[1])
            else
                log(string.format("[GESTOR] M1 dejó %d macro-islas. Evaluando M2...", #greedy_result_pool))
                local run_m2 = CONFIG.FORCE_M2_ALWAYS or true
                if run_m2 and #clean_pool <= CONFIG.MAX_ISLANDS_M2 then
                    log("\n=========================================")
                    log("FASE 4: RESOLUCIÓN DE CONFLICTOS (M2)")
                    log("=========================================")
                    local best_match = solve_smart_backtracking_all(clean_pool)
                    if best_match then
                        log("[GESTOR] ¡M2 Exitoso! Guardando el Estado Global (Multi-Cadena).")
                        for _, s in ipairs(best_match.pool_result) do
                            table.insert(all_final_results, s)
                        end
                    else
                        log("[GESTOR] M2 Falló. Salvando las ramas de M1.")
                        for _, rama in ipairs(greedy_result_pool) do
                            table.insert(all_final_results, rama)
                        end
                    end
                else
                    log("[GESTOR] Demasiadas islas para M2 o M2 desactivado. Guardando las ramas de M1.")
                    for _, rama in ipairs(greedy_result_pool) do
                        table.insert(all_final_results, rama)
                    end
                end
            end
        end
    end

    log("\n---------------- RESULTADOS FINALES ----------------")
    if #all_final_results > 0 then
        log(string.format(">> Se descubrieron %d patrones distintos:", #all_final_results))
        for i, res in ipairs(all_final_results) do
            log(string.format("   Patrón %d: %s", i, fmt_struct(res)))
        end
    else
        log(">> ERROR CRÍTICO: No se pudo recuperar ninguna estructura válida.")
    end
    log("----------------------------------------------------\n")

    Global_Reconstruction_Done = true
    for i, res in ipairs(all_final_results) do
        process_numeric_result(res, i)
    end
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
-- [ZONA 3] EL CICLO DE SIMULACIÓN
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