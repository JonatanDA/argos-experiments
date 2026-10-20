-- ============================================================================
-- Archivo: Functions/Observer/Parameter_Storage.lua
-- ============================================================================

local ParameterStorage = {}

ParameterStorage.USE_GLOBAL_MEMORY = true 

-- NUEVA VARIABLE DE LOGGEO INDEPENDIENTE:
-- 0: Sin logs
-- 1: Logs básicos (Cuando añade o refina datos)
-- 2: Logs detallados (Depuración de memoria global paso a paso)
ParameterStorage.PS_LOG_MODE = 2

local DataBuffer = { 
    stored = {},            
    global_stored = {}      
}
local Global_Reconstruction_Done = false 
local local_id_counter = 0 

function ParameterStorage.is_done() return Global_Reconstruction_Done end
function ParameterStorage.set_done() Global_Reconstruction_Done = true end
function ParameterStorage.get_stored_data() return DataBuffer.stored end
function ParameterStorage.get_global_stored_data() return DataBuffer.global_stored end

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

local function check_match(long_dat, short_dat, start_idx, cfg)
    if get_val_type(long_dat.type, start_idx) ~= get_val_type(short_dat.type, 1) then return false end
    for k = 1, #short_dat.value do
        local l_idx = start_idx + k - 1
        local v_type = get_val_type(short_dat.type, k)
        local diff = math.abs(long_dat.value[l_idx] - short_dat.value[k])
        local tol = (v_type == "A") and cfg.CONFIG.TOL_ANG or cfg.CONFIG.TOL_DIST

        if diff > (tol * 1.5) then return false end
    end
    return true
end

local function merge_into_buffer(buffer, new_dat, cfg, do_modify)
    local found_level = 0
    local exist_ref = nil

    for i, exist in ipairs(buffer) do
        if #exist.value == #new_dat.value and exist.type == new_dat.type then
            if check_match(exist, new_dat, 1, cfg) then
                found_level = 2
                if do_modify then
                    local c1 = exist.count or 1
                    local c2 = new_dat.count or 1
                    local new_total = c1 + c2

                    for k = 1, #exist.value do 
                        exist.value[k] = ((exist.value[k] * c1) + (new_dat.value[k] * c2)) / new_total
                    end
                    exist.count = new_total
                    table.insert(exist.same_size_merges, simple_fmt(new_dat.value))
                end
                exist_ref = exist
                break
            end
        end

        if found_level == 0 and #new_dat.value > #exist.value then
            for start_idx = 1, #new_dat.value - #exist.value + 1 do
                if check_match(new_dat, exist, start_idx, cfg) then
                    found_level = 1
                    if do_modify then
                        local c1 = exist.count or 1
                        local c2 = new_dat.count or 1
                        local new_total = c1 + c2

                        local merged_values = {}
                        for k = 1, #new_dat.value do
                            if k >= start_idx and k < start_idx + #exist.value then
                                local e_idx = k - start_idx + 1
                                local avg_val = ((exist.value[e_idx] * c1) + (new_dat.value[k] * c2)) / new_total
                                table.insert(merged_values, avg_val)
                            else
                                table.insert(merged_values, new_dat.value[k])
                            end
                        end
                        exist.value = merged_values
                        exist.type = new_dat.type 
                        exist.count = new_total
                        table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    end
                    exist_ref = exist
                    break
                end
            end
            if found_level > 0 then break end
        end

        if found_level == 0 and #exist.value > #new_dat.value then
            for start_idx = 1, #exist.value - #new_dat.value + 1 do
                if check_match(exist, new_dat, start_idx, cfg) then
                    found_level = 1
                    if do_modify then
                        local c1 = exist.count or 1
                        local c2 = new_dat.count or 1
                        local new_total = c1 + c2

                        for k = 1, #new_dat.value do
                            local e_idx = start_idx + k - 1
                            exist.value[e_idx] = ((exist.value[e_idx] * c1) + (new_dat.value[k] * c2)) / new_total
                        end
                        exist.count = new_total
                        table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    end
                    exist_ref = exist
                    break
                end
            end
            if found_level > 0 then break end
        end
    end

    return found_level, exist_ref
end

function ParameterStorage.add(new_dat, cfg)
    if Global_Reconstruction_Done then return 0, 0, 0 end 

    local_id_counter = local_id_counter + 1
    new_dat.id = tostring(local_id_counter)

    local history_decay_level = 0
    new_dat.count = 1 
    new_dat.decay_lvl1_count = 0
    new_dat.decay_lvl2_count = 0
    new_dat.decay_lvl3_count = 0

    local local_level, local_exist = merge_into_buffer(DataBuffer.stored, new_dat, cfg, true)

    local final_local_count = 1
    if local_level > 0 then
        final_local_count = local_exist.count
        if not local_exist.consumed_ids then local_exist.consumed_ids = { [local_exist.id] = true } end
        local_exist.consumed_ids[new_dat.id] = true
    else
        local new_v = {}
        for _, v in ipairs(new_dat.value) do table.insert(new_v, v) end
        local entry = { 
            value = new_v, type = new_dat.type, count = 1, 
            id = new_dat.id, consumed_ids = { [new_dat.id] = true }, 
            same_size_merges = {}, diff_size_merges = {},
            decay_lvl1_count = 0, decay_lvl2_count = 0, decay_lvl3_count = 0
        }
        table.insert(DataBuffer.stored, entry)
        local_exist = entry
    end

    local global_level = 0
    local global_exist = nil
    if ParameterStorage.USE_GLOBAL_MEMORY then
        global_level, global_exist = merge_into_buffer(DataBuffer.global_stored, new_dat, cfg, false)
    end

    local restore_level = 3
    local restore_count = final_local_count

    if global_level > 0 then
        restore_level = global_level
        restore_count = final_local_count + global_exist.count - 1 

        if global_level == 2 then
            global_exist.decay_lvl2_count = (global_exist.decay_lvl2_count or 0) + 1
            history_decay_level = global_exist.decay_lvl2_count - 1
        else
            global_exist.decay_lvl1_count = (global_exist.decay_lvl1_count or 0) + 1
            history_decay_level = global_exist.decay_lvl1_count - 1
        end

        if ParameterStorage.PS_LOG_MODE >= 1 then
            local lbl = (global_level == 2) and "Iguales" or "Similares"
            log(string.format("[INFO] Fragmento Refinado (Globalmente conocido [%s], total x%d) [NIVEL %d]: %s", lbl, restore_count, restore_level, fmt_struct(local_exist)))
        end

    elseif local_level > 0 then
        restore_level = local_level
        restore_count = final_local_count

        if local_level == 2 then
            local_exist.decay_lvl2_count = (local_exist.decay_lvl2_count or 0) + 1
            history_decay_level = local_exist.decay_lvl2_count - 1
        else
            local_exist.decay_lvl1_count = (local_exist.decay_lvl1_count or 0) + 1
            history_decay_level = local_exist.decay_lvl1_count - 1
        end

        if ParameterStorage.PS_LOG_MODE >= 1 then
            local lbl = (local_level == 2) and "Iguales" or "Similares"
            log(string.format("[INFO] Fragmento Refinado (Promedio Local [%s] de %d) [NIVEL %d]: %s", lbl, restore_count, restore_level, fmt_struct(local_exist)))
        end

    else
        restore_level = 3
        restore_count = final_local_count
        local_exist.decay_lvl3_count = (local_exist.decay_lvl3_count or 0) + 1
        history_decay_level = local_exist.decay_lvl3_count - 1

        if ParameterStorage.PS_LOG_MODE >= 1 then 
            log(string.format("[INFO] Fragmento Añadido (Totalmente Nuevo) [NIVEL 3]: %s", fmt_struct(local_exist))) 
        end
    end

    return restore_level, restore_count, history_decay_level
end

function ParameterStorage.package_and_add(params, valid_segments, cfg)
    if not params or #params == 0 then return 0, 0, 0 end

    local structure_values = {}
    local structure_type = nil 

    for i, param in ipairs(params) do
        local dist, ang, col_id = param[1], param[2], param[3]
        local is_segment_valid = valid_segments[col_id]

        if i > 1 then
            table.insert(structure_values, ang)
            if structure_type == nil then structure_type = 1 end
        end

        if is_segment_valid then
            table.insert(structure_values, dist)
            if structure_type == nil then structure_type = 0 end
        end
    end

    if #structure_values < 3 then
        if ParameterStorage.PS_LOG_MODE >= 1 then log("\n >> Fragmento ignorado: Estructura insuficiente ("..#structure_values.." parametros). Mínimo requerido: 3.") end
        return 0, 0, 0
    end

    if structure_type ~= nil then
        local data_packet = { value = structure_values, type = structure_type }
        if ParameterStorage.PS_LOG_MODE >= 1 then log("\n >> Fragmentos empaquetados y enviados al ParameterStorage. (Total values: "..#structure_values..")") end
        return ParameterStorage.add(data_packet, cfg)
    end
    return 0, 0, 0
end

function ParameterStorage.print_global_buffer(cfg)
    if ParameterStorage.PS_LOG_MODE >= 2 then
        log("\n=========================================")
        log(" TABLA DEL BUFFER GLOBAL (HISTORIAL DE APRENDIZAJE)")
        log("=========================================")
        if #DataBuffer.global_stored == 0 then
            log(" >> [Memoria Global Vacía]")
        else
            for i, v in ipairs(DataBuffer.global_stored) do 
                log(string.format(" [%d] %s", i, fmt_struct(v))) 
            end
        end
        log("=========================================\n")
    end
end

function ParameterStorage.clear(cfg, valid_ids_table)
    if not cfg then cfg = { CONFIG = _G.CONFIG or { TOL_ANG = 0.2, TOL_DIST = 2.0 }, LOG_MODE = 1 } end

    if ParameterStorage.PS_LOG_MODE >= 2 then
        log("\n=========================================")
        log(" [DEPURACIÓN] INICIANDO TRANSFERENCIA A MEMORIA GLOBAL")
    end

    local valid_str = ""
    if valid_ids_table then
        local count = 0
        for k, _ in pairs(valid_ids_table) do 
            valid_str = valid_str .. k .. ", " 
            count = count + 1
        end
        if count == 0 then
            valid_str = "VACÍA (Seguridad: Se aceptarán todos)"
            valid_ids_table = nil
        end
    else
        valid_str = "NIL (Se aceptarán todos)"
    end
    
    if ParameterStorage.PS_LOG_MODE >= 2 then log(" IDs Ganadores recibidos de forma nativa: {" .. valid_str .. "}") end

    if ParameterStorage.USE_GLOBAL_MEMORY then
        for _, local_dat in ipairs(DataBuffer.stored) do
            local local_ids_str = ""
            for c_id, _ in pairs(local_dat.consumed_ids or {}) do local_ids_str = local_ids_str .. c_id .. ", " end
            
            if ParameterStorage.PS_LOG_MODE >= 2 then 
                log(string.format(" Evaluando fragmento local ID [%s] con sub-IDs: {%s}", local_dat.id, local_ids_str))
            end

            local is_valid = false
            local matched_id = ""
            if not valid_ids_table then
                is_valid = true
            else
                for c_id, _ in pairs(local_dat.consumed_ids or {}) do
                    if valid_ids_table[c_id] then
                        is_valid = true
                        matched_id = c_id
                        break
                    end
                end
            end

            if is_valid then
                if ParameterStorage.PS_LOG_MODE >= 2 then log(string.format(" -> Fragmento ACEPTADO (ID ganador referencial: %s).", matched_id)) end
                local found_level, global_exist = merge_into_buffer(DataBuffer.global_stored, local_dat, cfg, true)

                if found_level == 0 then
                    if ParameterStorage.PS_LOG_MODE >= 2 then log("    - Guardado como NUEVO en memoria global.") end
                    local new_v = {}
                    for _, v in ipairs(local_dat.value) do table.insert(new_v, v) end
                    local c_ids = {}
                    for id_k, id_v in pairs(local_dat.consumed_ids or {}) do c_ids[id_k] = id_v end

                    local entry = { 
                        value = new_v, type = local_dat.type, count = local_dat.count, 
                        id = local_dat.id, consumed_ids = c_ids,
                        same_size_merges = {}, diff_size_merges = {},
                        decay_lvl1_count = local_dat.decay_lvl1_count,
                        decay_lvl2_count = local_dat.decay_lvl2_count,
                        decay_lvl3_count = local_dat.decay_lvl3_count
                    }
                    table.insert(DataBuffer.global_stored, entry)
                else
                    if ParameterStorage.PS_LOG_MODE >= 2 then log(string.format("    - Fusionado con éxito (Nivel %d) con fragmento global ID [%s].", found_level, global_exist.id)) end
                    global_exist.decay_lvl1_count = (global_exist.decay_lvl1_count or 0) + (local_dat.decay_lvl1_count or 0)
                    global_exist.decay_lvl2_count = (global_exist.decay_lvl2_count or 0) + (local_dat.decay_lvl2_count or 0)
                    global_exist.decay_lvl3_count = (global_exist.decay_lvl3_count or 0) + (local_dat.decay_lvl3_count or 0)

                    for cid, _ in pairs(local_dat.consumed_ids or {}) do
                        global_exist.consumed_ids[cid] = true
                    end
                end
            else
                if ParameterStorage.PS_LOG_MODE >= 2 then log(" -> Fragmento RECHAZADO (Es ruido descartado).") end
            end
        end
    else
        DataBuffer.global_stored = {}
    end

    ParameterStorage.print_global_buffer(cfg)

    -- Limpieza total para el próximo ciclo
    local_id_counter = 0
    DataBuffer.stored = {}
    Global_Reconstruction_Done = false
end

return ParameterStorage