-- Archivo: Functions/Observer/Parameter_Storage.lua

local ParameterStorage = {}

local DataBuffer = { stored = {} }
local Global_Reconstruction_Done = false 

function ParameterStorage.is_done()
    return Global_Reconstruction_Done
end

function ParameterStorage.set_done()
    Global_Reconstruction_Done = true
end

function ParameterStorage.get_stored_data()
    return DataBuffer.stored
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

local function check_match(long_dat, short_dat, start_idx, cfg)
    if get_val_type(long_dat.type, start_idx) ~= get_val_type(short_dat.type, 1) then return false end
    for k = 1, #short_dat.value do
        local l_idx = start_idx + k - 1
        local v_type = get_val_type(short_dat.type, k)
        local diff = math.abs(long_dat.value[l_idx] - short_dat.value[k])
        local tol = (v_type == "A") and cfg.CONFIG.TOL_ANG or cfg.CONFIG.TOL_DIST
        if diff > tol then return false end
    end
    return true
end

function ParameterStorage.add(new_dat, cfg)
    if Global_Reconstruction_Done then return nil, 0 end 
    local found = false
    local restore_type = nil
    local restore_count = 0

    for i, exist in ipairs(DataBuffer.stored) do
        if #exist.value == #new_dat.value and exist.type == new_dat.type then
            if check_match(exist, new_dat, 1, cfg) then
                found = true
                exist.count = (exist.count or 1) + 1
                for k = 1, #exist.value do 
                    exist.value[k] = exist.value[k] + (new_dat.value[k] - exist.value[k]) / exist.count 
                end
                table.insert(exist.same_size_merges, simple_fmt(new_dat.value))
                if cfg.LOG_MODE == 1 then 
                    log(string.format("[INFO] Dato Refinado (Promedio de %d): %s", exist.count, fmt_struct(exist)))
                end
                restore_type, restore_count = "partial", exist.count
                break
            end
        end

        if not found and #new_dat.value >= #exist.value + 2 then
            for start_idx = 2, #new_dat.value - #exist.value do
                if check_match(new_dat, exist, start_idx, cfg) then
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
                    if cfg.LOG_MODE == 1 then
                        log(string.format("[INFO] Dato Actualizado (Extendido a %d val, solape en medio): %s", #exist.value, fmt_struct(exist)))
                    end
                    restore_type, restore_count = "partial", exist.count
                    break
                end
            end
            if found then break end
        end

        if not found and #exist.value >= #new_dat.value + 2 then
            for start_idx = 2, #exist.value - #new_dat.value do
                if check_match(exist, new_dat, start_idx, cfg) then
                    found = true
                    local divisor = (exist.count or 1) + 1
                    for k = 1, #new_dat.value do
                        local e_idx = start_idx + k - 1
                        exist.value[e_idx] = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / divisor
                    end
                    table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    if cfg.LOG_MODE == 1 then
                        log(string.format("[INFO] Dato Actualizado (Subcadena promediada - Frec INTACTA %d): %s", exist.count, fmt_struct(exist)))
                    end
                    restore_type, restore_count = "partial", exist.count
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
        restore_type, restore_count = "full", 1
    end

    return restore_type, restore_count
end

-- ============================================================================
-- NUEVA FUNCIÓN EXPORTADA: Empaqueta los parámetros y los almacena
-- ============================================================================
function ParameterStorage.package_and_add(params, valid_segments, cfg)
    if not params or #params == 0 then return nil, 0 end

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

    if #structure_values > 0 and structure_type ~= nil then
        local data_packet = { value = structure_values, type = structure_type }
        
        if cfg.LOG_MODE == 1 then 
            log(">> Datos empaquetados y enviados al ParameterStorage. (Total values: "..#structure_values..")") 
        end
        
        -- Llamamos a la función de guardado pesado interna
        return ParameterStorage.add(data_packet, cfg)
    end
    
    return nil, 0
end

return ParameterStorage