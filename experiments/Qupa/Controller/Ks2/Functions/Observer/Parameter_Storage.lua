-- ============================================================================
-- Archivo: Functions/Observer/Parameter_Storage.lua
-- ============================================================================

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
    if Global_Reconstruction_Done then return 0, 0 end 
    local found = false
    local restore_level = 0
    local restore_count = 0

    for i, exist in ipairs(DataBuffer.stored) do
        -- Caso: Coincidencia del mismo tamaño
        if #exist.value == #new_dat.value and exist.type == new_dat.type then
            if check_match(exist, new_dat, 1, cfg) then
                found = true
                exist.count = (exist.count or 1) + 1
                for k = 1, #exist.value do 
                    exist.value[k] = exist.value[k] + (new_dat.value[k] - exist.value[k]) / exist.count 
                end
                table.insert(exist.same_size_merges, simple_fmt(new_dat.value))
                if cfg.LOG_MODE == 1 then 
                    log(string.format("[INFO] Dato Refinado (Promedio de %d) [NIVEL 1]: %s", exist.count, fmt_struct(exist)))
                end
                restore_level, restore_count = 1, exist.count
                break
            end
        end

        -- Caso: El nuevo dato es más grande e incluye al existente
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
                        log(string.format("[INFO] Dato Actualizado (Extendido a %d val, solape) [NIVEL 1]: %s", #exist.value, fmt_struct(exist)))
                    end
                    restore_level, restore_count = 1, exist.count
                    break
                end
            end
            if found then break end
        end

        -- Caso: El nuevo dato es más pequeño y está contenido en el existente
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
                        log(string.format("[INFO] Dato Actualizado (Subcadena promediada) [NIVEL 1]: %s", fmt_struct(exist)))
                    end
                    restore_level, restore_count = 1, exist.count
                    break
                end
            end
            if found then break end
        end
    end

    -- Caso: No se encontró una coincidencia exacta de integración, se añade como nuevo
    if not found then
        local has_overlap = false
        
        -- Buscamos si existe al menos un elemento de solapamiento para decidir si es Nivel 2 o 3
        for _, exist in ipairs(DataBuffer.stored) do
            for i = 1, #new_dat.value do
                local type_new = get_val_type(new_dat.type, i)
                for j = 1, #exist.value do
                    local type_exist = get_val_type(exist.type, j)
                    if type_new == type_exist then
                        local diff = math.abs(new_dat.value[i] - exist.value[j])
                        local tol = (type_new == "A") and cfg.CONFIG.TOL_ANG or cfg.CONFIG.TOL_DIST
                        if diff <= tol then
                            has_overlap = true
                            break
                        end
                    end
                end
                if has_overlap then break end
            end
            if has_overlap then break end
        end

        local new_v = {}
        for _, v in ipairs(new_dat.value) do table.insert(new_v, v) end
        local entry = { value = new_v, type = new_dat.type, count = 1, same_size_merges = {}, diff_size_merges = {} }
        table.insert(DataBuffer.stored, entry)
        
        if has_overlap then
            restore_level = 2
            if cfg.LOG_MODE == 1 then
                log(string.format("[INFO] Dato Añadido (Solapamiento Parcial) [NIVEL 2]: %s", fmt_struct(entry)))
            end
        else
            restore_level = 3
            if cfg.LOG_MODE == 1 then
                log(string.format("[INFO] Dato Añadido (Totalmente Nuevo) [NIVEL 3]: %s", fmt_struct(entry)))
            end
        end
        restore_count = 1
    end

    return restore_level, restore_count
end

-- ============================================================================
-- NUEVA FUNCIÓN EXPORTADA: Empaqueta los parámetros y los almacena
-- ============================================================================
function ParameterStorage.package_and_add(params, valid_segments, cfg)
    if not params or #params == 0 then return 0, 0 end

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
    
    -- RESTRICCIÓN: Ignorar si la cadena generada tiene menos de 3 parámetros
    if #structure_values < 3 then
        if cfg.LOG_MODE == 1 then 
            log(">> Dato ignorado: Estructura insuficiente ("..#structure_values.." parametros). Mínimo requerido: 3.") 
        end
        return 0, 0
    end

    if structure_type ~= nil then
        local data_packet = { value = structure_values, type = structure_type }
        
        if cfg.LOG_MODE == 1 then 
            log(">> Datos empaquetados y enviados al ParameterStorage. (Total values: "..#structure_values..")") 
        end
        
        -- Llamamos a la función de guardado pesado interna
        return ParameterStorage.add(data_packet, cfg)
    end
    
    return 0, 0
end

return ParameterStorage