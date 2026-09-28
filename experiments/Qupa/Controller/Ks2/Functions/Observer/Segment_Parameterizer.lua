-- Archivo: Functions/Observer/Segment_Parameterizer.lua

local SegmentParameterizer = {}

-- ============================================================================
-- VARIABLES DE CONTROL (LOGGEO Y AJUSTES)
-- ============================================================================
local LOG_APPLY        = 1 -- 1 para ver las regresiones iniciales
local LOG_MERGE        = 1 -- 1 para ver las fusiones de segmentos similares
local LOG_INTERSECT    = 1 -- 1 para ver las intersecciones calculadas
local LOG_DIST_ANGLES  = 1 -- 1 para ver distancias y ángulos relativos
local LOG_PARAMS       = 1 -- 1 para ver el resultado final empaquetado

-- Tolerancia en grados. Si dos rectas consecutivas tienen un ángulo
-- con una diferencia menor o igual a este valor, se fusionan.
local TOLERANCIA_ANGULO_FUSION = 15.0 

-- Helper para asegurar que el print funcione en ARGoS (log) o Lua estándar (print)
local function sys_log(msg)
    if log then log(msg) else print(msg) end
end

-- ============================================================================
-- HELPER: Calcula la regresión lineal para un conjunto de puntos
-- ============================================================================
local function compute_regression(points)
    local sum_x, sum_y = 0, 0
    local n = #points

    for _, point in ipairs(points) do
        sum_x = sum_x + point[1]
        sum_y = sum_y + point[2]
    end
    local mean_x = sum_x / n
    local mean_y = sum_y / n

    local s_xx, s_yy, s_xy = 0, 0, 0
    for _, point in ipairs(points) do
        local dx = point[1] - mean_x
        local dy = point[2] - mean_y
        s_xx = s_xx + (dx * dx)
        s_yy = s_yy + (dy * dy)
        s_xy = s_xy + (dx * dy)
    end

    local m, b
    local is_vertical = false

    if s_yy > s_xx then
        is_vertical = true
        if s_yy == 0 then m = 0 else m = s_xy / s_yy end
        b = mean_x - (m * mean_y)
    else
        is_vertical = false
        if s_xx == 0 then m = 0 else m = s_xy / s_xx end
        b = mean_y - (m * mean_x)
    end

    return {m = m, b = b, is_vertical = is_vertical, points_count = n}
end

-- ============================================================================
-- HELPER: Obtiene el ángulo de la recta en grados [0, 180)
-- ============================================================================
local function get_line_angle(eq)
    local angle
    if eq.is_vertical then
        angle = math.deg(math.atan2(1, eq.m))
    else
        angle = math.deg(math.atan2(eq.m, 1))
    end
    
    if angle < 0 then angle = angle + 180 end
    if angle >= 180 then angle = angle - 180 end
    
    return angle
end

-- ============================================================================
-- 1. APPLY: Calcula regresiones ortogonales dinámicas (Normal o Invertida)
-- ============================================================================
local function apply(segmentos)
    local equations = {}
    local valid_cols = {} 

    local ordered_keys = {}
    for k in pairs(segmentos) do
        table.insert(ordered_keys, k)
    end
    table.sort(ordered_keys)

    if LOG_APPLY == 1 then sys_log("\n[APPLY] Iniciando cálculo de regresiones...") end

    for _, col in ipairs(ordered_keys) do
        local points = segmentos[col]
        if #points > 3 then
            equations[col] = compute_regression(points)
            table.insert(valid_cols, col) 

            if LOG_APPLY == 1 then
                local eq = equations[col]
                sys_log(string.format("  - Seg %s: %d puntos | Vert: %s | m: %.3f | b: %.3f | Ángulo: %.2f°",
                    tostring(col), eq.points_count, tostring(eq.is_vertical), eq.m, eq.b, get_line_angle(eq)))
            end
        end
    end
    
    return equations, valid_cols
end

-- ============================================================================
-- 1.5. MERGE: Fusiona rectas consecutivas con pendientes (ángulos) similares
-- ============================================================================
local function merge_collinear_segments(equations, segmentos, valid_cols)
    if LOG_MERGE == 1 then sys_log(string.format("\n[MERGE] Evaluando fusión (Tolerancia: %.1f°)...", TOLERANCIA_ANGULO_FUSION)) end
    
    if #valid_cols < 2 then return equations, valid_cols end

    local i = 1
    while i < #valid_cols do
        local col1 = valid_cols[i]
        local col2 = valid_cols[i+1]
        local eq1 = equations[col1]
        local eq2 = equations[col2]

        local angle1 = get_line_angle(eq1)
        local angle2 = get_line_angle(eq2)

        local diff = math.abs(angle1 - angle2)
        if diff > 90 then diff = 180 - diff end 

        if diff <= TOLERANCIA_ANGULO_FUSION then
            if LOG_MERGE == 1 then
                sys_log(string.format("  -> Fusionando Seg %s y Seg %s (Diferencia: %.2f°)", 
                    tostring(col1), tostring(col2), diff))
            end

            for _, p in ipairs(segmentos[col2]) do
                table.insert(segmentos[col1], p)
            end

            equations[col1] = compute_regression(segmentos[col1])

            table.remove(valid_cols, i + 1)
            equations[col2] = nil
        else
            i = i + 1
        end
    end

    return equations, valid_cols
end

-- ============================================================================
-- 2. CALCULAR INTERSECCIONES: Cruza las rectas calculadas
-- ============================================================================
local function calcular_intersecciones_con_extremos(equations, segmentos, valid_cols)
    local intersections = {} 
    
    if LOG_INTERSECT == 1 then sys_log("\n[INTERSECCIONES] Calculando cruces...") end

    if #valid_cols == 0 then
        if resetOBS then resetOBS() end 
        return intersections
    end

    local first_key = valid_cols[1]
    local first_eq = equations[first_key]
    local first_segment = segmentos[first_key]

    if type(first_eq) ~= "table" or type(first_segment) ~= "table" or #first_segment == 0 then
        if resetalumno then resetalumno() end
        return intersections
    end

    local x_inicio, y_inicio
    if first_eq.is_vertical then
        y_inicio = first_segment[1][2]
        x_inicio = first_eq.m * y_inicio + first_eq.b
    else
        x_inicio = first_segment[1][1]
        y_inicio = first_eq.m * x_inicio + first_eq.b
    end
    table.insert(intersections, {x_inicio, y_inicio})
    if LOG_INTERSECT == 1 then sys_log(string.format("  - Extremo Inicial: (%.3f, %.3f)", x_inicio, y_inicio)) end

    for i = 1, #valid_cols - 1 do
        local eq1 = equations[valid_cols[i]]
        local eq2 = equations[valid_cols[i + 1]]
        local x_int, y_int

        if not eq1.is_vertical and not eq2.is_vertical then
            local den = eq1.m - eq2.m
            if math.abs(den) > 1e-6 then
                x_int = (eq2.b - eq1.b) / den
                y_int = eq1.m * x_int + eq1.b
            end
        elseif eq1.is_vertical and eq2.is_vertical then
            local den = eq1.m - eq2.m
            if math.abs(den) > 1e-6 then
                y_int = (eq2.b - eq1.b) / den
                x_int = eq1.m * y_int + eq1.b
            end
        else
            local eq_norm = eq1.is_vertical and eq2 or eq1
            local eq_vert = eq1.is_vertical and eq1 or eq2
            local den = 1 - (eq_vert.m * eq_norm.m)
            if math.abs(den) > 1e-6 then
                x_int = (eq_vert.m * eq_norm.b + eq_vert.b) / den
                y_int = eq_norm.m * x_int + eq_norm.b
            end
        end

        if x_int and y_int then
            table.insert(intersections, {x_int, y_int})
            if LOG_INTERSECT == 1 then 
                sys_log(string.format("  - Cruce %s y %s: (%.3f, %.3f)", tostring(valid_cols[i]), tostring(valid_cols[i+1]), x_int, y_int)) 
            end
        end
    end

    local last_key = valid_cols[#valid_cols]
    local last_eq = equations[last_key]
    local last_segment = segmentos[last_key]

    local x_final, y_final
    if last_eq.is_vertical then
        y_final = last_segment[#last_segment][2]
        x_final = last_eq.m * y_final + last_eq.b
    else
        x_final = last_segment[#last_segment][1]
        y_final = last_eq.m * x_final + last_eq.b
    end
    table.insert(intersections, {x_final, y_final}) 
    if LOG_INTERSECT == 1 then sys_log(string.format("  - Extremo Final: (%.3f, %.3f)", x_final, y_final)) end

    return intersections
end

-- ============================================================================
-- 3. CALCULAR DISTANCIAS Y ÁNGULOS
-- ============================================================================
local function calcular_distancias_y_angulos(intersecciones)
    local distancias_a_recorrer = {} 
    local angles_a_recorrer = {}
    
    if LOG_DIST_ANGLES == 1 then sys_log("\n[DISTANCIAS Y ÁNGULOS] Extrayendo métricas...") end

    if #intersecciones < 2 then return distancias_a_recorrer, angles_a_recorrer end

    local x1, y1 = intersecciones[1][1], intersecciones[1][2]
    local prev_angle = math.atan2(-y1, -x1) * (180/math.pi)

    for i = 1, #intersecciones - 1 do
        local xa, ya = intersecciones[i][1],   intersecciones[i][2]
        local xb, yb = intersecciones[i+1][1], intersecciones[i+1][2]

        local distancia = math.sqrt((xb - xa)^2 + (yb - ya)^2) 
        table.insert(distancias_a_recorrer, distancia)

        local curr_angle = math.atan2(yb - ya, xb - xa) * (180/math.pi)
        local rel_angle = curr_angle - prev_angle

        if     rel_angle >  180 then rel_angle = rel_angle - 360
        elseif rel_angle < -180 then rel_angle = rel_angle + 360 end

        table.insert(angles_a_recorrer, rel_angle)
        
        if LOG_DIST_ANGLES == 1 then 
            sys_log(string.format("  - Tramo %d: Distancia = %.3f, Ángulo Relativo = %.2f°", i, distancia, rel_angle)) 
        end
        
        prev_angle = curr_angle
    end

    return distancias_a_recorrer, angles_a_recorrer
end

-- ============================================================================
-- 4. CREATE PARAMS: Empaqueta los datos con su ID Original
-- ============================================================================
local function create_params(distancias, angulos, valid_cols, valid_segments)
    local params = {}  
    
    if LOG_PARAMS == 1 then sys_log("\n[PARAMS] Generando paquete final de parámetros:") end

    for i = 1, math.min(#distancias, #angulos) do
        local distancia = distancias[i]
        local angulo    = angulos[i]
        local original_col_id = valid_cols[i] 
        
        -- Empaquetado interno se mantiene intacto
        table.insert(params, { distancia, angulo, original_col_id })
        
        -- Lógica exclusiva para impresión visual limpia
        if LOG_PARAMS == 1 then
            local ang_str = (i == 1) and "nil" or string.format("%.2f°", angulo)
            local dist_str = "nil"
            
            if valid_segments and valid_segments[original_col_id] then
                dist_str = string.format("%.2f cm", distancia)
            end
            
            -- Solo imprimimos si al menos uno de los dos es útil
            if ang_str ~= "nil" or dist_str ~= "nil" then
                sys_log(string.format("  -> Param %d (Col %s) -> Ángulo: %s, Distancia: %s",
                    i, tostring(original_col_id), ang_str, dist_str))
            end
        end
    end

    return params
end

-- ============================================================================
-- FUNCIÓN PRINCIPAL EXPORTADA: Ejecuta toda la cadena geométrica
-- ============================================================================
function SegmentParameterizer.process_segments(segmentos_movement, valid_segments)
    local equations, valid_cols = apply(segmentos_movement)
    equations, valid_cols = merge_collinear_segments(equations, segmentos_movement, valid_cols)
    local intersections = calcular_intersecciones_con_extremos(equations, segmentos_movement, valid_cols)
    local distancias, angulos = calcular_distancias_y_angulos(intersections)
    
    -- Le pasamos la lista de booleanos a create_params
    local params = create_params(distancias, angulos, valid_cols, valid_segments)
    
    if LOG_APPLY  == 1 or LOG_MERGE == 1 or LOG_INTERSECT == 1 or LOG_DIST_ANGLES  == 1 or LOG_PARAMS == 1 then 
        log("============================================\n")
    end

    return params
end

return SegmentParameterizer