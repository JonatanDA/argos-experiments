-- Archivo: Functions/Observer/Segment_Parameterizer.lua

local GeometryBuilder = {}

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

    for _, col in ipairs(ordered_keys) do
        local points = segmentos[col]

        if #points > 3 then
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

            equations[col] = {m = m, b = b, is_vertical = is_vertical}
            table.insert(valid_cols, col) 
        end
    end
    
    return equations, valid_cols
end

-- ============================================================================
-- 2. CALCULAR INTERSECCIONES: Cruza las rectas calculadas
-- ============================================================================
local function calcular_intersecciones_con_extremos(equations, segmentos, valid_cols)
    local intersections = {} 

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

    return intersections
end

-- ============================================================================
-- 3. CALCULAR DISTANCIAS Y ÁNGULOS
-- ============================================================================
local function calcular_distancias_y_angulos(intersecciones)
    local distancias_a_recorrer = {} 
    local angles_a_recorrer = {}
    
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
        prev_angle = curr_angle
    end

    return distancias_a_recorrer, angles_a_recorrer
end

-- ============================================================================
-- 4. CREATE PARAMS: Empaqueta los datos con su ID Original
-- ============================================================================
local function create_params(distancias, angulos, valid_cols)
    local params = {}  
    
    for i = 1, math.min(#distancias, #angulos) do
        local distancia = distancias[i]
        local angulo    = angulos[i]
        local original_col_id = valid_cols[i] 
        table.insert(params, { distancia, angulo, original_col_id })
    end

    -- for idx, seg in ipairs(params) do
    --     log(string.format(
    --         "Segmento %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°",
    --         idx, seg[1], seg[2]
    --     ))
    -- end

    return params
end

-- ============================================================================
-- FUNCIÓN PRINCIPAL EXPORTADA: Ejecuta toda la cadena geométrica
-- ============================================================================
function GeometryBuilder.process_segments(segmentos_movement)
    local equations, valid_cols = apply(segmentos_movement)
    local intersections = calcular_intersecciones_con_extremos(equations, segmentos_movement, valid_cols)
    local distancias, angulos = calcular_distancias_y_angulos(intersections)
    local params = create_params(distancias, angulos, valid_cols)
    
    return params
end

return GeometryBuilder