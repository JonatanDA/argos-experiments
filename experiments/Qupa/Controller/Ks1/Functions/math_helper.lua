-- Archivo: Math_Functions/math_helper.lua

-- 1. Creamos la tabla que contendrá todas las funciones
local math_helper = {}

---------------------------------------------------------
-- ESTADÍSTICAS
---------------------------------------------------------
function math_helper.get_window_stats(start_idx, win_size, xx, yy)
    local sum_x, sum_y = 0, 0
    local n = 0
    
    -- 1. Calcular Centroide (Promedio)
    for j = start_idx, start_idx + win_size - 1 do
        if xx[j] and yy[j] then
            sum_x = sum_x + xx[j]
            sum_y = sum_y + yy[j]
            n = n + 1
        end
    end
    
    if n == 0 then return 0, 0, 0 end
    local avg_x = sum_x / n
    local avg_y = sum_y / n
    
    -- 2. Calcular Dispersión (Desviación Estándar combinada)
    local sum_sq_dist = 0
    for j = start_idx, start_idx + win_size - 1 do
        if xx[j] and yy[j] then
            local dx = xx[j] - avg_x
            local dy = yy[j] - avg_y
            sum_sq_dist = sum_sq_dist + (dx*dx + dy*dy)
        end
    end
    
    -- Retornamos: Desviación Estándar, Centroide X, Centroide Y
    return math.sqrt(sum_sq_dist / n), avg_x, avg_y
end

-- Función para encontrar el umbral óptimo usando el Método de Otsu 1D
function math_helper.otsu_1d(varianzas)
    if #varianzas == 0 then return 0, 0 end
    
    -- Encontrar min y max para crear el histograma
    local min_v, max_v = varianzas[1], varianzas[1]
    for i = 1, #varianzas do
        if varianzas[i] < min_v then min_v = varianzas[i] end
        if varianzas[i] > max_v then max_v = varianzas[i] end
    end
    
    if min_v == max_v then return min_v, 0 end
    
    -- Crear histograma aproximado (50 bins)
    local num_bins = 50
    local hist = {}
    for i = 1, num_bins do hist[i] = 0 end
    
    local bin_size = (max_v - min_v) / num_bins
    for i = 1, #varianzas do
        local bin = math.floor((varianzas[i] - min_v) / bin_size) + 1
        if bin > num_bins then bin = num_bins end
        hist[bin] = hist[bin] + 1
    end
    
    local total_datos = #varianzas
    local sum_total = 0
    for i = 1, num_bins do
        local bin_val = min_v + (i - 0.5) * bin_size
        sum_total = sum_total + (bin_val * hist[i])
    end
    
    local weight_bg = 0
    local sum_bg = 0
    local max_varianza_interclase = -1
    local umbral_optimo = 0
    
    -- Evaluar cada posible umbral (Otsu)
    for i = 1, num_bins do
        weight_bg = weight_bg + hist[i]
        
        if weight_bg > 0 then
            local weight_fg = total_datos - weight_bg
            if weight_fg == 0 then break end
            
            local bin_val = min_v + (i - 0.5) * bin_size
            sum_bg = sum_bg + (bin_val * hist[i])
            
            local mean_bg = sum_bg / weight_bg
            local mean_fg = (sum_total - sum_bg) / weight_fg
            
            -- Varianza Inter-clase
            local var_between = weight_bg * weight_fg * (mean_bg - mean_fg) * (mean_bg - mean_fg)
            
            if var_between > max_varianza_interclase then
                max_varianza_interclase = var_between
                umbral_optimo = bin_val
            end
        end
    end
    
    return umbral_optimo, max_varianza_interclase
end

---------------------------------------------------------
-- OPERACIONES Y CÁLCULOS
---------------------------------------------------------
function math_helper.polarACartesianoPAR(r, theta)
    local y = r * math.sin(theta)
    local x = r * math.cos(theta)
    return x, y
end

function math_helper.cartesianoAPolarPAR(x, y)
    local r = math.sqrt(x^2 + y^2)
    local theta = math.atan2(y, x)
    return r, theta
end

function math_helper.vec2_angle(v)
    local ang = math.atan2(2 * (v.w * v.z + v.x * v.y), 1 - 2 * (v.y ^ 2 + v.z ^ 2))
    ang = ang * 180 / math.pi
    return ang
end


-- 2. Retornamos la tabla completa
return math_helper