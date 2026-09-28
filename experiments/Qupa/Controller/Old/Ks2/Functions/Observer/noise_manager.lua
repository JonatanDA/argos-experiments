-- Archivo: Functions/Observer/noise_manager.lua

local NoiseManager = {}

-- ============================================================================
-- FUNCIÓN PRIVADA: Generador de Distribución Normal (Algoritmo Box-Muller)
-- ============================================================================
local function gaussian(mean, desviacion_estandar)
    local u1 = math.random()
    local u2 = math.random()
    if u1 <= 0 then u1 = 0.000001 end
    
    local z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return z0 * desviacion_estandar + mean
end

-- ============================================================================
-- FUNCIÓN EXPORTADA: Aplica las perturbaciones configuradas a la lectura cruda
-- ============================================================================
function NoiseManager.aplicar_ruido(dist_real, angulo_real, cfg)
    -- 1. Si el ruido está apagado globalmente, devolver valores reales (Ground Truth)
    if not cfg.ENABLE_NOISE then 
        return dist_real, angulo_real 
    end

    -- 2. OUTLIERS (Fallos catastróficos aleatorios)
    if cfg.USE_OUTLIERS then
        if math.random() < cfg.OUTLIER_PROB then
            return 1000.0, math.random(-180, 180) 
        end
    end

    local dist_final = dist_real
    local angulo_final = angulo_real

    -- 3. RUIDO ESPACIAL 2D
    -- Partimos del ruido constante mínimo del sensor
    local radio_actual = cfg.NOISE_BASE_RADIUS
    
    -- Si está activada la dependencia, el área de error crece con la distancia
    if cfg.USE_DISTANCE_DEPENDENT then
        radio_actual = radio_actual + (dist_real * cfg.NOISE_FACTOR)
    end
    
    -- Obtener la desviación estándar a partir de las variaciones definidas
    local sigma = radio_actual / cfg.NOISE_SIGMAS
    
    -- Convertir a radianes para matemáticas cartesianas
    local ang_rad = math.rad(angulo_real)
    
    -- Proyectar la lectura polar a (X, Y)
    local x_real = dist_real * math.cos(ang_rad)
    local y_real = dist_real * math.sin(ang_rad)
    
    -- Aplicar la campana de Gauss en ambos ejes con la sigma dinámica o estática
    local x_ruido = x_real + gaussian(0, sigma)
    local y_ruido = y_real + gaussian(0, sigma)
    
    -- Volver a coordenadas polares
    dist_final = math.sqrt(x_ruido^2 + y_ruido^2)
    angulo_final = math.deg(math.atan2(y_ruido, x_ruido))

    -- 4. BIAS (Sesgo sistemático constante del sensor)
    if cfg.USE_BIAS then
        dist_final = dist_final + cfg.BIAS_OFFSET
    end
    
    -- Corrección física final: la distancia no puede ser negativa
    if dist_final < 0 then 
        dist_final = 0 
    end

    return dist_final, angulo_final
end

return NoiseManager