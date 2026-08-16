-- Archivo: Functions/Observer/noise_manager.lua

local NoiseManager = {}

-- ============================================================================
-- FUNCIÓN PRIVADA: Generador de Distribución Normal (Algoritmo Box-Muller)
-- Transforma dos números uniformes (math.random) en uno gaussiano
-- ============================================================================
local function gaussian(mean, desviacion_estandar)
    local u1 = math.random()
    local u2 = math.random()
    -- Seguridad para evitar log(0)
    if u1 <= 0 then u1 = 0.000001 end
    
    local z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return z0 * desviacion_estandar + mean
end

-- ============================================================================
-- FUNCIÓN EXPORTADA: Aplica las perturbaciones configuradas a la lectura cruda
-- ============================================================================
function NoiseManager.aplicar_ruido(dist_real, angulo_real, cfg)
    -- 1. Si el ruido está apagado globalmente, devolver valores reales
    if not cfg.ENABLE_NOISE then 
        return dist_real, angulo_real 
    end

    -- 2. OUTLIERS: Fallos catastróficos (Salt & Pepper)
    if cfg.USE_OUTLIERS then
        if math.random() < cfg.OUTLIER_PROB then
            -- El sensor "alucina": devuelve una distancia enorme y ángulo random
            return 1000.0, math.random(-180, 180) 
        end
    end

    local dist_final = dist_real
    local angulo_final = angulo_real

    -- 3. RUIDO EN DISTANCIA
    if cfg.USE_DISTANCE_DEPENDENT then
        -- La desviación estándar (sigma) crece con la distancia
        local desviacion_estandar = 0.0 + (dist_real * cfg.NOISE_LEVEL_DIST)
        local error = gaussian(0, desviacion_estandar)
        dist_final = dist_final + error
        
        -- Corrección física: la distancia no puede ser negativa
        if dist_final < 0 then dist_final = 0 end
    else
        -- Ruido fijo simple por defecto
        dist_final = dist_final + gaussian(0, 0.5) 
    end

    -- 4. RUIDO EN ÁNGULO
    if cfg.USE_ANGLE_NOISE then
        -- Ruido gaussiano con media 0 y desviación configurada
        local error_angular = gaussian(0, cfg.NOISE_LEVEL_ANGLE)
        angulo_final = angulo_final + error_angular
    end

    -- 5. BIAS (Sesgo sistemático)
    if cfg.USE_BIAS then
        dist_final = dist_final + cfg.BIAS_OFFSET
    end

    return dist_final, angulo_final
end

return NoiseManager