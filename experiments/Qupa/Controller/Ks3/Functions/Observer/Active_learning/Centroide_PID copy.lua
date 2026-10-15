-- ============================================================================
-- VARIABLES GLOBALES PARA CONTROL PID DEL ACTIVE LEARNING
-- ============================================================================

-- [Kp] PROPORCIONAL: Reacción al error presente. 
-- * Si lo subes (ej. 1.5): El robot reaccionará más agresivamente, saltando rápido hacia el centroide.
-- * Si lo bajas (ej. 0.3): El robot será perezoso y tardará más en seguir al grupo.
Kp = 0.8                   

-- [Ki] INTEGRAL: Memoria del error pasado (vence la fricción o errores constantes).
-- * Si lo subes (ej. 0.1): Acumulará memoria muy rápido. Si el centroide se mueve lejos, el robot acumulará tanta "frustración" que cuando por fin arranque, saldrá disparado y se pasará de la marca (Windup).
-- * Si lo bajas (ej. 0.001): Será casi ignorado. Úsalo muy bajo (0.01 está bien) solo para ayudar a llegar al centroide si el robot se queda corto por milímetros.
Ki = 0.01                  

-- [Kd] DERIVATIVO: Predicción y frenado (analiza qué tan rápido cambia el error).
-- * Si lo subes (ej. 0.5): El robot amortiguará mucho su llegada, frenando antes de llegar. Excelente para evitar oscilar, pero si la cámara tiene ruido, el robot temblará mucho (el derivativo odia el ruido).
-- * Si lo bajas (ej. 0.0): Quitas el freno. El robot puede sobrepasar el centroide y tener que devolverse.
Kd = 0.1                   

-- [target_centroid_distance] REFERENCIA (Setpoint).
-- * 0.0 significa que el robot quiere estar en el centro exacto de la masa del enjambre.
-- * Si pones 20.0, el robot intentará orbitar a los maestros manteniendo una distancia de 20cm del centro.
target_centroid_distance = 0.0  

-- [pid_tolerance] ZONA MUERTA (Deadband).
-- * Rango en cm donde el PID se apaga. 2.0 significa que si el centroide está a 1.9 cm, el robot no hace nada. 
-- * Evita que el robot vibre infinitamente intentando corregir 1 milímetro.
pid_tolerance = 2.0             

-- [max_pid_output / min_pid_output] LÍMITES DE SATURACIÓN.
-- * Limitan la velocidad máxima y mínima que el algoritmo puede enviarle a las llantas por seguridad.
max_pid_output = 30.0           
min_pid_output = 00.0            

-- Variables internas de memoria del PID (¡No modificar en ejecución!)
pid_error_integral = 0          
pid_last_error = 0
local ActiveLearning = {}

local history_x = {}
local history_y = {}

local function get_smoothed_values(new_x, new_y)
    local max_ticks = AL_SMOOTH_TICKS or 1
    local method = AL_SMOOTH_METHOD or "median"
    table.insert(history_x, new_x)
    table.insert(history_y, new_y)
    while #history_x > max_ticks do table.remove(history_x, 1); table.remove(history_y, 1) end
    if max_ticks == 1 then return new_x, new_y end

    if method == "mean" then
        local sx, sy = 0, 0
        for i = 1, #history_x do sx = sx + history_x[i]; sy = sy + history_y[i] end
        return sx / #history_x, sy / #history_y
    else
        local tx, ty = {}, {}
        for i = 1, #history_x do tx[i] = history_x[i]; ty[i] = history_y[i] end
        table.sort(tx); table.sort(ty)
        local mid = math.floor(#tx / 2) + 1
        if #tx % 2 == 0 then return (tx[mid-1] + tx[mid]) / 2.0, (ty[mid-1] + ty[mid]) / 2.0
        else return tx[mid], ty[mid] end
    end
end

function ActiveLearning.process(obs_ang_offset_global, updated_anterior_indices)
    local active_count = 0
    local sum_x, sum_y = 0, 0
    local closest_master_dist = 999999 
    local angle_of_closest = 0

    for k, data in pairs(srt_anterior_data) do
        if updated_anterior_indices[k] and data.angle and data.distance then
            local mx, my = math_helper.polarACartesianoPAR(data.distance, data.angle * (math.pi/180))
            sum_x = sum_x + mx
            sum_y = sum_y + my
            active_count = active_count + 1

            if data.distance < closest_master_dist then
                closest_master_dist = data.distance
                angle_of_closest = data.angle
            end
        end
    end

    if active_count > 0 and ENABLE_ACTIVE_LEARNING then
        local raw_centroid_x = sum_x / active_count
        local raw_centroid_y = sum_y / active_count
        
        -- Filtro suavizador de ruido visual
        local smooth_x, smooth_y = get_smoothed_values(raw_centroid_x, raw_centroid_y)

        local centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        local centroid_angle = centroid_angle_rad * (180 / math.pi)

        if Print_tracker_trayectory_data then
            local ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            local cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            Print_tracker_trayectory_data("Active_learning", 3, "Qupa4", 0, 0, cgx/100, cgy/100)
        end

        if closest_master_dist < front_threshold and motion_active and al_current_state == "PERSIGUIENDO_CENTROIDE" then
            srt_ang_mark_ant = normalize_angle_rad(move_angle + srt_ang_mark_ant)
            x_ant_r = x_mark
            y_ant_r = y_mark
            motion_active = false
            r_move()  
            robot.wheels.set_velocity(0, 0)
            change_al_state("IDLE", "") 
            pid_error_integral = 0      
        end

        if not motion_active then
            if closest_master_dist < front_threshold then
                motion_active = true
                pid_error_integral = 0  
                
                local diff_ang = centroid_angle - angle_of_closest
                if diff_ang > 180 then diff_ang = diff_ang - 360 end
                if diff_ang < -180 then diff_ang = diff_ang + 360 end
                local safe_angle = 0
                local msg_evasion = ""
                if math.abs(diff_ang) < 60 then 
                    if diff_ang >= 0 then safe_angle = angle_of_closest + 60 
                    else safe_angle = angle_of_closest - 60 end
                    msg_evasion = string.format("EVADIENDO MASTER | Rodeando obstáculo hacia %.1f°", safe_angle)
                else
                    safe_angle = centroid_angle
                    msg_evasion = string.format("EVADIENDO MASTER | Ruta libre hacia %.1f°", safe_angle)
                end
                last_motion_marker = {back_threshold * porcentaje, safe_angle}
                change_al_state("EVADIENDO_MASTER", msg_evasion)
                r_move()
            else
                local error_dist = centroid_dist - target_centroid_distance
                local pid_output = 0

                if math.abs(error_dist) > pid_tolerance then
                    local p_term = Kp * error_dist
                    pid_error_integral = pid_error_integral + error_dist
                    local i_term = Ki * pid_error_integral
                    local d_term = Kd * (error_dist - pid_last_error)
                    pid_last_error = error_dist
                    
                    pid_output = p_term + i_term + d_term
                    if pid_output > max_pid_output then pid_output = max_pid_output 
                    elseif pid_output < min_pid_output then pid_output = min_pid_output end
                    
                    motion_active = true
                    last_motion_marker = {pid_output, centroid_angle}
                    change_al_state("PERSIGUIENDO_CENTROIDE", "PID")
                    r_move()
                else
                    pid_error_integral = 0
                end

                if Print_tracker_AL_PID then Print_tracker_AL_PID(target_centroid_distance, centroid_dist, pid_output) end
            end
        end
    else
        history_x = {}
        history_y = {}
    end
end

return ActiveLearning