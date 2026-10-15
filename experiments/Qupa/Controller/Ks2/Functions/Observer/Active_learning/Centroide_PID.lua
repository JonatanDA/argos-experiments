-- ============================================================================
-- VARIABLES GLOBALES PARA CONTROL PID DEL ACTIVE LEARNING
-- ============================================================================

-- [Kp] PROPORCIONAL: Reacción al error presente. 
Kp = 0.8                   

-- [Ki] INTEGRAL: Memoria del error pasado (vence la fricción o errores constantes).
Ki = 0.01                  

-- [Kd] DERIVATIVO: Predicción y frenado (analiza qué tan rápido cambia el error).
Kd = 0.1                   

-- [target_centroid_distance] REFERENCIA (Setpoint).
target_centroid_distance = 0.0  

-- [pid_tolerance] ZONA MUERTA (Deadband).
pid_tolerance = 10.0             

-- [max_pid_output / min_pid_output] LÍMITES DE SATURACIÓN DE LA DISTANCIA (No velocidad).
max_pid_output = 100.0           
min_pid_output = 0.0            

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
        
        local smooth_x, smooth_y = get_smoothed_values(raw_centroid_x, raw_centroid_y)

        local centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        local centroid_angle = centroid_angle_rad * (180 / math.pi)

        local out_pid_dist = 0
        local out_speed_d = 0
        local out_speed_g = 0
        local do_move = false

        -- Limpieza del integrador si el robot principal va a entrar en evasión visual
        if closest_master_dist < front_threshold then
            pid_error_integral = 0
        end

        local error_dist = centroid_dist - target_centroid_distance

        if math.abs(error_dist) > pid_tolerance then
            local p_term = Kp * error_dist
            pid_error_integral = pid_error_integral + error_dist
            local i_term = Ki * pid_error_integral
            local d_term = Kd * (error_dist - pid_last_error)
            pid_last_error = error_dist
            
            out_pid_dist = p_term + i_term + d_term
            
            -- Saturamos la distancia a recorrer (Evita Windup Integrador)
            if out_pid_dist > max_pid_output then out_pid_dist = max_pid_output 
            elseif out_pid_dist < min_pid_output then out_pid_dist = min_pid_output end
            
            -- CÁLCULO DE VELOCIDADES DINÁMICAS
            local global_d = d_speed or 30
            local global_g = g_speed or 80
            
            out_speed_d = math.min(global_d, math.max(5.0, math.abs(out_pid_dist)))
            out_speed_g = math.min(global_g, math.max(10.0, math.abs(centroid_angle)))
            
            do_move = true
        else
            pid_error_integral = 0
        end

        -- LOGS Y TRACKER
        if Print_tracker_AL_PID then 
            Print_tracker_AL_PID(target_centroid_distance, centroid_dist, {
                out_pid_dist = out_pid_dist,
                out_speed_d = out_speed_d,
                out_speed_g = out_speed_g
            }) 
        end

        -- >>> ENVÍO DE DATOS AL TRACKER PYTHON DE TRAYECTORIA <<<
        if Print_tracker_trayectory_data then
            local ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            local cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            
            -- Enviamos el Modo 3 junto con cgx, cgy, Y LAS 3 VARIABLES EXTRAS para python
            Print_tracker_trayectory_data("Active_learning", 3, "Qupa4", 0, 0, cgx/100, cgy/100, out_pid_dist, out_speed_d, out_speed_g)
        end

        return {
            should_move   = do_move,
            target_dist   = out_pid_dist,
            target_angle  = centroid_angle,
            speed_d       = out_speed_d,
            speed_g       = out_speed_g,
            closest_dist  = closest_master_dist,
            closest_angle = angle_of_closest,
            state_name    = "PERSIGUIENDO_CENTROIDE",
            state_msg     = "PID"
        }
    else
        history_x = {}
        history_y = {}
        return nil
    end
end

return ActiveLearning