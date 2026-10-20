-- ============================================================================
-- VARIABLES DEL MÓDULO: CENTROIDE PID
-- ============================================================================
-- AL_LOG_MODE              = 2
AL_SMOOTH_TICKS          = 3
AL_SMOOTH_METHOD         = "median"

Kp = 0.8                   
Ki = 0.01                  
Kd = 0.1                   
target_centroid_distance = 0.0   
pid_tolerance            = 10.0  
max_pid_output           = 100.0 
min_pid_output           = 0.0   
PAUSE_TICKS_ON_CHANGE    = 1     

-- Umbrales de Supervivencia
FRONT_THRESHOLD          = 25.0
BACK_THRESHOLD           = 10.0
PROX_CRITICAL_THRESHOLD  = 0.90

pid_error_integral = 0          
pid_last_error = 0         

local ActiveLearning = {}
local history_x, history_y = {}, {}
local last_do_move_state = false

local function get_smoothed_values(new_x, new_y)
    table.insert(history_x, new_x); table.insert(history_y, new_y)
    while #history_x > AL_SMOOTH_TICKS do table.remove(history_x, 1); table.remove(history_y, 1) end
    if AL_SMOOTH_TICKS == 1 then return new_x, new_y end
    if AL_SMOOTH_METHOD == "mean" then
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
    local closest_master_dist = 999999; local angle_of_closest = 0

    for k, data in pairs(srt_anterior_data) do
        if updated_anterior_indices[k] and data.angle and data.distance then
            local mx, my = math_helper.polarACartesianoPAR(data.distance, data.angle * (math.pi/180))
            sum_x = sum_x + mx; sum_y = sum_y + my
            active_count = active_count + 1
            if data.distance < closest_master_dist then closest_master_dist = data.distance; angle_of_closest = data.angle end
        end
    end

    if active_count > 0 and ENABLE_ACTIVE_LEARNING then
        if active_count == 1 then active_count = 2 end

        local smooth_x, smooth_y = get_smoothed_values(sum_x / active_count, sum_y / active_count)
        local centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        local centroid_angle = centroid_angle_rad * (180 / math.pi)

        local out_pid_dist, out_speed_d, out_speed_g = 0, 0, 0
        local do_move = false

        if closest_master_dist < FRONT_THRESHOLD then pid_error_integral = 0 end

        local error_dist = centroid_dist - target_centroid_distance

        if math.abs(error_dist) > pid_tolerance then
            local p_term = Kp * error_dist
            pid_error_integral = pid_error_integral + error_dist
            local i_term = Ki * pid_error_integral
            local d_term = Kd * (error_dist - pid_last_error)
            pid_last_error = error_dist
            
            out_pid_dist = p_term + i_term + d_term
            if out_pid_dist > max_pid_output then out_pid_dist = max_pid_output 
            elseif out_pid_dist < min_pid_output then out_pid_dist = min_pid_output end
            
            local global_d = d_speed or 30; local global_g = g_speed or 80
            out_speed_d = math.min(global_d, math.max(5.0, math.abs(out_pid_dist)))
            out_speed_g = math.min(global_g, math.max(10.0, math.abs(centroid_angle)))
            do_move = true
        else
            pid_error_integral = 0
        end

        local trigger_pause = false
        if do_move and not last_do_move_state then trigger_pause = true end
        last_do_move_state = do_move

        if Print_tracker_AL_PID then Print_tracker_AL_PID(target_centroid_distance, centroid_dist, {out_pid_dist=out_pid_dist, out_speed_d=out_speed_d, out_speed_g=out_speed_g}) end
        if Print_tracker_trayectory_data then
            local ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            local cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            Print_tracker_trayectory_data("Active_learning", 3, "Qupa4", 0, 0, cgx/100, cgy/100, out_pid_dist, out_speed_d, out_speed_g)
        end
        if AL_LOG_MODE == 2 then log(string.format("[ACTIVE LEARNING] PID | Dist: %.1f cm", centroid_dist)) end

        return {
            should_move         = do_move,
            pause_ticks         = trigger_pause and PAUSE_TICKS_ON_CHANGE or 0,
            target_x            = smooth_x, target_y = smooth_y,
            target_dist         = out_pid_dist, target_angle = centroid_angle,
            speed_d             = out_speed_d, speed_g = out_speed_g,
            closest_dist        = closest_master_dist, closest_angle = angle_of_closest,
            front_threshold     = FRONT_THRESHOLD, back_threshold = BACK_THRESHOLD, prox_threshold = PROX_CRITICAL_THRESHOLD,
            state_name          = "PERSIGUIENDO_CENTROIDE",
            state_msg           = "PID ACTIVO",
            is_centroid         = true
        }
    else
        history_x, history_y = {}, {}
        return nil
    end
end

return ActiveLearning