local ActiveLearning = {}
local history_x = {}
local history_y = {}

local function get_smoothed_values(new_x, new_y)
    local max_ticks = AL_SMOOTH_TICKS or 1
    local method = AL_SMOOTH_METHOD or "median"

    table.insert(history_x, new_x)
    table.insert(history_y, new_y)

    while #history_x > max_ticks do
        table.remove(history_x, 1)
        table.remove(history_y, 1)
    end

    if max_ticks == 1 then return new_x, new_y end

    if method == "mean" then
        local sx, sy = 0, 0
        for i = 1, #history_x do
            sx = sx + history_x[i]
            sy = sy + history_y[i]
        end
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
    local sum_fx, sum_fy = 0, 0
    local closest_master_dist = 999999 
    local angle_of_closest = 0

    local VIRTUAL_FORCE_EQUILIBRIUM = 25.0
    local K_ATTRACT = 1.0                   
    local K_REPEL = 2.5                     
    local MIN_FORCE_THRESHOLD = max_tracking_distance 

    for k, data in pairs(srt_anterior_data) do
        if updated_anterior_indices[k] and data.angle and data.distance then
            active_count = active_count + 1
            
            local d = data.distance
            local ang_rad = data.angle * (math.pi/180)
            
            local force_mag = 0
            if d >= VIRTUAL_FORCE_EQUILIBRIUM then
                force_mag = K_ATTRACT * (d - VIRTUAL_FORCE_EQUILIBRIUM)
            else
                force_mag = K_REPEL * (d - VIRTUAL_FORCE_EQUILIBRIUM)
            end

            local fx = force_mag * math.cos(ang_rad)
            local fy = force_mag * math.sin(ang_rad)
            
            sum_fx = sum_fx + fx
            sum_fy = sum_fy + fy

            if d < closest_master_dist then
                closest_master_dist = d
                angle_of_closest = data.angle
            end
        end
    end

    if active_count > 0 and ENABLE_ACTIVE_LEARNING then
        local smooth_fx, smooth_fy = get_smoothed_values(sum_fx, sum_fy)
        
        local resultant_force, resultant_angle_rad = math_helper.cartesianoAPolarPAR(smooth_fx, smooth_fy)
        local resultant_angle = resultant_angle_rad * (180 / math.pi)

        if Print_tracker_trayectory_data then
            local ang_total_res = resultant_angle_rad + obs_ang_offset_global
            local res_gx, res_gy = math_helper.polarACartesianoPAR(resultant_force, ang_total_res)
            Print_tracker_trayectory_data("Active_learning", 4, "Qupa4", 0, 0, res_gx/100, res_gy/100)
        end

        if AL_LOG_MODE == 2 then
            log(string.format("[ACTIVE LEARNING] FuerzaRes(%.1f) | Ang: %.1f° | Activos: %d | Cerca: %.1f cm", resultant_force, resultant_angle, active_count, closest_master_dist))
        end

        local move_dist = resultant_force 
        if move_dist > (max_tracking_distance * 2) then move_dist = (max_tracking_distance * 2) end
        
        -- CÁLCULO DE VELOCIDADES DINÁMICAS (SATURADAS)
        local global_d = d_speed or 30
        local global_g = g_speed or 80
        
        local dyn_speed_d = math.min(global_d, math.max(5.0, math.abs(move_dist)))
        local dyn_speed_g = math.min(global_g, math.max(10.0, math.abs(resultant_angle)))
        
        local msg_perseguir = string.format("SIGUIENDO FUERZAS | Magnitud: %.1f | Ángulo: %.1f°", resultant_force, resultant_angle)
        
        return {
            should_move   = (resultant_force > MIN_FORCE_THRESHOLD),
            target_dist   = move_dist,
            target_angle  = resultant_angle,
            speed_d       = dyn_speed_d,
            speed_g       = dyn_speed_g,
            closest_dist  = closest_master_dist,
            closest_angle = angle_of_closest,
            state_name    = "SIGUIENDO_FUERZAS",
            state_msg     = msg_perseguir
        }
    else
        history_x = {}
        history_y = {}
        return nil
    end
end

return ActiveLearning