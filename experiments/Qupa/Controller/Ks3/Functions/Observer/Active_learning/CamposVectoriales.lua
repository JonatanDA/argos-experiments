-- ============================================================================
-- VARIABLES DEL MÓDULO: CAMPOS VECTORIALES (FUERZAS FÍSICAS)
-- ============================================================================
-- AL_LOG_MODE              = 3
AL_SMOOTH_TICKS          = 3
AL_SMOOTH_METHOD         = "median"

local M_MAX_DIST         = 140.0  
local PUSH_LIMIT         = 30.0  
local NEUTRAL_ZONE       = 5.0   
local N_CRIT_DIST        = 15.0  
local K_FORCE            = 1.2   
local MIN_FORCE_IGNORE   = 10   

local VECTOR_CHANGE_TOL  = 10.0  
local PAUSE_TICKS_ON_CHANGE = 2  

-- Umbrales de Supervivencia
FRONT_THRESHOLD          = 25.0
BACK_THRESHOLD           = 10.0
PROX_CRITICAL_THRESHOLD  = 0.90

local ActiveLearning = {}
local history_x, history_y = {}, {}
local last_vector_mag = 0

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
    local sum_fx, sum_fy = 0, 0
    local closest_master_dist = 999999; local angle_of_closest = 0

    for k, data in pairs(srt_anterior_data) do
        if updated_anterior_indices[k] and data.angle and data.distance then
            active_count = active_count + 1
            local d = data.distance
            local ang_rad = data.angle * (math.pi/180)
            
            local force_mag = 0
            if d >= (PUSH_LIMIT + NEUTRAL_ZONE) then 
                local effective_d = math.min(d, M_MAX_DIST)
                local f_ratio = (effective_d - (PUSH_LIMIT + NEUTRAL_ZONE)) / (M_MAX_DIST - (PUSH_LIMIT + NEUTRAL_ZONE))
                force_mag = f_ratio * 100.0 
            elseif d <= (PUSH_LIMIT - NEUTRAL_ZONE) then 
                local effective_d = math.max(d, N_CRIT_DIST)
                local f_ratio = ((PUSH_LIMIT - NEUTRAL_ZONE) - effective_d) / ((PUSH_LIMIT - NEUTRAL_ZONE) - N_CRIT_DIST)
                force_mag = -(f_ratio * 100.0 * K_FORCE) 
            end

            sum_fx = sum_fx + (force_mag * math.cos(ang_rad))
            sum_fy = sum_fy + (force_mag * math.sin(ang_rad))

            if d < closest_master_dist then closest_master_dist = d; angle_of_closest = data.angle end
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

        local should_move = (math.abs(resultant_force) > MIN_FORCE_IGNORE)
        
        local trigger_pause = false
        if math.abs(resultant_force - last_vector_mag) > VECTOR_CHANGE_TOL then trigger_pause = true end
        last_vector_mag = resultant_force

        local dyn_speed_d = math.min(d_speed or 30, math.max(5.0, math.abs(resultant_force)))
        local dyn_speed_g = math.min(g_speed or 80, math.max(10.0, math.abs(resultant_angle)))

        if AL_LOG_MODE == 2 then log(string.format("[ACTIVE LEARNING] Campos | Mag: %.1f", resultant_force)) end

        return {
            should_move         = should_move,
            pause_ticks         = trigger_pause and PAUSE_TICKS_ON_CHANGE or 0,
            target_x            = smooth_fx, target_y = smooth_y,
            target_dist         = resultant_force, target_angle = resultant_angle,
            speed_d             = dyn_speed_d, speed_g = dyn_speed_g,
            closest_dist        = closest_master_dist, closest_angle = angle_of_closest,
            front_threshold     = FRONT_THRESHOLD, back_threshold = BACK_THRESHOLD, prox_threshold = PROX_CRITICAL_THRESHOLD,
            state_name          = "SIGUIENDO_FUERZAS",
            state_msg           = string.format("FUERZA | Mag: %.1f | Ang: %.1f°", resultant_force, resultant_angle),
            is_centroid         = false
        }
    else
        history_x, history_y = {}, {}
        return nil
    end
end

return ActiveLearning