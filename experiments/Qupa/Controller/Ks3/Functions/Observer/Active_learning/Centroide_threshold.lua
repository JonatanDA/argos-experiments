-- ============================================================================
-- VARIABLES DEL MÓDULO: CENTROIDE THRESHOLD
-- ============================================================================
-- AL_LOG_MODE              = 2
AL_SMOOTH_TICKS          = 3
AL_SMOOTH_METHOD         = "median"
MAX_TRACKING_DISTANCE    = 15.0      -- (cm) Si el centroide está dentro de este radio, NO se mueve.
PAUSE_TICKS_ON_CHANGE    = 1         -- Ticks de pausa al reanudar movimiento.

-- Umbrales de Supervivencia
FRONT_THRESHOLD          = 25.0
BACK_THRESHOLD           = 10.0
PROX_CRITICAL_THRESHOLD  = 0.90

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
            if data.distance < closest_master_dist then
                closest_master_dist = data.distance; angle_of_closest = data.angle
            end
        end
    end

    if active_count > 0 and ENABLE_ACTIVE_LEARNING then
        if active_count == 1 then active_count = 2 end

        local raw_centroid_x = sum_x / active_count
        local raw_centroid_y = sum_y / active_count
        local smooth_x, smooth_y = get_smoothed_values(raw_centroid_x, raw_centroid_y)

        local centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        local centroid_angle = centroid_angle_rad * (180 / math.pi)

        if Print_tracker_trayectory_data then
            local ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            local cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            Print_tracker_trayectory_data("Active_learning", 2, "Qupa4", 0, 0, cgx/100, cgy/100)
        end

        local do_move = (centroid_dist > MAX_TRACKING_DISTANCE)
        local trigger_pause = false
        if do_move and not last_do_move_state then trigger_pause = true end
        last_do_move_state = do_move

        if AL_LOG_MODE == 2 then log(string.format("[ACTIVE LEARNING] Threshold(%.1f) | Activos: %d | Cerca: %.1f cm", centroid_dist, active_count, closest_master_dist)) end

        return {
            should_move         = do_move,
            pause_ticks         = trigger_pause and PAUSE_TICKS_ON_CHANGE or 0,
            target_x            = smooth_x, target_y = smooth_y,
            target_dist         = centroid_dist, target_angle = centroid_angle,
            speed_d             = d_speed, speed_g = g_speed,
            closest_dist        = closest_master_dist, closest_angle = angle_of_closest,
            front_threshold     = FRONT_THRESHOLD, back_threshold = BACK_THRESHOLD, prox_threshold = PROX_CRITICAL_THRESHOLD,
            state_name          = "PERSIGUIENDO_CENTROIDE",
            state_msg           = string.format("THRESHOLD | Dist: %.1f cm", centroid_dist),
            is_centroid         = true
        }
    else
        history_x, history_y = {}, {}
        return nil
    end
end

return ActiveLearning