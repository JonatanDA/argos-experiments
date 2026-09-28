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
        
        -- Aplicamos memoria suavizada
        local smooth_x, smooth_y = get_smoothed_values(raw_centroid_x, raw_centroid_y)

        local centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        local centroid_angle = centroid_angle_rad * (180 / math.pi)

        if Print_tracker_trayectory_data then
            local ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            local cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            Print_tracker_trayectory_data("Active_learning", 1, "Qupa4", 0, 0, cgx/100, cgy/100)
        end

        if AL_LOG_MODE == 2 then
            log(string.format("[ACTIVE LEARNING] Centroide(%.1f) | Activos: %d | Cerca: %.1f cm", centroid_dist, active_count, closest_master_dist))
        end

        if closest_master_dist < front_threshold and motion_active and al_current_state == "PERSIGUIENDO_CENTROIDE" then
            srt_ang_mark_ant = normalize_angle_rad(move_angle + srt_ang_mark_ant)
            x_ant_r = x_mark
            y_ant_r = y_mark
            motion_active = false
            r_move()  
            robot.wheels.set_velocity(0, 0)
            change_al_state("IDLE", "") 
        end

        if not motion_active then
            if closest_master_dist < front_threshold then
                motion_active = true
                local diff_ang = centroid_angle - angle_of_closest
                if diff_ang > 180 then diff_ang = diff_ang - 360 end
                if diff_ang < -180 then diff_ang = diff_ang + 360 end
                local safe_angle = 0
                local msg_evasion = ""
                if math.abs(diff_ang) < 60 then 
                    if diff_ang >= 0 then safe_angle = angle_of_closest + 60
                    else safe_angle = angle_of_closest - 60 end
                    msg_evasion = string.format("EVADIENDO MASTER | Rodeando obstáculo visual hacia %.1f° | Dist: %.1f cm", safe_angle, closest_master_dist)
                else
                    safe_angle = centroid_angle
                    msg_evasion = string.format("EVADIENDO MASTER | Ruta al centroide libre (%.1f°) | Dist: %.1f cm", safe_angle, closest_master_dist)
                end
                last_motion_marker = {back_threshold * porcentaje, safe_angle}
                change_al_state("EVADIENDO_MASTER", msg_evasion)
                r_move()
            elseif centroid_dist > max_tracking_distance then
                motion_active = true
                last_motion_marker = {centroid_dist * porcentaje, centroid_angle}
                local msg_perseguir = string.format("PERSIGUIENDO CENTROIDE | Distancia: %.1f cm | Ángulo: %.1f°", centroid_dist, centroid_angle)
                change_al_state("PERSIGUIENDO_CENTROIDE", msg_perseguir)
                r_move()
            end
        end
    else
        history_x = {}
        history_y = {}
    end
end

return ActiveLearning