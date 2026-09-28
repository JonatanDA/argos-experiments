local ActiveLearning = {}

-- Memoria local para el suavizado de la ventana de tiempo
local history_x = {}
local history_y = {}

local function get_smoothed_values(new_x, new_y)
    local max_ticks = AL_SMOOTH_TICKS or 1
    local method = AL_SMOOTH_METHOD or "median"

    table.insert(history_x, new_x)
    table.insert(history_y, new_y)

    -- Mantener el tamaño del buffer
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
        -- Mediana (ignora valores extremos/ruido)
        local tx, ty = {}, {}
        for i = 1, #history_x do tx[i] = history_x[i]; ty[i] = history_y[i] end
        table.sort(tx)
        table.sort(ty)
        local mid = math.floor(#tx / 2) + 1
        if #tx % 2 == 0 then
            return (tx[mid-1] + tx[mid]) / 2.0, (ty[mid-1] + ty[mid]) / 2.0
        else
            return tx[mid], ty[mid]
        end
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

            -- La detección de cercanía es INSTANTÁNEA (para evasión segura)
            if d < closest_master_dist then
                closest_master_dist = d
                angle_of_closest = data.angle
            end
        end
    end

    if active_count > 0 and ENABLE_ACTIVE_LEARNING then
        -- Aplicamos el filtro de suavizado a las fuerzas resultantes
        local smooth_fx, smooth_fy = get_smoothed_values(sum_fx, sum_fy)
        
        local resultant_force, resultant_angle_rad = math_helper.cartesianoAPolarPAR(smooth_fx, smooth_fy)
        local resultant_angle = resultant_angle_rad * (180 / math.pi)

        if Print_tracker_trayectory_data then
            local ang_total_res = resultant_angle_rad + obs_ang_offset_global
            local res_gx, res_gy = math_helper.polarACartesianoPAR(resultant_force, ang_total_res)
            Print_tracker_trayectory_data("Active_learning", 2, "Qupa4", 0, 0, res_gx/100, res_gy/100)
        end

        if AL_LOG_MODE == 2 then
            log(string.format("[ACTIVE LEARNING] FuerzaRes(%.1f) | Ang: %.1f° | Activos: %d | Cerca: %.1f cm", resultant_force, resultant_angle, active_count, closest_master_dist))
        end

        if closest_master_dist < front_threshold and motion_active and al_current_state == "SIGUIENDO_FUERZAS" then
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
                local diff_ang = resultant_angle - angle_of_closest
                if diff_ang > 180 then diff_ang = diff_ang - 360 end
                if diff_ang < -180 then diff_ang = diff_ang + 360 end
                local safe_angle = 0
                local msg_evasion = ""
                if math.abs(diff_ang) < 60 then 
                    if diff_ang >= 0 then safe_angle = angle_of_closest + 60
                    else safe_angle = angle_of_closest - 60 end
                    msg_evasion = string.format("EVADIENDO MASTER | Rodeando obstáculo hacia %.1f° | Dist: %.1f cm", safe_angle, closest_master_dist)
                else
                    safe_angle = resultant_angle
                    msg_evasion = string.format("EVADIENDO MASTER | Ruta resultante libre (%.1f°) | Dist: %.1f cm", safe_angle, closest_master_dist)
                end
                last_motion_marker = {back_threshold * porcentaje, safe_angle}
                change_al_state("EVADIENDO_MASTER", msg_evasion)
                r_move()
            elseif resultant_force > MIN_FORCE_THRESHOLD then
                motion_active = true
                local move_dist = resultant_force 
                if move_dist > (max_tracking_distance * 2) then move_dist = (max_tracking_distance * 2) end
                last_motion_marker = {move_dist * porcentaje, resultant_angle}
                local msg_perseguir = string.format("SIGUIENDO FUERZAS | Magnitud: %.1f | Ángulo: %.1f°", resultant_force, resultant_angle)
                change_al_state("SIGUIENDO_FUERZAS", msg_perseguir)
                r_move()
            end
        end
    else
        -- Si no vemos a nadie, borramos la memoria para evitar fantasmas
        history_x = {}
        history_y = {}
    end
end

return ActiveLearning