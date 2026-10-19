-- ============================================================================
-- Archivo: Functions/Observer/ImitationQuality.lua
-- Módulo de Calidad de Imitación (Traducción de Python a Lua)
-- ============================================================================
local ImitationQuality = {}

-- Configuración calcada del script en Python
local USE_FIRST_ANGLE = false
local ASSUME_CLOSED_POLYGONS = true
local WEIGHT_L = 1.0
local WEIGHT_A = 1.0
local WEIGHT_S = 1.0

-- Limpia vectores y asume estructura: vector[1] = dist, vector[2] = ang
local function clean_vectors(vectors)
    if not vectors or #vectors == 0 then return {} end
    local vecs = {}
    for i, v in ipairs(vectors) do
        table.insert(vecs, {dist = v[1], ang = v[2]})
    end

    local leading_angle_sum = 0.0
    local idx = 1
    while idx <= #vecs and vecs[idx].dist == 0.0 do
        leading_angle_sum = leading_angle_sum + vecs[idx].ang
        idx = idx + 1
    end

    if idx > #vecs then return {} end

    local trailing_angle_sum = 0.0
    local end_idx = #vecs
    while end_idx >= idx and vecs[end_idx].dist == 0.0 do
        trailing_angle_sum = trailing_angle_sum + vecs[end_idx].ang
        end_idx = end_idx - 1
    end

    local cleaned = {}
    for i = idx, end_idx do
        table.insert(cleaned, {dist = vecs[i].dist, ang = vecs[i].ang})
    end

    if #cleaned > 0 then
        cleaned[1].ang = cleaned[1].ang + leading_angle_sum + trailing_angle_sum
    end

    return cleaned
end

local function heal_pattern(vectors)
    if not ASSUME_CLOSED_POLYGONS or #vectors <= 2 then
        local res = {}
        for i, v in ipairs(vectors) do table.insert(res, {dist=v.dist, ang=v.ang}) end
        return res
    end

    local sum_angles = 0
    for i = 2, #vectors do sum_angles = sum_angles + vectors[i].ang end

    local target_sum = (sum_angles >= 0) and 360.0 or -360.0
    local missing_turn = target_sum - sum_angles

    local healed = {}
    for i, v in ipairs(vectors) do table.insert(healed, {dist=v.dist, ang=v.ang}) end
    healed[1].ang = missing_turn
    return healed
end

local function calc_coords(vectors)
    local xs, ys = {0.0}, {0.0}
    local x, y = 0.0, 0.0
    local current_angle = 0.0
    for i, v in ipairs(vectors) do
        current_angle = current_angle + v.ang
        local rad = math.rad(current_angle)
        x = x + v.dist * math.cos(rad)
        y = y + v.dist * math.sin(rad)
        table.insert(xs, x)
        table.insert(ys, y)
    end
    return xs, ys
end

-- Función principal que evalúa qué tan similares son dos patrones.
-- Retorna un valor de 0.0 a 1.0 (0% a 100%) indicando la similitud.
function ImitationQuality.evaluate(orig_raw, imit_raw)
    local orig = clean_vectors(orig_raw)
    local imit = clean_vectors(imit_raw)

    local No = #orig
    local Nc = #imit
    if No == 0 or Nc == 0 then return 0.0 end

    local orig_healed = heal_pattern(orig)
    local Qs = 1.0 - (math.abs(Nc - No) / No)

    local orig_xs, orig_ys = calc_coords(orig_healed)
    local imit_xs, imit_ys = calc_coords(imit)

    local cx_o, cy_o = 0, 0
    for _, x in ipairs(orig_xs) do cx_o = cx_o + x end
    for _, y in ipairs(orig_ys) do cy_o = cy_o + y end
    cx_o = cx_o / #orig_xs; cy_o = cy_o / #orig_ys

    local cx_i, cy_i = 0, 0
    for _, x in ipairs(imit_xs) do cx_i = cx_i + x end
    for _, y in ipairs(imit_ys) do cy_i = cy_i + y end
    cx_i = cx_i / #imit_xs; cy_i = cy_i / #imit_ys

    local num, den = 0, 0
    local min_len = math.min(#orig_xs, #imit_xs)
    for k = 1, min_len do
        num = num + (orig_xs[k] - cx_o) * (imit_ys[k] - cy_i) - (orig_ys[k] - cy_o) * (imit_xs[k] - cx_i)
        den = den + (orig_xs[k] - cx_o) * (imit_xs[k] - cx_i) + (orig_ys[k] - cy_o) * (imit_ys[k] - cy_i)
    end

    -- Usamos math.atan2 de Lua para la orientación óptima
    local theta_opt = (den ~= 0) and math.atan2(num, den) or 0.0
    local deg_opt = math.deg(theta_opt)

    local imit_rotated = {}
    for i, v in ipairs(imit) do table.insert(imit_rotated, {dist=v.dist, ang=v.ang}) end
    if #imit_rotated > 0 then imit_rotated[1].ang = imit_rotated[1].ang + deg_opt end

    local best_qi = -math.huge

    if Nc >= No then
        local imit_healed = heal_pattern(imit_rotated)
        local Nmin = No
        for k = 1, Nc do
            local shifted_imit = {}
            for j = k, Nc do table.insert(shifted_imit, imit_healed[j]) end
            for j = 1, k-1 do table.insert(shifted_imit, imit_healed[j]) end

            local eval_orig = {}
            for i, v in ipairs(orig_healed) do table.insert(eval_orig, {dist=v.dist, ang= (not USE_FIRST_ANGLE and i==1) and 0.0 or v.ang}) end

            local eval_imit = {}
            for i, v in ipairs(shifted_imit) do table.insert(eval_imit, {dist=v.dist, ang= (not USE_FIRST_ANGLE and i==1) and 0.0 or v.ang}) end

            local sum_l_o, sum_diff_l = 0, 0
            for m = 1, Nmin do
                sum_l_o = sum_l_o + math.abs(eval_orig[m].dist)
                sum_diff_l = sum_diff_l + math.abs(eval_orig[m].dist - eval_imit[m].dist)
            end
            local Ql = (sum_l_o > 0) and (1.0 - (sum_diff_l / sum_l_o)) or ((sum_diff_l == 0) and 1.0 or 0.0)

            local sum_a_o, sum_diff_a = 0, 0
            local start_idx = USE_FIRST_ANGLE and 1 or 2
            if Nmin >= start_idx then
                for m = start_idx, Nmin do
                    sum_a_o = sum_a_o + math.abs(eval_orig[m].ang)
                    sum_diff_a = sum_diff_a + math.abs(eval_orig[m].ang - eval_imit[m].ang)
                end
            end
            local Qa = 1.0
            if Nmin >= start_idx then Qa = (sum_a_o > 0) and (1.0 - (sum_diff_a / sum_a_o)) or ((sum_diff_a == 0) and 1.0 or 0.0) end

            local Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
            if Qi > best_qi then best_qi = Qi end
        end
    else
        local Nmin = Nc
        local eval_orig_cycled = {}
        for _, v in ipairs(orig_healed) do table.insert(eval_orig_cycled, v) end
        for _, v in ipairs(orig_healed) do table.insert(eval_orig_cycled, v) end

        for offset = 1, No do
            local eval_orig_tmp = {}
            for j = 0, Nmin - 1 do table.insert(eval_orig_tmp, {dist=eval_orig_cycled[offset + j].dist, ang=eval_orig_cycled[offset + j].ang}) end

            local eval_imit_tmp = {}
            for _, v in ipairs(imit_rotated) do table.insert(eval_imit_tmp, {dist=v.dist, ang=v.ang}) end

            eval_orig_tmp[1].ang = eval_imit_tmp[1].ang

            local sum_l_o, sum_diff_l = 0, 0
            for m = 1, Nmin do
                sum_l_o = sum_l_o + math.abs(eval_orig_tmp[m].dist)
                sum_diff_l = sum_diff_l + math.abs(eval_orig_tmp[m].dist - eval_imit_tmp[m].dist)
            end
            local Ql = (sum_l_o > 0) and (1.0 - (sum_diff_l / sum_l_o)) or ((sum_diff_l == 0) and 1.0 or 0.0)

            local sum_a_o, sum_diff_a = 0, 0
            local start_idx = 2
            if Nmin >= start_idx then
                for m = start_idx, Nmin do
                    sum_a_o = sum_a_o + math.abs(eval_orig_tmp[m].ang)
                    sum_diff_a = sum_diff_a + math.abs(eval_orig_tmp[m].ang - eval_imit_tmp[m].ang)
                end
            end
            local Qa = 1.0
            if Nmin >= start_idx then Qa = (sum_a_o > 0) and (1.0 - (sum_diff_a / sum_a_o)) or ((sum_diff_a == 0) and 1.0 or 0.0) end

            local Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
            if Qi > best_qi then best_qi = Qi end
        end
    end

    return best_qi
end

return ImitationQuality