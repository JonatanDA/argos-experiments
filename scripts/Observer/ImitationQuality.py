# ============================================================================
# Archivo: Functions/Observer/ImitationQuality.py
# Módulo de Calidad de Imitación (Espejo 1:1 con ImitationQuality.lua)
# ============================================================================

import math

# Configuración calcada del script en Lua/Python
USE_FIRST_ANGLE = False
ASSUME_CLOSED_POLYGONS = True
WEIGHT_L = 1.0
WEIGHT_A = 1.0
WEIGHT_S = 1.0


# Limpia vectores y asume estructura: vector[0] = dist, vector[1] = ang
def clean_vectors(vectors):
    if not vectors or len(vectors) == 0:
        return []
    vecs = []
    for v in vectors:
        if isinstance(v, dict):
            vecs.append({'dist': float(v['dist']), 'ang': float(v['ang'])})
        else:
            vecs.append({'dist': float(v[0]), 'ang': float(v[1])})

    leading_angle_sum = 0.0
    idx = 0
    while idx < len(vecs) and vecs[idx]['dist'] == 0.0:
        leading_angle_sum = leading_angle_sum + vecs[idx]['ang']
        idx = idx + 1

    if idx >= len(vecs):
        return []

    trailing_angle_sum = 0.0
    end_idx = len(vecs) - 1
    while end_idx >= idx and vecs[end_idx]['dist'] == 0.0:
        trailing_angle_sum = trailing_angle_sum + vecs[end_idx]['ang']
        end_idx = end_idx - 1

    cleaned = []
    for i in range(idx, end_idx + 1):
        cleaned.append({'dist': vecs[i]['dist'], 'ang': vecs[i]['ang']})

    if len(cleaned) > 0:
        cleaned[0]['ang'] = cleaned[0]['ang'] + leading_angle_sum + trailing_angle_sum

    return cleaned


def heal_pattern(vectors):
    if not ASSUME_CLOSED_POLYGONS or len(vectors) <= 2:
        res = []
        for v in vectors:
            res.append({'dist': v['dist'], 'ang': v['ang']})
        return res

    sum_angles = 0.0
    for i in range(1, len(vectors)):
        sum_angles = sum_angles + vectors[i]['ang']

    target_sum = 360.0 if (sum_angles >= 0) else -360.0
    missing_turn = target_sum - sum_angles

    healed = []
    for v in vectors:
        healed.append({'dist': v['dist'], 'ang': v['ang']})
    healed[0]['ang'] = missing_turn
    return healed


def calc_coords(vectors):
    xs, ys = [0.0], [0.0]
    x, y = 0.0, 0.0
    current_angle = 0.0
    for v in vectors:
        current_angle = current_angle + v['ang']
        rad = math.radians(current_angle)
        x = x + v['dist'] * math.cos(rad)
        y = y + v['dist'] * math.sin(rad)
        xs.append(x)
        ys.append(y)
    return xs, ys


# Función principal que evalúa qué tan similares son dos patrones.
# Retorna un valor de 0.0 a 1.0 (0% a 100%) indicando la similitud.
def evaluate(orig_raw, imit_raw):
    orig = clean_vectors(orig_raw)
    imit = clean_vectors(imit_raw)

    No = len(orig)
    Nc = len(imit)
    if No == 0 or Nc == 0:
        return 0.0

    orig_healed = heal_pattern(orig)
    Qs = 1.0 - (abs(Nc - No) / No)

    orig_xs, orig_ys = calc_coords(orig_healed)
    imit_xs, imit_ys = calc_coords(imit)

    cx_o, cy_o = 0.0, 0.0
    for x in orig_xs:
        cx_o = cx_o + x
    for y in orig_ys:
        cy_o = cy_o + y
    cx_o = cx_o / len(orig_xs)
    cy_o = cy_o / len(orig_ys)

    cx_i, cy_i = 0.0, 0.0
    for x in imit_xs:
        cx_i = cx_i + x
    for y in imit_ys:
        cy_i = cy_i + y
    cx_i = cx_i / len(imit_xs)
    cy_i = cy_i / len(imit_ys)

    num, den = 0.0, 0.0
    min_len = min(len(orig_xs), len(imit_xs))
    for k in range(min_len):
        num = num + (orig_xs[k] - cx_o) * (imit_ys[k] - cy_i) - (orig_ys[k] - cy_o) * (imit_xs[k] - cx_i)
        den = den + (orig_xs[k] - cx_o) * (imit_xs[k] - cx_i) + (orig_ys[k] - cy_o) * (imit_ys[k] - cy_i)

    # Usamos math.atan2 para la orientación óptima
    theta_opt = math.atan2(num, den) if (den != 0) else 0.0
    deg_opt = math.degrees(theta_opt)

    imit_rotated = []
    for v in imit:
        imit_rotated.append({'dist': v['dist'], 'ang': v['ang']})
    if len(imit_rotated) > 0:
        imit_rotated[0]['ang'] = imit_rotated[0]['ang'] + deg_opt

    best_qi = -float('inf')

    if Nc >= No:
        imit_healed = heal_pattern(imit_rotated)
        Nmin = No
        for k in range(Nc):
            shifted_imit = []
            for j in range(k, Nc):
                shifted_imit.append(imit_healed[j])
            for j in range(0, k):
                shifted_imit.append(imit_healed[j])

            eval_orig = []
            for i, v in enumerate(orig_healed):
                eval_orig.append({
                    'dist': v['dist'],
                    'ang': 0.0 if (not USE_FIRST_ANGLE and i == 0) else v['ang']
                })

            eval_imit = []
            for i, v in enumerate(shifted_imit):
                eval_imit.append({
                    'dist': v['dist'],
                    'ang': 0.0 if (not USE_FIRST_ANGLE and i == 0) else v['ang']
                })

            sum_l_o, sum_diff_l = 0.0, 0.0
            for m in range(Nmin):
                sum_l_o = sum_l_o + abs(eval_orig[m]['dist'])
                sum_diff_l = sum_diff_l + abs(eval_orig[m]['dist'] - eval_imit[m]['dist'])
            Ql = (1.0 - (sum_diff_l / sum_l_o)) if (sum_l_o > 0) else (1.0 if (sum_diff_l == 0) else 0.0)

            sum_a_o, sum_diff_a = 0.0, 0.0
            # En Lua (1-based): start_idx = USE_FIRST_ANGLE and 1 or 2; if Nmin >= start_idx then for m = start_idx, Nmin
            # En Python (0-based): start_idx = 0 if USE_FIRST_ANGLE else 1; if Nmin > start_idx: for m in range(start_idx, Nmin)
            start_idx = 0 if USE_FIRST_ANGLE else 1
            if Nmin > start_idx:
                for m in range(start_idx, Nmin):
                    sum_a_o = sum_a_o + abs(eval_orig[m]['ang'])
                    sum_diff_a = sum_diff_a + abs(eval_orig[m]['ang'] - eval_imit[m]['ang'])

            Qa = 1.0
            if Nmin > start_idx:
                Qa = (1.0 - (sum_diff_a / sum_a_o)) if (sum_a_o > 0) else (1.0 if (sum_diff_a == 0) else 0.0)

            Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
            if Qi > best_qi:
                best_qi = Qi
    else:
        Nmin = Nc
        eval_orig_cycled = []
        for v in orig_healed:
            eval_orig_cycled.append(v)
        for v in orig_healed:
            eval_orig_cycled.append(v)

        for offset in range(No):
            eval_orig_tmp = []
            for j in range(Nmin):
                eval_orig_tmp.append({
                    'dist': eval_orig_cycled[offset + j]['dist'],
                    'ang': eval_orig_cycled[offset + j]['ang']
                })

            eval_imit_tmp = []
            for v in imit_rotated:
                eval_imit_tmp.append({'dist': v['dist'], 'ang': v['ang']})

            eval_orig_tmp[0]['ang'] = eval_imit_tmp[0]['ang']

            sum_l_o, sum_diff_l = 0.0, 0.0
            for m in range(Nmin):
                sum_l_o = sum_l_o + abs(eval_orig_tmp[m]['dist'])
                sum_diff_l = sum_diff_l + abs(eval_orig_tmp[m]['dist'] - eval_imit_tmp[m]['dist'])
            Ql = (1.0 - (sum_diff_l / sum_l_o)) if (sum_l_o > 0) else (1.0 if (sum_diff_l == 0) else 0.0)

            sum_a_o, sum_diff_a = 0.0, 0.0
            # En Lua (1-based): start_idx = 2; if Nmin >= start_idx
            # En Python (0-based): start_idx = 1; if Nmin > start_idx
            start_idx = 1
            if Nmin > start_idx:
                for m in range(start_idx, Nmin):
                    sum_a_o = sum_a_o + abs(eval_orig_tmp[m]['ang'])
                    sum_diff_a = sum_diff_a + abs(eval_orig_tmp[m]['ang'] - eval_imit_tmp[m]['ang'])

            Qa = 1.0
            if Nmin > start_idx:
                Qa = (1.0 - (sum_diff_a / sum_a_o)) if (sum_a_o > 0) else (1.0 if (sum_diff_a == 0) else 0.0)

            Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
            if Qi > best_qi:
                best_qi = Qi

    return best_qi


class ImitationQuality:
    USE_FIRST_ANGLE = USE_FIRST_ANGLE
    ASSUME_CLOSED_POLYGONS = ASSUME_CLOSED_POLYGONS
    WEIGHT_L = WEIGHT_L
    WEIGHT_A = WEIGHT_A
    WEIGHT_S = WEIGHT_S

    clean_vectors = staticmethod(clean_vectors)
    heal_pattern = staticmethod(heal_pattern)
    calc_coords = staticmethod(calc_coords)
    evaluate = staticmethod(evaluate)