import math

USE_FIRST_ANGLE = False
ASSUME_CLOSED_POLYGONS = True
WEIGHT_L = 1.0
WEIGHT_A = 1.0
WEIGHT_S = 1.0

def clean_vectors(vectors):
    if not vectors: return []
    vecs = [{'dist': v[0], 'ang': v[1]} for v in vectors]
    
    leading_angle_sum = 0.0
    idx = 0
    while idx < len(vecs) and vecs[idx]['dist'] == 0.0:
        leading_angle_sum += vecs[idx]['ang']
        idx += 1
    if idx >= len(vecs): return []
    
    trailing_angle_sum = 0.0
    end_idx = len(vecs) - 1
    while end_idx >= idx and vecs[end_idx]['dist'] == 0.0:
        trailing_angle_sum += vecs[end_idx]['ang']
        end_idx -= 1
        
    cleaned = [{'dist': vecs[i]['dist'], 'ang': vecs[i]['ang']} for i in range(idx, end_idx + 1)]
    if cleaned:
        cleaned[0]['ang'] += leading_angle_sum + trailing_angle_sum
    return cleaned

def heal_pattern(vectors):
    if not ASSUME_CLOSED_POLYGONS or len(vectors) <= 2:
        return [{'dist': v['dist'], 'ang': v['ang']} for v in vectors]
    
    sum_angles = sum(v['ang'] for v in vectors[1:])
    target_sum = 360.0 if sum_angles >= 0 else -360.0
    missing_turn = target_sum - sum_angles
    
    healed = [{'dist': v['dist'], 'ang': v['ang']} for v in vectors]
    healed[0]['ang'] = missing_turn
    return healed

def calc_coords(vectors):
    xs, ys = [0.0], [0.0]
    x, y, current_angle = 0.0, 0.0, 0.0
    for v in vectors:
        current_angle += v['ang']
        rad = math.radians(current_angle)
        x += v['dist'] * math.cos(rad)
        y += v['dist'] * math.sin(rad)
        xs.append(x)
        ys.append(y)
    return xs, ys

def evaluate(orig_raw, imit_raw):
    orig = clean_vectors(orig_raw)
    imit = clean_vectors(imit_raw)
    No = len(orig)
    Nc = len(imit)
    if No == 0 or Nc == 0: return 0.0
    
    orig_healed = heal_pattern(orig)
    Qs = 1.0 - (abs(Nc - No) / No)
    
    orig_xs, orig_ys = calc_coords(orig_healed)
    imit_xs, imit_ys = calc_coords(imit)
    
    cx_o = sum(orig_xs) / len(orig_xs)
    cy_o = sum(orig_ys) / len(orig_ys)
    cx_i = sum(imit_xs) / len(imit_xs)
    cy_i = sum(imit_ys) / len(imit_ys)
    
    num, den = 0.0, 0.0
    min_len = min(len(orig_xs), len(imit_xs))
    for k in range(min_len):
        num += (orig_xs[k] - cx_o)*(imit_ys[k] - cy_i) - (orig_ys[k] - cy_o)*(imit_xs[k] - cx_i)
        den += (orig_xs[k] - cx_o)*(imit_xs[k] - cx_i) + (orig_ys[k] - cy_o)*(imit_ys[k] - cy_i)
        
    theta_opt = math.atan2(num, den) if den != 0 else 0.0
    deg_opt = math.degrees(theta_opt)
    
    imit_rotated = [{'dist': v['dist'], 'ang': v['ang']} for v in imit]
    if imit_rotated: imit_rotated[0]['ang'] += deg_opt
    
    best_qi = float('-inf')
    
    if Nc >= No:
        imit_healed = heal_pattern(imit_rotated)
        Nmin = No
        for k in range(Nc):
            shifted_imit = imit_healed[k:] + imit_healed[:k]
            eval_orig = [{'dist': v['dist'], 'ang': 0.0 if (not USE_FIRST_ANGLE and i==0) else v['ang']} for i, v in enumerate(orig_healed)]
            eval_imit = [{'dist': v['dist'], 'ang': 0.0 if (not USE_FIRST_ANGLE and i==0) else v['ang']} for i, v in enumerate(shifted_imit)]
            
            sum_l_o = sum(abs(eval_orig[m]['dist']) for m in range(Nmin))
            sum_diff_l = sum(abs(eval_orig[m]['dist'] - eval_imit[m]['dist']) for m in range(Nmin))
            Ql = (1.0 - (sum_diff_l / sum_l_o)) if sum_l_o > 0 else (1.0 if sum_diff_l == 0 else 0.0)
            
            sum_a_o, sum_diff_a = 0.0, 0.0
            start_idx = 0 if USE_FIRST_ANGLE else 1
            if Nmin > start_idx:
                sum_a_o = sum(abs(eval_orig[m]['ang']) for m in range(start_idx, Nmin))
                sum_diff_a = sum(abs(eval_orig[m]['ang'] - eval_imit[m]['ang']) for m in range(start_idx, Nmin))
            Qa = 1.0
            if Nmin > start_idx:
                Qa = (1.0 - (sum_diff_a / sum_a_o)) if sum_a_o > 0 else (1.0 if sum_diff_a == 0 else 0.0)
                
            Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
            if Qi > best_qi: best_qi = Qi
    else:
        Nmin = Nc
        eval_orig_cycled = orig_healed + orig_healed
        for offset in range(No):
            eval_orig_tmp = [{'dist': eval_orig_cycled[offset+j]['dist'], 'ang': eval_orig_cycled[offset+j]['ang']} for j in range(Nmin)]
            eval_imit_tmp = [{'dist': v['dist'], 'ang': v['ang']} for v in imit_rotated]
            eval_orig_tmp[0]['ang'] = eval_imit_tmp[0]['ang']
            
            sum_l_o = sum(abs(eval_orig_tmp[m]['dist']) for m in range(Nmin))
            sum_diff_l = sum(abs(eval_orig_tmp[m]['dist'] - eval_imit_tmp[m]['dist']) for m in range(Nmin))
            Ql = (1.0 - (sum_diff_l / sum_l_o)) if sum_l_o > 0 else (1.0 if sum_diff_l == 0 else 0.0)
            
            sum_a_o, sum_diff_a = 0.0, 0.0
            start_idx = 1
            if Nmin > start_idx:
                sum_a_o = sum(abs(eval_orig_tmp[m]['ang']) for m in range(start_idx, Nmin))
                sum_diff_a = sum(abs(eval_orig_tmp[m]['ang'] - eval_imit_tmp[m]['ang']) for m in range(start_idx, Nmin))
            Qa = 1.0
            if Nmin > start_idx:
                Qa = (1.0 - (sum_diff_a / sum_a_o)) if sum_a_o > 0 else (1.0 if sum_diff_a == 0 else 0.0)
                
            Qi = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
            if Qi > best_qi: best_qi = Qi

    return best_qi
