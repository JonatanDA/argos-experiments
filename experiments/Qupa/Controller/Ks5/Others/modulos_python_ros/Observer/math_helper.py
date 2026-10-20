import math

def get_window_stats(start_idx, window_size, xx, yy):
    end_idx = min(start_idx + window_size, len(xx))
    wx = xx[start_idx:end_idx]
    wy = yy[start_idx:end_idx]
    if not wx:
        return 0.0, 0.0, 0.0
    cx = sum(wx) / len(wx)
    cy = sum(wy) / len(wy)
    var = sum((x - cx)**2 + (y - cy)**2 for x, y in zip(wx, wy)) / len(wx)
    return math.sqrt(var), cx, cy

def otsu_1d(data):
    if not data:
        return 0.0, 0.0
    sorted_data = sorted(data)
    total = len(sorted_data)
    sum_total = sum(sorted_data)
    sum_b, w_b = 0.0, 0
    max_var, threshold = 0.0, 0.0
    for i in range(total):
        w_b += 1
        if w_b == 0: continue
        w_f = total - w_b
        if w_f == 0: break
        sum_b += sorted_data[i]
        m_b = sum_b / w_b
        m_f = (sum_total - sum_b) / w_f
        var_between = w_b * w_f * (m_b - m_f) ** 2
        if var_between > max_var:
            max_var = var_between
            threshold = sorted_data[i]
    separacion = max_var / (sum_total**2 + 1e-9)
    return threshold, separacion

def polarACartesianoPAR(dist, ang_rad):
    return dist * math.cos(ang_rad), dist * math.sin(ang_rad)
