# Archivo: Math_Functions/math_helper.py

import math

# ---------------------------------------------------------
# ESTADÍSTICAS
# ---------------------------------------------------------
def get_window_stats(start_idx, win_size, xx, yy):
    sum_x, sum_y = 0.0, 0.0
    n = 0

    # 1. Calcular Centroide (Promedio)
    # Nota: start_idx es 0-based en Python (equivale a start_idx .. start_idx + win_size - 1 en Lua)
    for j in range(start_idx, start_idx + win_size):
        if 0 <= j < len(xx) and 0 <= j < len(yy) and xx[j] is not None and yy[j] is not None:
            sum_x = sum_x + xx[j]
            sum_y = sum_y + yy[j]
            n = n + 1

    if n == 0:
        return 0.0, 0.0, 0.0
    avg_x = sum_x / n
    avg_y = sum_y / n

    # 2. Calcular Dispersión (Desviación Estándar combinada)
    sum_sq_dist = 0.0
    for j in range(start_idx, start_idx + win_size):
        if 0 <= j < len(xx) and 0 <= j < len(yy) and xx[j] is not None and yy[j] is not None:
            dx = xx[j] - avg_x
            dy = yy[j] - avg_y
            sum_sq_dist = sum_sq_dist + (dx * dx + dy * dy)

    # Retornamos: Desviación Estándar, Centroide X, Centroide Y
    return math.sqrt(sum_sq_dist / n), avg_x, avg_y


# Función para encontrar el umbral óptimo usando el Método de Otsu 1D
def otsu_1d(varianzas):
    if not varianzas or len(varianzas) == 0:
        return 0.0, 0.0

    # Encontrar min y max para crear el histograma
    min_v, max_v = varianzas[0], varianzas[0]
    for i in range(len(varianzas)):
        if varianzas[i] < min_v:
            min_v = varianzas[i]
        if varianzas[i] > max_v:
            max_v = varianzas[i]

    if min_v == max_v:
        return min_v, 0.0

    # Crear histograma aproximado (50 bins)
    num_bins = 50
    hist = [0] * num_bins

    bin_size = (max_v - min_v) / num_bins
    for i in range(len(varianzas)):
        # En Lua: math.floor((varianzas[i] - min_v) / bin_size) + 1 (1..50)
        # En Python (0..49):
        bin_idx = int(math.floor((varianzas[i] - min_v) / bin_size))
        if bin_idx >= num_bins:
            bin_idx = num_bins - 1
        hist[bin_idx] = hist[bin_idx] + 1

    total_datos = len(varianzas)
    sum_total = 0.0
    for i in range(1, num_bins + 1):
        bin_val = min_v + (i - 0.5) * bin_size
        sum_total = sum_total + (bin_val * hist[i - 1])

    weight_bg = 0
    sum_bg = 0.0
    max_varianza_interclase = -1.0
    umbral_optimo = 0.0

    # Evaluar cada posible umbral (Otsu)
    for i in range(1, num_bins + 1):
        weight_bg = weight_bg + hist[i - 1]

        if weight_bg > 0:
            weight_fg = total_datos - weight_bg
            if weight_fg == 0:
                break

            bin_val = min_v + (i - 0.5) * bin_size
            sum_bg = sum_bg + (bin_val * hist[i - 1])

            mean_bg = sum_bg / weight_bg
            mean_fg = (sum_total - sum_bg) / weight_fg

            # Varianza Inter-clase
            var_between = weight_bg * weight_fg * (mean_bg - mean_fg) * (mean_bg - mean_fg)

            if var_between > max_varianza_interclase:
                max_varianza_interclase = var_between
                umbral_optimo = bin_val

    return umbral_optimo, max_varianza_interclase


# ---------------------------------------------------------
# OPERACIONES Y CÁLCULOS
# ---------------------------------------------------------
def polarACartesianoPAR(r, theta):
    y = r * math.sin(theta)
    x = r * math.cos(theta)
    return x, y


def cartesianoAPolarPAR(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(y, x)
    return r, theta


def vec2_angle(v):
    if isinstance(v, dict):
        w, x, y, z = v['w'], v['x'], v['y'], v['z']
    else:
        w, x, y, z = v.w, v.x, v.y, v.z
    ang = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    ang = ang * 180 / math.pi
    return ang