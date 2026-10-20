# Archivo: Functions/Observer/Segment_Parameterizer.py

import math


class SegmentParameterizer:
    # ============================================================================
    # VARIABLES DE CONTROL (LOGGEO Y AJUSTES)
    # ============================================================================
    LOG_APPLY        = 0  # 1 para ver las regresiones iniciales
    LOG_MERGE        = 0  # 1 para ver las fusiones de segmentos similares
    LOG_INTERSECT    = 0  # 1 para ver las intersecciones calculadas
    LOG_DIST_ANGLES  = 0  # 1 para ver distancias y ángulos relativos
    LOG_PARAMS       = 0  # 1 para ver el resultado final empaquetado

    # Tolerancia en grados. Si dos rectas consecutivas tienen un ángulo
    # con una diferencia menor o igual a este valor, se fusionan.
    TOLERANCIA_ANGULO_FUSION = 15.0

    # Helper para asegurar que el print funcione con el formato de PYTHON GESTOR
    @classmethod
    def sys_log(cls, msg):
        print(f"[PYTHON GESTOR] {msg}", flush=True)

    @classmethod
    def fmt_f(cls, val, decimals=2):
        return f"{val:.{decimals}f}".replace('.', ',')

    @staticmethod
    def _disp_col(col):
        """Muestra el índice de columna en base 1 (igual que Lua) para los logs."""
        return str(col + 1) if isinstance(col, int) else str(col)

    # ============================================================================
    # HELPER: Calcula la regresión lineal para un conjunto de puntos
    # ============================================================================
    @staticmethod
    def compute_regression(points):
        sum_x, sum_y = 0.0, 0.0
        n = len(points)

        for point in points:
            sum_x = sum_x + point[0]
            sum_y = sum_y + point[1]
        mean_x = sum_x / n
        mean_y = sum_y / n

        s_xx, s_yy, s_xy = 0.0, 0.0, 0.0
        for point in points:
            dx = point[0] - mean_x
            dy = point[1] - mean_y
            s_xx = s_xx + (dx * dx)
            s_yy = s_yy + (dy * dy)
            s_xy = s_xy + (dx * dy)

        is_vertical = False

        if s_yy > s_xx:
            is_vertical = True
            m = 0.0 if s_yy == 0 else (s_xy / s_yy)
            b = mean_x - (m * mean_y)
        else:
            is_vertical = False
            m = 0.0 if s_xx == 0 else (s_xy / s_xx)
            b = mean_y - (m * mean_x)

        return {'m': m, 'b': b, 'is_vertical': is_vertical, 'points_count': n}

    # ============================================================================
    # HELPER: Obtiene el ángulo de la recta en grados [0, 180)
    # ============================================================================
    @staticmethod
    def get_line_angle(eq):
        if eq['is_vertical']:
            angle = math.degrees(math.atan2(1, eq['m']))
        else:
            angle = math.degrees(math.atan2(eq['m'], 1))

        if angle < 0:
            angle = angle + 180
        if angle >= 180:
            angle = angle - 180

        return angle

    # ============================================================================
    # 1. APPLY: Calcula regresiones ortogonales dinámicas (Normal o Invertida)
    # ============================================================================
    @classmethod
    def apply(cls, segmentos):
        equations = {}
        valid_cols = []

        if isinstance(segmentos, dict):
            ordered_keys = sorted(segmentos.keys())
        else:
            ordered_keys = list(range(len(segmentos)))

        if cls.LOG_APPLY == 1:
            cls.sys_log("\n[APPLY] Iniciando cálculo de regresiones...")

        for col in ordered_keys:
            points = segmentos[col]
            if len(points) > 3:
                equations[col] = cls.compute_regression(points)
                valid_cols.append(col)

                if cls.LOG_APPLY == 1:
                    eq = equations[col]
                    vert_str = "true" if eq['is_vertical'] else "false"
                    cls.sys_log(
                        f"Seg {cls._disp_col(col)}: {eq['points_count']} puntos | Vert: {vert_str} | m: {cls.fmt_f(eq['m'], 3)} | b: {cls.fmt_f(eq['b'], 3)} | Ángulo: {cls.fmt_f(cls.get_line_angle(eq), 2)}°\n"
                    )

        return equations, valid_cols

    # ============================================================================
    # 1.5. MERGE: Fusiona rectas consecutivas con pendientes (ángulos) similares
    # ============================================================================
    @classmethod
    def merge_collinear_segments(cls, equations, segmentos, valid_cols):
        if cls.LOG_MERGE == 1:
            cls.sys_log(f"\n[MERGE] Evaluando fusión (Tolerancia: {cls.fmt_f(cls.TOLERANCIA_ANGULO_FUSION, 1)}°)...")

        if len(valid_cols) < 2:
            return equations, valid_cols

        i = 0
        while i < len(valid_cols) - 1:
            col1 = valid_cols[i]
            col2 = valid_cols[i + 1]
            eq1 = equations[col1]
            eq2 = equations[col2]

            angle1 = cls.get_line_angle(eq1)
            angle2 = cls.get_line_angle(eq2)

            diff = abs(angle1 - angle2)
            if diff > 90:
                diff = 180 - diff

            if diff <= cls.TOLERANCIA_ANGULO_FUSION:
                if cls.LOG_MERGE == 1:
                    cls.sys_log(
                        f"  -> Fusionando Seg {cls._disp_col(col1)} y Seg {cls._disp_col(col2)} (Diferencia: {cls.fmt_f(diff, 2)}°)"
                    )

                for p in segmentos[col2]:
                    segmentos[col1].append(p)

                equations[col1] = cls.compute_regression(segmentos[col1])

                valid_cols.pop(i + 1)
                del equations[col2]
            else:
                i = i + 1

        return equations, valid_cols

    # ============================================================================
    # 2. CALCULAR INTERSECCIONES: Cruza las rectas calculadas
    # ============================================================================
    @classmethod
    def calcular_intersecciones_con_extremos(cls, equations, segmentos, valid_cols):
        intersections = []

        if cls.LOG_INTERSECT == 1:
            cls.sys_log("\n[INTERSECCIONES] Calculando cruces...")

        if len(valid_cols) == 0:
            return intersections

        first_key = valid_cols[0]
        first_eq = equations.get(first_key)
        first_segment = segmentos[first_key] if (isinstance(segmentos, dict) or first_key < len(segmentos)) else None

        if not isinstance(first_eq, dict) or not isinstance(first_segment, list) or len(first_segment) == 0:
            return intersections

        if first_eq['is_vertical']:
            y_inicio = first_segment[0][1]
            x_inicio = first_eq['m'] * y_inicio + first_eq['b']
        else:
            x_inicio = first_segment[0][0]
            y_inicio = first_eq['m'] * x_inicio + first_eq['b']
        intersections.append([x_inicio, y_inicio])
        if cls.LOG_INTERSECT == 1:
            cls.sys_log(f"Extremo Inicial: ({cls.fmt_f(x_inicio, 3)}, {cls.fmt_f(y_inicio, 3)})\n")

        for i in range(len(valid_cols) - 1):
            eq1 = equations[valid_cols[i]]
            eq2 = equations[valid_cols[i + 1]]
            x_int, y_int = None, None

            if not eq1['is_vertical'] and not eq2['is_vertical']:
                den = eq1['m'] - eq2['m']
                if abs(den) > 1e-6:
                    x_int = (eq2['b'] - eq1['b']) / den
                    y_int = eq1['m'] * x_int + eq1['b']
            elif eq1['is_vertical'] and eq2['is_vertical']:
                den = eq1['m'] - eq2['m']
                if abs(den) > 1e-6:
                    y_int = (eq2['b'] - eq1['b']) / den
                    x_int = eq1['m'] * y_int + eq1['b']
            else:
                eq_norm = eq2 if eq1['is_vertical'] else eq1
                eq_vert = eq1 if eq1['is_vertical'] else eq2
                den = 1 - (eq_vert['m'] * eq_norm['m'])
                if abs(den) > 1e-6:
                    x_int = (eq_vert['m'] * eq_norm['b'] + eq_vert['b']) / den
                    y_int = eq_norm['m'] * x_int + eq_norm['b']

            if x_int is not None and y_int is not None:
                intersections.append([x_int, y_int])
                if cls.LOG_INTERSECT == 1:
                    cls.sys_log(
                        f"Cruce {cls._disp_col(valid_cols[i])} y {cls._disp_col(valid_cols[i + 1])}: ({cls.fmt_f(x_int, 3)}, {cls.fmt_f(y_int, 3)})\n"
                    )

        last_key = valid_cols[-1]
        last_eq = equations[last_key]
        last_segment = segmentos[last_key]

        if last_eq['is_vertical']:
            y_final = last_segment[-1][1]
            x_final = last_eq['m'] * y_final + last_eq['b']
        else:
            x_final = last_segment[-1][0]
            y_final = last_eq['m'] * x_final + last_eq['b']
        intersections.append([x_final, y_final])
        if cls.LOG_INTERSECT == 1:
            cls.sys_log(f"Extremo Final: ({cls.fmt_f(x_final, 3)}, {cls.fmt_f(y_final, 3)})\n")

        return intersections

    # Alias por retrocompatibilidad si algún script externo llama a calcular_intersecciones
    calcular_intersecciones = calcular_intersecciones_con_extremos

    # ============================================================================
    # 3. CALCULAR DISTANCIAS Y ÁNGULOS
    # ============================================================================
    @classmethod
    def calcular_distancias_y_angulos(cls, intersecciones):
        distancias_a_recorrer = []
        angles_a_recorrer = []

        if cls.LOG_DIST_ANGLES == 1:
            cls.sys_log("\n[DISTANCIAS Y ÁNGULOS] Extrayendo métricas...")

        if len(intersecciones) < 2:
            return distancias_a_recorrer, angles_a_recorrer

        x1, y1 = intersecciones[0][0], intersecciones[0][1]
        prev_angle = math.atan2(-y1, -x1) * (180 / math.pi)

        for i in range(len(intersecciones) - 1):
            xa, ya = intersecciones[i][0], intersecciones[i][1]
            xb, yb = intersecciones[i + 1][0], intersecciones[i + 1][1]

            distancia = math.sqrt((xb - xa) ** 2 + (yb - ya) ** 2)
            distancias_a_recorrer.append(distancia)

            curr_angle = math.atan2(yb - ya, xb - xa) * (180 / math.pi)
            rel_angle = curr_angle - prev_angle

            if rel_angle > 180:
                rel_angle = rel_angle - 360
            elif rel_angle < -180:
                rel_angle = rel_angle + 360

            angles_a_recorrer.append(rel_angle)

            if cls.LOG_DIST_ANGLES == 1:
                cls.sys_log(
                    f"Tramo {i + 1}: Distancia = {cls.fmt_f(distancia, 3)}, Ángulo Relativo = {cls.fmt_f(rel_angle, 2)}°\n"
                )

            prev_angle = curr_angle

        return distancias_a_recorrer, angles_a_recorrer

    # ============================================================================
    # 4. CREATE PARAMS: Empaqueta los datos con su ID Original
    # ============================================================================
    @classmethod
    def create_params(cls, distancias, angulos, valid_cols, valid_segments):
        params = []

        if cls.LOG_PARAMS == 1:
            cls.sys_log("\n[PARAMS] Generando paquete final de parámetros:")

        for i in range(min(len(distancias), len(angulos))):
            distancia = distancias[i]
            angulo = angulos[i]
            original_col_id = valid_cols[i]

            # Empaquetado interno se mantiene intacto
            params.append([distancia, angulo, original_col_id])

            # Lógica exclusiva para impresión visual limpia
            if cls.LOG_PARAMS == 1:
                ang_str = "nil" if (i == 0) else f"{cls.fmt_f(angulo, 2)}°"
                dist_str = "nil"

                is_valid_seg = False
                if valid_segments:
                    if isinstance(valid_segments, dict):
                        is_valid_seg = bool(valid_segments.get(original_col_id, False))
                    elif isinstance(valid_segments, (list, tuple)) and 0 <= original_col_id < len(valid_segments):
                        is_valid_seg = bool(valid_segments[original_col_id])

                if is_valid_seg:
                    dist_str = f"{cls.fmt_f(distancia, 2)} cm"

                # Solo imprimimos si al menos uno de los dos es útil
                if ang_str != "nil" or dist_str != "nil":
                    cls.sys_log(
                        f"  -> Param {i + 1} (Col {cls._disp_col(original_col_id)}) -> Ángulo: {ang_str}, Distancia: {dist_str}"
                    )

        return params

    # ============================================================================
    # FUNCIÓN PRINCIPAL EXPORTADA: Ejecuta toda la cadena geométrica
    # ============================================================================
    @classmethod
    def process_segments(cls, segmentos_movement, valid_segments):
        equations, valid_cols = cls.apply(segmentos_movement)
        equations, valid_cols = cls.merge_collinear_segments(equations, segmentos_movement, valid_cols)
        intersections = cls.calcular_intersecciones_con_extremos(equations, segmentos_movement, valid_cols)
        distancias, angulos = cls.calcular_distancias_y_angulos(intersections)

        # Le pasamos la lista de booleanos a create_params
        params = cls.create_params(distancias, angulos, valid_cols, valid_segments)

        if cls.LOG_APPLY == 1 or cls.LOG_MERGE == 1 or cls.LOG_INTERSECT == 1 or cls.LOG_DIST_ANGLES == 1 or cls.LOG_PARAMS == 1:
            cls.sys_log("============================================\n")

        return params