import math

class SegmentParameterizer:
    TOLERANCIA_ANGULO_FUSION = 15.0

    @staticmethod
    def compute_regression(points):
        sum_x = sum(p[0] for p in points)
        sum_y = sum(p[1] for p in points)
        n = len(points)
        mean_x, mean_y = sum_x / n, sum_y / n
        s_xx = sum((p[0] - mean_x)**2 for p in points)
        s_yy = sum((p[1] - mean_y)**2 for p in points)
        s_xy = sum((p[0] - mean_x)*(p[1] - mean_y) for p in points)
        
        if s_yy > s_xx:
            is_vertical = True
            m = 0 if s_yy == 0 else s_xy / s_yy
            b = mean_x - (m * mean_y)
        else:
            is_vertical = False
            m = 0 if s_xx == 0 else s_xy / s_xx
            b = mean_y - (m * mean_x)
        return {'m': m, 'b': b, 'is_vertical': is_vertical, 'points_count': n}

    @staticmethod
    def get_line_angle(eq):
        angle = math.degrees(math.atan2(1, eq['m'])) if eq['is_vertical'] else math.degrees(math.atan2(eq['m'], 1))
        if angle < 0: angle += 180
        if angle >= 180: angle -= 180
        return angle

    @classmethod
    def apply(cls, segmentos):
        equations, valid_cols = {}, []
        for col, points in enumerate(segmentos):
            if len(points) > 3:
                equations[col] = cls.compute_regression(points)
                valid_cols.append(col)
        return equations, valid_cols

    @classmethod
    def merge_collinear_segments(cls, equations, segmentos, valid_cols):
        if len(valid_cols) < 2: return equations, valid_cols
        i = 0
        while i < len(valid_cols) - 1:
            col1, col2 = valid_cols[i], valid_cols[i+1]
            angle1, angle2 = cls.get_line_angle(equations[col1]), cls.get_line_angle(equations[col2])
            diff = abs(angle1 - angle2)
            if diff > 90: diff = 180 - diff
            if diff <= cls.TOLERANCIA_ANGULO_FUSION:
                segmentos[col1].extend(segmentos[col2])
                equations[col1] = cls.compute_regression(segmentos[col1])
                valid_cols.pop(i + 1)
                del equations[col2]
            else:
                i += 1
        return equations, valid_cols

    @classmethod
    def calcular_intersecciones(cls, equations, segmentos, valid_cols):
        intersections = []
        if not valid_cols: return intersections
        
        first_key, last_key = valid_cols[0], valid_cols[-1]
        eq1, seg1 = equations[first_key], segmentos[first_key]
        
        if eq1['is_vertical']:
            y_in = seg1[0][1]
            x_in = eq1['m'] * y_in + eq1['b']
        else:
            x_in = seg1[0][0]
            y_in = eq1['m'] * x_in + eq1['b']
        intersections.append([x_in, y_in])
        
        for i in range(len(valid_cols) - 1):
            e1, e2 = equations[valid_cols[i]], equations[valid_cols[i+1]]
            x_int, y_int = None, None
            if not e1['is_vertical'] and not e2['is_vertical']:
                den = e1['m'] - e2['m']
                if abs(den) > 1e-6:
                    x_int = (e2['b'] - e1['b']) / den
                    y_int = e1['m'] * x_int + e1['b']
            elif e1['is_vertical'] and e2['is_vertical']:
                den = e1['m'] - e2['m']
                if abs(den) > 1e-6:
                    y_int = (e2['b'] - e1['b']) / den
                    x_int = e1['m'] * y_int + e1['b']
            else:
                e_norm, e_vert = (e2, e1) if e1['is_vertical'] else (e1, e2)
                den = 1 - (e_vert['m'] * e_norm['m'])
                if abs(den) > 1e-6:
                    x_int = (e_vert['m'] * e_norm['b'] + e_vert['b']) / den
                    y_int = e_norm['m'] * x_int + e_norm['b']
            if x_int is not None and y_int is not None:
                intersections.append([x_int, y_int])
                
        eq_l, seg_l = equations[last_key], segmentos[last_key]
        if eq_l['is_vertical']:
            y_out = seg_l[-1][1]
            x_out = eq_l['m'] * y_out + eq_l['b']
        else:
            x_out = seg_l[-1][0]
            y_out = eq_l['m'] * x_out + eq_l['b']
        intersections.append([x_out, y_out])
        
        return intersections

    @classmethod
    def calcular_distancias_y_angulos(cls, intersecciones):
        dists, angs = [], []
        if len(intersecciones) < 2: return dists, angs
        x1, y1 = intersecciones[0][0], intersecciones[0][1]
        prev_angle = math.degrees(math.atan2(-y1, -x1))
        
        for i in range(len(intersecciones) - 1):
            xa, ya = intersecciones[i][0], intersecciones[i][1]
            xb, yb = intersecciones[i+1][0], intersecciones[i+1][1]
            dist = math.sqrt((xb - xa)**2 + (yb - ya)**2)
            dists.append(dist)
            
            curr_angle = math.degrees(math.atan2(yb - ya, xb - xa))
            rel_angle = curr_angle - prev_angle
            if rel_angle > 180: rel_angle -= 360
            elif rel_angle < -180: rel_angle += 360
            angs.append(rel_angle)
            prev_angle = curr_angle
            
        return dists, angs

    @classmethod
    def process_segments(cls, segmentos_movement, valid_segments):
        equations, valid_cols = cls.apply(segmentos_movement)
        equations, valid_cols = cls.merge_collinear_segments(equations, segmentos_movement, valid_cols)
        intersections = cls.calcular_intersecciones(equations, segmentos_movement, valid_cols)
        dists, angs = cls.calcular_distancias_y_angulos(intersections)
        
        params = []
        for i in range(min(len(dists), len(angs))):
            col_id = valid_cols[i]
            params.append([dists[i], angs[i], col_id])
        return params
