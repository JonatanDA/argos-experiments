import math

class GlobalTrajectoryReconstructor:
    @staticmethod
    def is_fragment_closed(struct, cfg):
        values, current_type = struct['value'], struct['type']
        temp_dist, temp_ang, expecting_dist = 0, 0, (current_type == 0)
        params = []
        for val in values:
            if expecting_dist:
                temp_dist = val
                params.append([temp_dist, temp_ang])
                temp_ang = 0; expecting_dist = False
            else:
                temp_ang = val; expecting_dist = True
                
        if len(params) < 3: return False
        x, y, theta_deg = 0, 0, 0
        for p in params:
            theta_deg += p[1]
            rad = math.radians(theta_deg)
            x += p[0] * math.cos(rad)
            y += p[0] * math.sin(rad)
        tol = cfg['CONFIG'].get('TOL_DIST', 15.0) if 'CONFIG' in cfg else 15.0
        return math.sqrt(x**2 + y**2) <= tol

    @staticmethod
    def process_numeric_result(result_struct, patron_id, cfg):
        values, current_type = result_struct['value'], result_struct['type']
        temp_dist, temp_ang, expecting_dist = 0, 0, (current_type == 0)
        params = []
        for val in values:
            if expecting_dist:
                temp_dist = val
                params.append([temp_dist, temp_ang])
                temp_ang = 0; expecting_dist = False
            else:
                temp_ang = val; expecting_dist = True
                
        if len(params) >= 3:
            x, y, theta_deg, min_dist, best_closing_index = 0, 0, 0, float('inf'), -1
            tol_cierre = cfg['CONFIG'].get('TOL_DIST', 5.0) if 'CONFIG' in cfg else 5.0
            for i, p in enumerate(params):
                theta_deg += p[1]
                rad = math.radians(theta_deg)
                x += p[0] * math.cos(rad)
                y += p[0] * math.sin(rad)
                dist = math.sqrt(x**2 + y**2)
                if i >= 2 and dist < min_dist:
                    min_dist = dist
                    best_closing_index = i
            if best_closing_index != -1 and min_dist <= tol_cierre and best_closing_index < len(params) - 1:
                params = params[:best_closing_index + 1]
        return params

    @classmethod
    def run(cls, robot_id, stored_data, cfg, offset_comportamientos):
        # En una versión simplificada forzamos el empaquetado del resultado más largo o cerrado
        if not stored_data: return False, "", [], {}
        
        best_struct = max(stored_data, key=lambda x: len(x['value']))
        pattern_params = cls.process_numeric_result(best_struct, offset_comportamientos + 1, cfg)
        
        linea_resumen = f"{robot_id} & Imitado&p{offset_comportamientos + 1}"
        for i, seg in enumerate(pattern_params):
            if i > 0: linea_resumen += ","
            linea_resumen += f"({seg[1]:.2f},{seg[0]:.2f})"
            
        winner_ids_map = best_struct.get('consumed_ids', {best_struct.get('id', '1'): True})
        return True, linea_resumen, [pattern_params], winner_ids_map
