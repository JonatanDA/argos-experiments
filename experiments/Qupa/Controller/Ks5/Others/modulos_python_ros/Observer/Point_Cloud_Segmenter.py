from Observer import math_helper
import math

class PointCloudSegmenter:
    paso_exploracion_ventana = 1
    factor_suficiencia_otsu = 1.0
    paquetes_lookahead_ventana = 5
    porcentaje_confirmacion_lookahead = 0.8
    nivel_estricto_dos_segmentos = 0

    @classmethod
    def get_segment_dist(cls, seg):
        if not seg or len(seg) < 1: return 0
        p1, p2 = seg[0], seg[-1]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return math.sqrt(dx*dx + dy*dy)

    @classmethod
    def detect_seg(cls, master_id, xx, yy, ttypes, cfg):
        total_puntos = len(xx)
        if total_puntos < 3: return [], [], None
        
        max_avance_permitido = min(max(total_puntos // 3, cls.paso_exploracion_ventana), cfg.get('limite_maximo_ventana', 60))
        resultados_exploracion = []
        max_separacion_global = -1
        
        for av in range(cls.paso_exploracion_ventana, int(max_avance_permitido) + 1, cls.paso_exploracion_ventana):
            temp_vars, temp_cx, temp_cy = [], [], []
            for i in range(total_puntos - av):
                std_dev, cen_x, cen_y = math_helper.get_window_stats(i, av, xx, yy)
                temp_vars.append(std_dev)
                temp_cx.append(cen_x)
                temp_cy.append(cen_y)
            umbral, separacion = math_helper.otsu_1d(temp_vars)
            resultados_exploracion.append({'av': av, 'umbral': umbral, 'separacion': separacion, 'vars': temp_vars, 'cx': temp_cx, 'cy': temp_cy})
            if separacion > max_separacion_global: max_separacion_global = separacion

        is_u_shape, first_valley_idx, decreased = False, 0, False
        for i in range(1, len(resultados_exploracion)):
            if resultados_exploracion[i]['separacion'] < resultados_exploracion[i-1]['separacion']:
                decreased = True
            elif decreased and resultados_exploracion[i]['separacion'] > resultados_exploracion[i-1]['separacion']:
                is_u_shape = True
                first_valley_idx = i - 1
                break

        start_search_idx = first_valley_idx if is_u_shape else 0
        umbral_aceptacion = max(res['separacion'] for res in resultados_exploracion[start_search_idx:]) * cls.factor_suficiencia_otsu if is_u_shape else max_separacion_global * cls.factor_suficiencia_otsu
        
        mejor = resultados_exploracion[start_search_idx]
        for i in range(start_search_idx, len(resultados_exploracion)):
            if resultados_exploracion[i]['separacion'] >= (umbral_aceptacion - 0.0001):
                mejor = resultados_exploracion[i]
                break

        mejor_avance, mejor_umbral, mejores_varianzas = mejor['av'], mejor['umbral'], mejor['vars']
        mejores_centroides_x, mejores_centroides_y = mejor['cx'], mejor['cy']
        
        segmentos, segmentos_type = [[]], [[]]
        k = 0
        ESTADO_MOVIMIENTO, ESTADO_BUSCANDO_VALLE = 1, 2
        estado_actual = ESTADO_MOVIMIENTO
        valle_min_var, valle_best_idx = 999999, -1
        valle_cen_x, valle_cen_y = 0, 0

        for i in range(len(mejores_varianzas)):
            std_dev, x, y, ty = mejores_varianzas[i], xx[i], yy[i], ttypes[i]
            cx, cy = mejores_centroides_x[i], mejores_centroides_y[i]
            if estado_actual == ESTADO_MOVIMIENTO:
                if std_dev > mejor_umbral:
                    segmentos[k].append([x, y]); segmentos_type[k].append(ty)
                else:
                    limite = min(i + cls.paquetes_lookahead_ventana, len(mejores_varianzas))
                    count_bajo = sum(1 for j in range(i, limite) if mejores_varianzas[j] <= mejor_umbral)
                    if (count_bajo / (limite - i)) >= cls.porcentaje_confirmacion_lookahead:
                        estado_actual = ESTADO_BUSCANDO_VALLE
                        valle_min_var, valle_best_idx, valle_cen_x, valle_cen_y = std_dev, i, cx, cy
                    else:
                        segmentos[k].append([x, y]); segmentos_type[k].append(ty)
            elif estado_actual == ESTADO_BUSCANDO_VALLE:
                if std_dev <= mejor_umbral:
                    if std_dev < valle_min_var:
                        valle_min_var, valle_best_idx, valle_cen_x, valle_cen_y = std_dev, i, cx, cy
                else:
                    limite = min(i + cls.paquetes_lookahead_ventana, len(mejores_varianzas))
                    count_sobre = sum(1 for j in range(i, limite) if mejores_varianzas[j] > mejor_umbral)
                    if (count_sobre / (limite - i)) >= cls.porcentaje_confirmacion_lookahead:
                        pivot = [valle_cen_x, valle_cen_y]; pivot_type = ttypes[valle_best_idx]
                        segmentos[k].append(pivot); segmentos_type[k].append(pivot_type)
                        k += 1
                        segmentos.append([pivot]); segmentos_type.append([pivot_type])
                        segmentos[k].append([x, y]); segmentos_type[k].append(ty)
                        estado_actual = ESTADO_MOVIMIENTO

        if estado_actual == ESTADO_BUSCANDO_VALLE:
            pivot = [valle_cen_x, valle_cen_y]; pivot_type = ttypes[valle_best_idx]
            segmentos[k].append(pivot); segmentos_type[k].append(pivot_type)
            k += 1
            segmentos.append([pivot]); segmentos_type.append([pivot_type])
            
        for i in range(len(mejores_varianzas), total_puntos):
            segmentos[k].append([xx[i], yy[i]]); segmentos_type[k].append(ttypes[i])

        return segmentos, segmentos_type, {'mejor_separacion': mejor['separacion'], 'mejor_avance': mejor_avance, 'varianzas': mejores_varianzas}

    @classmethod
    def filter_and_validate_segments(cls, master_id, raw_seg_mov, raw_seg_type, cfg):
        n = len(raw_seg_mov)
        if n == 0: return [], [], []
        
        raw_start_is_static = cls.get_segment_dist(raw_seg_mov[0]) < cfg.get('distance_treshold', 20)
        raw_end_is_static = cls.get_segment_dist(raw_seg_mov[-1]) < cfg.get('distance_treshold', 20)
        
        final_mov, final_type, final_valid = [], [], []
        accum_x, accum_y, accum_count = 0, 0, 0
        
        for i in range(n):
            seg, types = raw_seg_mov[i], raw_seg_type[i]
            dist = cls.get_segment_dist(seg)
            if dist < cfg.get('distance_treshold', 20):
                accum_x += seg[0][0] + seg[-1][0]
                accum_y += seg[0][1] + seg[-1][1]
                accum_count += 2
            else:
                if accum_count > 0:
                    seg[0] = [accum_x / accum_count, accum_y / accum_count]
                    accum_x, accum_y, accum_count = 0, 0, 0
                final_mov.append(seg); final_type.append(types); final_valid.append(True)
                
        final_n = len(final_mov)
        if final_n > 0:
            if not raw_start_is_static: final_valid[0] = False
            if not raw_end_is_static: final_valid[-1] = False
            
        descartar_todo = False
        if final_n == 1:
            descartar_todo = True
        elif final_n == 2:
            if cls.nivel_estricto_dos_segmentos == 1 and not final_valid[0] and not final_valid[1]: descartar_todo = True
            elif cls.nivel_estricto_dos_segmentos == 2 and (not final_valid[0] or not final_valid[1]): descartar_todo = True
            
        if descartar_todo: return [], [], []
        return final_mov, final_type, final_valid

    @classmethod
    def process(cls, master_id, raw_x, raw_y, raw_type, cfg):
        procesado_x, procesado_y, procesado_type = raw_x, raw_y, raw_type
        if cfg.get('use_subsampling', False) and cfg.get('tamano_subsampling', 1) > 1:
            step = cfg['tamano_subsampling']
            procesado_x, procesado_y, procesado_type = raw_x[::step], raw_y[::step], raw_type[::step]
            
        raw_mov, r_type, extra_data = cls.detect_seg(master_id, procesado_x, procesado_y, procesado_type, cfg)
        clean_mov, clean_type, clean_valid = cls.filter_and_validate_segments(master_id, raw_mov, r_type, cfg)
        
        pattern_start, pattern_end = False, False
        if clean_type:
            if 1 in clean_type[0]: pattern_start = True
            if 3 in clean_type[-1]: pattern_end = True
            
        return clean_mov, clean_type, clean_valid, pattern_start, pattern_end
