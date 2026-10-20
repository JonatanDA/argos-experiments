import math
from Observer.Point_Cloud_Segmenter import PointCloudSegmenter
from Observer.Segment_Parameterizer import SegmentParameterizer
from Observer.Parameter_Storage import ParameterStorage
from Observer.Global_Trajectory_Reconstructor import GlobalTrajectoryReconstructor
from Observer.ImitationQuality import ImitationQuality

# Global config
cfg = {'CONFIG': {'TOL_DIST': 15, 'TOL_ANG': 15}, 'use_subsampling': False}
seg_cfg = {'distance_treshold': 15, 'limite_maximo_ventana': 30, 'use_subsampling': False}
distance_treshold = 20
max_patience = 160 * 4

current_patience = max_patience
srt_anterior_data = {}
master_tracking = {}
all_params = []
param_store = ParameterStorage()

def initialize_master_tracking(k, angle_deg, distance, ty, obs_ang_offset_global, x_mark, y_mark):
    angle_rad = math.radians(angle_deg)
    ang_total = angle_rad + obs_ang_offset_global
    x_rel = distance * math.cos(ang_total)
    y_rel = distance * math.sin(ang_total)
    x = x_rel + x_mark
    y = y_rel + y_mark
    master_tracking[k] = {
        'raw_x': [x], 'raw_y': [y], 'raw_type': [ty],
        'Pattern_start_detected': (ty == 1),
        'End_of_pattern_detected': (ty == 3),
        'processed': False,
        'segmentos': {'movement': [], 'type': [], 'valid': []},
        'params': []
    }

def process_single_observation(k, track):
    clean_mov, clean_type, clean_valid, p_start, p_end = PointCloudSegmenter.process(k, track['raw_x'], track['raw_y'], track['raw_type'], seg_cfg)
    track['segmentos']['movement'] = clean_mov
    track['segmentos']['type'] = clean_type
    track['segmentos']['valid'] = clean_valid
    if p_start: track['Pattern_start_detected'] = True
    if p_end: track['End_of_pattern_detected'] = True
    
    if len(clean_mov) > 1:
        track['params'] = SegmentParameterizer.process_segments(clean_mov, clean_valid)
        r_lvl, r_cnt, h_lvl = param_store.package_and_add(track['params'], clean_valid, cfg)
        print(f"-> Datos enviados al Storage: Nivel {r_lvl}")

def check_patience_and_reconstruct():
    global current_patience, all_params
    if param_store.is_done(): return
    
    current_patience -= 1
    if current_patience <= 0:
        print("[ALERTA] Paciencia agotada. Ejecutando Reconstrucción...")
        datos_guardados = param_store.get_stored_data()
        exito, linea, nuevos, winner_ids = GlobalTrajectoryReconstructor.run("Apprentice", datos_guardados, cfg, len(all_params))
        if exito:
            param_store.set_done()
            print(f"Resumen Reconstrucción: {linea}")
            for comp in nuevos:
                all_params.append(comp)
        current_patience = max_patience

def procesar_datos_logicos_y_reconstruccion(datos_fisicos):
    yellow_blobs = datos_fisicos['yellow_blobs']
    green_blobs = datos_fisicos['green_blobs']
    obs_ang_offset_global = datos_fisicos['obs_ang_offset_global']
    x_mark = datos_fisicos['x_mark']
    y_mark = datos_fisicos['y_mark']
    
    updated_anterior_indices = {}
    matched_new_indices = {}
    candidatos = []
    
    # B: Actualización de maestros
    for j, new_data in enumerate(yellow_blobs):
        ang_total_blob = math.radians(new_data['angle']) + obs_ang_offset_global
        bx_rel = new_data['distance'] * math.cos(ang_total_blob)
        by_rel = new_data['distance'] * math.sin(ang_total_blob)
        bx_global, by_global = bx_rel + x_mark, by_rel + y_mark
        
        for k, anterior_data in srt_anterior_data.items():
            if 'angle' in anterior_data and k in master_tracking and master_tracking[k]['raw_x']:
                mx_global = master_tracking[k]['raw_x'][-1]
                my_global = master_tracking[k]['raw_y'][-1]
                err = math.sqrt((bx_global - mx_global)**2 + (by_global - my_global)**2)
                if err <= distance_treshold:
                    candidatos.append({'blob_idx': j, 'master_idx': k, 'error': err})
                    
    candidatos.sort(key=lambda x: x['error'])
    for cand in candidatos:
        j, k = cand['blob_idx'], cand['master_idx']
        if j not in matched_new_indices and k not in updated_anterior_indices:
            srt_anterior_data[k].update(yellow_blobs[j])
            updated_anterior_indices[k] = True
            matched_new_indices[j] = True
            
    # C: Crear nuevos maestros
    for j, new_data in enumerate(yellow_blobs):
        if j not in matched_new_indices:
            new_k = len(srt_anterior_data) + 1
            srt_anterior_data[new_k] = new_data.copy()
            if 'type' not in srt_anterior_data[new_k]: srt_anterior_data[new_k]['type'] = 2
            updated_anterior_indices[new_k] = True
            initialize_master_tracking(new_k, new_data['angle'], new_data['distance'], srt_anterior_data[new_k]['type'], obs_ang_offset_global, x_mark, y_mark)

    # D: Blobs Verdes (Fin)
    for green_blob in green_blobs:
        ang_total_green = math.radians(green_blob['angle']) + obs_ang_offset_global
        gbx_rel, gby_rel = green_blob['distance'] * math.cos(ang_total_green), green_blob['distance'] * math.sin(ang_total_green)
        gbx, gby = gbx_rel + x_mark, gby_rel + y_mark
        for k, md in srt_anterior_data.items():
            if 'angle' in md and k not in updated_anterior_indices and k in master_tracking and master_tracking[k]['raw_x']:
                mx, my = master_tracking[k]['raw_x'][-1], master_tracking[k]['raw_y'][-1]
                if math.sqrt((gbx - mx)**2 + (gby - my)**2) <= distance_treshold:
                    srt_anterior_data[k]['type'] = 3
                    updated_anterior_indices[k] = True
                    master_tracking[k]['End_of_pattern_detected'] = True
                    break
                    
    # G & H: Grabación y Pérdidas
    for k in list(srt_anterior_data.keys()):
        track = master_tracking.get(k)
        is_lost = ('angle' in srt_anterior_data[k] and k not in updated_anterior_indices)
        is_finished = track and track['End_of_pattern_detected']
        
        if (is_lost or is_finished) and track and not track['processed']:
            process_single_observation(k, track)
            track['processed'] = True
            srt_anterior_data[k] = {} # clear
            
    for k in updated_anterior_indices:
        md = srt_anterior_data[k]
        track = master_tracking[k]
        if md and track and not track['processed']:
            ang_tot = math.radians(md['angle']) + obs_ang_offset_global
            x_new = md['distance'] * math.cos(ang_tot) + x_mark
            y_new = md['distance'] * math.sin(ang_tot) + y_mark
            track['raw_x'].append(x_new)
            track['raw_y'].append(y_new)
            track['raw_type'].append(md['type'])
            if md['type'] == 1: track['Pattern_start_detected'] = True
            if md['type'] == 3: track['End_of_pattern_detected'] = True
