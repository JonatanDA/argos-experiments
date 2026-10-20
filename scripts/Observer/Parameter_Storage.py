# ============================================================================
# Archivo: Functions/Observer/Parameter_Storage.py
# ============================================================================

import math


class ParameterStorage:
    USE_GLOBAL_MEMORY = True

    # NUEVA VARIABLE DE LOGGEO INDEPENDIENTE:
    # 0: Sin logs
    # 1: Logs básicos (Cuando añade o refina datos)
    # 2: Logs detallados (Depuración de memoria global paso a paso)
    PS_LOG_MODE = 2

    def __init__(self):
        self.USE_GLOBAL_MEMORY = ParameterStorage.USE_GLOBAL_MEMORY

        # NUEVA VARIABLE DE LOGGEO INDEPENDIENTE:
        # 0: Sin logs
        # 1: Logs básicos (Cuando añade o refina datos)
        # 2: Logs detallados (Depuración de memoria global paso a paso)
        self.PS_LOG_MODE = ParameterStorage.PS_LOG_MODE

        self.DataBuffer = {
            'stored': [],
            'global_stored': []
        }
        self.Global_Reconstruction_Done = False
        self.local_id_counter = 0

    def is_done(self):
        return self.Global_Reconstruction_Done

    def set_done(self):
        self.Global_Reconstruction_Done = True

    def get_stored_data(self):
        return self.DataBuffer['stored']

    def get_global_stored_data(self):
        return self.DataBuffer['global_stored']

    def log(self, msg):
        print(f"[PYTHON GESTOR] {msg}", flush=True)

    def fmt_f(self, val, decimals=2):
        return f"{val:.{decimals}f}".replace('.', ',')

    def fmt_struct(self, s):
        if not s:
            return "nil"
        vals = [self.fmt_f(v, 2) for v in s.get('value', [])]
        t_str = "Ang" if s.get('type') == 1 else "Dist"
        count = s.get('count')
        count_str = f" [x{count}]" if (count and count > 1) else ""
        return f"({t_str}){count_str} {{{', '.join(vals)}}}"

    def simple_fmt(self, val_table):
        vals = [self.fmt_f(v, 2) for v in val_table]
        return ",".join(vals)

    def get_val_type(self, struct_type, index):
        # En Lua (1-based): index % 2 ~= 0 es impar (1, 3, 5...).
        # En Python (0-based): index % 2 == 0 corresponde a las mismas posiciones (0, 2, 4...).
        is_odd = (index % 2 == 0)
        if struct_type == 1:
            return "A" if is_odd else "D"
        else:
            return "D" if is_odd else "A"

    def check_match(self, long_dat, short_dat, start_idx, cfg):
        if self.get_val_type(long_dat['type'], start_idx) != self.get_val_type(short_dat['type'], 0):
            return False
        for k in range(len(short_dat['value'])):
            l_idx = start_idx + k
            v_type = self.get_val_type(short_dat['type'], k)
            diff = abs(long_dat['value'][l_idx] - short_dat['value'][k])
            tol_cfg = cfg.get('CONFIG', {}) if isinstance(cfg, dict) else {}
            tol = tol_cfg.get('TOL_ANG', 15) if v_type == "A" else tol_cfg.get('TOL_DIST', 15)

            if diff > (tol * 1.5):
                return False
        return True

    def merge_into_buffer(self, buffer, new_dat, cfg, do_modify):
        found_level = 0
        exist_ref = None

        for exist in buffer:
            if len(exist['value']) == len(new_dat['value']) and exist['type'] == new_dat['type']:
                if self.check_match(exist, new_dat, 0, cfg):
                    found_level = 2
                    if do_modify:
                        c1 = exist.get('count') or 1
                        c2 = new_dat.get('count') or 1
                        new_total = c1 + c2

                        for k in range(len(exist['value'])):
                            exist['value'][k] = ((exist['value'][k] * c1) + (new_dat['value'][k] * c2)) / new_total
                        exist['count'] = new_total
                        if 'same_size_merges' not in exist:
                            exist['same_size_merges'] = []
                        exist['same_size_merges'].append(self.simple_fmt(new_dat['value']))
                    exist_ref = exist
                    break

            if found_level == 0 and len(new_dat['value']) > len(exist['value']):
                for start_idx in range(len(new_dat['value']) - len(exist['value']) + 1):
                    if self.check_match(new_dat, exist, start_idx, cfg):
                        found_level = 1
                        if do_modify:
                            c1 = exist.get('count') or 1
                            c2 = new_dat.get('count') or 1
                            new_total = c1 + c2

                            merged_values = []
                            for k in range(len(new_dat['value'])):
                                if k >= start_idx and k < start_idx + len(exist['value']):
                                    e_idx = k - start_idx
                                    avg_val = ((exist['value'][e_idx] * c1) + (new_dat['value'][k] * c2)) / new_total
                                    merged_values.append(avg_val)
                                else:
                                    merged_values.append(new_dat['value'][k])
                            exist['value'] = merged_values
                            exist['type'] = new_dat['type']
                            exist['count'] = new_total
                            if 'diff_size_merges' not in exist:
                                exist['diff_size_merges'] = []
                            exist['diff_size_merges'].append(self.simple_fmt(new_dat['value']))
                        exist_ref = exist
                        break
                if found_level > 0:
                    break

            if found_level == 0 and len(exist['value']) > len(new_dat['value']):
                for start_idx in range(len(exist['value']) - len(new_dat['value']) + 1):
                    if self.check_match(exist, new_dat, start_idx, cfg):
                        found_level = 1
                        if do_modify:
                            c1 = exist.get('count') or 1
                            c2 = new_dat.get('count') or 1
                            new_total = c1 + c2

                            for k in range(len(new_dat['value'])):
                                e_idx = start_idx + k
                                exist['value'][e_idx] = ((exist['value'][e_idx] * c1) + (new_dat['value'][k] * c2)) / new_total
                            exist['count'] = new_total
                            if 'diff_size_merges' not in exist:
                                exist['diff_size_merges'] = []
                            exist['diff_size_merges'].append(self.simple_fmt(new_dat['value']))
                        exist_ref = exist
                        break
                if found_level > 0:
                    break

        return found_level, exist_ref

    def add(self, new_dat, cfg):
        if self.Global_Reconstruction_Done:
            return 0, 0, 0

        self.local_id_counter = self.local_id_counter + 1
        new_dat['id'] = str(self.local_id_counter)

        history_decay_level = 0
        new_dat['count'] = 1
        new_dat['decay_lvl1_count'] = 0
        new_dat['decay_lvl2_count'] = 0
        new_dat['decay_lvl3_count'] = 0

        local_level, local_exist = self.merge_into_buffer(self.DataBuffer['stored'], new_dat, cfg, True)

        final_local_count = 1
        if local_level > 0:
            final_local_count = local_exist['count']
            if not local_exist.get('consumed_ids'):
                local_exist['consumed_ids'] = {local_exist['id']: True}
            local_exist['consumed_ids'][new_dat['id']] = True
        else:
            new_v = [v for v in new_dat['value']]
            entry = {
                'value': new_v, 'type': new_dat['type'], 'count': 1,
                'id': new_dat['id'], 'consumed_ids': {new_dat['id']: True},
                'same_size_merges': [], 'diff_size_merges': [],
                'decay_lvl1_count': 0, 'decay_lvl2_count': 0, 'decay_lvl3_count': 0
            }
            self.DataBuffer['stored'].append(entry)
            local_exist = entry

        global_level = 0
        global_exist = None
        if self.USE_GLOBAL_MEMORY:
            global_level, global_exist = self.merge_into_buffer(self.DataBuffer['global_stored'], new_dat, cfg, False)

        restore_level = 3
        restore_count = final_local_count

        if global_level > 0:
            restore_level = global_level
            restore_count = final_local_count + global_exist['count'] - 1

            if global_level == 2:
                global_exist['decay_lvl2_count'] = (global_exist.get('decay_lvl2_count') or 0) + 1
                history_decay_level = global_exist['decay_lvl2_count'] - 1
            else:
                global_exist['decay_lvl1_count'] = (global_exist.get('decay_lvl1_count') or 0) + 1
                history_decay_level = global_exist['decay_lvl1_count'] - 1

            if self.PS_LOG_MODE >= 1:
                lbl = "Iguales" if (global_level == 2) else "Similares"
                self.log(f"[INFO] Fragmento Refinado (Globalmente conocido [{lbl}], total x{restore_count}) [NIVEL {restore_level}]: {self.fmt_struct(local_exist)}")

        elif local_level > 0:
            restore_level = local_level
            restore_count = final_local_count

            if local_level == 2:
                local_exist['decay_lvl2_count'] = (local_exist.get('decay_lvl2_count') or 0) + 1
                history_decay_level = local_exist['decay_lvl2_count'] - 1
            else:
                local_exist['decay_lvl1_count'] = (local_exist.get('decay_lvl1_count') or 0) + 1
                history_decay_level = local_exist['decay_lvl1_count'] - 1

            if self.PS_LOG_MODE >= 1:
                lbl = "Iguales" if (local_level == 2) else "Similares"
                self.log(f"[INFO] Fragmento Refinado (Promedio Local [{lbl}] de {restore_count}) [NIVEL {restore_level}]: {self.fmt_struct(local_exist)}")

        else:
            restore_level = 3
            restore_count = final_local_count
            local_exist['decay_lvl3_count'] = (local_exist.get('decay_lvl3_count') or 0) + 1
            history_decay_level = local_exist['decay_lvl3_count'] - 1

            if self.PS_LOG_MODE >= 1:
                self.log(f"[INFO] Fragmento Añadido (Totalmente Nuevo) [NIVEL 3]: {self.fmt_struct(local_exist)}")

        return restore_level, restore_count, history_decay_level

    def package_and_add(self, params, valid_segments, cfg):
        if not params or len(params) == 0:
            return 0, 0, 0

        structure_values = []
        structure_type = None

        for i, param in enumerate(params):
            dist, ang, col_id = param[0], param[1], param[2]

            if isinstance(valid_segments, dict):
                is_segment_valid = valid_segments.get(col_id, False)
            elif isinstance(valid_segments, (list, tuple)):
                is_segment_valid = valid_segments[col_id] if (isinstance(col_id, int) and 0 <= col_id < len(valid_segments)) else False
            else:
                is_segment_valid = False

            if i > 0:
                structure_values.append(ang)
                if structure_type is None:
                    structure_type = 1

            if is_segment_valid:
                structure_values.append(dist)
                if structure_type is None:
                    structure_type = 0

        if len(structure_values) < 3:
            if self.PS_LOG_MODE >= 1:
                self.log(f"\n >> Fragmento ignorado: Estructura insuficiente ({len(structure_values)} parametros). Mínimo requerido: 3.")
            return 0, 0, 0

        if structure_type is not None:
            data_packet = {'value': structure_values, 'type': structure_type}
            if self.PS_LOG_MODE >= 1:
                self.log(f"\n >> Fragmentos empaquetados y enviados al ParameterStorage. (Total values: {len(structure_values)})")
            return self.add(data_packet, cfg)
        return 0, 0, 0

    def print_global_buffer(self, cfg=None):
        if self.PS_LOG_MODE >= 2:
            self.log("\n=========================================")
            self.log(" TABLA DEL BUFFER GLOBAL (HISTORIAL DE APRENDIZAJE)")
            self.log("=========================================")
            if len(self.DataBuffer['global_stored']) == 0:
                self.log(" >> [Memoria Global Vacía]")
            else:
                for i, v in enumerate(self.DataBuffer['global_stored']):
                    self.log(f" [{i + 1}] {self.fmt_struct(v)}")
            self.log("=========================================\n")

    def clear(self, cfg=None, valid_ids_table=None):
        if not cfg:
            cfg = {'CONFIG': {'TOL_ANG': 0.2, 'TOL_DIST': 2.0}, 'LOG_MODE': 1}

        if self.PS_LOG_MODE >= 2:
            self.log("\n=========================================")
            self.log(" [DEPURACIÓN] INICIANDO TRANSFERENCIA A MEMORIA GLOBAL")

        valid_str = ""
        if valid_ids_table:
            count = 0
            for k in valid_ids_table:
                valid_str = valid_str + f"{k}, "
                count = count + 1
            if count == 0:
                valid_str = "VACÍA (Seguridad: Se aceptarán todos)"
                valid_ids_table = None
        else:
            valid_str = "NIL (Se aceptarán todos)"

        if self.PS_LOG_MODE >= 2:
            self.log(" IDs Ganadores recibidos de forma nativa: {" + valid_str + "}")

        if self.USE_GLOBAL_MEMORY:
            for local_dat in self.DataBuffer['stored']:
                local_ids_str = ""
                for c_id in (local_dat.get('consumed_ids') or {}):
                    local_ids_str = local_ids_str + f"{c_id}, "

                if self.PS_LOG_MODE >= 2:
                    self.log(f" Evaluando fragmento local ID [{local_dat['id']}] con sub-IDs: {{{local_ids_str}}}")

                is_valid = False
                matched_id = ""
                if not valid_ids_table:
                    is_valid = True
                else:
                    for c_id in (local_dat.get('consumed_ids') or {}):
                        if valid_ids_table.get(c_id) if isinstance(valid_ids_table, dict) else (c_id in valid_ids_table):
                            is_valid = True
                            matched_id = c_id
                            break

                if is_valid:
                    if self.PS_LOG_MODE >= 2:
                        self.log(f" -> Fragmento ACEPTADO (ID ganador referencial: {matched_id}).")
                    found_level, global_exist = self.merge_into_buffer(self.DataBuffer['global_stored'], local_dat, cfg, True)

                    if found_level == 0:
                        if self.PS_LOG_MODE >= 2:
                            self.log("Guardado como NUEVO en memoria global.\n")
                        new_v = [v for v in local_dat['value']]
                        c_ids = {id_k: id_v for id_k, id_v in (local_dat.get('consumed_ids') or {}).items()}

                        entry = {
                            'value': new_v, 'type': local_dat['type'], 'count': local_dat['count'],
                            'id': local_dat['id'], 'consumed_ids': c_ids,
                            'same_size_merges': [], 'diff_size_merges': [],
                            'decay_lvl1_count': local_dat.get('decay_lvl1_count', 0),
                            'decay_lvl2_count': local_dat.get('decay_lvl2_count', 0),
                            'decay_lvl3_count': local_dat.get('decay_lvl3_count', 0)
                        }
                        self.DataBuffer['global_stored'].append(entry)
                    else:
                        if self.PS_LOG_MODE >= 2:
                            self.log(f"Fusionado con éxito (Nivel {found_level}) con fragmento global ID [{global_exist['id']}].\n")
                        global_exist['decay_lvl1_count'] = (global_exist.get('decay_lvl1_count') or 0) + (local_dat.get('decay_lvl1_count') or 0)
                        global_exist['decay_lvl2_count'] = (global_exist.get('decay_lvl2_count') or 0) + (local_dat.get('decay_lvl2_count') or 0)
                        global_exist['decay_lvl3_count'] = (global_exist.get('decay_lvl3_count') or 0) + (local_dat.get('decay_lvl3_count') or 0)

                        for cid in (local_dat.get('consumed_ids') or {}):
                            global_exist['consumed_ids'][cid] = True
                else:
                    if self.PS_LOG_MODE >= 2:
                        self.log(" -> Fragmento RECHAZADO (Es ruido descartado).")
        else:
            self.DataBuffer['global_stored'] = []

        self.print_global_buffer(cfg)

        # Limpieza total para el próximo ciclo
        self.local_id_counter = 0
        self.DataBuffer['stored'] = []
        self.Global_Reconstruction_Done = False