import math

class ParameterStorage:
    def __init__(self):
        self.USE_GLOBAL_MEMORY = True
        self.PS_LOG_MODE = 2
        self.DataBuffer = {'stored': [], 'global_stored': []}
        self.Global_Reconstruction_Done = False
        self.local_id_counter = 0

    def is_done(self): return self.Global_Reconstruction_Done
    def set_done(self): self.Global_Reconstruction_Done = True
    def get_stored_data(self): return self.DataBuffer['stored']
    def get_global_stored_data(self): return self.DataBuffer['global_stored']

    def get_val_type(self, struct_type, index):
        is_odd = (index % 2 != 0)
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
            tol = cfg['CONFIG'].get('TOL_ANG', 15) if v_type == "A" else cfg['CONFIG'].get('TOL_DIST', 15)
            if diff > (tol * 1.5): return False
        return True

    def merge_into_buffer(self, buffer, new_dat, cfg, do_modify):
        found_level = 0
        exist_ref = None
        for exist in buffer:
            if len(exist['value']) == len(new_dat['value']) and exist['type'] == new_dat['type']:
                if self.check_match(exist, new_dat, 0, cfg):
                    found_level = 2
                    if do_modify:
                        c1 = exist.get('count', 1)
                        c2 = new_dat.get('count', 1)
                        new_total = c1 + c2
                        for k in range(len(exist['value'])):
                            exist['value'][k] = ((exist['value'][k] * c1) + (new_dat['value'][k] * c2)) / new_total
                        exist['count'] = new_total
                    exist_ref = exist
                    break

            if found_level == 0 and len(new_dat['value']) > len(exist['value']):
                for start_idx in range(len(new_dat['value']) - len(exist['value']) + 1):
                    if self.check_match(new_dat, exist, start_idx, cfg):
                        found_level = 1
                        if do_modify:
                            c1 = exist.get('count', 1)
                            c2 = new_dat.get('count', 1)
                            new_total = c1 + c2
                            merged_values = []
                            for k in range(len(new_dat['value'])):
                                if start_idx <= k < start_idx + len(exist['value']):
                                    e_idx = k - start_idx
                                    avg_val = ((exist['value'][e_idx] * c1) + (new_dat['value'][k] * c2)) / new_total
                                    merged_values.append(avg_val)
                                else:
                                    merged_values.append(new_dat['value'][k])
                            exist['value'] = merged_values
                            exist['type'] = new_dat['type']
                            exist['count'] = new_total
                        exist_ref = exist
                        break
                if found_level > 0: break

            if found_level == 0 and len(exist['value']) > len(new_dat['value']):
                for start_idx in range(len(exist['value']) - len(new_dat['value']) + 1):
                    if self.check_match(exist, new_dat, start_idx, cfg):
                        found_level = 1
                        if do_modify:
                            c1 = exist.get('count', 1)
                            c2 = new_dat.get('count', 1)
                            new_total = c1 + c2
                            for k in range(len(new_dat['value'])):
                                e_idx = start_idx + k
                                exist['value'][e_idx] = ((exist['value'][e_idx] * c1) + (new_dat['value'][k] * c2)) / new_total
                            exist['count'] = new_total
                        exist_ref = exist
                        break
                if found_level > 0: break
        return found_level, exist_ref

    def add(self, new_dat, cfg):
        if self.Global_Reconstruction_Done: return 0, 0, 0
        self.local_id_counter += 1
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
            if 'consumed_ids' not in local_exist: local_exist['consumed_ids'] = {local_exist['id']: True}
            local_exist['consumed_ids'][new_dat['id']] = True
        else:
            entry = {
                'value': list(new_dat['value']), 'type': new_dat['type'], 'count': 1,
                'id': new_dat['id'], 'consumed_ids': {new_dat['id']: True},
                'decay_lvl1_count': 0, 'decay_lvl2_count': 0, 'decay_lvl3_count': 0
            }
            self.DataBuffer['stored'].append(entry)
            local_exist = entry

        global_level, global_exist = 0, None
        if self.USE_GLOBAL_MEMORY:
            global_level, global_exist = self.merge_into_buffer(self.DataBuffer['global_stored'], new_dat, cfg, False)

        restore_level, restore_count = 3, final_local_count
        if global_level > 0:
            restore_level = global_level
            restore_count = final_local_count + global_exist['count'] - 1
            if global_level == 2:
                global_exist['decay_lvl2_count'] = global_exist.get('decay_lvl2_count', 0) + 1
                history_decay_level = global_exist['decay_lvl2_count'] - 1
            else:
                global_exist['decay_lvl1_count'] = global_exist.get('decay_lvl1_count', 0) + 1
                history_decay_level = global_exist['decay_lvl1_count'] - 1
        elif local_level > 0:
            restore_level = local_level
            if local_level == 2:
                local_exist['decay_lvl2_count'] = local_exist.get('decay_lvl2_count', 0) + 1
                history_decay_level = local_exist['decay_lvl2_count'] - 1
            else:
                local_exist['decay_lvl1_count'] = local_exist.get('decay_lvl1_count', 0) + 1
                history_decay_level = local_exist['decay_lvl1_count'] - 1
        else:
            local_exist['decay_lvl3_count'] = local_exist.get('decay_lvl3_count', 0) + 1
            history_decay_level = local_exist['decay_lvl3_count'] - 1

        return restore_level, restore_count, history_decay_level

    def package_and_add(self, params, valid_segments, cfg):
        if not params: return 0, 0, 0
        structure_values, structure_type = [], None
        for i, param in enumerate(params):
            dist, ang, col_id = param[0], param[1], param[2]
            is_segment_valid = valid_segments.get(col_id, False)
            if i > 0:
                structure_values.append(ang)
                if structure_type is None: structure_type = 1
            if is_segment_valid:
                structure_values.append(dist)
                if structure_type is None: structure_type = 0
        if len(structure_values) < 3: return 0, 0, 0
        if structure_type is not None:
            data_packet = {'value': structure_values, 'type': structure_type}
            return self.add(data_packet, cfg)
        return 0, 0, 0

    def clear(self, cfg, valid_ids_table):
        if self.USE_GLOBAL_MEMORY:
            for local_dat in self.DataBuffer['stored']:
                is_valid, matched_id = False, ""
                if valid_ids_table is None:
                    is_valid = True
                else:
                    for c_id in local_dat.get('consumed_ids', {}):
                        if c_id in valid_ids_table:
                            is_valid = True
                            matched_id = c_id
                            break
                if is_valid:
                    found_level, global_exist = self.merge_into_buffer(self.DataBuffer['global_stored'], local_dat, cfg, True)
                    if found_level == 0:
                        entry = {
                            'value': list(local_dat['value']), 'type': local_dat['type'], 'count': local_dat['count'],
                            'id': local_dat['id'], 'consumed_ids': dict(local_dat.get('consumed_ids', {})),
                            'decay_lvl1_count': local_dat.get('decay_lvl1_count', 0),
                            'decay_lvl2_count': local_dat.get('decay_lvl2_count', 0),
                            'decay_lvl3_count': local_dat.get('decay_lvl3_count', 0)
                        }
                        self.DataBuffer['global_stored'].append(entry)
                    else:
                        global_exist['decay_lvl1_count'] = global_exist.get('decay_lvl1_count', 0) + local_dat.get('decay_lvl1_count', 0)
                        global_exist['decay_lvl2_count'] = global_exist.get('decay_lvl2_count', 0) + local_dat.get('decay_lvl2_count', 0)
                        global_exist['decay_lvl3_count'] = global_exist.get('decay_lvl3_count', 0) + local_dat.get('decay_lvl3_count', 0)
                        for cid in local_dat.get('consumed_ids', {}):
                            global_exist['consumed_ids'][cid] = True
        else:
            self.DataBuffer['global_stored'] = []

        self.local_id_counter = 0
        self.DataBuffer['stored'] = []
        self.Global_Reconstruction_Done = False
