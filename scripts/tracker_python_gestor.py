# ============================================================================
# Archivo: scripts/tracker_python_gestor.py
# ============================================================================

import re
import sys

from Python_Gestor import Ks5

def parse_observation_line(line):
    """Extrae los datos físicos de la trama [python];observation; enviada desde ARGoS."""
    # Formato: [python];observation;obs_ang_offset_global;x_mark;y_mark;yb_str;gb_str
    parts = line.strip().split(';')
    idx = parts.index('[python]')
    obs_ang_offset_global = float(parts[idx + 2].replace(',', '.'))
    x_mark = float(parts[idx + 3].replace(',', '.'))
    y_mark = float(parts[idx + 4].replace(',', '.'))

    yellow_blobs = []
    if len(parts) > idx + 5 and parts[idx + 5]:
        for b_str in parts[idx + 5].split('|'):
            if b_str:
                ang_s, dist_s, type_s = b_str.split(':')
                yellow_blobs.append({
                    'angle': float(ang_s.replace(',', '.')),
                    'distance': float(dist_s.replace(',', '.')),
                    'type': int(type_s),
                })

    green_blobs = []
    if len(parts) > idx + 6 and parts[idx + 6]:
        for b_str in parts[idx + 6].split('|'):
            if b_str:
                ang_s, dist_s = b_str.split(':')
                green_blobs.append({
                    'angle': float(ang_s.replace(',', '.')),
                    'distance': float(dist_s.replace(',', '.')),
                })

    return {
        'obs_ang_offset_global': obs_ang_offset_global,
        'x_mark': x_mark,
        'y_mark': y_mark,
        'yellow_blobs': yellow_blobs,
        'green_blobs': green_blobs,
    }


def main():
    Ks5.log_py("Iniciado. Escuchando datos de ARGoS en tiempo real...")
    pending_observation = None

    def flush_pending_observation():
        nonlocal pending_observation
        if pending_observation is not None:
            obs = pending_observation
            pending_observation = None
            Ks5.procesar_datos_logicos_y_reconstruccion(obs)
    for line in sys.stdin:
        # 1. Lectura de Timer=xx para sincronizar el reloj y la paciencia
        if "Timer=" in line:
            flush_pending_observation()
            m_timer = re.search(r"Timer=(\d+)", line)
            if m_timer:
                Ks5.registrar_timer(int(m_timer.group(1)))
            continue

        # 2. Detección de transferencia a memoria global en Lua (estado c4 == 2 al terminar Random Walk)
        if "[DEPURACIÓN] INICIANDO TRANSFERENCIA A MEMORIA GLOBAL" in line or (
            Ks5.esperando_limpieza_rw and "[trajectory];[offset];" in line
        ):
            pending_observation = None
            Ks5.ejecutar_limpieza_ciclo()
            continue

        # 3. Detección de fin de paciencia por señal explícita o por línea de [PACIENCIA](0)
        if "[python];patience_timeout" in line:
            pending_observation = None
            Ks5.check_patience_and_reconstruct(timeout_from_lua=True)
            continue

        if "[PACIENCIA](" in line and "segundos" in line:
            m_pac = re.search(r"\[PACIENCIA\]\(([-+]?[0-9]+(?:[.,][0-9]+)?)\)\s*segundos", line)
            if m_pac:
                pac_lua_val = float(m_pac.group(1).replace(',', '.'))
                if pac_lua_val <= 0.0:
                    pending_observation = None
                    Ks5.check_patience_and_reconstruct(timeout_from_lua=True)
                else:
                    flush_pending_observation()
                continue

        # 4. Recepción de datos de observación del tick actual
        if "[python];observation;" in line:
            flush_pending_observation()
            Ks5.preparar_ciclo_observacion()
            Ks5.check_patience_and_reconstruct(timeout_from_lua=False)

            try:
                pending_observation = parse_observation_line(line)
            except Exception as e:
                Ks5.log_py(f"[ERROR] Fallo procesando observación: {e}")
            continue

        # 5. Cualquier otra línea de ese tick confirma que no hubo timeout inmediato tras la observación
        if pending_observation is not None:
            flush_pending_observation()

        if "End_simulation" in line:
            break

    flush_pending_observation()


if __name__ == '__main__':
    main()