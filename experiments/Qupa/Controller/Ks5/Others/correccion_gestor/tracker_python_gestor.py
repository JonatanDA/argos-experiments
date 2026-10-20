import sys
from procesar_datos import procesar_datos_logicos_y_reconstruccion, check_patience_and_reconstruct

def parse_blob_str(blob_str):
    blobs = []
    if not blob_str: return blobs
    for p in blob_str.split('|'):
        parts = p.split(',')
        b = {'angle': float(parts[0]), 'distance': float(parts[1])}
        if len(parts) > 2: b['type'] = int(parts[2])
        blobs.append(b)
    return blobs

def main():
    print("[PYTHON GESTOR] Iniciado. Escuchando datos de ARGoS en tiempo real...", flush=True)
    
    # sys.stdin can be buffered. Using readline ensures real-time processing
    while True:
        line = sys.stdin.readline()
        if not line:
            break
            
        line = line.strip()
        if not line: continue
        
        if line.startswith("[python];observation;"):
            parts = line.split(';')
            if len(parts) >= 6:
                obs_ang = float(parts[2])
                x_mark = float(parts[3])
                y_mark = float(parts[4])
                yellows = parse_blob_str(parts[5])
                greens = parse_blob_str(parts[6]) if len(parts) > 6 else []
                
                datos = {
                    'obs_ang_offset_global': obs_ang,
                    'x_mark': x_mark,
                    'y_mark': y_mark,
                    'yellow_blobs': yellows,
                    'green_blobs': greens
                }
                procesar_datos_logicos_y_reconstruccion(datos)
                
        elif line.startswith("[python];patience_timeout"):
            check_patience_and_reconstruct()

if __name__ == "__main__":
    main()
