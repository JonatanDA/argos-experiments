import sys
from procesar_datos import procesar_datos_logicos_y_reconstruccion, check_patience_and_reconstruct

def parse_blob_str(blob_str):
    blobs = []
    if not blob_str: return blobs
    for p in blob_str.split('|'):
        # Usamos ':' como separador de campos para no chocar con las comas decimales
        if ':' in p:
            parts = p.split(':')
        else:
            # Fallback heurístico si el usuario olvidó actualizar el código Lua
            parts = p.split(',')
            if len(parts) >= 5:
                parts = [parts[0]+'.'+parts[1], parts[2]+'.'+parts[3], parts[4]]
        
        if len(parts) >= 2:
            # Reemplazar la coma decimal por punto antes de convertir a float
            ang = float(parts[0].replace(',', '.'))
            dist = float(parts[1].replace(',', '.'))
            b = {'angle': ang, 'distance': dist}
            if len(parts) > 2: b['type'] = int(parts[2].replace(',', '.'))
            blobs.append(b)
    return blobs

def main():
    print("[PYTHON GESTOR] Iniciado. Escuchando datos de ARGoS en tiempo real...", flush=True)
    
    while True:
        line = sys.stdin.readline()
        if not line:
            break
            
        line = line.strip()
        if not line: continue
        
        if line.startswith("[python];observation;"):
            parts = line.split(';')
            if len(parts) >= 6:
                # Aquí reemplazamos la coma por el punto para los campos globales
                obs_ang = float(parts[2].replace(',', '.'))
                x_mark = float(parts[3].replace(',', '.'))
                y_mark = float(parts[4].replace(',', '.'))
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
