#!/usr/bin/env python3
"""
tracker_paciencia.py

- Lee la salida estándar de ARGoS canalizada por run_sim.py.
- Filtra eventos de "[PACIENCIA]XX.X&TPS" y "[PACIENCIA](XX,X) segundos".
- Extrae los Ticks Por Segundo (TPS) del primer log para mostrar el tiempo real.
- Intercambia Ancho/Alto automáticamente si es vertical.
- Ajusta el tamaño de la fuente dinámicamente según la ventana.
"""

import sys
import re
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =============================================================================
# ---------- CONFIGURACIÓN DE VARIABLES ----------
# =============================================================================
ORIENTACION             = "horizontal" # Opciones: "horizontal" o "vertical"
SIEMPRE_AL_FRENTE       = True         # True = La ventana se mantendrá por encima de las demás

# Tamaño base de la ventana en pulgadas (Si es vertical, se invierten solos)
VENTANA_ANCHO           = 7
VENTANA_ALTO            = 1

TITULO_GRAFICA          = "Paciencia"
END_SIMULATION_PATTERN  = "End_simulation"
AUTO_CLOSE_WINDOWS      = True        # Cerrar al terminar (False = Se queda abierta hasta que cierres)

# =============================================================================
# ---------- ESTADO COMPARTIDO ----------
# =============================================================================
estado = {
    'paciencia': 0.0,
    'max_paciencia': None,  # Se auto-calibrará con el primer valor
    'timer_actual': 0,      # En ticks
    'ticks_per_sec': 10.0,  # Valor por defecto (se actualizará si viene el &x)
    'terminado': False
}
lock = threading.Lock()

# =============================================================================
# ---------- LECTURA DE STDIN (HILO EN SEGUNDO PLANO) ----------
# =============================================================================
def stdin_reader(stop_event):
    """
    Lee la entrada estándar línea por línea.
    """
    global estado
    
    # Expresión regular robusta:
    # 1. Busca "[PACIENCIA]"
    # 2. Permite paréntesis opcionales "\(? ... \)?"
    # 3. Captura el valor principal: "(\d+(?:[.,]\d+)?)" -> Ej: 80.0 o 79,9
    # 4. Opcionalmente busca un "&" seguido de un número "(\d+(?:[.,]\d+)?)" -> Ej: &10
    regex_paciencia = re.compile(r'\[PACIENCIA\]\s*\(?\s*(\d+(?:[.,]\d+)?)\s*\)?(?:[^&]*&(\d+(?:[.,]\d+)?))?', re.IGNORECASE)
    regex_timer = re.compile(r'timer\s*=\s*(\d+)', re.IGNORECASE)

    try:
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                break
            
            # 1. Condición de Finalización
            if END_SIMULATION_PATTERN in line:
                with lock:
                    estado['terminado'] = True
                stop_event.set()
                break
            
            if estado['terminado']:
                continue
            
            # 2. Actualizar Timer (Ticks)
            match_timer = regex_timer.search(line)
            if match_timer:
                with lock:
                    estado['timer_actual'] = int(match_timer.group(1))
                continue

            # 3. Extraer Paciencia y TPS (si existe)
            match_paciencia = regex_paciencia.search(line)
            if match_paciencia:
                # Reemplazamos coma por punto para Python
                valor_str = match_paciencia.group(1).replace(',', '.')
                valor_flotante = float(valor_str)
                
                with lock:
                    # Guardar el primer valor como máximo
                    if estado['max_paciencia'] is None:
                        estado['max_paciencia'] = valor_flotante
                        
                    estado['paciencia'] = valor_flotante
                    
                    # Si viene acompañado del "&x" (Ticks per second)
                    if match_paciencia.group(2):
                        tps_str = match_paciencia.group(2).replace(',', '.')
                        estado['ticks_per_sec'] = float(tps_str)

    except Exception as e:
        print(f"[Tracker Paciencia] Error leyendo stdin: {e}")
    finally:
        stop_event.set()

# =============================================================================
# ---------- UTILIDAD PARA MANTENER LA VENTANA AL FRENTE ----------
# =============================================================================
def aplicar_siempre_al_frente(fig):
    if not SIEMPRE_AL_FRENTE:
        return
    try:
        fig.canvas.manager.window.attributes('-topmost', True)
    except AttributeError:
        pass
    try:
        fig.canvas.manager.window.setWindowFlags(
            fig.canvas.manager.window.windowFlags() | 0x00040000 
        )
        fig.canvas.manager.window.show()
    except AttributeError:
        pass

# =============================================================================
# ---------- LÓGICA DE INTERFAZ GRÁFICA (MATPLOTLIB) ----------
# =============================================================================
def main():
    stop_event = threading.Event()
    reader_thread = threading.Thread(target=stdin_reader, args=(stop_event,), daemon=True)
    reader_thread.start()

    # Invertir las dimensiones automáticamente si es vertical
    ancho_efectivo = VENTANA_ANCHO
    alto_efectivo = VENTANA_ALTO
    if ORIENTACION.lower() == "vertical":
        ancho_efectivo, alto_efectivo = VENTANA_ALTO, VENTANA_ANCHO

    # Configuración de la Ventana
    fig, ax = plt.subplots(figsize=(ancho_efectivo, alto_efectivo))
    fig.canvas.manager.set_window_title("Tracker - Paciencia")
    aplicar_siempre_al_frente(fig)

    def update(frame):
        if not plt.fignum_exists(fig.number):
            return

        with lock:
            pac = estado['paciencia']
            max_pac = estado['max_paciencia']
            timer_val = estado['timer_actual']
            tps = estado['ticks_per_sec']
            terminado = estado['terminado']

        if max_pac is None:
            max_pac = 100.0

        # Calcular tiempo en segundos reales
        tiempo_segundos = timer_val / tps if tps > 0 else 0

        # Calcular el tamaño de fuente dinámicamente según la ventana
        fig_w, fig_h = fig.get_size_inches()
        menor_dimension = min(fig_w, fig_h)
        # Factor mágico: si la dimensión menor es 2, el tamaño será 12.
        tamano_fuente = max(8, int(menor_dimension * 6))

        ax.clear()
        
        # Color dinámico
        color_barra = 'green'
        if pac <= (max_pac * 0.25):
            color_barra = 'red'
        elif pac <= (max_pac * 0.5):
            color_barra = 'orange'

        texto_estado = f"{pac:.1f} / {max_pac:.1f} s"
        if pac <= 0:
            texto_estado = "AGOTADA"

        if ORIENTACION.lower() == "horizontal":
            ax.set_xlim(0, max_pac)
            ax.set_ylim(-0.5, 0.5)
            ax.set_yticks([])
            ax.set_xlabel('Paciencia (Segundos)', fontsize=tamano_fuente*0.8)
            
            ax.barh(0, pac, height=0.6, color=color_barra, edgecolor='black', alpha=0.8)
            
            ax.text(max_pac / 2, 0, texto_estado, 
                    ha='center', va='center', color='black', 
                    fontweight='bold', fontsize=tamano_fuente,
                    bbox=dict(facecolor='white', alpha=0.5, edgecolor='none', pad=2))
        else:
            ax.set_ylim(0, max_pac)
            ax.set_xlim(-0.5, 0.5)
            ax.set_xticks([])
            ax.set_ylabel('Paciencia (Segundos)', fontsize=tamano_fuente*0.8)
            
            ax.bar(0, pac, width=0.6, color=color_barra, edgecolor='black', alpha=0.8)
            
            ax.text(0, max_pac / 2, texto_estado, 
                    ha='center', va='center', color='black', 
                    fontweight='bold', fontsize=tamano_fuente, rotation=90,
                    bbox=dict(facecolor='white', alpha=0.5, edgecolor='none', pad=2))

        # Actualizar título (mostrando tiempo en segundos en lugar de Ticks)
        estado_txt = "(FINALIZADO)" if terminado else "En curso..."
        ax.set_title(f"{TITULO_GRAFICA} | Tiempo: {tiempo_segundos:.1f}s | {estado_txt}", 
                     fontsize=tamano_fuente*0.9)
        
        return []

    # Refresco en 50ms para fluidez
    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show(block=False) 
        
        while not stop_event.is_set() and plt.fignum_exists(fig.number):
            plt.pause(0.1) 
            
    except KeyboardInterrupt:
        stop_event.set()

    if plt.fignum_exists(fig.number):
        if AUTO_CLOSE_WINDOWS:
            plt.close(fig)
        else:
            print("[Tracker Paciencia] Simulación terminada. Cierra la ventana para salir.")
            ani.event_source.stop()
            plt.show(block=True)

if __name__ == '__main__':
    main()