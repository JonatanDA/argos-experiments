#!/usr/bin/env python3
"""
tracker_imitation_chain.py

- controllers: St10 -> Qupa - Footbot

- Captura pares consecutivos demostrador-imitador (y cuenta los huérfanos).
- Dibuja la cinemática de trayectorias (pop-ups automáticos opcionales).
- Muestra el Árbol de Transmisión dinámico a la izquierda.
- Permite hacer clic en los nodos del árbol para visualizar su historial de aprendizaje.
- Tolerancia a ejecuciones ultrarrápidas (Headless) y cierre controlado.
"""

import sys
import re
import math
import queue
import threading
import os
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Ventanas de Trayectoria (Cinemática) ---
SHOW_TRAJECTORY_WINDOWS = False  # True: Abre ventanas por cada patrón imitado en tiempo real. False: Solo por clic.
PLOT_MODE = 1                   # 1: Gráficos superpuestos, 2: Gráficos separados
MAX_WINDOWS = 5                 # Límite de ventanas de trayectorias abiertas simultáneamente
WINDOW_SCALE = 0.5              # Escala global de las ventanas
BASE_SIZE_MODE_1 = (6, 6)       # (Aplica si PLOT_MODE=1) Dimensiones base
BASE_SIZE_MODE_2 = (12, 5)      # (Aplica si PLOT_MODE=2) Dimensiones base

# --- Ventana del Árbol de Transmisión ---
SHOW_TREE_WINDOW = True         # True: Muestra la ventana interactiva del Árbol de Transmisión

# --- Posicionamiento en Pantalla ---
MOVE_WINDOWS_TO_RIGHT = True    # True: Fuerza la aparición de ventanas en coordenadas específicas
START_X_POS_TRAJ = 1200         # Coordenada X inicial para trayectorias
START_Y_POS = 100               # Coordenada Y inicial
CASCADE_OFFSET = 40             # Desplazamiento en píxeles para ventanas en cascada
START_X_POS_TREE = 100          # Coordenada X inicial para la ventana del árbol

# =============================================================================
# ---------- CONFIGURACIÓN DE GUARDADO DE DATOS ----------
# =============================================================================
IMAGE_DIR = "Resultados/Imitation_chain"  # Directorio de guardado
SAVE_TRAJ_IMAGE = True          # True: Guarda como imagen cada ventana de trayectoria generada
SAVE_TREE_IMAGE = False          # True: Guarda el mapa del árbol (se actualiza automáticamente)

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
END_PATTERN = "End_simulation"  # Frase en consola que finaliza la ejecución
AUTO_CLOSE_WINDOWS      = False # True: Cierra las gráficas automáticamente al terminar. False: Las mantiene abiertas.
AUTO_CLOSE_TIME         = 3.0   # (Aplica si AUTO_CLOSE_WINDOWS=True) Segundos de espera antes de cerrar.


RE_TIMER = re.compile(r'timer:(\d+)')
RE_ROLE = re.compile(r'(original|demostrado|imitado)')
RE_ROBOT = re.compile(r'^(Qupa\d+)')
RE_TUPLES = re.compile(r'\(([^)]+)\)')

def parse_line(line):
    timer_match = RE_TIMER.search(line)
    role_match = RE_ROLE.search(line)
    robot_match = RE_ROBOT.search(line)
    
    if not (timer_match and role_match and robot_match):
        return None
        
    timer = int(timer_match.group(1))
    role = role_match.group(1)
    robot_id = robot_match.group(1)
    
    tuples_str = RE_TUPLES.findall(line)
    vectors = []
    
    for t in tuples_str:
        parts = [p.strip() for p in t.split(',')]
        try:
            if len(parts) == 4:
                dist = float(f"{parts[0]}.{parts[1]}")
                ang = float(f"{parts[2]}.{parts[3]}")
                vectors.append((dist, ang))
            elif len(parts) == 2:
                dist = float(parts[0])
                ang = float(parts[1])
                vectors.append((dist, ang))
        except ValueError:
            pass 
            
    return {'id': robot_id, 'role': role, 'vectors': vectors, 'timer': timer}

def calc_coords(vectors):
    x, y = 0.0, 0.0
    current_angle = 0.0
    xs = [x]
    ys = [y]
    for dist, ang in vectors:
        current_angle += ang  
        rad = math.radians(current_angle)
        x += dist * math.cos(rad)
        y += dist * math.sin(rad)
        xs.append(x)
        ys.append(y)
    return xs, ys

def move_window(fig, x, y):
    backend = plt.get_backend()
    manager = fig.canvas.manager
    try:
        if backend == 'TkAgg':
            manager.window.wm_geometry(f"+{int(x)}+{int(y)}")
        elif backend == 'WXAgg':
            manager.window.SetPosition((int(x), int(y)))
        elif backend in ['Qt5Agg', 'QtAgg', 'Qt4Agg']:
            manager.window.move(int(x), int(y))
    except Exception:
        pass

def format_vectors_text(title, vectors):
    lines = [title]
    if not vectors:
        lines.append("Sin parámetros extraíbles.")
    for i, (d, a) in enumerate(vectors, 1):
        lines.append(f"{i}. D: {d:.2f} | A: {a:.2f}°")
    return "\n".join(lines)

def get_tree_layout(G):
    if len(G) == 0: 
        return {}
        
    in_degrees = dict(G.in_degree())
    roots = [n for n, d in in_degrees.items() if d == 0]
    
    levels = {}
    for n in G.nodes():
        max_d = 0
        for r in roots:
            if nx.has_path(G, r, n):
                d = nx.shortest_path_length(G, r, n)
                if d > max_d: max_d = d
        levels[n] = max_d
        
    level_nodes = defaultdict(list)
    for n, lvl in levels.items():
        level_nodes[lvl].append(n)
        
    pos = {}
    if not level_nodes: 
        return pos

    roots_list = level_nodes[0]
    for i, n in enumerate(roots_list):
        pos[n] = (i * 2.0, 0)

    max_lvl = max(level_nodes.keys())
    for lvl in range(1, max_lvl + 1):
        nodes = level_nodes[lvl]
        
        ideal_x = {}
        for n in nodes:
            parents = list(G.predecessors(n))
            placed_parents = [p for p in parents if p in pos]
            if placed_parents:
                ideal_x[n] = sum(pos[p][0] for p in placed_parents) / len(placed_parents)
            else:
                ideal_x[n] = 0

        sorted_nodes = sorted(nodes, key=lambda n: ideal_x[n])

        min_gap = 1.5 
        current_x = ideal_x[sorted_nodes[0]]
        
        for i, n in enumerate(sorted_nodes):
            if i > 0:
                current_x = max(ideal_x[n], current_x + min_gap)
            else:
                current_x = ideal_x[n]
            pos[n] = (current_x, -lvl)

        if sorted_nodes:
            avg_ideal = sum(ideal_x[n] for n in sorted_nodes) / len(sorted_nodes)
            avg_actual = sum(pos[n][0] for n in sorted_nodes) / len(sorted_nodes)
            shift = avg_ideal - avg_actual
            for n in sorted_nodes:
                pos[n] = (pos[n][0] + shift, -lvl)

    return pos

def stdin_reader(q, stop_event):
    try:
        while True:
            raw = sys.stdin.readline()
            if raw == '': break
            line = raw.strip()
            if not line: continue
            if END_PATTERN in line:
                stop_event.set()
                break
            parsed = parse_line(line)
            if parsed:
                q.put(parsed)
    except Exception:
        pass
    finally:
        stop_event.set()

def main():
    q = queue.Queue()
    stop_event = threading.Event()
    click_requests = queue.Queue()
    
    reader_thread = threading.Thread(target=stdin_reader, args=(q, stop_event), daemon=True)
    reader_thread.start()
    
    demo_counts = defaultdict(int)
    pending_demo = None
    
    active_figs = []
    imitation_history = {}      
    unmatched_demos_count = 0   
    
    if SAVE_TRAJ_IMAGE or SAVE_TREE_IMAGE:
        os.makedirs(IMAGE_DIR, exist_ok=True)

    plt.ion() 
    G = nx.DiGraph()
    tree_fig = None
    ax_tree = None
    
    def create_trajectory_figure(demo_data, imit_data, count, auto_position=True):
        if PLOT_MODE == 2:
            current_figsize = (BASE_SIZE_MODE_2[0] * WINDOW_SCALE, BASE_SIZE_MODE_2[1] * WINDOW_SCALE)
        else:
            current_figsize = (BASE_SIZE_MODE_1[0] * WINDOW_SCALE, BASE_SIZE_MODE_1[1] * WINDOW_SCALE)
        
        fig = plt.figure(figsize=current_figsize)
        window_name = f"{demo_data['id']}&{imit_data['id']}"
        title_text = f"T={imit_data['timer']} | Demo: {demo_data['id']} (vez #{count}) vs Imit: {imit_data['id']}"
        fig.canvas.manager.set_window_title(window_name)
        
        if MOVE_WINDOWS_TO_RIGHT and auto_position:
            offset_x = START_X_POS_TRAJ + (len(active_figs) * CASCADE_OFFSET)
            offset_y = START_Y_POS + (len(active_figs) * CASCADE_OFFSET)
            move_window(fig, offset_x, offset_y)
        
        demo_xs, demo_ys = calc_coords(demo_data['vectors'])
        imit_xs, imit_ys = calc_coords(imit_data['vectors'])
        
        text_props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        str_demo = format_vectors_text(f"Parámetros {demo_data['id']}:", demo_data['vectors'])
        str_imit = format_vectors_text(f"Parámetros {imit_data['id']}:", imit_data['vectors'])
        
        if PLOT_MODE == 1:
            ax = fig.add_subplot(111)
            ax.plot(demo_xs, demo_ys, 'b.-', label=f"Demo ({demo_data['id']})")
            ax.plot(imit_xs, imit_ys, 'r.--', label=f"Imit ({imit_data['id']})")
            ax.set_title(title_text)
            ax.legend(loc='lower right')
            ax.grid(True)
            ax.axis('equal')
            ax.text(0.02, 0.98, str_demo, transform=ax.transAxes, va='top', bbox=text_props, fontsize=8)
            ax.text(0.02, 0.70, str_imit, transform=ax.transAxes, va='top', bbox=dict(boxstyle='round', facecolor='lightpink', alpha=0.8), fontsize=8)
        else:
            ax1 = fig.add_subplot(121)
            ax1.plot(demo_xs, demo_ys, 'b.-', label=f"Demo ({demo_data['id']})")
            ax1.set_title("Original / Demostrado")
            ax1.legend(loc='lower right')
            ax1.grid(True)
            ax1.axis('equal')
            ax1.text(0.02, 0.98, str_demo, transform=ax1.transAxes, va='top', bbox=text_props, fontsize=8)
            
            ax2 = fig.add_subplot(122)
            ax2.plot(imit_xs, imit_ys, 'r.-', label=f"Imitado ({imit_data['id']})")
            ax2.set_title("Imitado")
            ax2.legend(loc='lower right')
            ax2.grid(True)
            ax2.axis('equal')
            ax2.text(0.02, 0.98, str_imit, transform=ax2.transAxes, va='top', bbox=dict(boxstyle='round', facecolor='lightpink', alpha=0.8), fontsize=8)
            
            fig.suptitle(title_text)
        
        plt.tight_layout()
        
        if SAVE_TRAJ_IMAGE:
            filename = os.path.join(IMAGE_DIR, f"{window_name}_T{imit_data['timer']}.png")
            plt.savefig(filename)
        
        active_figs.append(fig)
        fig.canvas.draw_idle()

    def on_tree_click(event):
        if event.inaxes is None or not hasattr(event.inaxes, 'node_pos'): 
            return
        
        click_x, click_y = event.xdata, event.ydata
        closest_node = None
        min_dist = 0.2 
        
        for node, (nx, ny) in event.inaxes.node_pos.items():
            dist = (nx - click_x)**2 + (ny - click_y)**2
            if dist < min_dist:
                min_dist = dist
                closest_node = node
                
        if closest_node:
            click_requests.put(closest_node)

    if SHOW_TREE_WINDOW:
        tree_fig, ax_tree = plt.subplots(figsize=(6, 8))
        tree_fig.canvas.manager.set_window_title("Árbol de Transmisión")
        move_window(tree_fig, START_X_POS_TREE, START_Y_POS)
        tree_fig.canvas.mpl_connect('button_press_event', on_tree_click)
        
    print("[Tracker Imitación] Escuchando dinámica social de enjambre...")
    
    try:
        while True:
            updated_tree = False
            
            # --- NUEVO: RECOLECTOR DE BASURA DE VENTANAS CERRADAS ---
            # Elimina de la lista interna las ventanas que el usuario cerró con la 'X'
            active_figs = [f for f in active_figs if plt.fignum_exists(f.number)]
            
            # Aplica el límite MAX_WINDOWS solo a las ventanas que siguen vivas
            while len(active_figs) > MAX_WINDOWS:
                old_fig = active_figs.pop(0)
                plt.close(old_fig)
            # ---------------------------------------------------------
            
            while not q.empty():
                try:
                    data = q.get_nowait()
                except queue.Empty:
                    break
                    
                if data['role'] in ['original', 'demostrado']:
                    if pending_demo is not None:
                        unmatched_demos_count += 1
                    pending_demo = data
                    
                elif data['role'] == 'imitado':
                    if pending_demo is not None:
                        if data['timer'] == pending_demo['timer'] + 1:
                            demo_data = pending_demo
                            imit_data = data
                            
                            demo_counts[demo_data['id']] += 1
                            count = demo_counts[demo_data['id']]
                            
                            imitation_history[imit_data['id']] = {
                                'demo': demo_data,
                                'imit': imit_data,
                                'count': count
                            }
                            
                            G.add_edge(demo_data['id'], imit_data['id'])
                            updated_tree = True
                            
                            if SHOW_TRAJECTORY_WINDOWS:
                                create_trajectory_figure(demo_data, imit_data, count, auto_position=True)
                                
                            pending_demo = None
                        else:
                            unmatched_demos_count += 1
                            pending_demo = None
                            
            while not click_requests.empty():
                node_to_plot = click_requests.get()
                if node_to_plot in imitation_history:
                    hist = imitation_history[node_to_plot]
                    print(f"\n[Interacción UI] Visualizando aprendizaje de {node_to_plot}...")
                    create_trajectory_figure(hist['demo'], hist['imit'], hist['count'], auto_position=False)
                else:
                    print(f"\n[Interacción UI] {node_to_plot} es raíz pura. Nadie le demostró un patrón previo.")

            if updated_tree and G.number_of_nodes() > 0:
                total_imitations = G.number_of_edges()
                try:
                    longest_path = nx.dag_longest_path(G)
                    chain_str = " -> ".join(longest_path)
                except nx.NetworkXUnfeasible:
                    chain_str = "Ciclo detectado (no evaluable)"
                    
                pos = get_tree_layout(G)
                
                if SHOW_TREE_WINDOW:
                    xs = [coord[0] for coord in pos.values()]
                    if xs:
                        breadth = max(xs) - min(xs)
                        new_width = max(6.0, min(18.0, 3.0 + breadth * 0.8))
                        tree_fig.set_size_inches(new_width, 8)

                colors = []
                for node in G.nodes():
                    if G.in_degree(node) == 0: colors.append('lightgreen')
                    elif G.out_degree(node) == 0: colors.append('lightyellow')
                    else: colors.append('lightblue')
                    
                title_str = f"Árbol de Transmisión\nTotal imitaciones: {total_imitations} | Huérfanos: {unmatched_demos_count}\nCadena Max: {chain_str}"
                
                if SHOW_TREE_WINDOW:
                    ax_tree.clear()
                    nx.draw(G, pos, ax=ax_tree, with_labels=True, node_size=1500, node_color=colors, font_size=10, font_weight='bold', arrows=True, arrowstyle='-|>', arrowsize=15)
                    ax_tree.set_title(title_str, fontsize=11, pad=15)
                    ax_tree.node_pos = pos 
                    tree_fig.canvas.draw_idle()
                    
                    if SAVE_TREE_IMAGE:
                        tree_filename = os.path.join(IMAGE_DIR, "Arbol_Transmision_Actual.png")
                        tree_fig.savefig(tree_filename)
                        
                elif SAVE_TREE_IMAGE:
                    temp_fig, temp_ax = plt.subplots(figsize=(6, 8))
                    nx.draw(G, pos, ax=temp_ax, with_labels=True, node_size=1500, node_color=colors, font_size=10, font_weight='bold', arrows=True, arrowstyle='-|>', arrowsize=15)
                    temp_ax.set_title(title_str, fontsize=11, pad=15)
                    tree_filename = os.path.join(IMAGE_DIR, "Arbol_Transmision_Actual.png")
                    temp_fig.savefig(tree_filename)
                    plt.close(temp_fig)
                    
            if stop_event.is_set() and q.empty():
                break
                
            # --- NUEVO: EL ANCLA DEL EVENT LOOP ---
            # Matplotlib enloquece si se hace plt.pause() sobre una figura que ha sido destruida.
            # Por seguridad, forzamos que el foco regrese al árbol de transmisión.
            if SHOW_TREE_WINDOW and tree_fig and plt.fignum_exists(tree_fig.number):
                plt.figure(tree_fig.number)
            
            plt.pause(0.1) 
            
    except KeyboardInterrupt:
        print("\n[Tracker Imitación] Interrupción forzada detectada (Ctrl+C). Asegurando el guardado...")
        
    finally:
        plt.ioff()
        
        if G.number_of_nodes() > 0 and SAVE_TREE_IMAGE and tree_fig:
            final_tree_path = os.path.join(IMAGE_DIR, "Arbol_Transmision_Final.png")
            tree_fig.savefig(final_tree_path)
            print(f"[Tracker Imitación] Mapa final guardado en {final_tree_path}")
            
        if not AUTO_CLOSE_WINDOWS:
            print("\n=======================================================")
            print("[Tracker Imitación] Simulación Finalizada.")
            print("Las ventanas se MANTENDRÁN ABIERTAS (AUTO_CLOSE_WINDOWS=False).")
            print("Por favor, cierra las gráficas manualmente en la 'X' para terminar.")
            print("=======================================================\n")
            try:
                plt.show() 
            except KeyboardInterrupt:
                pass
        else:
            print(f"\n[Tracker Imitación] Simulación Finalizada. Cerrando automáticamente en {AUTO_CLOSE_TIME} segundos...")
            try:
                plt.pause(AUTO_CLOSE_TIME)
            except KeyboardInterrupt:
                pass
            
            for fig in active_figs:
                if plt.fignum_exists(fig.number):
                    plt.close(fig)
            if SHOW_TREE_WINDOW and tree_fig and plt.fignum_exists(tree_fig.number):
                plt.close(tree_fig)

if __name__ == '__main__':
    main()