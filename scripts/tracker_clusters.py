#!/usr/bin/env python3
"""
tracker_trayectoria_clusters.py

- controllers -> J_exp1 y W_exp1 

- Lee la salida estándar de ARGoS buscando patrones de trayectoria.
- Agrupa los patrones en clústeres según la Calidad de Imitación (Qi) definida por Erbas et al.
- Visualiza las trayectorias en tiempo real mediante una ventana de Tkinter con scroll dinámico.
- Ventana secundaria de texto para resumen estadístico de clústeres.
- Optimizado para alto rendimiento (headless) usando actualización por lotes.

"""

import sys
import re
import math
import threading
import queue
import random
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# # ---------- CONFIGURACIÓN GENERAL ----------

# # Modo de ejecución de las interfaces gráficas
# LIVE_MODE = False        # <-- True: Actualiza gráficas en vivo. False: Oculta la ventana y muestra todo al final.
# LIVE_MODE_TEXT = True    # <-- True: Actualiza ventana de texto en vivo. False: Oculta y muestra al final.

# # Límite estricto para considerar que una agrupación es oficialmente un "Clúster"
# MIN_CLUSTER_SIZE = 5

# # Límite visual: Muestra solo el "Top N" de clústeres más grandes para evitar que Matplotlib colapse
# MAX_CLUSTERS_TO_SHOW = 30

# # Frecuencia de actualización (aplica a las ventanas que estén en LIVE_MODE = True)
# UPDATE_FREQUENCY = 1

# # Umbral de Calidad de Imitación para considerar que un patrón pertenece a un clúster
# QI_THRESHOLD = 0.85

# # Modo de comparación para añadir a un clúster y visualización:
# # 1 = "Centroide" (Promedio del Qi y visualiza trayectoria promedio)
# # 2 = "Padre" (Compara con el primer miembro y visualiza el padre)
# # 3 = "Aleatorio" (Compara al azar, pero visualiza el padre por estabilidad)
# CLUSTER_COMPARE_MODE = 1

# # Control de Visualización:
# # True  = Grafica inmediatamente cualquier patrón nuevo (incluso si el clúster tiene tamaño 1).
# # False = Solo muestra clústeres que cumplan con MIN_CLUSTER_SIZE.
# GRAPH_IMMEDIATE = False

# # Control de Cierre de Ventana:
# AUTO_CLOSE_WINDOWS = False     
# END_PATTERN = "End_simulation" 

# # Pesos de la ecuación de Calidad de Imitación (Qi)
# WEIGHT_L = 1.0  
# WEIGHT_A = 1.0  
# WEIGHT_S = 1.0  

# # -------------------------------------------

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
# --- Interfaces y Modos en Vivo ---
LIVE_MODE = False               # True: Actualiza gráficas en vivo. False: Oculta ventana y muestra al final.
LIVE_MODE_TEXT = True           # True: Actualiza ventana de texto en vivo. False: Oculta y muestra al final.
GRAPH_IMMEDIATE = False         # True: Grafica todo patrón nuevo. False: Solo muestra clústeres maduros.

# --- Límites Visuales y Refresco ---
MAX_CLUSTERS_TO_SHOW = 30       # "Top N" de clústeres más grandes a mostrar para no colapsar Matplotlib
UPDATE_FREQUENCY = 1            # Frecuencia de actualización en recepción de paquetes (si LIVE_MODE=True)

# =============================================================================
# ---------- CONFIGURACIÓN DEL TRACKER Y SISTEMA ----------
# =============================================================================
END_PATTERN = "End_simulation"  # Frase en consola que finaliza la ejecución
AUTO_CLOSE_WINDOWS = False      # True: Cierra las interfaces de Tkinter automáticamente al terminar

# =============================================================================
# ---------- CONFIGURACIÓN DE PROCESAMIENTO Y LÓGICA ----------
# =============================================================================
# --- Algoritmos de Agrupación (Clustering) ---
MIN_CLUSTER_SIZE = 5            # Tamaño mínimo para que un grupo se considere "Clúster" oficial
QI_THRESHOLD = 0.85             # Umbral de Calidad (Qi) para pertenecer a un clúster
CLUSTER_COMPARE_MODE = 1        # 1 = Centroide (Promedio), 2 = Padre (Original), 3 = Aleatorio

# --- Pesos de Ecuación de Calidad de Imitación (Qi) ---
WEIGHT_L = 1.0                  # Peso para la métrica de Longitud (Ql)
WEIGHT_A = 1.0                  # Peso para la métrica de Ángulos (Qa)
WEIGHT_S = 1.0                  # Peso para la métrica de Segmentos (Qs)

RE_PATTERN_START = re.compile(r'pattern\s*&\s*(\d+)\s*&\s*(\d+)')
RE_VAL_1 = re.compile(r'1\s*=\s*([-]?\d+[.,]?\d*)')
RE_VAL_2 = re.compile(r'2\s*=\s*([-]?\d+[.,]?\d*)')

def parse_float(val_str):
    return float(val_str.replace(',', '.'))

class ClusterData:
    _global_id = 1 
    
    def __init__(self, first_trajectory):
        self.id = ClusterData._global_id
        ClusterData._global_id += 1
        self.members = [first_trajectory]
        self.father = first_trajectory
        
    def add_member(self, trajectory):
        self.members.append(trajectory)
        
    def size(self):
        return len(self.members)
        
    def get_representative_trajectory(self):
        if CLUSTER_COMPARE_MODE == 1:
            avg_traj = []
            max_segments = max(len(m) for m in self.members)
            for i in range(max_segments):
                sum_l, sum_a, count = 0.0, 0.0, 0
                for m in self.members:
                    if i < len(m):
                        sum_l += m[i][0]
                        sum_a += m[i][1]
                        count += 1
                avg_traj.append((sum_l / count, sum_a / count))
            return avg_traj
        else:
            return self.father

def calculate_qi(orig, copy):
    N_o = len(orig)
    N_c = len(copy)
    
    if N_o == 0:
        return 0.0 
    Q_s = 1.0 - (abs(N_c - N_o) / N_o)
    
    min_N = min(N_o, N_c)
    if min_N == 0:
        Q_l, Q_a = 0.0, 0.0
    else:
        sum_l_o = sum(orig[i][0] for i in range(min_N))
        diff_l = sum(abs(orig[i][0] - copy[i][0]) for i in range(min_N))
        if sum_l_o == 0:
            Q_l = 1.0 if diff_l == 0 else 0.0
        else:
            Q_l = 1.0 - (diff_l / sum_l_o)
            
        sum_a_o = sum(abs(orig[i][1]) for i in range(min_N))
        diff_a = sum(abs(orig[i][1] - copy[i][1]) for i in range(min_N))
        if sum_a_o == 0:
            Q_a = 1.0 if diff_a == 0 else 0.0
        else:
            Q_a = 1.0 - (diff_a / sum_a_o)
            
    Qi = (WEIGHT_L * Q_l + WEIGHT_A * Q_a + WEIGHT_S * Q_s) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)
    return Qi

def check_similarity(trajectory, cluster):
    if CLUSTER_COMPARE_MODE == 1:
        total_qi = sum(calculate_qi(member, trajectory) for member in cluster.members)
        return total_qi / cluster.size()
    elif CLUSTER_COMPARE_MODE == 2:
        return calculate_qi(cluster.father, trajectory)
    elif CLUSTER_COMPARE_MODE == 3:
        rand_member = random.choice(cluster.members)
        return calculate_qi(rand_member, trajectory)
    return 0.0

def stdin_reader(data_queue, stop_event):
    reading_pattern = False
    current_trajectory = []
    current_len = None
    current_ang = None
    
    try:
        while True:
            line = sys.stdin.readline()
            if not line:
                stop_event.set()
                break
            
            clean_line = re.sub(r'^\[.*?\]\s*', '', line).strip()
            
            if END_PATTERN and END_PATTERN in clean_line:
                stop_event.set()
                break
                
            if "end_data_pattern" in clean_line:
                if current_len is not None and current_ang is not None:
                    current_trajectory.append((current_len, current_ang))
                if current_trajectory:
                    data_queue.put(current_trajectory)
                reading_pattern = False
                current_trajectory = []
                current_len = current_ang = None
                continue
                
            match_start = RE_PATTERN_START.search(clean_line)
            if match_start:
                reading_pattern = True
                current_trajectory = []
                current_len = current_ang = None
                continue
                
            if reading_pattern:
                if re.match(r'^\d+:', clean_line):
                    if current_len is not None and current_ang is not None:
                        current_trajectory.append((current_len, current_ang))
                        current_len = current_ang = None
                else:
                    m1 = RE_VAL_1.search(clean_line)
                    if m1:
                        current_len = parse_float(m1.group(1))
                    m2 = RE_VAL_2.search(clean_line)
                    if m2:
                        current_ang = parse_float(m2.group(1))
                        
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()

class TrackerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Visualizador de Gráficas de Trayectorias")
        self.root.geometry("800x800")
        
        # --- Manejo de visibilidad Ventana Principal (Gráficas) ---
        if not LIVE_MODE:
            self.root.withdraw()
            
        # --- Creación de Ventana Secundaria (Texto) ---
        self.text_window = tk.Toplevel(self.root)
        self.text_window.title("Resumen de Clústeres")
        self.text_window.geometry("500x600")
        
        if not LIVE_MODE_TEXT:
            self.text_window.withdraw()
            
        # Frame y Scrollbar para la ventana de texto
        text_frame = ttk.Frame(self.text_window)
        text_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.text_scroll = ttk.Scrollbar(text_frame)
        self.text_scroll.pack(side="right", fill="y")
        
        self.text_area = tk.Text(text_frame, yscrollcommand=self.text_scroll.set, wrap="word", font=("Consolas", 10))
        self.text_area.pack(side="left", fill="both", expand=True)
        self.text_scroll.config(command=self.text_area.yview)
        # ----------------------------------------------
        
        self.data_queue = queue.Queue()
        self.clusters = []
        self.stop_event = threading.Event()
        self.finished_handled = False
        self.packets_received = 0
        
        # --- Configuración del Scroll Ventana Principal ---
        self.canvas_tk = tk.Canvas(root, bg="#f0f0f0")
        self.scrollbar = ttk.Scrollbar(root, orient="vertical", command=self.canvas_tk.yview)
        self.scrollable_frame = ttk.Frame(self.canvas_tk)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas_tk.configure(scrollregion=self.canvas_tk.bbox("all"))
        )

        self.canvas_window = self.canvas_tk.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas_tk.configure(yscrollcommand=self.scrollbar.set)

        self.canvas_tk.bind(
            "<Configure>",
            lambda e: self.canvas_tk.itemconfig(self.canvas_window, width=e.width)
        )

        self.canvas_tk.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        
        # --- Configuración de Matplotlib ---
        self.fig = Figure(figsize=(7, 4), dpi=100) 
        self.canvas_mpl = FigureCanvasTkAgg(self.fig, master=self.scrollable_frame)
        self.canvas_mpl.get_tk_widget().pack(fill="x", expand=False)
        
        # Iniciar lectura de datos
        self.reader_thread = threading.Thread(target=stdin_reader, args=(self.data_queue, self.stop_event), daemon=True)
        self.reader_thread.start()
        
        self.check_queue()

    def check_queue(self):
        updated_plot = False
        updated_text = False
        
        while not self.data_queue.empty():
            try:
                new_traj = self.data_queue.get_nowait()
                self.process_trajectory(new_traj)
                self.packets_received += 1
                
                # Batching independiente
                if LIVE_MODE:
                    if self.packets_received == 1 or self.packets_received % UPDATE_FREQUENCY == 0:
                        updated_plot = True
                
                if LIVE_MODE_TEXT:
                    if self.packets_received == 1 or self.packets_received % UPDATE_FREQUENCY == 0:
                        updated_text = True
                    
            except queue.Empty:
                break
                
        # Simulación finalizada
        if self.stop_event.is_set() and self.data_queue.empty() and not self.finished_handled:
            self.finished_handled = True
            updated_plot = True 
            updated_text = True
            
            # Mostrar ventanas si estaban ocultas
            if not LIVE_MODE:
                self.root.deiconify()
            if not LIVE_MODE_TEXT:
                self.text_window.deiconify()
            
            if AUTO_CLOSE_WINDOWS:
                print("\n[Clústeres] Simulación terminada. Cerrando ventanas automáticamente...")
                self.root.destroy()
                return 
            else:
                print("\n[Clústeres] Simulación terminada. Cierra las ventanas manualmente para continuar.")
                
        if updated_plot:
            self.update_plot()
        if updated_text:
            self.update_text_summary()
            
        self.root.after(200, self.check_queue) 

    def process_trajectory(self, traj):
        best_cluster = None
        best_qi = -float('inf')
        
        for cluster in self.clusters:
            qi = check_similarity(traj, cluster)
            if qi > best_qi:
                best_qi = qi
                best_cluster = cluster
                
        if best_cluster is not None and best_qi >= QI_THRESHOLD:
            best_cluster.add_member(traj)
        else:
            self.clusters.append(ClusterData(traj))

    def get_valid_clusters(self):
        """Devuelve los clústeres filtrados y ordenados por tamaño."""
        if GRAPH_IMMEDIATE:
            valid_clusters = self.clusters
        else:
            valid_clusters = [c for c in self.clusters if c.size() >= MIN_CLUSTER_SIZE]
            
        valid_clusters.sort(key=lambda c: c.size(), reverse=True)
        return valid_clusters[:MAX_CLUSTERS_TO_SHOW]

    def update_text_summary(self):
        """Actualiza la ventana secundaria de solo texto."""
        # Verificar que la ventana no haya sido cerrada manualmente por el usuario
        if not self.text_window.winfo_exists():
            return
            
        valid_clusters = self.get_valid_clusters()
        
        lineas = []
        lineas.append(f"Total de paquetes: {self.packets_received}")
        lineas.append(f"Clústeres creados: {len(self.clusters)}")
        lineas.append("=========================================\n")
        
        mode_str = "Centroide (Promedio)" if CLUSTER_COMPARE_MODE == 1 else "Padre (Original)"
        
        for c in valid_clusters:
            rep_traj = c.get_representative_trajectory()
            params_str = " | ".join([f"({l:.1f}cm, {a:.1f}°)" for l, a in rep_traj])
            
            lineas.append(f"Clúster {c.id} -> {mode_str}")
            lineas.append(f"Miembros: {c.size()}")
            lineas.append(f"Parámetros: {params_str}\n")
            lineas.append("-" * 40 + "\n")
            
        texto_completo = "\n".join(lineas)
        
        self.text_area.config(state="normal") # Habilitar escritura
        self.text_area.delete(1.0, tk.END)    # Limpiar todo
        self.text_area.insert(tk.END, texto_completo) # Insertar nuevo texto
        self.text_area.config(state="disabled") # Bloquear escritura del usuario

    def update_plot(self):
        """Actualiza la ventana principal de gráficas con Matplotlib."""
        valid_clusters = self.get_valid_clusters()
        num_plots = len(valid_clusters)
        
        self.fig.clf()
        
        if num_plots == 0:
            self.canvas_mpl.draw()
            return
            
        new_height_inches = max(4.5, num_plots * 4.5)
        self.fig.set_figheight(new_height_inches)
        
        new_height_pixels = int(new_height_inches * self.fig.dpi)
        self.canvas_mpl.get_tk_widget().configure(height=new_height_pixels)
        
        mode_str = "Centroide (Promedio)" if CLUSTER_COMPARE_MODE == 1 else "Padre (Original)"
        
        for i, cluster in enumerate(valid_clusters):
            ax = self.fig.add_subplot(num_plots, 1, i+1)
            
            rep_traj = cluster.get_representative_trajectory()
            self.draw_kinematics(ax, rep_traj)
            
            params_lines = [f"{idx+1}: {l:.1f}cm, {a:.1f}°" for idx, (l, a) in enumerate(rep_traj)]
            chunks = [ " | ".join(params_lines[j:j+3]) for j in range(0, len(params_lines), 3) ]
            params_str = "\n".join(chunks)
            
            title = f"Clúster #{cluster.id} | Miembros: {cluster.size()} | Modalidad: {mode_str}\nParametros: {params_str}"
            ax.set_title(title, fontsize=9, loc='center')
            ax.grid(True)
            ax.axis('equal') 
            
        self.fig.tight_layout()
        self.canvas_mpl.draw()

    def draw_kinematics(self, ax, trajectory):
        x, y = 0.0, 0.0
        theta = 0.0 
        
        xs, ys = [x], [y]
        
        for length, angle_deg in trajectory:
            theta += math.radians(angle_deg)
            x += length * math.cos(theta)
            y += length * math.sin(theta)
            xs.append(x)
            ys.append(y)
            
        ax.plot(xs, ys, marker='o', markersize=4, linestyle='-', color='b', linewidth=2)
        ax.scatter([0], [0], color='red', s=50, zorder=5, label='Inicio')
        ax.legend(loc='lower right', fontsize='small')

if __name__ == '__main__':
    root = tk.Tk()
    app = TrackerApp(root)
    root.mainloop()