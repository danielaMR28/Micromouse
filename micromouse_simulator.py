import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import serial
import serial.tools.list_ports
import threading
import queue
import time
from collections import deque
import random

class MicromouseSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Micromouse Simulator - Flood Fill Algorithm")
        self.root.geometry("1200x800")
        
        # Configuración del laberinto
        self.maze_size = 16  # Tamaño estándar del laberinto de micromouse
        self.cell_size = 30
        self.wall_width = 2
        
        # Estado del robot
        self.robot_pos = [0, 0]  # Posición inicial
        self.robot_direction = 0  # 0=Norte, 1=Este, 2=Sur, 3=Oeste
        self.goal_pos = [(7, 7), (7, 8), (8, 7), (8, 8)]  # Centro del laberinto
        
        # Matrices para el algoritmo
        self.walls_h = np.zeros((self.maze_size + 1, self.maze_size), dtype=bool)  # Paredes horizontales
        self.walls_v = np.zeros((self.maze_size, self.maze_size + 1), dtype=bool)  # Paredes verticales
        self.flood_values = np.zeros((self.maze_size, self.maze_size), dtype=int)
        self.visited = np.zeros((self.maze_size, self.maze_size), dtype=bool)
        
        # Comunicación serial
        self.serial_port = None
        self.serial_thread = None
        self.data_queue = queue.Queue()
        self.running = False
        self.simulation_mode = True  # Modo simulación por defecto
        
        # Configurar la interfaz
        self.setup_ui()
        
        # Inicializar el laberinto
        self.init_maze()
        self.init_flood_values()
        
        # Iniciar el loop de actualización
        self.update_loop()
        
    def setup_ui(self):
        """Configurar la interfaz de usuario"""
        # Frame principal
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Frame izquierdo - Canvas del laberinto
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, padx=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.canvas = tk.Canvas(
            left_frame,
            width=self.maze_size * self.cell_size + 20,
            height=self.maze_size * self.cell_size + 20,
            bg='white'
        )
        self.canvas.pack()
        
        # Frame derecho - Controles
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, padx=10, sticky=(tk.N, tk.S))
        
        # Sección de conexión
        connection_frame = ttk.LabelFrame(right_frame, text="Conexión", padding="10")
        connection_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(connection_frame, text="Puerto COM:").grid(row=0, column=0, sticky=tk.W)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)
        
        self.refresh_ports_btn = ttk.Button(
            connection_frame, 
            text="Actualizar", 
            command=self.refresh_ports
        )
        self.refresh_ports_btn.grid(row=0, column=2, padx=5)
        
        self.connect_btn = ttk.Button(
            connection_frame, 
            text="Conectar", 
            command=self.toggle_connection
        )
        self.connect_btn.grid(row=1, column=0, columnspan=3, pady=5)
        
        self.simulation_var = tk.BooleanVar(value=True)
        self.simulation_check = ttk.Checkbutton(
            connection_frame,
            text="Modo Simulación",
            variable=self.simulation_var,
            command=self.toggle_simulation_mode
        )
        self.simulation_check.grid(row=2, column=0, columnspan=3, pady=5)
        
        # Sección de control
        control_frame = ttk.LabelFrame(right_frame, text="Control del Robot", padding="10")
        control_frame.pack(fill=tk.X, pady=5)
        
        self.start_btn = ttk.Button(
            control_frame,
            text="Iniciar Exploración",
            command=self.start_exploration,
            state=tk.NORMAL
        )
        self.start_btn.pack(fill=tk.X, pady=2)
        
        self.stop_btn = ttk.Button(
            control_frame,
            text="Detener",
            command=self.stop_exploration,
            state=tk.DISABLED
        )
        self.stop_btn.pack(fill=tk.X, pady=2)
        
        self.reset_btn = ttk.Button(
            control_frame,
            text="Reiniciar",
            command=self.reset_maze
        )
        self.reset_btn.pack(fill=tk.X, pady=2)
        
        # Sección de información
        info_frame = ttk.LabelFrame(right_frame, text="Información", padding="10")
        info_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Posición del robot
        ttk.Label(info_frame, text="Posición:").grid(row=0, column=0, sticky=tk.W)
        self.pos_label = ttk.Label(info_frame, text="(0, 0)")
        self.pos_label.grid(row=0, column=1, sticky=tk.W, padx=5)
        
        # Dirección
        ttk.Label(info_frame, text="Dirección:").grid(row=1, column=0, sticky=tk.W)
        self.dir_label = ttk.Label(info_frame, text="Norte")
        self.dir_label.grid(row=1, column=1, sticky=tk.W, padx=5)
        
        # Sensores
        ttk.Label(info_frame, text="Sensores:").grid(row=2, column=0, sticky=tk.W, pady=5)
        
        sensor_frame = ttk.Frame(info_frame)
        sensor_frame.grid(row=3, column=0, columnspan=2)
        
        ttk.Label(sensor_frame, text="Izq:").grid(row=0, column=0)
        self.left_sensor = ttk.Label(sensor_frame, text="0", width=5, relief=tk.SUNKEN)
        self.left_sensor.grid(row=0, column=1, padx=2)
        
        ttk.Label(sensor_frame, text="Front:").grid(row=0, column=2, padx=5)
        self.front_sensor = ttk.Label(sensor_frame, text="0", width=5, relief=tk.SUNKEN)
        self.front_sensor.grid(row=0, column=3, padx=2)
        
        ttk.Label(sensor_frame, text="Der:").grid(row=0, column=4, padx=5)
        self.right_sensor = ttk.Label(sensor_frame, text="0", width=5, relief=tk.SUNKEN)
        self.right_sensor.grid(row=0, column=5, padx=2)
        
        # Log de mensajes
        log_frame = ttk.LabelFrame(right_frame, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, height=10, width=40)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
        
    def init_maze(self):
        """Inicializar el laberinto con algunas paredes"""
        # Paredes exteriores
        self.walls_h[0, :] = True
        self.walls_h[self.maze_size, :] = True
        self.walls_v[:, 0] = True
        self.walls_v[:, self.maze_size] = True
        
        # Agregar algunas paredes internas para hacer el laberinto interesante
        # Esto es solo un ejemplo, puedes modificar el patrón
        
        # Paredes horizontales
        self.walls_h[4, 2:6] = True
        self.walls_h[8, 10:14] = True
        self.walls_h[12, 4:8] = True
        
        # Paredes verticales
        self.walls_v[2:6, 4] = True
        self.walls_v[10:14, 8] = True
        self.walls_v[4:8, 12] = True
        
        # Algunas paredes aleatorias
        for _ in range(20):
            x, y = random.randint(1, self.maze_size-2), random.randint(1, self.maze_size-2)
            if random.choice([True, False]):
                self.walls_h[y, x] = True
            else:
                self.walls_v[x, y] = True
                
    def init_flood_values(self):
        """Inicializar los valores de flood fill"""
        # Inicializar todos con valor máximo
        self.flood_values.fill(255)
        
        # Establecer el centro (meta) con valor 0
        for pos in self.goal_pos:
            self.flood_values[pos[0], pos[1]] = 0
            
        # Ejecutar flood fill inicial
        self.update_flood_values()
        
    def update_flood_values(self):
        """Actualizar los valores usando el algoritmo flood fill"""
        changed = True
        while changed:
            changed = False
            for y in range(self.maze_size):
                for x in range(self.maze_size):
                    if (x, y) not in self.goal_pos:
                        min_neighbor = 255
                        
                        # Verificar vecinos accesibles
                        # Norte
                        if y > 0 and not self.walls_h[y, x]:
                            min_neighbor = min(min_neighbor, self.flood_values[y-1, x])
                        # Sur
                        if y < self.maze_size-1 and not self.walls_h[y+1, x]:
                            min_neighbor = min(min_neighbor, self.flood_values[y+1, x])
                        # Oeste
                        if x > 0 and not self.walls_v[x, y]:
                            min_neighbor = min(min_neighbor, self.flood_values[y, x-1])
                        # Este
                        if x < self.maze_size-1 and not self.walls_v[x+1, y]:
                            min_neighbor = min(min_neighbor, self.flood_values[y, x+1])
                        
                        if min_neighbor != 255 and self.flood_values[y, x] != min_neighbor + 1:
                            self.flood_values[y, x] = min_neighbor + 1
                            changed = True
                            
    def draw_maze(self):
        """Dibujar el laberinto en el canvas"""
        self.canvas.delete("all")
        
        offset = 10
        
        # Dibujar celdas y valores de flood fill
        for y in range(self.maze_size):
            for x in range(self.maze_size):
                x1 = x * self.cell_size + offset
                y1 = y * self.cell_size + offset
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                # Color de fondo basado en si fue visitado
                if self.visited[y, x]:
                    color = "#e0ffe0"
                else:
                    color = "white"
                    
                # Resaltar la meta
                if (y, x) in self.goal_pos:
                    color = "#ffcccc"
                    
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color, outline=""
                )
                
                # Mostrar valor de flood fill
                if self.flood_values[y, x] < 255:
                    self.canvas.create_text(
                        x1 + self.cell_size//2,
                        y1 + self.cell_size//2,
                        text=str(self.flood_values[y, x]),
                        font=("Arial", 8),
                        fill="gray"
                    )
        
        # Dibujar paredes horizontales
        for y in range(self.maze_size + 1):
            for x in range(self.maze_size):
                if self.walls_h[y, x]:
                    x1 = x * self.cell_size + offset
                    y1 = y * self.cell_size + offset
                    x2 = x1 + self.cell_size
                    self.canvas.create_line(
                        x1, y1, x2, y1,
                        width=self.wall_width, fill="black"
                    )
        
        # Dibujar paredes verticales
        for y in range(self.maze_size):
            for x in range(self.maze_size + 1):
                if self.walls_v[x, y]:
                    x1 = x * self.cell_size + offset
                    y1 = y * self.cell_size + offset
                    y2 = y1 + self.cell_size
                    self.canvas.create_line(
                        x1, y1, x1, y2,
                        width=self.wall_width, fill="black"
                    )
        
        # Dibujar el robot
        self.draw_robot()
        
    def draw_robot(self):
        """Dibujar el robot en su posición actual"""
        x, y = self.robot_pos
        offset = 10
        
        x_center = x * self.cell_size + self.cell_size // 2 + offset
        y_center = y * self.cell_size + self.cell_size // 2 + offset
        
        # Cuerpo del robot
        self.canvas.create_oval(
            x_center - 10, y_center - 10,
            x_center + 10, y_center + 10,
            fill="blue", outline="darkblue", width=2
        )
        
        # Indicador de dirección
        directions = [
            (0, -15),   # Norte
            (15, 0),    # Este
            (0, 15),    # Sur
            (-15, 0)    # Oeste
        ]
        dx, dy = directions[self.robot_direction]
        self.canvas.create_line(
            x_center, y_center,
            x_center + dx, y_center + dy,
            width=3, fill="red", arrow=tk.LAST
        )
        
    def refresh_ports(self):
        """Actualizar la lista de puertos COM disponibles"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
        self.log("Puertos actualizados")
        
    def toggle_connection(self):
        """Conectar o desconectar del puerto serial"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
            
    def connect_serial(self):
        """Conectar al puerto serial"""
        if self.simulation_mode:
            self.log("Modo simulación activado")
            self.connect_btn.config(text="Desconectar")
            return
            
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Selecciona un puerto COM")
            return
            
        try:
            self.serial_port = serial.Serial(port, 9600, timeout=0.1)
            self.running = True
            self.serial_thread = threading.Thread(target=self.serial_reader)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            self.connect_btn.config(text="Desconectar")
            self.log(f"Conectado a {port}")
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar: {e}")
            
    def disconnect_serial(self):
        """Desconectar del puerto serial"""
        self.running = False
        
        if self.serial_thread:
            self.serial_thread.join(timeout=1)
            
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
        self.connect_btn.config(text="Conectar")
        self.log("Desconectado")
        
    def serial_reader(self):
        """Thread para leer datos del puerto serial"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.readline().decode().strip()
                    if data:
                        self.data_queue.put(data)
            except Exception as e:
                print(f"Error leyendo serial: {e}")
            time.sleep(0.01)
            
    def toggle_simulation_mode(self):
        """Cambiar entre modo simulación y modo real"""
        self.simulation_mode = self.simulation_var.get()
        if self.simulation_mode:
            self.log("Modo simulación activado")
        else:
            self.log("Modo real activado")
            
    def send_command(self, command):
        """Enviar comando al Arduino"""
        if self.simulation_mode:
            # En modo simulación, procesar comandos localmente
            self.process_simulated_command(command)
        elif self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode())
                self.log(f"Enviado: {command}")
            except Exception as e:
                self.log(f"Error enviando: {e}")
                
    def process_simulated_command(self, command):
        """Procesar comandos en modo simulación"""
        if command == "F":  # Forward
            self.move_forward_simulated()
        elif command == "L":  # Turn left
            self.turn_left_simulated()
        elif command == "R":  # Turn right
            self.turn_right_simulated()
        elif command == "S":  # Read sensors
            self.read_sensors_simulated()
            
    def move_forward_simulated(self):
        """Mover hacia adelante en simulación"""
        x, y = self.robot_pos
        
        if self.robot_direction == 0:  # Norte
            if y > 0 and not self.walls_h[y, x]:
                self.robot_pos[1] -= 1
        elif self.robot_direction == 1:  # Este
            if x < self.maze_size-1 and not self.walls_v[x+1, y]:
                self.robot_pos[0] += 1
        elif self.robot_direction == 2:  # Sur
            if y < self.maze_size-1 and not self.walls_h[y+1, x]:
                self.robot_pos[1] += 1
        elif self.robot_direction == 3:  # Oeste
            if x > 0 and not self.walls_v[x, y]:
                self.robot_pos[0] -= 1
                
        self.visited[self.robot_pos[1], self.robot_pos[0]] = True
        self.update_position_display()
        
    def turn_left_simulated(self):
        """Girar a la izquierda en simulación"""
        self.robot_direction = (self.robot_direction - 1) % 4
        self.update_direction_display()
        
    def turn_right_simulated(self):
        """Girar a la derecha en simulación"""
        self.robot_direction = (self.robot_direction + 1) % 4
        self.update_direction_display()
        
    def read_sensors_simulated(self):
        """Leer sensores en simulación"""
        x, y = self.robot_pos
        
        # Sensor frontal
        front = 0
        if self.robot_direction == 0 and (y == 0 or self.walls_h[y, x]):
            front = 1
        elif self.robot_direction == 1 and (x == self.maze_size-1 or self.walls_v[x+1, y]):
            front = 1
        elif self.robot_direction == 2 and (y == self.maze_size-1 or self.walls_h[y+1, x]):
            front = 1
        elif self.robot_direction == 3 and (x == 0 or self.walls_v[x, y]):
            front = 1
            
        # Sensor izquierdo
        left_dir = (self.robot_direction - 1) % 4
        left = 0
        if left_dir == 0 and (y == 0 or self.walls_h[y, x]):
            left = 1
        elif left_dir == 1 and (x == self.maze_size-1 or self.walls_v[x+1, y]):
            left = 1
        elif left_dir == 2 and (y == self.maze_size-1 or self.walls_h[y+1, x]):
            left = 1
        elif left_dir == 3 and (x == 0 or self.walls_v[x, y]):
            left = 1
            
        # Sensor derecho
        right_dir = (self.robot_direction + 1) % 4
        right = 0
        if right_dir == 0 and (y == 0 or self.walls_h[y, x]):
            right = 1
        elif right_dir == 1 and (x == self.maze_size-1 or self.walls_v[x+1, y]):
            right = 1
        elif right_dir == 2 and (y == self.maze_size-1 or self.walls_h[y+1, x]):
            right = 1
        elif right_dir == 3 and (x == 0 or self.walls_v[x, y]):
            right = 1
            
        self.update_sensor_display(left, front, right)
        return left, front, right
        
    def start_exploration(self):
        """Iniciar la exploración del laberinto"""
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.exploring = True
        
        # Thread de exploración
        self.exploration_thread = threading.Thread(target=self.exploration_algorithm)
        self.exploration_thread.daemon = True
        self.exploration_thread.start()
        
        self.log("Exploración iniciada")
        
    def stop_exploration(self):
        """Detener la exploración"""
        self.exploring = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.log("Exploración detenida")
        
    def exploration_algorithm(self):
        """Algoritmo principal de exploración usando flood fill"""
        while self.exploring:
            # Leer sensores
            left, front, right = self.read_sensors_simulated()
            
            # Actualizar paredes conocidas basándose en sensores
            self.update_walls_from_sensors(left, front, right)
            
            # Actualizar valores de flood fill
            self.update_flood_values()
            
            # Decidir siguiente movimiento
            next_move = self.decide_next_move()
            
            # Ejecutar movimiento
            if next_move == "F":
                self.send_command("F")
                time.sleep(0.5)  # Delay para visualización
            elif next_move == "L":
                self.send_command("L")
                time.sleep(0.3)
            elif next_move == "R":
                self.send_command("R")
                time.sleep(0.3)
            elif next_move == "GOAL":
                self.log("¡Meta alcanzada!")
                self.stop_exploration()
                break
                
            # Actualizar visualización
            self.root.after(0, self.draw_maze)
            
            # Verificar si llegamos a la meta
            if tuple(self.robot_pos) in [(y, x) for x, y in self.goal_pos]:
                self.log("¡Meta alcanzada!")
                self.stop_exploration()
                break
                
    def update_walls_from_sensors(self, left, front, right):
        """Actualizar el conocimiento de paredes basándose en los sensores"""
        x, y = self.robot_pos
        
        # Pared frontal
        if front == 1:
            if self.robot_direction == 0:  # Norte
                self.walls_h[y, x] = True
            elif self.robot_direction == 1:  # Este
                self.walls_v[x+1, y] = True
            elif self.robot_direction == 2:  # Sur
                self.walls_h[y+1, x] = True
            elif self.robot_direction == 3:  # Oeste
                self.walls_v[x, y] = True
                
        # Pared izquierda
        if left == 1:
            left_dir = (self.robot_direction - 1) % 4
            if left_dir == 0:  # Norte
                self.walls_h[y, x] = True
            elif left_dir == 1:  # Este
                self.walls_v[x+1, y] = True
            elif left_dir == 2:  # Sur
                self.walls_h[y+1, x] = True
            elif left_dir == 3:  # Oeste
                self.walls_v[x, y] = True
                
        # Pared derecha
        if right == 1:
            right_dir = (self.robot_direction + 1) % 4
            if right_dir == 0:  # Norte
                self.walls_h[y, x] = True
            elif right_dir == 1:  # Este
                self.walls_v[x+1, y] = True
            elif right_dir == 2:  # Sur
                self.walls_h[y+1, x] = True
            elif right_dir == 3:  # Oeste
                self.walls_v[x, y] = True
                
    def decide_next_move(self):
        """Decidir el siguiente movimiento basándose en flood fill"""
        x, y = self.robot_pos
        
        # Si estamos en la meta
        if tuple(self.robot_pos) in [(y, x) for x, y in self.goal_pos]:
            return "GOAL"
            
        # Obtener valores de las celdas adyacentes accesibles
        neighbors = []
        
        # Frontal
        if self.robot_direction == 0 and y > 0 and not self.walls_h[y, x]:
            neighbors.append(("F", self.flood_values[y-1, x]))
        elif self.robot_direction == 1 and x < self.maze_size-1 and not self.walls_v[x+1, y]:
            neighbors.append(("F", self.flood_values[y, x+1]))
        elif self.robot_direction == 2 and y < self.maze_size-1 and not self.walls_h[y+1, x]:
            neighbors.append(("F", self.flood_values[y+1, x]))
        elif self.robot_direction == 3 and x > 0 and not self.walls_v[x, y]:
            neighbors.append(("F", self.flood_values[y, x-1]))
            
        # Izquierda (requiere girar)
        left_dir = (self.robot_direction - 1) % 4
        if left_dir == 0 and y > 0 and not self.walls_h[y, x]:
            neighbors.append(("L", self.flood_values[y-1, x]))
        elif left_dir == 1 and x < self.maze_size-1 and not self.walls_v[x+1, y]:
            neighbors.append(("L", self.flood_values[y, x+1]))
        elif left_dir == 2 and y < self.maze_size-1 and not self.walls_h[y+1, x]:
            neighbors.append(("L", self.flood_values[y+1, x]))
        elif left_dir == 3 and x > 0 and not self.walls_v[x, y]:
            neighbors.append(("L", self.flood_values[y, x-1]))
            
        # Derecha (requiere girar)
        right_dir = (self.robot_direction + 1) % 4
        if right_dir == 0 and y > 0 and not self.walls_h[y, x]:
            neighbors.append(("R", self.flood_values[y-1, x]))
        elif right_dir == 1 and x < self.maze_size-1 and not self.walls_v[x+1, y]:
            neighbors.append(("R", self.flood_values[y, x+1]))
        elif right_dir == 2 and y < self.maze_size-1 and not self.walls_h[y+1, x]:
            neighbors.append(("R", self.flood_values[y+1, x]))
        elif right_dir == 3 and x > 0 and not self.walls_v[x, y]:
            neighbors.append(("R", self.flood_values[y, x-1]))
            
        # Elegir el vecino con menor valor
        if neighbors:
            best_move = min(neighbors, key=lambda x: x[1])
            return best_move[0]
        else:
            # No hay movimientos disponibles
            return "STOP"
            
    def reset_maze(self):
        """Reiniciar el laberinto y el robot"""
        self.robot_pos = [0, 0]
        self.robot_direction = 0
        self.visited = np.zeros((self.maze_size, self.maze_size), dtype=bool)
        self.visited[0, 0] = True
        
        # Reiniciar valores de flood fill
        self.init_flood_values()
        
        # Actualizar displays
        self.update_position_display()
        self.update_direction_display()
        self.draw_maze()
        
        self.log("Laberinto reiniciado")
        
    def update_position_display(self):
        """Actualizar la visualización de la posición"""
        self.pos_label.config(text=f"({self.robot_pos[0]}, {self.robot_pos[1]})")
        
    def update_direction_display(self):
        """Actualizar la visualización de la dirección"""
        directions = ["Norte", "Este", "Sur", "Oeste"]
        self.dir_label.config(text=directions[self.robot_direction])
        
    def update_sensor_display(self, left, front, right):
        """Actualizar la visualización de los sensores"""
        self.left_sensor.config(text=str(left))
        self.front_sensor.config(text=str(front))
        self.right_sensor.config(text=str(right))
        
        # Cambiar color según detección
        self.left_sensor.config(background="red" if left else "green")
        self.front_sensor.config(background="red" if front else "green")
        self.right_sensor.config(background="red" if right else "green")
        
    def log(self, message):
        """Agregar mensaje al log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
    def update_loop(self):
        """Loop principal de actualización"""
        # Procesar datos de la cola
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.process_arduino_data(data)
            except queue.Empty:
                break
                
        # Actualizar el canvas
        self.draw_maze()
        
        # Programar siguiente actualización
        self.root.after(100, self.update_loop)
        
    def process_arduino_data(self, data):
        """Procesar datos recibidos del Arduino"""
        self.log(f"Recibido: {data}")
        
        # Formato esperado: "SENSORS:left,front,right"
        if data.startswith("SENSORS:"):
            values = data.split(":")[1].split(",")
            if len(values) == 3:
                left = int(values[0])
                front = int(values[1])
                right = int(values[2])
                self.update_sensor_display(left, front, right)
                
        # Otros formatos de datos pueden agregarse aquí

def main():
    root = tk.Tk()
    app = MicromouseSimulator(root)
    root.mainloop()

if __name__ == "__main__":
    main()