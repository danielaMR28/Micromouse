"""
Micromouse-Robotica 2025B
Visualizador de Laberinto para Micromouse V3.0
Con panel de calibración, modo carrera y control completo

Nuevas funciones:
- Panel de calibración de parámetros en tiempo real
- Botón DETENER MAPEO (envía STOP al Arduino)
- Botón RECALIBRAR (reposicionar robot)
- Botón RESOLVER (flood fill)
- Botón INICIAR CARRERA

Instalar: pip install pyserial pygame

Uso:
    python maze_mapper_v3.py COM5
    python maze_mapper_v3.py /dev/cu.HC-05
"""

# Librerías necesarias
import pygame  # Para la interfaz gráfica
import serial  # Para comunicación Bluetooth con Arduino
import sys  # Para argumentos de línea de comandos
import threading  # Para lectura asíncrona de datos Bluetooth
import time  # Para timestamps y delays
from collections import defaultdict  # Para almacenar paredes del laberinto

# Constantes de visualización del laberinto
CELL_SIZE_PX = 60  # Tamaño de cada celda en píxeles
MAZE_WIDTH = 12  # Ancho del laberinto en celdas
MAZE_HEIGHT = 7  # Alto del laberinto en celdas
WALL_THICKNESS = 4  # Grosor de las paredes en píxeles

# Paleta de colores RGB para la interfaz
WHITE = (255, 255, 255)  # Fondo general
BLACK = (0, 0, 0)  # Paredes detectadas
GRAY = (200, 200, 200)  # Líneas de grid
LIGHT_GRAY = (240, 240, 240)  # Fondo del panel
DARK_GRAY = (100, 100, 100)  # Texto secundario
RED = (255, 80, 80)  # Robot/sensores en alerta
DARK_RED = (180, 50, 50)  # Botón detener
GREEN = (80, 200, 80)  # Sensores OK/conexión activa
DARK_GREEN = (50, 150, 50)  # Botón iniciar
BLUE = (80, 80, 255)  # Botones secundarios
YELLOW = (255, 220, 80)  # Celdas meta/inicio
LIGHT_GREEN = (220, 255, 220)  # Celdas visitadas
LIGHT_BLUE = (200, 220, 255)  # Trayectoria recorrida
ORANGE = (255, 165, 0)  # Modo calibración
PURPLE = (180, 100, 255)  # Modo carrera/mejor ruta
CYAN = (100, 200, 200)  # Modo recalibración

# Sistema de direcciones (0=Norte, 1=Este, 2=Sur, 3=Oeste)
DIR_NAMES = ['Norte ↑', 'Este →', 'Sur ↓', 'Oeste ←']  # Nombres para mostrar
DIR_DX = [0, 1, 0, -1]  # Desplazamiento en X para cada dirección
DIR_DY = [-1, 0, 1, 0]  # Desplazamiento en Y para cada dirección


class CalibrationParams:
    """Almacena parámetros de calibración del robot que se envían al Arduino"""
    def __init__(self):
        self.speed_normal = 80  # Velocidad normal de avance (PWM)
        self.speed_turn = 85  # Velocidad durante giros (PWM)
        self.speed_race = 100  # Velocidad en modo carrera (PWM)
        self.turn_angle = 38  # Ángulo de giro en grados
        self.front_stop = 8  # Distancia frontal para detenerse (cm)
        self.pulses_per_cell = 25  # Pulsos del encoder por celda
        self.kp_wall = 18  # Ganancia proporcional para seguimiento de pared (x10)
        self.correction_factor = 30  # Factor de corrección de trayectoria
    
    def to_command(self):
        """Convierte los parámetros a un comando para enviar al Arduino"""
        return f"PARAMS,{self.speed_normal},{self.speed_turn},{self.turn_angle},{self.kp_wall},{self.front_stop},{self.pulses_per_cell},{self.correction_factor},{self.speed_race}"
    
    def from_arduino(self, values):
        """Actualiza los parámetros desde los valores recibidos del Arduino"""
        if len(values) >= 6:
            self.speed_normal = int(values[0])
            self.speed_turn = int(values[1])
            self.turn_angle = int(values[2])
            self.kp_wall = int(values[3])
            self.front_stop = int(values[4])
            self.pulses_per_cell = int(values[5])
            if len(values) >= 7: self.correction_factor = int(values[6])
            if len(values) >= 8: self.speed_race = int(values[7])


class MazeMapper:
    """Clase principal que maneja el mapeo del laberinto y el estado del robot"""
    def __init__(self):
        # Estructura del laberinto
        self.walls = defaultdict(lambda: {0: None, 1: None, 2: None, 3: None})  # Paredes por celda (None=desconocido, True=pared, False=libre)
        self.visited = set()  # Conjunto de celdas visitadas
        self.path = []  # Trayectoria recorrida por el robot
        
        # Posición inicial
        self.start_x = 0  # Coordenada X de inicio
        self.start_y = 0  # Coordenada Y de inicio
        self.start_set = False  # Indica si ya se estableció el punto de inicio
        
        # Posición actual del robot
        self.mouse_x = 0  # Coordenada X actual
        self.mouse_y = 0  # Coordenada Y actual
        self.mouse_dir = 0  # Dirección actual (0-3)
        
        # Offset para coordenadas del Arduino
        self.offset_x = 0  # Desplazamiento en X
        self.offset_y = 0  # Desplazamiento en Y
        
        # Lecturas de sensores
        self.dist_front = 0  # Distancia frontal en cm
        self.dist_left = 0  # Distancia izquierda en cm
        self.dist_right = 0  # Distancia derecha en cm
        self.yaw = 0.0  # Ángulo del giroscopio
        
        # Estado de conexión y mensajes
        self.connected = False  # Estado de conexión Bluetooth
        self.messages = []  # Log de mensajes
        self.robot_state = "WAITING"  # Estado actual del robot
        
        # Algoritmo de resolución
        self.goal_cells = set()  # Celdas meta
        self.flood_values = {}  # Valores del algoritmo flood fill
        self.best_path = []  # Mejor ruta calculada hacia la meta
        
        # Parámetros de calibración
        self.params = CalibrationParams()  # Objeto con parámetros del robot
    
    def set_start_position(self, x, y):
        """Establece la posición inicial del robot en el laberinto"""
        self.start_x = x
        self.start_y = y
        self.start_set = True
        self.mouse_x = x
        self.mouse_y = y
        self.offset_x = x  # Guarda el offset para convertir coordenadas del Arduino
        self.offset_y = y
        self.path = [(x, y)]  # Inicia la trayectoria
        self.visited.add((x, y))  # Marca como visitada
        self.add_message(f"Inicio en ({x}, {y})")
    
    def update_position_from_arduino(self, ax, ay, direction):
        """Actualiza la posición del robot desde las coordenadas recibidas del Arduino"""
        self.mouse_x = self.offset_x + ax  # Convierte coordenadas relativas a absolutas
        self.mouse_y = self.offset_y + ay
        self.mouse_dir = direction  # Actualiza dirección
        self.visited.add((self.mouse_x, self.mouse_y))  # Marca celda como visitada
        # Añade a la trayectoria si es una nueva posición
        if len(self.path) == 0 or self.path[-1] != (self.mouse_x, self.mouse_y):
            self.path.append((self.mouse_x, self.mouse_y))
    
    def update_walls_from_sensors(self, wall_front, wall_left, wall_right, direction):
        """Actualiza el mapa de paredes basado en las lecturas de los sensores"""
        x, y = self.mouse_x, self.mouse_y
        self.visited.add((x, y))
        
        # Calcula las direcciones absolutas de cada sensor
        dir_front = direction
        dir_left = (direction + 3) % 4  # 90° a la izquierda
        dir_right = (direction + 1) % 4  # 90° a la derecha
        
        # Registra las paredes detectadas en la celda actual
        self.walls[(x, y)][dir_front] = wall_front
        self.walls[(x, y)][dir_left] = wall_left
        self.walls[(x, y)][dir_right] = wall_right
        
        # Actualiza también las paredes de las celdas adyacentes (simetría)
        for d, w in [(dir_front, wall_front), (dir_left, wall_left), (dir_right, wall_right)]:
            nx = x + DIR_DX[d]
            ny = y + DIR_DY[d]
            self.walls[(nx, ny)][(d + 2) % 4] = w  # Dirección opuesta
    
    def add_message(self, msg):
        """Añade un mensaje al log con timestamp"""
        self.messages.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        if len(self.messages) > 15:  # Limita el historial a 15 mensajes
            self.messages.pop(0)
    
    def reset_mapping(self):
        """Resetea el mapeo pero mantiene la posición de inicio"""
        self.walls.clear()  # Limpia todas las paredes
        self.visited.clear()  # Limpia celdas visitadas
        self.path = [(self.start_x, self.start_y)] if self.start_set else []  # Reinicia trayectoria
        self.mouse_x = self.start_x  # Vuelve al inicio
        self.mouse_y = self.start_y
        self.mouse_dir = 0  # Dirección norte
        if self.start_set:
            self.visited.add((self.start_x, self.start_y))
        self.best_path.clear()  # Limpia la ruta calculada
        self.add_message("Mapeo reseteado")
    
    def full_reset(self):
        """Resetea completamente el sistema incluyendo posición de inicio y metas"""
        self.walls.clear()
        self.visited.clear()
        self.path = []
        self.start_x = 0
        self.start_y = 0
        self.start_set = False  # Requiere establecer nuevo inicio
        self.mouse_x = 0
        self.mouse_y = 0
        self.offset_x = 0
        self.offset_y = 0
        self.mouse_dir = 0
        self.robot_state = "WAITING"  # Vuelve a estado inicial
        self.goal_cells.clear()  # Limpia las metas
        self.flood_values.clear()  # Limpia valores de flood fill
        self.best_path.clear()
        self.add_message("Reset completo")
    
    def toggle_goal(self, x, y):
        """Añade o remueve una celda como meta"""
        if (x, y) in self.goal_cells:
            self.goal_cells.remove((x, y))
            self.add_message(f"Meta removida ({x}, {y})")
        else:
            self.goal_cells.add((x, y))
            self.add_message(f"Meta agregada ({x}, {y})")
    
    def calculate_flood_fill(self):
        """Calcula distancias desde cada celda hasta las metas usando algoritmo flood fill"""
        if not self.goal_cells:
            self.add_message("No hay metas")
            return False
        
        # Inicializa todas las celdas con valor alto (999 = infinito)
        self.flood_values = {(x, y): 999 for x in range(MAZE_WIDTH) for y in range(MAZE_HEIGHT)}
        
        # Las celdas meta tienen valor 0
        queue = list(self.goal_cells)
        for cell in self.goal_cells:
            self.flood_values[cell] = 0
        
        # Propaga valores desde las metas hacia afuera
        while queue:
            x, y = queue.pop(0)
            val = self.flood_values[(x, y)]
            
            # Revisa las 4 direcciones
            for d in range(4):
                nx = x + DIR_DX[d]
                ny = y + DIR_DY[d]
                
                # Verifica límites del laberinto
                if not (0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT):
                    continue
                
                # Si hay pared, no puede pasar
                if self.walls.get((x, y), {}).get(d) == True:
                    continue
                
                # Actualiza si encuentra un camino más corto
                if val + 1 < self.flood_values[(nx, ny)]:
                    self.flood_values[(nx, ny)] = val + 1
                    queue.append((nx, ny))
        
        self.calculate_best_path()  # Calcula la mejor ruta
        self.add_message("Flood fill OK")
        return True
    
    def calculate_best_path(self):
        """Calcula la mejor ruta desde el inicio hasta la meta usando los valores de flood fill"""
        if not self.goal_cells or not self.start_set:
            self.best_path = []
            return
        
        self.best_path = []
        current = (self.start_x, self.start_y)  # Comienza desde el inicio
        visited = {current}  # Evita ciclos
        
        # Sigue el gradiente descendente hasta llegar a la meta
        while current not in self.goal_cells and len(self.best_path) < 200:
            self.best_path.append(current)
            x, y = current
            val = self.flood_values.get(current, 999)
            best = None
            best_val = val
            
            # Busca el vecino con menor valor de flood fill
            for d in range(4):
                nx = x + DIR_DX[d]
                ny = y + DIR_DY[d]
                
                # Verifica límites
                if not (0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT):
                    continue
                
                # Verifica paredes
                if self.walls.get((x, y), {}).get(d) == True:
                    continue
                
                # Evita volver a celdas ya visitadas
                if (nx, ny) in visited:
                    continue
                
                # Selecciona el vecino con menor valor
                nv = self.flood_values.get((nx, ny), 999)
                if nv < best_val:
                    best_val = nv
                    best = (nx, ny)
            
            if best is None:  # No hay camino
                break
            
            current = best
            visited.add(current)
        
        if current in self.goal_cells:
            self.best_path.append(current)  # Añade la celda meta
            self.add_message(f"Ruta: {len(self.best_path)} celdas")
    
    def get_path_command(self):
        """Genera el comando para enviar la ruta al Arduino"""
        if not self.best_path:
            return None
        
        # Convierte coordenadas absolutas a relativas (para el Arduino)
        path_arduino = [(x - self.offset_x, y - self.offset_y) for (x, y) in self.best_path]
        
        # Formato: PATH,cantidad,x1,y1,x2,y2,...
        cmd = f"PATH,{len(path_arduino)}"
        for (x, y) in path_arduino:
            cmd += f",{x},{y}"
        return cmd
    
    def load_dummy_maze(self):
        """Carga un laberinto de prueba con paredes predefinidas"""
        # Crea los bordes del laberinto
        for x in range(MAZE_WIDTH):
            self.walls[(x, 0)][0] = True  # Borde superior
            self.walls[(x, MAZE_HEIGHT-1)][2] = True  # Borde inferior
        for y in range(MAZE_HEIGHT):
            self.walls[(0, y)][3] = True  # Borde izquierdo
            self.walls[(MAZE_WIDTH-1, y)][1] = True  # Borde derecho
        
        # Define paredes internas (x, y, dirección)
        walls = [(1,1,1),(2,1,2),(3,1,3),(4,1,1),(1,2,0),(3,2,1),(5,2,2),
                 (2,3,3),(4,3,0),(6,3,1),(1,4,1),(3,4,2),(5,4,3),
                 (2,5,0),(4,5,1),(6,5,2),(7,1,2),(8,2,1),(9,3,0),(7,4,1),(8,5,2),(10,2,3)]
        
        # Coloca las paredes internas
        for x, y, d in walls:
            if 0 <= x < MAZE_WIDTH and 0 <= y < MAZE_HEIGHT:
                self.walls[(x, y)][d] = True
                # Actualiza la pared simétrica en la celda adyacente
                nx = x + DIR_DX[d]
                ny = y + DIR_DY[d]
                if 0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT:
                    self.walls[(nx, ny)][(d + 2) % 4] = True
        
        # Marca algunas celdas como visitadas para simular exploración
        for x in range(MAZE_WIDTH):
            for y in range(MAZE_HEIGHT):
                if (x + y) % 3 == 0:
                    self.visited.add((x, y))
        
        self.add_message("Laberinto de prueba")


class BluetoothReader:
    """Maneja la comunicación serial Bluetooth con el Arduino"""
    def __init__(self, port, maze):
        self.port = port  # Puerto serial (ej: COM5, /dev/cu.HC-05)
        self.maze = maze  # Referencia al objeto MazeMapper
        self.serial = None  # Objeto de conexión serial
        self.running = False  # Estado del hilo de lectura
    
    def connect(self):
        """Establece la conexión serial con el Arduino"""
        try:
            self.serial = serial.Serial(self.port, 9600, timeout=1)  # 9600 baudios
            time.sleep(2)  # Espera a que se estabilice la conexión
            self.maze.connected = True
            self.maze.add_message(f"Conectado a {self.port}")
            return True
        except Exception as e:
            self.maze.add_message(f"Error: {str(e)[:25]}")
            return False
    
    def send_command(self, cmd):
        """Envía un comando al Arduino por serial"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{cmd}\n".encode())  # Envía con salto de línea
                self.serial.flush()  # Asegura que se envíe inmediatamente
                self.maze.add_message(f"> {cmd[:25]}")
                return True
            except:
                pass
        return False
    
    def send_go(self):
        """Inicia el mapeo del laberinto"""
        if self.send_command("GO"):
            self.maze.robot_state = "MAPPING"
            return True
        return False
    
    def send_stop(self):
        """Detiene el robot"""
        if self.send_command("STOP"):
            self.maze.robot_state = "STOPPED"
            return True
        return False
    
    def send_recal(self):
        """Activa el modo de recalibración (reposicionar robot)"""
        if self.send_command("RECAL"):
            self.maze.robot_state = "RECAL"
            return True
        return False
    
    def send_recal_done(self):
        """Confirma que la recalibración está completa"""
        if self.send_command("RECAL_DONE"):
            self.maze.robot_state = "RACE_READY"
            return True
        return False
    
    def send_race(self):
        """Envía la ruta calculada e inicia el modo carrera"""
        path_cmd = self.maze.get_path_command()
        if path_cmd:
            self.send_command(path_cmd)  # Envía primero la ruta
            time.sleep(0.1)  # Pequeña pausa para que el Arduino procese
            if self.send_command("RACE"):  # Luego inicia la carrera
                self.maze.robot_state = "RACING"
                return True
        return False
    
    def send_params(self):
        """Envía los parámetros de calibración al Arduino"""
        return self.send_command(self.maze.params.to_command())
    
    def request_params(self):
        """Solicita los parámetros actuales del Arduino"""
        return self.send_command("GET_PARAMS")
    
    def start(self):
        """Inicia el hilo de lectura asíncrona de datos del Arduino"""
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
    
    def stop(self):
        """Detiene el hilo de lectura y cierra la conexión serial"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def _read_loop(self):
        """Bucle que lee continuamente datos del Arduino en un hilo separado"""
        while self.running:
            try:
                # Verifica si hay datos disponibles en el buffer serial
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    # Lee una línea completa
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._parse(line)  # Procesa el mensaje recibido
            except:
                pass
            time.sleep(0.01)  # Pequeña pausa para no saturar el CPU
    
    def _parse(self, line):
        """Interpreta los mensajes recibidos del Arduino y actualiza el estado"""
        try:
            # Mensajes de estado simples
            if line == "READY":
                self.maze.add_message("Arduino listo")
                self.request_params()  # Solicita parámetros al conectar
            elif line == "WAITING":
                self.maze.robot_state = "WAITING"  # Esperando comando GO
            elif line == "STARTING":
                self.maze.robot_state = "MAPPING"  # Iniciando mapeo
                self.maze.add_message("Mapeando...")
            elif line == "STOPPED":
                self.maze.robot_state = "STOPPED"  # Robot detenido
                self.maze.add_message("Detenido")
            elif line == "RECAL_MODE":
                self.maze.robot_state = "RECAL"  # Modo recalibración activo
                self.maze.add_message("Modo recalibracion")
            elif line == "RACE_READY":
                self.maze.robot_state = "RACE_READY"  # Listo para iniciar carrera
                self.maze.add_message("Listo para carrera")
            elif line == "RACING":
                self.maze.robot_state = "RACING"  # Ejecutando carrera
                self.maze.add_message("Carrera iniciada")
            elif line == "RACE_COMPLETE":
                self.maze.robot_state = "STOPPED"  # Carrera completada
                self.maze.add_message("Meta alcanzada!")
            elif line == "GYRO_CAL_DONE":
                self.maze.add_message("Giroscopio OK")  # Calibración del giroscopio completa
            elif line == "PARAMS_SET":
                self.maze.add_message("Params actualizados")  # Parámetros recibidos por Arduino
            
            # Mensajes con datos (formato CSV)
            else:
                parts = line.split(',')  # Separa por comas
                # Parámetros actuales del Arduino
                if parts[0] == "CURRENT_PARAMS" and len(parts) >= 7:
                    self.maze.params.from_arduino(parts[1:])
                    self.maze.add_message("Params sincronizados")
                
                # Nueva celda alcanzada durante mapeo
                elif parts[0] == "NEW_CELL" and len(parts) >= 4:
                    self.maze.update_position_from_arduino(int(parts[1]), int(parts[2]), int(parts[3]))
                
                # Cambio de dirección
                elif parts[0] == "DIR_CHANGE" and len(parts) >= 2:
                    self.maze.mouse_dir = int(parts[1])
                
                # Datos completos: sensores, paredes, dirección, yaw
                elif parts[0] == "DATA" and len(parts) >= 11:
                    self.maze.dist_front = int(parts[1])  # Distancia frontal
                    self.maze.dist_left = int(parts[2])  # Distancia izquierda
                    self.maze.dist_right = int(parts[3])  # Distancia derecha
                    self.maze.mouse_dir = int(parts[7])  # Dirección
                    self.maze.yaw = float(parts[8])  # Ángulo del giroscopio
                    # Actualiza paredes (parts[4-6] son booleanos "0" o "1")
                    self.maze.update_walls_from_sensors(
                        parts[4] == "1", parts[5] == "1", parts[6] == "1", int(parts[7]))
                
                # Celda alcanzada durante carrera
                elif parts[0] == "RACE_CELL" and len(parts) >= 4:
                    self.maze.mouse_x = self.maze.offset_x + int(parts[1])
                    self.maze.mouse_y = self.maze.offset_y + int(parts[2])
                
                # Confirmación de ruta cargada
                elif parts[0] == "PATH_LOADED" and len(parts) >= 2:
                    self.maze.add_message(f"Ruta: {parts[1]} celdas")
                
                # Lecturas de sensores (modo recalibración o normal)
                elif parts[0] in ["SENSORS", "RECAL_SENSORS"] and len(parts) >= 4:
                    self.maze.dist_front = int(parts[1])
                    self.maze.dist_left = int(parts[2])
                    self.maze.dist_right = int(parts[3])
        except:
            pass


class Slider:
    """Control deslizante para ajustar parámetros numéricos en la interfaz"""
    def __init__(self, x, y, w, min_v, max_v, val, label, step=1):
        self.rect = pygame.Rect(x, y, w, 18)  # Área del slider
        self.min_v = min_v  # Valor mínimo
        self.max_v = max_v  # Valor máximo
        self.value = val  # Valor actual
        self.label = label  # Etiqueta descriptiva
        self.step = step  # Incremento por paso
        self.dragging = False  # Estado de arrastre
    
    def draw(self, screen, font):
        """Dibuja el slider en la pantalla"""
        # Etiqueta con valor actual
        label = font.render(f"{self.label}: {self.value}", True, BLACK)
        screen.blit(label, (self.rect.x, self.rect.y - 14))
        
        # Barra del slider
        pygame.draw.rect(screen, GRAY, self.rect)
        pygame.draw.rect(screen, DARK_GRAY, self.rect, 1)
        
        # Barra de progreso (parte llena)
        ratio = (self.value - self.min_v) / (self.max_v - self.min_v)
        knob_x = int(self.rect.x + ratio * self.rect.width)
        pygame.draw.rect(screen, BLUE, (self.rect.x, self.rect.y, knob_x - self.rect.x, self.rect.height))
        
        # Perilla deslizante
        pygame.draw.circle(screen, WHITE, (knob_x, self.rect.centery), 7)
        pygame.draw.circle(screen, DARK_GRAY, (knob_x, self.rect.centery), 7, 2)
    
    def handle_event(self, event):
        """Maneja eventos de mouse para interacción con el slider"""
        if event.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(event.pos):
            self.dragging = True  # Inicia arrastre
            self._update(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False  # Termina arrastre
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._update(event.pos[0])  # Actualiza mientras arrastra
    
    def _update(self, mx):
        """Actualiza el valor del slider según la posición del mouse"""
        rx = max(0, min(mx - self.rect.x, self.rect.width))  # Limita al ancho del slider
        raw = self.min_v + (rx / self.rect.width) * (self.max_v - self.min_v)  # Calcula valor
        self.value = round(raw / self.step) * self.step  # Redondea según el paso


class MazeVisualizer:
    """Interfaz gráfica principal usando Pygame para visualizar el laberinto y controlar el robot"""
    def __init__(self, maze, bt_reader=None):
        pygame.init()  # Inicializa Pygame
        self.maze = maze  # Referencia al objeto MazeMapper
        self.bt_reader = bt_reader  # Referencia al lector Bluetooth
        
        # Dimensiones de la ventana
        self.maze_px_w = MAZE_WIDTH * CELL_SIZE_PX  # Ancho del laberinto en píxeles
        self.maze_px_h = MAZE_HEIGHT * CELL_SIZE_PX  # Alto del laberinto en píxeles
        self.panel_width = 380  # Ancho del panel de control
        self.width = self.maze_px_w + self.panel_width  # Ancho total de ventana
        self.height = max(self.maze_px_h, 720)  # Alto total de ventana
        
        # Configuración de la ventana
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("Micromouse-Robotica 2025B - Panel de Control")
        
        # Fuentes para texto
        self.font = pygame.font.Font(None, 20)  # Fuente normal
        self.font_large = pygame.font.Font(None, 26)  # Fuente grande
        self.font_small = pygame.font.Font(None, 16)  # Fuente pequeña
        self.font_btn = pygame.font.Font(None, 20)  # Fuente para botones
        
        # Control de FPS
        self.clock = pygame.time.Clock()
        
        # Posicionamiento del panel
        self.panel_x = self.maze_px_w + 12  # Posición X del panel
        self.btn_w = 168  # Ancho de botones
        self.btn_h = 32  # Alto de botones
        
        # Crea elementos de la interfaz
        self._create_buttons()
        self.show_calibration = False  # Panel de calibración oculto por defecto
        self._create_sliders()
    
    def _create_buttons(self):
        """Crea los rectángulos de todos los botones del panel de control"""
        px = self.panel_x
        y = 105
        gap = 38  # Espacio vertical entre filas de botones
        
        # Fila 1: Control de mapeo
        self.btn_go = pygame.Rect(px, y, self.btn_w, self.btn_h)  # Botón INICIAR
        self.btn_stop = pygame.Rect(px + self.btn_w + 8, y, self.btn_w, self.btn_h)  # Botón DETENER
        y += gap
        
        # Fila 2: Recalibración
        self.btn_recal = pygame.Rect(px, y, self.btn_w, self.btn_h)  # Botón RECALIBRAR
        self.btn_recal_done = pygame.Rect(px + self.btn_w + 8, y, self.btn_w, self.btn_h)  # Botón LISTO
        y += gap
        
        # Fila 3: Resolución y carrera
        self.btn_solve = pygame.Rect(px, y, self.btn_w, self.btn_h)  # Botón RESOLVER
        self.btn_race = pygame.Rect(px + self.btn_w + 8, y, self.btn_w, self.btn_h)  # Botón CARRERA
        y += gap
        
        # Fila 4: Calibración y prueba
        self.btn_calib = pygame.Rect(px, y, self.btn_w, self.btn_h)  # Botón CALIBRAR
        self.btn_dummy = pygame.Rect(px + self.btn_w + 8, y, self.btn_w, self.btn_h)  # Botón PRUEBA
        y += gap
        
        # Fila 5: Reset
        self.btn_reset = pygame.Rect(px, y, self.btn_w, self.btn_h)  # Botón RESET
        self.btn_full_reset = pygame.Rect(px + self.btn_w + 8, y, self.btn_w, self.btn_h)  # Botón FULL
    
    def _create_sliders(self):
        """Crea los sliders para ajustar parámetros de calibración"""
        px = self.panel_x
        y = 430  # Posición Y inicial
        w = 345  # Ancho de los sliders
        gap = 42  # Espacio vertical entre sliders
        
        # Diccionario de sliders para cada parámetro
        self.sliders = {
            'speed': Slider(px, y, w, 40, 150, 80, "Velocidad Normal", 5),  # PWM 40-150
            'turn_speed': Slider(px, y + gap, w, 40, 150, 85, "Velocidad Giro", 5),  # PWM 40-150
            'race_speed': Slider(px, y + gap*2, w, 60, 200, 100, "Velocidad Carrera", 5),  # PWM 60-200
            'turn_angle': Slider(px, y + gap*3, w, 30, 50, 38, "Angulo Giro", 1),  # Grados 30-50
            'kp_wall': Slider(px, y + gap*4, w, 5, 50, 18, "KP Pared (x10)", 1),  # Ganancia 5-50
            'front_stop': Slider(px, y + gap*5, w, 5, 15, 8, "Dist. Frontal", 1),  # cm 5-15
            'pulses': Slider(px, y + gap*6, w, 15, 50, 25, "Pulsos/Celda", 1),  # Pulsos 15-50
        }
        
        # Botones para enviar/recibir parámetros
        self.btn_send_params = pygame.Rect(px, y + gap*7, w, 28)  # Botón ENVIAR PARAMS
        self.btn_get_params = pygame.Rect(px, y + gap*7 + 32, w, 28)  # Botón LEER PARAMS
    
    def _sync_sliders_to_params(self):
        """Actualiza los valores de los sliders desde los parámetros almacenados"""
        p = self.maze.params
        self.sliders['speed'].value = p.speed_normal
        self.sliders['turn_speed'].value = p.speed_turn
        self.sliders['race_speed'].value = p.speed_race
        self.sliders['turn_angle'].value = p.turn_angle
        self.sliders['kp_wall'].value = p.kp_wall
        self.sliders['front_stop'].value = p.front_stop
        self.sliders['pulses'].value = p.pulses_per_cell
    
    def _sync_params_to_sliders(self):
        """Actualiza los parámetros almacenados desde los valores de los sliders"""
        p = self.maze.params
        p.speed_normal = self.sliders['speed'].value
        p.speed_turn = self.sliders['turn_speed'].value
        p.speed_race = self.sliders['race_speed'].value
        p.turn_angle = self.sliders['turn_angle'].value
        p.kp_wall = self.sliders['kp_wall'].value
        p.front_stop = self.sliders['front_stop'].value
        p.pulses_per_cell = self.sliders['pulses'].value
    
    def draw(self):
        """Dibuja toda la interfaz gráfica"""
        self.screen.fill(WHITE)  # Limpia la pantalla
        
        if not self.maze.start_set:
            # Modo de selección de posición inicial
            self._draw_start_selection()
        else:
            # Modo normal: dibuja el laberinto y el panel
            self._draw_grid()  # Grid del laberinto
            self._draw_flood_values()  # Valores de flood fill
            self._draw_best_path()  # Mejor ruta calculada
            self._draw_path()  # Trayectoria recorrida
            self._draw_visited()  # Celdas visitadas
            self._draw_goal_cells()  # Celdas meta
            self._draw_walls()  # Paredes detectadas
            self._draw_mouse()  # Robot
            self._draw_start_marker()  # Marcador de inicio
            self._draw_panel()  # Panel de control
        
        pygame.display.flip()  # Actualiza la pantalla
    
    def _draw_start_selection(self):
        self._draw_grid()
        pos = pygame.mouse.get_pos()
        if pos[0] < self.maze_px_w and pos[1] < self.maze_px_h:
            gx = pos[0] // CELL_SIZE_PX
            gy = pos[1] // CELL_SIZE_PX
            rect = pygame.Rect(gx * CELL_SIZE_PX + 2, gy * CELL_SIZE_PX + 2, CELL_SIZE_PX - 4, CELL_SIZE_PX - 4)
            pygame.draw.rect(self.screen, YELLOW, rect)
            cx = gx * CELL_SIZE_PX + CELL_SIZE_PX // 2
            cy = gy * CELL_SIZE_PX + CELL_SIZE_PX // 2
            arrow_len = CELL_SIZE_PX // 3
            dx = DIR_DX[self.maze.mouse_dir] * arrow_len
            dy = DIR_DY[self.maze.mouse_dir] * arrow_len
            pygame.draw.line(self.screen, BLUE, (cx, cy), (cx + dx, cy + dy), 3)
            pygame.draw.circle(self.screen, BLUE, (cx + dx, cy + dy), 4)
        pygame.draw.rect(self.screen, LIGHT_GRAY, (self.maze_px_w, 0, self.panel_width, self.height))
        px = self.panel_x
        y = 20
        self.screen.blit(self.font_large.render("MICROMOUSE V3.0", True, BLACK), (px, y))
        y += 35
        for text in ["1. Click para seleccionar inicio", "2. Flechas para cambiar direccion",
                     f"   Dir: {DIR_NAMES[self.maze.mouse_dir]}", "", "3. Presiona ESPACIO o GO"]:
            self.screen.blit(self.font.render(text, True, BLACK), (px, y))
            y += 20
        y += 15
        color = GREEN if self.maze.connected else RED
        text = "Conectado" if self.maze.connected else "Sin conexion"
        self.screen.blit(self.font.render(text, True, color), (px, y))
        y += 35
        for msg in self.maze.messages[-10:]:
            self.screen.blit(self.font_small.render(msg[:45], True, DARK_GRAY), (px, y))
            y += 15
    
    def _draw_grid(self):
        for x in range(MAZE_WIDTH + 1):
            pygame.draw.line(self.screen, LIGHT_GRAY, (x * CELL_SIZE_PX, 0), (x * CELL_SIZE_PX, self.maze_px_h), 1)
        for y in range(MAZE_HEIGHT + 1):
            pygame.draw.line(self.screen, LIGHT_GRAY, (0, y * CELL_SIZE_PX), (self.maze_px_w, y * CELL_SIZE_PX), 1)
    
    def _draw_path(self):
        if len(self.maze.path) >= 2:
            pts = [(gx * CELL_SIZE_PX + CELL_SIZE_PX // 2, gy * CELL_SIZE_PX + CELL_SIZE_PX // 2) for (gx, gy) in self.maze.path]
            pygame.draw.lines(self.screen, LIGHT_BLUE, False, pts, 3)
    
    def _draw_visited(self):
        for (gx, gy) in self.maze.visited:
            if (gx, gy) not in self.maze.goal_cells:
                rect = pygame.Rect(gx * CELL_SIZE_PX + 2, gy * CELL_SIZE_PX + 2, CELL_SIZE_PX - 4, CELL_SIZE_PX - 4)
                pygame.draw.rect(self.screen, LIGHT_GREEN, rect)
    
    def _draw_goal_cells(self):
        for (gx, gy) in self.maze.goal_cells:
            sx, sy = gx * CELL_SIZE_PX, gy * CELL_SIZE_PX
            rect = pygame.Rect(sx + 2, sy + 2, CELL_SIZE_PX - 4, CELL_SIZE_PX - 4)
            pygame.draw.rect(self.screen, YELLOW, rect)
            star = self.font_large.render("*", True, ORANGE)
            self.screen.blit(star, star.get_rect(center=(sx + CELL_SIZE_PX//2, sy + CELL_SIZE_PX//2)))
    
    def _draw_flood_values(self):
        for (gx, gy), val in self.maze.flood_values.items():
            if val < 999:
                text = self.font_small.render(str(val), True, DARK_GRAY)
                self.screen.blit(text, (gx * CELL_SIZE_PX + 3, gy * CELL_SIZE_PX + 3))
    
    def _draw_best_path(self):
        if len(self.maze.best_path) >= 2:
            pts = [(gx * CELL_SIZE_PX + CELL_SIZE_PX // 2, gy * CELL_SIZE_PX + CELL_SIZE_PX // 2) for (gx, gy) in self.maze.best_path]
            pygame.draw.lines(self.screen, PURPLE, False, pts, 4)
    
    def _draw_walls(self):
        for (gx, gy), walls in self.maze.walls.items():
            sx, sy = gx * CELL_SIZE_PX, gy * CELL_SIZE_PX
            if walls[0] == True:
                pygame.draw.line(self.screen, BLACK, (sx, sy), (sx + CELL_SIZE_PX, sy), WALL_THICKNESS)
            elif walls[0] == False:
                pygame.draw.line(self.screen, GREEN, (sx + 5, sy), (sx + CELL_SIZE_PX - 5, sy), 2)
            if walls[1] == True:
                pygame.draw.line(self.screen, BLACK, (sx + CELL_SIZE_PX, sy), (sx + CELL_SIZE_PX, sy + CELL_SIZE_PX), WALL_THICKNESS)
            elif walls[1] == False:
                pygame.draw.line(self.screen, GREEN, (sx + CELL_SIZE_PX, sy + 5), (sx + CELL_SIZE_PX, sy + CELL_SIZE_PX - 5), 2)
            if walls[2] == True:
                pygame.draw.line(self.screen, BLACK, (sx, sy + CELL_SIZE_PX), (sx + CELL_SIZE_PX, sy + CELL_SIZE_PX), WALL_THICKNESS)
            elif walls[2] == False:
                pygame.draw.line(self.screen, GREEN, (sx + 5, sy + CELL_SIZE_PX), (sx + CELL_SIZE_PX - 5, sy + CELL_SIZE_PX), 2)
            if walls[3] == True:
                pygame.draw.line(self.screen, BLACK, (sx, sy), (sx, sy + CELL_SIZE_PX), WALL_THICKNESS)
            elif walls[3] == False:
                pygame.draw.line(self.screen, GREEN, (sx, sy + 5), (sx, sy + CELL_SIZE_PX - 5), 2)
    
    def _draw_mouse(self):
        sx, sy = self.maze.mouse_x * CELL_SIZE_PX, self.maze.mouse_y * CELL_SIZE_PX
        cx, cy = sx + CELL_SIZE_PX // 2, sy + CELL_SIZE_PX // 2
        color = PURPLE if self.maze.robot_state == "RACING" else (CYAN if self.maze.robot_state == "RECAL" else RED)
        pygame.draw.circle(self.screen, color, (cx, cy), CELL_SIZE_PX // 3)
        arrow_len = CELL_SIZE_PX // 3
        dx = DIR_DX[self.maze.mouse_dir] * arrow_len
        dy = DIR_DY[self.maze.mouse_dir] * arrow_len
        pygame.draw.line(self.screen, WHITE, (cx, cy), (cx + dx, cy + dy), 3)
        pygame.draw.circle(self.screen, WHITE, (cx + dx, cy + dy), 4)
    
    def _draw_start_marker(self):
        sx, sy = self.maze.start_x * CELL_SIZE_PX, self.maze.start_y * CELL_SIZE_PX
        rect = pygame.Rect(sx + 1, sy + 1, CELL_SIZE_PX - 2, CELL_SIZE_PX - 2)
        pygame.draw.rect(self.screen, YELLOW, rect, 3)
        self.screen.blit(self.font_small.render("START", True, DARK_GRAY), (sx + 5, sy + CELL_SIZE_PX - 14))
    
    def _draw_button(self, rect, text, color, text_color=WHITE, enabled=True):
        c = color if enabled else GRAY
        tc = text_color if enabled else DARK_GRAY
        pygame.draw.rect(self.screen, c, rect)
        pygame.draw.rect(self.screen, DARK_GRAY, rect, 2)
        txt = self.font_btn.render(text, True, tc)
        self.screen.blit(txt, txt.get_rect(center=rect.center))
    
    def _draw_panel(self):
        pygame.draw.rect(self.screen, LIGHT_GRAY, (self.maze_px_w, 0, self.panel_width, self.height))
        px = self.panel_x
        y = 8
        self.screen.blit(self.font_large.render("MICROMOUSE V3.0", True, BLACK), (px, y))
        y += 25
        state_colors = {"WAITING": ORANGE, "MAPPING": GREEN, "STOPPED": RED, "RECAL": CYAN, "RACE_READY": PURPLE, "RACING": PURPLE}
        state_names = {"WAITING": "Esperando", "MAPPING": "Mapeando", "STOPPED": "Detenido", "RECAL": "Recalibrando", "RACE_READY": "Listo", "RACING": "Corriendo"}
        conn_color = GREEN if self.maze.connected else RED
        conn_text = "OK" if self.maze.connected else "--"
        self.screen.blit(self.font.render(conn_text, True, conn_color), (px, y))
        st_color = state_colors.get(self.maze.robot_state, GRAY)
        st_name = state_names.get(self.maze.robot_state, self.maze.robot_state)
        self.screen.blit(self.font.render(st_name, True, st_color), (px + 40, y))
        y += 22
        self.screen.blit(self.font.render(f"Pos: ({self.maze.mouse_x},{self.maze.mouse_y}) {DIR_NAMES[self.maze.mouse_dir]}", True, BLACK), (px, y))
        y += 18
        self.screen.blit(self.font.render(f"Yaw: {self.maze.yaw:.1f} Celdas: {len(self.maze.visited)}", True, BLACK), (px, y))
        y += 22
        state = self.maze.robot_state
        self._draw_button(self.btn_go, "INICIAR", DARK_GREEN, enabled=state in ["WAITING", "STOPPED"] and self.maze.connected)
        self._draw_button(self.btn_stop, "DETENER", DARK_RED, enabled=state in ["MAPPING", "RACING"] and self.maze.connected)
        self._draw_button(self.btn_recal, "RECALIBRAR", CYAN, BLACK, enabled=state == "STOPPED" and self.maze.connected)
        self._draw_button(self.btn_recal_done, "LISTO", DARK_GREEN, enabled=state == "RECAL" and self.maze.connected)
        self._draw_button(self.btn_solve, "RESOLVER", BLUE, enabled=len(self.maze.goal_cells) > 0)
        self._draw_button(self.btn_race, "CARRERA", PURPLE, enabled=len(self.maze.best_path) > 0 and state == "RACE_READY" and self.maze.connected)
        calib_color = ORANGE if self.show_calibration else DARK_GRAY
        self._draw_button(self.btn_calib, "CALIBRAR", calib_color)
        self._draw_button(self.btn_dummy, "PRUEBA", BLUE)
        self._draw_button(self.btn_reset, "RESET", DARK_GRAY)
        self._draw_button(self.btn_full_reset, "FULL", DARK_GRAY)
        y = 295
        pygame.draw.line(self.screen, GRAY, (px, y), (px + 355, y), 1)
        y += 8
        self.screen.blit(self.font.render("SENSORES (cm)", True, DARK_GRAY), (px, y))
        y += 18
        for text, val, thresh in [("Frente", self.maze.dist_front, 10), ("Izq", self.maze.dist_left, 10), ("Der", self.maze.dist_right, 10)]:
            color = RED if val <= thresh else GREEN
            self.screen.blit(self.font.render(f"{text}: {val:3d}", True, color), (px, y))
            y += 16
        y += 5
        self.screen.blit(self.font.render(f"Metas: {len(self.maze.goal_cells)} Ruta: {len(self.maze.best_path)}", True, PURPLE), (px, y))
        y += 22
        if self.show_calibration:
            pygame.draw.line(self.screen, ORANGE, (px, y), (px + 355, y), 2)
            y += 8
            self.screen.blit(self.font.render("CALIBRACION", True, ORANGE), (px, y))
            for slider in self.sliders.values():
                slider.draw(self.screen, self.font_small)
            self._draw_button(self.btn_send_params, "ENVIAR PARAMS", DARK_GREEN)
            self._draw_button(self.btn_get_params, "LEER PARAMS", BLUE)
            y = self.btn_get_params.bottom + 12
        else:
            y += 8
        pygame.draw.line(self.screen, GRAY, (px, y), (px + 355, y), 1)
        y += 8
        self.screen.blit(self.font.render("LOG", True, DARK_GRAY), (px, y))
        y += 16
        max_msgs = 6 if not self.show_calibration else 3
        for msg in self.maze.messages[-max_msgs:]:
            self.screen.blit(self.font_small.render(msg[:48], True, DARK_GRAY), (px, y))
            y += 14
        y = self.height - 35
        self.screen.blit(self.font_small.render("Click=Meta | SPACE=GO | S=Stop | C=Calibrar", True, DARK_GRAY), (px, y))
    
    def handle_click(self, pos):
        if self.btn_go.collidepoint(pos):
            if self.maze.robot_state in ["WAITING", "STOPPED"] and self.bt_reader:
                self.bt_reader.send_go()
            return
        if self.btn_stop.collidepoint(pos):
            if self.bt_reader:
                self.bt_reader.send_stop()
            return
        if self.btn_recal.collidepoint(pos):
            if self.maze.robot_state == "STOPPED" and self.bt_reader:
                self.bt_reader.send_recal()
            return
        if self.btn_recal_done.collidepoint(pos):
            if self.maze.robot_state == "RECAL" and self.bt_reader:
                self.bt_reader.send_recal_done()
                self.maze.mouse_x = self.maze.start_x
                self.maze.mouse_y = self.maze.start_y
                self.maze.mouse_dir = 0
                self.maze.path = [(self.maze.start_x, self.maze.start_y)]
            return
        if self.btn_solve.collidepoint(pos):
            if len(self.maze.goal_cells) > 0:
                self.maze.calculate_flood_fill()
            return
        if self.btn_race.collidepoint(pos):
            if len(self.maze.best_path) > 0 and self.bt_reader:
                self.bt_reader.send_race()
            return
        if self.btn_calib.collidepoint(pos):
            self.show_calibration = not self.show_calibration
            if self.show_calibration:
                self._sync_sliders_to_params()
            return
        if self.btn_dummy.collidepoint(pos):
            self._load_dummy()
            return
        if self.btn_reset.collidepoint(pos):
            self.maze.reset_mapping()
            return
        if self.btn_full_reset.collidepoint(pos):
            self.maze.full_reset()
            return
        if self.show_calibration:
            if self.btn_send_params.collidepoint(pos):
                self._sync_params_to_sliders()
                if self.bt_reader:
                    self.bt_reader.send_params()
                return
            if self.btn_get_params.collidepoint(pos):
                if self.bt_reader:
                    self.bt_reader.request_params()
                return
        if not self.maze.start_set:
            if pos[0] < self.maze_px_w and pos[1] < self.maze_px_h:
                gx = pos[0] // CELL_SIZE_PX
                gy = pos[1] // CELL_SIZE_PX
                self.maze.set_start_position(gx, gy)
        else:
            if pos[0] < self.maze_px_w and pos[1] < self.maze_px_h:
                gx = pos[0] // CELL_SIZE_PX
                gy = pos[1] // CELL_SIZE_PX
                self.maze.toggle_goal(gx, gy)
    
    def _load_dummy(self):
        self.maze.full_reset()
        self.maze.load_dummy_maze()
        self.maze.set_start_position(0, MAZE_HEIGHT - 1)
        cx, cy = MAZE_WIDTH // 2, MAZE_HEIGHT // 2
        for dx in range(2):
            for dy in range(2):
                self.maze.goal_cells.add((cx - dx, cy - dy))
        self.maze.calculate_flood_fill()
        self.maze.robot_state = "STOPPED"
        self.maze.add_message("Modo prueba")
    
    def run(self):
        """Bucle principal de la aplicación"""
        running = True
        while running:
            # Procesa todos los eventos de Pygame
            for event in pygame.event.get():
                # Cerrar ventana
                if event.type == pygame.QUIT:
                    running = False
                
                # Redimensionar ventana
                elif event.type == pygame.VIDEORESIZE:
                    self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                
                # Clicks del mouse
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Click izquierdo
                        self.handle_click(event.pos)
                    elif event.button == 3:  # Click derecho: toggle meta
                        if event.pos[0] < self.maze_px_w and event.pos[1] < self.maze_px_h:
                            gx = event.pos[0] // CELL_SIZE_PX
                            gy = event.pos[1] // CELL_SIZE_PX
                            self.maze.toggle_goal(gx, gy)
                # Teclas presionadas
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:  # ESC: Salir
                        running = False
                    
                    elif event.key == pygame.K_SPACE:  # ESPACIO: Iniciar mapeo
                        if self.maze.start_set and self.maze.robot_state in ["WAITING", "STOPPED"]:
                            if self.bt_reader:
                                self.bt_reader.send_go()
                    
                    elif event.key == pygame.K_s:  # S: Detener
                        if self.bt_reader:
                            self.bt_reader.send_stop()
                    
                    elif event.key == pygame.K_r:  # R: Reset mapeo
                        self.maze.reset_mapping()
                    
                    elif event.key == pygame.K_f:  # F: Full reset
                        self.maze.full_reset()
                    
                    elif event.key == pygame.K_c:  # C: Toggle calibración
                        self.show_calibration = not self.show_calibration
                        if self.show_calibration:
                            self._sync_sliders_to_params()
                    
                    elif event.key == pygame.K_g:  # G: Calcular flood fill
                        if len(self.maze.goal_cells) > 0:
                            self.maze.calculate_flood_fill()
                    
                    elif event.key == pygame.K_d:  # D: Cargar laberinto de prueba
                        self._load_dummy()
                    
                    # Flechas: Cambiar dirección del robot
                    elif event.key == pygame.K_UP:
                        self.maze.mouse_dir = 0  # Norte
                    elif event.key == pygame.K_RIGHT:
                        self.maze.mouse_dir = 1  # Este
                    elif event.key == pygame.K_DOWN:
                        self.maze.mouse_dir = 2  # Sur
                    elif event.key == pygame.K_LEFT:
                        self.maze.mouse_dir = 3  # Oeste
                # Maneja eventos de sliders si el panel de calibración está visible
                if self.show_calibration:
                    for slider in self.sliders.values():
                        slider.handle_event(event)
            
            self.draw()  # Dibuja la interfaz
            self.clock.tick(30)  # Limita a 30 FPS
        
        pygame.quit()  # Cierra Pygame al salir


def main():
    """Función principal que inicializa y ejecuta la aplicación"""
    # Banner de inicio
    print("=" * 55)
    print("MICROMOUSE-ROBOTICA 2025B")
    print("MAZE MAPPER V3.0")
    print("Con panel de calibracion y modo carrera")
    print("=" * 55)
    
    # Obtiene el puerto serial desde argumentos de línea de comandos
    port = None
    if len(sys.argv) >= 2:
        port = sys.argv[1]  # Primer argumento es el puerto
    else:
        # Muestra instrucciones si no se proporciona puerto
        print("\nUso: python maze_mapper_v3.py <PUERTO>")
        print("  Ej Win: python maze_mapper_v3.py COM5")
        print("  Ej Mac: python maze_mapper_v3.py /dev/cu.HC-05")
        print("\nIniciando sin Bluetooth...")
        print("Usa 'PRUEBA' para modo demo")
    
    # Crea el objeto principal del laberinto
    maze = MazeMapper()
    bt_reader = None
    
    # Intenta conectar por Bluetooth si se proporcionó un puerto
    if port:
        bt_reader = BluetoothReader(port, maze)
        if bt_reader.connect():
            bt_reader.start()  # Inicia el hilo de lectura
            print(f"Conectado a {port}")
        else:
            print(f"No se pudo conectar a {port}")
    
    # Ejecuta la interfaz gráfica
    try:
        viz = MazeVisualizer(maze, bt_reader)
        viz.run()  # Bucle principal
    finally:
        # Limpieza al salir
        if bt_reader:
            bt_reader.stop()
    
    print("\nHasta luego!")


# Punto de entrada del programa
if __name__ == "__main__":
    main()  # Ejecuta la función principal
