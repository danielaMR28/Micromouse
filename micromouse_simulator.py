"""
Micromouse V3.1 - Flood Fill en Tiempo Real
Meta fija 2x2 en centro | Sensores siempre activos
Uso: python maze_mapper_v3_1.py COM5
"""
import pygame, serial, sys, threading, time
from collections import defaultdict

CELL_SIZE_PX, MAZE_WIDTH, MAZE_HEIGHT, WALL_THICKNESS = 60, 12, 7, 4
GOAL_CENTER_X, GOAL_CENTER_Y = MAZE_WIDTH // 2, MAZE_HEIGHT // 2

WHITE, BLACK, GRAY, LIGHT_GRAY, DARK_GRAY = (255,255,255), (0,0,0), (200,200,200), (240,240,240), (100,100,100)
RED, DARK_RED, GREEN, DARK_GREEN, BLUE = (255,80,80), (180,50,50), (80,200,80), (50,150,50), (80,80,255)
YELLOW, LIGHT_GREEN, LIGHT_BLUE, ORANGE, PURPLE, CYAN, GOLD = (255,220,80), (220,255,220), (200,220,255), (255,165,0), (180,100,255), (100,200,200), (255,215,0)

DIR_NAMES, DIR_ARROWS = ['N','E','S','W'], ['↑','→','↓','←']
DIR_DX, DIR_DY = [0,1,0,-1], [-1,0,1,0]

class CalibrationParams:
    def __init__(self):
        self.speed_normal, self.speed_turn, self.speed_race = 80, 85, 100
        self.turn_angle, self.front_stop, self.pulses_per_cell = 38, 8, 25
        self.kp_wall, self.correction_factor = 18, 30
    def to_command(self):
        return f"PARAMS,{self.speed_normal},{self.speed_turn},{self.turn_angle},{self.kp_wall},{self.front_stop},{self.pulses_per_cell},{self.correction_factor},{self.speed_race}"
    def from_arduino(self, v):
        if len(v)>=6: self.speed_normal,self.speed_turn,self.turn_angle,self.kp_wall,self.front_stop,self.pulses_per_cell = int(v[0]),int(v[1]),int(v[2]),int(v[3]),int(v[4]),int(v[5])
        if len(v)>=7: self.correction_factor = int(v[6])
        if len(v)>=8: self.speed_race = int(v[7])

class MazeMapper:
    def __init__(self):
        self.walls = defaultdict(lambda: {0:None,1:None,2:None,3:None})
        self.visited, self.path = set(), []
        self.start_x, self.start_y, self.start_set = 0, 0, False
        self.mouse_x, self.mouse_y, self.mouse_dir = 0, 0, 0
        self.offset_x, self.offset_y = 0, 0
        self.dist_front, self.dist_left, self.dist_right, self.yaw = 0, 0, 0, 0.0
        self.connected, self.messages, self.robot_state = False, [], "WAITING"
        self.goal_cells, self.flood_values, self.best_path = set(), {}, []
        self.params = CalibrationParams()
        self.walls_changed = False
        self._set_center_goal()
    
    def _set_center_goal(self):
        self.goal_cells = {(GOAL_CENTER_X-1,GOAL_CENTER_Y-1),(GOAL_CENTER_X,GOAL_CENTER_Y-1),(GOAL_CENTER_X-1,GOAL_CENTER_Y),(GOAL_CENTER_X,GOAL_CENTER_Y)}
    
    def set_start_position(self, x, y):
        self.start_x, self.start_y, self.start_set = x, y, True
        self.mouse_x, self.mouse_y = x, y
        self.offset_x, self.offset_y = x, y
        self.path, self.visited = [(x,y)], {(x,y)}
        self.add_message(f"Inicio: ({x},{y})")
        self.calculate_flood_fill()
    
    def update_position_from_arduino(self, ax, ay, d):
        self.mouse_x, self.mouse_y, self.mouse_dir = self.offset_x+ax, self.offset_y+ay, d
        self.visited.add((self.mouse_x, self.mouse_y))
        if not self.path or self.path[-1] != (self.mouse_x, self.mouse_y): self.path.append((self.mouse_x, self.mouse_y))
        if (self.mouse_x, self.mouse_y) in self.goal_cells: self.add_message("*** META! ***")
    
    def update_walls_from_sensors(self, wf, wl, wr, d):
        x, y = self.mouse_x, self.mouse_y
        self.visited.add((x,y))
        df, dl, dr = d, (d+3)%4, (d+1)%4
        old = dict(self.walls[(x,y)])
        self.walls[(x,y)][df], self.walls[(x,y)][dl], self.walls[(x,y)][dr] = wf, wl, wr
        for di, w in [(df,wf),(dl,wl),(dr,wr)]:
            nx, ny = x+DIR_DX[di], y+DIR_DY[di]
            if 0<=nx<MAZE_WIDTH and 0<=ny<MAZE_HEIGHT: self.walls[(nx,ny)][(di+2)%4] = w
        if old != dict(self.walls[(x,y)]): self.walls_changed = True
    
    def update_flood_fill_if_needed(self):
        if self.walls_changed and self.start_set: self.calculate_flood_fill(); self.walls_changed = False
    
    def add_message(self, m):
        self.messages.append(f"[{time.strftime('%H:%M:%S')}] {m}")
        if len(self.messages) > 15: self.messages.pop(0)
    
    def reset_mapping(self):
        self.walls.clear(); self.visited.clear()
        self.path = [(self.start_x,self.start_y)] if self.start_set else []
        self.mouse_x, self.mouse_y, self.mouse_dir = self.start_x, self.start_y, 0
        if self.start_set: self.visited.add((self.start_x,self.start_y))
        self.best_path.clear(); self.flood_values.clear(); self._set_center_goal()
        if self.start_set: self.calculate_flood_fill()
        self.add_message("Reset")
    
    def full_reset(self):
        self.walls.clear(); self.visited.clear(); self.path = []
        self.start_x, self.start_y, self.start_set = 0, 0, False
        self.mouse_x, self.mouse_y, self.offset_x, self.offset_y, self.mouse_dir = 0, 0, 0, 0, 0
        self.robot_state = "WAITING"; self.flood_values.clear(); self.best_path.clear(); self._set_center_goal()
        self.add_message("Full reset")
    
    def calculate_flood_fill(self):
        if not self.goal_cells: return False
        self.flood_values = {(x,y):999 for x in range(MAZE_WIDTH) for y in range(MAZE_HEIGHT)}
        queue = list(self.goal_cells)
        for c in self.goal_cells: self.flood_values[c] = 0
        while queue:
            x, y = queue.pop(0)
            v = self.flood_values[(x,y)]
            for d in range(4):
                nx, ny = x+DIR_DX[d], y+DIR_DY[d]
                if not (0<=nx<MAZE_WIDTH and 0<=ny<MAZE_HEIGHT): continue
                if self.walls.get((x,y),{}).get(d)==True: continue
                if v+1 < self.flood_values[(nx,ny)]: self.flood_values[(nx,ny)] = v+1; queue.append((nx,ny))
        self.calculate_best_path()
        return True
    
    def calculate_best_path(self):
        if not self.goal_cells: self.best_path = []; return
        start = (self.mouse_x, self.mouse_y) if self.start_set else (self.start_x, self.start_y)
        self.best_path = []; current, visited = start, {start}
        while current not in self.goal_cells and len(self.best_path) < 200:
            self.best_path.append(current)
            x, y = current
            v = self.flood_values.get(current, 999)
            best, best_val = None, v
            for d in range(4):
                nx, ny = x+DIR_DX[d], y+DIR_DY[d]
                if not (0<=nx<MAZE_WIDTH and 0<=ny<MAZE_HEIGHT): continue
                if self.walls.get((x,y),{}).get(d)==True: continue
                if (nx,ny) in visited: continue
                nv = self.flood_values.get((nx,ny),999)
                if nv < best_val: best_val, best = nv, (nx,ny)
            if best is None: break
            current = best; visited.add(current)
        if current in self.goal_cells: self.best_path.append(current)
    
    def get_path_command(self):
        if not self.best_path: return None
        pa = [(x-self.offset_x, y-self.offset_y) for (x,y) in self.best_path]
        return f"PATH,{len(pa)}" + "".join(f",{x},{y}" for x,y in pa)
    
    def load_dummy_maze(self):
        for x in range(MAZE_WIDTH): self.walls[(x,0)][0]=True; self.walls[(x,MAZE_HEIGHT-1)][2]=True
        for y in range(MAZE_HEIGHT): self.walls[(0,y)][3]=True; self.walls[(MAZE_WIDTH-1,y)][1]=True
        for x,y,d in [(1,1,1),(2,1,2),(3,1,3),(4,1,1),(1,2,0),(3,2,1),(5,2,2),(2,3,3),(4,3,0),(6,3,1),(1,4,1),(3,4,2),(5,4,3),(2,5,0),(4,5,1),(6,5,2),(7,1,2),(8,2,1),(9,3,0),(7,4,1),(8,5,2),(10,2,3)]:
            if 0<=x<MAZE_WIDTH and 0<=y<MAZE_HEIGHT:
                self.walls[(x,y)][d]=True
                nx,ny = x+DIR_DX[d], y+DIR_DY[d]
                if 0<=nx<MAZE_WIDTH and 0<=ny<MAZE_HEIGHT: self.walls[(nx,ny)][(d+2)%4]=True
        self.add_message("Laberinto prueba")

class BluetoothReader:
    def __init__(self, port, maze):
        self.port, self.maze, self.serial, self.running = port, maze, None, False
    
    def connect(self):
        try: self.serial = serial.Serial(self.port, 9600, timeout=1); time.sleep(2); self.maze.connected = True; self.maze.add_message(f"OK: {self.port}"); return True
        except Exception as e: self.maze.add_message(f"Err: {str(e)[:20]}"); return False
    
    def send_command(self, cmd):
        if self.serial and self.serial.is_open:
            try: self.serial.write(f"{cmd}\n".encode()); self.serial.flush(); self.maze.add_message(f"> {cmd[:25]}"); return True
            except: pass
        return False
    
    def send_go(self): 
        if self.send_command("GO"): self.maze.robot_state = "MAPPING"; return True
        return False
    def send_stop(self):
        if self.send_command("STOP"): self.maze.robot_state = "STOPPED"; return True
        return False
    def send_recal(self):
        if self.send_command("RECAL"): self.maze.robot_state = "RECAL"; return True
        return False
    def send_recal_done(self):
        if self.send_command("RECAL_DONE"): self.maze.robot_state = "RACE_READY"; return True
        return False
    def send_race(self):
        self.maze.mouse_x, self.maze.mouse_y = self.maze.start_x, self.maze.start_y
        self.maze.calculate_flood_fill()
        pc = self.maze.get_path_command()
        if pc: self.send_command(pc); time.sleep(0.1)
        if self.send_command("RACE"): self.maze.robot_state = "RACING"; return True
        return False
    def send_params(self): return self.send_command(self.maze.params.to_command())
    def request_params(self): return self.send_command("GET_PARAMS")
    
    def start(self): self.running = True; threading.Thread(target=self._read_loop, daemon=True).start()
    def stop(self): self.running = False; self.serial and self.serial.is_open and self.serial.close()
    
    def _read_loop(self):
        while self.running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line: self._parse(line)
            except: pass
            time.sleep(0.01)
    
    def _parse(self, line):
        try:
            if line == "READY": self.maze.add_message("Arduino OK"); self.request_params()
            elif line == "WAITING": self.maze.robot_state = "WAITING"
            elif line == "STARTING": self.maze.robot_state = "MAPPING"; self.maze.add_message("Mapeando...")
            elif line == "STOPPED": self.maze.robot_state = "STOPPED"; self.maze.add_message("Detenido")
            elif line == "RECAL_MODE": self.maze.robot_state = "RECAL"
            elif line == "RACE_READY": self.maze.robot_state = "RACE_READY"
            elif line == "RACING": self.maze.robot_state = "RACING"
            elif line == "RACE_COMPLETE": self.maze.robot_state = "STOPPED"; self.maze.add_message("*** META! ***")
            elif line == "GYRO_CAL_DONE": self.maze.add_message("Gyro OK")
            elif line == "PARAMS_SET": self.maze.add_message("Params OK")
            else:
                p = line.split(',')
                if p[0]=="CURRENT_PARAMS" and len(p)>=7: self.maze.params.from_arduino(p[1:])
                elif p[0]=="NEW_CELL" and len(p)>=4: self.maze.update_position_from_arduino(int(p[1]),int(p[2]),int(p[3])); self.maze.update_flood_fill_if_needed()
                elif p[0]=="DIR_CHANGE" and len(p)>=2: self.maze.mouse_dir = int(p[1])
                elif p[0] in ["SENSORS","DATA"] and len(p)>=11:
                    self.maze.dist_front,self.maze.dist_left,self.maze.dist_right = int(p[1]),int(p[2]),int(p[3])
                    self.maze.mouse_dir, self.maze.yaw = int(p[7]), float(p[8])
                    if self.maze.robot_state == "MAPPING": self.maze.update_walls_from_sensors(p[4]=="1",p[5]=="1",p[6]=="1",int(p[7])); self.maze.update_flood_fill_if_needed()
                elif p[0]=="RACE_CELL" and len(p)>=3: self.maze.mouse_x,self.maze.mouse_y = self.maze.offset_x+int(p[1]), self.maze.offset_y+int(p[2])
        except: pass

class Slider:
    def __init__(self, x, y, w, min_v, max_v, val, label, step=1):
        self.rect, self.min_v, self.max_v, self.value, self.label, self.step, self.dragging = pygame.Rect(x,y,w,18), min_v, max_v, val, label, step, False
    def draw(self, screen, font):
        screen.blit(font.render(f"{self.label}: {self.value}", True, BLACK), (self.rect.x, self.rect.y-14))
        pygame.draw.rect(screen, GRAY, self.rect); pygame.draw.rect(screen, DARK_GRAY, self.rect, 1)
        kx = int(self.rect.x + (self.value-self.min_v)/(self.max_v-self.min_v)*self.rect.width)
        pygame.draw.rect(screen, BLUE, (self.rect.x, self.rect.y, kx-self.rect.x, self.rect.height))
        pygame.draw.circle(screen, WHITE, (kx, self.rect.centery), 7); pygame.draw.circle(screen, DARK_GRAY, (kx, self.rect.centery), 7, 2)
    def handle_event(self, e):
        if e.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(e.pos): self.dragging = True; self._update(e.pos[0])
        elif e.type == pygame.MOUSEBUTTONUP: self.dragging = False
        elif e.type == pygame.MOUSEMOTION and self.dragging: self._update(e.pos[0])
    def _update(self, mx):
        rx = max(0, min(mx-self.rect.x, self.rect.width))
        self.value = round((self.min_v + rx/self.rect.width*(self.max_v-self.min_v))/self.step)*self.step

class MazeVisualizer:
    def __init__(self, maze, bt=None):
        pygame.init()
        self.maze, self.bt = maze, bt
        self.maze_px_w, self.maze_px_h, self.panel_w = MAZE_WIDTH*CELL_SIZE_PX, MAZE_HEIGHT*CELL_SIZE_PX, 380
        self.w, self.h = self.maze_px_w+self.panel_w, max(self.maze_px_h, 720)
        self.screen = pygame.display.set_mode((self.w, self.h), pygame.RESIZABLE)
        pygame.display.set_caption("Micromouse V3.1 - Flood Fill Tiempo Real")
        self.font, self.font_lg, self.font_sm, self.font_btn = pygame.font.Font(None,20), pygame.font.Font(None,26), pygame.font.Font(None,16), pygame.font.Font(None,20)
        self.clock = pygame.time.Clock()
        self.px, self.bw, self.bh = self.maze_px_w+12, 168, 32
        self._mk_btns(); self.show_calib, self.show_flood = False, True; self._mk_sliders()
    
    def _mk_btns(self):
        px, y, g = self.px, 105, 38
        self.btn_go, self.btn_stop = pygame.Rect(px,y,self.bw,self.bh), pygame.Rect(px+self.bw+8,y,self.bw,self.bh); y+=g
        self.btn_recal, self.btn_recal_done = pygame.Rect(px,y,self.bw,self.bh), pygame.Rect(px+self.bw+8,y,self.bw,self.bh); y+=g
        self.btn_race = pygame.Rect(px,y,self.bw*2+8,self.bh); y+=g
        self.btn_calib, self.btn_dummy = pygame.Rect(px,y,self.bw,self.bh), pygame.Rect(px+self.bw+8,y,self.bw,self.bh); y+=g
        self.btn_reset, self.btn_full = pygame.Rect(px,y,self.bw,self.bh), pygame.Rect(px+self.bw+8,y,self.bw,self.bh)
    
    def _mk_sliders(self):
        px, y, w, g = self.px, 430, 345, 42
        self.sliders = {'speed':Slider(px,y,w,40,150,80,"Vel Normal",5),'turn':Slider(px,y+g,w,40,150,85,"Vel Giro",5),'race':Slider(px,y+g*2,w,60,200,100,"Vel Carrera",5),'angle':Slider(px,y+g*3,w,30,50,38,"Angulo",1),'kp':Slider(px,y+g*4,w,5,50,18,"KP x10",1),'front':Slider(px,y+g*5,w,5,15,8,"Dist Front",1),'pulses':Slider(px,y+g*6,w,15,50,25,"Pulsos",1)}
        self.btn_send, self.btn_get = pygame.Rect(px,y+g*7,w,28), pygame.Rect(px,y+g*7+32,w,28)
    
    def _sync_to(self):
        p = self.maze.params
        self.sliders['speed'].value, self.sliders['turn'].value, self.sliders['race'].value = p.speed_normal, p.speed_turn, p.speed_race
        self.sliders['angle'].value, self.sliders['kp'].value, self.sliders['front'].value, self.sliders['pulses'].value = p.turn_angle, p.kp_wall, p.front_stop, p.pulses_per_cell
    
    def _sync_from(self):
        p = self.maze.params
        p.speed_normal, p.speed_turn, p.speed_race = self.sliders['speed'].value, self.sliders['turn'].value, self.sliders['race'].value
        p.turn_angle, p.kp_wall, p.front_stop, p.pulses_per_cell = self.sliders['angle'].value, self.sliders['kp'].value, self.sliders['front'].value, self.sliders['pulses'].value
    
    def draw(self):
        self.screen.fill(WHITE)
        if not self.maze.start_set: self._draw_init()
        else: self._draw_grid(); self._draw_flood(); self._draw_best(); self._draw_path(); self._draw_visited(); self._draw_goals(); self._draw_walls(); self._draw_mouse(); self._draw_start(); self._draw_panel()
        pygame.display.flip()
    
    def _draw_init(self):
        self._draw_grid(); self._draw_goals()
        pos = pygame.mouse.get_pos()
        if pos[0]<self.maze_px_w and pos[1]<self.maze_px_h:
            gx, gy = pos[0]//CELL_SIZE_PX, pos[1]//CELL_SIZE_PX
            pygame.draw.rect(self.screen, LIGHT_BLUE, (gx*CELL_SIZE_PX+2,gy*CELL_SIZE_PX+2,CELL_SIZE_PX-4,CELL_SIZE_PX-4))
            cx, cy = gx*CELL_SIZE_PX+CELL_SIZE_PX//2, gy*CELL_SIZE_PX+CELL_SIZE_PX//2
            dx, dy = DIR_DX[self.maze.mouse_dir]*20, DIR_DY[self.maze.mouse_dir]*20
            pygame.draw.line(self.screen, BLUE, (cx,cy), (cx+dx,cy+dy), 3)
        pygame.draw.rect(self.screen, LIGHT_GRAY, (self.maze_px_w,0,self.panel_w,self.h))
        px, y = self.px, 20
        self.screen.blit(self.font_lg.render("MICROMOUSE V3.1", True, BLACK), (px,y)); y+=30
        self.screen.blit(self.font.render("Flood Fill Tiempo Real", True, PURPLE), (px,y)); y+=30
        for t in ["1. Click = INICIO","2. Flechas = Direccion",f"   Dir: {DIR_ARROWS[self.maze.mouse_dir]}","","3. ESPACIO = Iniciar","","META: Centro 2x2 (dorado)"]:
            self.screen.blit(self.font.render(t, True, BLACK), (px,y)); y+=20
        y+=15
        self.screen.blit(self.font.render("BT OK" if self.maze.connected else "Sin BT", True, GREEN if self.maze.connected else RED), (px,y)); y+=25
        self.screen.blit(self.font.render("SENSORES:", True, DARK_GRAY), (px,y)); y+=18
        for n,v in [("F",self.maze.dist_front),("L",self.maze.dist_left),("R",self.maze.dist_right)]:
            self.screen.blit(self.font.render(f"  {n}: {v}cm", True, RED if v<=10 else GREEN), (px,y)); y+=16
        y+=15
        for m in self.maze.messages[-8:]: self.screen.blit(self.font_sm.render(m[:45], True, DARK_GRAY), (px,y)); y+=14
    
    def _draw_grid(self):
        for x in range(MAZE_WIDTH+1): pygame.draw.line(self.screen, LIGHT_GRAY, (x*CELL_SIZE_PX,0), (x*CELL_SIZE_PX,self.maze_px_h), 1)
        for y in range(MAZE_HEIGHT+1): pygame.draw.line(self.screen, LIGHT_GRAY, (0,y*CELL_SIZE_PX), (self.maze_px_w,y*CELL_SIZE_PX), 1)
    
    def _draw_flood(self):
        if not self.show_flood: return
        for (gx,gy),v in self.maze.flood_values.items():
            if v<999:
                i = min(255, v*15)
                pygame.draw.rect(self.screen, (255,255-i//2,255-i//2), (gx*CELL_SIZE_PX+1,gy*CELL_SIZE_PX+1,CELL_SIZE_PX-2,CELL_SIZE_PX-2))
                self.screen.blit(self.font_sm.render(str(v), True, DARK_GRAY), (gx*CELL_SIZE_PX+3,gy*CELL_SIZE_PX+3))
    
    def _draw_path(self):
        if len(self.maze.path)>=2: pygame.draw.lines(self.screen, LIGHT_BLUE, False, [(gx*CELL_SIZE_PX+CELL_SIZE_PX//2,gy*CELL_SIZE_PX+CELL_SIZE_PX//2) for gx,gy in self.maze.path], 3)
    
    def _draw_best(self):
        if len(self.maze.best_path)>=2: pygame.draw.lines(self.screen, PURPLE, False, [(gx*CELL_SIZE_PX+CELL_SIZE_PX//2,gy*CELL_SIZE_PX+CELL_SIZE_PX//2) for gx,gy in self.maze.best_path], 4)
    
    def _draw_visited(self):
        for gx,gy in self.maze.visited:
            if (gx,gy) not in self.maze.goal_cells: pygame.draw.rect(self.screen, GREEN, (gx*CELL_SIZE_PX+2,gy*CELL_SIZE_PX+2,CELL_SIZE_PX-4,CELL_SIZE_PX-4), 2)
    
    def _draw_goals(self):
        for gx,gy in self.maze.goal_cells:
            sx,sy = gx*CELL_SIZE_PX, gy*CELL_SIZE_PX
            pygame.draw.rect(self.screen, GOLD, (sx+2,sy+2,CELL_SIZE_PX-4,CELL_SIZE_PX-4))
            pygame.draw.rect(self.screen, ORANGE, (sx+2,sy+2,CELL_SIZE_PX-4,CELL_SIZE_PX-4), 3)
    
    def _draw_walls(self):
        for (gx,gy),ws in self.maze.walls.items():
            sx, sy = gx*CELL_SIZE_PX, gy*CELL_SIZE_PX
            if ws[0]: pygame.draw.line(self.screen, BLACK, (sx,sy), (sx+CELL_SIZE_PX,sy), WALL_THICKNESS)
            if ws[1]: pygame.draw.line(self.screen, BLACK, (sx+CELL_SIZE_PX,sy), (sx+CELL_SIZE_PX,sy+CELL_SIZE_PX), WALL_THICKNESS)
            if ws[2]: pygame.draw.line(self.screen, BLACK, (sx,sy+CELL_SIZE_PX), (sx+CELL_SIZE_PX,sy+CELL_SIZE_PX), WALL_THICKNESS)
            if ws[3]: pygame.draw.line(self.screen, BLACK, (sx,sy), (sx,sy+CELL_SIZE_PX), WALL_THICKNESS)
    
    def _draw_mouse(self):
        sx,sy = self.maze.mouse_x*CELL_SIZE_PX, self.maze.mouse_y*CELL_SIZE_PX
        cx, cy = sx+CELL_SIZE_PX//2, sy+CELL_SIZE_PX//2
        c = PURPLE if self.maze.robot_state=="RACING" else (CYAN if self.maze.robot_state=="RECAL" else RED)
        pygame.draw.circle(self.screen, c, (cx,cy), CELL_SIZE_PX//3)
        dx, dy = DIR_DX[self.maze.mouse_dir]*20, DIR_DY[self.maze.mouse_dir]*20
        pygame.draw.line(self.screen, WHITE, (cx,cy), (cx+dx,cy+dy), 3)
    
    def _draw_start(self):
        sx,sy = self.maze.start_x*CELL_SIZE_PX, self.maze.start_y*CELL_SIZE_PX
        pygame.draw.rect(self.screen, CYAN, (sx+1,sy+1,CELL_SIZE_PX-2,CELL_SIZE_PX-2), 3)
    
    def _btn(self, r, t, c, tc=WHITE, en=True):
        pygame.draw.rect(self.screen, c if en else GRAY, r); pygame.draw.rect(self.screen, DARK_GRAY, r, 2)
        self.screen.blit(self.font_btn.render(t, True, tc if en else DARK_GRAY), self.font_btn.render(t,True,tc).get_rect(center=r.center))
    
    def _draw_panel(self):
        pygame.draw.rect(self.screen, LIGHT_GRAY, (self.maze_px_w,0,self.panel_w,self.h))
        px, y = self.px, 8
        self.screen.blit(self.font_lg.render("MICROMOUSE V3.1", True, BLACK), (px,y)); y+=22
        self.screen.blit(self.font_sm.render("Flood Fill Tiempo Real", True, PURPLE), (px,y)); y+=20
        sc = {"WAITING":ORANGE,"MAPPING":GREEN,"STOPPED":RED,"RECAL":CYAN,"RACE_READY":PURPLE,"RACING":PURPLE}
        sn = {"WAITING":"Esperando","MAPPING":"MAPEANDO","STOPPED":"Detenido","RECAL":"Recalib","RACE_READY":"Listo","RACING":"CORRIENDO"}
        self.screen.blit(self.font.render("BT OK" if self.maze.connected else "--", True, GREEN if self.maze.connected else RED), (px,y))
        self.screen.blit(self.font.render(sn.get(self.maze.robot_state,"?"), True, sc.get(self.maze.robot_state,GRAY)), (px+55,y)); y+=22
        self.screen.blit(self.font.render(f"({self.maze.mouse_x},{self.maze.mouse_y}) {DIR_ARROWS[self.maze.mouse_dir]} Yaw:{self.maze.yaw:.0f}", True, BLACK), (px,y)); y+=22
        st = self.maze.robot_state
        self._btn(self.btn_go, "INICIAR", DARK_GREEN, en=st in ["WAITING","STOPPED"] and self.maze.connected)
        self._btn(self.btn_stop, "DETENER", DARK_RED, en=st in ["MAPPING","RACING"] and self.maze.connected)
        self._btn(self.btn_recal, "RECALIB", CYAN, BLACK, en=st=="STOPPED" and self.maze.connected)
        self._btn(self.btn_recal_done, "LISTO", DARK_GREEN, en=st=="RECAL" and self.maze.connected)
        self._btn(self.btn_race, f"CARRERA ({len(self.maze.best_path)})", PURPLE, en=len(self.maze.best_path)>0 and st=="RACE_READY" and self.maze.connected)
        self._btn(self.btn_calib, "CALIBRAR", ORANGE if self.show_calib else DARK_GRAY)
        self._btn(self.btn_dummy, "PRUEBA", BLUE)
        self._btn(self.btn_reset, "RESET", DARK_GRAY); self._btn(self.btn_full, "FULL", DARK_GRAY)
        y = 295; pygame.draw.line(self.screen, GRAY, (px,y), (px+355,y), 1); y+=8
        self.screen.blit(self.font.render("SENSORES", True, DARK_GRAY), (px,y)); y+=18
        for n,v,th in [("F",self.maze.dist_front,10),("L",self.maze.dist_left,14),("R",self.maze.dist_right,14)]:
            bw = min(v*3,150); c = RED if v<=th else GREEN
            pygame.draw.rect(self.screen, c, (px+30,y,bw,12)); pygame.draw.rect(self.screen, DARK_GRAY, (px+30,y,150,12), 1)
            self.screen.blit(self.font.render(f"{n}:", True, BLACK), (px,y))
            self.screen.blit(self.font_sm.render(f"{v}", True, BLACK), (px+185,y)); y+=18
        y+=5; pygame.draw.line(self.screen, PURPLE, (px,y), (px+355,y), 2); y+=8
        cv = self.maze.flood_values.get((self.maze.mouse_x,self.maze.mouse_y),999)
        self.screen.blit(self.font.render(f"Dist meta: {cv} celdas", True, PURPLE), (px,y)); y+=22
        if self.show_calib:
            pygame.draw.line(self.screen, ORANGE, (px,y), (px+355,y), 2); y+=8
            for s in self.sliders.values(): s.draw(self.screen, self.font_sm)
            self._btn(self.btn_send, "ENVIAR", DARK_GREEN); self._btn(self.btn_get, "LEER", BLUE)
            y = self.btn_get.bottom+12
        pygame.draw.line(self.screen, GRAY, (px,y), (px+355,y), 1); y+=8
        self.screen.blit(self.font.render("LOG", True, DARK_GRAY), (px,y)); y+=16
        for m in self.maze.messages[-(4 if self.show_calib else 6):]: self.screen.blit(self.font_sm.render(m[:48], True, DARK_GRAY), (px,y)); y+=14
        self.screen.blit(self.font_sm.render("SPACE=GO S=Stop C=Calib V=Flood", True, DARK_GRAY), (px,self.h-35))
    
    def click(self, pos):
        if self.btn_go.collidepoint(pos) and self.maze.robot_state in ["WAITING","STOPPED"] and self.bt: self.bt.send_go(); return
        if self.btn_stop.collidepoint(pos) and self.bt: self.bt.send_stop(); return
        if self.btn_recal.collidepoint(pos) and self.maze.robot_state=="STOPPED" and self.bt: self.bt.send_recal(); return
        if self.btn_recal_done.collidepoint(pos) and self.maze.robot_state=="RECAL" and self.bt:
            self.bt.send_recal_done(); self.maze.mouse_x,self.maze.mouse_y,self.maze.mouse_dir = self.maze.start_x,self.maze.start_y,0
            self.maze.path = [(self.maze.start_x,self.maze.start_y)]; self.maze.calculate_flood_fill(); return
        if self.btn_race.collidepoint(pos) and len(self.maze.best_path)>0 and self.bt: self.bt.send_race(); return
        if self.btn_calib.collidepoint(pos): self.show_calib = not self.show_calib; self.show_calib and self._sync_to(); return
        if self.btn_dummy.collidepoint(pos): self._dummy(); return
        if self.btn_reset.collidepoint(pos): self.maze.reset_mapping(); return
        if self.btn_full.collidepoint(pos): self.maze.full_reset(); return
        if self.show_calib:
            if self.btn_send.collidepoint(pos): self._sync_from(); self.bt and self.bt.send_params(); return
            if self.btn_get.collidepoint(pos) and self.bt: self.bt.request_params(); return
        if not self.maze.start_set and pos[0]<self.maze_px_w and pos[1]<self.maze_px_h:
            self.maze.set_start_position(pos[0]//CELL_SIZE_PX, pos[1]//CELL_SIZE_PX)
    
    def _dummy(self):
        self.maze.full_reset(); self.maze.load_dummy_maze(); self.maze.set_start_position(0, MAZE_HEIGHT-1)
        self.maze.robot_state = "STOPPED"; self.maze.add_message("Modo prueba")
    
    def run(self):
        running = True
        while running:
            for e in pygame.event.get():
                if e.type == pygame.QUIT: running = False
                elif e.type == pygame.VIDEORESIZE: self.screen = pygame.display.set_mode((e.w,e.h), pygame.RESIZABLE)
                elif e.type == pygame.MOUSEBUTTONDOWN and e.button==1: self.click(e.pos)
                elif e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_ESCAPE: running = False
                    elif e.key == pygame.K_SPACE and self.maze.start_set and self.maze.robot_state in ["WAITING","STOPPED"] and self.bt: self.bt.send_go()
                    elif e.key == pygame.K_s and self.bt: self.bt.send_stop()
                    elif e.key == pygame.K_r: self.maze.reset_mapping()
                    elif e.key == pygame.K_f: self.maze.full_reset()
                    elif e.key == pygame.K_c: self.show_calib = not self.show_calib; self.show_calib and self._sync_to()
                    elif e.key == pygame.K_v: self.show_flood = not self.show_flood
                    elif e.key == pygame.K_d: self._dummy()
                    elif e.key == pygame.K_UP: self.maze.mouse_dir = 0
                    elif e.key == pygame.K_RIGHT: self.maze.mouse_dir = 1
                    elif e.key == pygame.K_DOWN: self.maze.mouse_dir = 2
                    elif e.key == pygame.K_LEFT: self.maze.mouse_dir = 3
                if self.show_calib:
                    for s in self.sliders.values(): s.handle_event(e)
            self.draw(); self.clock.tick(30)
        pygame.quit()

def main():
    print("="*50); print("MICROMOUSE V3.1 - FLOOD FILL TIEMPO REAL"); print("="*50)
    port = sys.argv[1] if len(sys.argv)>=2 else None
    if not port: print("\nUso: python maze_mapper_v3_1.py COM5\nIniciando sin BT...")
    maze = MazeMapper()
    bt = None
    if port:
        bt = BluetoothReader(port, maze)
        if bt.connect(): bt.start(); print(f"OK: {port}")
        else: print(f"Fallo: {port}")
    try: MazeVisualizer(maze, bt).run()
    finally: bt and bt.stop()
    print("Bye!")

if __name__ == "__main__": main()
