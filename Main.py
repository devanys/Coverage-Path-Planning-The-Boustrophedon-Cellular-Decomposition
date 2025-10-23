import tkinter as tk
from tkinter import filedialog
import numpy as np
import cv2
from PIL import Image, ImageTk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from scipy import ndimage
import heapq

def load_binary_image(path, invert=False):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError('Failed to load image: ' + path)
    _, bw = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    if invert:
        bw = 255 - bw
    return bw

def occupancy_grid_from_bw(bw):
    return (bw == 0).astype(np.uint8)

def astar(grid, start, goal):
    h = lambda a,b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    rows, cols = grid.shape
    start = tuple(start); goal = tuple(goal)
    open_set = [(h(start,goal), 0, start, None)]
    came_from = {}
    gscore = {start:0}
    closed = set()
    while open_set:
        f, g, node, parent = heapq.heappop(open_set)
        if node in closed:
            continue
        came_from[node] = parent
        if node == goal:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = came_from.get(cur, None)
            path.reverse()
            return path
        closed.add(node)
        x,y = node
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0<=nx<rows and 0<=ny<cols and grid[nx,ny]==0:
                ng = g+1
                neigh = (nx,ny)
                if neigh in gscore and ng >= gscore[neigh]:
                    continue
                gscore[neigh] = ng
                heapq.heappush(open_set, (ng + h(neigh,goal), ng, neigh, node))
    return None

def vertical_boustrophedon_cells(occ):
    rows, cols = occ.shape
    free_intervals = []
    for c in range(cols):
        col = occ[:, c]
        is_free = (col == 0).astype(int)
        labeled, ncomp = ndimage.label(is_free)
        intervals = []
        for k in range(1, ncomp+1):
            inds = np.where(labeled == k)[0]
            if inds.size:
                intervals.append((int(inds[0]), int(inds[-1])))
        free_intervals.append(intervals)
    crit = [0]
    prev = free_intervals[0]
    for c in range(1, cols):
        cur = free_intervals[c]
        if len(cur) != len(prev):
            crit.append(c)
        else:
            for a, b in zip(prev, cur):
                if a[0] != b[0] or a[1] != b[1]:
                    crit.append(c)
                    break
        prev = cur
    crit.append(cols)
    merged = [crit[0]]
    for v in crit[1:]:
        if v - merged[-1] > 1:
            merged.append(v)
    crit = merged
    cells = []
    for i in range(len(crit)-1):
        x0, x1 = crit[i], crit[i+1]-1
        if x1 < x0: continue
        slab = occ[:, x0:x1+1]
        free = (slab == 0).astype(np.uint8)
        labeled, n = ndimage.label(free)
        for lab in range(1, n+1):
            ys, xs = np.where(labeled == lab)
            if ys.size == 0: continue
            y0, y1 = int(ys.min()), int(ys.max())
            cells.append({'x0': x0, 'x1': x1, 'y0': y0, 'y1': y1, 'centroid': (int((y0+y1)/2), int((x0+x1)/2))})
    return cells

def generate_boustrophedon_paths_for_cell(cell, occ, spacing=12):
    x0, x1, y0, y1 = cell['x0'], cell['x1'], cell['y0'], cell['y1']
    paths = []
    direction = 1
    for y in range(y0 + spacing//2, y1+1, spacing):
        row = occ[y, x0:x1+1]
        free_runs = []
        start = None
        for i, v in enumerate(row):
            if v == 0 and start is None:
                start = i
            if v == 1 and start is not None:
                free_runs.append((start, i-1))
                start = None
        if start is not None:
            free_runs.append((start, len(row)-1))
        if not free_runs:
            continue
        run = max(free_runs, key=lambda r: r[1]-r[0])
        sx = x0 + run[0]
        ex = x0 + run[1]
        if direction == 1:
            paths.append([(y, sx), (y, ex)])
        else:
            paths.append([(y, ex), (y, sx)])
        direction *= -1
    waypoints = []
    for seg in paths:
        for p in seg:
            waypoints.append((int(p[0]), int(p[1])))
    return waypoints

def plan_coverage(occ, start, goal, spacing=12):
    cells = vertical_boustrophedon_cells(occ)
    cell_paths = []
    for c in cells:
        wps = generate_boustrophedon_paths_for_cell(c, occ, spacing=spacing)
        if wps:
            cell_paths.append({'cell': c, 'wps': wps})
    order = []
    rem = cell_paths.copy()
    cur = start
    while rem:
        best_i = None
        best_d = None
        for i, cp in enumerate(rem):
            cy, cx = cp['cell']['centroid']
            d = abs(cur[0]-cy) + abs(cur[1]-cx)
            if best_d is None or d < best_d:
                best_d = d
                best_i = i
        next_cp = rem.pop(best_i)
        order.append(next_cp)
        cur = next_cp['cell']['centroid']
    full_path = []
    cur_pt = start
    grid = occ
    for cp in order:
        wps = cp['wps']
        wp_tups = [(int(p[0]), int(p[1])) for p in wps]
        entry = min(wp_tups, key=lambda p: abs(p[0]-cur_pt[0]) + abs(p[1]-cur_pt[1]))
        path_to_entry = astar(grid, cur_pt, entry)
        if path_to_entry is not None:
            full_path.extend(path_to_entry)
        full_path.extend(wp_tups)
        cur_pt = wp_tups[-1]
    p_to_goal = astar(grid, cur_pt, goal)
    if p_to_goal:
        full_path.extend(p_to_goal)
    return cells, cell_paths, full_path

class BCDApp:
    def __init__(self, root):
        self.root = root
        root.title('CPP: Boustrophedon Cellular Decomposition')
        root.geometry('1100x700')
        root.update_idletasks()
        w = root.winfo_screenwidth(); h = root.winfo_screenheight()
        gw, gh = 1100, 700
        root.geometry(f'{gw}x{gh}+{w//2-gw//2}+{h//2-gh//2}')
        main = tk.Frame(root)
        main.pack(fill=tk.BOTH, expand=True)
        self.frame = tk.Frame(main)
        self.frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ctrl = tk.Frame(main, width=260)
        ctrl.pack(side=tk.RIGHT, fill=tk.Y)
        self.load_btn = tk.Button(ctrl, text='Load PNG', command=self.load_png)
        self.load_btn.pack(pady=6, fill=tk.X, padx=6)
        tk.Label(ctrl, text='Robot spacing (px)').pack(padx=6)
        self.spacing_var = tk.IntVar(value=12)
        tk.Entry(ctrl, textvariable=self.spacing_var).pack(fill=tk.X, padx=6)
        self.info_label = tk.Label(ctrl, text='Click once for start (green), once for goal (red)')
        self.info_label.pack(pady=8, padx=6)
        self.canvas_fig = None
        self.bw = None
        self.occ = None
        self.start = None
        self.goal = None
        fig, ax = plt.subplots(figsize=(7,7))
        ax.axis('off')
        self.fig = fig; self.ax = ax
        self.canvas_fig = FigureCanvasTkAgg(fig, master=self.frame)
        self.canvas_fig.draw()
        self.canvas_fig.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas_fig.mpl_connect('button_press_event', self.on_click)

    def load_png(self):
        path = filedialog.askopenfilename(filetypes=[('PNG','*.png'), ('All','*.*')])
        if not path: return
        bw = load_binary_image(path, invert=False)
        self.bw = bw
        self.occ = occupancy_grid_from_bw(bw)
        self.start = None; self.goal = None
        self.info_label.config(text=f'Image loaded: {bw.shape[1]}x{bw.shape[0]}. Click start.')
        self.show_image()

    def on_click(self, event):
        if self.bw is None: return
        if event.xdata is None or event.ydata is None: 
            return
        col = int(round(event.xdata))
        row = int(round(event.ydata))
        h, w = self.bw.shape
        if not (0 <= row < h and 0 <= col < w):
            return
        if self.start is None:
            self.start = (row, col)
            self.info_label.config(text='Start set. Click goal.')
            self.show_image()
        elif self.goal is None:
            self.goal = (row, col)
            self.info_label.config(text='Goal set. Running planner...') 
            self.show_image()
            self.root.after(50, self.auto_run)
        else:
            self.start = (row, col)
            self.goal = None
            self.info_label.config(text='Start reset. Click goal.')
            self.show_image()

    def show_image(self):
        self.ax.clear()
        if self.bw is not None:
            self.ax.imshow(self.bw, cmap='gray')
        if self.start:
            self.ax.plot(self.start[1], self.start[0], 'go')
        if self.goal:
            self.ax.plot(self.goal[1], self.goal[0], 'ro')
        self.ax.axis('off')
        self.canvas_fig.draw()

    def auto_run(self):
        if self.occ is None or self.start is None or self.goal is None:
            return
        spacing = max(4, int(self.spacing_var.get()))
        cells, cell_paths, full_path = plan_coverage(self.occ, self.start, self.goal, spacing=spacing)
        self.ax.clear()
        self.ax.imshow(self.bw, cmap='gray')
        for c in cells:
            rect = plt.Rectangle((c['x0'], c['y0']), c['x1']-c['x0']+1, c['y1']-c['y0']+1, edgecolor='cyan', facecolor='none', linewidth=1)
            self.ax.add_patch(rect)
        for cp in cell_paths:
            xs = [p[1] for p in cp['wps']]
            ys = [p[0] for p in cp['wps']]
            if len(xs)>0:
                self.ax.plot(xs, ys, linewidth=1)
        if full_path:
            xs = [p[1] for p in full_path]
            ys = [p[0] for p in full_path]
            self.ax.plot(xs, ys, '-r', linewidth=1)
        self.ax.plot(self.start[1], self.start[0], 'go')
        self.ax.plot(self.goal[1], self.goal[0], 'ro')
        self.ax.set_title('Coverage Path Result')
        self.ax.axis('off')
        self.canvas_fig.draw()
        self.info_label.config(text='Coverage complete!')

if __name__ == '__main__':
    root = tk.Tk()
    app = BCDApp(root)
    root.mainloop()
