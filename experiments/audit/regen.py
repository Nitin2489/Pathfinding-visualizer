"""Independent Python reimplementation of the framework's grid generator and
shortest-path references (written for the audit from the C++ semantics).
Shares no code with the C++ project."""
import heapq
from collections import deque

M64 = (1 << 64) - 1

class MT64:  # std::mt19937_64
    def __init__(self, seed):
        mt = [0] * 312
        mt[0] = seed & M64
        for i in range(1, 312):
            mt[i] = (6364136223846793005 * (mt[i - 1] ^ (mt[i - 1] >> 62)) + i) & M64
        self.mt, self.i = mt, 312
    def _twist(self):
        mt = self.mt
        for i in range(312):
            x = (mt[i] & 0xFFFFFFFF80000000) | (mt[(i + 1) % 312] & 0x7FFFFFFF)
            xa = x >> 1
            if x & 1:
                xa ^= 0xB5026F5AA96619E9
            mt[i] = mt[(i + 156) % 312] ^ xa
        self.i = 0
    def next(self):
        if self.i >= 312:
            self._twist()
        y = self.mt[self.i]; self.i += 1
        y ^= (y >> 29) & 0x5555555555555555
        y ^= (y << 17) & 0x71D67FFFEDA60000
        y ^= (y << 37) & 0xFFF7EEE000000000
        y ^= y >> 43
        return y & M64

class Rng:
    def __init__(self, seed): self.e = MT64(seed)
    def uniform_int(self, n):
        thr = ((1 << 64) - n) % n
        while True:
            r = self.e.next()
            if r >= thr: return r % n
    def real(self): return (self.e.next() >> 11) * (1.0 / 9007199254740992.0)
    def chance(self, p): return self.real() < p

def splitmix64(x):
    x = (x + 0x9E3779B97F4A7C15) & M64
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & M64
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & M64
    return x ^ (x >> 31)

def stream_seed(seed, s):
    return seed if s == 0 else splitmix64((seed + s * 0xD1B54A32D192ED03) & M64)

WALL, EMPTY = ord('#'), ord(' ')
COST = {ord(' '): 1, ord('~'): 3, ord('W'): 5, ord('M'): 10}

def neighbours(i, rows, cols):
    r, c = divmod(i, cols)
    if r > 0: yield i - cols
    if c + 1 < cols: yield i + 1
    if r + 1 < rows: yield i + cols
    if c > 0: yield i - 1

def generate(rows, cols, density, terrain, placement, seed):
    g = bytearray([EMPTY]) * (rows * cols)
    for r in range(rows):
        g[r * cols] = g[r * cols + cols - 1] = WALL
    for c in range(cols):
        g[c] = g[(rows - 1) * cols + c] = WALL
    start, end = 1 * cols + 1, (rows - 2) * cols + (cols - 2)
    rng = Rng(stream_seed(seed, 0))
    for r in range(1, rows - 1):
        for c in range(1, cols - 1):
            w = rng.chance(density)
            i = r * cols + c
            if w and i != start and i != end: g[i] = WALL
    attempts = 0
    if placement != "corners":
        cells = [r * cols + c for r in range(1, rows - 1) for c in range(1, cols - 1) if g[r * cols + c] != WALL]
        er = Rng(stream_seed(seed, 1)); n = len(cells)
        def draw():
            i = er.uniform_int(n); j = er.uniform_int(n - 1)
            if j >= i: j += 1
            return cells[i], cells[j]
        if placement == "random":
            start, end = draw() if n > 1 else (cells[0], cells[0]); attempts = 1
        else:
            lab = components(g, rows, cols)
            for attempts in range(1, 10001):
                a, b = draw()
                if lab[a] == lab[b]: start, end = a, b; break
            else:
                return None
        g[start] = g[end] = EMPTY
    if terrain > 0:
        tr = Rng(stream_seed(seed, 2)); kinds = [ord('W'), ord('~'), ord('M')]
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                i = r * cols + c
                if g[i] == WALL: continue
                if not tr.chance(terrain): continue
                t = kinds[tr.uniform_int(3)]
                if i != start and i != end: g[i] = t
    return g, start, end, attempts

def components(g, rows, cols):
    lab = [-1] * len(g); k = 0
    for s in range(len(g)):
        if g[s] == WALL or lab[s] != -1: continue
        lab[s] = k; q = deque([s])
        while q:
            u = q.popleft()
            for v in neighbours(u, rows, cols):
                if g[v] != WALL and lab[v] == -1: lab[v] = k; q.append(v)
        k += 1
    return lab

def fnv(g, rows, cols, start, end):
    h = 14695981039346656037
    for v in (rows, cols, start, end):
        for b in range(8):
            h ^= (v >> (8 * b)) & 0xFF; h = (h * 1099511628211) & M64
    for ch in g:
        h ^= ch; h = (h * 1099511628211) & M64
    return h

def min_moves(g, rows, cols, s, t):
    d = {s: 0}; q = deque([s])
    while q:
        u = q.popleft()
        if u == t: return d[u]
        for v in neighbours(u, rows, cols):
            if g[v] != WALL and v not in d: d[v] = d[u] + 1; q.append(v)
    return None

def min_cost(g, rows, cols, s, t):
    dist = {s: 0}; pq = [(0, s)]
    while pq:
        du, u = heapq.heappop(pq)
        if u == t: return du
        if du > dist[u]: continue
        for v in neighbours(u, rows, cols):
            if g[v] == WALL: continue
            nd = du + COST[g[v]]
            if nd < dist.get(v, 1 << 62): dist[v] = nd; heapq.heappush(pq, (nd, v))
    return None
