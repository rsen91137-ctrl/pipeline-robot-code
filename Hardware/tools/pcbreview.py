#!/usr/bin/env python3
# 已布线PCB的检查工具: 把嘉立创EDA专业版工程里的铜皮(焊盘/走线/过孔/泪滴/铺铜)按真实形状栅格化,
# 检查连通性、短路、最小间距、走线线宽、悬空线头、关键网络的走线情况, 并出图.
#   python3 pcbreview.py 工程.epro2 输出目录
# 对外坐标: mm, 原点在板框左上角, x向右, y向下.
import sys, json, math, os, collections
import numpy as np
from scipy import ndimage as ndi
from PIL import Image, ImageDraw, ImageFont
import pcbtool as T

MIL = 0.0254
PX = 40                      # 每毫米像素数, 分辨率0.025mm
TOP, BOT = 1, 2
POUR_UNIT = 10              # 铺铜填充(POURED)坐标的单位: 10mil

def arc_points(x0, y0, ang, x1, y1, step=6.0):
    """从(x0,y0)到(x1,y1)、圆心角ang(度, 逆时针为正, y向上坐标系)的圆弧, 化成折线点(不含起点)"""
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-9 or abs(ang) < 1e-6: return [(x1, y1)]
    a = math.radians(ang); mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    nx, ny = -(y1 - y0) / d, (x1 - x0) / d
    h = d / (2 * math.tan(a / 2)); cx, cy = mx + nx * h, my + ny * h
    r = math.hypot(x0 - cx, y0 - cy); s = math.atan2(y0 - cy, x0 - cx)
    n = max(2, int(abs(ang) / step) + 1)
    return [(cx + r * math.cos(s + a * i / n), cy + r * math.sin(s + a * i / n)) for i in range(1, n + 1)]

def path_to_polys(path):
    """EDA路径 -> 若干条折线(文件坐标)"""
    if path and isinstance(path[0], list):
        out = []
        for q in path: out += path_to_polys(q)
        return out
    if not path: return []
    if path[0] == 'R':
        x, y, w, h = path[1:5]; return [[(x, y), (x + w, y), (x + w, y - h), (x, y - h)]]
    if path[0] == 'CIRCLE':
        cx, cy, r = path[1:4]; return [[(cx + r * math.cos(t * math.pi / 18), cy + r * math.sin(t * math.pi / 18)) for t in range(36)]]
    pts = []; mode = 'L'; nums = []
    def flush():
        nonlocal nums
        if mode == 'L':
            for i in range(0, len(nums) - 1, 2): pts.append((nums[i], nums[i + 1]))
        else:
            for i in range(0, len(nums) - 2, 3):
                a, x, y = nums[i:i + 3]; pts.extend(arc_points(pts[-1][0], pts[-1][1], a, x, y))
        nums = []
    for t in path:
        if isinstance(t, str): flush(); mode = 'ARC' if t in ('ARC', 'CARC') else 'L'
        else: nums.append(t)
    flush(); return [pts]

class Routed:
    def __init__(self, path):
        self.prj = p = T.Project(path)
        self.W = p.outline[3] * MIL; self.H = p.outline[4] * MIL; self.R = (p.outline[6] if len(p.outline) > 6 else 0) * MIL
        self.N = int(round(self.W * PX)); self.M = int(round(self.H * PX))
        self.layout = p.current_layout()
        self.lines = []; self.vias = []; self.tears = []; self.pours = []; self.poured = {}
        for h, b, i in p.pcb['rows']:
            t = h['type']
            if t == 'LINE':
                self.lines.append({'id': h['id'], 'net': b['netName'], 'layer': b['layerId'], 'w': b['width'] * MIL, 'a': self.mm(b['startX'], b['startY']), 'b': self.mm(b['endX'], b['endY']), 'row': i})
            elif t == 'VIA':
                self.vias.append({'id': h['id'], 'net': b['netName'], 'p': self.mm(b['centerX'], b['centerY']), 'drill': b['holeDiameter'] * MIL, 'dia': b['viaDiameter'] * MIL, 'row': i})
            elif t == 'TEARDROP':
                self.tears.append({'net': b['netName'], 'layer': b['layerId'], 'path': b['path']})
            elif t == 'POUR':
                self.pours.append({'id': h['id'], 'net': b['netName'], 'layer': b['layerId'], 'name': b.get('name'), 'path': b['path'], 'row': i})
            elif t == 'POURED':
                self.poured[json.loads(h['id'])[1]] = (b, i)
        # 焊盘(带形状)
        self.pads = []
        fpraw = {}
        for d in p.docs:
            if d['head'].get('docType') == 'FOOTPRINT':
                fpraw[d['head']['uuid']] = [b for h, b, _ in d['rows'] if h['type'] == 'PAD']
        for ref in p.refs:
            c = p.comps[ref]; x, y, ang = self.layout[ref]
            for b in fpraw[c['fp']]:
                dp = b['defaultPad']; w = dp.get('width', 0); hh = dp.get('height', w)
                px, py = T._rot(b['centerX'], b['centerY'], ang)
                rot = (ang + (b.get('padAngle') or 0)) % 180
                if rot: w, hh = hh, w
                hole = b.get('hole') or {}
                th = b.get('layerId') == 12 or (hole.get('width') or 0) > 0
                self.pads.append({'ref': ref, 'num': b['num'], 'net': c['nets'].get(b['num'], ''), 'x': x + px * MIL, 'y': y - py * MIL, 'w': w * MIL, 'h': hh * MIL,
                                  'shape': dp.get('padType'), 'th': th, 'drill': (hole.get('width') or 0) * MIL, 'layer': b.get('layerId')})
        self.nets = sorted({q['net'] for q in self.pads if q['net']} | {l['net'] for l in self.lines if l['net']})

    def mm(self, x, y):
        return ((x - self.prj.ox) * MIL, -(y - self.prj.oy) * MIL)
    def px(self, x, y): return (x * PX, y * PX)

    # ---------- 栅格化 ----------
    def _canvas(self): return Image.new('1', (self.N, self.M), 0)
    def _draw_pad(self, d, q, grow=0.0):
        x0, y0, x1, y1 = (q['x'] - q['w'] / 2 - grow) * PX, (q['y'] - q['h'] / 2 - grow) * PX, (q['x'] + q['w'] / 2 + grow) * PX, (q['y'] + q['h'] / 2 + grow) * PX
        if q['shape'] == 'RECT': d.rectangle([x0, y0, x1, y1], fill=1)
        elif q['shape'] == 'ELLIPSE': d.ellipse([x0, y0, x1, y1], fill=1)
        else: d.rounded_rectangle([x0, y0, x1, y1], radius=min(x1 - x0, y1 - y0) / 2, fill=1)
    def _draw_line(self, d, l):
        (ax, ay), (bx, by) = l['a'], l['b']; w = l['w'] * PX
        d.line([ax * PX, ay * PX, bx * PX, by * PX], fill=1, width=max(1, int(round(w))))
        for x, y in (l['a'], l['b']): d.ellipse([x * PX - w / 2, y * PX - w / 2, x * PX + w / 2, y * PX + w / 2], fill=1)
    def _poly_px(self, pts): return [self.px(*self.mm(x, y)) for x, y in pts]

    def pour_mask(self, layer):
        """该层所有GND铺铜的实际填充(偶奇填充处理挖空)"""
        acc = np.zeros((self.M, self.N), bool)
        for po in self.pours:
            if po['layer'] != layer or po['id'] not in self.poured: continue
            for item in self.poured[po['id']][0]['pourFill']:
                # 铺铜填充数据的坐标单位是10mil(与走线的mil不同), 所以要乘POUR_UNIT
                polys = [[(x * POUR_UNIT, y * POUR_UNIT) for x, y in poly] for poly in path_to_polys(item['path'])]
                if item.get('fill'):
                    reg = np.zeros((self.M, self.N), bool)
                    for poly in polys:
                        if len(poly) < 3: continue
                        im = self._canvas(); ImageDraw.Draw(im).polygon(self._poly_px(poly), fill=1)
                        reg ^= np.array(im, bool)
                    acc |= reg
                elif item.get('strokeWidth'):
                    # 不填充的线: 焊盘与铺铜之间的十字连接(热焊盘辐条)
                    w = item['strokeWidth'] * POUR_UNIT * MIL * PX; im = self._canvas(); d = ImageDraw.Draw(im)
                    for poly in polys:
                        pts = self._poly_px(poly)
                        if len(pts) >= 2:
                            d.line(pts, fill=1, width=max(1, int(round(w))))
                            for x, y in pts: d.ellipse([x - w / 2, y - w / 2, x + w / 2, y + w / 2], fill=1)
                    acc |= np.array(im, bool)
        return acc

    def build(self):
        """masks[layer][net] = bool数组; 无网络的东西用 '~名字' 表示"""
        self.masks = {TOP: {}, BOT: {}}
        def get(layer, net):
            m = self.masks[layer]
            if net not in m: m[net] = self._canvas()
            return m[net]
        for q in self.pads:
            net = q['net'] or f"~{q['ref']}.{q['num']}"
            for layer in ((TOP, BOT) if q['th'] else (TOP,)):
                self._draw_pad(ImageDraw.Draw(get(layer, net)), q)
        for l in self.lines:
            if l['layer'] in (TOP, BOT): self._draw_line(ImageDraw.Draw(get(l['layer'], l['net'] or '~line')), l)
        for v in self.vias:
            net = v['net'] or '~hole'
            for layer in (TOP, BOT):
                x, y = v['p']; r = v['dia'] / 2
                ImageDraw.Draw(get(layer, net)).ellipse([(x - r) * PX, (y - r) * PX, (x + r) * PX, (y + r) * PX], fill=1)
        for t in self.tears:
            if t['layer'] in (TOP, BOT):
                for poly in path_to_polys(t['path']):
                    if len(poly) >= 3: ImageDraw.Draw(get(t['layer'], t['net'] or '~tear')).polygon(self._poly_px(poly), fill=1)
        for layer in (TOP, BOT):
            self.masks[layer] = {n: np.array(im, bool) for n, im in self.masks[layer].items()}
        self.pour = {layer: self.pour_mask(layer) for layer in (TOP, BOT)}
        self.gnd0 = {}
        for layer in (TOP, BOT):
            g = self.masks[layer].get('GND', np.zeros((self.M, self.N), bool))
            self.gnd0[layer] = g                      # 不含铺铜的GND铜皮(焊盘 走线 过孔)
            self.masks[layer]['GND'] = g | self.pour[layer]
        return self

    def edge_distance(self):
        """每个像素到板框(含圆角)的距离 mm, 板外为负"""
        yy, xx = np.mgrid[0:self.M, 0:self.N]; x = (xx + 0.5) / PX; y = (yy + 0.5) / PX; r = self.R
        dx = np.minimum(x, self.W - x); dy = np.minimum(y, self.H - y); d = np.minimum(dx, dy)
        cx = np.clip(x, r, self.W - r); cy = np.clip(y, r, self.H - r)
        return np.where((dx < r) & (dy < r), r - np.hypot(x - cx, y - cy), d)

    def simulate_pour(self, clear=0.25, edge=0.3):
        """按规则模拟重新铺一遍GND铜(整板 避开其他网络clear 距板边edge 去掉孤岛), 用来预估改线之后重建铺铜的结果.
        与EDA的差别: 这里焊盘直接连铺铜, EDA是十字连接, 所以结果略偏乐观, 最终以EDA的DRC为准"""
        inside = self.edge_distance() >= edge
        for layer in (TOP, BOT):
            other = np.zeros((self.M, self.N), bool)
            for n, m in self.masks[layer].items():
                if n != 'GND': other |= m
            fill = inside & (ndi.distance_transform_edt(~other) / PX >= clear)
            g0 = self.gnd0[layer]
            L, _ = ndi.label(fill | g0, structure=np.ones((3, 3)))
            ids = np.unique(L[g0]); ids = ids[ids > 0]
            fill &= np.isin(L, ids)
            self.pour[layer] = fill; self.masks[layer]['GND'] = g0 | fill
        return self

    # ---------- 检查 ----------
    def connectivity(self):
        """每个网络的焊盘是否全部连通. 返回 {net: [各连通组的焊盘列表]} 只列出分成多组的网络"""
        res = {}
        for net in self.nets:
            lab = {}; parent = {}
            def find(a):
                while parent[a] != a: parent[a] = parent[parent[a]]; a = parent[a]
                return a
            def union(a, b):
                ra, rb = find(a), find(b)
                if ra != rb: parent[ra] = rb
            for layer in (TOP, BOT):
                m = self.masks[layer].get(net)
                if m is None: lab[layer] = None; continue
                L, n = ndi.label(m, structure=np.ones((3, 3))); lab[layer] = L
                for k in range(1, n + 1): parent[(layer, k)] = (layer, k)
            def at(layer, x, y):
                L = lab[layer]
                if L is None: return None
                i, j = int(round(y * PX)), int(round(x * PX))
                if 0 <= i < self.M and 0 <= j < self.N and L[i, j]: return (layer, int(L[i, j]))
                return None
            for v in self.vias:
                if v['net'] == net:
                    a, b = at(TOP, *v['p']), at(BOT, *v['p'])
                    if a and b: union(a, b)
            groups = collections.defaultdict(list); where = []
            for q in self.pads:
                if q['net'] != net: continue
                a = at(TOP, q['x'], q['y'])
                if q['th']:
                    b = at(BOT, q['x'], q['y'])
                    if a and b: union(a, b)
                    a = a or b
                where.append((q, a))
            for q, a in where:      # 所有合并做完之后再分组
                groups[find(a) if a else ('none', q['ref'] + q['num'])].append(f"{q['ref']}.{q['num']}")
            if len(groups) > 1: res[net] = sorted(groups.values(), key=len, reverse=True)
        return res

    def clearance(self, limit=0.30):
        """同层不同网络铜皮之间的最小间距. 返回 [(间距mm, 层, 网络A, 网络B, x, y)] 只列小于limit的, 以及重叠(短路)"""
        out = []
        for layer in (TOP, BOT):
            names = list(self.masks[layer]); stack = np.zeros((self.M, self.N), np.int16)
            for n in names: stack += self.masks[layer][n]
            other_any = {}
            for i, a in enumerate(names):
                ma = self.masks[layer][a]
                if not ma.any(): continue
                dist = ndi.distance_transform_edt(~ma) / PX
                for b in names[i + 1:]:
                    if a.startswith('~') and b.startswith('~') and a.split('.')[0] == b.split('.')[0]: continue   # 同一元件的空脚之间不算
                    mb = self.masks[layer][b]
                    if not mb.any(): continue
                    dv = np.where(mb, dist, 9e9); k = int(np.argmin(dv)); d = float(dv.flat[k])
                    if d < limit:
                        y, x = divmod(k, self.N); out.append((round(d, 3), layer, a, b, round(x / PX, 2), round(y / PX, 2)))
        return sorted(out)

    def dangling(self):
        """走线端点没有接到同网络的焊盘/过孔/其他走线上"""
        out = []
        for l in self.lines:
            for end in ('a', 'b'):
                x, y = l[end]; ok = False
                for q in self.pads:
                    if q['net'] == l['net'] and (q['th'] or l['layer'] == TOP) and abs(x - q['x']) <= q['w'] / 2 + 0.05 and abs(y - q['y']) <= q['h'] / 2 + 0.05: ok = True; break
                if not ok:
                    for v in self.vias:
                        if v['net'] == l['net'] and math.hypot(x - v['p'][0], y - v['p'][1]) <= v['dia'] / 2 + 0.02: ok = True; break
                if not ok:
                    for m in self.lines:
                        if m is l or m['net'] != l['net'] or m['layer'] != l['layer']: continue
                        if seg_dist((x, y), m['a'], m['b']) <= m['w'] / 2 + 0.02: ok = True; break
                if not ok and l['net'] == 'GND':
                    i, j = int(round(y * PX)), int(round(x * PX)); ok = bool(self.pour[l['layer']][min(i, self.M - 1), min(j, self.N - 1)])
                if not ok: out.append((l['net'], l['layer'], round(x, 2), round(y, 2), l['id']))
        return out

    def via_in_pad(self):
        """过孔打在贴片焊盘上(回流焊时焊锡会被吸进孔里). 返回 [(元件.脚, 网络, x, y, 焊盘尺寸)]"""
        out = []
        for v in self.vias:
            if not v['net']: continue
            for q in self.pads:
                if q['th']: continue
                if abs(v['p'][0] - q['x']) < q['w'] / 2 + v['drill'] / 2 and abs(v['p'][1] - q['y']) < q['h'] / 2 + v['drill'] / 2:
                    out.append((f"{q['ref']}.{q['num']}", v['net'], round(v['p'][0], 2), round(v['p'][1], 2), f"{q['w']:.2f}x{q['h']:.2f}", q['net']))
        return out

    def edge_clearance(self):
        """各层非GND铜皮和GND铺铜到板框的最小距离(mm)"""
        yy, xx = np.mgrid[0:self.M, 0:self.N]; x = (xx + 0.5) / PX; y = (yy + 0.5) / PX; r = self.R
        dx = np.minimum(x, self.W - x); dy = np.minimum(y, self.H - y); d = np.minimum(dx, dy)
        cx = np.clip(x, r, self.W - r); cy = np.clip(y, r, self.H - r)
        corner = (dx < r) & (dy < r); d = np.where(corner, r - np.hypot(x - cx, y - cy), d)
        res = {}
        for layer in (TOP, BOT):
            for n, m in self.masks[layer].items():
                if n == '~hole' or not m.any(): continue
                k = int(np.argmin(np.where(m, d, 9e9))); v = float(np.where(m, d, 9e9).flat[k]); yy0, xx0 = divmod(k, self.N)
                if v < 0.5: res[(layer, n)] = (round(v, 3), round(xx0 / PX, 2), round(yy0 / PX, 2))
        return res

    def cross_layer(self, nets, aggressors):
        """nets中的走线, 在另一层正对着的位置有没有aggressors里的走线(上下层交叉/重叠). 返回 [(网络, 干扰网络, 重叠面积mm2)]"""
        out = []
        for n in nets:
            for layer, ol in ((TOP, BOT), (BOT, TOP)):
                m = self.masks[layer].get(n)
                if m is None: continue
                for a in aggressors:
                    o = self.masks[ol].get(a)
                    if o is None: continue
                    area = float((m & o).sum()) / PX / PX
                    if area > 0.005: out.append((n, layer, a, round(area, 3)))
        return out

    def ground_under(self, net, layer=TOP):
        """net在layer层的走线正下方(另一层)被GND铜皮覆盖的比例"""
        m = self.masks[layer].get(net)
        if m is None or not m.any(): return None
        ol = BOT if layer == TOP else TOP
        return float((m & self.masks[ol]['GND']).sum()) / float(m.sum())

    def min_dist(self, a, b, layer):
        ma, mb = self.masks[layer].get(a), self.masks[layer].get(b)
        if ma is None or mb is None or not ma.any() or not mb.any(): return None
        d = ndi.distance_transform_edt(~ma) / PX; dv = np.where(mb, d, 9e9); k = int(np.argmin(dv)); y, x = divmod(k, self.N)
        return (round(float(dv.flat[k]), 3), round(x / PX, 2), round(y / PX, 2))

    def net_stats(self):
        st = {}
        for n in self.nets:
            ls = [l for l in self.lines if l['net'] == n]
            st[n] = {'top': sum(seg_len(l) for l in ls if l['layer'] == TOP), 'bot': sum(seg_len(l) for l in ls if l['layer'] == BOT),
                     'widths': sorted({round(l['w'], 3) for l in ls}), 'vias': sum(1 for v in self.vias if v['net'] == n), 'segs': len(ls)}
        return st

def seg_len(l): return math.hypot(l['a'][0] - l['b'][0], l['a'][1] - l['b'][1])
def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p; dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
    t = 0 if L2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)

COL = {'GND': (190, 190, 190), 'U3-SW': (230, 0, 200), 'U3-FB': (255, 120, 0), 'ADC1': (0, 90, 230), 'ADC2': (0, 170, 255), '+5V': (220, 30, 30), 'H11': (220, 30, 30), '$1N3': (220, 30, 30),
       '+12V': (240, 120, 0), 'H21': (200, 140, 60), 'H31': (200, 140, 60), '3V3': (150, 0, 0), 'VDDA': (150, 0, 120), 'TX+': (0, 150, 60), 'TX-': (0, 200, 120), 'RX+': (0, 120, 120), 'RX-': (60, 190, 190),
       'S1_OUT': (90, 90, 255), 'S2_OUT': (130, 130, 255)}

def render(rb, layer, out, box=None, scale=1.0, other=True, labels=True):
    img = np.full((rb.M, rb.N, 3), 255, np.uint8)
    if other:
        ol = BOT if layer == TOP else TOP
        for n, m in rb.masks[ol].items():
            if n != 'GND': img[m] = (225, 235, 225)
    img[rb.masks[layer]['GND']] = COL['GND']
    for n, m in rb.masks[layer].items():
        if n == 'GND': continue
        img[m] = COL.get(n, (40, 40, 40) if not n.startswith('~') else (120, 120, 120))
    im = Image.fromarray(img); d = ImageDraw.Draw(im)
    for v in rb.vias:
        x, y = v['p']; r = v['drill'] / 2
        d.ellipse([(x - r) * PX, (y - r) * PX, (x + r) * PX, (y + r) * PX], fill=(255, 255, 255), outline=(0, 0, 0))
    for q in rb.pads:
        if q['th']:
            r = q['drill'] / 2; d.ellipse([(q['x'] - r) * PX, (q['y'] - r) * PX, (q['x'] + r) * PX, (q['y'] + r) * PX], fill=(255, 255, 255), outline=(0, 0, 0))
    try: f = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 22)
    except Exception: f = ImageFont.load_default()
    if labels:
        for ref in rb.prj.refs:
            A = rb.prj.box(ref, rb.layout[ref])
            d.rectangle([A[0] * PX, A[1] * PX, A[2] * PX, A[3] * PX], outline=(200, 160, 0))
            d.text(((A[0] + A[2]) / 2 * PX - 6 * len(ref), (A[1] + A[3]) / 2 * PX - 11), ref, fill=(0, 0, 0), font=f, stroke_width=3, stroke_fill=(255, 255, 255))
    if box:
        im = im.crop([int(box[0] * PX), int(box[1] * PX), int(box[2] * PX), int(box[3] * PX)])
    if scale != 1.0: im = im.resize((int(im.size[0] * scale), int(im.size[1] * scale)), Image.LANCZOS)
    im.save(out); return im.size

def main():
    src, outdir = sys.argv[1], sys.argv[2]; os.makedirs(outdir, exist_ok=True)
    rb = Routed(src).build(); R = []
    if len(sys.argv) > 3 and sys.argv[3] == 'sim':      # 铺铜填充被清空的文件: 先模拟重建铺铜再检查
        rb.simulate_pour(); R.append("注意: 本报告的GND铺铜是按规则模拟重建的, 不是文件里的实际填充")
    R.append(f"板框 {rb.W:.1f} x {rb.H:.1f} mm  圆角 {rb.R:.1f} mm   铜层: 顶层 + 底层 (双层板)")
    R.append(f"走线 {len(rb.lines)} 段 (顶层 {sum(l['layer']==TOP for l in rb.lines)}, 底层 {sum(l['layer']==BOT for l in rb.lines)})  过孔 {len(rb.vias)} 个  泪滴 {len(rb.tears)} 个  铺铜 {len(rb.pours)} 个: " + ', '.join(f"{p['name']}(层{p['layer']},{p['net']})" for p in rb.pours))
    for layer, nm in ((TOP, '顶层'), (BOT, '底层')):
        g = rb.pour[layer]; L, n = ndi.label(rb.masks[layer]['GND'], structure=np.ones((3, 3)))
        sizes = sorted((ndi.sum(rb.masks[layer]['GND'], L, range(1, n + 1)) / PX / PX).tolist(), reverse=True)
        R.append(f"{nm}GND铜皮覆盖 {g.sum() / PX / PX:.0f} mm2 (板面积 {rb.W * rb.H:.0f}), GND铜皮在本层被分成 {n} 块, 最大几块面积(mm2): {[round(s, 1) for s in sizes[:8]]}")
    con = rb.connectivity()
    R.append(f"\n== 连通性: 未完全连通的网络 {len(con)} 个 ==")
    for n, gs in con.items(): R.append(f"  {n}: 分成 {len(gs)} 组 -> " + ' | '.join(','.join(g) for g in gs)[:600])
    cl = rb.clearance(0.30)
    shorts = [c for c in cl if c[0] <= 0.0]
    R.append(f"\n== 不同网络铜皮重叠(短路) {len(shorts)} 处 ==")
    for c in shorts: R.append(f"  层{c[1]} {c[2]} 与 {c[3]} 在 ({c[4]},{c[5]})")
    R.append(f"== 同层不同网络最小间距 小于0.30mm的 {len([c for c in cl if c[0] > 0])} 对 (分辨率0.025mm) ==")
    for c in [c for c in cl if c[0] > 0][:60]: R.append(f"  {c[0]:.3f} mm  层{c[1]}  {c[2]} - {c[3]}  位置({c[4]},{c[5]})")
    dg = rb.dangling()
    R.append(f"\n== 悬空的走线端点 {len(dg)} 处 ==")
    for g in dg: R.append(f"  {g[0]} 层{g[1]} ({g[2]},{g[3]})")
    R.append("\n== 各网络走线统计: 顶层长度mm / 底层长度mm / 线宽mm / 过孔数 ==")
    for n, s in sorted(rb.net_stats().items()):
        R.append(f"  {n:9s} 顶 {s['top']:6.1f}  底 {s['bot']:6.1f}  线宽 {s['widths']}  过孔 {s['vias']}  段数 {s['segs']}")
    vip = rb.via_in_pad()
    R.append(f"\n== 打在贴片焊盘上的过孔(盘中孔) {len(vip)} 个: 元件.脚 / 过孔网络 / 位置 / 焊盘尺寸 ==")
    for v in vip: R.append(f"  {v[0]:8s} {v[1]:8s} ({v[2]},{v[3]})  焊盘 {v[4]} mm" + ('' if v[1] == v[5] else f'  !!过孔网络与焊盘网络({v[5]})不同'))
    ec = rb.edge_clearance()
    R.append("\n== 铜皮到板边距离小于0.5mm的 ==")
    for (layer, n), v in sorted(ec.items(), key=lambda t: t[1][0]): R.append(f"  {v[0]:.3f} mm  层{layer} {n}  位置({v[1]},{v[2]})")
    ep = [q for q in rb.pads if q['ref'] == 'U1' and q['num'] == '33'][0]
    nep = [v for v in rb.vias if abs(v['p'][0] - ep['x']) < ep['w'] / 2 and abs(v['p'][1] - ep['y']) < ep['h'] / 2]
    R.append(f"\n== U1中心散热/接地焊盘({ep['w']:.2f}x{ep['h']:.2f}mm)内的过孔数: {len(nep)} ==")
    sens = ['ADC1', 'ADC2', 'VDDA', 'U3-FB', 'S1_OUT', 'S2_OUT']; aggr = ['U3-SW', '+12V', '+5V', '3V3', 'NRST', 'DBG_TX', 'DBG_RX', 'SWCLK', 'SWDIO', 'UART_TX', 'UART_RX', 'TX+', 'TX-', 'RX+', 'RX-', 'H21', 'H31']
    R.append("\n== 敏感网络与其他网络的同层最近距离(mm) ==")
    for n in sens:
        best = []
        for a in aggr + [x for x in sens if x != n]:
            for layer in (TOP, BOT):
                r = rb.min_dist(n, a, layer)
                if r and r[0] < 1.0: best.append((r[0], a, layer, r[1], r[2]))
        R.append(f"  {n}: " + '; '.join(f"{a} {d}mm 层{l} @({x},{y})" for d, a, l, x, y in sorted(best)[:8]))
    R.append("\n== 敏感网络走线在另一层正对位置与其他走线重叠(上下层交叉) ==")
    for n, layer, a, area in rb.cross_layer(sens, aggr): R.append(f"  {n}(层{layer}) 与 {a}(另一层) 重叠 {area} mm2")
    R.append("\n== 走线正下方(另一层)有完整地铜皮的比例 ==")
    for n in ['ADC1', 'ADC2', 'S1_OUT', 'S2_OUT', 'VDDA', 'U3-SW', 'U3-FB', '+12V', '+5V', 'UART_TX', 'UART_RX', 'TX+', 'TX-', 'RX+', 'RX-', 'NRST', 'SWCLK', 'SWDIO', 'DBG_TX', 'DBG_RX']:
        g = rb.ground_under(n, TOP)
        if g is not None: R.append(f"  {n:8s} 顶层走线下方有地: {g * 100:.0f}%")
    open(os.path.join(outdir, 'report.txt'), 'w', encoding='utf-8').write('\n'.join(R)); print('\n'.join(R))
    geo = {'units': 'mm, origin top-left, y down', 'board': [rb.W, rb.H, rb.R],
           'pads': [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in q.items()} for q in rb.pads],
           'lines': [{'id': l['id'], 'net': l['net'], 'layer': l['layer'], 'w': round(l['w'], 3), 'a': [round(l['a'][0], 3), round(l['a'][1], 3)], 'b': [round(l['b'][0], 3), round(l['b'][1], 3)]} for l in rb.lines],
           'vias': [{'id': v['id'], 'net': v['net'], 'x': round(v['p'][0], 3), 'y': round(v['p'][1], 3), 'drill': round(v['drill'], 3), 'dia': round(v['dia'], 3)} for v in rb.vias],
           'components': {r: {'pos': rb.layout[r], 'box': [round(v, 2) for v in rb.prj.box(r, rb.layout[r])]} for r in rb.prj.refs}}
    json.dump(geo, open(os.path.join(outdir, 'geometry.json'), 'w'), ensure_ascii=False)
    for name, box in (('boost', (30, 3, 50, 23)), ('input', (0, 5, 32, 24)), ('rs422', (0, 12, 24, 30)), ('mcu', (14, 32, 34, 50)), ('analog', (26, 28, 50, 48)), ('debug', (0, 28, 24, 50))):
        for layer, nm in ((TOP, 'top'), (BOT, 'bot')):
            render(rb, layer, os.path.join(outdir, f'{name}_{nm}.png'), box=box, scale=2.0)
    render(rb, TOP, os.path.join(outdir, 'top.png')); render(rb, BOT, os.path.join(outdir, 'bottom.png'))
    import pickle
    return rb

if __name__ == '__main__':
    main()
