#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对已经布线的嘉立创EDA专业版工程(.epro2)做修改, 输出一个新的工程文件:
  1. 走线优化 (见 apply_fixes)
  2. 隐藏所有元件位号丝印 (元件外形框 极性标记保留)
  3. 给连接器 测试点 按键加引脚名丝印 (用丝印层折线画出的笔画字)
  4. 删掉重复的铺铜区 清空铺铜填充 —— 导入后必须重建铺铜(Shift+B)

用法: python3 pcbfix.py 输入.epro2 输出.epro2 [预览图目录] [--no-pour]
      输入必须是 已布线工程_原版.epro2: 改线部分(apply_fixes)是按那个文件里走线的编号和坐标写死的, 不能用在别的版本上, 也不能对改过的文件再跑一遍
      --no-pour: 把铺铜区整个删掉(备用: 万一清空填充的文件导入不了, 用这个版本, 导入后自己重画两块整板GND铺铜)
      python3 pcbfix.py 任意版本.epro2 输出.epro2 [预览图目录] --labels-only [--replace-silk]
      --labels-only: 只隐藏位号 + 画引脚名, 不动走线 过孔 铺铜. 以后挪了元件想重新排引脚名就用这个.
                     工程里已经有丝印折线(比如上次生成的引脚名)时要加 --replace-silk, 会先把PCB上顶层/底层丝印的折线全部删掉再画
坐标约定: 毫米, 原点在板子左上角, y向下 (与 pcbreview.py 的 geometry.json 一致)
"""
import sys, os, io, json, math, random, zipfile
import numpy as np
from scipy import ndimage as ndi
import pcbreview as RV
from pcbreview import MIL, TOP, BOT
from PIL import Image, ImageDraw

TOP_SILK, BOT_SILK = 3, 4

# 笔画字库: 每个字符画在 宽4 x 高6 的格子里, y向上, 基线y=0
O = [(0, 1), (0, 5), (1, 6), (3, 6), (4, 5), (4, 1), (3, 0), (1, 0), (0, 1)]
FONT = {
    '0': [O], 'O': [O],
    '1': [[(1, 5), (2, 6), (2, 0)], [(1, 0), (3, 0)]],
    '2': [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (0, 0), (4, 0)]],
    '3': [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (3, 3), (1.5, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    '5': [[(4, 6), (0, 6), (0, 3.4), (3, 3.4), (4, 2.5), (4, 1), (3, 0), (1, 0), (0, 1)]],
    'A': [[(0, 0), (0, 4), (2, 6), (4, 4), (4, 0)], [(0, 2.2), (4, 2.2)]],
    'C': [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1)]],
    'D': [[(0, 0), (0, 6), (2.5, 6), (4, 4.5), (4, 1.5), (2.5, 0), (0, 0)]],
    'G': [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1), (4, 2.8), (2.2, 2.8)]],
    'H': [[(0, 0), (0, 6)], [(4, 0), (4, 6)], [(0, 3), (4, 3)]],
    'I': [[(2, 0), (2, 6)], [(1, 0), (3, 0)], [(1, 6), (3, 6)]],
    'K': [[(0, 0), (0, 6)], [(4, 6), (0, 2.4)], [(1.4, 3.6), (4, 0)]],
    'L': [[(0, 6), (0, 0), (4, 0)]],
    'N': [[(0, 0), (0, 6), (4, 0), (4, 6)]],
    'R': [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(2, 3), (4, 0)]],
    'S': [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 4), (1, 3), (3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    'T': [[(0, 6), (4, 6)], [(2, 6), (2, 0)]],
    'V': [[(0, 6), (2, 0), (4, 6)]],
    'X': [[(0, 6), (4, 0)], [(0, 0), (4, 6)]],
    '+': [[(0.5, 3), (3.5, 3)], [(2, 1.3), (2, 4.7)]],
    '-': [[(0.5, 3), (3.5, 3)]],
}
CHAR_W, CHAR_ADV = 4.0, 6.2          # 字宽 字间距(格). 间距6.2格: 1mm字高时相邻两字笔画之间净空0.21mm, 印出来不会粘在一起

def text_size(s, h):
    u = h / 6.0
    return len(s) * CHAR_W * u + (len(s) - 1) * (CHAR_ADV - CHAR_W) * u

def text_strokes(s, cx, cy, h, rot=0, mirror=False):
    """返回 [[(x,y),...],...] 以(cx,cy)为中心. rot: 俯视逆时针角度. mirror: 左右镜像(底层丝印用)"""
    u = h / 6.0; W = text_size(s, h); c, sn = math.cos(math.radians(rot)), math.sin(math.radians(rot)); out = []
    for i, ch in enumerate(s):
        for st in FONT[ch]:
            pts = []
            for gx, gy in st:
                lx = i * CHAR_ADV * u + gx * u - W / 2; ly = gy * u - h / 2
                if mirror: lx = -lx
                pts.append((cx + lx * c - ly * sn, cy - (lx * sn + ly * c)))
            out.append(pts)
    return out

def text_box(s, cx, cy, h, rot=0):
    W = text_size(s, h)
    return (cx - W / 2, cy - h / 2, cx + W / 2, cy + h / 2) if rot % 180 == 0 else (cx - h / 2, cy - W / 2, cx + h / 2, cy + W / 2)

def num(v):
    """写进文件的数: 保留4位小数, 整数不带小数点(和EDA自己写的格式一致)"""
    r = round(v, 4)
    return int(r) if r == int(r) else r

def hit(a, b, gap=0.0):
    return a[0] - gap < b[2] and b[0] - gap < a[2] and a[1] - gap < b[3] and b[1] - gap < a[3]


class Editor:
    def __init__(self, path):
        self.rb = RV.Routed(path); self.prj = self.rb.prj
        self.lines = list(self.prj.lines); self.new = []; self.log = []
        heads = [json.loads(l.split('||')[0]) for l in self.lines if l.strip()]
        self.ticket = max(h.get('ticket', 0) for h in heads) + 1
        self.ids = {h.get('id') for h in heads}; self.rng = random.Random(20261123)
        self.item = {l['id']: l for l in self.rb.lines}; self.item.update({v['id']: v for v in self.rb.vias})
        for k, it in self.item.items(): assert k in self.lines[it['row']], k
        self.tears = {}                       # 走线/过孔id -> 泪滴所在行
        for h, b, i in self.prj.pcb['rows']:
            if h['type'] == 'TEARDROP':
                for r in b.get('refs', []): self.tears.setdefault(r, []).append(i)
        self.labels = []                      # (层, 文字, 外框, 笔画)
        # 已有走线端点和过孔中心的精确坐标(mil). 新线接到它们上面时直接用原值, 不经过毫米换算, 免得端点之间出现零点几微米的错位
        self._known = None                    # 每次删线/改线后作废, 用到时按当前还存在的走线和过孔重建

    def known(self):
        if self._known is None:
            self._known = []
            rows = [self.lines[i] for _, _, i in self.prj.pcb['rows']] + self.new
            for l in rows:
                if not l or not l.startswith(('{"type":"LINE"', '{"type":"VIA"')): continue
                b = json.loads(l.partition('||')[2].rstrip('|'))
                self._known += [(b['startX'], b['startY']), (b['endX'], b['endY'])] if 'startX' in b else [(b['centerX'], b['centerY'])]
        return self._known

    def mil(self, x, y, snap=True):
        mx, my = self.prj.ox + x / MIL, self.prj.oy - y / MIL
        if snap and self.known():
            p = min(self.known(), key=lambda p: abs(p[0] - mx) + abs(p[1] - my))
            if math.hypot(p[0] - mx, p[1] - my) < 0.04: return p          # 0.04mil = 1微米以内视为同一点
        return (num(mx), num(my))

    def _edit(self, row, fn):
        head, _, body = self.lines[row].partition('||'); tail = '|' if body.endswith('|') else ''
        b = json.loads(body.rstrip('|')); fn(b)
        self.lines[row] = head + '||' + json.dumps(b, ensure_ascii=False, separators=(',', ':')) + tail

    def _add(self, typ, body):
        while True:
            i = '%016x' % self.rng.getrandbits(64)
            if i not in self.ids: break
        self.ids.add(i)
        head = json.dumps({'type': typ, 'ticket': self.ticket, 'id': i}, separators=(',', ':')); self.ticket += 1
        self.new.append(head + '||' + json.dumps(body, ensure_ascii=False, separators=(',', ':')) + '|'); self._known = None; return i

    def _drop_tears(self, i):
        for row in self.tears.get(i, []): self.lines[row] = None

    # ---- 走线 过孔 ----
    def move(self, lid, a0, b0, a1, b1):
        """把走线lid从 a0->b0 改成 a1->b1 (先核对原来的端点, 防止改错线)"""
        l = self.item[lid]
        assert math.dist(l['a'], a0) < 0.01 and math.dist(l['b'], b0) < 0.01, (lid, l['a'], l['b'])
        def fn(b):       # 没挪的那一头原样保留
            if math.dist(a0, a1) > 1e-6: b['startX'], b['startY'] = self.mil(*a1)
            if math.dist(b0, b1) > 1e-6: b['endX'], b['endY'] = self.mil(*b1)
        self._edit(l['row'], fn); self._drop_tears(lid); self._known = None
        self.log.append(f"改线 {l['net']} 层{l['layer']} ({a0[0]},{a0[1]})->({b0[0]},{b0[1]})  =>  ({a1[0]},{a1[1]})->({b1[0]},{b1[1]})")

    def remove(self, *ids):
        for i in ids:
            it = self.item[i]; self.lines[it['row']] = None; self._drop_tears(i); self._known = None
            self.log.append((f"删线 {it['net']} 层{it['layer']} ({it['a'][0]:.2f},{it['a'][1]:.2f})->({it['b'][0]:.2f},{it['b'][1]:.2f})" if 'a' in it
                             else f"删过孔 {it['net']} ({it['p'][0]:.2f},{it['p'][1]:.2f})"))

    def add_line(self, net, layer, a, b, w_mil):
        (sx, sy), (ex, ey) = self.mil(*a), self.mil(*b)
        self._add('LINE', {"partitionId": "", "groupId": 0, "netName": net, "layerId": layer, "startX": sx, "startY": sy, "endX": ex, "endY": ey,
                           "width": w_mil, "locked": False, "zIndex": -1})
        self.log.append(f"加线 {net} 层{layer} ({a[0]},{a[1]})->({b[0]},{b[1]}) 线宽{w_mil * MIL:.2f}mm")

    def add_via(self, net, x, y, hole=12.0078, dia=24.0158):
        cx, cy = self.mil(x, y)
        self._add('VIA', {"partitionId": "", "groupId": 0, "netName": net, "ruleName": "", "centerX": cx, "centerY": cy, "holeDiameter": hole,
                          "viaDiameter": dia, "viaType": "NORMAL", "topSolderExpansion": None, "bottomSolderExpansion": None, "locked": False,
                          "unusedInnerLayers": [], "propagationDelay": 0})
        self.log.append(f"加过孔 {net} ({x},{y})")

    # ---- 自动找位置加地过孔 ----
    def prepare(self):
        """算出每层上 离非GND铜皮/离焊盘/离板边 的距离图, 供找过孔位置用"""
        rb = self.rb; rb.build(); PX = RV.PX; self.d_other = {}; self.d_pad = {}
        for layer in (TOP, BOT):
            other = np.zeros((rb.M, rb.N), bool)
            for n, m in rb.masks[layer].items():
                if n != 'GND': other |= m
            self.d_other[layer] = ndi.distance_transform_edt(~other) / PX
            im = rb._canvas(); d = ImageDraw.Draw(im)
            for q in rb.pads:
                if layer == TOP or q['th']: rb._draw_pad(d, q)
            self.d_pad[layer] = ndi.distance_transform_edt(~np.array(im, bool)) / PX
        self.d_edge = rb.edge_distance(); self.via_pts = [v['p'] for v in rb.vias]
        self.pad_bbox = {}                     # 每个元件所有焊盘的外接框: 过孔不打在元件肚子底下(两个焊盘之间)
        for q in rb.pads:
            b = self.pad_bbox.get(q['ref'], (9e9, 9e9, -9e9, -9e9))
            self.pad_bbox[q['ref']] = (min(b[0], q['x'] - q['w'] / 2), min(b[1], q['y'] - q['h'] / 2), max(b[2], q['x'] + q['w'] / 2), max(b[3], q['y'] + q['h'] / 2))

    def via_ok(self, x, y, clear=0.25, pad_gap=0.2, pitch=0.9, r=0.305):
        """(x,y)能不能打一个地过孔: 两层都离别的网络>=clear 不压任何焊盘 不在元件肚子底下 离别的过孔>=pitch 焊盘边离板边>=1mm 不压丝印字"""
        PX = RV.PX; i, j = int(y * PX), int(x * PX)
        if not (0 <= i < self.rb.M and 0 <= j < self.rb.N) or self.d_edge[i, j] < 1.0 + r: return False
        for layer in (TOP, BOT):
            if self.d_other[layer][i, j] < r + clear or self.d_pad[layer][i, j] < r + pad_gap: return False
        if any(hit((x - r, y - r, x + r, y + r), b, 0.15) for _, _, b, *_ in self.labels): return False
        if any(hit((x - r, y - r, x + r, y + r), b) for b in self.pad_bbox.values()): return False
        return all(math.dist((x, y), p) >= pitch for p in self.via_pts)

    def track_ok(self, a, b, w, layer, clear=0.2):
        n = max(2, int(math.dist(a, b) / 0.05) + 1); PX = RV.PX
        for k in range(n + 1):
            x = a[0] + (b[0] - a[0]) * k / n; y = a[1] + (b[1] - a[1]) * k / n
            if self.d_other[layer][int(y * PX), int(x * PX)] < w / 2 + clear: return False
        return True

    def gnd_via_near(self, target, why, pad=None, w_mil=12, reach=1.5, grid=0.127):
        """在target附近找最近的合法位置打一个地过孔. pad=(x,y): 再从这个焊盘拉一条顶层地线连过去,
        这时过孔只在焊盘的 横/竖/45度 八个方向上找, 连线才规整; 不带pad时对齐5mil网格"""
        best = None
        if pad:
            for k in range(8):
                ux, uy = math.cos(k * math.pi / 4), math.sin(k * math.pi / 4); t = 0.5
                while t <= reach + 1.0:
                    x, y = round(pad[0] + ux * t, 3), round(pad[1] + uy * t, 3); d = math.dist((x, y), target); t += 0.05
                    if d > reach or (best and d >= best[0]) or not self.via_ok(x, y) or not self.track_ok(pad, (x, y), w_mil * MIL, TOP): continue
                    best = (d, x, y)
        else:
            n = int(reach / grid); gx0, gy0 = round(target[0] / grid), round(target[1] / grid)
            for ix in range(gx0 - n, gx0 + n + 1):
                for iy in range(gy0 - n, gy0 + n + 1):
                    x, y = ix * grid, iy * grid; d = math.dist((x, y), target)
                    if d > reach or (best and d >= best[0]) or not self.via_ok(x, y): continue
                    best = (d, round(x, 3), round(y, 3))
        if not best: self.log.append(f"!! {why}: 在({target[0]},{target[1]})附近{reach}mm内找不到能打地过孔的位置, 跳过"); return None
        _, x, y = best; self.add_via('GND', x, y); self.via_pts.append((x, y)); self.log[-1] += f"  [{why}]"
        if pad: self.add_line('GND', TOP, pad, (x, y), w_mil)
        return (x, y)

    def gnd_vias_in(self, box, k, why, spacing=2.0, grid=0.635):
        """在矩形区域里均匀补k个地过孔: 每次选离所有已有过孔最远的合法位置"""
        x0, y0, x1, y1 = box; cand = []
        for ix in range(int(math.ceil(x0 / grid)), int(x1 / grid) + 1):
            for iy in range(int(math.ceil(y0 / grid)), int(y1 / grid) + 1):
                x, y = round(ix * grid, 3), round(iy * grid, 3)
                if self.via_ok(x, y): cand.append((x, y))
        done = []
        for _ in range(k):
            best = None
            for c in cand:
                d = min(math.dist(c, p) for p in self.via_pts)
                if d >= spacing and (best is None or d > best[0]): best = (d, c)
            if not best: break
            x, y = best[1]; self.add_via('GND', x, y); self.via_pts.append((x, y)); self.log[-1] += f"  [{why}]"; done.append((x, y))
        if len(done) < k: self.log.append(f"!! {why}: 只找到{len(done)}个合法位置(想加{k}个)")
        return done

    # ---- 丝印 ----
    def hide_designators(self):
        n = 0
        for h, b, i in self.prj.pcb['rows']:
            if h['type'] == 'ATTR' and b.get('key') == 'Designator' and self.lines[i]:
                def fn(b): b['valueVisible'] = False; b['keyVisible'] = False
                self._edit(i, fn); n += 1
        self.log.append(f"隐藏位号丝印 {n} 个"); return n

    def text(self, s, cx, cy, h, layer=TOP_SILK, rot=0, stroke_mil=6):
        strokes = text_strokes(s, cx, cy, h, rot, mirror=(layer == BOT_SILK))
        for pts in strokes:
            path = []
            for k, (x, y) in enumerate(pts):
                mx, my = self.mil(x, y, snap=False)
                if k == 1: path.append('L')
                path += [mx, my]
            self._add('POLY', {"partitionId": "", "groupId": 0, "netName": "", "layerId": layer, "width": stroke_mil, "path": path,
                               "locked": False, "zIndex": -1, "polyType": "NORMAL"})
        self.labels.append((layer, s, text_box(s, cx, cy, h, rot), strokes, h, rot))

    # ---- 铺铜 ----
    def pours(self, drop=(), drop_all=False):
        gone = set()
        for h, b, i in self.prj.pcb['rows']:
            if h['type'] == 'POUR' and (drop_all or b.get('name') in drop):
                self.lines[i] = None; gone.add(h['id'])
                self.log.append(f"删除{'重复' if b.get('name') in drop else ''}铺铜区 {b.get('name')} (层{b['layerId']} {b['netName']})")
        for h, b, i in self.prj.pcb['rows']:
            if h['type'] == 'POURED':
                if json.loads(h['id'])[1] in gone: self.lines[i] = None
                else:
                    def fn(b): b['pourFill'] = []
                    self._edit(i, fn)
        self.log.append("铺铜区已全部删除: 导入后要自己重画顶层 底层两块整板GND铺铜" if drop_all else "清空铺铜填充数据: 导入后必须重建铺铜")

    def save(self, out):
        last = max(i for _, _, i in self.prj.pcb['rows'])  # 新图元追加在PCB文档的最后
        res = []
        for i, l in enumerate(self.lines):
            if l is not None: res.append(l)
            if i == last: res += self.new
        # 原文件的约定: 每条记录以'|'结尾, 只有全文件最后一条不带. 保持一致
        bare_end = not self.prj.lines[-1].endswith('|')
        res = [l if l.endswith('|') else l + '|' for l in res]
        if bare_end: res[-1] = res[-1][:-1]
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as zo:
            for it in self.prj.zip.infolist():
                zo.writestr(it, '\n'.join(res).encode('utf-8') if it.filename == self.prj.epru else self.prj.zip.read(it.filename))
        open(out, 'wb').write(buf.getvalue())


def apply_fixes(ed):
    """走线优化. 每一条的原因见 Hardware/布线检查报告.md"""
    # 1. 升压芯片U3的输入脚(5脚IN 4脚EN)原来靠一条20mm长的底层线从TP2供电, 身边的去耦电容C4反而没有直接连到它.
    #    改成: C4 -> U3.5 -> U3.4 在顶层直接相连(2.6mm), 删掉底层那条长线和3个打在焊盘上的过孔, 底层地平面恢复完整
    ed.remove('4b0e51bb68044917', '7098e47507dab3c3', 'ee51bab9e1fee759', 'c9f9e85b78608eb7', '0fcf24f57bc7c5ab',
              'b40337b2bd859045', '71999d78950b7da0', '263aa91c180883ed')
    ed.add_line('+5V', TOP, (34.990, 11.430), (37.586, 11.430), 16)
    ed.add_line('+5V', TOP, (37.586, 11.430), (37.586, 10.480), 12)
    # 2. 单片机U1底部的接地焊盘原来没有过孔, 只靠顶层细铜皮接地. 加3个过孔直通底层地(位置避开了底层的DBG_RX线). 只是电气上接地更结实, 孔太小不能用来从背面灌锡
    for x in (22.9, 24.0, 25.1): ed.add_via('GND', x, 42.35)
    # 3. 顶层 DBG_TX 出线左移0.13mm: 它和ADC2的间距原来只有0.105mm(规则下限0.102), 改后0.197mm
    ed.move('389238168ddaf2a1', (26.490, 39.550), (27.125, 39.550), (26.490, 39.550), (26.995, 39.550))
    ed.move('a33a4b6574b66263', (27.125, 39.550), (27.305, 39.370), (26.995, 39.550), (27.175, 39.370))
    ed.move('31ea42fde0ff4461', (27.305, 39.370), (27.305, 37.465), (27.175, 39.370), (27.175, 37.335))
    ed.move('9c6b441959729117', (26.035, 36.195), (27.305, 37.465), (26.035, 36.195), (27.175, 37.335))
    # 4. 底层 NRST 竖线左移0.2mm: 它和C5焊盘上3V3过孔/线头的间距原来只有约0.11mm, 改后0.32mm
    #    同时把贴着下板边的那段NRST横线上移0.1mm: 离板边 0.34mm -> 0.44mm
    ed.move('105659fb6f9a2ee1', (27.940, 41.910), (27.305, 41.910), (27.940, 41.910), (27.105, 41.910))
    ed.move('1e01a20d53e9431a', (27.305, 41.910), (27.125, 42.090), (27.105, 41.910), (26.925, 42.090))
    ed.move('4456e704980dee14', (27.125, 42.090), (27.125, 49.075), (26.925, 42.090), (26.925, 49.175))
    ed.move('5aad7f488b78926d', (27.125, 49.075), (26.670, 49.530), (26.925, 49.175), (26.670, 49.430))
    ed.move('6d99ac63fc027a4b', (26.670, 49.530), (13.335, 49.530), (26.670, 49.430), (13.235, 49.430))
    ed.move('a48ea9441e7639ae', (13.335, 49.530), (12.700, 48.895), (13.235, 49.430), (12.700, 48.895))
    # 5. 顶层 ADC2 进芯片前的那个小拐弯左移0.07mm: 它和ADC1的间距 0.127mm -> 0.197mm (第3条把DBG_TX挪开后腾出了位置)
    ed.move('ceb4bf69d33f8240', (28.067, 39.116), (27.559, 39.624), (28.067, 39.116), (27.489, 39.694))
    ed.move('ec3a2448fceefb34', (27.559, 39.624), (27.559, 39.878), (27.489, 39.694), (27.489, 39.878))
    ed.move('8e3c021343652ed6', (27.559, 39.878), (27.379, 40.058), (27.489, 39.878), (27.309, 40.058))
    ed.move('68ed507c3bcf95ab', (27.379, 40.058), (26.498, 40.058), (27.309, 40.058), (26.498, 40.058))
    # 6. 传感器的12V原来是从二极管D3的焊盘上引出去的, 改成从输出电容C3的焊盘引出(从板子东侧绕下来).
    #    二极管到电容这一小段铜线上流的是每个开关周期一次的脉冲电流, 从这段线中间取电会把开关尖峰直接带给传感器;
    #    规矩是: 二极管 -> 输出电容 -> 负载, 负载从电容的焊盘上取电
    ed.remove('95856dac68c4178c', '339bb872623ce4b0')
    #    竖线贴着C3 D3的焊盘边走(x=46.75): 中间不留缝, 重铺铜时这里就不会夹出一条又细又长的死胡同地铜皮
    pts = [(45.493, 10.000), (46.250, 10.000), (46.750, 10.500), (46.750, 15.000), (46.250, 15.500), (44.825, 15.500), (43.900, 16.425)]
    for a, b in zip(pts, pts[1:]): ed.add_line('+12V', TOP, a, b, 30)
    # 7. 顶层 UART_RX 竖线右移0.08mm: 它离U4第1脚(3V3)焊盘原来是0.152mm, 正好压在"线到焊盘"规则的下限上, EDA的DRC可能因为舍入报错. 改后0.23mm
    ed.move('7165f251d6bce044', (20.574, 41.021), (19.759, 40.206), (20.574, 41.021), (19.839, 40.286))
    ed.move('b73c007ee46b9241', (19.759, 40.206), (19.759, 26.089), (19.839, 40.286), (19.839, 26.169))
    ed.move('6d1eec595b0377a4', (19.759, 26.089), (19.505, 25.835), (19.839, 26.169), (19.505, 25.835))


def add_ground_vias(ed):
    """补地过孔(位置由程序在目标点附近找, 保证两层都离别的网络0.25mm以上 不压焊盘 不压丝印字)"""
    ed.prepare()
    P = {f"{q['ref']}.{q['num']}": (round(q['x'], 3), round(q['y'], 3)) for q in ed.rb.pads}
    # 升压输入电容C2的地脚: 原来只靠一条4.7mm的顶层线绕到C4旁边才下到底层地. 开关电流的回路是 C2 -> L1 -> U3 -> 地 -> C2, 地这一段越短越好
    #   C2地脚本来就有一条0.76mm的顶层地线通到C4地脚, 两个过孔直接打在这条线上
    ed.gnd_via_near((33.2, 14.6), 'C2地脚(打在原有的地线上)'); ed.gnd_via_near((33.655, 12.8), 'C2/C4之间的地线上')
    # U3和L1之间那块顶层地铜皮被+5V线 开关节点和电感焊盘围住, 里面一个过孔都没有
    ed.gnd_via_near((36.3, 14.5), '升压区顶层地'); ed.gnd_via_near((38.8, 14.8), '升压区顶层地')
    # D3下方 开关节点旁边那一小块顶层地铜皮, 也给一个过孔
    ed.gnd_via_near((43.7, 14.6), 'D3下方顶层地')
    # 稳压芯片U2的地脚和输出电容C12的地脚旁边没有过孔
    ed.gnd_via_near((21.6, 18.288), 'U2地脚', pad=P['U2.2'], w_mil=12)
    ed.gnd_via_near((29.275, 18.75), 'C12地脚', pad=P['C12.2'], w_mil=12)
    # 防浪涌二极管D1的地脚: 浪涌电流要就近进地平面
    ed.gnd_via_near((12.7, 15.7), 'D1地脚', pad=P['D1.2'], w_mil=20)
    # 模拟前端下面的底层地被3V3线和NRST线围了一圈, 只剩一个2.5mm的口子和3个过孔与外面相连, 补几个过孔把它和顶层地缝起来
    ed.gnd_vias_in((28.6, 34.0, 38.6, 45.6), 6, '模拟区地', spacing=2.0)
    # 板子东边(升压区右侧)一个地过孔都没有
    ed.gnd_vias_in((48.0, 8.5, 49.0, 21.0), 3, '东侧地', spacing=2.5)


# 引脚名: 按网络取名, 所以即使以后原理图里调换了引脚, 重新运行也不会标错
NET_NAME = {'H11': '5V', '+5V': '5V', '3V3': '3V3', '+12V': '12V', 'H21': '12V', 'H31': '12V', 'GND': 'GND', 'TX+': 'TX+', 'TX-': 'TX-', 'RX+': 'RX+',
            'RX-': 'RX-', 'S1_OUT': 'S1', 'S2_OUT': 'S2', 'SWCLK': 'CLK', 'SWDIO': 'DIO', 'NRST': 'RST', 'DBG_TX': 'TX', 'DBG_RX': 'RX', 'ADC1': 'ADC1', 'ADC2': 'ADC2'}

def add_labels(ed):
    rb = ed.rb; pads = {}
    for q in rb.pads: pads.setdefault(q['ref'], []).append(q)
    boxes = {r: rb.prj.box(r, rb.layout[r]) for r in rb.prj.refs}
    holes = [(v['p'][0] - v['dia'] / 2, v['p'][1] - v['dia'] / 2, v['p'][0] + v['dia'] / 2, v['p'][1] + v['dia'] / 2) for v in rb.vias if not v['net']]
    board = (0.5, 0.5, rb.W - 0.5, rb.H - 0.5); problems = []

    def free(box, layer, own_ok=None):
        if not (box[0] >= board[0] and box[1] >= board[1] and box[2] <= board[2] and box[3] <= board[3]): return '出板'
        for hb in holes:
            if hit(box, hb, 0.2): return '安装孔'
        if layer == TOP_SILK:
            for r, b in boxes.items():
                if hit(box, b, 0.12): return r
        else:
            for q in rb.pads:
                if q['th'] and hit(box, (q['x'] - q['w'] / 2, q['y'] - q['h'] / 2, q['x'] + q['w'] / 2, q['y'] + q['h'] / 2), 0.2): return q['ref']
        for layer2, s, b, *_ in ed.labels:
            if layer2 == layer and hit(box, b, 0.2): return '丝印' + s
        return None

    def put(s, cx, cy, h, layer, rot=0):
        why = free(text_box(s, cx, cy, h, rot), layer)
        if why: problems.append(f"{s} @({cx:.2f},{cy:.2f}) 与 {why} 冲突")
        ed.text(s, cx, cy, h, layer, rot)

    def around(ref, s, h=1.0, gap=0.3, order=('above', 'below', 'right', 'left')):
        x0, y0, x1, y1 = boxes[ref]; W = text_size(s, h)
        cand = {'above': ((x0 + x1) / 2, y0 - gap - h / 2), 'below': ((x0 + x1) / 2, y1 + gap + h / 2),
                'right': (x1 + gap + W / 2, (y0 + y1) / 2), 'left': (x0 - gap - W / 2, (y0 + y1) / 2)}
        for k in order:
            if free(text_box(s, *cand[k], h), TOP_SILK) is None:
                ed.text(s, *cand[k], h, TOP_SILK); return k
        problems.append(f"{ref} 的丝印 {s} 四周都放不下"); return None

    # 调试口CN1: 1.27mm间距, 字竖着写在连接器上方, 每个字对准自己的引脚
    top = min(q['y'] - q['h'] / 2 for q in pads['CN1'])
    for q in pads['CN1']:
        s = NET_NAME[q['net']]; h = 0.9; put(s, q['x'], top - 0.35 - text_size(s, h) / 2, h, TOP_SILK, rot=90)
    # 测试点 复位键: 字放在旁边空位
    for ref in ('TP1', 'TP2', 'TP3', 'TP4', 'TP5', 'TP6'):
        around(ref, NET_NAME[pads[ref][0]['net']])
    around('SW1', 'RST')
    # 三个直插排针: 完整的一套引脚名写在底层(镜像, 从背面看是正的), 字在引脚靠板内一侧.
    # 顶层紧挨着排针就是别的元件, 只有部分引脚旁边有空位: 能放下的也在顶层放一份(首脚放不下就放在排针上方, 末脚放不下就放在下方)
    top_done = []
    for ref in ('H1', 'H2', 'H3'):
        inner = 1 if pads[ref][0]['x'] < rb.W / 2 else -1; gnd = 0
        ps = sorted(pads[ref], key=lambda q: int(q['num'])); x0, y0, x1, y1 = boxes[ref]
        for k, q in enumerate(ps):
            s = NET_NAME[q['net']]
            if ref != 'H1' and s == 'GND':
                gnd += 1
                if gnd == 2: s = 'SHD'          # 传感器第3脚接屏蔽线(板上也是地)
            h = 1.0; W = text_size(s, h)
            put(s, q['x'] + inner * (q['w'] / 2 + 0.4 + W / 2), q['y'], h, BOT_SILK)
            cand = [((x1 + 0.3 + W / 2) if inner > 0 else (x0 - 0.3 - W / 2), q['y'])]
            if k == 0: cand.append((q['x'], y0 - 0.3 - h / 2))
            if k == len(ps) - 1: cand.append((q['x'], y1 + 0.3 + h / 2))
            for c in cand:
                if free(text_box(s, *c, h), TOP_SILK) is None:
                    ed.text(s, *c, h, TOP_SILK); top_done.append(f"{ref}.{q['num']}={s}"); break
    ed.log.append("排针顶层能放下的引脚名: " + ' '.join(top_done))
    return problems


def preview(ed, out_dir):
    """丝印预览: top_silk.png(俯视) bottom_silk.png(从背面看, 已左右翻转)"""
    rb = ed.rb; S = 24
    for layer, name in ((TOP_SILK, 'top_silk.png'), (BOT_SILK, 'bottom_silk.png')):
        im = Image.new('RGB', (int(rb.W * S), int(rb.H * S)), (20, 70, 40)); d = ImageDraw.Draw(im)
        for q in rb.pads:
            if layer == TOP_SILK or q['th']:
                d.rectangle([(q['x'] - q['w'] / 2) * S, (q['y'] - q['h'] / 2) * S, (q['x'] + q['w'] / 2) * S, (q['y'] + q['h'] / 2) * S], fill=(200, 170, 60))
        for v in rb.vias:
            x, y = v['p']; r = (v['dia'] if not v['net'] else v['drill']) / 2
            d.ellipse([(x - r) * S, (y - r) * S, (x + r) * S, (y + r) * S], fill=(10, 30, 20))
        if layer == TOP_SILK:
            for r in rb.prj.refs:
                b = rb.prj.box(r, rb.layout[r]); d.rectangle([b[0] * S, b[1] * S, b[2] * S, b[3] * S], outline=(120, 160, 130))
        for l2, s, b, strokes, h, rot in ed.labels:
            if l2 != layer: continue
            for pts in strokes: d.line([(x * S, y * S) for x, y in pts], fill=(255, 255, 255), width=max(2, int(round(0.15 * S))), joint='curve')
        if layer == BOT_SILK: im = im.transpose(Image.FLIP_LEFT_RIGHT)
        im.save(os.path.join(out_dir, name))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]; no_pour = '--no-pour' in sys.argv
    src, out = args[0], args[1]; pv = args[2] if len(args) > 2 else None
    if '--labels-only' in sys.argv:
        ed = Editor(src)
        old = [i for h, b, i in ed.prj.pcb['rows'] if h['type'] == 'POLY' and b.get('layerId') in (TOP_SILK, BOT_SILK)]
        if old and '--replace-silk' not in sys.argv:
            sys.exit(f"工程的PCB上已经有{len(old)}条丝印折线(可能是上次生成的引脚名). 确认要全部删掉重画的话, 加上 --replace-silk")
        for i in old: ed.lines[i] = None
        if old: ed.log.append(f"删除原有的丝印折线 {len(old)} 条")
        ed.hide_designators(); problems = add_labels(ed); ed.save(out)
        print('\n'.join(ed.log)); print(f"加引脚丝印 {len(ed.labels)} 处: " + ' '.join(s for _, s, *_ in ed.labels))
        for p in problems: print('!! 丝印冲突:', p)
        if pv: os.makedirs(pv, exist_ok=True); preview(ed, pv)
        print('已写出(只改丝印, 走线和铺铜没有动)', out); return
    tmp = out + '.tmp'
    e1 = Editor(src); apply_fixes(e1); e1.save(tmp)          # 第一步: 改线. 存一遍再读回来, 第二步找过孔位置时看到的就是改完线的板子
    ed = Editor(tmp); os.remove(tmp)
    ed.hide_designators()
    problems = add_labels(ed)
    add_ground_vias(ed)
    ed.pours(drop=('POUR3', 'POUR4'), drop_all=no_pour)
    ed.save(out)
    print('\n'.join(e1.log + ed.log)); print(f"加引脚丝印 {len(ed.labels)} 处: " + ' '.join(f"{s}" for _, s, *_ in ed.labels))
    for p in problems: print('!! 丝印冲突:', p)
    if pv: os.makedirs(pv, exist_ok=True); preview(ed, pv)
    # 验证: 重新读入输出文件, 模拟重建铺铜后检查连通 短路 间距 悬空线头
    if no_pour: print('已写出(无铺铜版)', out); return
    rb = RV.Routed(out).build().simulate_pour()
    opens = rb.connectivity(); cl = rb.clearance(0.16); dang = rb.dangling()
    print(f"\n== 验证(模拟重建铺铜) ==\n未连通网络 {len(opens)} 个: {opens}")
    print(f"短路 {sum(1 for c in cl if c[0] <= 0)} 处; 间距小于0.16mm的 {len(cl)} 对:")
    for c in cl: print(f"  {c[0]:.3f} mm 层{c[1]} {c[2]} - {c[3]} @({c[4]},{c[5]})")
    print(f"悬空线头 {len(dang)} 处: {dang}")
    print('已写出', out)

if __name__ == '__main__':
    main()
