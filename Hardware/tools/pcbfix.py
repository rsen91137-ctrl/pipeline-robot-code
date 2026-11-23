#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对已经布线的嘉立创EDA专业版工程(.epro2)做修改, 输出一个新的工程文件:
  1. 走线优化 (见 apply_fixes)
  2. 隐藏所有元件位号丝印 (元件外形框 极性标记保留)
  3. 给连接器 测试点 按键加引脚名丝印 (用丝印层折线画出的笔画字)
  4. 删掉重复的铺铜区 清空铺铜填充 —— 导入后必须重建铺铜(Shift+B)

用法: python3 pcbfix.py 输入.epro2 输出.epro2 [预览图目录]
坐标约定: 毫米, 原点在板子左上角, y向下 (与 pcbreview.py 的 geometry.json 一致)
"""
import sys, os, io, json, math, random, zipfile
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
CHAR_W, CHAR_ADV = 4.0, 5.3          # 字宽 字间距(格)

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

    def mil(self, x, y): return (round(self.prj.ox + x / MIL, 4), round(self.prj.oy - y / MIL, 4))

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
        self.new.append(head + '||' + json.dumps(body, ensure_ascii=False, separators=(',', ':')) + '|'); return i

    def _drop_tears(self, i):
        for row in self.tears.get(i, []): self.lines[row] = None

    # ---- 走线 过孔 ----
    def move(self, lid, a0, b0, a1, b1):
        """把走线lid从 a0->b0 改成 a1->b1 (先核对原来的端点, 防止改错线)"""
        l = self.item[lid]
        assert math.dist(l['a'], a0) < 0.01 and math.dist(l['b'], b0) < 0.01, (lid, l['a'], l['b'])
        def fn(b):
            b['startX'], b['startY'] = self.mil(*a1); b['endX'], b['endY'] = self.mil(*b1)
        self._edit(l['row'], fn); self._drop_tears(lid)
        self.log.append(f"改线 {l['net']} 层{l['layer']} ({a0[0]},{a0[1]})->({b0[0]},{b0[1]})  =>  ({a1[0]},{a1[1]})->({b1[0]},{b1[1]})")

    def remove(self, *ids):
        for i in ids:
            it = self.item[i]; self.lines[it['row']] = None; self._drop_tears(i)
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
                mx, my = self.mil(x, y)
                if k == 1: path.append('L')
                path += [mx, my]
            self._add('POLY', {"partitionId": "", "groupId": 0, "netName": "", "layerId": layer, "width": stroke_mil, "path": path,
                               "locked": False, "zIndex": -1, "polyType": "NORMAL"})
        self.labels.append((layer, s, text_box(s, cx, cy, h, rot), strokes, h, rot))

    # ---- 铺铜 ----
    def pours(self, drop=()):
        gone = set()
        for h, b, i in self.prj.pcb['rows']:
            if h['type'] == 'POUR' and b.get('name') in drop:
                self.lines[i] = None; gone.add(h['id']); self.log.append(f"删除重复铺铜区 {b.get('name')} (层{b['layerId']} {b['netName']})")
        for h, b, i in self.prj.pcb['rows']:
            if h['type'] == 'POURED':
                if json.loads(h['id'])[1] in gone: self.lines[i] = None
                else:
                    def fn(b): b['pourFill'] = []
                    self._edit(i, fn)
        self.log.append("清空铺铜填充数据: 导入后必须重建铺铜")

    def save(self, out):
        last = max(l['row'] for l in self.rb.lines)       # 新图元插在最后一条走线后面
        res = []
        for i, l in enumerate(self.lines):
            if l is not None: res.append(l)
            if i == last: res += self.new
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as zo:
            for it in self.prj.zip.infolist():
                zo.writestr(it, '\n'.join(res).encode('utf-8') if it.filename == self.prj.epru else self.prj.zip.read(it.filename))
        open(out, 'wb').write(buf.getvalue())


def apply_fixes(ed):
    """走线优化. 每一条的原因见 Hardware/布线检查报告.md"""
    # 1. 升压芯片U3的输入脚(5脚IN 4脚EN)原来靠一条20mm长的底层线从TP2供电, 身边的去耦电容C4反而没有直接连到它.
    #    改成: C4 -> U3.5 -> U3.4 在顶层直接相连(2.6mm), 删掉底层那条长线和3个打在焊盘上的过孔, 底层地平面恢复完整
    ed.add_line('+5V', TOP, (34.990, 11.430), (37.586, 11.430), 16)
    ed.add_line('+5V', TOP, (37.586, 11.430), (37.586, 10.480), 12)
    ed.remove('4b0e51bb68044917', '7098e47507dab3c3', 'ee51bab9e1fee759', 'c9f9e85b78608eb7', '0fcf24f57bc7c5ab',
              'b40337b2bd859045', '71999d78950b7da0', '263aa91c180883ed')
    # 2. 单片机U1底部的接地焊盘原来没有过孔, 只靠顶层细铜皮接地. 加3个过孔直通底层地(位置避开了底层的DBG_RX线)
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
    # 三个直插排针: 顶层紧挨着别的元件放不下字, 写在底层(镜像, 从背面看是正的), 字在引脚靠板内一侧
    for ref in ('H1', 'H2', 'H3'):
        inner = 1 if pads[ref][0]['x'] < rb.W / 2 else -1; gnd = 0
        for q in sorted(pads[ref], key=lambda q: int(q['num'])):
            s = NET_NAME[q['net']]
            if ref != 'H1' and s == 'GND':
                gnd += 1
                if gnd == 2: s = 'SHD'          # 传感器第3脚接屏蔽线(板上也是地)
            h = 1.0; W = text_size(s, h)
            put(s, q['x'] + inner * (q['w'] / 2 + 0.4 + W / 2), q['y'], h, BOT_SILK)
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
    src, out = sys.argv[1], sys.argv[2]; pv = sys.argv[3] if len(sys.argv) > 3 else None
    ed = Editor(src)
    apply_fixes(ed)
    ed.hide_designators()
    problems = add_labels(ed)
    ed.pours(drop=('POUR3', 'POUR4'))
    ed.save(out)
    print('\n'.join(ed.log)); print(f"加引脚丝印 {len(ed.labels)} 处: " + ' '.join(f"{s}" for _, s, *_ in ed.labels))
    for p in problems: print('!! 丝印冲突:', p)
    if pv: os.makedirs(pv, exist_ok=True); preview(ed, pv)
    # 验证: 重新读入输出文件, 模拟重建铺铜后检查连通 短路 间距 悬空线头
    rb = RV.Routed(out).build().simulate_pour()
    opens = rb.connectivity(); cl = rb.clearance(0.16); dang = rb.dangling()
    print(f"\n== 验证(模拟重建铺铜) ==\n未连通网络 {len(opens)} 个: {opens}")
    print(f"短路 {sum(1 for c in cl if c[0] <= 0)} 处; 间距小于0.16mm的 {len(cl)} 对:")
    for c in cl: print(f"  {c[0]:.3f} mm 层{c[1]} {c[2]} - {c[3]} @({c[4]},{c[5]})")
    print(f"悬空线头 {len(dang)} 处: {dang}")
    print('已写出', out)

if __name__ == '__main__':
    main()
