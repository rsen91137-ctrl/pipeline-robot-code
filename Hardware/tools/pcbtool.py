#!/usr/bin/env python3
# 测距转换板布局工具: 读取嘉立创EDA专业版工程(.epro2)里的真实封装(焊盘+丝印+本体外形)和网络,
# 对一份布局(layout.json: 位号 -> [x, y, 角度])做检查、打分、出图, 并能写回一份工程副本.
#
# 坐标约定(对外): 单位mm, 原点在板框左上角, x向右, y向下. 角度为逆时针(与嘉立创一致), 只用0/90/180/270.
# 文件内部: 单位mil, y向上.
#
#   python3 pcbtool.py info   工程.epro2                     列出每个封装的外形尺寸和各焊盘(0度时)的位置与网络
#   python3 pcbtool.py check  工程.epro2 layout.json [--png out.png] [--show U1,U3]   检查并打分
#   python3 pcbtool.py export 工程.epro2 layout.json 输出.epro2      写出改好位置和位号丝印的工程副本
#   python3 pcbtool.py dump   工程.epro2 layout.json                 把工程里当前的位置导出成layout.json
import zipfile, json, math, sys, io, itertools

MIL = 0.0254
BOARD_W, BOARD_H = 50.0, 50.0          # 目标板框
HOLE_INSET, HOLE_D, HOLE_PAD = 2.6, 3.0, 4.0   # 四角固定孔: 圆心离两边距离, 孔径, 焊环直径
GAP_MIN = 0.6      # 两个元件外形(焊盘+丝印+本体)之间的最小间隙, 小于它算违规
GAP_GOOD = 1.0     # 小于它算"偏挤", 越少越好
EDGE_MIN = 1.0     # 元件外形到板边的最小距离
HOLE_CLEAR = 0.8   # 元件外形到固定孔焊环的最小距离
FS_MIL = 40        # 位号丝印字高(mil), 约1.0mm
CHAR_W = 0.66      # 每个字符的宽度 = CHAR_W * 字高
TXT_GAP = 0.25     # 位号文字到任何元件外形/其他文字的最小间隙

def _parse(path):
    z = zipfile.ZipFile(path)
    name = [n for n in z.namelist() if n.endswith('.epru')][0]
    lines = z.read(name).decode('utf-8').split('\n')
    docs = []
    for i, l in enumerate(lines):
        if not l.strip():
            continue
        head, _, body = l.partition('||')
        h = json.loads(head); b = body.rstrip('|'); b = json.loads(b) if b else None
        if h.get('type') == 'DOCHEAD':
            docs.append({'head': b, 'rows': []})
        else:
            docs[-1]['rows'].append((h, b, i))
    return z, name, lines, docs

def _rot(x, y, deg):
    deg %= 360
    if deg == 0: return x, y
    if deg == 90: return -y, x
    if deg == 180: return -x, -y
    if deg == 270: return y, -x
    a = math.radians(deg); return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)

def _path_points(p):
    """把封装里的图形路径化成点集(只用于求外接矩形)"""
    pts = []
    if not p: return pts
    if isinstance(p[0], list):
        for q in p: pts += _path_points(q)
        return pts
    if p[0] == 'CIRCLE':
        cx, cy, r = p[1:4]; return [(cx - r, cy - r), (cx + r, cy + r)]
    if p[0] == 'R':
        x, y, w, h = p[1:5]; return [(x, y), (x + w, y - h)]
    mode = 'L'; nums = []; last = None
    def flush():
        nonlocal nums, last
        if mode == 'L':
            for i in range(0, len(nums) - 1, 2):
                last = (nums[i], nums[i + 1]); pts.append(last)
        else:
            for i in range(0, len(nums) - 2, 3):
                a, x, y = nums[i:i + 3]
                if last:
                    ch = math.hypot(x - last[0], y - last[1]); s = abs(math.sin(math.radians(a) / 2)) or 1e-6
                    r = ch / (2 * s); sag = r * (1 - math.cos(math.radians(abs(a)) / 2)) if abs(a) < 180 else 2 * r
                    for q in (last, (x, y)): pts.append((q[0] - sag, q[1] - sag)); pts.append((q[0] + sag, q[1] + sag))
                last = (x, y); pts.append(last)
        nums = []
    for t in p:
        if isinstance(t, str):
            flush(); mode = 'ARC' if t in ('ARC', 'CARC') else 'L'
        else:
            nums.append(t)
    flush(); return pts

class Project:
    def __init__(self, path):
        self.path = path
        self.zip, self.epru, self.lines, self.docs = _parse(path)
        self.fps = {}
        for d in self.docs:
            if d['head'].get('docType') != 'FOOTPRINT': continue
            pads = []; ext = []; silk = []
            for h, b, _ in d['rows']:
                if h['type'] == 'PAD':
                    dp = b['defaultPad']; w = dp.get('width', 0); hh = dp.get('height', w)
                    cw, ch = [abs(v) for v in _rot(w, hh, b.get('padAngle', 0) or 0)] if (b.get('padAngle', 0) or 0) % 180 else (w, hh)
                    pads.append({'num': b['num'], 'x': b['centerX'], 'y': b['centerY'], 'w': cw, 'h': ch})
                    ext += [(b['centerX'] - cw / 2, b['centerY'] - ch / 2), (b['centerX'] + cw / 2, b['centerY'] + ch / 2)]
                elif h['type'] in ('POLY', 'FILL') and b.get('layerId') in (3, 48):
                    pp = _path_points(b.get('path')); hw = (b.get('width') or 0) / 2
                    for x, y in pp: ext += [(x - hw, y - hw), (x + hw, y + hw)]
                    if b.get('layerId') == 3 and b.get('path') and not isinstance(b['path'][0], list) and b['path'][0] not in ('CIRCLE', 'R'):
                        silk.append([q for q in _path_points(b['path'])])
            xs = [e[0] for e in ext]; ys = [e[1] for e in ext]
            self.fps[d['head']['uuid']] = {'pads': pads, 'box': (min(xs), min(ys), max(xs), max(ys)), 'silk': silk}
        self.pcb = [d for d in self.docs if d['head'].get('docType') == 'PCB'][0]
        comps = {}; attrs = {}; padnet = {}; self.via_lines = []; self.outline_line = None
        for h, b, i in self.pcb['rows']:
            t = h['type']
            if t == 'COMPONENT':
                comps[h['id']] = {'id': h['id'], 'x': b['x'], 'y': b['y'], 'angle': b['angle'], 'line': i}
            elif t == 'ATTR' and b.get('parentId'):
                attrs.setdefault(b['parentId'], {})[b['key']] = (b, i)
            elif t == 'PAD_NET':
                cid, num = json.loads(h['id'])[1:3]; padnet[(cid, num)] = b['padNet']
            elif t == 'POLY' and b.get('layerId') == 11:
                self.outline = b['path']; self.outline_line = i
            elif t == 'VIA':
                self.via_lines.append((i, b))
        self.comps = {}
        for cid, c in comps.items():
            a = attrs.get(cid, {})
            c['ref'] = a['Designator'][0]['value']; c['fp'] = a['Footprint'][0]['value']
            c['des_line'] = a['Designator'][1]
            c['nets'] = {num: n for (ci, num), n in padnet.items() if ci == cid}
            self.comps[c['ref']] = c
        self.ox, self.oy = self.outline[1], self.outline[2]
        self.refs = sorted(self.comps, key=lambda s: (s.rstrip('0123456789'), int(''.join(ch for ch in s if ch.isdigit()) or 0)))

    def current_layout(self):
        return {r: [round((c['x'] - self.ox) * MIL, 3), round(-(c['y'] - self.oy) * MIL, 3), int(c['angle']) % 360] for r, c in self.comps.items()}

    def pads(self, ref, pose):
        c = self.comps[ref]; x, y, ang = pose; out = []
        for p in self.fps[c['fp']]['pads']:
            px, py = _rot(p['x'], p['y'], ang); w, h = (p['w'], p['h']) if ang % 180 == 0 else (p['h'], p['w'])
            out.append({'num': p['num'], 'x': x + px * MIL, 'y': y - py * MIL, 'w': w * MIL, 'h': h * MIL, 'net': c['nets'].get(p['num'], '')})
        return out

    def box(self, ref, pose):
        """元件外形(焊盘+丝印+本体)的外接矩形 (x0, y0, x1, y1)"""
        x, y, ang = pose; b = self.fps[self.comps[ref]['fp']]['box']
        cs = [_rot(px, py, ang) for px, py in ((b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3]))]
        xs = [x + q[0] * MIL for q in cs]; ys = [y - q[1] * MIL for q in cs]
        return (min(xs), min(ys), max(xs), max(ys))

    def silk(self, ref, pose):
        x, y, ang = pose
        return [[(x + _rot(px, py, ang)[0] * MIL, y - _rot(px, py, ang)[1] * MIL) for px, py in poly] for poly in self.fps[self.comps[ref]['fp']]['silk']]

HOLES = [(HOLE_INSET, HOLE_INSET), (BOARD_W - HOLE_INSET, HOLE_INSET), (HOLE_INSET, BOARD_H - HOLE_INSET), (BOARD_W - HOLE_INSET, BOARD_H - HOLE_INSET)]

def rect_gap(A, B):
    dx = max(A[0] - B[2], B[0] - A[2]); dy = max(A[1] - B[3], B[1] - A[3])
    if dx <= 0 and dy <= 0: return max(dx, dy)          # 重叠: 负值
    return math.hypot(max(dx, 0), max(dy, 0))

def rect_circle_gap(A, cx, cy, r):
    qx = min(max(cx, A[0]), A[2]); qy = min(max(cy, A[1]), A[3]); return math.hypot(qx - cx, qy - cy) - r

def mst(pts):
    if len(pts) < 2: return 0.0, []
    done = [pts[0]]; todo = pts[1:]; segs = []; tot = 0
    while todo:
        d, a, b = min(((math.hypot(a[0] - b[0], a[1] - b[1]), a, b) for a in done for b in todo), key=lambda t: t[0])
        segs.append((a, b)); tot += d; done.append(b); todo.remove(b)
    return tot, segs

def place_labels(prj, layout, boxes):
    """给每个元件的位号找一个不压任何元件、不互相重叠的位置(文字一律水平). 返回 {位号: (x0,y0,x1,y1)} 和放不下的位号列表"""
    fs = FS_MIL * MIL; g = TXT_GAP
    cand = {}
    for r in prj.refs:
        A = boxes[r]; w = CHAR_W * fs * len(r); h = fs; cx = (A[0] + A[2]) / 2; cy = (A[1] + A[3]) / 2; opts = []
        for dx, dy in ((cx - w / 2, A[1] - g - h), (cx - w / 2, A[3] + g), (A[2] + g, cy - h / 2), (A[0] - g - w, cy - h / 2),
                       (A[0], A[1] - g - h), (A[2] - w, A[1] - g - h), (A[0], A[3] + g), (A[2] - w, A[3] + g),
                       (A[2] + g, A[1]), (A[2] + g, A[3] - h), (A[0] - g - w, A[1]), (A[0] - g - w, A[3] - h)):
            T = (dx, dy, dx + w, dy + h)
            if T[0] < 0.6 or T[1] < 0.6 or T[2] > BOARD_W - 0.6 or T[3] > BOARD_H - 0.6: continue
            if any(rect_circle_gap(T, hx, hy, HOLE_PAD / 2) < 0.3 for hx, hy in HOLES): continue
            if any(rect_gap(T, boxes[o]) < g - 1e-6 for o in prj.refs if o != r): continue
            opts.append(T)
        cand[r] = opts
    placed = {}; failed = []
    for r in sorted(prj.refs, key=lambda k: len(cand[k])):
        for T in cand[r]:
            if all(rect_gap(T, U) >= 0.15 for U in placed.values()):
                placed[r] = T; break
        else:
            failed.append(r)
    return placed, failed

# 需要靠近的去耦/配套关系: (元件, 网络, 芯片, 引脚号, 上限mm 焊盘中心距)
NEAR = [('C5', '3V3', 'U1', '1', 3.0), ('C6', '3V3', 'U1', '17', 3.0), ('C7', 'VDDA', 'U1', '5', 3.5), ('C11', 'VDDA', 'U1', '5', 5.0),
        ('C8', 'NRST', 'U1', '4', 6.0), ('C9', '3V3', 'U4', '1', 3.0), ('C10', '+5V', 'U2', '1', 3.5), ('C12', '3V3', 'U2', '5', 3.5),
        ('C4', '+5V', 'U3', '5', 3.0), ('C2', '+5V', 'L1', '1', 5.0), ('C14', 'H21', 'H2', '1', 4.0), ('C15', 'H31', 'H3', '1', 4.0),
        ('R2', None, 'U1', '31', 4.0), ('R7', None, 'U1', '26', 4.0)]
ANALOG = ['R3', 'R4', 'R5', 'R6', 'C16', 'C17', 'D4', 'D5']

def evaluate(prj, layout):
    L = {r: tuple(layout[r]) for r in prj.refs}
    boxes = {r: prj.box(r, L[r]) for r in prj.refs}
    pads = {r: prj.pads(r, L[r]) for r in prj.refs}
    def pad(ref, num=None, net=None):
        return [p for p in pads[ref] if (num is None or p['num'] == num) and (net is None or p['net'] == net)]
    def pd(a, b): return math.hypot(a['x'] - b['x'], a['y'] - b['y'])
    viol = []; tight = []; mingap = 9e9
    for r in prj.refs:
        if L[r][2] % 90: viol.append(f'{r}: 角度必须是0/90/180/270')
        A = boxes[r]; e = min(A[0], A[1], BOARD_W - A[2], BOARD_H - A[3])
        if e < EDGE_MIN - 1e-6: viol.append(f'{r}: 离板边只有 {e:.2f}mm (要求>={EDGE_MIN})')
        for hx, hy in HOLES:
            gh = rect_circle_gap(A, hx, hy, HOLE_PAD / 2)
            if gh < HOLE_CLEAR - 1e-6: viol.append(f'{r}: 离固定孔只有 {gh:.2f}mm (要求>={HOLE_CLEAR})')
    for a, b in itertools.combinations(prj.refs, 2):
        g = rect_gap(boxes[a], boxes[b]); mingap = min(mingap, g)
        if g < GAP_MIN - 1e-6: viol.append(f'{a}-{b}: 间隙 {g:.2f}mm (要求>={GAP_MIN})')
        elif g < GAP_GOOD - 1e-6: tight.append((round(g, 2), a, b))
    labels, failed = place_labels(prj, L, boxes)
    for r in failed: viol.append(f'{r}: 位号丝印没有地方放(周围至少留出 {FS_MIL * MIL + 2 * TXT_GAP:.1f}mm 高的空隙)')
    nets = {}
    for r in prj.refs:
        for p in pads[r]:
            if p['net']: nets.setdefault(p['net'], []).append((p['x'], p['y']))
    netlen = {n: mst(pts)[0] for n, pts in nets.items() if n != 'GND'}
    rules = []
    def rule(name, val, limit, ok):
        rules.append((name, val, limit, ok))
    for c, net, ic, pin, lim in NEAR:
        P = pad(ic, pin)[0]; cs = pad(c, net=net or P['net'])
        d = min(pd(P, q) for q in cs) if cs else 99
        rule(f'{c} 到 {ic} 第{pin}脚', d, f'<= {lim}', d <= lim)
    sw = pad('U3', '1')[0]; gnd = pad('U3', '2')[0]; d3a = pad('D3', net='U3-SW')[0]; d3k = pad('D3', net='+12V')[0]
    c3p = pad('C3', net='+12V')[0]; c3g = pad('C3', net='GND')[0]; l1s = pad('L1', net='U3-SW')[0]
    loop = pd(sw, d3a) + pd(d3k, c3p) + pd(c3g, gnd) + pd(d3a, d3k) + pd(c3p, c3g)
    rule('升压开关回路周长 U3.SW-D3-C3-U3.GND', loop, '<= 16', loop <= 16)
    rule('U3-SW 网络长度', netlen.get('U3-SW', 0), '<= 8', netlen.get('U3-SW', 0) <= 8)
    rule('U3-FB 网络长度', netlen.get('U3-FB', 0), '<= 5', netlen.get('U3-FB', 0) <= 5)
    for n in ('ADC1', 'ADC2'):
        tp = 'TP5' if n == 'ADC1' else 'TP6'
        core = [(p['x'], p['y']) for r in prj.refs if r != tp for p in pads[r] if p['net'] == n]
        v = mst(core)[0]; rule(f'{n} 网络长度(不含测试点)', v, '<= 14', v <= 14)
    l1 = boxes['L1']; da = min(rect_gap(l1, boxes[a]) for a in ANALOG)
    rule('电感L1到模拟前端元件的最近距离', da, '>= 12', da >= 12)
    du = min(rect_gap(boxes['U3'], boxes[a]) for a in ANALOG); rule('U3到模拟前端元件的最近距离', du, '>= 12', du >= 12)
    for d, pins in (('D6', ('3', '4')), ('D7', ('5', '6'))):
        v = max(min(pd(q, hp) for q in pads[d] if q['net'] == hp['net']) for hp in pads['H1'] if hp['num'] in pins)
        rule(f'{d} 到 H1 对应引脚', v, '<= 7', v <= 7)
    for n in ('TX+', 'TX-', 'RX+', 'RX-', 'UART_TX', 'UART_RX'):
        rule(f'{n} 网络长度', netlen[n], '<= 16', netlen[n] <= 16)
    e = boxes['H1'][0]; rule('H1 贴左边缘(外形左边到板边)', e, '<= 2.5', e <= 2.5)
    for h in ('H2', 'H3'):
        e = BOARD_W - boxes[h][2]; rule(f'{h} 贴右边缘', e, '<= 2.5', e <= 2.5)
    for h in ('CN1', 'SW1'):
        A = boxes[h]; e = min(A[0], A[1], BOARD_W - A[2], BOARD_H - A[3]); rule(f'{h} 靠板边', e, '<= 4', e <= 4)
    for t, n in (('TP5', 'ADC1'), ('TP6', 'ADC2')):
        P = pads[t][0]; v = min(pd(P, q) for r in ANALOG for q in pads[r] if q['net'] == n); rule(f'{t} 到 {n} 最近焊盘', v, '<= 6', v <= 6)
    total = sum(netlen.values())
    return {'layout': L, 'boxes': boxes, 'pads': pads, 'labels': labels, 'viol': viol, 'tight': sorted(tight), 'mingap': mingap,
            'rules': rules, 'netlen': netlen, 'total': total, 'nets': nets}

def report(prj, ev, show=()):
    out = []
    out.append(f"板框 {BOARD_W:.0f} x {BOARD_H:.0f} mm   元件 {len(prj.refs)} 个")
    out.append(f"硬性违规 {len(ev['viol'])} 处:")
    out += ['  ' + v for v in ev['viol']] or ['  (无)']
    out.append(f"偏挤的元件对(间隙 {GAP_MIN}~{GAP_GOOD}mm) {len(ev['tight'])} 对: " + ', '.join(f'{a}-{b} {g}' for g, a, b in ev['tight'][:40]))
    out.append(f"最小间隙 {ev['mingap']:.2f} mm")
    bad = [r for r in ev['rules'] if not r[3]]
    out.append(f"设计规则: {len(ev['rules']) - len(bad)}/{len(ev['rules'])} 项达标")
    for name, val, lim, ok in ev['rules']:
        out.append(f"  [{'达标' if ok else '未达'}] {name}: {val:.1f} mm (要求 {lim})")
    out.append(f"飞线总长(不含GND): {ev['total']:.0f} mm")
    score = len(ev['viol']) * 1000 + len(bad) * 100 + len(ev['tight']) * 3 + ev['total'] * 0.2
    out.append(f"综合评分(越低越好): {score:.1f}   = 违规x1000 + 未达标x100 + 偏挤对数x3 + 飞线总长x0.2")
    for r in show:
        if r in prj.comps:
            A = ev['boxes'][r]
            out.append(f"{r} 外形 x {A[0]:.2f}~{A[2]:.2f}  y {A[1]:.2f}~{A[3]:.2f}  焊盘: " + '  '.join(f"{p['num']}:{p['net'] or '-'}({p['x']:.2f},{p['y']:.2f})" for p in ev['pads'][r]))
    return '\n'.join(out), score

def draw(prj, ev, out, title=''):
    from PIL import Image, ImageDraw, ImageFont
    S = 44; M = 46
    im = Image.new('RGB', (int(BOARD_W * S) + 2 * M, int(BOARD_H * S) + 2 * M + 34), 'white'); d = ImageDraw.Draw(im)
    def font(sz):
        for p in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
            try: return ImageFont.truetype(p, sz)
            except Exception: pass
        return ImageFont.load_default()
    fs = font(11); ft = font(17)
    X = lambda v: M + v * S
    for g in range(0, int(BOARD_W) + 1, 5):
        d.line([X(g), X(0), X(g), X(BOARD_H)], fill='#eeeeee'); d.text((X(g) - 5, X(0) - 18), str(g), fill='#999', font=fs)
        d.line([X(0), X(g), X(BOARD_W), X(g)], fill='#eeeeee'); d.text((X(0) - 24, X(g) - 7), str(g), fill='#999', font=fs)
    d.rounded_rectangle([X(0), X(0), X(BOARD_W), X(BOARD_H)], radius=3 * S, outline='#7b1fa2', width=4)
    for hx, hy in HOLES:
        d.ellipse([X(hx - HOLE_PAD / 2), X(hy - HOLE_PAD / 2), X(hx + HOLE_PAD / 2), X(hy + HOLE_PAD / 2)], fill='#dddddd', outline='#888')
        d.ellipse([X(hx - HOLE_D / 2), X(hy - HOLE_D / 2), X(hx + HOLE_D / 2), X(hy + HOLE_D / 2)], fill='white', outline='#222', width=2)
    hot = {'ADC1': '#1565c0', 'ADC2': '#1565c0', 'U3-SW': '#e65100', 'U3-FB': '#e65100'}
    for n, pts in ev['nets'].items():
        if n == 'GND': continue
        for a, b in mst(pts)[1]:
            d.line([X(a[0]), X(a[1]), X(b[0]), X(b[1])], fill=hot.get(n, '#a8b4ff'), width=3 if n in hot else 1)
    for r in prj.refs:
        A = ev['boxes'][r]
        d.rectangle([X(A[0]), X(A[1]), X(A[2]), X(A[3])], outline='#bdbdbd')
        for poly in prj.silk(r, ev['layout'][r]):
            if len(poly) > 1: d.line([(X(x), X(y)) for x, y in poly], fill='#c99700', width=2)
        for p in ev['pads'][r]:
            d.rectangle([X(p['x'] - p['w'] / 2), X(p['y'] - p['h'] / 2), X(p['x'] + p['w'] / 2), X(p['y'] + p['h'] / 2)], fill='#00897b' if p['num'] == '1' else '#d32f2f')
    fl = font(int(FS_MIL * MIL * S * 0.95))
    for r, T in ev['labels'].items():
        d.text((X(T[0]), X(T[1]) - 3), r, fill='black', font=fl)
    for r in prj.refs:
        if r not in ev['labels']:
            A = ev['boxes'][r]; d.text((X(A[0]), X(A[1])), r + '?', fill='#ff00ff', font=fl)
    d.text((X(0), X(BOARD_H) + 10), f"{title}  违规{len(ev['viol'])} 偏挤{len(ev['tight'])}对 最小间隙{ev['mingap']:.2f}mm 飞线{ev['total']:.0f}mm  绿焊盘=第1脚 蓝粗线=ADC 橙粗线=升压SW/FB", fill='black', font=ft)
    im.save(out)

def export(prj, ev, out):
    lines = list(prj.lines)
    def edit(i, fn):
        head, _, body = lines[i].partition('||'); tail = '|' if body.endswith('|') else ''
        b = json.loads(body.rstrip('|')); fn(b)
        lines[i] = head + '||' + json.dumps(b, ensure_ascii=False, separators=(',', ':')) + tail
    fs = FS_MIL * MIL
    for r in prj.refs:
        c = prj.comps[r]; x, y, ang = ev['layout'][r]
        def fc(b, x=x, y=y, ang=ang): b['x'] = round(prj.ox + x / MIL, 4); b['y'] = round(prj.oy - y / MIL, 4); b['angle'] = int(ang)
        edit(c['line'], fc)
        T = ev['labels'].get(r)
        if T is None:
            A = ev['boxes'][r]; T = (A[0], A[1] - fs - TXT_GAP, 0, A[1] - TXT_GAP)
        def fa(b, T=T):
            b['x'] = round(prj.ox + T[0] / MIL, 4); b['y'] = round(prj.oy - (T[1] + fs) / MIL, 4)
            b['angle'] = 0; b['fontSize'] = FS_MIL; b['strokeWidth'] = 6; b['origin'] = 'LEFT_BOTTOM'; b['valueVisible'] = True
        edit(c['des_line'], fa)
    def fo(b): b['path'][3] = round(BOARD_W / MIL, 4); b['path'][4] = round(BOARD_H / MIL, 4)
    edit(prj.outline_line, fo)
    vias = sorted(prj.via_lines, key=lambda t: (-t[1]['centerY'], t[1]['centerX']))   # 左上 右上 左下 右下
    for (i, b0), (hx, hy) in zip(vias, HOLES):
        def fv(b, hx=hx, hy=hy): b['centerX'] = round(prj.ox + hx / MIL, 4); b['centerY'] = round(prj.oy - hy / MIL, 4)
        edit(i, fv)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as zo:
        for it in prj.zip.infolist():
            zo.writestr(it, '\n'.join(lines).encode('utf-8') if it.filename == prj.epru else prj.zip.read(it.filename))
    open(out, 'wb').write(buf.getvalue())

def main():
    cmd = sys.argv[1]; prj = Project(sys.argv[2])
    if cmd == 'info':
        for r in prj.refs:
            A = prj.box(r, (0, 0, 0)); ps = prj.pads(r, (0, 0, 0))
            print(f"{r:5s} 外形 {A[2] - A[0]:.2f} x {A[3] - A[1]:.2f} mm (0度时 x {A[0]:.2f}~{A[2]:.2f}, y {A[1]:.2f}~{A[3]:.2f})  焊盘(0度, 相对元件原点): " + '  '.join(f"{p['num']}:{p['net'] or '-'}({p['x']:.2f},{p['y']:.2f})" for p in ps if r != 'U1' or p['net']))
        return
    if cmd == 'dump':
        json.dump(prj.current_layout(), open(sys.argv[3], 'w'), ensure_ascii=False, indent=0); return
    layout = json.load(open(sys.argv[3])); cur = prj.current_layout()
    miss = [r for r in prj.refs if r not in layout]
    if miss: print('layout里缺少元件(将按原位置处理):', miss)
    layout = {r: layout.get(r, cur[r]) for r in prj.refs}
    ev = evaluate(prj, layout)
    if cmd == 'check':
        show = []; png = None
        for i, a in enumerate(sys.argv):
            if a == '--show': show = sys.argv[i + 1].split(',')
            if a == '--png': png = sys.argv[i + 1]
        txt, score = report(prj, ev, show); print(txt)
        if png: draw(prj, ev, png, title=sys.argv[3].split('/')[-1])
    elif cmd == 'export':
        txt, score = report(prj, ev); print(txt.split('\n')[1]); export(prj, ev, sys.argv[4]); print('已写出', sys.argv[4])

if __name__ == '__main__':
    main()
