# 读取嘉立创EDA专业版工程(.epro2), 提取PCB上每个元件的封装焊盘、位置和网络,
# 用于按真实封装检查布局: 画图、算飞线总长、查重叠; 也可以把新的元件位置写回一份工程副本
# 用法见文件末尾. 坐标单位: 文件内部是mil, y向上为正; 本脚本对外一律用mm, 原点在板框左上角, y向下为正
import zipfile, json, math, sys, io
MIL = 0.0254

def parse(path):
    z = zipfile.ZipFile(path)
    name = [n for n in z.namelist() if n.endswith('.epru')][0]
    lines = z.read(name).decode('utf-8').split('\n')
    docs = []
    for i, l in enumerate(lines):
        if not l.strip():
            continue
        head, _, body = l.partition('||')
        h = json.loads(head)
        b = body.rstrip('|')
        b = json.loads(b) if b else None
        if h.get('type') == 'DOCHEAD':
            docs.append({'head': b, 'rows': []})
        else:
            docs[-1]['rows'].append((h, b, i))
    return z, name, lines, docs

def rot(x, y, deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c

class Board:
    def __init__(self, path):
        self.path = path
        self.zip, self.epru, self.lines, self.docs = parse(path)
        fps = {}
        for d in self.docs:
            if d['head'].get('docType') == 'FOOTPRINT':
                pads = []; shape = []
                for h, b, _ in d['rows']:
                    if h['type'] == 'PAD':
                        dp = b['defaultPad']
                        pads.append({'num': b['num'], 'x': b['centerX'], 'y': b['centerY'], 'w': dp.get('width', 0), 'h': dp.get('height', dp.get('width', 0)), 'a': b.get('padAngle', 0), 'hole': b.get('hole'), 'layer': b['layerId']})
                    elif h['type'] in ('POLY', 'FILL') and b.get('layerId') in (3, 48):
                        shape.append(b.get('path'))
                fps[d['head']['uuid']] = {'pads': pads, 'shape': shape}
        self.fps = fps
        pcb = [d for d in self.docs if d['head'].get('docType') == 'PCB'][0]
        self.pcb = pcb
        comps = {}; attrs = {}; padnet = {}
        self.holes = []
        for h, b, i in pcb['rows']:
            t = h['type']
            if t == 'COMPONENT':
                comps[h['id']] = {'id': h['id'], 'x': b['x'], 'y': b['y'], 'angle': b['angle'], 'layer': b['layerId'], 'line': i}
            elif t == 'ATTR' and b.get('parentId'):
                attrs.setdefault(b['parentId'], {})[b['key']] = (b, i)
            elif t == 'PAD_NET':
                cid, num = json.loads(h['id'])[1:3]
                padnet[(cid, num)] = b['padNet']
            elif t == 'POLY' and b.get('layerId') == 11:
                self.outline = b['path']
            elif t == 'VIA':
                self.holes.append((b['centerX'] * MIL, -b['centerY'] * MIL, b['holeDiameter'] * MIL, b['viaDiameter'] * MIL))
        self.comps = {}
        for cid, c in comps.items():
            a = attrs.get(cid, {})
            c['ref'] = a['Designator'][0]['value']
            c['fp'] = a['Footprint'][0]['value']
            c['attr_lines'] = {k: v[1] for k, v in a.items()}
            c['nets'] = {num: n for (ci, num), n in padnet.items() if ci == cid}
            self.comps[c['ref']] = c
        p = self.outline  # ["R", x, y, w, h, rot, radius]
        self.ox, self.oy = p[1], p[2]
        self.W, self.H = p[3] * MIL, p[4] * MIL

    # 元件当前位置(mm, 左上原点, y向下) 和角度
    def pose(self, ref):
        c = self.comps[ref]
        return (c['x'] - self.ox) * MIL, -(c['y'] - self.oy) * MIL, c['angle'] % 360

    # 给定位置和角度, 算出每个焊盘的中心、外接矩形和网络
    def pads(self, ref, pose=None):
        c = self.comps[ref]
        x, y, ang = pose if pose else self.pose(ref)
        out = []
        for p in self.fps[c['fp']]['pads']:
            px, py = rot(p['x'], p['y'], ang)
            hw, hh = p['w'] / 2, p['h'] / 2
            cs = [rot(dx, dy, ang + p['a']) for dx, dy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))]
            w = (max(q[0] for q in cs) - min(q[0] for q in cs)) * MIL; h = (max(q[1] for q in cs) - min(q[1] for q in cs)) * MIL
            out.append({'num': p['num'], 'x': x + px * MIL, 'y': y - py * MIL, 'w': w, 'h': h, 'net': c['nets'].get(p['num'], ''), 'hole': p['hole']})
        return out

    def bbox(self, ref, pose=None):
        ps = self.pads(ref, pose)
        return (min(p['x'] - p['w'] / 2 for p in ps), min(p['y'] - p['h'] / 2 for p in ps), max(p['x'] + p['w'] / 2 for p in ps), max(p['y'] + p['h'] / 2 for p in ps))

    def nets(self, poses=None):
        nets = {}
        for ref in self.comps:
            for p in self.pads(ref, (poses or {}).get(ref)):
                if p['net']:
                    nets.setdefault(p['net'], []).append((p['x'], p['y'], ref, p['num']))
        return nets

    # 每个网络的最小生成树: 近似飞线
    def ratsnest(self, poses=None, skip=('GND',)):
        segs = []; total = 0
        for n, pts in self.nets(poses).items():
            if n in skip or len(pts) < 2:
                continue
            done = [pts[0]]; todo = pts[1:]
            while todo:
                d, a, b = min(((math.hypot(a[0] - b[0], a[1] - b[1]), a, b) for a in done for b in todo), key=lambda t: t[0])
                segs.append((n, a, b, d)); total += d; done.append(b); todo.remove(b)
        return segs, total

    def overlaps(self, poses=None, gap=0.2):
        refs = list(self.comps); bb = {r: self.bbox(r, (poses or {}).get(r)) for r in refs}; out = []
        for i, a in enumerate(refs):
            A = bb[a]
            if A[0] < 0.5 or A[1] < 0.5 or A[2] > self.W - 0.5 or A[3] > self.H - 0.5:
                out.append((a, '板边'))
            for hx, hy, hd, vd in self.holes:
                cx = min(max(hx, A[0]), A[2]); cy = min(max(hy, A[1]), A[3])
                if math.hypot(cx - hx, cy - hy) < vd / 2 + 0.5:
                    out.append((a, '固定孔'))
            for b in refs[i + 1:]:
                B = bb[b]
                if A[0] - gap < B[2] and B[0] - gap < A[2] and A[1] - gap < B[3] and B[1] - gap < A[3]:
                    out.append((a, b))
        return out

    def draw(self, out, poses=None, H=None, scale=40, title=''):
        from PIL import Image, ImageDraw, ImageFont
        Hh = H or self.H; S = scale; M = 50
        im = Image.new('RGB', (int(self.W * S) + 2 * M, int(Hh * S) + 2 * M + 40), 'white'); d = ImageDraw.Draw(im)
        def font(sz):
            for p in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
                try: return ImageFont.truetype(p, sz)
                except Exception: pass
            return ImageFont.load_default()
        f = font(16); fs = font(11)
        X = lambda v: M + v * S
        for g in range(0, int(self.W) + 1, 5):
            d.line([X(g), X(0), X(g), X(Hh)], fill='#eeeeee'); d.text((X(g) - 6, X(0) - 20), str(g), fill='#999', font=fs)
        for g in range(0, int(Hh) + 1, 5):
            d.line([X(0), X(g), X(self.W), X(g)], fill='#eeeeee'); d.text((X(0) - 26, X(g) - 7), str(g), fill='#999', font=fs)
        d.rounded_rectangle([X(0), X(0), X(self.W), X(Hh)], radius=3 * S, outline='#7b1fa2', width=4)
        for hx, hy, hd, vd in self.holes:
            if hy > Hh: hy = Hh - (self.H - hy)
            d.ellipse([X(hx - vd / 2), X(hy - vd / 2), X(hx + vd / 2), X(hy + vd / 2)], outline='#888', width=2); d.ellipse([X(hx - hd / 2), X(hy - hd / 2), X(hx + hd / 2), X(hy + hd / 2)], outline='#222', width=3)
        segs, total = self.ratsnest(poses)
        for n, a, b, dist in segs:
            col = {'ADC1': '#1565c0', 'ADC2': '#1565c0', 'U3-SW': '#e65100', 'U3-FB': '#e65100'}.get(n, '#9aa7ff')
            d.line([X(a[0]), X(a[1]), X(b[0]), X(b[1])], fill=col, width=1 if col == '#9aa7ff' else 3)
        for ref in self.comps:
            A = self.bbox(ref, (poses or {}).get(ref))
            d.rectangle([X(A[0]) - 3, X(A[1]) - 3, X(A[2]) + 3, X(A[3]) + 3], outline='#c9a400', width=1)
            for p in self.pads(ref, (poses or {}).get(ref)):
                col = '#d32f2f' if p['num'] != '1' else '#00897b'
                d.rectangle([X(p['x'] - p['w'] / 2), X(p['y'] - p['h'] / 2), X(p['x'] + p['w'] / 2), X(p['y'] + p['h'] / 2)], fill=col)
            tx = d.textlength(ref, font=f)
            d.text((X((A[0] + A[2]) / 2) - tx / 2, X(A[1]) - 22), ref, fill='black', font=f)
        d.text((X(0), X(Hh) + 14), f'{title}   飞线总长(不含GND) {total:.0f} mm   绿色焊盘=第1脚   蓝粗线=ADC 橙粗线=升压SW/FB', fill='black', font=f)
        im.save(out); return total

    # 把新位置写回一份工程副本: poses = {位号: (x_mm, y_mm, 角度)}; 同时可改板框高度并移动下方两个固定孔
    def save(self, out, poses, H=None):
        lines = list(self.lines)
        def edit(i, fn):
            head, _, body = lines[i].partition('||'); tail = '|' if body.endswith('|') else ''
            b = json.loads(body.rstrip('|')); fn(b)
            lines[i] = head + '||' + json.dumps(b, ensure_ascii=False, separators=(',', ':')) + tail
        for ref, (x, y, ang) in poses.items():
            c = self.comps[ref]; nx = self.ox + x / MIL; ny = self.oy - y / MIL
            dx0, dy0, a0 = c['x'], c['y'], c['angle']
            def fc(b): b['x'] = round(nx, 4); b['y'] = round(ny, 4); b['angle'] = ang
            edit(c['line'], fc)
            for k, i in c['attr_lines'].items():
                def fa(b):
                    if b.get('x') is None: return
                    rx, ry = rot(b['x'] - dx0, b['y'] - dy0, ang - a0); b['x'] = round(nx + rx, 4); b['y'] = round(ny + ry, 4)
                edit(i, fa)
        if H:
            for h, b, i in self.pcb['rows']:
                if h['type'] == 'POLY' and b.get('layerId') == 11:
                    edit(i, lambda bb: bb['path'].__setitem__(4, round(H / MIL, 4)))
                elif h['type'] == 'VIA' and -b['centerY'] * MIL > self.H / 2:
                    edit(i, lambda bb: bb.__setitem__('centerY', round(bb['centerY'] + (self.H - H) / MIL, 4)))
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as zo:
            for it in self.zip.infolist():
                data = '\n'.join(lines).encode('utf-8') if it.filename == self.epru else self.zip.read(it.filename)
                zo.writestr(it, data)
        open(out, 'wb').write(buf.getvalue())

if __name__ == '__main__':
    b = Board(sys.argv[1])
    print(f'板框 {b.W:.1f} x {b.H:.1f} mm, 元件 {len(b.comps)} 个, 固定孔 {len(b.holes)} 个')
    for r in sorted(b.comps, key=lambda s: (s.rstrip('0123456789'), int(''.join(ch for ch in s if ch.isdigit()) or 0))):
        x, y, a = b.pose(r); print(f'{r:5s} x={x:6.2f} y={y:6.2f} 角度={a:5.0f}  ' + ' '.join(f"{p['num']}:{p['net']}" for p in b.pads(r))[:150])
    if len(sys.argv) > 2:
        print('飞线总长', round(b.draw(sys.argv[2], title='当前布局')), 'mm'); print('间距问题:', b.overlaps())
