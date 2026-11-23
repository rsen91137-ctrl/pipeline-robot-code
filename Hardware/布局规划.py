# 测距转换板 50x50mm 布局规划: 生成按比例的布局图和坐标表, 并检查元件之间是否重叠
# 坐标原点在板子左上角, x向右, y向下, 单位mm; 每个元件给出中心坐标和画出来的宽高
from PIL import Image, ImageDraw, ImageFont
import csv
W=H=50.0
HOLES=[(3.5,3.5),(46.5,3.5),(3.5,46.5),(46.5,46.5)]; HOLE_KEEP=3.2
# 位号: (中心x, 中心y, 宽, 高, 分组, 朝向说明)
P={
'H1':(3.5,17.0,2.54,15.24,'conn','竖放 第1脚(5V)在上'),
'F1':(8.0,8.0,3.2,1.6,'in','横放'),
'D1':(11.5,11.0,2.6,5.0,'in','竖放 阴极朝上(接F1输出)'),
'D2':(15.2,8.0,3.8,1.6,'in','横放 阴极朝右'),
'C1':(19.5,12.5,7.6,6.6,'in','正极朝D2'),
'TP1':(21.5,6.5,3.4,1.8,'tp','+5V'),
'C10':(24.4,9.0,0.8,1.6,'ldo','竖放 贴U2的VIN脚'),
'U2':(27.0,9.0,3.0,2.8,'ldo','VIN/CE一侧朝左 VOUT一侧朝右'),
'C12':(29.6,9.0,0.8,1.6,'ldo','竖放 贴U2的VOUT脚'),
'TP3':(27.0,12.2,3.4,1.8,'tp','3V3'),
'C2':(33.2,11.0,1.6,3.2,'boost','竖放'),
'C4':(34.9,11.5,0.8,1.6,'boost','竖放 紧贴U3的IN脚'),
'U3':(37.5,11.5,3.0,2.8,'boost','IN/EN一侧朝左 SW/GND/FB一侧朝右'),
'L1':(37.5,7.0,5.0,5.0,'boost','一端朝左下接+5V 一端朝右下接SW'),
'D3':(42.3,10.5,3.8,1.6,'boost','横放 阳极朝左(SW) 阴极朝右'),
'C3':(45.4,11.8,1.6,3.2,'boost','竖放 上端接D3阴极 下端接地'),
'R9':(41.2,12.6,1.6,0.8,'boost','横放 贴U3的FB脚'),
'R1':(41.2,13.9,1.6,0.8,'boost','横放'),
'TP2':(38.0,15.4,3.4,1.8,'tp','+12V'),
'L3':(43.3,16.5,1.6,0.8,'spwr','横放'),
'C14':(43.3,18.3,2.0,1.25,'spwr','贴H2第1脚'),
'H2':(46.5,22.5,2.54,10.16,'conn','竖放 第1脚(12V)在上'),
'H3':(46.5,34.0,2.54,10.16,'conn','竖放 第1脚(12V)在上'),
'C15':(43.4,30.2,2.0,1.25,'spwr','贴H3第1脚'),
'L4':(43.4,32.0,1.6,0.8,'spwr','横放'),
'C17':(29.3,25.6,3.2,1.6,'ana','通道2 横放 最靠近U1'),
'R6':(32.0,25.6,0.8,1.6,'ana','通道2 竖放'),
'R4':(33.8,25.6,1.6,0.8,'ana','通道2 横放'),
'D5':(36.6,25.6,2.9,2.4,'ana','通道2'),
'TP6':(40.2,25.6,3.4,1.8,'tp','ADC2'),
'C16':(29.3,28.6,3.2,1.6,'ana','通道1 横放 最靠近U1'),
'R5':(32.0,28.6,0.8,1.6,'ana','通道1 竖放'),
'R3':(33.8,28.6,1.6,0.8,'ana','通道1 横放'),
'D4':(36.6,28.6,2.9,2.4,'ana','通道1'),
'TP5':(40.2,28.6,3.4,1.8,'tp','ADC1'),
'TP4':(40.2,31.6,3.4,1.8,'tp','GND'),
'U1':(24.0,31.0,5.0,5.0,'mcu','第6、7脚(PA0 PA1)朝右 第19、20脚(PA9 PA10)朝左'),
'C6':(20.2,28.6,1.6,0.8,'mcu','贴第17脚'),
'C13':(20.2,27.3,1.6,0.8,'mcu','紧挨C6'),
'C7':(28.0,31.4,1.6,0.8,'mcu','贴第5脚VDDA'),
'C11':(30.0,31.4,1.6,0.8,'mcu','紧挨C7'),
'L2':(32.0,31.4,1.6,0.8,'mcu','VDDA磁珠'),
'C5':(28.0,32.9,1.6,0.8,'mcu','贴第1脚'),
'C8':(28.0,34.5,1.6,0.8,'mcu','贴第4脚NRST'),
'R2':(25.3,35.3,0.8,1.6,'mcu','竖放 对着第31脚'),
'R7':(22.7,35.3,0.8,1.6,'mcu','竖放 对着第26脚'),
'LED1':(22.7,37.6,1.6,0.8,'mcu','横放'),
'D6':(7.6,17.0,2.4,2.9,'rs','贴H1第3、4脚'),
'D7':(7.6,22.1,2.4,2.9,'rs','贴H1第5、6脚'),
'R8':(10.2,22.1,0.8,1.6,'rs','竖放 不焊'),
'U4':(14.0,21.0,6.0,5.0,'rs','A/B/Z/Y一侧朝左(H1) VCC/RO/DI/GND一侧朝右(U1)'),
'C9':(14.0,17.3,1.6,0.8,'rs','贴U4的VCC脚'),
'CN1':(14.0,44.0,9.0,5.0,'conn','横放 贴下边缘'),
'SW1':(26.0,43.5,6.6,5.2,'conn','贴下边缘'),
}
COL={'conn':'#8a6d00','in':'#b03030','ldo':'#2d7d46','boost':'#c05a00','spwr':'#7a4fb0','ana':'#1565c0','mcu':'#00838f','rs':'#ad1457','tp':'#555555'}
ZONE={'in':'5V输入保护','ldo':'3.3V稳压','boost':'12V升压','spwr':'传感器供电','ana':'模拟前端','mcu':'单片机','rs':'RS422','conn':'接口','tp':'测试点'}
def box(k,m=0.0):
    x,y,w,h=P[k][:4]; return (x-w/2-m,y-h/2-m,x+w/2+m,y+h/2+m)
bad=[]; ks=list(P)
for i,a in enumerate(ks):
    A=box(a,0.15)
    if A[0]<0.8 or A[1]<0.8 or A[2]>W-0.8 or A[3]>H-0.8: bad.append((a,'太靠板边'))
    for hx,hy in HOLES:
        cx=min(max(hx,A[0]),A[2]); cy=min(max(hy,A[1]),A[3])
        if (cx-hx)**2+(cy-hy)**2<HOLE_KEEP**2: bad.append((a,'压到固定孔'))
    for b in ks[i+1:]:
        B=box(b,0.15)
        if A[0]<B[2] and B[0]<A[2] and A[1]<B[3] and B[1]<A[3]: bad.append((a,b))
print('冲突:',bad if bad else '无')
S=36; M=40
im=Image.new('RGB',(int(W*S)+2*M,int(H*S)+2*M+120),'white'); d=ImageDraw.Draw(im)
def font(sz):
    for p in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
        try: return ImageFont.truetype(p,sz)
        except Exception: pass
    return ImageFont.load_default()
f=font(15); fs=font(12); fb=font(20)
X=lambda v:M+v*S; Y=lambda v:M+v*S
for g in range(0,51,5):
    d.line([X(g),Y(0),X(g),Y(H)],fill='#e6e6e6'); d.line([X(0),Y(g),X(W),Y(g)],fill='#e6e6e6')
    d.text((X(g)-6,Y(0)-18),str(g),fill='#888',font=fs); d.text((X(0)-24,Y(g)-8),str(g),fill='#888',font=fs)
d.rounded_rectangle([X(0),Y(0),X(W),Y(H)],radius=2*S,outline='#7b1fa2',width=4)
for hx,hy in HOLES:
    d.ellipse([X(hx-HOLE_KEEP),Y(hy-HOLE_KEEP),X(hx+HOLE_KEEP),Y(hy+HOLE_KEEP)],outline='#bbb'); d.ellipse([X(hx-1.6),Y(hy-1.6),X(hx+1.6),Y(hy+1.6)],outline='#333',width=3)
for k,(x,y,w,h,g,_) in P.items():
    b=box(k); d.rectangle([X(b[0]),Y(b[1]),X(b[2]),Y(b[3])],outline=COL[g],fill=COL[g]+'' if False else None,width=3)
    d.rectangle([X(b[0])+2,Y(b[1])+2,X(b[2])-2,Y(b[3])-2],fill=tuple(int(COL[g][i:i+2],16)//4+191 for i in (1,3,5)))
    ff=f if min(w,h)*S>30 and max(w,h)*S>40 else fs
    tw=d.textlength(k,font=ff)
    if tw<=w*S-4 or w>=h: d.text((X(x)-tw/2,Y(y)-9),k,fill='black',font=ff)
    else:
        t=Image.new('RGBA',(int(tw)+4,20),(0,0,0,0)); ImageDraw.Draw(t).text((2,1),k,fill='black',font=ff); t=t.rotate(90,expand=True); im.paste(t,(int(X(x)-10),int(Y(y)-t.size[1]/2)),t)
# 引脚1标记与关键提示
d.text((X(0),Y(H)+14),'测距转换板布局规划 50 x 50 mm   坐标原点在左上角 单位 mm   网格 5 mm',fill='black',font=fb)
lx=X(0); ly=Y(H)+50
for g,name in ZONE.items():
    d.rectangle([lx,ly,lx+18,ly+18],outline=COL[g],width=3); d.text((lx+24,ly-1),name,fill='black',font=f); lx+=24+d.textlength(name,font=f)+22
    if lx>X(W)-120: lx=X(0); ly+=28
im.save('布局规划图.png')
with open('布局坐标表.csv','w',encoding='utf-8-sig',newline='') as fh:
    w=csv.writer(fh); w.writerow(['位号','所属模块','中心X(mm)','中心Y(mm)','宽','高','朝向说明'])
    for k,(x,y,ww,hh,g,note) in P.items(): w.writerow([k,ZONE[g],x,y,ww,hh,note])
print(len(P),'个元件; 图片',im.size)
