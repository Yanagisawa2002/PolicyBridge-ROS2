"""Render a repository's source-locked portfolio figure: Python + Matplotlib."""
from pathlib import Path
import hashlib
import json
import textwrap
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
C=json.loads((HERE/'figure.json').read_text(encoding='utf-8'))
for source in C['sources']:
    assert hashlib.sha256((ROOT/source['path']).read_bytes().replace(b'\r\n',b'\n')).hexdigest()==source['sha256'], source['path']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'text.color':'#172B4D',
 'axes.labelcolor':'#172B4D','xtick.color':'#42526E','ytick.color':'#42526E',
 'svg.fonttype':'none','svg.hashsalt':'portfolio-v1','figure.facecolor':'white'})
BLUE='#3572B0'; PALE='#E7EFF8'; GOLD='#C99A2E'; INK='#172B4D'; GRAY='#68778D'
fig=plt.figure(figsize=(12,8.6),dpi=160)
fig.text(.055,.95,C['title'],fontsize=23,weight='bold',va='top')
fig.text(.055,.895,C['subtitle'],fontsize=11.5,va='top',color=GRAY)
top=fig.add_axes([.055,.60,.89,.23]); top.set_xlim(0,100); top.set_ylim(0,100); top.axis('off')
top.text(0,98,C.get('diagram_title','HOW IT WORKS · schematic'),fontsize=10,weight='bold',color=GRAY)

def flow(labels,y=42):
    width=90/len(labels)-3
    for i,label in enumerate(labels):
        x=3+i*96/len(labels)
        top.add_patch(Rectangle((x,y-20),width,40,facecolor=PALE,edgecolor=BLUE,lw=1.2))
        top.text(x+width/2,y,label,ha='center',va='center',fontsize=11)
        if i<len(labels)-1:
            top.add_patch(FancyArrowPatch((x+width+1,y),(x+96/len(labels)-1,y),arrowstyle='->',mutation_scale=13,color=INK))

if C.get('diagram')=='layouts':
    for y,label,cells in [(72,'AoS',['x1','y1','z1','x2','y2','z2','x3','y3','z3','x4','y4','z4']),
                          (43,'SoA',['x1','x2','x3','x4','y1','y2','y3','y4','z1','z2','z3','z4']),
                          (14,'AoSoA (2)', ['x1','x2','y1','y2','z1','z2','x3','x4','y3','y4','z3','z4'])]:
        top.text(0,y,label,va='center',weight='bold',fontsize=11)
        for j,cell in enumerate(cells):
            x=18+j*6.5
            top.add_patch(Rectangle((x,y-10),6,20,facecolor=PALE if cell[0]=='x' else 'white',edgecolor=BLUE,lw=1))
            top.text(x+3,y,cell,ha='center',va='center',fontsize=10)
elif C.get('diagram')=='before_after':
    for y,label,items in [(67,'BEFORE',C['before']),(18,'AFTER',C['after'])]:
        top.text(0,y,label,fontsize=10,weight='bold',va='center')
        for i,item in enumerate(items):
            x=15+i*29
            top.add_patch(Rectangle((x,y-15),25,30,facecolor=PALE if label=='AFTER' else 'white',edgecolor=BLUE,lw=1))
            top.text(x+12.5,y,item,ha='center',va='center',fontsize=10.5)
            if i<2: top.annotate('',xy=(x+28,y),xytext=(x+25,y),arrowprops={'arrowstyle':'->','color':INK})
else: flow(C['flow'])

def style(ax,title,xlabel,maxx):
    ax.set_title(title,loc='left',fontsize=14,weight='bold',pad=14)
    ax.set_xlim(0,maxx); ax.set_xlabel(xlabel,fontsize=11)
    ax.spines[['top','right']].set_visible(False)
    ax.spines[['bottom','left']].set_color('#BDC7D4')
    ax.set_axisbelow(True); ax.grid(axis='x',color='#E8ECF1',lw=.7)

if C['kind']=='bars':
    count=len(C['panels'])
    for i,p in enumerate(C['panels']):
        ax=fig.add_axes([.21+i*.46/count*2,.205,.30 if count==2 else .70,.285]) if count==2 else fig.add_axes([.21,.205,.70,.285])
        style(ax,p['title'],p['unit'],p['max'])
        ys=list(range(len(p['values'])))
        ax.barh(ys,p['values'],height=.5,color=BLUE,edgecolor=INK,lw=.6)
        ax.set_yticks(ys,p['labels'],fontsize=11); ax.invert_yaxis()
        for y,v in zip(ys,p['values']): ax.text(v+p['max']*.018,y,p.get('format','{:.2f}').format(v),va='center',fontsize=11,weight='bold')
elif C['kind']=='interval':
    ax=fig.add_axes([.24,.205,.68,.285]); p=C['panel']
    ax.set_title(p['title'],loc='left',fontsize=14,weight='bold',pad=14)
    for i,row in enumerate(p['rows']):
        ax.errorbar(row['value'],i,xerr=[[row['value']-row['low']],[row['high']-row['value']]],fmt='o',color=BLUE,capsize=5,markersize=7)
        ax.text(row['high']+.012*(p['max']-p['min']),i,f"{row['value']:.2f}"+p['suffix'],va='center',fontsize=11,weight='bold')
    ax.set_yticks(range(len(p['rows'])),[r['label'] for r in p['rows']],fontsize=11)
    ax.invert_yaxis(); ax.set_xlim(p['min'],p['max']); ax.set_xlabel(p['unit'],fontsize=11)
    ax.spines[['top','right']].set_visible(False); ax.grid(axis='x',color='#E8ECF1'); ax.set_axisbelow(True)
    if 'reference' in p: ax.axvline(p['reference'],color=GRAY,ls='--',lw=1)
elif C['kind']=='grouped':
    ax=fig.add_axes([.20,.205,.71,.285]); p=C['panel']
    style(ax,p['title'],'Score (0–1)',1)
    for i,(label,vals) in enumerate(zip(p['labels'],p['values'])):
        for k,v in enumerate(vals):
            y=i+(k-.5)*.28
            ax.barh(y,v,height=.25,color=BLUE if k==0 else 'white',edgecolor=BLUE,hatch=None if k==0 else '///')
            ax.text(v+.015,y,f'{v:.3f}',va='center',fontsize=10)
    ax.set_yticks(range(3),p['labels']); ax.invert_yaxis()
    fig.text(.20,.535,'Solid: instance detection     Hatched: detection + correct pose',fontsize=11,color=GRAY)
else:
    ax=fig.add_axes([.07,.18,.86,.36]); ax.set_xlim(0,100); ax.set_ylim(0,100); ax.axis('off')
    lanes=[('Client',8),('Runtime',37),('Policy worker',66),('Command output',94)]
    for label,x in lanes:
        ax.text(x,100,label,ha='center',va='bottom',fontsize=11,weight='bold')
        ax.plot([x,x],[5,94],color='#BDC7D4',lw=1,ls='--')
    for y,a,b,label in [(85,37,66,'Start inference'),(64,8,37,'Cancel goal'),(43,37,94,'Send latest-state hold'),(22,66,37,'Late inference returns')]:
        ax.annotate('',xy=(b,y),xytext=(a,y),arrowprops={'arrowstyle':'->','color':BLUE,'lw':1.6})
        ax.text((a+b)/2,y+4,label,ha='center',va='bottom',fontsize=10.5)
    ax.text(37,5,'Discard late result; no new command',ha='left',fontsize=11,weight='bold')

fig.text(.055,.115,C['note'],fontsize=10.5,color=GRAY,va='top',linespacing=1.5)
fig.text(.055,.035,'Source: '+C['source_label'],fontsize=9,color=GRAY)
for ext in ('svg','png'):
    fig.savefig(HERE/('overview.'+ext),facecolor='white',metadata={'Creator':'Matplotlib / source-locked portfolio renderer','Date':None} if ext=='svg' else None)
    if ext=='svg':
        path=HERE/'overview.svg'
        path.write_text('\n'.join(line.rstrip() for line in path.read_text(encoding='utf-8').splitlines())+'\n',encoding='utf-8',newline='\n')
plt.close(fig)
print(C['title']+': SVG + PNG rendered')
