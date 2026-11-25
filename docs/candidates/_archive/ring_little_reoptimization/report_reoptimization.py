from pathlib import Path
import base64,csv,html,json,shutil,sys,platform,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle,Polygon
from reoptimize_fingers import ROOT,OUT,load,m

font=Path('C:/Windows/Fonts/msyh.ttc')
font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name();plt.rcParams['axes.unicode_minus']=False
names={'ring':'四指 · 无名指 · 短基座','little':'五指 · 小指 · 长基座'}
summary=json.loads((OUT/'summary.json').read_text(encoding='utf-8'))

def capsule(ax,a,b,r,color='#d49b6c',alpha=.18):
    a,b=np.array(a),np.array(b);n=np.array([-(b-a)[1],(b-a)[0]])/np.linalg.norm(b-a)*r
    ax.add_patch(Polygon([a+n,b+n,b-n,a-n],color=color,alpha=alpha))
    for pt in [a,b]:ax.add_patch(Circle(pt,r,color=color,alpha=alpha))

fig,axes=plt.subplots(2,3,figsize=(15,9),constrained_layout=True)
for row,entry in enumerate(summary):
    finger=entry['finger'];p,base=load(finger);o=p.objectives[0]
    sweep=json.loads((OUT/finger/'exact_sweep.json').read_text(encoding='utf-8'))['records']
    selected=[min(sweep,key=lambda r:abs(r['curl_s']-s)) for s in [0,.5,1]]
    for ax,record in zip(axes[row],selected):
        pos=record['points'];s=record['curl_s'];r4=m._hand_distal_contact_progress(p,o,s)
        for _,a,b,r in m._hand_capsules(p,o,s):capsule(ax,a,b,r)
        for nodes,color in [('adea','#7b8797'),('ab','#2479b4'),('bc','#2479b4'),('cdgc','#159a8b'),('ef','#dc8a22'),('fghf','#937292')]:
            ax.plot([pos[k][0] for k in nodes],[pos[k][1] for k in nodes],'-o',lw=1.7,ms=3,color=color)
        ax.plot([pos['h'][0],r4[0]],[pos['h'][1],r4[1]],'-',color='#762baa',lw=3)
        ax.scatter(*r4,s=25,marker='s',color='#762baa')
        offsets={'a':(-8,5),'b':(3,4),'c':(3,-9),'d':(-9,-10),'e':(3,5),'f':(3,-10),'g':(3,5),'h':(3,5)}
        for k,offset in offsets.items():ax.annotate(k.upper(),pos[k],xytext=offset,textcoords='offset points',fontsize=8)
        ax.annotate('R4',r4,xytext=(4,-10),textcoords='offset points',fontsize=8)
        ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlim(-90,120);ax.set_ylim(-110,85);ax.set_xlabel('x / mm');ax.set_ylabel('y / mm')
        ax.set_title(f'{names[finger]}\n弯曲 {s:.0%} · 此姿态间隙 {record["minimum_clearance_mm"]:.2f} mm',fontsize=10)
fig.suptitle('重新优化后的运动姿态：固定基座，联合调整活动尺寸与起始角度',fontsize=17)
fig.supxlabel('紫色为 H–R4 输出杆；显示二维手指碰撞模型。FGH 为刚体上的孔距，不等于三根独立杆。',fontsize=10)
fig.savefig(OUT/'optimized_motion.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(12,7),constrained_layout=True)
for ax,entry in zip(axes,summary):
    keys=[k for k in entry['lengths_before'] if k not in ['L_ad','L_ae','L_de']]
    before=np.array([entry['lengths_before'][k] for k in keys]);after=np.array([entry['lengths_after'][k] for k in keys]);y=np.arange(len(keys))
    ax.barh(y-.18,before,height=.34,color='#bac4d1',label='上次优化保留值')
    ax.barh(y+.18,after,height=.34,color='#2d8b80',label='本次通过复查的值')
    for j,(b,a) in enumerate(zip(before,after)):ax.text(max(a,b)+1,j,f'{b:.2f} → {a:.2f}',va='center',fontsize=9)
    ax.set_yticks(y,[k.removeprefix('L_').upper().replace('TIP_ROD','H–R4') for k in keys]);ax.invert_yaxis();ax.set_xlim(0,max(before.max(),after.max())+35);ax.set_xlabel('孔距 / 杆长（mm）');ax.set_title(names[entry['finger']]);ax.legend(loc='lower right',fontsize=8);ax.grid(axis='x',alpha=.15)
fig.suptitle('尺寸变化：避碰会使部分尺寸增加，不能只看一根杆是否缩短',fontsize=15)
fig.savefig(OUT/'length_comparison.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
for ax,entry in zip(axes,summary):
    finger=entry['finger'];rs=json.loads((OUT/finger/'exact_sweep.json').read_text(encoding='utf-8'))['records']
    ax.plot([r['curl_s']*100 for r in rs],[r['minimum_clearance_mm'] for r in rs],color='#267d74');ax.axhline(0,color='#c33',ls='--');ax.axhline(1,color='#999',ls=':',label='软目标 1 mm');ax.set_title(names[finger]);ax.set_xlabel('弯曲进度 / %');ax.set_ylabel('最小模型间隙 / mm');ax.grid(alpha=.2);ax.legend()
fig.savefig(OUT/'clearance_sweep.png',dpi=140);plt.close(fig)

old=json.loads((ROOT/'outputs/finger_optimization/previous_glove_lengths.json').read_text(encoding='utf-8'))['fingers']
sections=[];metricrows=[];allrows=[]
for entry in summary:
    finger=entry['finger'];doc=json.loads((OUT/finger/'result.json').read_text(encoding='utf-8'));v=entry['after'];ev=doc['selected']['evaluation']
    metricrows.append(f'<tr><td>{names[finger]}</td><td>{entry["baseline"]["maximum_rod_closure_error_mm"]:.3f} → {ev["maximum_rod_closure_error_mm"]:.5f} mm</td><td>{entry["baseline"]["minimum_clearance_mm"]:.3f} → {v["minimum_clearance_mm"]:.3f} mm</td><td>{v["phase_deg"]:+.3f}°</td><td>{v["stroke_deg"]:.3f}°</td></tr>')
    rows=[]
    for k,before in entry['lengths_before'].items():
        after=entry['lengths_after'][k];oldval=old[finger].get(k);fixed=k in ['L_ad','L_ae','L_de']
        label=k.removeprefix('L_').upper().replace('TIP_ROD','H–R4');oldtext=f'{oldval:.3f}' if oldval is not None else '未提供'
        rows.append(f'<tr><td>{label}{"（固定）" if fixed else ""}</td><td>{oldtext}</td><td>{before:.3f}</td><td>{after:.3f}</td><td>{after-before:+.3f}</td></tr>')
        allrows.append(dict(finger=finger,dimension=k,old_repository_mm=oldval,previous_optimization_mm=before,new_mm=after,change_mm=after-before,fixed=fixed))
    sections.append(f'<h2>{names[finger]}：完整尺寸 / mm</h2><table><tr><th>尺寸</th><th>旧仓库模型</th><th>上次优化保留值</th><th>本次</th><th>本次减上次</th></tr>{"".join(rows)}</table>')
    # Explicitly describes phase convention; no unmodified upstream export may silently drop it.
    params=dict(finger=finger,status='2d_sampled_feasible_candidate_not_cad_release',lengths_mm=entry['lengths_after'],mount_tilt_deg=doc['mount_tilt_deg'],dorsal_clearance_mm=doc['dorsal_clearance_mm'],phase_deg=v['phase_deg'],phase_convention='original model q = relative commanded q + phase; canonical crank direction = 90deg - original q; then rotate all nodes by -mount_tilt',relative_input_range_deg=[0,v['stroke_deg']],phalanx_lengths_mm=load(finger)[0].objectives[0].data['phalanx_lengths_mm'],joint_flexion_ranges_deg=load(finger)[0].objectives[0].data['joint_flexion_ranges_deg'],validation=v,cad_or_hardware_modified=False)
    (OUT/finger/'parameters.json').write_text(json.dumps(params,ensure_ascii=False,indent=2),encoding='utf-8')

def img(name):return '<img src="data:image/png;base64,'+base64.b64encode((OUT/name).read_bytes()).decode()+'">'
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>四五指重新优化</title><style>body{font:16px/1.7 system-ui,"Microsoft YaHei";max-width:1200px;margin:32px auto;padding:0 24px;color:#253445}h1,h2{line-height:1.3}table{border-collapse:collapse;width:100%;margin:20px 0}td,th{border-bottom:1px solid #d6dee8;padding:9px;text-align:right}td:first-child,th:first-child{text-align:left}th{background:#edf3f6}img{width:100%;height:auto}aside{background:#edf5f3;padding:16px;border-left:4px solid #298578}.muted{color:#596575;font-size:14px}</style><h1>四指短基座、五指长基座：重新优化结果</h1>'''
page+='<aside>两指均找到通过二维模型复查的候选：从伸直到目标弯曲，连续求解 1,201 个姿态，未检出模型内碰撞。此结果是局部搜索得到的可行候选，不是全局最短解。未修改 CAD、舵机设置或原仓库代码。</aside>'
page+='<h2>与上次保留候选比较</h2><table><tr><th>手指</th><th>最大闭合误差（原评估器 301 姿态）</th><th>最小间隙（本次 1,201 姿态）</th><th>新起始角度</th><th>相对输入行程</th></tr>'+''.join(metricrows)+'</table>'
page+='<p>正间隙表示分离，负值表示模型穿透。更精确的逐步闭合求解中，两指的残差均小于 10⁻⁸ mm；这是数值方程的求解精度，不是实物制造精度。起始角度相对原模型 q=0，不能直接作为舵机编码器设定值。</p>'
page+=img('length_comparison.png')+img('optimized_motion.png')+img('clearance_sweep.png')+''.join(sections)
page+='''<h2>这次具体改了什么</h2><p>固定 A、D、E 三个基座铰点对应的 AD、AE、DE 孔距、安装倾角和 D–MCP 7 mm 间距；保留用户给出的指骨长度和最大关节弯曲角。联合优化 9 个活动机构尺寸、H–R4 输出杆长度以及整条输入角度轨迹，起始角度限制在原模型 ±15° 内，输入总行程不超过 90°。</p><p>使用有界最小二乘进行直接轨迹优化：先求闭合与避碰，再加入活动尺寸平方和的缩短项。碰撞软目标为 1 mm，最终硬检查为间隙不小于 0。采用 13、25 个优化姿态，原评估器用 301 个姿态检查，再对 1,201 个等输入角姿态逐一求精确被动手指闭合。机构组装分支以不超过 0.25° 的步长连续跟踪。五指对展开起点的多个解分别复查，舍弃起步不能连续闭合的解。</p><p>这是一次分阶段局部优化；部分轮次达到计算次数上限，未证明全局收敛。所有尝试与被拒方案保存在 trials.json 和 validation_trials.json。优化更换了求解策略，并非仅调低 Adam 学习率。</p><h2>结果的使用范围</h2><p>碰撞检查沿用项目的二维手掌、指骨胶囊和连杆/刚体模型，保留原本的基座与 R4 接触豁免区。尚未检查实际 CAD 的三维厚度、相邻手指、螺钉、舵机外壳、自碰撞、制造公差、受力或驱动能力。FGH 的边长是同一刚体上的孔距；杆长平方和也不是机构的实际外形长度。</p><p>“上次”指上一轮优化保留值，不等同于当前实物。旧仓库模型列来自 glove-servo-bridge 的已保存模型；它的基座尺寸与优化配置略有差异，未验证当前 Onshape。基座固定是指相对本次采用的优化起点保持不变。</p><p>本次参数请使用各手指 parameters.json，其中包含起始角度约定。原项目的普通 mechanism.yaml 不支持本次新增的零位参数，不能忽略零位直接导入。</p><p class="muted">源码：Chiniklas/kinematic_chain，wanwan，879f499cdf6fa16e822807b4534cc1cc403f4ab5。旧模型：zjn6060/modular_teleoperation_glove，glove-servo-bridge，d258e913a08f7d7f32bcdd5c92855d7b9d37d2f4。本地使用 Python 3.12 环境，非项目锁定依赖的完全复现。运行环境与文件哈希见 provenance.json。</p></html>'''
(OUT/'report.html').write_text(page,encoding='utf-8')
with (OUT/'all_length_comparison.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.DictWriter(f,fieldnames=allrows[0].keys());w.writeheader();w.writerows(allrows)
shutil.copy2(ROOT/'work/report_reoptimization.py',OUT/'report_reoptimization.py')
import scipy,yaml
provenance=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,matplotlib=matplotlib.__version__,pyyaml=yaml.__version__,platform=platform.platform(),upstream_commit='879f499cdf6fa16e822807b4534cc1cc403f4ab5',commands=['python work/reoptimize_fingers.py ring --max-nfev 160','python work/reoptimize_fingers.py little --max-nfev 160','python work/validate_reoptimization.py','python work/reoptimize_fingers.py ring --resume --max-nfev 160','python work/reoptimize_fingers.py little --resume --max-nfev 160','python work/validate_reoptimization.py','python work/report_reoptimization.py'])
provenance['sha256']={name:hashlib.sha256((OUT/name).read_bytes()).hexdigest() for name in ['reoptimize_fingers.py','validate_reoptimization.py','report_reoptimization.py']}
(OUT/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2),encoding='utf-8')
for finger,profile in [('ring','short_ad'),('little','long_ad')]:
    folder=OUT/'input_snapshot'/finger
    shutil.copytree(ROOT/f'outputs/finger_optimization/{finger}_{profile}/input_snapshot',folder,dirs_exist_ok=True)
    shutil.copy2(ROOT/f'outputs/finger_optimization/{finger}_{profile}/fingers/{finger}/candidate_0001/candidate.yaml',folder/'baseline_candidate.yaml')
source=OUT/'source_snapshot';source.mkdir(exist_ok=True)
for name in ['run_adam.py','tensorboard_logger.py']:shutil.copy2(ROOT/'work/kinematic_chain/src/co-optimization'/name,source/name)
shutil.copy2(ROOT/'outputs/finger_optimization/previous_glove_lengths.json',OUT/'input_snapshot/previous_glove_lengths.json')
(OUT/'report.html').write_text(page.replace('这是一次分阶段局部优化；','本轮没有把输出杆垂直度加入优化目标，传动效率与受力需另行验证。为防止基座豁免区造成刚体漏检，CD、DG 均限制为超过豁免半径至少 0.5 mm（四指 CD 下限由 8 调为 12.5 mm）。这是一次分阶段局部优化；'),encoding='utf-8')
(OUT/'READ_ME.txt').write_text('先打开 report.html 查看结果。两指 parameters.json 是包含起始角度的最终候选参数，exact_sweep.csv 是 1201 姿态检查。all_length_comparison.csv 含旧仓库、上次优化和本次三方尺寸。源码仍需同目录 work/kinematic_chain 与 outputs/finger_optimization 的已有输入；本包并非独立安装程序。二维数值候选，未更新 CAD/硬件。',encoding='utf-8')
print('Wrote report and figures.',flush=True)
