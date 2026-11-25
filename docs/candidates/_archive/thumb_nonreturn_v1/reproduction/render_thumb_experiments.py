import json,math,io,shutil,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from matplotlib.font_manager import FontProperties
from PIL import Image
import thumb_antifold_experiment as t

OUT=t.OUT
FONT=FontProperties(fname=r'C:\Windows\Fonts\msyh.ttc')
plt.rcParams['axes.unicode_minus']=False
TITLES={'baseline':'原方案：折返布局','order':'A  节点位置约束','angle':'B  输出件夹角 ≥ 90°',
        'compact':'C  活动范围＋紧凑性','combined':'D  组合约束，E 固定','relocated_E':'E  组合约束，移动 E 点'}
COLORS={'ADE':'#7c8799','AB':'#3886bd','BC':'#66a8bd','CDG':'#546f94','EF':'#d09636','FGH':'#c65f65'}
FAIL_LABELS={'assembly':'闭环无解','rigid_lengths':'杆长不闭合','direct_closure':'固定座不闭合','endpoint':'末端角度不达标',
 'trajectory':'轨迹误差超限','monotone_mcp':'MCP 反转','monotone_ip':'IP 反转','continuous_branches':'装配分支改变',
 'intersection_margin':'圆交点接近相切','length_bounds':'杆长越界','rigid_shape_margin':'输出件形状退化',
 'node_order':'节点越过 H','FGH_angle':'夹角不足90°','envelope':'外形越界','layered_clearance_screen':'简化干涉筛选未通过'}

def pick(variant):
    if variant=='relocated_E':
        pool=json.loads((OUT/'relocated_E_search.json').read_text())['trials']
        v='combined'
    else:
        v=variant;pool=json.loads((OUT/(variant+'_search.json')).read_text())['trials']
        for path in ['polished.json','final_polished.json']:
            if (OUT/path).exists():pool += [r for r in json.loads((OUT/path).read_text()) if r['variant']==variant]
        if variant in ['angle','combined']:
            pool+=json.loads((OUT/'targeted_search.json').read_text())['trials']
    pool=[dict(r,**t.evaluate(np.array(r['x']),v)) for r in pool]
    successful=[r for r in pool if r['passed']]
    if successful:
        selected=min(successful,key=lambda r:r['moving_length_sum_mm']+.2*r['trajectory_rms_mm'])
    else:
        # For a rejected experiment show an assembling attempt that actually
        # exhibits the requested shape, instead of silently relaxing its goal.
        keys=['assembly','rigid_lengths','direct_closure']
        if variant in ['angle','combined','relocated_E']:keys+=['FGH_angle']
        if variant in ['combined','relocated_E']:keys+=['node_order','envelope']
        eligible=[r for r in pool if all(r['checks'].get(k,False) for k in keys)]
        selected=min(eligible or pool,key=t.ranking)
    selected=dict(selected,**t.evaluate(np.array(selected['x']),v,n=5001))
    selected['panel']=variant;selected['title']=TITLES[variant]
    return selected

def capsule(ax,a,b,r,color,alpha=1,z=1):
    v=b-a;u=v/max(np.linalg.norm(v),1e-9);n=np.array([-u[1],u[0]])*r
    ax.add_patch(Polygon([a+n,b+n,b-n,a-n],closed=True,color=color,alpha=alpha,zorder=z,linewidth=0))
    for p in [a,b]:ax.add_patch(Circle(p,r,color=color,alpha=alpha,zorder=z,linewidth=0))

def draw(ax,record,s,small=False):
    x=np.array(record['x']);pts,viol,m,ip,r=t.geometry(x,np.array([s]));pts={k:v[0] for k,v in pts.items()}
    m=float(m[0]);ip=float(ip[0]);mr,ir=np.radians([m,ip]);o=np.array([0.,-7.])
    j=o+30*np.array([np.cos(mr),-np.sin(mr)]);tip=j+40*np.array([np.cos(mr+ir),-np.sin(mr+ir)])
    for a,b,angle,depth in [(o,j,-mr,20),(j,tip,-mr-ir,10)]:
        n=np.array([-np.sin(angle),np.cos(angle)])
        capsule(ax,a-depth/2*n,b-depth/2*n,depth/2,'#cbb49c',alpha=.48,z=0)
    # Draw manufactured chain segments, not the diagonal dimension lines DG/FH.
    for name,nodes,radius,z in [('ADE','adea',3,1),('AB','ab',3,2),('EF','ef',4,3),('CDG','dcg',4,4),('BC','bc',4,5),('FGH','fgh',4,6)]:
        for a,b in zip(nodes,nodes[1:]):capsule(ax,pts[a],pts[b],radius,COLORS[name],z=z)
    for name,p in pts.items():
        ax.add_patch(Circle(p,1.7,facecolor='#f5f6f9',edgecolor='#334155',linewidth=.7,zorder=8))
        ax.annotate(name.upper(),p,xytext=(4,5),textcoords='offset points',fontsize=7 if small else 9,zorder=9)
    target=t.model.target(np.linspace(0,1,101))@t.model.R
    ax.plot(*target.T,ls=':',color='#64748b',lw=.8,zorder=1)
    ax.set_xlim(112,-62);ax.set_ylim(-88,76);ax.set_aspect('equal');ax.grid(alpha=.12)
    ax.tick_params(labelsize=6 if small else 8)
    status='原方案参考' if record['panel']=='baseline' else ('通过本轮筛选' if record['passed'] else '未通过 · 不可作为可用设计')
    color='#475569' if record['panel']=='baseline' else ('#166534' if record['passed'] else '#b91c1c')
    ax.set_title(record['title']+'\n'+status,fontproperties=FONT,fontsize=10 if small else 13,color=color,pad=7)
    note=f"电机 {np.degrees(x[-1])*s:.1f}°   MCP {m:.1f}° / IP {ip:.1f}°"
    ax.text(.5,.025,note,transform=ax.transAxes,ha='center',fontproperties=FONT,fontsize=7 if small else 10,
            bbox=dict(facecolor='white',alpha=.8,edgecolor='none'),zorder=10)
    if record['panel']=='relocated_E':
        ax.text(.5,.96,'需要重新设计 ADE 支架',transform=ax.transAxes,ha='center',fontproperties=FONT,fontsize=7 if small else 9,color='#92400e',zorder=10)

def raster(fig):
    fig.canvas.draw();return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy())

def gif(frames,path):
    palette=frames[0].quantize(colors=128,method=Image.Quantize.MEDIANCUT)
    converted=[frame.quantize(palette=palette,dither=Image.Dither.NONE) for frame in frames]
    loop=converted+converted[-2:0:-1]
    durations=[90]*len(loop);durations[0]=650;durations[len(converted)-1]=650
    loop[0].save(path,save_all=True,append_images=loop[1:],duration=durations,loop=0,optimize=True,disposal=2)

def main():
    comparison_only='--comparison-only' in sys.argv
    baseline=t.evaluate(t.X0,'baseline',n=5001);baseline.update(panel='baseline',title=TITLES['baseline'])
    records=[baseline]+[pick(v) for v in ['order','angle','compact','combined','relocated_E']]
    (OUT/'selected_results.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    sequences={r['panel']:[] for r in records};combined=[]
    samples=np.linspace(0,1,41)
    for index,s in enumerate(samples):
        fig,axes=plt.subplots(2,3,figsize=(12,8.6),dpi=110)
        fig.subplots_adjust(left=.025,right=.985,bottom=.055,top=.85,wspace=.12,hspace=.34)
        fig.suptitle('拇指防折返实验：相同行程进度、统一比例',fontproperties=FONT,fontsize=17,y=.985)
        fig.text(.5,.94,'向左为指尖方向；虚线为目标 H 轨迹；红色标题表示未通过筛选',ha='center',fontproperties=FONT,fontsize=10)
        for ax,record in zip(axes.flat,records):draw(ax,record,float(s),small=True)
        fig.text(.5,.012,'侧视投影：不同轴向层可在画面重叠；仅做简化间隙筛选，未重建三维 CAD',ha='center',fontproperties=FONT,fontsize=9,color='#475569')
        frame=raster(fig);combined.append(frame)
        if index==0:frame.save(OUT/'comparison_start.png')
        if index==20:frame.save(OUT/'comparison_mid.png')
        if index==40:frame.save(OUT/'comparison_end.png')
        plt.close(fig)
        for record in ([] if comparison_only else records):
            fig,ax=plt.subplots(figsize=(6.6,6.4),dpi=100)
            fig.subplots_adjust(left=.07,right=.97,bottom=.12,top=.86)
            draw(ax,record,float(s))
            summary=f"FGH 内角 {record['FGH_angle_deg']:.1f}°  |  最大 H 轨迹误差 {record['trajectory_max_error_mm']:.3f} mm"
            fig.text(.5,.05,summary,ha='center',fontproperties=FONT,fontsize=10)
            sequences[record['panel']].append(raster(fig));plt.close(fig)
        if index%10==0:print('Rendered frame',index,flush=True)
    gif(combined,OUT/'comparison_all.gif')
    if comparison_only:
        print('Comparison layout refreshed',flush=True)
        return
    for key,frames in sequences.items():gif(frames,OUT/(key+'.gif'))
    successful=[r for r in records if r['panel']!='baseline' and r['passed'] and r['FGH_angle_deg']>=90 and r['early_FG_overshoot_mm']<=2 and r['envelope_excess_mm']<=.01]
    if successful:
        # Preserve E whenever its fixed-ground solution passes the same combined gates.
        recommended=min(successful,key=lambda r:(r['panel']=='relocated_E',r['trajectory_rms_mm']))
        key=recommended['panel'];side_by_side=[]
        for old,new in zip(sequences['baseline'],sequences[key]):
            canvas=Image.new('RGB',(old.width+new.width,old.height),'white');canvas.paste(old,(0,0));canvas.paste(new,(old.width,0));side_by_side.append(canvas)
        gif(side_by_side,OUT/'recommended_vs_original.gif')
        (OUT/'recommended.json').write_text(json.dumps(recommended,indent=2),encoding='utf-8')
    lines=['拇指防折返优化实验报告','',
     '未改动仓库中原方案，也未重新优化其他手指。原模型根关节位置、30/40 mm 指节长度及 H 直连远节中点保持不变。',
     'A-D 基线21.112 mm，倾角37.054°保持不变。A/B/C/D方案还固定E；E方案明确允许E移动，需要新ADE支架。','',
     '实验定义：',
     'A：前25%输入行程内，F/G在MCP→H方向上越过H不得超过2 mm。',
     'B：FGH刚体在G处内角90–170°。',
     'C：所有机构节点整段行程在x=[-40,65]、y=[-65,50] mm内，并惩罚活动尺寸平方和。',
     'D：A+B+C，同时保持E固定。',
     'E：D的约束不变，允许改变E的位置；A、D仍固定。',
     '共通条件：杆长8–85 mm；FGH内角15–170°；MCP/IP单调；端点45°/60°误差≤0.25°；H目标轨迹最大误差≤1mm；闭环误差≤1e-6mm；圆交点正弦裕量≥0.04。',
     '搜索使用有限多起点和罚函数；最终严格按条件筛选。未找到合格解不表示数学上无解。',
     '最终复核每方案5001姿态；相同动画百分比代表各自电机行程的同一比例，不表示相同绝对电机角度。','',
     '简化间隙筛选：沿用原板件z层高，检查轴销与非关联杆件、轴销之间，以及与人体参考胶囊的距离。通过门槛≥0.01mm。',
     '未检查完整电机/支架外壳、固定座、隔套、螺钉头、公差和连续扫掠；不等于三维CAD碰撞验证。原方案可能出现很小的简化人体重叠，不能据此推翻原先仅硬件的CAD检查。','']
    for r in records:
        lines += [r['title'],'状态：'+('原方案参考' if r['panel']=='baseline' else ('通过本轮筛选' if r['passed'] else '未通过')),
                  '未通过项：'+('、'.join(FAIL_LABELS[k] for k,v in r['checks'].items() if not v) or '无'),
                  'FGH角=%.4f°；前段最大节点超前=%.4f mm；H最大轨迹误差=%.6f mm'%(r['FGH_angle_deg'],r['early_FG_overshoot_mm'],r['trajectory_max_error_mm']),
                  '运动终点MCP/IP='+str(r['endpoint_deg'])+'；电机总行程=%.6f°'%r['motor_travel_deg'],
                  '最小简化间隙=%.6f mm（%s）'%(r['collision_screen']['minimum_clearance_mm'],r['collision_screen']['worst_pair']),
                  '边界(mm)='+str(r['bounds_mm']),
                  '活动尺寸合计=%.6f mm（含定义刚体形状的对角孔距，不是材料长度）'%r['moving_length_sum_mm'],
                  '孔中心距(mm)：'+', '.join('%s=%.6f'%(k[2:].upper(),v) for k,v in r['lengths_mm'].items()),
                  '初始参数direct_synthesis='+str(r['x']),'']
    (OUT/'实验说明与结果.txt').write_text('\n'.join(lines),encoding='utf-8')
    for r in records:
        dest=OUT/('parameters_'+r['panel']+'.json')
        p=json.loads((t.SOURCE/'parameters.json').read_text());p['direct_synthesis']=r['x'];p['lengths']=r['lengths_mm']
        p['q_schedule_deg']=np.linspace(0,r['motor_travel_deg'],301).tolist()
        p['experiment_status']='passed_simplified_screen_not_CAD_release' if r['passed'] else 'reference_or_rejected_do_not_build'
        p['experiment']=r
        dest.write_text(json.dumps(p,indent=2),encoding='utf-8')
    print('DELIVERABLES',OUT,flush=True)

if __name__=='__main__':main()
