"""Export direct-H feasibility experiments, including failed candidates."""
from direct import *
import copy
import csv
import hashlib
import shutil
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.animation import FuncAnimation, PillowWriter

HERE=Path(__file__).resolve().parent



def draw_capsule(ax,a,b,radius,color='#a8adb5',alpha=.45,zorder=1):
    a=np.array(a);b=np.array(b);direction=b-a
    normal=rot90(direction)/max(np.linalg.norm(direction),1e-12)*radius
    ax.add_patch(Polygon([a+normal,b+normal,b-normal,a-normal],color=color,alpha=alpha,zorder=zorder,lw=0))
    for p in [a,b]:ax.add_patch(Circle(p,radius,color=color,alpha=alpha,zorder=zorder,lw=0))


def plot_pose(ax,f,result,rows,nodes,index):
    m,p=load_design(f,f);row=rows[index];pos={k:v[index] for k,v in nodes.items()}
    ax.clear();ax.set_aspect('equal');ax.set_xlim(-100,120);ax.set_ylim(-105,90);ax.grid(alpha=.15)
    for _,a,b,radius in m._hand_capsules(p,p.objectives[0],row['progress']):draw_capsule(ax,a,b,radius)
    for u,v in [('a','d'),('d','e'),('e','a')]:ax.plot(*np.array([pos[u],pos[v]]).T,color='#485567',lw=1.5)
    for rowdim in p.nominal_design['dimensions']:
        if rowdim['body']=='ground':continue
        u,v=rowdim['nodes']
        if rowdim['id']=='L_fh':
            ax.plot(*np.array([pos[u],pos[v]]).T,color='#93b7cb',lw=.8,ls='--',alpha=.7)
        else:
            draw_capsule(ax,pos[u],pos[v],2,color='#287ba4',alpha=.7,zorder=2)
    ax.plot(*pos['h'],'o',color='#ed6e26',ms=6,zorder=6,label='H: mechanism output')
    ax.plot(*row['r4_target_mm'],'x',color='#111111',ms=7,zorder=7,label='R4: hand target')
    ax.add_patch(Circle(pos['h'],4,fill=False,edgecolor='#ed6e26',ls=':',lw=1,zorder=5))
    for key in ['f','g','h']:ax.annotate(key.upper(),pos[key],xytext=(4,6),textcoords='offset points',fontsize=9,zorder=8)
    v=result['validation'];status='SAMPLED PASS' if v['passed'] else 'NOT PASSED'
    ax.set_title(f'{f.title()} - {status}',fontsize=11,color='#246d41' if v['passed'] else '#a5382b',fontweight='bold')
    ax.set_xlabel('Horizontal coordinate from finger root (mm)',fontsize=8);ax.set_ylabel('Vertical coordinate (mm)',fontsize=8)
    ax.text(.02,.02,f'Curl {100*row["progress"]:.1f}% | endpoint error {row["endpoint_error_mm"]:.3f} mm\nClearance {row["clearance_mm"]:+.3f} mm | no H-R4 rod',transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',edgecolor='none',alpha=.9),zorder=9)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True,type=Path,help='New directory for the reproduced report')
    parser.add_argument('--skip-gif',action='store_true',help='Reuse the archived GIF; regenerate data and static figures')
    args=parser.parse_args();OUT=args.out.resolve()
    if OUT==HERE or HERE in OUT.parents:
        parser.error('--out must be outside the source archive')
    if OUT.exists() and any(OUT.iterdir()):
        parser.error('--out must be empty or not exist')
    OUT.mkdir(parents=True,exist_ok=True);allresults={};visual={};comparison=[]
    shutil.copytree(HERE/'input_snapshot',OUT/'input_snapshot')
    for f in FINGERS:
        # Save exact inputs and every completed experimental variant.
        src=OUT/'source_candidates'/f;src.mkdir(parents=True,exist_ok=True)
        shutil.copy(HERE/'source_candidates'/f/'result.json',src/'result.json')
        converted=json.loads((HERE/'direct_conversion'/f/'result.json').read_text())
        variants={}
        for kind,folder,shape in [('direct_conversion','direct_conversion','filled_triangle'),
                                  ('direct_conversion_bracket','direct_conversion','l_bracket'),
                                  ('refined_triangle','refined_triangle','filled_triangle'),
                                  ('refined_bracket','refined_bracket','l_bracket')]:
            candidate=json.loads((HERE/folder/f/'result.json').read_text());candidate['output_shape']=shape
            report,rows,nodes=validate(candidate)
            candidate['validation']=report;candidate['status']='sampled_planar_candidate' if report['passed'] else 'rejected_search_candidate'
            target=OUT/kind/f;target.mkdir(parents=True,exist_ok=True)
            (target/'result.json').write_text(json.dumps(candidate,indent=2));(target/'sweep.json').write_text(json.dumps(rows,indent=2))
            variants[kind]=candidate
            if kind.endswith('bracket'):visual[(kind,f)]=(candidate,rows,nodes)
        search=OUT/'searches'/f;search.mkdir(parents=True,exist_ok=True)
        for suffix in ['refined','bracket']:
            for path in (HERE/'searches'/f).glob(suffix+'_stage_*.json'):
                shutil.copy(path,search/path.name)
            shutil.copy(HERE/'searches'/f/(suffix+'.log'),search/(suffix+'.log'))
        # Evaluate intermediate bracket snapshots independently as well.
        alternatives=[(variants['refined_bracket'],'result')]
        for path in (HERE/'searches'/f).glob('bracket_stage_*.json'):
            c=json.loads(path.read_text());c['output_shape']='l_bracket';v,rows,nodes=validate(c);c['validation']=v
            alternatives.append((c,path.stem.removeprefix('bracket_')))
        def score(item):
            v=item[0]['validation']
            return (not v['passed'],not v['collision_passed'],not v['assembly_valid'],v['maximum_endpoint_error_mm'])
        chosen,stage=min(alternatives,key=score)
        chosen['selected_stage']=stage
        v,rows,nodes=validate(chosen);chosen['validation']=v;chosen['status']='sampled_planar_candidate' if v['passed'] else 'rejected_search_candidate'
        target=OUT/'selected'/f;target.mkdir(parents=True,exist_ok=True)
        (target/'result.json').write_text(json.dumps(chosen,indent=2));(target/'sweep.json').write_text(json.dumps(rows,indent=2))
        visual[('selected',f)]=(chosen,rows,nodes)
        variants['selected']=chosen
        allresults[f]={k:{'validation':c['validation'],'lengths_mm':c['lengths_mm']} for k,c in variants.items()}
        allresults[f]['maximum_conversion_trajectory_change_mm']=converted['conversion_max_endpoint_change_mm']
        for key,value in chosen['lengths_mm'].items():comparison.append({'finger':f,'dimension':key,'direct_conversion_mm':converted['lengths_mm'][key],'selected_refinement_mm':value,'change_mm':value-converted['lengths_mm'][key]})
    summary={'source_commit':json.loads((HERE/'summary.json').read_text())['source_commit'],
             'date':'2026-11-18','model':'H directly pins to distal hand attachment R4; no H-R4 rod or perpendicularity constraint',
             'fixed_inputs':['motor-hole locations','per-finger horizontal root offsets','vertical mount clearance','phalanx lengths','joint maximum angles','existing proportional joint-flexion path'],
             'geometry_policy':{'main':'L bracket FG + GH, 2 mm link radius; FH is a geometric dimension, not an extra physical strut',
                                'conservative_comparison':'Filled FGH triangle, including FH edge',
                                'attachment_exemption_mm':4,'mount_exemption_mm':12},
             'all_fingers_passed':all(allresults[f]['selected']['validation']['passed'] for f in FINGERS),
             'global_infeasibility_proven':False,'finite_search':True,'results':allresults}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2))
    with (OUT/'dimension_changes.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(comparison[0]));writer.writeheader();writer.writerows(comparison)
    for script in ['direct.py','test_direct.py','package.py']:shutil.copy(HERE/script,OUT/script)
    for kind,title,name in [('direct_conversion_bracket','Direct conversion: unchanged endpoint path, new straight GH attachment','direct_conversion.png'),
                            ('selected','Direct-H refinement: independent 1,201-pose checks','refined.png')]:
        fig,axes=plt.subplots(2,2,figsize=(12,10),layout='constrained')
        for ax,f in zip(axes.flat,FINGERS):
            c,rows,nodes=visual[(kind,f)];i=int(np.argmin([r['clearance_mm'] for r in rows]));plot_pose(ax,f,c,rows,nodes,i)
        fig.suptitle(title+'\nOrange = H; black cross = hand R4; dashed FH = construction dimension',fontsize=12)
        fig.savefig(OUT/name,dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,10),layout='constrained')
    fig.suptitle('H directly attached at R4 - experimental refinements\nFailed candidates are retained for diagnosis, not approved designs',fontsize=12)
    sequence=np.r_[np.linspace(0,1200,25,dtype=int),np.linspace(1200,0,25,dtype=int)[1:-1]]
    def frame(i):
        for ax,f in zip(axes.flat,FINGERS):plot_pose(ax,f,*visual[('selected',f)],i)
    if args.skip_gif:shutil.copy(HERE/'motion.gif',OUT/'motion.gif')
    else:
        animation=FuncAnimation(fig,frame,frames=sequence,interval=100)
        animation.save(OUT/'motion.gif',writer=PillowWriter(fps=10),dpi=72)
    plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for ax,f in zip(axes.flat,FINGERS):
        _,rows,_=visual[('selected',f)];s=[r['progress']*100 for r in rows]
        ax.plot(s,[r['endpoint_error_mm'] for r in rows],label='H / hand R4 mismatch',color='#c24b36')
        ax.plot(s,[r['clearance_mm'] for r in rows],label='Minimum clearance',color='#297ca5')
        ax.axhline(.1,color='#c24b36',ls='--',lw=.8,label='0.1 mm endpoint limit')
        ax.axhline(0,color='#444444',ls=':',lw=.8);ax.set_title(f.title());ax.set_xlabel('Hand curl (%)');ax.set_ylabel('mm');ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Independent dense sweep of selected experimental refinements',fontsize=12)
    fig.savefig(OUT/'error_clearance.png',dpi=150);plt.close(fig)
    cn={'index':'食指','middle':'中指','ring':'无名指','little':'小指'}
    text=['# H 与 R4 直接连接：验证报告','',
          '日期：2026-11-18。此报告是独立的数值验证，不是制造定稿。','',
          '## 结论','',
          '取消 H–R4 杆在运动学上成立：将新 GH、FH 设为旧 G–R4、F–R4，原刚性输出的 R4 轨迹得到保留。但是直接改成两臂 FG/GH 的输出件会改变实体外形，原先的避碰结论不能沿用。',
          '本轮在保留安装孔、根部偏移、手指尺寸及原有比例屈曲轨迹的条件下，进行了两种外形、每指两级局部优化；结果如下。尚未证明全局不可行。','',
          '## 直接转换后的两臂折杆','',
          '| 手指 | 最大轨迹误差 mm | 最小间隙 mm | 最危险部位 |',
          '|---|---:|---:|---|']
    for f in FINGERS:
        v=allresults[f]['direct_conversion_bracket']['validation'];text.append(f'| {cn[f]} | {v["maximum_endpoint_error_mm"]:.4f} | {v["minimum_clearance_mm"]:.4f} | {v["worst_clearance_pair"]} |')
    text+=['','负间隙表示模型中的重叠，不能直接当作真实零件的穿透深度。四指最危险位置均接近最大弯曲。','',
           '## 调整尺寸后的候选','',
           '| 手指 | 最大轨迹误差 mm | 最小间隙 mm | 判定 |',
           '|---|---:|---:|---|']
    for f in FINGERS:
        v=allresults[f]['selected']['validation'];text.append(f'| {cn[f]} | {v["maximum_endpoint_error_mm"]:.4f} | {v["minimum_clearance_mm"]:.4f} | {"采样通过" if v["passed"] else "未通过"} |')
    text+=['','判据：每指独立采样 1,201 个输入位置；全行程 H 对手指 R4 轨迹的误差 ≤ 0.1 mm，非允许接触处间隙 ≥ 0，并满足装配、行程覆盖及单调屈曲检查。局部优化使用 17、33 个姿态，密集验证独立进行；中间迭代也重新验证。',
           '不能将优化器返回 success 等同于机构通过验证，也不能将存在小的拟合误差表述为代数上严格闭合。','',
           '## 外形与接触的处理','',
           '- 新模型没有 `L_tip_rod`，也没有垂直约束；H 是末端唯一的转动连接点。',
           '- 原库将输出件绘制为 FG/GH 两臂折杆，但旧避碰计算使用实心 FGH 三角形。本报告两者都验证，主要图和候选采用折杆。FH 仍然定义刚体形状，不代表必须有一根实体 FH 杆。',
           '- 两臂按 2 mm 半径处理。原 R4 处 4 mm 安装接触豁免移到 H，仅裁去连接端附近的局部区域；D 处保留原 12 mm 安装豁免。没有忽略整个末节手指或整个输出件。',
           '- 中央 CDG 部件仍采用原保守实体几何。没有重新确认实际 CAD 厚度、轴向错层或连接件外形。','',
           '## 下一步建议','',
           '优先保持 H=R4，研究 GH 整体输出件的避让外形，而非继续增加连接杆。端点直接连接不要求端点之间的材料必须是直线；一体式弯曲件可能绕开手指，但其刚度、厚度和三维装配尚需验证。',
           '如果要求 GH 保持直线，则还需要继续联合优化 F/G 位置和其他杆长；本轮局部搜索尚不足以保证成功。原手指轨迹将三个关节规定为同一进度按比例弯曲，并非实测完整轨迹，也可能是限制因素。改变这条路径或 R4 的安装位置会改变验证前提，需要在后续结果中明确说明。','',
           '## 验证范围与复现','',
           '三项数值测试检查线段距离、全行程转换等价性和局部接触豁免。验证仅为平面采样结果；未认证连续区间、三维/邻指碰撞、全局最优性、力传递、刚度或完整奇异性。较小正间隙也不代表制造余量充分。',
           '源仓库提交：`'+summary['source_commit']+'`。程序自动定位所属仓库，也可用 `KINEMATIC_CHAIN_REPO` 指向另一个检出位置；手指输入使用本目录的 `input_snapshot/`。','',
           '```bash','python3 -B test_direct.py','python3 -B direct.py validate --source selected/index/result.json --out /tmp/h-direct-check','```','',
           '数据：`summary.json`、`dimension_changes.csv`、`selected/`；原始直接转换：`direct_conversion/`；两种外形的所有局部优化结果保存在 `searches/` 和相应的 `refined_*` 目录。动画：`motion.gif`。']
    text+=['', '运行前进入本候选目录，安装 NumPy、SciPy、PyYAML；绘图还需 Matplotlib 和 Pillow。', '', '```bash', 'python3 -B package.py --out /tmp/h-direct-report', '```', '', '![候选运动动画（尚未通过）](motion.gif)']
    (OUT/'README.md').write_text('\n'.join(text)+'\n')
    manifest={str(path.relative_to(OUT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(OUT.rglob('*')) if path.is_file() and path.name!='manifest.json'}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({f:allresults[f]['selected']['validation'] for f in FINGERS},indent=2),flush=True)


if __name__=='__main__':main()
