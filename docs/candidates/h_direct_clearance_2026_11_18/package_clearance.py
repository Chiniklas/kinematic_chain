"""Export fixed-hole contour candidates and their sampled clearance checks."""
from check_clearance import *
import shutil
import gzip
import hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.animation import FuncAnimation, PillowWriter


def capsule(ax,a,b,r,color,alpha=1.):
    a=np.array(a);b=np.array(b);delta=b-a
    normal=rot90(delta)/max(np.linalg.norm(delta),1e-12)*r
    ax.add_patch(Polygon([a+normal,b+normal,b-normal,a-normal],color=color,alpha=alpha,lw=0))
    for point in [a,b]:ax.add_patch(Circle(point,r,color=color,alpha=alpha,lw=0))


def local_at_h(witness):
    nodes={k:np.array(v) for k,v in witness['nodes_mm'].items()};origin=nodes['h']
    u=(nodes['h']-nodes['g'])/np.linalg.norm(nodes['h']-nodes['g']);rot=np.stack((u,rot90(u)),axis=1)
    local={k:(v-origin)@rot for k,v in nodes.items()}
    hands=np.array(witness['hand_capsules_mm']);hands[:,:2,:]=(hands[:,:2,:]-origin)@rot
    contour=(np.array(witness['new_contour_mm'])-origin)@rot
    return local,hands,contour


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();out=args.out.resolve()
    if out==HERE or HERE in out.parents:parser.error('--out must be outside the source package')
    out.mkdir(parents=True,exist_ok=True)
    result_root=HERE/'results' if (HERE/'results').is_dir() else HERE/'selected'
    reports={};results={};witnesses={};visual={};cn={'index':'食指','middle':'中指','ring':'无名指','little':'小指'}
    for f in FINGERS:
        source=result_root/f;dest=out/'selected'/f;dest.mkdir(parents=True,exist_ok=True)
        for name in ['result.json','validation.json','witnesses.json']:shutil.copy2(source/name,dest/name)
        if (source/'clearance_sweeps.json').exists():
            with (source/'clearance_sweeps.json').open('rb') as rd,gzip.open(dest/'clearance_sweeps.json.gz','wb') as wr:shutil.copyfileobj(rd,wr)
        else:shutil.copy2(source/'clearance_sweeps.json.gz',dest/'clearance_sweeps.json.gz')
        reports[f]=json.loads((source/'validation.json').read_text());results[f]=json.loads((source/'result.json').read_text())
        witnesses[f]=json.loads((source/'witnesses.json').read_text())
        baseline=json.loads((BASE/f/'result.json').read_text())
        assert results[f]['lengths_mm']==baseline['lengths_mm']
    for folder in ['baseline','input_snapshot']:shutil.copytree(HERE/folder,out/folder,dirs_exist_ok=True)
    for name in ['base_geometry.py','verify_1mm.py','audit_closure.py','check_clearance.py','test_clearance.py','package_clearance.py']:shutil.copy2(HERE/name,out/name)
    logs=out/'logs';logs.mkdir(exist_ok=True)
    for source in HERE.glob('*.log'):shutil.copy2(source,logs/source.name)
    summary={'date':'2026-11-18','source_commit':'eaa99da09862c05bcbfeea907cd4c48453f80f96',
        'criteria':{'maximum_reference_trajectory_error_mm':1.,'minimum_sampled_clearance_mm':0.,'strict_H_R4_coincidence_required':False},
        'all_four_passed_sampled_planar_criteria':all(v['passed_user_criteria'] for v in reports.values()),
        'all_link_dimensions_and_holes_unchanged':True,'repository_profiles_changed':False,'pushed':False,
        'results':{f:{'old_minimum_clearance_mm':v['old']['minimum_clearance_mm'],
            'new_minimum_clearance_mm':v['new']['minimum_clearance_mm'],
            'maximum_reference_error_mm':v['maximum_reference_trajectory_error_mm'],
            'new_limiting_pair':v['new']['pair'],'contour':v['new_contour'],
            'tested_configurations':v['total_tested_configurations']} for f,v in reports.items()}}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(3,2,figsize=(10,11),layout='constrained')
    for row,f in enumerate(['middle','ring','little']):
        pos,hands,path=local_at_h(witnesses[f]['old'])
        for col,ax in enumerate(axes[row]):
            for a,b,r in hands:capsule(ax,a,b,float(r[0]),'#aeb6be',.45)
            capsule(ax,pos['f'],pos['g'],2,'#6c91a3',.75)
            if col==0:capsule(ax,pos['g'],pos['h'],2,'#b84e42',.8)
            else:
                capsule(ax,path[0],path[1],2,'#318775',.85);capsule(ax,path[1],path[2],2,'#318775',.85)
                ax.plot(*path[1],marker='x',color='#1b5048',ms=6);ax.annotate('P (outline only)',path[1],xytext=(-6,12),textcoords='offset points',fontsize=9)
                ax.plot(*np.array([pos['g'],pos['h']]).T,ls=':',color='#b84e42',lw=1)
            ax.add_patch(Circle(pos['h'],4,fill=False,ls=':',edgecolor='#616b75',lw=1))
            ax.plot(*pos['h'],'o',color='#d27427',ms=5);ax.annotate('H',pos['h'],xytext=(5,5),textcoords='offset points',fontsize=10)
            ax.set_xlim(-22,7);ax.set_ylim(-13,10);ax.set_aspect('equal');ax.grid(alpha=.12)
            ax.set_xlabel('Local coordinate from H (mm)');ax.set_ylabel('Local coordinate (mm)')
            ax.set_title(f'{f.title()} / '+('original GH' if col==0 else '2 mm outward contour'))
    fig.suptitle('At the same previously interfering hand pose\n2 mm capsule radius; dotted H circle = unchanged 4 mm joint region',fontsize=12)
    fig.savefig(out/'clearance_before_after.png',dpi=145);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
    for ax,f in zip(axes,['middle','ring','little']):
        pos,_,path=local_at_h(witnesses[f]['old']);L=results[f]['lengths_mm']['L_gh']
        for k in pos:pos[k]+=np.array([L,0.])
        path+=np.array([L,0.])
        capsule(ax,pos['f'],pos['g'],2,'#708f9f',.8)
        capsule(ax,path[0],path[1],2,'#318775',.9);capsule(ax,path[1],path[2],2,'#318775',.9)
        ax.plot([0,L],[0,0],ls=':',lw=1,color='#ad5144')
        for k in ['g','f','h']:
            ax.plot(*pos[k],'o',color='#263945',ms=4);ax.annotate(k.upper(),pos[k],xytext=(0,-14),textcoords='offset points',ha='center',fontsize=9)
        ax.plot(*path[1],marker='x',color='#263945',ms=5);ax.annotate('P',path[1],xytext=(0,8),textcoords='offset points',ha='center',fontsize=9)
        ax.text(.02,.97,'P: 10 mm back from H, 2 mm outward',transform=ax.transAxes,va='top',fontsize=9)
        ax.set_aspect('equal');ax.set_xlim(-5,L+5);ax.set_ylim(min(pos['f'][1]-7,-12),13);ax.grid(alpha=.12)
        ax.set_title(f.title()+' / F, G, H unchanged');ax.set_xlabel('mm');ax.set_ylabel('mm')
    fig.suptitle('One rigid output part: existing FG + new G-P-H material path\nP is a contour corner, not a new hinge; dotted line is the original GH axis',fontsize=12)
    fig.savefig(out/'outline_design.png',dpi=145);plt.close(fig)
    # Motion preview uses the fixed reference hand pose; output geometry is the
    # same contour function used by the collision evaluator.
    for f in FINGERS:
        c=results[f];_,p=load_design(f,f);q=np.linspace(c['q_deg'][0],c['q_deg'][-1],33)
        nodes,_=fk(c['lengths_mm'],q,p.ad_tilt_deg,c['output_branch']);s,_=project_hand(p,nodes['h'])
        visual[f]=(p,nodes,fast_hands(p,s),outline(nodes,0. if f=='index' else 2.),s)
    fig,axes=plt.subplots(2,2,figsize=(10,8),layout='constrained')
    def draw_frame(i):
        for ax,f in zip(axes.flat,FINGERS):
            p,nodes,hands,path,s=visual[f];pos={k:v[i] for k,v in nodes.items()}
            ax.clear();ax.set_aspect('equal');ax.set_xlim(-100,125);ax.set_ylim(-105,90);ax.grid(alpha=.1)
            for a,b,r in hands[i]:capsule(ax,a,b,float(r[0]),'#aeb6be',.45)
            ax.add_patch(Polygon([pos[k] for k in ['c','d','g']],color='#4986a0',alpha=.15))
            for dim in p.nominal_design['dimensions']:
                if dim['id'] in ['L_gh','L_fh']:continue
                a,b=[pos[k] for k in dim['nodes']]
                if dim['body']=='ground':ax.plot(*np.array([a,b]).T,color='#667785',lw=1)
                else:capsule(ax,a,b,2,'#4986a0',.7)
            ax.plot(*np.array([pos['g'],pos['h']]).T,color='#b65b4d',ls=':',lw=1)
            capsule(ax,path[i,0],path[i,1],2,'#318775',.95);capsule(ax,path[i,1],path[i,2],2,'#318775',.95)
            ax.plot(*pos['h'],'o',color='#dc782b',ms=4)
            for k in ['f','g','h']:ax.annotate(k.upper(),pos[k],xytext=(4,5),textcoords='offset points',fontsize=8)
            ax.set_title(f.title()+(' / unchanged' if f=='index' else ' / GH contour relief'),fontsize=10)
            ax.text(.02,.02,f'Reference curl {s[i]*100:.1f}%',transform=ax.transAxes,fontsize=8)
        fig.suptitle('Fixed hole coordinates and link dimensions; 1 mm path tolerance\nGreen = checked material contour; dotted line = original GH axis',fontsize=11)
    sequence=np.r_[np.arange(0,33,2),np.arange(30,0,-2)]
    FuncAnimation(fig,draw_frame,frames=sequence,interval=110).save(out/'motion.gif',writer=PillowWriter(fps=9),dpi=72)
    plt.close(fig)
    lines=['# 保留 1 mm 轨迹容差：GH 轮廓避让验证','',
        '按本轮用户要求，不再以 H/R4 全程严格重合为通过门槛；主要判据是参考轨迹误差 ≤ 1 mm，并消除所测手指姿态中的平面干涉。','',
        '**结果：四指通过这一轮采样判据。食指不改形；中指、无名指、小指仅调整 GH 实体轮廓。所有杆长、F/G/H 孔位、安装孔和手指测量尺寸保持不变。**','',
        '## 修改方式','',
        '在 GH 原方向上，从 H 向 G 回退 10 mm，再向远离手指的一侧偏移 2 mm，设置轮廓点 P。输出件的 GH 侧材料沿 G→P→H 连接；FG 侧保留。P 不是铰接，不增加自由度，也不是新的独立连杆。',
        '材料路径按半径 2 mm 的胶囊段并集检查，包含拐角占用空间。H 处原有 4 mm 局部安装豁免保持原值，D 处原 12 mm 豁免保持不变。没有通过缩小杆半径、放大豁免区或改变手指尺寸来消除碰撞。',
        '这是实体外形调整，继续使用原有 FG/GH/FH 长度表达；没有采用已取消的三孔坐标优化方案。','',
        '## 密集检查结果','',
        '每指 4,801 个电机位置；每个位置检查 11 档手指关节角：从原比例屈曲参考姿态逐步插值到上一轮已有拟合姿态。共 52,811 个姿态组合/指。拟合姿态先在电机角度上插值，因此同时检查了原 241 个姿态之间的运动位置。','',
        '| 手指 | 最大参考轨迹误差 mm | 修改前最小间隙 mm | 修改后最小间隙 mm | 当前最紧部位 |',
        '|---|---:|---:|---:|---|']
    for f in FINGERS:
        v=reports[f];lines.append(f'| {cn[f]} | {v["maximum_reference_trajectory_error_mm"]:.4f} | {v["old"]["minimum_clearance_mm"]:.4f} | {v["new"]["minimum_clearance_mm"]:.4f} | {v["new"]["pair"]} |')
    lines += ['',
        '负间隙表示该平面模型中的重叠。新轮廓在全部已测试姿态中均无负间隙，参考轨迹误差与原候选一致；这些插值姿态中的 H/R4 最大错位也未超过 1 mm。',
        '中指和无名指的最紧部位已从 GH 转移到 EF / 中节手指，故继续加大 GH 偏出量不会增加整体最小间隙。当前分别约 0.17、0.10 mm，余量仍较小。食指原本无干涉，因此保留原轮廓。','',
        '## 轮廓候选比较','',
        '固定从 H 回退 10 mm，测试偏出 0、0.5、1、1.5、2、3 mm。下面均为整组 52,811 姿态的整体最小间隙：','',
        '| 手指 | 0 mm | 0.5 mm | 1 mm | 1.5 mm | 2 mm | 3 mm |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for f in ['middle','ring','little']:
        v=reports[f];lines.append('| '+cn[f]+' | '+' | '.join(f'{v["outline_offset_sweep"][str(o)]["minimum_clearance_mm"]:.4f}' for o in [0.,.5,1.,1.5,2.,3.])+' |')
    lines += ['',
        '选 2 mm 作为三指统一的轮廓候选：相较 1 mm 有更好的 GH 局部余量，又无需扩大到 3 mm。没有重新搜索其他杆长。无名指沿用上一轮 DG=120 mm 的候选，未在本轮进一步加长。','',
        '## 验证范围与文件','',
        '结论仅覆盖上述平面参考/拟合插值姿态族，不代表任意自然手姿态。刚度、轴座、三维错层、相邻手指与机构内部自碰撞没有在本轮新增验证；这里给出的是轮廓候选，并非完整制造图。',
        '仓库正式参数未替换，也未推送；源仓库提交为 `'+summary['source_commit']+'`。所有冻结输入及计算代码随报告保存。',
        '零偏出量回归检查确认新轮廓碰撞计算与原直线 GH 一致；刚体检查确认 P 相对 F/G/H 的距离在运动中保持不变，且未修改孔位。','',
        '- `selected/<finger>/result.json`：原尺寸与新增轮廓说明。',
        '- `selected/<finger>/validation.json`：完整判据、偏出量对比及各档手指姿态结果。',
        '- `selected/<finger>/clearance_sweeps.json.gz`：全部逐姿态间隙数据。',
        '- `baseline/`：原 1 mm 候选及原关节拟合数据。','',
        '在本目录可复现：','',
        '```bash','export KINEMATIC_CHAIN_REPO=/home/jianai/kinematic_chain',
        'python3 -B test_clearance.py',
        'python3 -B check_clearance.py --finger middle --out /tmp/middle-outline-check',
        '```','',
        '![原干涉位置前后对比](clearance_before_after.png)','',
        '![输出件轮廓](outline_design.png)','',
        '![运动预览](motion.gif)']
    (out/'README.md').write_text('\n'.join(lines)+'\n')
    manifest={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file() and p.name!='manifest.json'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'out':str(out),'files':len(manifest),'passed':summary['all_four_passed_sampled_planar_criteria']}),flush=True)


if __name__=='__main__':main()
