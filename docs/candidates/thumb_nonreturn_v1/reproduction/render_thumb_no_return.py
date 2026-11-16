import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from matplotlib.font_manager import FontProperties
import thumb_no_return as n
from render_thumb_experiments import capsule,raster,gif,COLORS
OUT=n.OUT;FONT=FontProperties(fname=r'C:\Windows\Fonts\msyh.ttc');plt.rcParams['axes.unicode_minus']=False
TITLES={'baseline':'原方案：红色件折返','straight':'方案1：直杆，保留 A/D','gentle':'方案2：150°～180°，保留 A/D','relayout':'方案3：直杆，重新布置 D/E'}
FAIL={'trajectory':'轨迹误差超限','monotone_mcp':'MCP出现反向运动','monotone_ip':'IP出现反向运动','length_bounds':'孔距超出8～85mm',
 'initial_HG_direction':'初始GH方向/指尖方向投影不足要求','all_pose_HG_direction':'GH全程佩戴朝向不满足',
 'vibration_and_strap_reserve':'模块/绑带占位间隙不足2mm','layered_clearance':'简化杆销/手指间隙不足0.1mm',
 'assembly':'闭环无解','rigid_lengths':'刚体孔距不守恒','direct_closure':'H连接不闭合','endpoint':'屈曲末端角超差',
 'continuous_branches':'装配分支变化','intersection_margin':'闭环接近相切','no_return':'红色件角度不达标','ground_bounds':'固定点移动超限'}

def draw(ax,rec,s):
    baseline=rec['kind']=='baseline'
    if baseline:pts,_,m,ip,_=n.t.geometry(n.t.X0,np.array([s]))
    else:pts,_,m,ip,_=n.geometry(np.array(rec['z']),np.array([s]))
    pts={k:p[0] for k,p in pts.items()};m=float(m[0]);ip=float(ip[0]);a,b=np.radians([m,ip])
    root=np.array([0.,-7.]);joint=root+30*np.array([np.cos(a),-np.sin(a)]);tip=joint+40*np.array([np.cos(a+b),-np.sin(a+b)])
    for p0,p1,angle,depth in [(root,joint,-a,20),(joint,tip,-a-b,10)]:
        normal=np.array([-np.sin(angle),np.cos(angle)]);capsule(ax,p0-depth/2*normal,p1-depth/2*normal,depth/2,'#d3bda5',alpha=.5,z=0)
    tangent=np.array([np.cos(a+b),-np.sin(a+b)]);normal=np.array([np.sin(a+b),np.cos(a+b)])
    def box(coords):
        x0,x1,y0,y1=coords;return [pts['h']+x*tangent+y*normal for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]]
    ax.add_patch(Polygon(box(n.v.MODULE),facecolor='#7c6cbd',edgecolor='#48347b',lw=1,zorder=7))
    ax.add_patch(Polygon(box(n.v.BELT),fill=False,edgecolor='#e2932f',linestyle='--',lw=1.3,zorder=9))
    for name,nodes,r,z in [('ADE','adea',3,1),('AB','ab',3,2),('EF','ef',4,3),('CDG','dcg',4,4),('BC','bc',4,5),('FGH','fgh',4,6)]:
        for p0,p1 in zip(nodes,nodes[1:]):capsule(ax,pts[p0],pts[p1],r,COLORS[name],z=z)
    for k,p in pts.items():
        ax.add_patch(Circle(p,1.5,facecolor='white',edgecolor='#334155',lw=.7,zorder=10))
        ax.annotate(k.upper(),p,xytext=(4,5),textcoords='offset points',fontsize=8,zorder=11)
    track=n.model.target(np.linspace(0,1,101))@n.model.R;ax.plot(*track.T,':',color='#64748b',lw=.9)
    status='参考' if baseline else ('通过简化筛选' if rec['passed'] else '未通过：仅展示失败尝试')
    ax.set_title(TITLES[rec['kind']]+'\n'+status,fontproperties=FONT,fontsize=12,pad=9,color='#475569' if baseline else '#166534' if rec['passed'] else '#b91c1c')
    ax.set_xlim(112,-45);ax.set_ylim(-90,80);ax.set_aspect('equal');ax.grid(alpha=.12);ax.tick_params(labelsize=7)
    ax.text(.5,.025,'MCP %.1f° / IP %.1f°   电机 %.1f°'%(m,ip,rec['motor_travel_deg']*s),transform=ax.transAxes,ha='center',fontproperties=FONT,fontsize=9,bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
    note='FGH %.1f°'%rec['FGH_angle_deg']
    if not baseline:note+='   最大轨迹误差 %.2f mm'%rec['trajectory_max_error_mm']
    ax.text(.5,.96,note,transform=ax.transAxes,ha='center',fontproperties=FONT,fontsize=9)

def reports(records):
    config={'scope':'red rigid F-G-H output part; does not prohibit every projected crossing of the whole mechanism',
     'parameterization':'G=H+GH*u(phi), F=G+FG*u(phi+bend); positive GH and FG; bend=0 or bounded +/-30deg',
     'straight':{'bend_deg':0,'ground':'A,D fixed; E displacement <=5mm'},
     'gentle':{'bend_deg':[-30,30],'ground':'A,D fixed; E displacement <=5mm'},
     'relayout':{'bend_deg':0,'ground':'A fixed; D displacement <=25mm, E displacement <=50mm; existing connectivity retained'},
     'common':{'initial_HG_deviation_deg_max':15,'all_pose_HG_deviation_deg_max':25,'initial_HG_forward_projection_mm_min':3,
       'all_pose_HG_dorsal_projection_mm_min':3,'hole_center_distances_mm_range':[8,85],
       'target_trajectory_max_error_mm':1,'MCP_IP_endpoint_tolerance_deg':.25,'minimum_circle_intersection_sine':.04,
       'sampled_layered_clearance_gate_mm':.1,'sampled_layered_clearance_optimization_target_mm':.3,
       'module_strap_reserve_gate_mm':2,'verification_samples':5001,
       'module_dimensions_assumed_mm':[14,12,6],'module_location':'distal palmar pad midpoint',
       'strap_width_assumed_mm':8,'strap_thickness_assumed_mm':1,'thumb_phalanges_mm':[30,40]},
     'validation_exclusions':'full motor/carrier/cuff solids, actual strap routing and forces, screw heads, tolerances, continuous swept-volume CAD',
     'search':'deterministic multistart bounded nonlinear least squares; failure to find a valid solution is not proof of impossibility'}
    (OUT/'constraints.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    lines=['禁止红色件折返：三组实验结果','',
      '范围：红色刚体F-G-H本身不再出现锐角回钩；没有要求整套机构所有杆件在二维投影中都不交叉。',
      '方案1、2固定A/D，E最多移动5mm；方案3固定电机A，允许D移动25mm、E移动50mm。',
      '方案3本次采用重新布置固定铰点，保留原连接拓扑；不是已完成的新CAD结构。',
      '采用正长度+角度参数化，方案1/3强制F-G-H共线且G在中间；方案2夹角硬限制150～180度。',
      '取消旧GF方向限制，保留初始HG方向偏差≤15度、全程≤25度以及指背侧要求。',
      '原模型15～170度的FGH形状筛选不再用于直杆；180度刚体本身不构成闭环圆交点奇异，单独检查BCD/EFG闭环。',
      '模块：远节指腹，绑带连到H附近。14×12×6mm、绑带宽8mm厚1mm均为未确认占位尺寸。','']
    for r in records[1:]:
        lines += [TITLES[r['kind']], '结果：'+('通过5001姿态的本轮简化筛选' if r['passed'] else '本轮搜索未找到通过全部筛选的解，以下为代表失败尝试'),
          '搜索起点数：%d；选中起点编号：%d'%(r['trials'],r['selected_trial']),
          '失败项：'+('无' if r['passed'] else '、'.join(FAIL.get(k,k) for k,val in r['checks'].items() if not val)),
          'FGH夹角：%.6f度；全程最小角：%.6f度'%(r['FGH_angle_deg'],r['minimum_FGH_angle_deg']),
          '最大轨迹误差：%.6fmm；末端MCP/IP：%s'%(r['trajectory_max_error_mm'],r['endpoint_deg']),
          'MCP/IP相邻采样最小增量：%.9f / %.9f度'%(r['minimum_mcp_step_deg'],r['minimum_ip_step_deg']),
          'D位移：%.6fmm；E位移：%.6fmm'%(r['ground_D_shift_mm'],r['ground_E_shift_mm']),
          'GH初始/全程最大方向偏差：%.6f / %.6f度'%(r['initial_HG_deviation_deg'],r['maximum_HG_deviation_deg']),
          '最小模块/绑带投影间隙：%.6fmm（%s）'%(r['reserve_clearance_mm'],r['reserve_worst_pair']),
          '最小分层杆/销/人体间隙：%.6fmm（%s）'%(r['collision_screen']['minimum_clearance_mm'],r['collision_screen']['worst_pair']),
          '孔中心距(mm)：'+json.dumps(r['lengths_mm'],ensure_ascii=False),
          '节点运动包络(mm)：'+json.dumps(r['bounds_mm'],ensure_ascii=False),'']
    lines += ['结论：本轮方案3找到了红色件完全拉直的运动学候选。方案1/2未找到解不代表数学上不可能。',
      '方案3中最紧的约0.30mm是ADE支架与近节手指的简化间隙，不是模块间隙；不足以证明实际佩戴/加工可行。',
      '必须重新设计ADE支架，并对实际模块、绑带走线、完整舵机/固定座/螺钉、制造误差做三维校核后再考虑加工。',
      '失败方案动画展示的是可闭合但未满足全部目标的运动，不能作为可用设计。',
      '数据z为本实验专用13参数格式，与原项目11参数direct_synthesis不兼容，不能直接替换原参数文件。',
      'z顺序：AB长度、输入初角rad、Cx、Cy、Ex、Ey、GH长度、H→G初角rad、FG长度、弯折角rad、电机行程rad、Dx、Dy。',
      'z坐标使用原项目未旋转坐标系；所有图和world_pivots_mm使用原项目旋转后的世界坐标。']
    (OUT/'实验结果与说明.txt').write_text('\n'.join(lines),encoding='utf-8')
    for r in records[1:]:
        z=np.array(r['z']);pts=n.setup(z)
        p={'format':'thumb_nonreturn_experiment_v1_NOT_direct_synthesis','status':'preview_only' if r['passed'] else 'rejected_do_not_build',
         'z_parameter_order':['AB_mm','input_initial_rad','Cx','Cy','Ex','Ey','GH_mm','HG_initial_rad','FG_mm','bend_rad','motor_travel_rad','Dx','Dy'],
         'z':r['z'],'world_pivots_mm':{k:(q@n.model.R).tolist() for k,q in pts.items()},'lengths_mm':r['lengths_mm'],'validation':r,'constraints':config}
        (OUT/('parameters_'+r['kind']+'.json')).write_text(json.dumps(p,indent=2),encoding='utf-8')

def main():
    baseline=n.t.evaluate(n.t.X0,'baseline',n=5001);baseline['kind']='baseline'
    records=[baseline]
    for kind in n.NAMES:
        old=json.loads((OUT/(kind+'.json')).read_text());rec=n.evaluate(np.array(old['z']),kind,n=5001)
        rec.update(trials=old['trials'],selected_trial=old['selected_trial'],residual_norm=old['residual_norm'])
        # Never animate clipped nonassemblies as if they were a rigid mechanism.
        assert all(rec['checks'][key] for key in ['assembly','rigid_lengths','direct_closure','continuous_branches'])
        (OUT/(kind+'.json')).write_text(json.dumps(rec,indent=2),encoding='utf-8');records.append(rec)
    reports(records);allframes=[];comparison=[];solo={kind:[] for kind in n.NAMES}
    for i,s in enumerate(np.linspace(0,1,41)):
        fig,axs=plt.subplots(2,2,figsize=(12,12.5),dpi=110)
        fig.subplots_adjust(left=.04,right=.97,bottom=.075,top=.86,hspace=.28,wspace=.13)
        fig.suptitle('禁止红色件折返：原方案与三组实验',fontproperties=FONT,fontsize=19,y=.987)
        fig.text(.5,.945,'紫色：14×12×6 mm指腹模块占位；橙色虚线：绑带侧视投影',ha='center',fontproperties=FONT,fontsize=11)
        fig.text(.5,.925,'相同比例、相同行程百分比。红色“未通过”面板仅展示失败尝试。',ha='center',fontproperties=FONT,fontsize=11,color='#9f1239')
        for ax,rec in zip(axs.flat,records):draw(ax,rec,float(s))
        fig.text(.5,.035,'方案3需要重做ADE支架；仅简化运动/占位筛选，尚未完成真实模块及三维CAD校核。',ha='center',fontproperties=FONT,fontsize=11)
        frame=raster(fig);allframes.append(frame)
        if i in [0,20,40]:frame.save(OUT/(['start.png','middle.png','end.png'][i//20]))
        plt.close(fig)
        fig,axs=plt.subplots(1,2,figsize=(12,7.5),dpi=110);fig.subplots_adjust(left=.035,right=.98,bottom=.14,top=.8,wspace=.12)
        fig.suptitle('方案3：红色F–G–H完全拉直，重新布置固定铰点',fontproperties=FONT,fontsize=17,y=.98)
        fig.text(.5,.925,'左：原方案   右：直杆候选；模块为14×12×6 mm占位，尺寸未确认',ha='center',fontproperties=FONT,fontsize=11)
        draw(axs[0],records[0],float(s));draw(axs[1],records[-1],float(s))
        fig.text(.5,.065,'D移动 %.2f mm，E移动 %.2f mm；A电机位置保持。需要重做ADE支架。'%(records[-1]['ground_D_shift_mm'],records[-1]['ground_E_shift_mm']),ha='center',fontproperties=FONT,fontsize=11)
        fig.text(.5,.025,'红色件不折返 ≠ 整套机构二维投影没有交叉；尚未完成实物/CAD验证',ha='center',fontproperties=FONT,fontsize=10,color='#475569')
        comparison.append(raster(fig));plt.close(fig)
        for rec in records[1:]:
            fig,ax=plt.subplots(figsize=(7,7.5),dpi=100);fig.subplots_adjust(left=.08,right=.98,bottom=.16,top=.85);draw(ax,rec,float(s))
            fig.text(.5,.075,'紫色为指腹模块占位，橙色为绑带示意；真实尺寸未确认',ha='center',fontproperties=FONT,fontsize=10)
            fig.text(.5,.035,'需要重新设计支架及三维复核' if rec['passed'] else '未满足全部约束，不能作为可用设计',ha='center',fontproperties=FONT,fontsize=11,color='#92400e')
            solo[rec['kind']].append(raster(fig));plt.close(fig)
        if i%10==0:print('frame',i,flush=True)
    gif(allframes,OUT/'three_schemes.gif');gif(comparison,OUT/'original_vs_straight_relayout.gif')
    for kind,frames in solo.items():gif(frames,OUT/(kind+'.gif'))
    print('render complete',flush=True)
if __name__=='__main__':main()
