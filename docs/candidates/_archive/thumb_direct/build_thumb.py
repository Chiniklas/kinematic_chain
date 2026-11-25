"""Rebuild a true B-rep thumb linkage prototype and STEP assemblies, in mm.

Requires cadquery==2.6.1, numpy, matplotlib. See README.md for the source
geometry and the provisional mounting / hardware assumptions.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np

ROOT=Path(__file__).resolve().parent
P=json.loads((ROOT/'parameters.json').read_text(encoding='utf-8'))
L=P['lengths']
TILT=P['fixed']['ad_tilt_deg']
GAP=P['fixed']['dorsal_clearance_mm']
S=P['plates']
HOLES=P['reference_holes']
ASSUME=P['design_assumptions']
PALETTE={'ground':'#aeb5bf','AB':'#426da9','BC':'#74a2b5','CDG':'#ced4dc',
         'EF':'#e4ac62','FGH':'#ce714d','HR4':'#7579b7','saddle':'#b18dc3',
         'servo':'#e7b633','dark':'#3c4149','metal':'#8d97a0','brass':'#b5a071','hand':'#d5b79b'}


def circles(a,ra,b,rb):
    a=np.array(a);b=np.array(b);d=np.linalg.norm(b-a)
    if d<1e-10 or d>ra+rb+1e-8 or d<abs(ra-rb)-1e-8:
        raise ValueError('unassemblable linkage geometry')
    u=(b-a)/d;x=(ra*ra-rb*rb+d*d)/(2*d)
    h=math.sqrt(max(0,ra*ra-x*x));v=np.array([-u[1],u[0]])
    return (a+x*u+h*v,a+x*u-h*v)


def mechanism(q_values):
    from optimize_direct import poses,R
    x=np.array(P['direct_synthesis'])
    s=np.deg2rad(np.asarray(q_values))/x[-1]
    raw,viol=poses(x,s)
    if np.max(viol)>1e-8: raise ValueError('Unassemblable direct linkage')
    return [{k:raw[k][i]@R for k in raw} for i in range(len(s))]


def finger(progress):
    from optimize_direct import poses,ik
    points,viol=poses(np.array(P['direct_synthesis']),np.array([progress]))
    m,ip,r=ik(points['h'])
    if not 10 < r[0] < 50: raise ValueError('Thumb closure outside reach')
    m=math.radians(float(m[0]));dist=m+math.radians(float(ip[0]))
    p0=np.array([0.,-GAP]);p1=p0+30*np.array([math.cos(m),-math.sin(m)])
    p2=p1+40*np.array([math.cos(dist),-math.sin(dist)])
    return p0,p1,p2,(p1+p2)/2,(-m,-dist)


def box(dx,dy,dz,center):
    return cq.Workplane('XY').box(dx,dy,dz).translate(tuple(center))


def cylinder(x,y,z,r,t):
    return cq.Workplane('XY',origin=(x,y,z)).circle(r).extrude(t)


def capsule(a,b,r,z,t):
    a=np.asarray(a);b=np.asarray(b);length=float(np.linalg.norm(b-a))
    angle=math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
    middle=(a+b)/2
    body=box(length,2*r,t,(0,0,z+t/2)).rotate((0,0,0),(0,0,1),angle).translate((middle[0],middle[1],0))
    return body.union(cylinder(*a,z,r,t)).union(cylinder(*b,z,r,t)).clean()


def drill(part,point,diameter,z,t):
    return part.cut(cylinder(*point,z-1,diameter/2,t+2)).clean()


def chain(points,z,t,r=4):
    part=capsule(points[0],points[1],r,z,t)
    for a,b in zip(points[1:-1],points[2:]):
        part=part.union(capsule(a,b,r,z,t))
    return part.clean()


def tilt(part):
    return part.rotate((0,0,0),(0,0,1),-TILT)


def make_hardware(points,progress):
    parts=[]
    def add(name,shape,color,kind,info=''):
        assert shape.val().isValid(),f'invalid B-rep: {name}'
        assert len(shape.solids().vals())==1,f'{name} is not one connected solid'
        parts.append(dict(name=name,shape=shape,color=PALETTE[color],kind=kind,info=info))
    a,b,c,d,e,f,g,h=(points[k] for k in 'abcdefgh')
    p0,p1,p2,r4,headings=finger(progress)

    # New ADE ground/carrier: servo hole pattern is based on the measured STL.
    ax=-L['L_ad']
    a0=np.array([ax,0.]);d0=np.zeros(2)
    e0=max(circles(a0,L['L_ae'],d0,L['L_de']),key=lambda q:q[1])
    ground=box(40,27,3,(ax-7.5,0,1.5)).edges('|Z').fillet(2)
    ground=ground.union(chain([a0,d0,e0],0,3,r=5)).union(capsule(e0,a0,5,0,3))
    ground=drill(ground,a0,ASSUME['ground_horn_clearance_diameter'],0,3)
    for point in (d0,e0):
        ground=drill(ground,point,4.05,0,3)
    mounts=[(ax+x,y) for x in (-22.5,7.5) for y in (-8,8)]
    for point in mounts:
        ground=drill(ground,point,2.5,0,3)
    # Orthogonal palm hinge, retained as a free-position mounting interface.
    hx,hz=ax-38,-12
    rotor=chain([(hx,hz),(ax-27,1.5)],0,8,r=3)
    rotor=rotor.union(cylinder(hx,hz,0,5,8))
    rotor=drill(rotor,(hx,hz),3.05,0,8)
    rotor=rotor.rotate((0,0,0),(1,0,0),90).translate((0,4,0))
    ground=ground.union(rotor).clean()
    add('01_servo_carrier_ADE_and_hinge_rotor',tilt(ground),'ground','custom',
        'new ADE hole pattern; provisional orthogonal palm-hinge interface')

    ab=capsule(a,b,3,S['AB']['z'],S['AB']['thickness']).union(cylinder(*a,S['AB']['z'],8,2.5))
    ab=drill(ab,b,3.05,S['AB']['z'],2.5)
    u=(b-a)/np.linalg.norm(b-a);v=np.array([-u[1],u[0]])
    for vector,diameter in [(v*6,2.5),(-v*6,2.5),(u*6,2),(-u*6,2)]:
        ab=drill(ab,a+vector,diameter,S['AB']['z'],2.5)
    add('02_input_AB_servo_disc',ab,'AB','custom','AB pitch; reference horn pattern radius 6 mm')
    bc=capsule(b,c,4,S['BC']['z'],3)
    bc=drill(drill(bc,b,3.05,S['BC']['z'],3),c,4.05,S['BC']['z'],3)
    add('03_coupler_BC',bc,'BC','custom')
    for name,nodes,color in [('CDG',[d,c,g],'CDG'),('EF',[e,f],'EF'),('FGH',[f,g,h],'FGH')]:
        spec=S[name];part=chain(nodes,spec['z'],spec['thickness'])
        for point in nodes:
            part=drill(part,point,4.05,spec['z'],spec['thickness'])
        add({'CDG':'04_rocker_CDG','EF':'05_coupler_EF','FGH':'06_output_FGH','HR4':'07_output_rod_HR4'}[name],
            part,color,'custom')

    inner_half=ASSUME['distal_saddle_inner_width']/2
    outer_half=inner_half+2.5
    cuff_length=ASSUME['distal_saddle_length']
    saddle=box(cuff_length,2.5,2*outer_half,(0,2.25,0))
    for z in (-(inner_half+1.25),inner_half+1.25):
        saddle=saddle.union(box(cuff_length,15.5,2.5,(0,-4.25,z)))
    tab_z=ASSUME['distal_saddle_tab_z']
    tab_top=tab_z+2.5
    web_start=outer_half-.1
    saddle=saddle.union(box(8,3,tab_top-web_start,(0,-6.5,(tab_top+web_start)/2)))
    saddle=saddle.union(cylinder(0,0,tab_z,6,2.5))
    saddle=drill(saddle,(0,0),4.05,tab_z,2.5)
    saddle=saddle.cut(box(8,2,40,(0,-10,0)))
    saddle=saddle.rotate((0,0,0),(0,0,1),math.degrees(headings[-1])).translate((*r4,0))
    add('08_distal_saddle_R4',saddle,'saddle','custom','22 mm provisional inner width; strap slots')

    # Motor is a rebuilt envelope, not a catalog solid or mesh-to-STEP conversion.
    motor=box(34,20,23,(ax-7.5,0,-14.5)).edges('|Z').fillet(.8)
    motor=motor.union(cylinder(ax,0,-3,7,3)).union(cylinder(ax,0,-.1,3.2,3.6))
    for point in mounts:
        motor=drill(motor,point,1.6,-26,23)
    add('09_DC15_A01_reference_envelope',tilt(motor),'servo','reference_envelope',
        '23 mm axial x 34 x 20 mm envelope measured from source STL; not exact case detail')
    horn=cylinder(*a,.5,8,2.5)
    horn=drill(horn,a,6.5,.5,2.5)
    for vector in (v*6,-v*6,u*6,-u*6):
        horn=drill(horn,a+vector,2,.5,2.5)
    add('10_servo_horn_reference',horn,'dark','reference_envelope','simplified horn, spline omitted')

    clevis=box(3,22,20,(hx-10.5,0,hz))
    for y in (-6,6):
        ear=cq.Workplane().newObject([cq.Solid.makeCylinder(6,3,cq.Vector(hx,y-1.5,hz),cq.Vector(0,1,0))])
        ear=ear.union(box(9,3,8,(hx-4.5,y,hz)))
        bore=cq.Solid.makeCylinder(1.525,5,cq.Vector(hx,y-2.5,hz),cq.Vector(0,1,0))
        clevis=clevis.union(ear.cut(bore))
    for y in (-7,7):
        bore=cq.Solid.makeCylinder(1.6,5,cq.Vector(hx-13,y,hz),cq.Vector(1,0,0))
        clevis=clevis.cut(bore)
    add('11_palm_hinge_clevis_interface',tilt(clevis),'ground','custom',
        'provisional 3 mm hinge pin and two 3.2 mm mounting holes; palm placement undecided')
    hinge_pin=cq.Workplane().newObject([cq.Solid.makeCylinder(1.5,18,cq.Vector(hx,-9,hz),cq.Vector(0,1,0))])
    add('12_palm_hinge_pin',tilt(hinge_pin),'metal','hardware_placeholder')

    joints=[('B',b,3,3.2,19.1),('C',c,4,12.5,19.1),('D',d,4,-.3,15.5),
            ('E',e,4,-.3,9.3),('F',f,4,6.3,22.8),('G',g,4,12.5,22.8),
            ('H_direct_R4',h,4,19.7,25.8)]
    for name,point,diam,z0,z1 in joints:
        add('pin_'+name,cylinder(*point,z0,diam/2,z1-z0),'metal','hardware_placeholder')
    for name,point,z0,z1 in [('D',d,3.2,12.6),('E',e,3.2,6.4),('G',g,15.5,19.8)]:
        spacer=drill(cylinder(*point,z0,3.4,z1-z0),point,4.1,z0,z1-z0)
        add('spacer_'+name,spacer,'brass','hardware_placeholder')
    for i,(x,y) in enumerate(mounts):
        spacer=drill(cylinder(x,y,-3,2.25,3),[x,y],2.5,-3,3)
        add(f'motor_standoff_{i+1}',tilt(spacer),'metal','hardware_placeholder')
    return parts


def context(progress):
    p0,p1,p2,_,headings=finger(progress)
    result=[]
    for name,a,b,depth,angle in [('proximal',p0,p1,20,headings[0]),('distal',p1,p2,10,headings[1])]:
        normal=np.array([-math.sin(angle),math.cos(angle)])
        offset=-depth/2*normal
        shape=capsule(a+offset,b+offset,depth/2,-10,20)
        result.append(dict(name='REFERENCE_'+name,shape=shape,color=PALETTE['hand'],kind='human_context'))
    return result


def assembly(parts,name):
    assy=cq.Assembly(name=name)
    for part in parts:
        color=part['color'].lstrip('#')
        rgb=[int(color[i:i+2],16)/255 for i in (0,2,4)]
        assy.add(part['shape'],name=part['name'],color=cq.Color(*rgb))
    return assy


def main():
    (ROOT/'parts').mkdir(exist_ok=True)
    poses=mechanism(P['q_schedule_deg'])
    states={'straight':0,'half_curl':min(range(len(poses)),key=lambda i:abs(P['hand_progress'][i]-.5)),
            'full_curl':len(poses)-1}
    report={'units':'mm','reference_commit':P['reference']['commit'],'states':{},'parts':[]}
    for state,index in states.items():
        parts=make_hardware(poses[index],P['hand_progress'][index])
        assy=assembly(parts,'Thumb_'+state)
        assy.save(str(ROOT/f'thumb_{state}.step'),'STEP',mode='default')
        combined=assembly(parts+context(P['hand_progress'][index]),'Thumb_'+state+'_with_reference')
        combined.save(str(ROOT/f'thumb_{state}_with_reference.step'),'STEP',mode='default')
        error=np.linalg.norm(poses[index]['h']-finger(P['hand_progress'][index])[3])
        report['states'][state]={'q_deg':P['q_schedule_deg'][index],'curl_progress':P['hand_progress'][index],
                                 'closure_error_mm':float(error),'hardware_solids':len(parts)}
        if state=='straight':
            for part in parts:
                cq.exporters.export(part['shape'],str(ROOT/'parts'/f'{part["name"]}.step'))
                report['parts'].append({k:v for k,v in part.items() if k!='shape'} |
                                       {'volume_mm3':part['shape'].val().Volume(),'valid':part['shape'].val().isValid()})
            assembly(parts,'Thumb_straight').save(str(ROOT/'thumb_straight.glb'),'GLTF',tolerance=.08,angularTolerance=.15)
        from render_thumb import render
        render(parts+context(P['hand_progress'][index]),ROOT/f'preview_{state}.png',state)
        if state=='straight':
            render(parts,ROOT/'preview_exploded.png','exploded',exploded=True)
        # STEP round trip: verify solids survive as real valid B-reps.
        loaded=cq.importers.importStep(str(ROOT/f'thumb_{state}.step'))
        assert len(loaded.solids().vals())==len(parts)
        assert all(s.isValid() for s in loaded.solids().vals())
    (ROOT/'cad_validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    with (ROOT/'parts_list.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['name','kind','info','volume_mm3','valid','color'])
        writer.writeheader();writer.writerows(report['parts'])
    print(json.dumps(report['states'],indent=2),flush=True)


if __name__=='__main__':
    main()
