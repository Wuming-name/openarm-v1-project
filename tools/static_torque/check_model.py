#!/usr/bin/env python3
"""Offline MuJoCo model inspection. Does not import/open CAN or enable motors."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True,type=Path)
    parser.add_argument('--snapshot',type=Path)
    args=parser.parse_args()
    model_path=args.model.expanduser().resolve(strict=True)
    root=ET.parse(model_path).getroot()
    compiler=root.find('compiler')
    meshdir=compiler.get('meshdir','') if compiler is not None else ''
    missing=[]
    for mesh in root.findall('./asset/mesh'):
        if mesh.get('file'):
            mesh_path=Path(mesh.get('file'))
            if not mesh_path.is_absolute(): mesh_path=model_path.parent/meshdir/mesh_path
            if not mesh_path.is_file(): missing.append(str(mesh_path))
    if missing:
        raise ValueError('Missing meshes:\n'+'\n'.join(missing))
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(model_path))
    data=mujoco.MjData(model)
    print('MuJoCo:',mujoco.__version__)
    print('Model:',model_path)
    print('XML SHA256:',hashlib.sha256(model_path.read_bytes()).hexdigest())
    print(f'nq={model.nq}, nv={model.nv}, njnt={model.njnt}, nu={model.nu}')
    if (model.nq,model.nv,model.njnt)!=(7,7,7):
        raise ValueError('Expected fixed-base seven-hinge right arm')
    names=[f'openarm_right_joint{i}' for i in range(1,8)]
    ids=[]
    for i,name in enumerate(names,1):
        jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
        if jid<0 or model.jnt_type[jid]!=mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f'Missing/nonhinge {name}')
        ids.append(jid)
        low,high=model.jnt_range[jid]
        dof=int(model.jnt_dofadr[jid])
        print(f'J{i} {name}: model_range=[{low:.6f}, {high:.6f}] rad '
              f'([{math.degrees(low):.2f}, {math.degrees(high):.2f}] deg); '
              f'damping={model.dof_damping[dof]:.4g}; frictionloss={model.dof_frictionloss[dof]:.4g}')
    if args.snapshot:
        snapshot=json.loads(args.snapshot.read_text(encoding='utf-8-sig'))
        if snapshot.get('schema')!='openarm-snapshot-v1': raise ValueError('Invalid snapshot schema')
        q=snapshot['q_motor_rad']
        if len(q)!=7 or any(type(v) not in (int,float) or not math.isfinite(v) for v in q):
            raise ValueError('Snapshot must contain seven finite angles')
        print('Evaluating recorded angles, assuming signs +1 and zero offsets 0 (NOT calibrated).')
    else:
        q=[0.0]*7
        print('Evaluating MODEL zero angles; this is NOT a command to the real arm.')
    for i,jid in enumerate(ids): data.qpos[int(model.jnt_qposadr[jid])]=q[i]
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    print('Gravity world vector:',list(model.opt.gravity))
    print('Model contacts at this pose:',data.ncon)
    for i,jid in enumerate(ids,1):
        dof=int(model.jnt_dofadr[jid])
        print(f'J{i}: q_model={q[i-1]:.6f} rad; gravity_bias={data.qfrc_bias[dof]:.6f} Nm; '
              f'passive={data.qfrc_passive[dof]:.6f} Nm')
    for ci in range(data.ncon):
        contact=data.contact[ci]
        geom_names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in (contact.geom1,contact.geom2)]
        print('Contact:',geom_names,'distance=',float(contact.dist))
    print('LOAD CHECK COMPLETE. No CAN access. This is not motion approval or a physical mass/zero calibration.')
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (ValueError,OSError,ImportError,KeyError,TypeError) as exc:
        print('ERROR:',exc,file=sys.stderr)
        raise SystemExit(1)
