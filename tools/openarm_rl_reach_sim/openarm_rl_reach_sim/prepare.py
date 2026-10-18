"""Prepare the agreed A->B task. Simulation only; never imports hardware tools."""
import argparse
from pathlib import Path
import numpy as np
from core import load,read,save,Physics,vec

HERE=Path(__file__).absolute().parent
def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--model',type=Path,default=HERE.parent.parent/'models/openarm_v1_right/openarm_v1_7dof.xml')
    a.add_argument('--config',type=Path,default=Path('config.sim.json'))
    args=a.parse_args()
    if args.config.exists():raise ValueError('Config already exists; not overwritten. Use preflight.py to recheck.')
    c=read(HERE/'config.template.json');c['model']=str(args.model.expanduser().absolute())
    p=Physics(c);p.check_path(c['home'],c['goal_q'])
    print('A joint radians:',c['home']);print('B joint radians:',c['goal_q'])
    print('A position m:',p.fk(c['home']));print('B position m:',p.fk(c['goal_q']))
    print('Nominal displacement m:',np.linalg.norm(p.fk(c['home'])-p.fk(c['goal_q'])))
    save(args.config,c);load(args.config)
    print('Created',args.config,'SIMULATION ONLY. Next run preflight.py.')
if __name__=='__main__':main()
