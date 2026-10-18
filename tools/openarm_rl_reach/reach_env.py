import numpy as np
import gymnasium as gym
from gymnasium import spaces
from core import Physics, Reference, observation, vec, DT, DECIMATION

class ReachEnv(gym.Env):
    metadata = {"render_modes": []}
    def __init__(self, config, randomize=True):
        self.c = config
        self.p = Physics(config)
        self.randomize = randomize
        self.action_space = spaces.Box(-1,1,(7,),np.float32)
        self.observation_space = spaces.Box(-np.inf,np.inf,(34,),np.float32)
        self.goal = self.p.fk(vec(config["goal_q"]))
        self.p.check_path(config["home"],config["goal_q"])
        if np.linalg.norm(self.goal-self.p.fk(config["home"]))<2*config["success_distance"]:
            raise ValueError("Goal too close to home: choose a meaningful goal or a reviewed tighter success tolerance")

    def obs(self):
        q,dq = self.p.state()
        if self.randomize:
            q=q+self.np_random.normal(0,.0003,7)
            dq=dq+self.np_random.normal(0,.001,7)
        return observation(self.c,q,dq,self.ref.q,self.previous,self.goal,self.p.fk(q))

    def reset(self,seed=None,options=None):
        super().reset(seed=seed)
        jitter = self.c["reset_jitter"] if self.randomize else 0
        q=vec(self.c["home"])+self.np_random.uniform(-jitter,jitter,7)
        self.p.check_pose(q)
        self.p.reset(q,self.np_random if self.randomize else None)
        self.ref=Reference(self.c,vec(self.c["home"]))
        # Settle the same position servo used during real homing before exposing policy.
        for _ in range(50):self.p.advance(self.ref.q)
        self.previous=np.zeros(7);self.steps=0;self.stable=0
        self.distance=float(np.linalg.norm(self.goal-self.p.fk(q)))
        return self.obs(),{}

    def step(self,action):
        action=np.clip(vec(action),-1,1)
        old=self.previous.copy();fault=""
        try:
            for _ in range(DECIMATION):
                q,_=self.p.state()
                self.p.check_pose(q)
                before=self.ref.q.copy()
                target=self.ref.step(action)
                self.p.check_path(before,target)
                self.p.advance(target)
        except ValueError as e:
            fault=str(e)
        self.steps+=1
        q,dq=self.p.state()
        distance=float(np.linalg.norm(self.goal-self.p.fk(q)))
        # Policy must stop both actual motion and its reference before declaring success.
        settled=(distance<self.c["success_distance"] and np.max(np.abs(dq))<self.c["success_speed"]
                 and np.max(np.abs(self.ref.v))<.002 and not fault)
        self.stable=self.stable+1 if settled else 0
        success=self.stable>=10
        reward=30*(self.distance-distance)-2*distance-.02*float(dq@dq)-.03*float(np.sum((action-old)**2))
        reward-=.01*float(np.sum((q-vec(self.c["home"]))**2))
        if fault:reward-=20
        if success:reward+=10
        self.previous=action;self.distance=distance
        info={"is_success":success,"distance_m":distance,"fault":fault}
        return self.obs(),reward,bool(success or fault),self.steps*DT*DECIMATION>=self.c["episode_seconds"],info
