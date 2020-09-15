import sys
import time

from stable_baselines import PPO2
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv

from tensegrity_env_d_ppo2 import TensegrityEnvDeltaPpo2
import tensegrity_env_base as teb
import common as cm

MAX_CPU = 16 # cap it per current aws instance type

#STEPS = 10000
STEPS = 50000
#STEPS = 100000
#STEPS = 200000
#STEPS = 2000000
#DIMENSION = 16
#DIMENSION = 32
#DIMENSION = 64
#DIMENSION = 128

def callback(lcl, glb):
    return False

def train(env, file):
    start = time.time()
    #env.setRender(False)
    
    # create the learning agent
    model = PPO2(
        #tensorboard_log=saver.data_dir,
        policy=MlpPolicy,
        #policy_kwargs=dict(net_arch=[dict(pi=[DIMENSION, DIMENSION], vf=[DIMENSION, DIMENSION])]),
        policy_kwargs=dict(net_arch=[64, 64, 32]),
        env=env,
        gamma=0.998,
        #n_steps=1000,
        n_steps=300,
        ent_coef=0,
        learning_rate=1e-3,
        vf_coef=0.5,
        max_grad_norm=0.5,
        lam=0.95,
        nminibatches=10,
        noptepochs=10,
        cliprange=0.2,
        verbose=1,
    )
        
    # train the agent on the environment
    model.learn(
        #total_timesteps=10000000,
        total_timesteps=STEPS,
        log_interval=10,
        #log_dir=".",
        #record_video=False
    )

    # save trained model
    model.save(teb.PATH_POLICY + file)
    print("Duration: %.1f" % ((time.time() - start)/60))

def main(n_cpu):
    env = SubprocVecEnv([lambda: TensegrityEnvDeltaPpo2(cm.TRAIN) for i in range(n_cpu)])
    train(env, teb.NAME_POLICY)

if __name__ == '__main__':
    n_cpu = 2
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        arg = int(arg)
        if arg > MAX_CPU:
            arg = MAX_CPU
        n_cpu = arg
    main(n_cpu)

