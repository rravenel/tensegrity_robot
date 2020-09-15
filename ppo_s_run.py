import sys
import time

from stable_baselines import PPO2

import tensegrity_env_s_ppo2 as tes
import tensegrity_env_base as teb
import common as cm


def main(arg):
    env = tes.TensegrityEnvSinePpo2(arg, render = arg == cm.RUN)
    model = PPO2.load(teb.PATH_POLICY + tes.NAME_POLICY)

    while True:
        obs, done = env.reset(), False
        
        running_reward = 0.0
        ep_len = 0
        for _ in range(10000):
            action, _ = model.predict(obs)
            
            obs, reward, done, infos = env.step(action)
            running_reward += reward
            ep_len += 1
            
            if done:
                print("Episode Reward: %.2f" % (running_reward))
                break
                
if __name__ == '__main__':
    command = cm.RUN
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg.lower() == cm.TEST:
            command = cm.TEST
    main(command)
