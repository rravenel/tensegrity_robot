from stable_baselines import PPO2

import time

from tensegrity_env_d_ppo2 import TensegrityEnvDeltaPpo2
import tensegrity_env_base as teb
import common as cm

                
def main():
    env = TensegrityEnvDeltaPpo2(cm.RUN)
    env.setRender(True)
    model = PPO2.load(teb.MODEL_PATH + "ppo2_a.zip")

    while True:
        obs, done = env.reset(), False
        
        running_reward = 0.0
        ep_len = 0
        for _ in range(100000):
            action, _ = model.predict(obs)
            
            obs, reward, done, infos = env.step(action)
            running_reward += reward
            ep_len += 1
            
            if done:
                print("Episode Reward: %.2f" % (running_reward))
                print("Episode Length: %d" % ep_len)
                running_reward = 0.0
                ep_len = 0
                break
                
if __name__ == '__main__':
    main()
