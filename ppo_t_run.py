from stable_baselines import PPO2

import time

from furuta_env_t_ppo import FurutaEnvTorquePpo2
import furuta_env_base as feb
import common as cm


def main():
    env = FurutaEnvTorquePpo2(cm.RUN)
    env.setRender(True)

    while True:
        obs, done = env.reset(), False
        
        running_reward = 0.0
        ep_len = 0
        for _ in range(100000):
            time.sleep(1)
            
            obs, reward, done, infos = env.step(0)
            running_reward += reward
            ep_len += 1
            print("loop...")
            if done:
                print("Episode Reward: %.2f" % (running_reward))
                print("Episode Length: %d" % ep_len)
                running_reward = 0.0
                ep_len = 0
                break
        break
                
def main2():
    env = FurutaEnvTorquePpo2(cm.RUN)
    env.setRender(True)
    model = PPO2.load("ppo2_policy_nn.zip")

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
