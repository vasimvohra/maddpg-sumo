from Classes.Environment_Platoon_simple_env import SUMOPlatoonEnv

env = SUMOPlatoonEnv(gui=True)
obs = env.reset()
for _ in range(100):
    action = env.action_space.sample()
    obs, reward, done, _ = env.step(action)
    if done:
        break
env.close()
