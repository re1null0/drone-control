import numpy as np
from Simulation import FireMapEnv
sim = FireMapEnv()
while True:
    sim.reset()
    for _ in range(30):
        for _ in range(10):
            actions = np.reshape(np.random.randint(0, 45/3, 6), (3,2))
            obs, _, _, _ = sim.step(actions)
        sim.render()