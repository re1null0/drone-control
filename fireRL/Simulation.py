import numpy as np
import gym
from gym import spaces
from FireMap import FireMap, make_board, dimensions
from kernels import IntensityKernel


class FireMapEnv(gym.Env):
    def __init__(self):
        super(FireMapEnv, self).__init__()
        obs_dimensions = dimensions()
        self.action_space = spaces.MultiDiscrete((30, 30, 30, 30, 30, 30))
        self.observation_space = spaces.Box(
            low=0, high=1,
            shape=obs_dimensions,
            dtype=np.float32
        )
        self.state = None  # Initialize state as None
        self.current_iter = 0
        self.last_render = 0  # Track rendering frequency

    def step(self, action):
        if self.state is None:
            raise ValueError("Environment not initialized. Call reset() first.")
        
        formatted_actions = [(int(x), int(y)) for x, y in action]
        self.state.next(action)
        
        obs = self.state.state
        reward = self.state.get_reward(action)
        done = self.state.get_done()
        info = self.state.get_info()
        self.current_iter += 1
        return obs, reward, done, info

    def reset(self, humidity=0.4, wind_dir=(1,1), num_fires=1):
        board = make_board(num_points=num_fires)
        self.state = FireMap(board, wind_dir, humidity)  # Properly initialize FireMap instance
        
        self.state.state[:, :, 7] = humidity
        self.state.kernel = IntensityKernel().get_kernel(wind_dir)
        
        
        self.current_iter = 0
        self.last_render = 0
        return self.state.state

    def render(self, mode='human'):
        if (self.current_iter - self.last_render) >= 10:  # Render every 10 iterations
            self.state.show(iter=self.current_iter)
            self.last_render = self.current_iter
            

    def render_web(self, current_iter):
        self.current_iter = current_iter
        return self.state.render(self.current_iter)

    def close(self):
        pass

