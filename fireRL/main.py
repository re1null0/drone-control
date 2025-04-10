from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from Simulation import FireMapEnv
import base64
import io
import matplotlib.pyplot as plt

app = FastAPI()
env = FireMapEnv()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Use your frontend domain in prod
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# actions for adding moisture?
class ActionRequest(BaseModel):
    actions: list[list[int]]  # e.g. [[5,5],[7,8],[9,10]]


# intital things we need from the frontend
class InitRequest(BaseModel):
    humidity: float
    wind_x: float
    wind_y: float
    num_fires: int

@app.post("/reset")
def reset(init: InitRequest):
    env.reset(                                # ✅ call the reset method with parameters
        humidity=init.humidity,
        wind_dir=(init.wind_x, init.wind_y),
        num_fires=init.num_fires
    )
    return {"status": "ok"}

@app.post("/step")
def step(actions: ActionRequest):
    obs, reward, done, info = env.step(actions.actions)
    return {"done": done}

@app.get("/render")
def render():
    image = env.render_web(env.current_iter)
    return {"image": image}
