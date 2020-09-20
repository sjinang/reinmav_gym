import gym
import gym_reinmav
import numpy as np
from tkinter import *
import matplotlib.pyplot as plt

root = Tk()

def wKey(event):
  w = Label(root,text=event.char, font=("Helvetica", 64))
  w.place(x=65,y=40)
  w.after(90, lambda: w.destroy() )

  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

def sKey(event):
  w = Label(root,text=event.char, font=("Helvetica", 64))
  w.place(x=65,y=40)
  w.after(90, lambda: w.destroy() )

  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

def dKey(event):
  w = Label(root,text=event.char, font=("Helvetica", 64))
  w.place(x=65,y=40)
  w.after(90, lambda: w.destroy() )

  env.step([c+dw, c-dw, c+dw, c-dw])
  env.step([c-dw, c+dw, c-dw, c+dw])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6)) # making qvel=0 after each sequence of steps

def aKey(event):
  w = Label(root,text=event.char, font=("Helvetica", 64))
  w.place(x=65,y=40)
  w.after(90, lambda: w.destroy() )

  env.step([c-dw, c+dw, c-dw, c+dw])
  env.step([c+dw, c-dw, c+dw, c-dw])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

def eKey(event):
  w = Label(root,text=event.char, font=("Helvetica", 64))
  w.place(x=65,y=40)
  w.after(90, lambda: w.destroy() )

  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

  env.step([c+dw, c-dw, c+dw, c-dw])
  env.step([c-dw, c+dw, c-dw, c+dw])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

def qKey(event):
  w = Label(root,text=event.char, font=("Helvetica", 64))
  w.place(x=65,y=40)
  w.after(90, lambda: w.destroy() )

  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

  env.step([c-dw, c+dw, c-dw, c+dw])
  env.step([c+dw, c-dw, c+dw, c-dw])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

root.bind('<w>', wKey)
root.bind('<d>', dKey)
root.bind('<a>', aKey)
root.bind('<s>', sKey)
root.bind('<e>', eKey)
root.bind('<q>', qKey)

env = gym.make('MujocoQuadForce-v2')
env.reset()
c = 0.73575
# v = np.sort(np.random.uniform(0,0.05,100))
# temp = -np.copy(v)
# v = np.sort(np.append(temp,v))
dv = 0.15
dw = 0.07

while True:
  root.update()
  env.step([c]*4)
  env.render()
# for dv in v:
#   for n in range(3000):
#     # root.update()
    
#     if n<500: 
#       # env.step([c+dw, c-dw, c+dw, c-dw])
#       # env.step([c-dw, c+dw, c-dw, c+dw])
      
#     # env.step([c+dv, c-dv, c-dv, c+dv])
#   #   env.step([c-dv, c+dv, c+dv, c-dv])
#   #   env.step([c-dv, c+dv, c+dv, c-dv])
#   #   env.step([c+dv, c-dv, c-dv, c+dv])
#       # env.step([c]*4)
#     else:
#       env.step([c]*4)
      
#     Vy.append(env.sim.data.qvel[1])
#     env.render()
#     print(n,env.sim.data.qvel[1])
    

#   env.reset()
  


# plt.plot(np.arange(len(Vy)),Vy,'o-')
# plt.show()

root.mainloop()


