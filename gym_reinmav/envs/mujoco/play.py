import gym
import gym_reinmav
import numpy as np
from tkinter import *
import matplotlib.pyplot as plt


# def wKey(event):
#   w = Label(root,text=event.char, font=("Helvetica", 64))
#   w.place(x=65,y=40)
#   w.after(90, lambda: w.destroy() )

#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.render()
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

# def sKey(event):
#   w = Label(root,text=event.char, font=("Helvetica", 64))
#   w.place(x=65,y=40)
#   w.after(90, lambda: w.destroy() )

#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.render()
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

# def dKey(event):
#   w = Label(root,text=event.char, font=("Helvetica", 64))
#   w.place(x=65,y=40)
#   w.after(90, lambda: w.destroy() )

#   env.step([c+dw, c-dw, c+dw, c-dw])
#   env.step([c-dw, c+dw, c-dw, c+dw])
#   env.render()
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6)) # making qvel=0 after each sequence of steps

# def aKey(event):
#   w = Label(root,text=event.char, font=("Helvetica", 64))
#   w.place(x=65,y=40)
#   w.after(90, lambda: w.destroy() )

#   env.step([c-dw, c+dw, c-dw, c+dw])
#   env.step([c+dw, c-dw, c+dw, c-dw])
#   env.render()
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

# def eKey(event):
#   w = Label(root,text=event.char, font=("Helvetica", 64))
#   w.place(x=65,y=40)
#   w.after(90, lambda: w.destroy() )

#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))
#   env.render()

#   env.step([c+dw, c-dw, c+dw, c-dw])
#   env.step([c-dw, c+dw, c-dw, c+dw])
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

# def qKey(event):
#   w = Label(root,text=event.char, font=("Helvetica", 64))
#   w.place(x=65,y=40)
#   w.after(90, lambda: w.destroy() )

#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))
#   env.render()

#   env.step([c-dw, c+dw, c-dw, c+dw])
#   env.step([c+dw, c-dw, c+dw, c-dw])
#   pos = env.sim.data.qpos
#   vel = env.sim.data.qvel
#   env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

def wKey(env,c,dv,dw):

  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  # env.render()
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))
  print(pos[0],pos[1],pos[2])

def sKey(env,c,dv,dw):

  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.render()
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))

def dKey(env,c,dv,dw):

  env.step([c+dw, c-dw, c+dw, c-dw])
  env.step([c-dw, c+dw, c-dw, c+dw])
  env.render()
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6)) # making qvel=0 after each sequence of steps

def aKey(env,c,dv,dw):

  env.step([c-dw, c+dw, c-dw, c+dw])
  env.step([c+dw, c-dw, c+dw, c-dw])
  env.render()
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))

def eKey(env,c,dv,dw):

  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))
  env.render()

  env.step([c+dw, c-dw, c+dw, c-dw])
  env.step([c-dw, c+dw, c-dw, c+dw])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))

def qKey(env,c,dv,dw):

  env.step([c-dv, c+dv, c+dv, c-dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c+dv, c-dv, c-dv, c+dv])
  env.step([c-dv, c+dv, c+dv, c-dv])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))
  env.render()

  env.step([c-dw, c+dw, c-dw, c+dw])
  env.step([c+dw, c-dw, c+dw, c-dw])
  pos = env.sim.data.qpos
  vel = env.sim.data.qvel
  env.set_state(np.array([pos[0],pos[1],pos[2],pos[3],0,0,pos[6]]),np.array([0.0]*6))


def play(env,c,dv,dw,a):
  # 6 motion functions
  if a==1:
    wKey(env,c,dv,dw)
  elif a==2:
    sKey(env,c,dv,dw)
  elif a==3:
    aKey(env,c,dv,dw) 
  elif a==4:
    dKey(env,c,dv,dw)
  elif a==5:
    qKey(env,c,dv,dw)
  elif a==6:
    eKey(env,c,dv,dw)
  
  
  # root = Tk()

  # root.bind('<w>', wKey)
  # root.bind('<d>', dKey)
  # root.bind('<a>', aKey)
  # root.bind('<s>', sKey)
  # root.bind('<e>', eKey)
  # root.bind('<q>', qKey)

  # env = gym.make('MujocoQuadForce-v2')
  # env.reset()
  
  # c = 0.73575
  # dv = 0.05
  # dw = 0.02

  # args = [env,c,dv,dw]

  # while True:
    # root.update()
    # _,_,done,_ = env.step([c]*4)
    # env.render()

  # root.mainloop()
