import gym
import gym_reinmav
import numpy as np
from tkinter import *
import matplotlib.pyplot as plt
# root = Tk()
# def wKey(event):
#   print('w')
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])

# def sKey(event):
#   print('s')
#   env.step([c+dv, c-dv, c-dv, c+dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c-dv, c+dv, c+dv, c-dv])
#   env.step([c+dv, c-dv, c-dv, c+dv])
# def dKey(event):
#   # print('e')
#   env.step([c+dw, c-dw, c+dw, c-dw])
#   # env.step([c-dw, c+dw, c-dw, c+dw])
# def aKey(event):
#   # print('q')
#   env.step([c-dw, c+dw, c-dw, c+dw])
#   # env.step([c+dw, c-dw, c+dw, c-dw])

# root.bind('<w>', wKey)
# root.bind('<d>', dKey)
# root.bind('<a>', aKey)
# root.bind('<s>', sKey)

env = gym.make('MujocoQuadForce-v1')
env.reset()
c = 0.73575
# v = np.sort(np.random.uniform(0,0.05,100))
# temp = -np.copy(v)
# v = np.sort(np.append(temp,v))
w = [1e-1]
Vy=[]
for dw in w:
  for n in range(10000):
    # root.update()
    
    if n<500: 
      env.step([c+dw, c-dw, c+dw, c-dw])
      env.step([c-dw, c+dw, c-dw, c+dw])
    else:
      env.step([c]*4)
    Vy.append(env.sim.data.qvel[5])
    env.render()
    print(n,env.sim.data.qvel[5])
    

  # env.reset()
  


# plt.plot(np.arange(len(Vy)),Vy,'o-')
plt.show()
# root.mainloop()


