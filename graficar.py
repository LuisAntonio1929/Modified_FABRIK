import matplotlib.pyplot as plt
from serialbot import SerialBot
import numpy as np
import pandas as pd
dof=7
goals = pd.read_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof),usecols=['x','y','z','pitch','roll','yaw'])
n,_=goals.shape
x=[]
y=[]
z=[]
ori=[]

for i in range(n):
    x_i,y_i,z_i,pitch,roll,yaw=goals.iloc[i].tolist()
    x.append(x_i)
    y.append(y_i)
    z.append(z_i)
    R=np.array([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])@np.array([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    ori.append(R)

sr=SerialBot()
pos,ors=sr.FK()

figure = plt.figure(figsize=(12, 6))
ax= figure.add_subplot(1, 1, 1, projection='3d')
ax.set_xlim(-30,30)
ax.set_ylim(-30,30)
ax.set_zlim(-15,45)
ax.set_xlabel('x (cm)')
ax.set_ylabel('y (cm)')
ax.set_zlabel('z (cm)')
ax.autoscale(enable=False)


ax.quiver(np.concatenate([[i,i,i]for i in pos[0]]),np.concatenate([[i,i,i]for i in pos[1]]),np.concatenate([[i,i,i]for i in pos[2]]),np.concatenate(np.array([ors[i][0,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][1,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][2,:]for i in range(pos[0].shape[0])])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(pos[0].shape[0])])))
#ax.quiver(np.concatenate([[i,i,i]for i in x]),np.concatenate([[i,i,i]for i in y]),np.concatenate([[i,i,i]for i in z]),np.concatenate(np.array([ori[i][0,:]for i in range(len(x))])),np.concatenate(np.array([ori[i][1,:]for i in range(len(x))])),np.concatenate(np.array([ori[i][2,:]for i in range(len(x))])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(len(x))])))
pos=pos.T
for i in range(pos.shape[0]-1):
    plt.plot(pos[i:i+2,0],pos[i:i+2,1],[pos[i,2],pos[i+1,2]])

plt.show()