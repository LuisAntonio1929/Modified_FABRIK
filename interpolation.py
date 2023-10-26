import matplotlib.pyplot as plt
from serialbot import SerialBot
import numpy as np
import pandas as pd
from scipy.linalg import fractional_matrix_power as fmp

def rotx(a):
    return np.array([[1,0,0,0],
               [0,np.cos(a),-np.sin(a),0],
               [0,np.sin(a),np.cos(a),0],
               [0,0,0,1]])
def roty(a):
    return np.array([[np.cos(a),0,np.sin(a),0],
               [0,1,0,0],
               [-np.sin(a),0,np.cos(a),0],
               [0,0,0,1]])
def rotz(a):
    return np.array([[np.cos(a),-np.sin(a),0,0],
               [np.sin(a),np.cos(a),0,0],
               [0,0,1,0],
               [0,0,0,1]])
def trasl(tx,ty,tz):
    return np.array([[1,0,0,tx],
               [0,1,0,ty],
               [0,0,1,tz],
               [0,0,0,1]])

sr=SerialBot()
sr.FK()
init_pos=sr.positions[-1]
init_ori=sr.orientations[-1]

HTM1=np.vstack((np.hstack((init_ori,init_pos.reshape((3,1)))),[0,0,0,1]))
HTM2=HTM1@trasl(0,2*init_pos[-1]/3,-2*init_pos[-1]/3)@rotx(-np.pi*0.9)
HTM3=HTM1@trasl(2*init_pos[-1]/3,0,-2*init_pos[-1]/3)@roty(np.pi*0.9)
grupos=[[HTM1,HTM2],[HTM2,HTM3],[HTM3,HTM1]]
dof=15
x=[]
y=[]
z=[]
ori=[]
nits=[]
#dets=[]
for h1,h2 in grupos:
    for t in np.linspace(0,1):
        m=fmp(h1,1-t)@fmp(h2,t)
        m=m.real
        x_i=m[0,3]
        y_i=m[1,3]
        z_i=m[2,3]
        x.append(x_i)
        y.append(y_i)
        z.append(z_i)
        P=np.array([x_i,y_i,z_i])
        R=m[:3,:3]
        #dets.append(np.isclose(np.linalg.det(R),1) and np.isclose(np.trace(R@R.T),3))
        ori.append(R)
        err,ind=sr.IK(P,R)
        sr.FK()
        if len(ind)>0:
            print(err[-1],' ',ind[-1])
            nits.append(ind[-1])

#sr=SerialBot()
#pos,ors=sr.FK()
#print(all(dets))
print(nits)
figure = plt.figure(figsize=(12, 6))
ax= figure.add_subplot(1, 1, 1, projection='3d')
ax.set_xlim([-init_pos[-1],init_pos[-1]])
ax.set_ylim([-init_pos[-1],init_pos[-1]])
ax.set_zlim([0,1.1*init_pos[-1]])
ax.set_xlabel('x (cm)')
ax.set_ylabel('y (cm)')
ax.set_zlabel('z (cm)')
ax.autoscale(enable=False)

pos,ors=sr.FK()
ax.quiver(np.concatenate([[i,i,i]for i in pos[0]]),np.concatenate([[i,i,i]for i in pos[1]]),np.concatenate([[i,i,i]for i in pos[2]]),np.concatenate(np.array([ors[i][0,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][1,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][2,:]for i in range(pos[0].shape[0])])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(pos[0].shape[0])])))
ax.quiver(np.concatenate([[i,i,i]for i in x]),np.concatenate([[i,i,i]for i in y]),np.concatenate([[i,i,i]for i in z]),np.concatenate(np.array([ori[i][0,:]for i in range(len(x))])),np.concatenate(np.array([ori[i][1,:]for i in range(len(x))])),np.concatenate(np.array([ori[i][2,:]for i in range(len(x))])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(len(x))])))
pos=pos.T

for i in range(pos.shape[0]-1):
    plt.plot(pos[i:i+2,0],pos[i:i+2,1],[pos[i,2],pos[i+1,2]])

plt.show()