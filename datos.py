import pandas as pd
from math import pi
import numpy as np
from serialbot import SerialBot

dof=15

def pry(m):
  y=-np.arctan2(m[0,1],m[0,0])
  r=np.arctan2(m[0,2]*np.cos(y),m[0,0])
  p=-np.arctan2(m[1,2],m[2,2])
  return p,r,y
np.random.seed(42)
ranges=[(-pi,pi) if i%2==0 else (-5*pi/6,5*pi/6) for i in range(dof)]
d={}
n=500
for i in range(len(ranges)):d["q_%i"%i]=np.random.uniform(ranges[i][0],ranges[i][1],n)
d=pd.DataFrame(d)
x=[]
y=[]
z=[]
pitch=[]
roll=[]
yaw=[]
sr=SerialBot()
for i in range(n):
  sr.FK(d.iloc[i].tolist())
  xi,yi,zi=sr.positions[-1]
  x.append(xi.copy())
  y.append(yi.copy())
  z.append(zi.copy())
  pi,ri,yi=pry(sr.orientations[-1])
  pitch.append(pi.copy())
  roll.append(ri.copy())
  yaw.append(yi.copy())
d2={'x':x,'y':y,'z':z,'pitch':pitch,'roll':roll,'yaw':yaw}
d2=pd.DataFrame(d2)
df=pd.concat([d, d2], axis=1)
df.to_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof), index=False)