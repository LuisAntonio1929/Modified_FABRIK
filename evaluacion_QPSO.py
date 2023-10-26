import pandas as pd
from serialbot import SerialBot
import numpy as np
from time import time
from Qpso import QDPSO
dof=15
sr=SerialBot()
goals = pd.read_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof),usecols=['x','y','z','pitch','roll','yaw'])
n,_=goals.shape
rms=[]
n_it=[]
tiempo=[]
qs=[[] for i in range(dof)]
init_vals=[0 for h in range(dof)]

for i in range(n):
    x,y,z,pitch,roll,yaw=goals.iloc[i].tolist()
    P=np.array([x,y,z])
    R=np.array([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])@np.array([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    pos,ors=sr.FK(init_vals)
    
    NParticle = 40
    MaxIters = 1000
    NDim = dof
    bounds = [(-np.pi,np.pi) if h%2==0 else (-5*np.pi/6,5*np.pi/6) for h in range(dof)]
    g = 0.96

    def optimizar(vals):
        mm,rr=R,P
        sr.FK(vals)
        eo=sr.orientations[-1]@R.T
        return np.sqrt(((sr.positions[-1]-rr)**2).sum()+(eo[1,2]-eo[2,1])**2+(eo[0,2]-eo[2,0])**2+(eo[1,0]-eo[0,1])**2)/np.sqrt(6)
    s = QDPSO(optimizar, NParticle, NDim, bounds, MaxIters, g)
    tim=time()
    s.update()
    tiempo.append((time()-tim)*1000)
    n_it.append(s.iters)
    rms.append(s.gbest_value)
    for j in range(dof):qs[j].append(s.gbest[j])
    print(i)

df={}
for i in range(dof):df['q%i'%i]=qs[i]
df['Tiempo']=tiempo
df['N° Iteraciones']=n_it
df['RMSE Final']=rms
df=pd.DataFrame(df)
df.to_excel('datos_%idof/QPSO_%idof_resultados.xlsx'%(dof,dof), index=False)