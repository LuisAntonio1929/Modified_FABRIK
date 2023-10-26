import pandas as pd
from serialbot import SerialBot
import numpy as np
from time import time
dof=15
goals = pd.read_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof),usecols=['x','y','z','pitch','roll','yaw'])
n,_=goals.shape
sr=SerialBot()
rms=[]
n_it=[]
tiempo=[]
qs=[[] for i in range(dof)]
init_vals=[0 for h in range(dof)]
for i in range(n):
    x,y,z,pitch,roll,yaw=goals.iloc[i].tolist()
    P=np.array([x,y,z])
    R=np.array([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])@np.array([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    mm,rr=R,P
    pos,ors=sr.FK(init_vals)
    tim=time()
    error,ind=sr.IK(P,R)
    tiempo.append((time()-tim)*1000)
    n_it.append(ind[-1])
    rms.append(error[-1])
    for j in range(dof):qs[j].append(sr.values[j])
    print(i)
df={}
for i in range(dof):df['q%i'%i]=qs[i]
df['Tiempo']=tiempo
df['N° Iteraciones']=n_it
df['RMSE Final']=rms
df=pd.DataFrame(df)
df.to_excel('datos_%idof/FABRIK_M_%idof_resultados.xlsx'%(dof,dof), index=False)