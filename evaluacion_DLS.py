import pandas as pd
from DualBot import DualBot
from dualnumber import DualNumber
from quaternion import quaternion
import numpy as np
from time import time
dof=15
goals = pd.read_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof),usecols=['x','y','z','pitch','roll','yaw'])
n,_=goals.shape
sr=DualBot()
rms=[]
n_it=[]
tiempo=[]
qs=[[] for i in range(dof)]
init_vals=[0 for h in range(dof)]

for i in range(n):
    x,y,z,pitch,roll,yaw=goals.iloc[i].tolist()
    R=DualNumber(quaternion(np.cos(pitch/2),np.sin(pitch/2),0,0),quaternion(0,0,0,0))*DualNumber(quaternion(np.cos(roll/2),0,np.sin(roll/2),0),quaternion(0,0,0,0))*DualNumber(quaternion(np.cos(yaw/2),0,0,np.sin(yaw/2)),quaternion(0,0,0,0))
    P=DualNumber(quaternion(1,0,0,0),quaternion(0,x/2,y/2,z/2))
    RP=P*R
    pos,ors=sr.FK(init_vals)
    tim=time()
    error,ind=sr.IK_DLS(RP)
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
df.to_excel('datos_%idof/DLS_%idof_resultados.xlsx'%(dof,dof), index=False)