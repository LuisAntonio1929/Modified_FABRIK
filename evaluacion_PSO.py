import pandas as pd
from serialbot import SerialBot
import numpy as np
from time import time
from pso import Enjambre
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
    enjambre = Enjambre(
        n_particulas = 40,
        n_variables  = dof,
        limites_inf  = [-np.pi if i%2==0 else -5*np.pi/6 for i in range(dof)],
        limites_sup  = [np.pi if i%2==0 else 5*np.pi/6 for i in range(dof)],
        verbose      = False
    )
    def optimizar(*vals):
        mm,rr=R,P
        sr.FK(vals)
        eo=sr.orientations[-1]@R.T
        return np.sqrt(((sr.positions[-1]-rr)**2).sum()+(eo[1,2]-eo[2,1])**2+(eo[0,2]-eo[2,0])**2+(eo[1,0]-eo[0,1])**2)/np.sqrt(6)
    tim=time()
    enjambre.optimizar(
        funcion_objetivo = optimizar,
        optimizacion     = "minimizar",
        n_iteraciones    = 1000,
        inercia          = 0.8,
        reduc_inercia    = True,
        inercia_max      = 0.9,
        inercia_min      = 0.4,
        peso_cognitivo   = 1,
        peso_social      = 2,
        parada_temprana  = True,
        rondas_parada    = 5,
        tolerancia_parada = 1e-3,
        verbose          = False
    )
    tiempo.append((time()-tim)*1000)
    n_it.append(enjambre.iter_optimizacion)
    rms.append(enjambre.valor_optimo )
    for j in range(dof):qs[j].append(enjambre.posicion_optima[j])
    print(i)

df={}
for i in range(dof):df['q%i'%i]=qs[i]
df['Tiempo']=tiempo
df['N° Iteraciones']=n_it
df['RMSE Final']=rms
df=pd.DataFrame(df)
df.to_excel('datos_%idof/PSO_%idof_resultados.xlsx'%(dof,dof), index=False)