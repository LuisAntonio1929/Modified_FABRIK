import pandas as pd
from serialbot import SerialBot
import numpy as np
from time import time
import pyswarms.backend as Pp
from pyswarms.backend.topology import Star

dof=9
sr=SerialBot()
goals = pd.read_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof),usecols=['x','y','z','pitch','roll','yaw'])
n,_=goals.shape
rms=[]
n_it=[]
tiempo=[]
qs=[[] for i in range(dof)]
init_vals=[0 for h in range(dof)]

bounds=[[-np.pi if i%2==0 else -5*np.pi/6 for i in range(dof)],[np.pi if i%2==0 else 5*np.pi/6 for i in range(dof)]]
my_options = {'c1': 0.6, 'c2': 0.3, 'w': 0.4} # arbitrarily set

for i in range(n):
    x,y,z,pitch,roll,yaw=goals.iloc[i].tolist()
    P=np.array([x,y,z])
    R=np.array([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])@np.array([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    pos,ors=sr.FK(init_vals)
    
    my_topology = Star()
    my_swarm = Pp.create_swarm(n_particles=40, dimensions=dof, options=my_options,bounds=bounds) # The Swarm Class
    
    def optimizar(vals):
        mm,rr=R,P
        sr.FK(vals)
        eo=sr.orientations[-1]@R.T
        return np.sqrt(((sr.positions[-1]-rr)**2).sum()+(eo[1,2]-eo[2,1])**2+(eo[0,2]-eo[2,0])**2+(eo[1,0]-eo[0,1])**2)/np.sqrt(6)
    
    nn=0
    tim=time()
    for k in range(1000):
        # Part 1: Update personal best
        costes=[]
        bcostes=[]
        for j in range(len(my_swarm.position)):
            costes.append(optimizar(my_swarm.position[j])) # Compute current cost
            bcostes.append(optimizar(my_swarm.pbest_pos[j]))  # Compute personal best pos
        my_swarm.current_cost = np.array(costes)
        my_swarm.pbest_cost = np.array(bcostes)
        my_swarm.pbest_pos, my_swarm.pbest_cost = Pp.compute_pbest(my_swarm) # Update and store

        # Part 2: Update global best
        # Note that gbest computation is dependent on your topology
        if np.min(my_swarm.pbest_cost) < my_swarm.best_cost:
            my_swarm.best_pos, my_swarm.best_cost = my_topology.compute_gbest(my_swarm)

        if my_swarm.best_cost<1e-3:
            nn=k
            break

        # Part 3: Update position and velocity matrices
        # Note that position and velocity updates are dependent on your topology
        my_swarm.velocity = my_topology.compute_velocity(my_swarm)
        my_swarm.position = my_topology.compute_position(my_swarm)
    tiempo.append((time()-tim)*1000)
    n_it.append(nn)
    rms.append(my_swarm.best_cost)
    for j in range(dof):qs[j].append(my_swarm.best_pos)
    print(i)

df={}
for i in range(dof):df['q%i'%i]=qs[i]
df['Tiempo']=tiempo
df['N° Iteraciones']=n_it
df['RMSE Final']=rms
df=pd.DataFrame(df)
df.to_excel('datos_%idof/PSO_%idof_resultados.xlsx'%(dof,dof), index=False)