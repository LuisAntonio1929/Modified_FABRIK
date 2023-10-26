import pandas as pd
import random
from serialbot import SerialBot
import numpy as np
from time import time
from deap import base,creator,tools
random.seed(42)
sr=SerialBot()
dof=15
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
    def optimizar(solution):
        mm,rr=R,P
        if all([-np.pi<=solution[h]<=np.pi if h%2==0 else -5*np.pi/6<=solution[h]<=5*np.pi/6 for h in range(dof)]):
            sr.FK(solution)
            eo=sr.orientations[-1]@R.T
            return np.sqrt(((sr.positions[-1]-rr)**2).sum()+(eo[1,2]-eo[2,1])**2+(eo[0,2]-eo[2,0])**2+(eo[1,0]-eo[0,1])**2)/np.sqrt(6),
        else:
            return 1000,
    creator.create('FitnesMin',base.Fitness,weights=(-1.0,))#1 maximizar, -1 minimizar
    creator.create('Individual',list,fitness=creator.FitnesMin)
    toolbox=base.Toolbox()
    entorno=[]
    for k in range(dof):
        if k%2==0:
            toolbox.register('q%i'%k,random.uniform,-np.pi,np.pi)
        else:
            toolbox.register('q%i'%k,random.uniform,-5*np.pi/6,5*np.pi/6)
        eval('entorno.append(toolbox.q%i)'%k)
    entorno=tuple(entorno)
    toolbox.register("individual", tools.initCycle, creator.Individual,entorno,1) #Variables de decisión
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("evaluate", optimizar)
    toolbox.register("mate", tools.cxSimulatedBinary, eta=0.5)
    toolbox.register("mutate",tools.mutGaussian, mu=0, sigma=1, indpb=0.05)
    toolbox.register("select",tools.selTournament, tournsize=3)
    def GA():
        pop = toolbox.population(n=300)
        # Evaluate the entire population
        fitnesses = list(map(toolbox.evaluate, pop))
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit
        # CXPB  is the probability with which two individuals
        #       are crossed
        #
        # MUTPB is the probability for mutating an individual
        CXPB, MUTPB = 0.5, 0.2
        # Extracting all the fitnesses of 
        fits = [ind.fitness.values[0] for ind in pop]
        # Variable keeping track of the number of generations
        g = 0

        # Begin the evolution
        while min(fits) > 0.001 and g < 1000:
            # A new generation
            g = g + 1
            #print("-- Generation %i --" % g)
            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))
            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < CXPB:
                    toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
                    del mutant.fitness.values
            # Evaluate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = map(toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
            pop[:] = offspring
            # Gather all the fitnesses in one list and print the stats
            fits = [ind.fitness.values[0] for ind in pop]
        return tools.selBest(pop, 1)[0],min(fits),g

    tim=time()
    solution,error,nn=GA()
    tiempo.append((time()-tim)*1000)
    n_it.append(nn)
    rms.append(error)
    for j in range(dof):qs[j].append(solution[j])
    print(i)
df={}
for i in range(dof):df['q%i'%i]=qs[i]
df['Tiempo']=tiempo
df['N° Iteraciones']=n_it
df['RMSE Final']=rms
df=pd.DataFrame(df)
df.to_excel('datos_%idof/GA_%idof_resultados.xlsx'%(dof,dof), index=False)