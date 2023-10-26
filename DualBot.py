from math import pi,sin,cos,tan,atan
from time import time
import numpy as np
from pyrsistent import v
from dualnumber import DualNumber
from quaternion import quaternion
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.animation import FuncAnimation
import pygad
import pybullet_data as pd
import pybullet as p
from pso import Enjambre

class DualBot:
    def __init__(self):
        self.sequence = [[['pi',[-pi,pi]],2],[['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],7],
                                            [['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],7],
                                            [['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],7],
                                            [['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],7],
                                            [['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],7],
                                            [['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],7],
                                            [['h',[-5*pi/6,5*pi/6]],2],[['pi',[-pi,pi]],6]]
        self.rootDQ=DualNumber(quaternion(1,0,0,0),quaternion(0,0,0,0))
        self.values=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        #self.values=[0,0,0,0,0,0,0]
    def FK(self,values=None,jacp=False):
        if not(values is None):
            self.values=values
        self.DQlist=[self.rootDQ]
        R,P=self.rootDQ.toVectorMatrix()
        bodyP=[P]
        bodyR=[R]
        jac=[]
        aux=[]
        contv=0
        for i in self.sequence:
            if i[0][0]=='pi':
                jac.append(self.DQlist[-1]*DualNumber(quaternion(-sin(self.values[contv]/2),0,0,cos(self.values[contv]/2)),quaternion(0,0,0,0)))
                self.DQlist.append(self.DQlist[-1]*DualNumber(quaternion(cos(self.values[contv]/2),0,0,sin(self.values[contv]/2)),quaternion(0,0,0,0)))
                aux.append(self.DQlist[-1])
                contv+=1
            elif i[0][0]=='h':
                jac.append(self.DQlist[-1]*DualNumber(quaternion(-sin(self.values[contv]/2),cos(self.values[contv]/2),0,0),quaternion(0,0,0,0)))
                self.DQlist.append(self.DQlist[-1]*DualNumber(quaternion(cos(self.values[contv]/2),sin(self.values[contv]/2),0,0),quaternion(0,0,0,0)))
                aux.append(self.DQlist[-1])
                contv+=1
            if type(i[1])==type([]):
                jac.append(self.DQlist[-1]*DualNumber(quaternion(0,0,0,0),quaternion(0,0,0,1)))
                self.DQlist.append(self.DQlist[-1]*DualNumber(quaternion(1,0,0,0),quaternion(0,0,0,self.values[contv]/2)))
                aux.append(self.DQlist[-1])
                R,P=self.DQlist[-1].toVectorMatrix()
                bodyP.append(P)
                bodyR.append(R)
                contv+=1
            else:
                self.DQlist.append(self.DQlist[-1]*DualNumber(quaternion(1,0,0,0),quaternion(0,0,0,i[1]/2)))
                R,P=self.DQlist[-1].toVectorMatrix()
                bodyP.append(P)
                bodyR.append(R)
        if not jacp:self.jac=0.5*np.array([(jac[i]*aux[i].inverse()*self.DQlist[-1]).toArray()for i in range(len(jac))]).T
        else:
            self.jac=[]
            for i in range(len(jac)):
                ele=jac[i]*aux[i].inverse()*self.DQlist[-1]
                self.jac.append((ele*self.DQlist[-1].conjugate3()+self.DQlist[-1]*ele.conjugate3()).toArray())
            self.jac=0.5*np.array(self.jac).T
        return np.array(bodyP).T, np.array(bodyR)
    def IK_DLS(self,target,jp=False):
        def Func1(v):
            contv=0
            l=[]
            for i in range(len(self.sequence)):
                if v[contv]>self.sequence[i][0][1][1]:
                    l.append(self.sequence[i][0][1][1])
                elif v[contv]<self.sequence[i][0][1][0]:
                    l.append(self.sequence[i][0][1][0])
                else:
                    l.append(v[contv])
                contv+=1
                if type(self.sequence[i][1])==type([]):
                    if v[contv]>self.sequence[i][1][1]:
                        l.append(self.sequence[i][1][1])
                    elif v[contv]<self.sequence[i][1][0]:
                        l.append(self.sequence[i][1][0])
                    else:
                        l.append(v[contv])
                    contv+=1
            return l
        def Func2(v):
            contv=0
            l=[]
            for i in range(len(self.sequence)):
                l.append(v[contv])
                contv+=1
                if type(self.sequence[i][1])==type([]):
                    l.append(v[contv])
                    contv+=1
            return l
        def Func3(v):
            contv=0
            l=np.eye(len(self.values))
            for i in range(len(self.sequence)):
                if v[contv]>self.sequence[i][0][1][1]:
                    l[contv][contv]=0.01
                elif v[contv]<self.sequence[i][0][1][0]:
                    l[contv][contv]=0.01
                else:
                    l[contv][contv]=1
                contv+=1
                if type(self.sequence[i][1])==type([]):
                    if v[contv]>self.sequence[i][1][1]:
                        l[contv][contv]=0.01
                    elif v[contv]<self.sequence[i][1][0]:
                        l[contv][contv]=0.01
                    else:
                        l[contv][contv]=1
                    contv+=1
            return l
        def alfa(err):
            err=np.array(err)
            j=self.jac@self.jac.T@err
            return err.dot(j)/j.dot(j)

        def dampingMat(v):
            contv=0
            l=np.eye(10)
            for i in range(len(self.sequence)):
                l[contv][contv]=((2*v[contv]-self.sequence[i][0][1][0]-self.sequence[i][0][1][1])/(self.sequence[i][0][1][1]-self.sequence[i][0][1][0]))**32
                contv+=1
                if type(self.sequence[i][1])==type([]):
                    l[contv][contv]=((2*v[contv]-self.sequence[i][1][0]-self.sequence[i][1][1])/(self.sequence[i][1][1]-self.sequence[i][1][0]))**32
                    contv+=1
            return l
        def cart_error(t,v):
            rt,pt=t.toVectorMatrix()
            rv,pv=v.toVectorMatrix()
            eo=rv@rt.T
            ep=np.array(pt)-np.array(pv)
            return np.sqrt((ep**2).sum()+(eo[1,2]-eo[2,1])**2+(eo[0,2]-eo[2,0])**2+(eo[1,0]-eo[0,1])**2)/np.sqrt(6)
        errList=[]
        indList=[]
        if not jp:
            for i in range(1000):
                error=(target-self.DQlist[-1]).toArray()
                me=cart_error(target,self.DQlist[-1])
                errList.append(me)
                indList.append(i)
                if me>1e-3:
                    vals=Func2(self.values)
                    m1=np.linalg.inv(Func3(vals))
                    m2=self.jac.T@np.linalg.inv(self.jac@self.jac.T+np.eye(8)*alfa(error)**2)
                    vals=(np.array(vals)+m1@m2@error).tolist()
                    self.values=Func1(vals)
                    self.FK()
                else:
                    break
        else:
            for i in range(1000):
                error=(target*target.conjugate3()-self.DQlist[-1]*self.DQlist[-1].conjugate3()).toArray()
                me=np.linalg.norm(error)/np.sqrt(3)
                errList.append(me)
                indList.append(i)
                if me>1e-3:
                    vals=Func2(self.values)
                    m1=np.linalg.inv(Func3(vals))
                    m2=np.linalg.pinv(self.jac)
                    vals=(np.array(vals)+m1@m2@error).tolist()
                    self.values=Func1(vals)
                    self.FK(jacp=True)
                else:
                    break
            #print(i+1,np.linalg.norm(error),self.values)
        #print(self.values)
        return errList,indList
    def IK_PSO(self,target):
        def cart_error(*vals,t=target,self=self):
            self.FK(values=vals)
            v=self.DQlist[-1]
            rt,pt=t.toVectorMatrix()
            rv,pv=v.toVectorMatrix()
            eo=rv@rt.T
            theta=-np.arcsin(eo[2,0])
            gamma=np.arctan2(eo[2,1]/np.cos(theta),eo[2,2]/np.cos(theta))
            phi=np.arctan2(eo[1,0]/np.cos(theta),eo[0,0]/np.cos(theta))
            ep=np.array(pt)-np.array(pv)
            return np.sqrt((ep**2).sum()+(theta*10)**2+(gamma*10)**2+(phi*10)**2)/np.sqrt(6)
        enjambre = Enjambre(
               n_particulas = 15,
               n_variables  = 7,
               limites_inf  = [-pi,-5*pi/6,-pi,-5*pi/6,-pi,-5*pi/6,-pi],
               limites_sup  = [pi,5*pi/6,pi,5*pi/6,pi,5*pi/6,pi],
               verbose      = False
            )
        enjambre.optimizar(
                funcion_objetivo = cart_error,
                optimizacion     = "minimizar",
                n_iteraciones    = 250,
                inercia          = 0.8,
                reduc_inercia    = True,
                inercia_max      = 0.9,
                inercia_min      = 0.4,
                peso_cognitivo   = 1,
                peso_social      = 2,
                parada_temprana  = True,
                rondas_parada    = 5,
                tolerancia_parada = 10**-3,
                verbose          = False
            )
        self.values=list(enjambre.posicion_optima)
        return enjambre.resultados_df['mejor_valor_enjambre'].tolist(),[i for i in range(len(enjambre.resultados_df['mejor_valor_enjambre']))]
    def IK_GA(self,target):
        def cart_error(vals,solution_idx):
            self.FK(values=vals)
            v=self.DQlist[-1]
            t=target
            rt,pt=t.toVectorMatrix()
            rv,pv=v.toVectorMatrix()
            eo=rv@rt.T
            theta=-np.arcsin(eo[2,0])
            gamma=np.arctan2(eo[2,1]/np.cos(theta),eo[2,2]/np.cos(theta))
            phi=np.arctan2(eo[1,0]/np.cos(theta),eo[0,0]/np.cos(theta))
            ep=np.array(pt)-np.array(pv)
            if -pi<=vals[0]<=pi and -5*pi/6<=vals[1]<=5*pi/6 and -pi<=vals[2]<=pi and -5*pi/6<=vals[3]<=5*pi/6 and -pi<=vals[4]<=pi and -5*pi/6<=vals[5]<=5*pi/6 and -pi<=vals[6]<=pi:
                return np.sqrt(6)/np.sqrt((ep**2).sum()+(theta*10)**2+(gamma*10)**2+(phi*10)**2)
            else:
                return 0
        fitness_function = cart_error
        num_generations = 200
        num_parents_mating = 10
        sol_per_pop = 20
        num_genes = 7
        init_range_low = 0
        init_range_high = 0
        parent_selection_type = "sss"
        keep_parents = 1
        crossover_type = "single_point"
        mutation_type = "random"
        mutation_percent_genes = 10
        stop_criteria=["reach_1000"]

        ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating,
                       fitness_func=fitness_function,
                       sol_per_pop=sol_per_pop,
                       num_genes=num_genes,
                       init_range_low=init_range_low,
                       init_range_high=init_range_high,
                       parent_selection_type=parent_selection_type,
                       keep_parents=keep_parents,
                       crossover_type=crossover_type,
                       mutation_type=mutation_type,
                       mutation_percent_genes=mutation_percent_genes,
                       save_best_solutions=True,
                       stop_criteria=stop_criteria)
        ga_instance.run()
        self.values,_,_=ga_instance.best_solution()
        self.values=self.values.tolist()
        return [1/i for i in ga_instance.best_solutions_fitness],[i for i in range(len(ga_instance.best_solutions_fitness))]

if __name__ == "__main__":
    figure = plt.figure(figsize=(12, 6))
    ax= figure.add_subplot(1, 2, 1, projection='3d')
    ax.set_xlim(-8,8)
    ax.set_ylim(-8,8)
    ax.set_zlim(0,15)
    ax.set_xlabel('x (cm)')
    ax.set_ylabel('y (cm)')
    ax.set_zlabel('z (cm)')
    ax.autoscale(enable=False)

    sr=DualBot()
    lx=3*pi/4
    ly=pi/4
    lz=pi/4
    R=DualNumber(quaternion(cos(lx/2),sin(lx/2),0,0),quaternion(0,0,0,0))*DualNumber(quaternion(cos(ly/2),0,sin(ly/2),0),quaternion(0,0,0,0))*DualNumber(quaternion(cos(lz/2),0,0,sin(lz/2)),quaternion(0,0,0,0))
    P=DualNumber(quaternion(1,0,0,0),quaternion(0,10/2,1/2,10/2))
    RP=P*R
    mm,rr=RP.toVectorMatrix()

    #pos,ors=sr.FK([pi/3,-pi/6,pi/6,5,-pi/3,pi/6,6,-pi/3,pi/3,8])
    vt=False
    pos,ors=sr.FK(jacp=vt)
    tim=time()
    error,ind=sr.IK_GA(RP)
    print("Time:",(time()-tim)*1000)
    print("Value:",sr.values)
    pos,ors=sr.FK(jacp=vt)
    #pos,ors=sr.plot_body()
    ax.quiver(np.concatenate([[i,i,i]for i in pos[0]]),np.concatenate([[i,i,i]for i in pos[1]]),np.concatenate([[i,i,i]for i in pos[2]]),np.concatenate(np.array([ors[i][0,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][1,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][2,:]for i in range(pos[0].shape[0])])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(pos[0].shape[0])])))
    ax.quiver([rr[0]]*3,[rr[1]]*3,[rr[2]]*3,*mm*2,color=[(1.0, .0, .0, 1.0), (.0, .0, 1.0, 1.0), (.0, 1.0, .0, 1.0)])
    ax.plot([rr[0]],[rr[1]],[rr[2]],'or')
    pos=pos.T
    for i in range(pos.shape[0]-1):
        plt.plot(pos[i:i+2,0],pos[i:i+2,1],[pos[i,2],pos[i+1,2]])
    ax2= figure.add_subplot(1, 2, 2)
    ax2.plot(ind,error,ind,error,'bo')
    ax2.set_xticks(ind)
    ax2.grid()
    print("Error ",min(error))
    print('N:',ind)
    #print(np.array(sr.angulos)*180/np.pi)
    ax2.set_xlabel('N iterations')
    ax2.set_ylabel('RMSE')
    ax2.set_title('GA Method\'s RMSE per Iteration')
    #|ax2.xaxis.set_major_locator(MaxNLocator(integer=True))
    '''
    physicsClient=p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    plane=p.loadURDF("plane.urdf")
    robot=p.loadURDF("C:/Users/User/Desktop/amber_b1_test/amber_b1_test.urdf",useFixedBase=1)
    p.setRealTimeSimulation(1)
    p.setGravity(0,0,-9.81)
    vv=sr.values.copy()
    vv[3]=(vv[3]-7)/100
    vv[6]=(vv[6]-7)/100
    vv[9]=(vv[9]-6)/100
    print(sr.values)
    print(vv)
    p.setJointMotorControlArray(robot,range(len(sr.values)),p.POSITION_CONTROL,targetPositions=vv)
    '''
    plt.show()