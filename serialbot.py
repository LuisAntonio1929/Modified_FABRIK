from math import pi,sin,cos
from time import time
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.animation import FuncAnimation
from quaternion import quaternion

import pybullet_data as pd
import pybullet as p


class SerialBot:
    def __init__(self):
        self.sequence = [[['pi',[-pi,pi]],['pr',2]],[['h',[-5*pi/6,5*pi/6]],['pr',2]],[['pi',[-pi,pi]],['pr',7]],
                                          [['h',[-5*pi/6,5*pi/6]],['pr',2]],[['pi',[-pi,pi]],['pr',7]],
                                          [['h',[-5*pi/6,5*pi/6]],['pr',2]],[['pi',[-pi,pi]],['pr',6]]]
        self.root_orientation=np.eye(3)
        self.root_position=np.array([0,0,0])
        self.values=[0,0,0,0,0,0,0]
        #self.values=[0,0,0,0,0,0,0]
    def matRotX(self,a):
        return np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
    def matRotZ(self,a):
        return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    def plot_body(self):
        pos=[]
        ori=[]
        for i in self.body:
            pos.append(i[0])
            ori.append(i[1])
        return  np.array(pos).T,np.array(ori)
    def FK(self,values=None):
        if not(values is None):
            assert len(self.values)==len(values), 'DOF no coinciden %i != %i'%(len(self.values),len(values))
            self.values=values
        self.orientations=[self.root_orientation.copy()]
        self.positions=[self.root_position.copy()]
        self.body=[]
        cont=0
        for i in self.sequence:
            if i[0][0]=='pi':
                self.orientations.append(self.orientations[-1]@self.matRotZ(self.values[cont]))
                #self.positions.append(self.positions[-1])
                cont+=1
            else:
                self.orientations.append(self.orientations[-1]@self.matRotX(self.values[cont]))
                #self.positions.append(self.positions[-1])
                cont+=1
            if i[1][0]=='pr'and type(i[1][1])==type([]):
                self.positions.append(self.positions[-1]+self.orientations[-1][:,2]*self.values[cont])
                self.body.append([self.positions[-1],self.orientations[-1],self.positions[-2]])
                cont+=1
            else:
                self.positions.append(self.positions[-1]+self.orientations[-1][:,2]*i[1][1])
                self.body.append([self.positions[-1],self.orientations[-1],self.positions[-2]])
        #print(len(self.body),len(self.sequence))
        return np.array(self.positions.copy()).T,np.array(self.orientations.copy())
    def IK(self,P,O=None):
        def error(v,vc):
            if len(v)==2:
                p,o=v
                pc,oc=vc
                ep=pc-p
                eo=oc@o.T
                return np.sqrt((ep**2).sum()+(eo[1,2]-eo[2,1])**2+(eo[0,2]-eo[2,0])**2+(eo[1,0]-eo[0,1])**2)/np.sqrt(6)
            else:
                ep=vc-v
                return np.sqrt((ep**2).sum())/np.sqrt(3)
        def define_orientation(p):
            dp=p-self.body[-1][2]
            n=np.linalg.norm(dp)
            if n>0:
                z=dp/n
                rotaxis=np.cross(self.body[-1][1][:,2],z)
                n2=np.linalg.norm(rotaxis)
                if n2>0:
                    caxis=np.dot(self.body[-1][1][:,2],z)
                    naxis=rotaxis/n2
                    q=quaternion(caxis,*naxis)
                    x=(q*quaternion(0,*self.body[-1][1][:,0])*q.conjugate()).vector
                else:
                    x=self.body[-1][1][:,0]
                y=np.cross(z,x)
                return np.array([x,y,z]).T
            else:
                return self.body[-1][1]
        def find_hinge(n,num,back=True):
            cont=0
            if back:
                for i in range(n-1,0,-1):
                    if self.sequence[i][0][0]=='h':
                        cont+=1
                    if cont==num:
                        return i
                return 0
            else:
                for i in range(n+1,len(self.sequence)):
                    if self.sequence[i][0][0]=='h':
                        cont+=1
                    if cont==num:
                        return i
                return len(self.sequence)-1
        def backward(P,O):
            contv=len(self.values)-1
            self.body[-1][0]=P.copy()
            self.body[-1][1]=O.copy()
            for i in range(len(self.sequence)-1,0,-1):
                #print(i)
                if type(self.sequence[i][1][1])==type([]):
                    d=self.body[i][0]-self.body[i][2]
                    l=d.dot(self.body[i][1][:,2])
                    if l>self.sequence[i][1][1][1]:
                        l=self.sequence[i][1][1][1]
                    elif l<self.sequence[i][1][1][0]:
                        l=self.sequence[i][1][1][0]
                    self.values[contv]=l
                    self.body[i][2]=self.body[i][0]-l*self.body[i][1][:,2]
                    self.body[i-1][0]=self.body[i][2].copy()
                    contv-=1
                else:
                    self.body[i][2]=self.body[i][0]-self.sequence[i][1][1]*self.body[i][1][:,2]
                    self.body[i-1][0]=self.body[i][2].copy()
                if self.sequence[i][0][0]=='pi':
                    i_h=find_hinge(i,2)
                    d=self.body[i][2]-self.body[i_h][2]
                    nd=np.linalg.norm(d)
                    if nd>0:
                        zi_1=d/nd
                    else:
                        zi_1=self.body[i_h][1][:,2]
                    x=np.cross(self.body[i][1][:,2],zi_1)
                    nx=np.linalg.norm(x)
                    if not np.isclose(nx,0):
                        x=x/nx
                    else:
                        x=self.body[i-1][1][:,0].copy()
                    if np.sign(x.dot(self.body[i-1][1][:,0]))<0:
                        x=-x
                    csn=x.dot(self.body[i][1][:,0])
                    if abs(csn)>1:csn=np.sign(csn)
                    q=np.arccos(csn)*np.sign(np.dot(np.cross(x,self.body[i][1][:,0]),self.body[i][1][:,2]))
                    if q<self.sequence[i][0][1][0]:
                        q=self.sequence[i][0][1][0]
                        x=np.cos(q)*self.body[i][1][:,0]-np.sin(q)*self.body[i][1][:,1]
                    elif q>self.sequence[i][0][1][1]:
                        q=self.sequence[i][0][1][1]
                        x=np.cos(q)*self.body[i][1][:,0]-np.sin(q)*self.body[i][1][:,1]
                    y=np.cross(self.body[i][1][:,2],x)
                    self.body[i-1][1]=np.array([x,y,self.body[i][1][:,2]]).T
                    self.values[contv]=q
                    contv-=1
                elif self.sequence[i][0][0]=='h':
                    i_h=find_hinge(i,1)
                    d=self.body[i][2]-self.body[i_h][2]
                    d=d-d.dot(self.body[i][1][:,0])*self.body[i][1][:,0]
                    nd=np.linalg.norm(d)
                    if nd>0:
                        z=d/nd
                    else:
                        z=self.body[i_h][1][:,2].copy()
                    csn=z.dot(self.body[i][1][:,2])
                    if abs(csn)>1:csn=np.sign(csn)
                    q=np.arccos(csn)*np.sign(np.dot(np.cross(z,self.body[i][1][:,2]),self.body[i][1][:,0]))
                    if q<self.sequence[i][0][1][0]:
                        q=self.sequence[i][0][1][0]
                        z=np.cos(q)*self.body[i][1][:,2]-np.sin(q)*self.body[i][1][:,1]
                    elif q>self.sequence[i][0][1][1]:
                        q=self.sequence[i][0][1][1]
                        z=np.cos(q)*self.body[i][1][:,0]-np.sin(q)*self.body[i][1][:,1]
                    y=np.cross(z,self.body[i][1][:,0])
                    self.body[i-1][1]=np.array([self.body[i][1][:,0],y,z]).T
                    self.values[contv]=q
                    contv-=1
            q=np.arccos(self.root_orientation[:,0].dot(self.body[0][1][:,0]))*np.sign(np.dot(np.cross(self.root_orientation[:,0],self.body[0][1][:,0]),self.body[0][1][:,2]))
            if q<self.sequence[0][0][1][0]:
                q=self.sequence[0][0][1][0]
                x=np.cos(q)*self.body[0][1][:,0]-np.sin(q)*self.body[0][1][:,1]
            elif q>self.sequence[0][0][1][1]:
                q=self.sequence[0][0][1][1]
                x=np.cos(q)*self.body[0][1][:,0]-np.sin(q)*self.body[0][1][:,1]
            self.values[0]=q
        def forward(O):
            contv=0
            self.body[0][2]=self.root_position
            for i in range(len(self.sequence)):
                if self.sequence[i][0][0]=='pi':
                    i_h=find_hinge(i,2,False)
                    d=self.body[i_h][2]-self.body[i][0]
                    nd=np.linalg.norm(d)
                    if nd>0:
                        zi_1=d/nd
                    else:
                        zi_1=self.body[i_h][1][:,2]
                    if i==0:
                        x=np.cross(self.root_orientation[:,2],zi_1)
                        nx=np.linalg.norm(x)
                        if not np.isclose(nx,0):
                            x=x/nx
                        else:
                            x=self.matRotZ(self.values[0])[:,0]
                        if np.sign(x.dot(self.body[i][1][:,0]))<0:
                            x=-x
                    elif 0<i<len(self.sequence)-1:
                        x=np.cross(self.body[i-1][1][:,2],zi_1)
                        nx=np.linalg.norm(x)
                        if not np.isclose(nx,0):
                            x=x/nx
                        else:
                            x=self.body[i-1][1][:,0]
                        if np.sign(x.dot(self.body[i][1][:,0]))<0:
                            x=-x
                    else:
                        x=O[:,0]-O[:,0].dot(self.body[-1][1][:,2])*self.body[-1][1][:,2]
                        x=x/np.linalg.norm(x)
                    if i>0:
                        csn=x.dot(self.body[i-1][1][:,0])
                        if abs(csn)>1:csn=np.sign(csn)
                        q=np.arccos(csn)*np.sign(np.dot(np.cross(self.body[i-1][1][:,0],x),self.body[i-1][1][:,2]))
                    else:q=np.arccos(x.dot(self.root_orientation[:,0]))*np.sign(np.dot(np.cross(self.root_orientation[:,0],x),self.root_orientation[:,2]))
                    if q<self.sequence[i][0][1][0]:
                        q=self.sequence[i][0][1][0]
                        x=np.cos(q)*self.body[i][1][:,0]+np.sin(q)*self.body[i][1][:,1]
                    elif q>self.sequence[i][0][1][1]:
                        q=self.sequence[i][0][1][1]
                        x=np.cos(q)*self.body[i][1][:,0]+np.sin(q)*self.body[i][1][:,1]
                    if i>0:
                        y=np.cross(self.body[i-1][1][:,2],x)
                        self.body[i][1]=np.array([x,y,self.body[i-1][1][:,2]]).T
                    else:
                        y=np.cross(self.root_orientation[:,2],x)
                        self.body[i][1]=np.array([x,y,self.root_orientation[:,2]]).T
                    self.values[contv]=q
                    contv+=1
                elif self.sequence[i][0][0]=='h':
                    i_h=find_hinge(i,1,False)
                    d=self.body[i_h][2]-self.body[i][2]
                    d=d-d.dot(self.body[i-1][1][:,0])*self.body[i-1][1][:,0]
                    nd=np.linalg.norm(d)
                    if nd>0:
                        z=d/nd
                    else:
                        z=self.body[i_h][1][:,2]
                    csn=z.dot(self.body[i-1][1][:,2])
                    if abs(csn)>1:csn=np.sign(csn)
                    q=np.arccos(csn)*np.sign(np.dot(np.cross(self.body[i-1][1][:,2],z),self.body[i-1][1][:,0]))
                    if q<self.sequence[i][0][1][0]:
                        q=self.sequence[i][0][1][0]
                        z=np.cos(q)*self.body[i-1][1][:,2]+np.sin(q)*self.body[i-1][1][:,1]
                    elif q>self.sequence[i][0][1][1]:
                        q=self.sequence[i][0][1][1]
                        z=np.cos(q)*self.body[i-1][1][:,0]+np.sin(q)*self.body[i-1][1][:,1]
                    y=np.cross(z,self.body[i-1][1][:,0])
                    self.body[i][1]=np.array([self.body[i-1][1][:,0],y,z]).T
                    self.values[contv]=q
                    contv+=1
                if type(self.sequence[i][1][1])==type([]):
                    d=self.body[i][0]-self.body[i][2]
                    l=d.dot(self.body[i][1][:,2])
                    if l>self.sequence[i][1][1][1]:
                        l=self.sequence[i][1][1][1]
                    elif l<self.sequence[i][1][1][0]:
                        l=self.sequence[i][1][1][0]
                    self.values[contv]=l
                    self.body[i][0]=self.body[i][2]+l*self.body[i][1][:,2]
                    if i<len(self.sequence)-1:self.body[i+1][2]=self.body[i][0]
                    contv+=1
                else:
                    self.body[i][0]=self.body[i][2]+self.sequence[i][1][1]*self.body[i][1][:,2]
                    if i<len(self.sequence)-1:self.body[i+1][2]=self.body[i][0] 
        #backward(P,O)
        if not(O is None):
            err=error([self.body[-1][0],self.body[-1][1]],[P,O])
        else:
            err=error(self.body[-1][0],P)
        errList=[]
        indList=[]
        if err>1e-3:
            #print(err,0)
            errList.append(err)
            indList.append(0)
            for i in range(1000):
                if O is None:
                    Op=define_orientation(P)
                    backward(P,Op)
                    err=error([self.body[0][2],self.body[0][1]],[self.root_position,self.matRotZ(self.values[0])])
                    errList.append(err)
                    indList.append(i+0.5)
                    if err<1e-3:
                        break
                    #self.FK()
                    forward(Op)
                    err=error(self.body[-1][0],P)
                    errList.append(err)
                    indList.append(i+1)
                    if err<1e-3:
                        break
                else:
                    Op=O
                    backward(P,Op)
                    err=error([self.body[0][2],self.body[0][1]],[self.root_position,self.matRotZ(self.values[0])])
                    errList.append(err)
                    indList.append(i+0.5)
                    if err<1e-3:
                        break
                    #self.FK()
                    forward(Op)
                    err=error([self.body[-1][0],self.body[-1][1]],[P,Op])
                    errList.append(err)
                    indList.append(i+1)
                    if err<1e-3:
                        break
                
        #backward(P,O)
        #print(self.values)
        return errList,indList
'''
class gui:
    def __init__(self):
        self.tk_root = tk.Tk()
        self.figure = Figure(figsize=(12, 8))
        self.ax= self.figure.add_subplot(1, 1, 1, projection='3d')
        self.ax.set_xlim(-10,10)
        self.ax.set_ylim(-10,10)
        self.ax.set_zlim(0,20)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.ax.autoscale(enable=False)
        self.canvas = FigureCanvasTkAgg(self.figure, self.tk_root)
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.valins=[0,0,0,4,0,0,4,0,0,4]
        self.sr=SerialBot()
        pos,ors=self.sr.FK(self.valins)
        #**********************************
        #self.physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        #p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        #p.setGravity(0,0,-10)
        #p.setRealTimeSimulation(True)
        #self.planeId = p.loadURDF("plane.urdf")
        #cubeStartPos = [0,0,0]
        #cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        #self.robotId = p.loadURDF("amber_b1_test2.urdf",cubeStartPos, cubeStartOrientation,flags=p.URDF_USE_INERTIA_FROM_FILE,useFixedBase=True)
        #p.setJointMotorControlArray(self.robotId,range(len(self.valins)),p.POSITION_CONTROL,targetPositions=self.valins)
        #p.stepSimulation()
        #**********************************
        self.lineas=self.ax.plot(pos[0],pos[1],pos[2],linewidth=5)[0]
        self.ax.quiver([pos[0,0]]*3,[pos[1,0]]*3,[pos[2,0]]*3,*np.array([[1,0,0],[0,1,0],[0,0,1]]),color=[(1.0, .0, .0, 1.0), (.0, .0, 1.0, 1.0), (.0, 1.0, .0, 1.0)])
        self.vx=self.ax.quiver(np.concatenate([[i,i,i]for i in pos[0]]),np.concatenate([[i,i,i]for i in pos[1]]),np.concatenate([[i,i,i]for i in pos[2]]),0.1*np.concatenate(np.array([ors[i][0,:]for i in range(pos[0].shape[0])])),0.1*np.concatenate(np.array([ors[i][1,:]for i in range(pos[0].shape[0])])),0.1*np.concatenate(np.array([ors[i][2,:]for i in range(pos[0].shape[0])])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(pos[0].shape[0])])))
        #**********************************
        self.wv=tk.DoubleVar()
        self.w = tk.Scale(self.tk_root, from_=-180, to=180, orient=tk.HORIZONTAL,variable=self.wv,resolution=0.5,length=200)
        self.w.place(x=80,y=0)
        self.selection=0
        self.wv.set(self.valins[self.selection]*180/np.pi)
        self.list_box=tk.Listbox(self.tk_root,height=pos[0].shape[0]-1,width=10)
        self.list_box.bind('<<ListboxSelect>>',self.CurSelet)
        for i in range(pos[0].shape[0]-1):
            self.list_box.insert(i,'q'+str(i))
        self.list_box.place(x=0,y=0)
        self.wv2=tk.DoubleVar()
        self.w2 = tk.Scale(self.tk_root, from_=180, to=-180, orient=tk.VERTICAL,variable=self.wv2,resolution=0.5,length=200)
        self.w2.place(x=0,y=((pos.shape[0]-1)*20))
        self.wv3=tk.DoubleVar()
        self.w3 = tk.Scale(self.tk_root, from_=180, to=-180, orient=tk.VERTICAL,variable=self.wv3,resolution=0.5,length=200)
        self.w3.place(x=50,y=((pos.shape[0]-1)*20))
        self.wv4=tk.DoubleVar()
        self.w4 = tk.Scale(self.tk_root, from_=180, to=-180, orient=tk.VERTICAL,variable=self.wv4,resolution=0.5,length=200)
        self.w4.place(x=100,y=((pos.shape[0]-1)*20))
        ee,rr=(0,0,0),pos.T[-1]
        pitch,roll,yaw=ee
        self.wv2.set(pitch*180/pi)
        self.wv3.set(roll*180/pi)
        self.wv4.set(yaw*180/pi)
        self.tx,self.ty,self.tz=rr
        self.rx=self.wv2.get()*pi/180
        self.ry=self.wv3.get()*pi/180
        self.rz=self.wv4.get()*pi/180
        self.l1 = tk.Label(self.tk_root, text='|     x     |')
        self.l1.bind('<MouseWheel>',lambda ev,cord=0: self.target(ev,cord))
        self.l1.place(x=0,y=(pos.shape[0]*20)+200)
        self.l2 = tk.Label(self.tk_root, text='|     y     |')
        self.l2.bind('<MouseWheel>',lambda ev,cord=1: self.target(ev,cord))
        self.l2.place(x=50,y=(pos.shape[0]*20)+200)
        self.l3 = tk.Label(self.tk_root, text='|     z     |')
        self.l3.bind('<MouseWheel>',lambda ev,cord=2: self.target(ev,cord))
        self.l3.place(x=100,y=(pos.shape[0]*20)+200)
        self.target_list=[0]
        self.b1=tk.Button(self.tk_root, text='Add',command=lambda n=True:self.new_target(n))
        self.b1.place(x=0,y=(pos.shape[0]*20)+300)
        self.b2=tk.Button(self.tk_root, text='Remove',command=lambda n=False:self.new_target(n))
        self.b2.place(x=100,y=(pos.shape[0]*20)+300)
        self.doit=False
        self.num=0
        self.dire=1
        self.nqtnsl=[]
        self.l_p_nq=self.ax.plot([0],[0],[0],'y-.')[0]
        self.b3=tk.Button(self.tk_root, text='Start',command=lambda n=True:self.hacer(n))
        self.b3.place(x=0,y=(pos.shape[0]*20)+400)
        self.b4=tk.Button(self.tk_root, text='Stop',command=lambda n=False:self.hacer(n))
        self.b4.place(x=100,y=(pos.shape[0]*20)+400)
        #**********************************
        self.dpo=ors[-1],pos.T[-1]
        mm,rr=self.dpo
        self.vdpo=self.ax.quiver([rr[0]]*3,[rr[1]]*3,[rr[2]]*3,*mm*0.2,color=[(1.0, .0, .0, 1.0), (.0, .0, 1.0, 1.0), (.0, 1.0, .0, 1.0)])
        self.lvdpo=self.ax.quiver([0],[0],[0],[0],[0],[0])
        self.ani = FuncAnimation(self.figure, self.update2, interval=10)
    def CurSelet(self,ev):
        widget = ev.widget
        selection=widget.curselection()[0]
        self.wv.set(self.valins[selection]*180/np.pi)
    def target(self,ev,cord):
        if cord==0:
            self.tx+=np.sign(ev.delta)*0.5
        elif cord==1:
            self.ty+=np.sign(ev.delta)*0.5
        else:
            self.tz+=np.sign(ev.delta)*0.5
    def new_target(self,n):
        if n:
            self.target_list.append(self.dpo)
        else:
            self.target_list.pop()
    def update2(self,t):
        self.vdpo.remove()
        self.vx.remove()
        lx=self.wv2.get()*pi/180
        ly=self.wv3.get()*pi/180
        lz=self.wv4.get()*pi/180
        R=np.array([[1,0,0],[0,cos(lx),-sin(lx)],[0,sin(lx),cos(lx)]])@np.array([[cos(ly),0,sin(ly)],[0,1,0],[-sin(ly),0,cos(ly)]])@np.array([[cos(lz),-sin(lz),0],[sin(lz),cos(lz),0],[0,0,1]])
        P=np.array([self.tx,self.ty,self.tz])
        self.dpo=R,P
        self.rx=lx
        self.ry=ly
        self.rz=lz
        mm,rr=R,P
        #**********************************
        self.vdpo=self.ax.quiver([rr[0]]*3,[rr[1]]*3,[rr[2]]*3,*mm*2,color=[(1.0, .0, .0, 1.0), (.0, .0, 1.0, 1.0), (.0, 1.0, .0, 1.0)])
        self.sr.IK(P,R)
        #p.setJointMotorControlArray(self.robotId,range(len(self.sr.angulos)),p.POSITION_CONTROL,targetPositions=self.sr.angulos)
        #p.stepSimulation()
        #pos,ors=self.sr.plot_body()
        pos,ors=self.sr.FK()
        self.vx=self.ax.quiver(np.concatenate([[i,i,i]for i in pos[0]]),np.concatenate([[i,i,i]for i in pos[1]]),np.concatenate([[i,i,i]for i in pos[2]]),np.concatenate(np.array([ors[i][0,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][1,:]for i in range(pos[0].shape[0])])),np.concatenate(np.array([ors[i][2,:]for i in range(pos[0].shape[0])])),color=np.concatenate(np.array([[[1.0, .0, .0, 1.0], [.0, .0, 1.0, 1.0], [.0, 1.0, .0, 1.0]]for i in range(pos[0].shape[0])])))
        self.lineas.set_data(pos[0],pos[1])
        self.lineas.set_3d_properties(pos[2])
if __name__ == '__main__':
    GUI=gui()
    GUI.tk_root.mainloop()
'''
if __name__=='__main__':
    figure = plt.figure(figsize=(12, 6))
    ax= figure.add_subplot(1, 2, 1, projection='3d')
    ax.set_xlim(-8,8)
    ax.set_ylim(-4,10)
    ax.set_zlim(0,16)
    ax.set_xlabel('x (cm)')
    ax.set_ylabel('y (cm)')
    ax.set_zlabel('z (cm)')
    ax.autoscale(enable=False)

    sr=SerialBot()

    lx=3*pi/4
    ly=pi/4
    lz=pi/4
    P=np.array([10,1,10])

    R=np.array([[1,0,0],[0,cos(lx),-sin(lx)],[0,sin(lx),cos(lx)]])@np.array([[cos(ly),0,sin(ly)],[0,1,0],[-sin(ly),0,cos(ly)]])@np.array([[cos(lz),-sin(lz),0],[sin(lz),cos(lz),0],[0,0,1]])
    mm,rr=R,P

    #pos,ors=sr.FK([pi/3,-pi/6,pi/6,5,-pi/3,pi/6,6,-pi/3,pi/3,8])
    pos,ors=sr.FK()
    tim=time()
    error,ind=sr.IK(P,R)
    print("Time:",(time()-tim)*1000)
    print("Values:",sr.values)
    pos,ors=sr.FK()
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
    print("Error: ",error[-1])
    print('N:',ind)
    #print(np.array(sr.angulos)*180/np.pi)
    ax2.set_xlabel('N iterations')
    ax2.set_ylabel('RMSE')
    ax2.set_title('The Proposed Method\'s RMSE per Iteration')
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
