import pandas as pd
import matplotlib.pyplot as plt
#Tiempo | N° Iteraciones | RMSE Final
criterio='RMSE Final'
valor=1e-3
dof=15
#DLS | FABRIK_M | GA | PSO | QPSO
metodo=['DLS','FABRIK_M','GA','PSO','QPSO']
goals = pd.read_excel('datos_%idof/datos_%idof.xlsx'%(dof,dof),usecols=['x','y','z','pitch','roll','yaw'])
df_dls = pd.read_excel('datos_%idof/DLS_%idof_resultados.xlsx'%(dof,dof))
df_fabrik = pd.read_excel('datos_%idof/FABRIK_M_%idof_resultados_corregidos.xlsx'%(dof,dof))
df_ga = pd.read_excel('datos_%idof/GA_%idof_resultados.xlsx'%(dof,dof))
df_pso = pd.read_excel('datos_%idof/PSO_%idof_resultados.xlsx'%(dof,dof))
df_qpso = pd.read_excel('datos_%idof/QPSO_%idof_resultados.xlsx'%(dof,dof))

punts_fabrik=goals[df_fabrik[criterio]>=valor]
punts_dls=goals[df_dls[criterio]>=valor]
punts_ga=goals[df_ga[criterio]>=valor]
punts_pso=goals[df_pso[criterio]>=valor]
punts_qpso=goals[df_qpso[criterio]>=valor]

axes = plt.subplot(111, projection='3d')
axes.scatter(punts_fabrik['x'].tolist(), punts_fabrik['y'].tolist(), punts_fabrik['z'].tolist(), marker="*", label="%i FABRIK"%(punts_fabrik.shape[0]))
axes.scatter(punts_dls['x'].tolist(), punts_dls['y'].tolist(), punts_dls['z'].tolist(), marker="o", label="%i DLS"%(punts_dls.shape[0]))
axes.scatter(punts_ga['x'].tolist(), punts_ga['y'].tolist(), punts_ga['z'].tolist(), marker="^", label="%i GA"%(punts_ga.shape[0]))
axes.scatter(punts_pso['x'].tolist(), punts_pso['y'].tolist(), punts_pso['z'].tolist(), marker=".", label="%i PSO"%(punts_pso.shape[0]))
axes.scatter(punts_qpso['x'].tolist(), punts_qpso['y'].tolist(), punts_qpso['z'].tolist(), marker="p", label="%i QPSO"%(punts_qpso.shape[0]))
axes.set_xlabel('x (cm)')
axes.set_ylabel('y (cm)')
axes.set_zlabel('z (cm)')
plt.legend(loc="upper right")
plt.show()
