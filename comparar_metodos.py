import pandas as pd
import matplotlib.pyplot as plt

#Tiempo | N° Iteraciones | RMSE Final
columna="Tiempo"
fig, axes = plt.subplots(3,2) # create figure and axes
fig.suptitle(columna)
grados=[7,9,11,13,15]
for i,dof in enumerate(grados):
    # Cargar los archivos Excel en DataFrames
    df_dls = pd.read_excel('datos_%idof/DLS_%idof_resultados.xlsx'%(dof,dof))
    df_fabrik = pd.read_excel('datos_%idof/FABRIK_M_%idof_resultados.xlsx'%(dof,dof))
    df_ga = pd.read_excel('datos_%idof/GA_%idof_resultados.xlsx'%(dof,dof))
    df_pso = pd.read_excel('datos_%idof/PSO_%idof_resultados.xlsx'%(dof,dof))
    df_qpso = pd.read_excel('datos_%idof/QPSO_%idof_resultados.xlsx'%(dof,dof))

    # Obtener la columna "tiempo" de cada DataFrame

    tiempo_dls = df_dls[columna]
    tiempo_fabrik = df_fabrik[columna]
    tiempo_ga = df_ga[columna]
    tiempo_pso = df_pso[columna]
    tiempo_qpso = df_qpso[columna]

    # Crear un DataFrame combinando los tiempos de todos los algoritmos
    df_tiempo = pd.DataFrame({
        'DLS': tiempo_dls,
        'FABRIK': tiempo_fabrik,
        'GA': tiempo_ga,
        'PSO': tiempo_pso,
        'QPSO': tiempo_qpso
    })

    df_tiempo.boxplot(column=['DLS', 'FABRIK', 'GA','PSO','QPSO'], ax=axes.flatten()[i]) 
    axes.flat[i].set_title("DOF: %i"%dof)


plt.tight_layout() 
plt.show()