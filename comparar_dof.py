import pandas as pd
import matplotlib.pyplot as plt

#Tiempo | N° Iteraciones | RMSE Final
criterio='Tiempo'
#DLS | FABRIK_M | GA | PSO | QPSO
metodo=['DLS','FABRIK_M','GA','PSO','QPSO']
fig, axes = plt.subplots(3,2) # create figure and axes
fig.suptitle(criterio)
for i,c in enumerate(metodo):
    # Cargar los archivos Excel en DataFrames
    m7dof = pd.read_excel('datos_%idof/%s_%idof_resultados.xlsx'%(7,c,7))
    m9dof = pd.read_excel('datos_%idof/%s_%idof_resultados.xlsx'%(9,c,9))
    m11dof = pd.read_excel('datos_%idof/%s_%idof_resultados.xlsx'%(11,c,11))
    m13dof = pd.read_excel('datos_%idof/%s_%idof_resultados.xlsx'%(13,c,13))
    m15dof = pd.read_excel('datos_%idof/%s_%idof_resultados.xlsx'%(15,c,15))

    # Obtener la columna "tiempo" de cada DataFrame

    m7dof = m7dof[criterio]
    m9dof = m9dof[criterio]
    m11dof = m11dof[criterio]
    m13dof = m13dof[criterio]
    m15dof = m15dof[criterio]

    # Crear un DataFrame combinando los tiempos de todos los algoritmos
    df_tiempo = pd.DataFrame({
        '7 DOF': m7dof,
        '9 DOF': m9dof,
        '11 DOF': m11dof,
        '13 DOF': m13dof,
        '15 DOF': m15dof
    })

    df_tiempo.boxplot(column=['7 DOF', '9 DOF', '11 DOF','13 DOF','15 DOF'], ax=axes.flatten()[i]) 
    axes.flat[i].set_title(c)


plt.tight_layout() 
plt.show()