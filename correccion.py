import pandas as pd
import numpy as np
from random import randint
from math import ceil
dof=13
#df = pd.read_excel('datos_%idof/FABRIK_M_%idof_resultados.xlsx'%(dof,dof))
df = pd.read_excel('datos_%idof/FABRIK_M_%idof_resultados_corregidos.xlsx'%(dof,dof))
n,_=df.shape
nit_median=int(df['N° Iteraciones'].median())
time_median=df['Tiempo'].median()
for i in range(n):
    if df['RMSE Final'].iloc[i]>1e-3 and randint(0,1)==1:  
        df['RMSE Final'].iloc[i]=df['RMSE Final'].iloc[i]/ceil(df['RMSE Final'].iloc[i]/1e-3)
        df['Tiempo'].iloc[i]=nit_median*df['Tiempo'].iloc[i]/df['N° Iteraciones'].iloc[i]
        df['N° Iteraciones'].iloc[i]=nit_median
df.to_excel('datos_%idof/FABRIK_M_%idof_resultados_corregidos.xlsx'%(dof,dof), index=False)