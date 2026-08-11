# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
# import copy

# import rospy

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

np.set_printoptions(formatter={
    'float': lambda x: (
        f"{x:10.0f}" if abs(x) < 1e-4 else
        f"{x:10,.0f}")})

# Define a matriz homogenea padrao
def hTransform(parameters):
    
    gama = parameters[0]
    b    = parameters[1]
    teta = parameters[2]
    alfa = parameters[3]
    r    = parameters[4]
    d    = parameters[5]

    H = np.array([[np.cos(gama)*np.cos(teta) - np.sin(gama)*np.cos(alfa)*np.sin(teta),
                -np.cos(gama)*np.sin(teta)-np.sin(gama)*np.cos(alfa)*np.cos(teta),
                np.sin(gama)*np.sin(alfa),
                d*np.cos(gama) + r*np.sin(gama)*np.sin(alfa)],

                [np.sin(gama)*np.cos(teta) + np.cos(gama)*np.cos(alfa)*np.sin(teta),
                -np.sin(gama)*np.sin(teta) + np.cos(gama)*np.cos(alfa)*np.cos(teta),
                -np.cos(gama)*np.sin(alfa),
                d*np.sin(gama) - r*np.cos(gama)*np.sin(alfa)],

                [np.sin(alfa)*np.sin(teta),
                np.sin(alfa)*np.cos(teta),
                np.cos(alfa),
                r*np.cos(alfa) + b],

                [0,0,0,1]])
    return H

# Matrizes de transformação nos eixos
def Tx(tetax):
    T = np.array([[1, 0            , 0             , 0],
                  [0, np.cos(tetax), -np.sin(tetax), 0],
                  [0, np.sin(tetax), np.cos(tetax) , 0],
                  [0, 0            , 0             , 1]])
    return T

def Ty(tetay):
    T = np.array([[np.cos(tetay) , 0, np.sin(tetay), 0],
                  [0             , 1, 0            , 0],
                  [-np.sin(tetay), 0, np.cos(tetay), 0],
                  [0             , 0, 0            , 1]])
    return T

def Tz(tetaz):
    T = np.array([[np.cos(tetaz), -np.sin(tetaz), 0, 0],
                  [np.sin(tetaz), np.cos(tetaz) , 0, 0],
                  [0            , 0             , 1, 0],
                  [0            , 0             , 0, 1]])
    return T

def trans(x,y,z):
    T = np.array([[1, 0, 0, x],
                  [0, 1, 0, y],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]])
    return T    

def initRobot():
    ################################################
    #variáveis de entrada:
    roll = 0
    pitch = 0
    yaw = 0
    x = 0
    y = 0
    z = 0

    c = 0.35 # comprimento do chassi
    l = 0.3 # largura do chassi
    h = 0.1 # altura do chassi

    legLenght = 0.28125
    legHeight = 0.10625

    #################################################
    
    # Valores dos parâmetros de Denavit-Hartemberg Modificado para cada perna
    #perna 1
    alpha10 = np.pi/2
    d10 = c/2

    teta11 = 0 # variável
    r11 = l/2

    gama16 = -np.pi/2

    teta12 = 2*np.pi + gama16 - teta11
    d12 = legLenght

    teta13 = 2*np.pi - gama16
    d13 = legHeight

    teta16 = teta11 - gama16
    r16 = r11
    d16 = d13

    teta17 = 2*np.pi - teta11
    d17 = d12

    teta14 = 0 # variável (ângulo de esterçamento)
    alpha14 = np.pi/2
    r14 = 0
    d14 = 0.04

    teta15 = 0 # variável (ângulo de giro da roda)
    alpha15 = 0
    r15 = 0.077
    d15 = 0 

    #---------------------------------------------------------
    #perna 2
    alpha20 = np.pi/2
    d20 = c/2

    teta21 = 0  #variável
    r21 = l/2

    gama26 = -np.pi/2

    teta22 = 2*np.pi + gama26 - teta21
    d22 = legLenght

    teta23 = 2*np.pi - gama26
    d23 = legHeight

    teta26 = teta21 - gama26
    r26 = r21
    d26 = d23

    teta27 = 2*np.pi - teta21
    d27 = d22

    teta24 = 0 #variável (ângulo de esterçamento)
    alpha24 = np.pi/2
    r24 = 0
    d24 = 0.04

    teta25 = 0
    alpha25 = 0
    r25 = 0.077
    d25 = 0   

    #---------------------------------------------------------
    #perna 3
    gama30 = np.pi
    alpha30 = np.pi/2
    d30 = c/2

    teta31 = 0 # variável
    r31 = l/2

    gama36 = -np.pi/2

    teta32 = 2*np.pi + gama36 - teta31
    d32 = legLenght

    teta33 = 2*np.pi - gama36
    d33 = legHeight

    teta36 = teta31 - gama36
    r36 = r31
    d36 = d33

    teta37 = 2*np.pi - teta31
    d37 = d32

    teta34 = 0 # variável (ângulo de esterçamento)
    alpha34 = np.pi/2
    r34 = 0
    d34 = 0.04

    teta35 = 0
    alpha35 = 0
    r35 = 0.077
    d35 = 0 

    #---------------------------------------------------------
    #perna 4
    gama40 = np.pi
    alpha40 = np.pi/2
    d40 = c/2

    teta41 = 0 # variável
    r41 = l/2

    gama46 = -np.pi/2

    teta42 = 2*np.pi + gama46 - teta41
    d42 = legLenght

    teta43 = 2*np.pi - gama46
    d43 = legHeight

    teta46 = teta41 - gama46
    r46 = r41
    d46 = d43

    teta47 = 2*np.pi - teta41
    d47 = d42

    teta44 = 0 # variável (ângulo de esterçamento)
    alpha44 = np.pi/2
    r44 = 0
    d44 = 0.04

    teta45 = 0
    alpha45 = 0
    r45 = 0.077
    d45 = 0 

    # pose do baseLink
    basePose = np.array([[roll, pitch, yaw],
                         [x,    y,      z]], dtype='float64')

    # Definição dos parâmetros de transformação para a base anterior
    # A base anterior à link0 é a baseLink
    baseLink = {'name': 'baseLink', 'gama':0    , 'b':0, 'teta':0    , 'alpha':0, 'r':0   , 'd':0, 'children':[]} 
    
    #Perna 1
    link10 = {'name': 'link10', 'gama':0     , 'b':0, 'teta':0     , 'alpha':alpha10,'r':0   , 'd':d10, 'children':[]} 
    link11 = {'name': 'link11', 'gama':0     , 'b':0, 'teta':teta11, 'alpha':0      ,'r':r11 , 'd':0  , 'children':[]} 
    link12 = {'name': 'link12', 'gama':0     , 'b':0, 'teta':teta12, 'alpha':0      ,'r':0   , 'd':d12, 'children':[]} 
    link13 = {'name': 'link13', 'gama':0     , 'b':0, 'teta':teta13, 'alpha':0      ,'r':0   , 'd':d13, 'children':[]} 
    link14 = {'name': 'link14', 'gama':0     , 'b':0, 'teta':teta14, 'alpha':alpha14,'r':r14 , 'd':d14, 'children':[]} 
    link15 = {'name': 'link15', 'gama':0     , 'b':0, 'teta':teta15, 'alpha':alpha15,'r':r15 , 'd':d15, 'children':[]}  
    link16 = {'name': 'link16', 'gama':gama16, 'b':0, 'teta':teta16, 'alpha':0      ,'r':r16 , 'd':d16, 'children':[]} 
    link17 = {'name': 'link17', 'gama':0     , 'b':0, 'teta':teta17, 'alpha':0      ,'r':0   , 'd':d17, 'children':[]} 
   
    #Perna 2
    link20 = {'name': 'link20', 'gama':0    , 'b':0, 'teta':0     , 'alpha':alpha20,'r':0   , 'd':d20, 'children':[]} 
    link21 = {'name': 'link21', 'gama':0    , 'b':0, 'teta':teta21, 'alpha':0,      'r':-r21, 'd':0  , 'children':[]} 
    link22 = {'name': 'link22', 'gama':0    , 'b':0, 'teta':teta22, 'alpha':0,      'r':0   , 'd':d22, 'children':[]} 
    link23 = {'name': 'link23', 'gama':0    , 'b':0, 'teta':teta23, 'alpha':0,      'r':0   , 'd':d23, 'children':[]} 
    link24 = {'name': 'link24', 'gama':0    , 'b':0, 'teta':teta24, 'alpha':alpha24,'r':r24 , 'd':d24, 'children':[]} 
    link25 = {'name': 'link25', 'gama':0    , 'b':0, 'teta':teta25, 'alpha':alpha25,'r':r25 , 'd':d25, 'children':[]}  
    link26 = {'name': 'link26', 'gama':gama26,'b':0, 'teta':teta26, 'alpha':0     , 'r':-r26, 'd':d26, 'children':[]} 
    link27 = {'name': 'link27', 'gama':0    , 'b':0, 'teta':teta27, 'alpha':0     , 'r':0   , 'd':d27, 'children':[]} 

    #Perna 3
    link30 = {'name': 'link30', 'gama':gama30, 'b':0, 'teta':0    ,  'alpha':alpha30, 'r':0    , 'd':d30 ,'children':[]} 
    link31 = {'name': 'link31', 'gama':0     , 'b':0, 'teta':teta31, 'alpha':0,       'r':r31  , 'd':0   ,'children':[]} 
    link32 = {'name': 'link32', 'gama':0     , 'b':0, 'teta':teta32, 'alpha':0,       'r':0    , 'd':d32 ,'children':[]} 
    link33 = {'name': 'link33', 'gama':0     , 'b':0, 'teta':teta33, 'alpha':0,       'r':0    , 'd':d33 ,'children':[]} 
    link34 = {'name': 'link34', 'gama':0     , 'b':0, 'teta':teta34, 'alpha':alpha34, 'r':r34  , 'd':d34 ,'children':[]} 
    link35 = {'name': 'link35', 'gama':0     , 'b':0, 'teta':teta35, 'alpha':alpha35, 'r':r35  , 'd':d35 ,'children':[]}
    link36 = {'name': 'link36', 'gama':gama36, 'b':0, 'teta':teta36, 'alpha':0     ,  'r':r36  , 'd':d36 ,'children':[]} 
    link37 = {'name': 'link37', 'gama':0     , 'b':0, 'teta':teta37, 'alpha':0     ,  'r':0    , 'd':d37 ,'children':[]} 

    #Perna 4
    link40 = {'name': 'link40', 'gama':gama40, 'b':0, 'teta':0     , 'alpha':alpha40, 'r':0    , 'd':d40 , 'children':[]} 
    link41 = {'name': 'link41', 'gama':0     , 'b':0, 'teta':teta41, 'alpha':0,       'r':-r41 , 'd':0   , 'children':[]} 
    link42 = {'name': 'link42', 'gama':0     , 'b':0, 'teta':teta42, 'alpha':0,       'r':0    , 'd':d42 , 'children':[]} 
    link43 = {'name': 'link43', 'gama':0     , 'b':0, 'teta':teta43, 'alpha':0,       'r':0    , 'd':d43 , 'children':[]} 
    link44 = {'name': 'link44', 'gama':0     , 'b':0, 'teta':teta44, 'alpha':alpha44, 'r':-r44 , 'd':d44 , 'children':[]} 
    link45 = {'name': 'link45', 'gama':0     , 'b':0, 'teta':teta45, 'alpha':alpha45, 'r':r45  , 'd':d45 , 'children':[]}  
    link46 = {'name': 'link46', 'gama':gama46, 'b':0, 'teta':teta46, 'alpha':0     ,  'r':-r46 , 'd':d46 , 'children':[]} 
    link47 = {'name': 'link47', 'gama':0     , 'b':0, 'teta':teta47, 'alpha':0     ,  'r':0    , 'd':d47 , 'children':[]} 


    #define as cadeias cinemáticas
    #define as cadeias cinemáticas do baseLink
    baseLink['children'].append(link10)
    baseLink['children'].append(link20)
    baseLink['children'].append(link30)
    baseLink['children'].append(link40)

    #Perna 1
    link10['children'].append(link11)
    link11['children'].append(link12)
    link12['children'].append(link13)
    link13['children'].append(link14)
    link14['children'].append(link15)
    
    link10['children'].append(link16)
    link16['children'].append(link17)

    #Perna 2
    link20['children'].append(link21)
    link21['children'].append(link22)
    link22['children'].append(link23)
    link23['children'].append(link24)
    link24['children'].append(link25)
    
    link20['children'].append(link26)
    link26['children'].append(link27)

    #Perna 3
    link30['children'].append(link31)
    link31['children'].append(link32)
    link32['children'].append(link33)
    link33['children'].append(link34)
    link34['children'].append(link35)
    
    link30['children'].append(link36)
    link36['children'].append(link37)

    #Perna 4    
    link40['children'].append(link41)
    link41['children'].append(link42)
    link42['children'].append(link43)
    link43['children'].append(link44)
    link44['children'].append(link45)
    
    link40['children'].append(link46)
    link46['children'].append(link47)

    # return basePose, 'baseLink', baseLink, ['link15', 'link25', 'link35', 'link45']
# return basePose, 'baseLink', baseLink, ['link15', 'link25', 'link35', 'link45']

    return baseLink

def iterLink(link):
        row = [link['gama'] , link['b'], link['teta'] , link['alpha'] , link['r'] , link['d']]
        if len(link['children']) == 0:
            return {link['name']:[row]}

        nestedChains = {}
        for children in link['children']:
            chains = iterLink(children)
            for key, c in chains.items():
                c.insert(0,row[:])
            nestedChains.update(chains)
        return nestedChains


def rec(child):
    if len(child['children']) != 0:
        return rec(child['children'])
     
    print(len(child['children']))    
    

    
    
def main():
    robotParam = initRobot()
    
    for child in robotParam['children']:
        leg_chain_params = iterLink(child)
        print(leg_chain_params.items())        
        # for key, c in chains.items():
        #     print(c, sep="\n")
        #     print("************")
        #     for line in c:
        #         for x in line:
        #             print(x)
        #         print("************")

    # print('baseLink', np.shape(robotParam))
    # for chain in robotParam:
    #     for mat in chain:
    #         mat_formatted = [ '%.2f' % elem for elem in mat ]
    #         print(mat_formatted, sep="\n")
    #         # "{0:0.2f}".format(round(mat_formatted, 2))
    #     print("***********")



if __name__ == '__main__':
    main()