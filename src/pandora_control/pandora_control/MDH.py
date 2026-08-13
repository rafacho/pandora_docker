#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# Define a matriz homogenea padrao
def hTransform(parameters):

    gama = parameters[0]
    b    = parameters[1]
    teta = parameters[2]
    alfa = parameters[3]
    r    = parameters[4]
    d    = parameters[5]

    # FIXME(ros2-port): r and d are swapped in the translation column below
    # (should be r*cos(gama)+d*sin(gama)*sin(alfa), r*sin(gama)-d*cos(gama)*sin(alfa),
    # d*cos(alfa)+b for the rotation matrix actually implemented here). Ported
    # unchanged per the "preserve control-law bugs, fix later" decision -- see
    # code review 2026-08-04.
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

    legLenght = 0.3
    legHeight = 0.1

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

    return basePose, 'baseLink', baseLink, ['link15', 'link25', 'link35', 'link45']

class robotModel:
    def __init__(self, basePose, baseLinkName, kinematicTree, endEffectorsNames):
        # self.basePose = basePose
        # self.baseLinkName = baseLinkName
        self.resetKinematicTree = kinematicTree
        self.endEffectorsNames = endEffectorsNames
        self.imuPose = np.array([[0, 0, 0], [0, 0, 0]])
        self.angles = np.array([0, 0, 0, 0])

    def iterLink(self, link):
        row = [link['gama'] , link['b'], link['teta'] , link['alpha'] , link['r'] , link['d']]
        if len(link['children']) == 0:
            return {link['name']:[row]}

        nestedChains = {}
        for children in link['children']:
            chains = self.iterLink(children)
            for key, c in chains.items():
                c.insert(0,row[:])
            nestedChains.update(chains)
        return nestedChains

    def move(self, linkName, angleName, angleValue, kinematicTree):

        if linkName == 'link11':
            linkNames  = ['link11', 'link12', 'link13', 'link16', 'link17']
        elif linkName == 'link21':
            linkNames  = ['link21', 'link22', 'link23', 'link26', 'link27']
        elif linkName == 'link31':
            linkNames  = ['link31', 'link32', 'link33', 'link36', 'link37']
        elif linkName == 'link41':
            linkNames  = ['link41', 'link42', 'link43', 'link46', 'link47']

        gama6 = -np.pi/2
        teta1 = angleValue
        teta2 = 2*np.pi + gama6 - teta1
        teta3 = 2*np.pi - gama6
        teta6 = teta1 - gama6
        teta7 = 2*np.pi - teta1

        legArray = [teta1, teta2, teta3, teta6, teta7]

        for link in linkNames:
            self._move(link, 'teta', legArray[linkNames.index(link)], kinematicTree)
        return kinematicTree

    def _move(self, linkName, angleName, anglevalue, link):
        if link['name'] == linkName:
            link[angleName] = anglevalue
        for l in link['children']:
            self._move(linkName, angleName, anglevalue, l)

        return link

    def setState(self, basePose, legAngles, kinematicTree):
        legs = ['link11', 'link21', 'link31', 'link41']
        angleValues = [legAngles['teta11'], legAngles['teta21'], legAngles['teta31'], legAngles['teta41']]

        newTree = kinematicTree
        for leg in legs:
            l = legs.index(leg)
            #              move(linkName, angleName, angleValue, kinematicTree)
            newTree = self.move(leg,'teta', angleValues[l], newTree)

        pose = self.pose(basePose, newTree, useEff = True)

        return pose

    def pose(self, basePose, kinematicTree, useEff = True):

        roll  = basePose[0,0]
        pitch = basePose[0,1]
        yaw   = basePose[0,2]

        x = basePose[1,0]
        y = basePose[1,1]
        z = basePose[1,2]


        # Transformação da base inercial para a base do chassi (dado pela IMU + translação)
        Tw_b = np.matmul(trans(x, y, z), np.matmul(Tz(yaw), np.matmul(Ty(pitch), Tx(roll))))
        transformArray = []

        if useEff == True:
            chains = [self.iterLink(kinematicTree)[endLink] for endLink in self.endEffectorsNames]
        else:
            chains = self.iterLink(kinematicTree).values()


        for endLink in chains:
            Ta_j = [np.matmul(hTransform(endLink[0]), Tw_b)]
            for i in range(1, len(endLink)):

                Ta_j.append(np.matmul(Ta_j[i-1], hTransform(endLink[i])))
            transformArray.append(Ta_j)

        return transformArray

    def plotPose(self, pose):
        x = int(pose[0][0][0][3])
        y = int(pose[0][0][1][3])
        z = int(pose[0][0][2][3])

        contacts = self.contacts(pose)
        xContacts = contacts[:,0]
        yContacts = contacts[:,1]

        xMax = xContacts[xContacts.argmax()]
        xMin = xContacts[xContacts.argmin()]

        yMax = yContacts[yContacts.argmax()]
        yMin = yContacts[yContacts.argmin()]

        xArray = np.linspace(xMin, xMax, 5)
        yArray = np.linspace(yMin, yMax, 5)

        xx, yy = np.meshgrid(xArray, yArray)

        point, normal = self.findSupportPlane(pose)
        d = -np.dot(point,normal)

        zz = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')

        fig.tight_layout()
        fig.subplots_adjust(left=-0.05, bottom=-0.01, right=0.95, top=1.1)

        ax.plot_surface(xx, yy, zz, color='grey', alpha=0.5)

        for Ta_j in pose:
            xs = []
            ys = []
            zs = []
            for linkPos in Ta_j:
                xs.append(linkPos[0,3])
                ys.append(linkPos[1,3])
                zs.append(linkPos[2,3])

            ax.plot(xs, ys, zs)

        # Without this, matplotlib scales each 3D axis independently to fit
        # that call's data range -- so the same real-world distance between
        # two points looks different from one plotPose() call to the next as
        # the leg pose (and thus the plotted extent) changes, even though
        # nothing in the kinematics actually moved.
        ax.set_box_aspect((1, 1, 1))

        plt.show()

    def contacts(self, pose):

        px=[]
        py=[]
        pz=[]

        for line in pose:
            m=0
            for matrix in line:
                if(m == 6):
                    px.append(matrix[0,3])
                    py.append(matrix[1,3])
                    pz.append(matrix[2,3])
                m = m+1
        contacts = [px,py,pz]

        points = np.transpose(contacts)

        return points

    def findSupportPlane(self, pose):
        contacts = self.contacts(pose)
        coords = np.array(contacts)
        pl = []

        # barycenter of the points
        # compute centered coordinates
        G = coords.sum(axis=0) / coords.shape[0]

        # run SVD
        u, s, vh = np.linalg.svd(coords - G)

        # unitary normal vector
        normal = vh[2, :]

        if normal[2] < 0:
            normal = -1 * normal

        pl.append(G)
        pl.append(normal)

        return pl

    def distanceToPlane(self, point, plane):
        p = np.array(plane[0])
        normal = np.array(plane[1])

        nHat = normal / np.linalg.norm(normal)

        D = np.dot(nHat, (point-p))

        return D

    def inverseKinematics(self, currentLegAngles, currentBasePose, desiredBasePose, kinematicTree):

        # FIXME(ros2-port): mutates currentBasePose/desiredBasePose in place --
        # callers that pass a reference they still hold elsewhere (e.g.
        # ik_server.py's self.robot.imuPose) get that state silently zeroed
        # here. Ported unchanged per the "preserve control-law bugs, fix later"
        # decision -- see code review 2026-08-04.
        # zera coordenadas que não influenciam na estabilidade
        currentBasePose[0][2] = 0 # yaw
        currentBasePose[1][0] = 0 # x
        currentBasePose[1][1] = 0 # y

        desiredBasePose[0][2] = 0 # yaw
        desiredBasePose[1][0] = 0 # x
        desiredBasePose[1][1] = 0 # y

            

        currentPose = self.setState(currentBasePose, currentLegAngles, kinematicTree)
        supportPlane = self.findSupportPlane(currentPose)

        tolerance = 0.005

        jointAngles = []

        angleMin = {'teta11': -np.pi/4, 'teta21': -np.pi/4, 'teta31': -np.pi/4, 'teta41': -np.pi/4}
        angleMax = {'teta11': np.pi/4, 'teta21': np.pi/4, 'teta31': np.pi/4, 'teta41': np.pi/4}
        error = {'teta11': 1, 'teta21': 1, 'teta31': 1, 'teta41': 1}
        legAngles = {'teta11': 0, 'teta21': 0, 'teta31': 0, 'teta41': 0}
        br = 0
        hmax = 0.368

        zError = desiredBasePose[1][2] - currentBasePose[1][2]
        if zError >= hmax:
            desiredBasePose[1][2] = hmax
            print('************* Set Point z limit exceeded ***************')
            print('z changed to:', hmax)

        # distBasePlane = self.distanceToPlane(desiredBasePose[1], supportPlane)
        # if distBasePlane > hmax:
        #     print('************* Set Point z limit exceeded ***************')
        #     success = False
        #     for angle in range(4):
        #         jointAngles.append(-np.pi/4)
           
        #     legAngles = {
        #         'teta11': jointAngles[0], 'teta21': jointAngles[1],
        #         'teta31': jointAngles[2], 'teta41': jointAngles[3],
        #     }
        #     # print('desiredBasePose', desiredBasePose)
        #     # print('supportPlane', supportPlane)
        #     # pose = self.setState(desiredBasePose, legAngles, kinematicTree)
        #     # newSupportPlane = self.findSupportPlane(pose)
        #     # print('newSupportPlane', newSupportPlane)
        #     # distBasePlane = self.distanceToPlane(pose, supportPlane)
        #     # zDiff = desiredBasePose[1][2] - newSupportPlane[0][2]
        #     # desiredBasePose[1][2] -= zDiff
        #     # print('**desiredBasePose changed to:', desiredBasePose)
        # else:

        while max(error.values()) >= tolerance:
            dist = []
            if error['teta11'] >= tolerance:
                legAngles['teta11'] = (angleMin['teta11'] + angleMax['teta11'])/2
            if error['teta21'] >= tolerance:
                legAngles['teta21'] = (angleMin['teta21'] + angleMax['teta21'])/2
            if error['teta31'] >= tolerance:
                legAngles['teta31'] = (angleMin['teta31'] + angleMax['teta31'])/2
            if error['teta41'] >= tolerance:
                legAngles['teta41'] = (angleMin['teta41'] + angleMax['teta41'])/2

            pose = self.setState(desiredBasePose, legAngles, kinematicTree)
            contacts = self.contacts(pose)

            for l in range(4):
                contactPoint = contacts[l]
                dist.append(self.distanceToPlane(contactPoint, supportPlane))

            distDic = {'teta11': dist[0], 'teta21': dist[1], 'teta31': dist[2], 'teta41': dist[3]}

            for leg in distDic:
                if distDic[leg] >= 0:
                    angleMax[leg] = legAngles[leg]
                else:
                    angleMin[leg] = legAngles[leg]

            error = {'teta11': abs(dist[0]), 'teta21': abs(dist[1]), 'teta31': abs(dist[2]), 'teta41': abs(dist[3])}

            if br>25:
                print('break:')
                print('error:', error)
                print('basePose', currentBasePose)
                print('-----------------------------')
                success = False
                break
            else:
                success = True
            br = br + 1

            for leg in legAngles:
                jointAngles.append(legAngles[leg])

        return {'success': success, 'joint_cmd_angles': jointAngles}

    def jacobian(self, pose):
        # FIXME(ros2-port): accumulates z from the *running* z instead of the
        # local z-axis of each linkPos, so joint axes past the first are wrong.
        # Not called anywhere in this workspace today. Ported unchanged per
        # the "preserve control-law bugs, fix later" decision -- see code
        # review 2026-08-04.
        Jtotal = []
        for Ta_j in pose:
            r_eff = Ta_j[-1][0:3,3]
            J = []
            z = np.array([0,0,1,1])

            for linkPos in Ta_j:
                r = linkPos[0:3,3]
                z = np.matmul(linkPos,z)

                z3 = z[0:3]
                z3/=np.linalg.norm(z3)

                Jv = np.cross(z3,(r_eff - r))

                J.append(np.concatenate([Jv, z3]))

            Jtotal.append(np.array(J).T)
        return Jtotal
