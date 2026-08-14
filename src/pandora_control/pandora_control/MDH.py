#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import numpy as np

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# Define a matriz homogenea padrao
def h_transform(parameters):

    gamma = parameters[0]
    b    = parameters[1]
    theta = parameters[2]
    alpha = parameters[3]
    r    = parameters[4]
    d    = parameters[5]

    h = np.array([[np.cos(gamma)*np.cos(theta) - np.sin(gamma)*np.cos(alpha)*np.sin(theta),
                -np.cos(gamma)*np.sin(theta)-np.sin(gamma)*np.cos(alpha)*np.cos(theta),
                np.sin(gamma)*np.sin(alpha),
                d*np.cos(gamma) + r*np.sin(gamma)*np.sin(alpha)],

                [np.sin(gamma)*np.cos(theta) + np.cos(gamma)*np.cos(alpha)*np.sin(theta),
                -np.sin(gamma)*np.sin(theta) + np.cos(gamma)*np.cos(alpha)*np.cos(theta),
                -np.cos(gamma)*np.sin(alpha),
                d*np.sin(gamma) - r*np.cos(gamma)*np.sin(alpha)],

                [np.sin(alpha)*np.sin(theta),
                np.sin(alpha)*np.cos(theta),
                np.cos(alpha),
                r*np.cos(alpha) + b],

                [0,0,0,1]])
    return h

# Matrizes de transformação nos eixos
def tx(thetax):
    t = np.array([[1, 0            , 0             , 0],
                  [0, np.cos(thetax), -np.sin(thetax), 0],
                  [0, np.sin(thetax), np.cos(thetax) , 0],
                  [0, 0            , 0             , 1]])
    return t

def ty(thetay):
    t = np.array([[np.cos(thetay) , 0, np.sin(thetay), 0],
                  [0             , 1, 0            , 0],
                  [-np.sin(thetay), 0, np.cos(thetay), 0],
                  [0             , 0, 0            , 1]])
    return t

def tz(thetaz):
    t = np.array([[np.cos(thetaz), -np.sin(thetaz), 0, 0],
                  [np.sin(thetaz), np.cos(thetaz) , 0, 0],
                  [0            , 0             , 1, 0],
                  [0            , 0             , 0, 1]])
    return t

def trans(x,y,z):
    t = np.array([[1, 0, 0, x],
                  [0, 1, 0, y],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]])
    return t

def init_robot():
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

    leg_length = 0.3
    leg_height = 0.1

    #################################################

    # Valores dos parâmetros de Denavit-Hartemberg Modificado para cada perna.
    # As 4 pernas compartilham a mesma cadeia cinemática; o que muda entre elas
    # é o angulo de montagem no chassi (gamma0: 0 para as pernas frontais,
    # pi para as traseiras) e o lado do chassi (r_sign: espelha o offset r).
    def build_leg(n, gamma0, r_sign):
        alpha_n0 = np.pi/2
        d_n0 = c/2

        theta_n1 = 0 # variável
        r_n1 = r_sign * (l/2)

        gamma_n6 = -np.pi/2

        theta_n2 = 2*np.pi + gamma_n6 - theta_n1
        d_n2 = leg_length

        theta_n3 = 2*np.pi - gamma_n6
        d_n3 = leg_height

        theta_n6 = theta_n1 - gamma_n6
        r_n6 = r_n1
        d_n6 = d_n3

        theta_n7 = 2*np.pi - theta_n1
        d_n7 = d_n2

        theta_n4 = 0 # variável (ângulo de esterçamento)
        alpha_n4 = np.pi/2
        r_n4 = 0
        d_n4 = 0.04

        theta_n5 = 0 # variável (ângulo de giro da roda)
        alpha_n5 = 0
        r_n5 = 0.077
        d_n5 = 0

        link_n0 = {'name': f'link{n}0', 'gamma':gamma0  , 'b':0, 'theta':0       , 'alpha':alpha_n0, 'r':0   , 'd':d_n0, 'children':[]}
        link_n1 = {'name': f'link{n}1', 'gamma':0       , 'b':0, 'theta':theta_n1, 'alpha':0       , 'r':r_n1, 'd':0   , 'children':[]}
        link_n2 = {'name': f'link{n}2', 'gamma':0       , 'b':0, 'theta':theta_n2, 'alpha':0       , 'r':0   , 'd':d_n2, 'children':[]}
        link_n3 = {'name': f'link{n}3', 'gamma':0       , 'b':0, 'theta':theta_n3, 'alpha':0       , 'r':0   , 'd':d_n3, 'children':[]}
        link_n4 = {'name': f'link{n}4', 'gamma':0       , 'b':0, 'theta':theta_n4, 'alpha':alpha_n4, 'r':r_n4, 'd':d_n4, 'children':[]}
        link_n5 = {'name': f'link{n}5', 'gamma':0       , 'b':0, 'theta':theta_n5, 'alpha':alpha_n5, 'r':r_n5, 'd':d_n5, 'children':[]}
        link_n6 = {'name': f'link{n}6', 'gamma':gamma_n6, 'b':0, 'theta':theta_n6, 'alpha':0       , 'r':r_n6, 'd':d_n6, 'children':[]}
        link_n7 = {'name': f'link{n}7', 'gamma':0       , 'b':0, 'theta':theta_n7, 'alpha':0       , 'r':0   , 'd':d_n7, 'children':[]}

        link_n0['children'].append(link_n1)
        link_n1['children'].append(link_n2)
        link_n2['children'].append(link_n3)
        link_n3['children'].append(link_n4)
        link_n4['children'].append(link_n5)

        link_n0['children'].append(link_n6)
        link_n6['children'].append(link_n7)

        return link_n0

    # pose do base_link
    base_pose = np.array([[roll, pitch, yaw],
                         [x,    y,      z]], dtype='float64')

    # Definição dos parâmetros de transformação para a base anterior
    # A base anterior à link0 é a baseLink
    base_link = {'name': 'baseLink', 'gamma':0    , 'b':0, 'theta':0    , 'alpha':0, 'r':0   , 'd':0, 'children':[]}

    base_link['children'].append(build_leg(1, gamma0=0     , r_sign= 1))
    base_link['children'].append(build_leg(2, gamma0=0     , r_sign=-1))
    base_link['children'].append(build_leg(3, gamma0=np.pi , r_sign= 1))
    base_link['children'].append(build_leg(4, gamma0=np.pi , r_sign=-1))

    return base_pose, 'baseLink', base_link, ['link15', 'link25', 'link35', 'link45']

class RobotModel:
    HMAX = 0.368               # distancia maxima base->support_plane (m)
    HMIN = 0.05                # distancia minima base->support_plane (m)
    IK_TOLERANCE = 0.005       # tolerancia de convergencia da bissecao (m)
    LEG_ANGLE_LIMIT = np.pi/4  # limite de esterçamento de cada perna (rad)
    IK_MAX_ITERATIONS = 15     # iteracoes de bissecao antes de desistir

    def __init__(self, base_pose, base_link_name, kinematic_tree, end_effectors_names):
        # self.base_pose = base_pose
        # self.base_link_name = base_link_name
        self.reset_kinematic_tree = kinematic_tree
        self.end_effectors_names = end_effectors_names
        # imu_pose/angles are not read by any method in this class -- they are a
        # live-state mailbox written by ik_server.py's IMU and joint_states
        # callbacks, then read back by ik_server.py when building IK requests.
        self.imu_pose = np.array([[0, 0, 0], [0, 0, 0]])
        self.angles = np.array([0, 0, 0, 0])

    def iter_link(self, link):
        row = [link['gamma'] , link['b'], link['theta'] , link['alpha'] , link['r'] , link['d']]
        if len(link['children']) == 0:
            return {link['name']:[row]}

        nested_chains = {}
        for children in link['children']:
            chains = self.iter_link(children)
            for key, c in chains.items():
                c.insert(0,row[:])
            nested_chains.update(chains)
        return nested_chains

    def move(self, link_name, angle_name, angle_value, kinematic_tree):

        if link_name == 'link11':
            link_names  = ['link11', 'link12', 'link13', 'link16', 'link17']
        elif link_name == 'link21':
            link_names  = ['link21', 'link22', 'link23', 'link26', 'link27']
        elif link_name == 'link31':
            link_names  = ['link31', 'link32', 'link33', 'link36', 'link37']
        elif link_name == 'link41':
            link_names  = ['link41', 'link42', 'link43', 'link46', 'link47']
        else:
            raise ValueError(f"move: link_name unknown: {link_name!r}")

        gamma6 = -np.pi/2
        theta1 = angle_value
        theta2 = 2*np.pi + gamma6 - theta1
        theta3 = 2*np.pi - gamma6
        theta6 = theta1 - gamma6
        theta7 = 2*np.pi - theta1

        leg_array = [theta1, theta2, theta3, theta6, theta7]

        for link, angle in zip(link_names, leg_array):
            self._move(link, 'theta', angle, kinematic_tree)
        return kinematic_tree

    def _move(self, link_name, angle_name, angle_value, link):
        if link['name'] == link_name:
            link[angle_name] = angle_value
            return True
        for l in link['children']:
            if self._move(link_name, angle_name, angle_value, l):
                return True

        return False

    def set_state(self, base_pose, leg_angles, kinematic_tree):
        legs = ['link11', 'link21', 'link31', 'link41']
        angle_values = [leg_angles['theta11'], leg_angles['theta21'], leg_angles['theta31'], leg_angles['theta41']]

        new_tree = copy.deepcopy(kinematic_tree)
        for leg in legs:
            l = legs.index(leg)
            #              move(link_name, angle_name, angle_value, kinematic_tree)
            new_tree = self.move(leg,'theta', angle_values[l], new_tree)

        pose = self.pose(base_pose, new_tree, use_eff=True)

        return pose

    def pose(self, base_pose, kinematic_tree, use_eff=True):

        roll  = base_pose[0,0]
        pitch = base_pose[0,1]
        yaw   = base_pose[0,2]

        x = base_pose[1,0]
        y = base_pose[1,1]
        z = base_pose[1,2]


        # Transformação da base inercial para a base do chassi (dado pela IMU + translação)
        tw_b = np.matmul(trans(x, y, z), np.matmul(tz(yaw), np.matmul(ty(pitch), tx(roll))))
        transform_array = []

        if use_eff:
            chains = [self.iter_link(kinematic_tree)[end_link] for end_link in self.end_effectors_names]
        else:
            chains = self.iter_link(kinematic_tree).values()


        for end_link in chains:
            ta_j = [np.matmul(h_transform(end_link[0]), tw_b)]
            for i in range(1, len(end_link)):

                ta_j.append(np.matmul(ta_j[i-1], h_transform(end_link[i])))
            transform_array.append(ta_j)

        return transform_array

    def plot_pose(self, pose):
        # x = int(pose[0][0][0][3])
        # y = int(pose[0][0][1][3])
        # z = int(pose[0][0][2][3])

        contacts = self.contacts(pose)
        x_contacts = contacts[:,0]
        y_contacts = contacts[:,1]

        x_max = x_contacts[x_contacts.argmax()]
        x_min = x_contacts[x_contacts.argmin()]

        y_max = y_contacts[y_contacts.argmax()]
        y_min = y_contacts[y_contacts.argmin()]

        x_array = np.linspace(x_min, x_max, 5)
        y_array = np.linspace(y_min, y_max, 5)

        xx, yy = np.meshgrid(x_array, y_array)

        point, normal = self.find_support_plane(pose)
        d = -np.dot(point,normal)

        zz = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')

        fig.tight_layout()
        fig.subplots_adjust(left=-0.05, bottom=-0.01, right=0.95, top=1.1)

        ax.plot_surface(xx, yy, zz, color='grey', alpha=0.5)

        for ta_j in pose:
            xs = []
            ys = []
            zs = []
            for link_pos in ta_j:
                xs.append(link_pos[0,3])
                ys.append(link_pos[1,3])
                zs.append(link_pos[2,3])

            ax.plot(xs, ys, zs)

        ax.set_box_aspect((1, 1, 1))

        plt.show()

    def contacts(self, pose):
        # pose() com use_eff=True devolve exatamente uma cadeia por end
        # effector (base->roda da perna); e o unico caso suportado aqui, pois e
        # o unico em que "o ultimo elo da cadeia" e um ponto de contato bem
        # definido. use_eff=False devolve cadeias extras (ex.: a cadeia
        # inferior do 4 barras, mais curta) sem um contato equivalente, entao
        # rejeitamos explicitamente em vez de devolver dado incompleto.
        if len(pose) != len(self.end_effectors_names):
            raise ValueError(
                f"contacts: esperava {len(self.end_effectors_names)} cadeias, uma por "
                f"end effector (pose() com use_eff=True); recebeu {len(pose)} "
                "(use_eff=False nao e suportado)."
            )

        px=[]
        py=[]
        pz=[]

        for line in pose:
            matrix = line[-1]
            px.append(matrix[0,3])
            py.append(matrix[1,3])
            pz.append(matrix[2,3])
        contacts = [px,py,pz]

        points = np.transpose(contacts)

        return points

    def find_support_plane(self, pose):
        contacts = self.contacts(pose)
        coords = np.array(contacts)
        pl = []

        # barycenter of the points
        # compute centered coordinates
        g = coords.sum(axis=0) / coords.shape[0]

        # run SVD
        u, s, vh = np.linalg.svd(coords - g)

        # unitary normal vector
        normal = vh[2, :]

        if normal[2] < 0:
            normal = -1 * normal

        pl.append(g)
        pl.append(normal)

        return pl

    def distance_to_plane(self, point, plane):
        p = np.array(plane[0])
        normal = np.array(plane[1])

        n_hat = normal / np.linalg.norm(normal)

        d = np.dot(n_hat, (point-p))

        return d

    def inverse_kinematics(self, current_leg_angles, current_base_pose, desired_base_pose, kinematic_tree):

        # zera coordenadas que não influenciam na estabilidade
        desired_base_pose[0][2] = current_base_pose[0][2] # yaw
        desired_base_pose[1][0] = current_base_pose[1][0] # x
        desired_base_pose[1][1] = current_base_pose[1][1] # y

        current_pose = self.set_state(current_base_pose, current_leg_angles, kinematic_tree)
        support_plane = self.find_support_plane(current_pose)

        tolerance = self.IK_TOLERANCE

        joint_angles = []
        success = False

        angle_min = {'theta11': -self.LEG_ANGLE_LIMIT, 'theta21': -self.LEG_ANGLE_LIMIT, 'theta31': -self.LEG_ANGLE_LIMIT, 'theta41': -self.LEG_ANGLE_LIMIT}
        angle_max = {'theta11': self.LEG_ANGLE_LIMIT, 'theta21': self.LEG_ANGLE_LIMIT, 'theta31': self.LEG_ANGLE_LIMIT, 'theta41': self.LEG_ANGLE_LIMIT}
        error = {'theta11': 1, 'theta21': 1, 'theta31': 1, 'theta41': 1}
        leg_angles = {'theta11': 0, 'theta21': 0, 'theta31': 0, 'theta41': 0}
        br = 0
        hmax = self.HMAX
        hmin = self.HMIN

        dist_to_support_plane = self.distance_to_plane(desired_base_pose[1], support_plane)
        if dist_to_support_plane >= hmax:
            desired_base_pose[1][2] -= (dist_to_support_plane - hmax)
            success = False
            print('************* Set Point z limit exceeded ***************')
            print('********** z clamped to: ', desired_base_pose[1][2], '***********')
        elif dist_to_support_plane <= hmin:
            desired_base_pose[1][2] -= (dist_to_support_plane - hmin)
            success = False
            print('************* Set Point z below hmin ***************')
            print(f"************* z clamped to: {desired_base_pose[1][2]} ****************")
        while max(error.values()) >= tolerance:
            dist = []
            if error['theta11'] >= tolerance:
                leg_angles['theta11'] = (angle_min['theta11'] + angle_max['theta11'])/2 # type: ignore
            if error['theta21'] >= tolerance:
                leg_angles['theta21'] = (angle_min['theta21'] + angle_max['theta21'])/2 # type: ignore
            if error['theta31'] >= tolerance:
                leg_angles['theta31'] = (angle_min['theta31'] + angle_max['theta31'])/2 # type: ignore
            if error['theta41'] >= tolerance:
                leg_angles['theta41'] = (angle_min['theta41'] + angle_max['theta41'])/2 # type: ignore

            pose = self.set_state(desired_base_pose, leg_angles, kinematic_tree)
            contacts = self.contacts(pose)

            for l in range(4):
                contact_point = contacts[l]
                dist.append(self.distance_to_plane(contact_point, support_plane))

            dist_dic = {'theta11': dist[0], 'theta21': dist[1], 'theta31': dist[2], 'theta41': dist[3]}

            for leg in dist_dic:
                if dist_dic[leg] >= 0:
                    angle_max[leg] = leg_angles[leg]
                else:
                    angle_min[leg] = leg_angles[leg]

            error = {'theta11': abs(dist[0]), 'theta21': abs(dist[1]), 'theta31': abs(dist[2]), 'theta41': abs(dist[3])}

            if br > self.IK_MAX_ITERATIONS:
                print('break:')
                print('error:', error)
                print('base_pose', current_base_pose)
                print('-----------------------------')
                success = False
                break
            else:
                success = True
            br = br + 1

        for leg in leg_angles:
            joint_angles.append(leg_angles[leg])

        return {'success': success, 'joint_cmd_angles': joint_angles}

    def jacobian(self, pose):
        # Jacobiano geometrico de cada cadeia de pose() (ta_j = lista de
        # transformadas absolutas base->elo, uma por junta revoluta, na ordem
        # da cadeia DH -- ja monta hTransform/pose()/set_state()).
        #
        # Segue a composicao de velocidades de cadeias abertas com juntas
        # revolutas (Briot & Khalil, "Dynamics of Parallel Robots", sec.
        # 5.2.4, eqs. 5.18-5.20; ver jacobian.pdf): para juntas revolutas
        # (sigma_j=0, sigma_barra_j=1),
        #
        #     omega_j = omega_i + a_j*dq_j        (5.18)
        #     v_j     = v_i + omega_i x r_{Oi,Oj}  (5.19, sem termo de junta)
        #
        # Desenrolando essa recorrencia da base ate o efetuador (indice n), a
        # contribuicao da velocidade da junta k para v_n e a_k x (p_n - p_k)
        # (a_k continua presente em todos os transportes seguintes), e para
        # omega_n e simplesmente a_k -- logo a coluna k do jacobiano, expressa
        # no referencial da base, e:
        #
        #     J[:, k] = [ a_k x (p_n - p_k) ]
        #               [        a_k        ]
        #
        # onde a_k e o eixo z do referencial DH da junta k (3a coluna da
        # rotacao de ta_j[k], ja em coordenadas da base) e p_k sua origem.
        j_total = []
        for ta_j in pose:
            p_n = ta_j[-1][0:3, 3]
            j = []

            for link_pos in ta_j:
                a_k = link_pos[0:3, 2]
                p_k = link_pos[0:3, 3]

                jv = np.cross(a_k, (p_n - p_k))

                j.append(np.concatenate([jv, a_k]))

            j_total.append(np.array(j).T)
        return j_total

    # d(theta_k)/d(theta1) para o ramo n1->n2->n3 do paralelogramo, tirado
    # por diferenciacao direta das relacoes de fechamento ja usadas em move()
    # (linhas com theta2 = 2*pi + gamma6 - theta1 etc.): theta3 nao depende
    # de theta1 (e constante), entao sua derivada e zero.
    _LEG_COUPLING = np.array([
        [ 1., 0., 0.],  # d(theta1)/d[theta1, theta4, theta5]
        [-1., 0., 0.],  # d(theta2)/d[theta1, theta4, theta5]
        [ 0., 0., 0.],  # d(theta3)/d[theta1, theta4, theta5]
        [ 0., 1., 0.],  # d(theta4)/d[theta1, theta4, theta5]
        [ 0., 0., 1.],  # d(theta5)/d[theta1, theta4, theta5]
    ])

    def leg_jacobian(self, pose):
        # Jacobiano geometrico real de cada perna (3 colunas: theta1
        # esterçamento do paralelogramo, theta4 esterçamento do cubo da
        # roda, theta5 giro da roda), compondo o jacobiano por junta
        # (jacobian(), acima) com a derivada das relacoes de fechamento do
        # paralelogramo.
        #
        # Segue Briot & Khalil, "Dynamics of Parallel Robots", sec. 7.3.2.6
        # (eqs. 7.72-7.78; ver velocity_analysis.pdf): quando a relacao
        # geometrica entre juntas passivas e a ativa ja e conhecida em forma
        # fechada -- aqui, x = g(q_a) explicito, as mesmas formulas de
        # move() -- o jacobiano da malha fechada e obtido diferenciando g em
        # vez de recorrer a wrenches reciprocos (A_d=I por x=g(q_a) ser
        # explicito, logo J_x = -A_d^-1 B_d = -B_d = _LEG_COUPLING acima).
        #
        # O ramo n6->n7 nao entra nessa composicao: e uma cadeia redundante
        # que fisicamente coincide ponto a ponto com o ramo n1->n2->n3 (loop
        # de posicao ja fechado pelas formulas de move(), conferido
        # numericamente para varios valores de theta1), entao nao adiciona
        # nenhum grau de liberdade de posicao a mais -- so importaria para
        # um jacobiano de forcas/wrenches (analise estatica), fora do escopo
        # aqui.
        #
        # A ordem das colunas em cada ta_j e [baseLink, link_n0, n1..n5]; as
        # duas primeiras tem theta fixo (nunca variam) e ficam de fora.
        j_per_link = self.jacobian(pose)
        return [j[:, 2:7] @ self._LEG_COUPLING for j in j_per_link]
