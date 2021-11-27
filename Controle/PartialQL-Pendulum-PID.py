import time,os
from classes import PartialQLearning, Encoder, PID

PID_Arm = PID(1, 12)
PID_Pen = PID(0, 1.6)

#Parâmetros
epsilon = 0                 #Epsilon inicial: Determina probabilidade de ação randomica
lr = 0.2                    #Learning rate: Determina quão rápido os valores da QTable mudarão
gamma = 0                   #Balanço entre recompensas futuras e imediatas (Geralmente 0.8 a 0.99)
episodes = 300              #Número de episodios
future = 25
epsilon_rate = 0.002

#Pendulum Position
# P_Positions = [0, 2, 5, 9, 14]
# P_Speed = [32, -25, -15, -7, 0, 7, 15, 25, 32]
# A_Speed = [-30, -10, 10, 30]
# actions = [-150, -100, -50, 0, 50, 100, 150]

P_Positions = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18]
P_Speed = [0]
A_Speed = [0]
actions = [-150, -125, -100, -75, -50, -25, 0, 25, 50, 75, 100, 125, 150]

#Inicializando IA
IA = PartialQLearning(lr, gamma, actions, P_Positions, P_Speed, A_Speed, future)

IA.set_epsilon(epsilon)

#Carregando modelo anterior
path = os.path.dirname(os.path.realpath(__file__))
IA.load_q_table('V_PID_PP.npy',path)

#Inicializando Encoders
encoders = Encoder()

time.sleep(2)

try:
    for index_episode in range (episodes):

        print('\nAguardando posição de inicio ! (Ctrl + C para interromper)\n')
        while(True):

            states = encoders.update()
            IA.set_state(states, True)

            if (IA.start_check()):
                break

        IA.reward = 0

        initial_time = time.time()

        t,U,posicao,velocidade, posicao_b, vel_b,rewards = [],[],[],[],[], [], []

        while(not IA.get_stop()):
            action = IA.action()
            action_PID = PID_Arm.step(states[2], states[3])
            action_PID += PID_Pen.step(states[0], states[1])

            states = encoders.update_go(action + action_PID)
            #states = encoders.update_go(action)
            
            IA.set_state(states, False)

            if(not IA.get_stop()):

                reward = IA.get_reward_new()
                IA.update_q_table(False)

            t.append(time.time()-initial_time)
            U.append(action + action_PID)
            posicao.append(states[0])
            velocidade.append(states[1])
            posicao_b.append(states[2])
            vel_b.append(states[3])
            rewards.append(reward)

        IA.update_q_table(True)

        encoders.update()#Desliga motor
        time.sleep(0.5)

        print('Episódio atual: ', (index_episode+1))

        if epsilon > 0:
            epsilon = IA.epsilon - epsilon_rate
            IA.set_epsilon(epsilon)

        if(len(rewards) > future):
            rewards = rewards[future:] + [0]*future

        IA.save_step(index_episode, t, U, posicao, velocidade, posicao_b, vel_b, rewards, path)

except:
    time.sleep(2)
    print("Programa interrompido")
    encoders.retorno()

filename = input('Com qual nome deseja salvar ? (n para não salvar):')
if('n' != filename):
    IA.save_q_table(filename + '.npy',path)

time.sleep(1)
print('\nFim do programa !')