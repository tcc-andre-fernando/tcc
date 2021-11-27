import time,os
from classes import PartialQLearning, Encoder

#Parâmetros
epsilon = 0.4               #Epsilon inicial: Determina probabilidade de ação randomica
lr = 0.2                    #Learning rate: Determina quão rápido os valores da QTable mudarão
gamma = 0.8                 #Balanço entre recompensas futuras e imediatas (Geralmente 0.8 a 0.99)
episodes = 400              #Número de episodios
future = 25
epsilon_rate = 0.004

#Pendulum Position
P_Positions = [0, 2, 4, 7, 10, 15, 25]
P_Speed = [-29, -22, -15, -7, -3, 0, 3, 7, 15, 22, 29]
actions = [-150, 100, -75, 0, 75, 100, 150]

#Inicializando IA
IA = PartialQLearning(lr, gamma, actions, P_Positions, P_Speed, future)

IA.set_epsilon(epsilon)

#Carregando modelo anterior
path = os.path.dirname(os.path.realpath(__file__))
IA.load_q_table('VPID_PP.npy',path)

#Inicializando Encoders
encoders = Encoder()

time.sleep(2)

try:
    for index_episode in range (episodes):

        print('\nAguardando posição de inicio ! (Ctrl + C para interromper)\n')
        while(True):

            IA.set_state(encoders.update(), True)

            if (IA.start_check()):
                break

        IA.reward = 0

        initial_time = time.time()

        t,U,posicao,velocidade,rewards = [],[],[],[],[]

        while(not IA.get_stop()):
            action = IA.action()

            states = encoders.update_go(action)
            
            IA.set_state(states, False)

            if(not IA.get_stop()):

                reward = IA.get_reward()
                #IA.update_q_table(False)

            t.append(time.time()-initial_time)
            U.append(action)
            posicao.append(states[0])
            velocidade.append(states[1])
            rewards.append(reward)

        IA.update_q_table(True)

        encoders.update()#Desliga motor
        time.sleep(0.5)

        print('Episódio atual: ', (index_episode+1))

        if epsilon > 0:
            epsilon = IA.epsilon - epsilon_rate
            IA.set_epsilon(epsilon)

        rewards = rewards[future:] + [0]*future

        IA.save_step(index_episode, t, U, posicao, velocidade, rewards)

        
except:
    time.sleep(2)
    print("Programa interrompido")
    encoders.retorno()

filename = input('Com qual nome deseja salvar ? (n para não salvar):')
if('n' != filename):
    IA.save_q_table(filename + '.npy',path)

time.sleep(1)
print('\nFim do programa !')