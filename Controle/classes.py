import serial, math
from numpy.random import randint
from numpy.random import rand
import numpy as np
import matplotlib.pyplot as plt

class Encoder():
    def __init__(self):
        self.arduino = serial.Serial(
            port = 'COM5',
            baudrate= 2000000,
            timeout= 1,                      #Read_timeout(usando o readline, a leitura terminará quando o caracter /n for recebido)
            write_timeout= 0.02)

        self.last_p = 0
        self.last_ps = 0
        self.last_a = 0
        self.last_as = 0

    """Recebe valores novos dos encoders e manda zero de pwm para o motor"""
    def update(self):
        try:
            self.arduino.write('0_'.encode())
            data = self.arduino.readline()


            if data:
                data = data.decode("utf-8")[0:-2].split(";")
                self.last_p = (float(data[0]) - 600)*0.15
                self.last_ps = float(data[1])
                self.last_a = float(data[2])
                self.last_as = float(data[3])
                return [self.last_p, self.last_ps, self.last_a, self.last_as]

        except Exception as e:
            print(e)

        self.last_p = -1
        self.last_ps = -1
        self.last_a = -1
        self.last_as = -1

        return [self.last_p, self.last_ps, self.last_a, self.last_as]

    """Recebe os dados dos encoders e manda 'value' de pwm para o motor"""
    def update_go(self, value):
        try:
            #Segredo
            #############
            if(value > 10):
                value += 25
            if(value < -10):
                value -= 25
            #############

            #Manda o motor para o determinado pwm, e atualiza os valores do encoder
            if (value > 250): value = 250
            if (value < -250): value = -250

            self.arduino.write((str(value) + '_').encode())
            data = self.arduino.readline()

            if data:
                data = data.decode("utf-8")[0:-2].split(";")
                self.last_p = (float(data[0]) - 600)*0.15
                self.last_ps = float(data[1])
                self.last_a = float(data[2])
                self.last_as = float(data[3])


                return [self.last_p, self.last_ps, self.last_a, self.last_as]

        except Exception as e:
            print(e)

        self.last_p = -1
        self.last_ps = -1
        self.last_a = -1
        self.last_as = -1

        return [self.last_p, self.last_ps, self.last_a, self.last_as]

    """Retorna os estados atuais"""
    def get_states(self):
        return [self.last_p, self.last_ps, self.last_a, self.last_as]

    def retorno(self):
        stop = False
        self.update()

        while(stop == False):

            #Algoritmo do retorno
            U = math.copysign(100, self.last_a) + 10*self.last_as

            #Os valores do encoder são atualizados em .go
            self.update_go(U)

            #Condição de parada
            if(abs(self.last_a) < 5 and abs(self.last_as) < 1):
                stop = True

        self.update()
        return True

class LQR():
    def __init__(self, P_Gain, PS_Gain, A_Gain, AS_Gain, encoders):
        self.P_Gain = -P_Gain
        self.PS_Gain = -PS_Gain
        self.A_Gain = -A_Gain
        self.AS_Gain = -AS_Gain
        self.encoders = encoders
        self.last_state = []
        self.memory = []

    def check_start(self):
        while(True):
            self.last_state = self.encoders.update_go(0)

            if(abs(self.last_state[0]) < 1):
                return

    def step(self):
        while(True):

            U = self.last_state[0]*self.P_Gain
            U += self.last_state[1]*self.PS_Gain
            U += self.last_state[2]*self.A_Gain
            U += self.last_state[3]*self.AS_Gain

            memory = [self.last_state[0], self.last_state[1], self.last_state[2], self.last_state[3], U]

            if(abs(self.last_state[0]) < 20):
                self.last_state = self.encoders.update_go(U)

                memory += [self.last_state[0], self.last_state[1], self.last_state[2], self.last_state[3]]

                self.memory.append(memory)
            else:
                self.last_state = self.encoders.update_go(0)
                return

class PID():
    def __init__(self, GainP_, GainD_):
        self.GainP = -GainP_
        self.GainD = -GainD_

    def check_start(encoders):
        while(True):
            state = encoders.update_go(0)

            if(abs(state[0]) < 1):
                return

    def check_stop(state):
        if(abs(state[0]) > 20):
            return True

    def step(self, stateP, stateD):
        U = (self.GainP*stateP + self.GainD*stateD)
        return U

class PartialQLearning():
    def __init__(self, lr, gamma, action, P_Positions, P_Speeds, A_Speeds, future):

        self.lr_ = lr                  #Learning rate
        self.gamma_ = gamma             #Balanço entre recompensas futuras e imediatas

        self.action_ = action
        self.action_size_ = len(self.action_)

        self.P_positions_ = np.array(P_Positions)
        self.P_speeds_ = np.array(P_Speeds)
        self.A_speeds_ = np.array(A_Speeds)

        self.index_P = 0
        self.index_PS = 0
        self.index_AS = 0
        self.P_negative = True

        self.reward = 0
        self.episode_reward = 0
        self.max_reward = -999999

        self.stop = False

        #Números aleatórios entre 0 e 1
        #self.Q_Table = np.random.rand(len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_), self.action_size_)
        self.Q_Table = np.ones((len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_), self.action_size_))
        self.Q_Table = self.Q_Table*0

        self.actual_state = 0

        self.random_actions = randint(0, (self.action_size_-1), (len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_)))

        self.epsilon = 1
        self.random_states = rand(len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_))

        self.log = np.array([[1, 2, 3, 4]])

        self.memory = []
        self.future = future

        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, sharex = True)

    """Verifica se braço já passou dos +-180 ou se o pêndulo desequilibrou,
    finaliza o programa e atualiza os valores finais"""
    def stop_check(self):
        end = False

        #Verificar se braço já passou dos +-180
        if (abs(self.actual_state[2])> 180):
            print('Braço fora do limite !')
            
            end = True
            log = 'B'

        #Verificar se o pêndulo desequilibrou
        if (abs(self.actual_state[0])>20):
            print('Pêndulo caído !')
            
            end = True
            log = 'P'

        if end == True:

            if self.max_reward < self.episode_reward:
                self.max_reward = self.episode_reward

            print('Recompensa final do episódio: ', self.episode_reward)
            print('Recompensa máxima: ', self.max_reward)

            self.log = np.append(self.log, np.array([[self.max_reward, self.episode_reward, self.epsilon, log]]), axis = 0)

            self.episode_reward = 0

            self.stop = True
            return
        self.stop = False
        return

    """Checa se o episódio já pode começar"""
    def start_check(self):
        self.stop = False
        if (abs(self.actual_state[0])<5):

            if(self.actual_state[0] < 0):
                self.P_negative = True
                self.index_P = (np.abs(self.P_positions_ + self.actual_state[0])).argmin()
                self.index_PS = (np.abs(self.P_speeds_ + self.actual_state[1])).argmin()
                self.index_AS = (np.abs(self.A_speeds_ + self.actual_state[3])).argmin()

            else:
                self.P_negative = False
                self.index_P = (np.abs(self.P_positions_ - self.actual_state[0])).argmin()
                self.index_PS = (np.abs(self.P_speeds_ - self.actual_state[1])).argmin()
                self.index_AS = (np.abs(self.A_speeds_ - self.actual_state[3])).argmin()

            self.random_actions = randint(0, (self.action_size_-1), (len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_)))
            self.random_states = rand(len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_))

            return True
        return False

    def action(self):
        if (self.random_states[self.index_P, self.index_PS, self.index_AS] < self.epsilon):
            index_action = self.random_actions[self.index_P, self.index_PS, self.index_AS]
            # print('Ação: ', index_action, ' (Random)')

            self.memory.append([self.index_P, self.index_PS, self.index_AS, self.P_negative, index_action])
            
            if(self.P_negative):
                return -self.action_[index_action]
            return self.action_[index_action]

        index_action = np.argmax(self.Q_Table[self.index_P, self.index_PS, self.index_AS])
        # print('Ação: ', index_action)

        self.memory.append([self.index_P, self.index_PS, self.index_AS, self.P_negative, index_action])
        if(self.P_negative):
            return -self.action_[index_action]
        return self.action_[index_action]

    def load_q_table(self, file_name, path):
        with open(path + '/QTables/' + file_name, 'rb') as f:
            self.Q_Table = np.load(f)

        with open(path + '/Logs/' + file_name, 'rb') as f:
            self.log = np.load(f)

        #self.epsilon = self.log[-1, 2].astype(float)

    def save_q_table(self, file_name, path):
        with open(path + '/QTables/' + file_name, 'wb') as f:
            np.save(f, self.Q_Table)

        with open(path + '/Logs/' + file_name, 'wb') as f:
            np.save(f, self.log)

    def update_q_table(self, done):
        if ((not done)):
            if(self.reward != 0):
                self.Q_Table[self.memory[-self.future][0], self.memory[-self.future][1] , self.memory[-self.future][2], self.memory[-self.future][4]] +=  self.lr_*(self.reward + self.gamma_*np.amax(self.Q_Table[self.memory[-self.future+1][0], self.memory[-self.future+1][1], self.memory[-self.future+1][2]]) - self.Q_Table[self.memory[-self.future][0], self.memory[-self.future][1], self.memory[-self.future][2], self.memory[-self.future][4]])            

        else:
            print(self.log[-1][-1])

            if(len(self.memory) > self.future):
                if(self.log[-1][-1] == "P"):
                    for i in range(self.future):
                        self.Q_Table[self.memory[-i-1][0], self.memory[-i-1][1], self.memory[-i-1][2], self.memory[-i-1][4]] += self.lr_*(-1 + self.gamma_*np.amax(self.Q_Table[self.memory[-i][0], self.memory[-i][1], self.memory[-i][2]]) - self.Q_Table[self.memory[-i-1][0], self.memory[-i-1][1], self.memory[-i-1][2], self.memory[-i-1][4]])            
                
            else:
                if(self.log[-1][-1] == "P"):
                    for i in range(len(self.memory)):
                        self.Q_Table[self.memory[-i-1][0], self.memory[-i-1][1], self.memory[-i-1][2], self.memory[-i-1][4]] += self.lr_*(-1 + self.gamma_*np.amax(self.Q_Table[self.memory[-i][0], self.memory[-i][1], self.memory[-i][2]]) - self.Q_Table[self.memory[-i-1][0], self.memory[-i-1][1], self.memory[-i-1][2], self.memory[-i-1][4]])   
                
            self.memory = []
            self.random_actions = randint(0, (self.action_size_-1), (len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_)))
            self.random_states = rand(len(self.P_positions_), len(self.P_speeds_), len(self.A_speeds_))

    """Seta os actual state e os last_index (Para não começar o episódio com last_index do episódio anterior)"""
    def set_state(self, state, first):

        self.actual_state = state

        if not first:

            self.stop_check()

            if(self.get_stop()):
                return

            if(self.actual_state[0] < 0):
                self.P_negative = True
                self.index_P = (np.abs(self.P_positions_ + self.actual_state[0])).argmin()
                self.index_PS = (np.abs(self.P_speeds_ + self.actual_state[1])).argmin()
                self.index_AS = (np.abs(self.A_speeds_ + self.actual_state[3])).argmin()
                
            else:
                self.P_negative = False
                self.index_P = (np.abs(self.P_positions_ - self.actual_state[0])).argmin()
                self.index_PS = (np.abs(self.P_speeds_ - self.actual_state[1])).argmin()
                self.index_AS = (np.abs(self.A_speeds_ - self.actual_state[3])).argmin()

    def get_stop(self):
        return self.stop

    def get_reward(self):

        self.episode_reward = len(self.memory)
        if(len(self.memory) > self.future):
            if (abs(self.actual_state[0]) < 4):
                self.reward = 0.7 - abs(self.actual_state[0])*0.125 #Se no futuro estiver em determinada região
                if(abs(self.actual_state[1]) > 25):
                    self.reward -= -0.8 + abs(self.actual_state[1]*0.04) #Se no futuro tiver uma velocidade muito alta

            elif((abs(self.actual_state[0]) - abs(self.memory[-self.future][0]) < 0)):
                self.reward = 0.2 #Se no futuro estiver se aproximando da região
                if(abs(self.actual_state[1]) > 25):
                    self.reward -= -0.8 + abs(self.actual_state[1]*0.04) #Se no futuro tiver uma velocidade muito alta

            else:
                self.reward = 1 - abs(self.actual_state[0])*0.25
            
            return self.reward
        else:
            return 0 #Se ainda não tiver memória o suficiente

        # print (self.reward)
        #print('Episode Reward: ',self.episode_reward)

    def get_reward_simple(self):
        
        self.episode_reward = len(self.memory)

        if(len(self.memory) > self.future):
            if (abs(self.actual_state[0]) < 4):
                self.reward = 0.5 #Se no futuro estiver em determinada região
                if(abs(self.actual_state[1]) > 25):
                    self.reward = 0.2 #Se no futuro tiver uma velocidade muito alta

            elif((abs(self.actual_state[0]) - abs(self.memory[-self.future][0]) < 0)):
                self.reward = 0.3 #Se no futuro estiver se aproximando da região
                if(abs(self.actual_state[1]) > 25):
                    self.reward = 0.2 #Se no futuro tiver uma velocidade muito alta

            else:
                self.reward = -0.5
            
            return self.reward
        else:
            return 0 #Se ainda não tiver memória o suficiente

        # print (self.reward)
        #print('Episode Reward: ',self.episode_reward)

    def get_reward_bin(self):
        self.episode_reward = len(self.memory)

        if(len(self.memory) > self.future):
            self.reward = 1 #Se no futuro estiver em determinada região
            
            return self.reward
        else:
            return 0 #Se ainda não tiver memória o suficiente

        # print (self.reward)
        #print('Episode Reward: ',self.episode_reward)

    def get_reward_new(self):

        self.episode_reward = len(self.memory)
        if(len(self.memory) > self.future):
            if (abs(self.actual_state[0]) < 6):
                self.reward = 1 - abs(self.actual_state[0])*0.125 #Se no futuro estiver em determinada região
                if(abs(self.actual_state[1]) > 15):
                    self.reward -= -0.6 + abs(self.actual_state[1]*0.04) #Se no futuro tiver uma velocidade muito alta

            elif((abs(self.actual_state[0]) - abs(self.memory[-self.future][0]) < 0)):
                self.reward = 0.15 #Se no futuro estiver se aproximando da região
                if(abs(self.actual_state[1]) > 15):
                    self.reward -= -0.6 + abs(self.actual_state[1]*0.04) #Se no futuro tiver uma velocidade muito alta

            else:
                self.reward = 1 - abs(self.actual_state[0])*0.25
            
            return self.reward
        else:
            return 0 #Se ainda não tiver memória o suficiente

    def set_epsilon(self, epsilon):
        self.epsilon = epsilon

    def save_step(self, index_episode, t, U, posicao, velocidade, posicao_b, vel_b, rewards, path):
        
        self.ax1.set_title('Input')
        self.ax1.set_ylabel("PWM")
        self.ax1.grid()
        self.ax1.plot(t, U, label='PWM')

        self.ax2.set_title('Pêndulo')
        self.ax2.set_ylabel("Degrees")
        self.ax2.grid()
        self.ax2.axhspan(-25, 25, color='red', alpha=0.2)
        self.ax2.axhspan(-4, 4, color='blue', alpha=0.2)
        self.ax2.plot(t, posicao, label="Posição")
        self.ax2.plot(t, velocidade, label="Velocidade")
        self.ax2.legend(loc="upper left")
        
        self.ax3.set_title('Recompensa')
        self.ax3.set_xlabel("Time [s]")
        self.ax3.grid()
        self.ax3.plot(t, rewards)

        self.fig.savefig('imagens/{}.png'.format(index_episode))

        self.ax1.cla()
        self.ax2.cla()
        self.ax3.cla()

        with open(path + '/imagens/' + str(index_episode) + '.npy', 'wb') as f:
            np.save(f, np.array([t, U, posicao, velocidade, posicao_b, vel_b, rewards]))

class Log():
    def __init__(self):
        self.U = []
        self.states = []
        self.timestamp = []

    def get_step(self, U, state, timestamp):
        self.U.append(U)
        self.states.append(state)
        self.timestamp.append(timestamp)

    def save_log(self, path, file_name):
        with open(path + '/PID/LOGs/' + file_name, 'wb') as f:
            np.save(f, np.array([self.U, self.states, self.timestamp]))

    def print(self):
        U = np.array(self.U)
        states = np.array(self.states)
        timestamp = np.array(self.timestamp)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex = True)
        ax1.set_title('Input')
        #ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("PWM")
        ax1.grid()
        ax1.plot(timestamp, U, label='PWM')

        ax2.set_title('Pêndulo')
        #ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Degrees")
        ax2.grid()
        ax2.plot(timestamp, states[:,0], label='Degrees')
        #ax2.plot(timestamp, states[:,1], label='Graus/s')
        
        ax3.set_title('Braço')
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Degrees")
        ax3.grid()
        ax3.plot(timestamp, states[:,2], label='Degrees')
        #ax3.plot(timestamp, states[:,3], label='Graus/s')

        #plt.legend()
        plt.show()

