from classes import Encoder, PID, Log
import time, os

#Inicializando Encoders
encoders = Encoder()

#Inicializando algoritmo
# PID_Pendulum = PID(20, 2.4)
# PID_Arm = PID(1, 10)

PID_Pendulum = PID(15, 1.6)
PID_Arm = PID(1, 12)

#Inicializando Log
PID_LOG = Log()

path = os.path.dirname(os.path.realpath(__file__))

time.sleep(1)

try:
    print('Aguardando posição de inicio ! \n')
    PID.check_start(encoders)

    initial_time = time.time()

    states = encoders.update()

    print('Pressione Ctrl + C para sair do Loop \n')
    while(True):

        if(PID.check_stop(states)):
            encoders.update()
            break

        U = PID_Pendulum.step(states[0], states[1])
        U += PID_Arm.step(states[2], states[3])

        states = encoders.update_go(U)

        PID_LOG.get_step(U, states, time.time()-initial_time)

except:
    print('Programa interrompido !')
    encoders.update()

filename = input('Com qual nome deseja salvar ? (n para não salvar):')

if('n' != filename):
    PID_LOG.save_log(path, filename + '.npy')

PID_LOG.print()
