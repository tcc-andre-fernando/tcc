import numpy as np
import matplotlib.pyplot as plt

with open('imagens/' + 'V_PID_PP/141.npy', 'rb') as f:
    log = np.load(f, allow_pickle=True)

t = log[0]
U = log[1]
posicao = log[2]
velocidade = log[3]
posicao_b = log[4]
vel_b = log[5]
rewards = log[6]

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex = True)

ax1.set_title('Input')
ax1.set_ylabel("PWM")
ax1.grid()
ax1.plot(t, U, label='PWM')

ax2.set_title('Pêndulo')
ax2.set_ylabel("Degrees")
ax2.grid()
ax2.axhspan(-25, 25, color='red', alpha=0.2)
ax2.axhspan(-4, 4, color='blue', alpha=0.2)
ax2.plot(t, posicao, label="Posição")
ax2.plot(t, velocidade, label="Velocidade")
ax2.legend(loc="upper left")

ax3.set_title('Recompensa')
ax3.set_xlabel("Time [s]")
ax3.grid()
ax3.plot(t, rewards)

plt.show()
