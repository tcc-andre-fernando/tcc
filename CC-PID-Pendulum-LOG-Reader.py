import numpy as np
import matplotlib.pyplot as plt

with open('PID/LOGs/' + 'PID_Video_final.npy', 'rb') as f:
    log = np.load(f, allow_pickle=True)

#print(log)
U = log[0]
states = np.stack(log[1], axis=0)
timestamp = log[2]

U_pen = []
U_arm = []

for i in range(len(states)):
    U_pen.append(-15*states[i, 0] - 1.6*states[i, 1])
    U_arm.append(- 12*states[i, 3])


fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex = True)
ax1.set_title('Input')
#ax1.set_xlabel("Time [s]")
ax1.set_ylabel("PWM")
ax1.grid()
ax1.plot(timestamp, U, label='PWM')
# ax1.plot(timestamp, U_pen)
# ax1.plot(timestamp, U_arm)

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