import numpy as np
import matplotlib.pyplot as plt

with open('Logs/' + 'V_PID_PP.npy', 'rb') as f:
    log = np.load(f)

max_reward = log[1:,0].astype(float)
ep_reward = log[1:,1].astype(float)
epsilon = log[1:,2].astype(float)
end = np.array(log[1:,3])

ep_reward_mean = []

for i in range(len(ep_reward)-5):
    ep_reward_mean.append((ep_reward[i] + ep_reward[i+1] +ep_reward[i+2] +ep_reward[i+3] +ep_reward[i+4])/5)

ep_reward_mean = 5*[0] + ep_reward_mean

# Create two subplots and unpack the output array immediately
f, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)

ax1.set_title('Desempenho [max]')
ax1.grid()
ax1.plot(max_reward)

ax2.set_title('Desempenho')
ax2.grid()
ax2.plot(ep_reward)
ax2.plot(ep_reward_mean)


ax3.set_title('Epsilon')
ax3.grid()
ax3.plot(epsilon)

ax4.set_title('Final')
ax4.plot(end)

plt.show()