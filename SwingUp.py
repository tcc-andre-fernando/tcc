import math, time
from classes import Encoder

#Inicializando Encoders
encoders = Encoder()

time.sleep(1)

#Rotina

stop = False
encoders.update()

try:
    while(stop == False):

        U = int(250*((math.cos(math.radians(encoders.last_p))*(encoders.last_ps)>0)-(math.cos(math.radians(encoders.last_p))*encoders.last_ps<0)))
        
        encoders.update_go(U)
        
        if(abs(encoders.last_p)<10):
            stop = True
except:
    encoders.update_go(0)
encoders.update_go(0)