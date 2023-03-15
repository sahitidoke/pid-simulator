import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def vehicle (v, t, u, load):
# v = vehicle velocity
# t = time
# u = gas pedal position
# load = passenger load + cargo (kg)
        cd = 0.24 #drag coefficient
        rho = 1.225 #air density
        A = 5.0 #cross-sectional area
        Fp = 30 #thrust parameter
        m = 500 # vehicle mass (kg)
        #derivative:
        dv_dt = (1.0/(m+load))* (Fp*u - 0.5*rho*cd*A*v**2)
        return dv_dt

tf = 300.0 #final time
nsteps = 301 #number of time steps
delta_t = tf/(nsteps-1) #length of each time step
ts = np.linspace(0,tf,nsteps) #linearly spaced time vector

#simulate step test operation
step = np.zeros(nsteps) #u = valve%open
load = 200.0 ##passenger (s)+ cargo load in kilograms
v0 = 0.0 #initial velocity condition
vs = np.zeros(nsteps) #ffor storing the results
ubias = 0.0
Kc = 1.0/1.2
tauI =20.0
sum_int = 0.0
es = np.zeros(nsteps)
ies = np.zeros(nsteps)
sp_store = np.zeros(nsteps)
sp = 25

for i in range (nsteps - 1):#simulate with ode int
  # schedule changes in set point
        if i==50:
                sp = 0
        if i==100:
                sp = 15
        if i==150:
                sp = 20
        if i==200:
                sp =10 
        sp_store[i+1] = sp
        error = sp-v0
        es[i+1] = error
        sum_int=  sum_int+error *delta_t
        u = ubias + Kc *error +Kc/tauI * sum_int
        if u >= 100.0: #clip inputs to -50% - 100%
                u = 100.0
                sum_int=  sum_int+error *delta_t
        if u <=-50.0:
                u = -50.0
                sum_int=  sum_int+error *delta_t
        ies[i+1] = sum_int
        step[i+1]=  u  
        v = odeint(vehicle, v0, [0,delta_t], args = (u,load))
        v0 = v[-1] #last value of v
        vs [i+1] = v0 #sotre velocity for plotting
plt.figure() #plot results
plt.subplot(2,2,1)
plt.plot (ts, vs, 'b-', linewidth=3)
plt.plot (ts,sp_store, 'k--', linewidth=2)
plt.ylabel ('Velocity (m/s)')
plt.legend (['Velocity' , 'Set Point' ], loc=2)
plt.subplot(2,2,2)
plt.plot(ts,step,'r--', linewidth=3)
plt.ylabel('Gas Pedal')
plt.legend(['Gas Pedal (%) '], loc = 'best')
plt.subplot(2,2,3)
plt.plot(ts, ies, 'b-', linewidth=3)
plt.legend(['Error (SP-PV)'])
plt.xlabel('Time (sec)')
plt.subplot(2,2,4)
plt.plot(ts,ies,'k--', linewidth = 3)
plt.legend(['Integral of Error'])
plt.xlabel('Time (sec)')
plt.show()



           

        
