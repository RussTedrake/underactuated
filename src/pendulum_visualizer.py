import numpy as np
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt

class PendulumVisualizer:
    a1 = 0.75;  
    ac1 = 0.75;
    av = np.linspace(0,math.pi,20);
    rb = .03; 
    hb=.07;
    aw = .01;
    base_x = rb*np.concatenate(([1], np.cos(av), [-1]));
    base_y = np.concatenate(([-hb], rb*np.sin(av), [-hb]));
    arm_x = np.concatenate((aw*np.sin(av-math.pi/2), aw*np.sin(av+math.pi/2)));
    arm_y = np.concatenate((aw*np.cos(av-math.pi/2), -a1+aw*np.cos(av+math.pi/2)));
    fig = plt.figure(25);
    ax = plt.axes();
    ax.axis('equal');
    ax.axis('off');
    ax.set_xlim([-1.2,1.2]);
    ax.set_ylim([-1.2,1.2]);
    base = plt.fill(base_x,base_y,zorder=1,color=(.3, .6, .4),edgecolor='k');
    # arm_x and arm_y are closed (last element == first element), but don't pass the 
    # last element to the fill command, because it gets closed anyhow (and we want the
    # sizes to match for the update).
    arm = plt.fill(arm_x[0:-1],arm_y[0:-1],zorder=0,color=(.9, .1, 0),edgecolor='k');
    center_of_mass = plt.plot(0,-ac1,zorder=1,color='b',marker='o',markersize=14);
    
    def draw(self, theta):
        path = self.arm[0].get_path();
        path.vertices[:,0] = self.arm_x*math.cos(theta)-self.arm_y*math.sin(theta);
        path.vertices[:,1] = self.arm_x*math.sin(theta)+self.arm_y*math.cos(theta);
        self.center_of_mass[0].set_data(self.ac1*math.sin(theta),-self.ac1*math.cos(theta));
        print theta
        
    def animation_update(self, i):
        self.draw(math.sin(0.03*i));

    def animate(self, log):
        # log - a reference to a pydrake.systems.primitives.SignalLogger that
        # constains the pendulum state, or pendulum output (the first element
        # should be the pendulum's theta) after running a simulation.
        t = log.sample_times();
        x = log.data();
        ani = animation.FuncAnimation(self.fig, self.animation_update, 500, interval=25, repeat=False);
        return ani



