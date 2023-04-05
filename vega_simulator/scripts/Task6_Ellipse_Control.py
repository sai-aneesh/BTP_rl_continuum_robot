#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
import time
from math import ceil, pi
from std_msgs.msg import Float32MultiArray, Float32, Bool
from time import sleep
import numpy as np
import os
import gym
from gym import spaces
from time import sleep
import stable_baselines3
from stable_baselines3.common.callbacks import CallbackList,CheckpointCallback,EvalCallback,StopTrainingOnRewardThreshold,ProgressBarCallback,StopTrainingOnNoModelImprovement
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, A2C, SAC# DQN coming soon
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.sac.policies import MlpPolicy

class vega_env(gym.Env):

    def __init__(self, n_sections):
        super(vega_env, self).__init__()
        
        self.n_actions=n_sections

        #creating all publishers
        self.force_pub=rospy.Publisher("/steady_system/force",Float32MultiArray,queue_size=1)
        self.episode_reset_pub = rospy.Publisher("/actual_system/reset",Bool,queue_size=1)
        self.target_pub = rospy.Publisher("/actual_system/target",Float32MultiArray,queue_size=1)
        self.radius_pub = rospy.Publisher("/actual_system/radius",Float32MultiArray,queue_size=1)

        #creating all subscibers
        # rospy.Subscriber("/steady_system/state",Float32MultiArray,self.state_callback,queue_size=1)
        rospy.Subscriber("/steady_system/complete_pose",Float32MultiArray,self.complete_pose_callback,queue_size=1)
        rospy.Subscriber("/steady_system/velocity",Float32MultiArray,self.velocity_callback,queue_size=1)


        #defining the messages and state varaibles
        self.forces_msg=Float32MultiArray()
        self.target_msg = Float32MultiArray()
        self.reset_msg = Bool()
        self.radius_msg = Float32MultiArray()
        self.measured_state = np.zeros(shape=(18,))
        self.velocity = np.array([0,0])
        self.theta = 0.02
        self.clck = False

        self.n = 0
        self.n_limit = 15
        self.radius = 0.25
        self.forces_msg.data=np.zeros(shape=(2,))
        self.eccen = 0.0
        self.tar_vel = 0.5

        self.steps = 0
        self.reward_mean = 0
        self.dist_mean = 0
        self.vel_mean = 0
        self.eng_mean = 0


        self.t_x =  self.radius*np.sin(self.theta)
        self.t_y =  self.radius*(1-np.cos(self.theta))
        self.target = np.array([self.t_x,self.t_y,self.radius*self.tar_vel*np.cos(self.theta),self.radius*np.sqrt(1- self.eccen**2)*self.tar_vel*np.sin(self.theta)]).astype(np.float32)

        # Define action and observation space
        self.observation_space = spaces.Box(low=-1, high=1, shape = (18,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1, high=1, shape = (5,), dtype=np.float32)
        # They must be gym.spaces objects

        #starting the controller by starting with zero input for first timestep so that step is obtained for next step
        sleep(2)
        self.forces_msg.data=np.zeros(shape=(5,))
        # self.force_pub.publish(self.forces_msg)

    def complete_pose_callback(self,msg):

        # rospy.loginfo("A3. received new state from steady state checker!")
        #get current state
        pose=np.array(msg.data).reshape((12,))
        self.measured_state = np.concatenate((pose,self.velocity,self.target))
        self.check_state = True

        #Publish the Target or Goal after we get the steady 
        self.target_msg.data = self.target
        self.target_pub.publish(self.target_msg)


    def velocity_callback(self,msg):

        #get the velocity
        self.velocity=np.array(msg.data).reshape((2,))


    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        # Initialize the agent at position to default
        self.reset_msg.data = True
        self.episode_reset_pub.publish(self.reset_msg)
        # rospy.loginfo("A0. Resetting the environment")

        self.target = np.array([self.t_x,self.t_y,self.radius*self.tar_vel*np.cos(self.theta),self.radius*np.sqrt(1- self.eccen**2)*self.tar_vel*np.sin(self.theta)]).astype(np.float32)
        # observation should match observation space
        return np.array(self.measured_state).astype(np.float32)

    def step(self, action):
                                #### what does this step mean?
                                ### basically, we want to know what action to take  
                                ### to minimize distance between tip and target. 
                                # We start by taking random actions and observing reward
        start = time.time()
        self.check_state = False
        action =  np.zeros(shape=(5,)) + action*3              ### to rescale the action space from -1 to 1 ---> -3 to 3
        torques = np.zeros(shape=(5,))
        torques[4] = action[4]
        torques[3] = action[3]-action[4]
        torques[2] = action[2]-action[3]
        torques[1] = action[1]-action[2]
        torques[0] = action[0]-action[1]

        self.forces_msg.data= torques
        self.force_pub.publish(self.forces_msg)
        # rospy.loginfo("A1. Publishing a force to the steady state checker!!")

        while not (self.check_state):
            # rospy.loginfo("A2. Steady State not achieved yet")
            continue

        end = time.time() 
        dt = end-start

        # print("time elapsed:",dt)

        if( self.n> self.n_limit):            
            self.radius = np.random.uniform(0.12,0.25)
            self.tar_vel = np.random.uniform(0.4,1.0)
            self.eccen = np.random.uniform(0.25,0.8)
            self.n = 0
            self.n_limit = np.random.uniform(15,25)
            self.radius_msg.data = np.array([self.radius,self.eccen]).astype(np.float32)
            self.radius_pub.publish(self.radius_msg)
        self.n += 1

        if(self.n == int(self.n_limit/2)):
            self.clck = bool(random.choice([True, False]))

        self.theta += self.tar_vel*dt

        # if(self.theta <= 1.38) and (self.clck == False):
        #     self.theta += self.tar_vel*dt
        # elif(self.theta > 1.38) and (self.clck == False):
        #     self.theta -= self.tar_vel*dt
        #     self.clck = True
        # elif(self.theta >= -1.38) and (self.clck == True):
        #     self.theta -= self.tar_vel*dt
        # elif (self.theta < -1.38) and (self.clck == True):
        #     self.theta += self.tar_vel*dt
        #     self.clck = False

        self.t_x = self.radius*np.sin(self.theta)
        self.t_y = self.radius*np.sqrt(1- self.eccen**2)*(1-np.cos(self.theta))

        self.target = np.array([self.t_x,self.t_y,self.radius*self.tar_vel*np.cos(self.theta),self.radius*np.sqrt(1- self.eccen**2)*self.tar_vel*np.sin(self.theta)]).astype(np.float32)

        # rospy.loginfo("A4. Steady state achieved!!")

        x = self.measured_state[10]
        y = self.measured_state[11]
        
        # rospy.loginfo("A5. Calculating the error between the steady state tip pose and target pose")

        distance = np.sqrt(((x - self.target[0])**2+ (y - self.target[1])**2))
        vel_diff = np.sqrt(((self.velocity[0] - self.radius*self.tar_vel*np.cos(self.theta))**2+ (self.velocity[1] - self.radius*np.sqrt(1- self.eccen**2)*self.tar_vel*np.sin(self.theta))**2))

        #check if we reached the goal
        done = bool((distance < 0.01) and (vel_diff < 0.04))

        #CODE FOR ENERGY MINIMIZATION REWARD
        # returns square of distance b/w two points 
        def length(X, Y): 
            xDiff = X[0] - Y[0] 
            yDiff = X[1] - Y[1] 
            return np.sqrt(xDiff * xDiff + yDiff * yDiff)

        def Angle(A, B, C): 
            
            # Square of lengths be a2, b2, c2 
            a = length(B, C) 
            b = length(A, C) 
            c = length(A, B) 

            # From Cosine law 
            alpha = np.arccos((b**2 + c**2 - a**2) /(2 * b * c)); 
            return alpha


        angle = np.array([])
        for i in range(0,10,2):
            A  = self.measured_state[0:2]
            B  = [0 , 0.75 * (1-(int(i/2)+1)/5)]
            C  = self.measured_state[i+2:i+4] 
            angle = np.append(angle,Angle(A,B,C))
        

        # Reward
        reward = - 40*distance  -10*(vel_diff)- np.absolute((np.dot(action,angle)))

        energy_penalty = np.absolute((np.dot(action,angle)))
        print("R1 : ", 40*distance)
        print("R2 :      ", 10*vel_diff)
        print("R3 :            ", energy_penalty)
       

        self.steps += 1
        self.reward_mean = (self.reward_mean*(self.steps -1) + reward)/self.steps
        self.dist_mean = (self.dist_mean*(self.steps -1) + distance)/self.steps
        self.vel_mean = (self.vel_mean*(self.steps -1) + vel_diff)/self.steps
        self.eng_mean = (self.eng_mean*(self.steps -1) + energy_penalty)/self.steps

        print('MEAN distance error : ', self.dist_mean, '..................... MEAN velocity error:',self.vel_mean,'.....................MEAN Energy term:',self.eng_mean)
        print('STEPS:', self.steps,'MEAN reward:', self.reward_mean )

        print('self.target_velocity: ',self.radius*self.tar_vel , ".........TIME ELAPSED",dt )
        print('velocity_x :',self.velocity[0], '....target x_velocity:',self.radius*self.tar_vel*np.cos(self.theta))
        print('velocity_y :',self.velocity[1], '....target y_velocity:',self.radius*np.sqrt(1- self.eccen**2)*self.tar_vel*np.sin(self.theta))
      
        info = {}
        return np.array(self.measured_state).astype(np.float32), reward, done, info


    def close(self):
        pass

## IMPORTANT : THIS CODE ONLY WORKS FOR L=0.75, because that I assumed that in the c++ code of vega

if __name__ == '__main__':

    rospy.init_node('Controller_node', anonymous = True)

    n_sections = int(sys.argv[1])
    # float(sys.argv[2]),int(sys.argv[3])

    env = vega_env(n_sections)
    # obs = env.reset()
    # print(obs)
    # # If the environment don't follow the interface, an error will be thrown
    # check_env(env, warn=True)
    # # wrap it
    # env = make_vec_env(lambda: env, n_envs=1)
    # # Train the agent

    # #if no pretrained model
    # # model = SAC('MlpPolicy', env, verbose=1, learning_starts = 1000, batch_size=256, gamma = 0.99, use_sde = True, tensorboard_log ="/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/results")
    
    # #continue training from a saved checkpoint
    # model = SAC.load("/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/config/Mar31_Task5_Energy_Min_45000_steps", tensorboard_log="/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/results")
    # model.set_env(env)

    # # Save a checkpoint every 5000 steps
    # log_dir = os.getcwd()
    # # print(log_dir)
    # checkpoint_callback = CheckpointCallback(save_freq=5000, save_path=log_dir +'/logs/',name_prefix='Mar31_Task5_Energy_Min')

    # # Separate evaluation env
    # # eval_env = vega_env(n_sections)
    # # eval_env = make_vec_env(lambda: eval_env, n_envs=1)
    # # # Stop training when the model reaches the reward threshold
    # # callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=-12, verbose=1)

    # # eval_callback = EvalCallback(eval_env, callback_on_new_best=callback_on_best, verbose=1)
    # # Almost infinite number of timesteps, but the training will stop
    # # early as soon as the reward threshold is reached

    # # Stop training if there is no improvement after more than 3 evaluations
    # # stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improvement_evals=3, min_evals=5, verbose=1)
    # # eval_callback = EvalCallback(eval_env, n_eval_episodes= 40, callback_after_eval=stop_train_callback, verbose=1)

    # callback = CallbackList([ProgressBarCallback(),checkpoint_callback ])

    # model.learn(total_timesteps=int(1e7), tb_log_name = "Mar31_Task5_Energy_Min", callback=callback)

    # # # save the model
    # model.save(log_dir + "/Mar31_Task5_Energy_Min")

    # # load the trained model
    model = SAC.load("/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/config/V3_Mar31_Task5_Energy_Min_235000_steps")    
    # # Save the policy independently from the model
    # # Note: if you don't save the complete model with `model.save()`
    # # you cannot continue training afterward
    # policy = model.policy
    # policy.save("Mar31_Task5_Energy_Min_Policy")
    ######### independent attempt to test########
    # start a new episode
    reward_mean = 0
    obs = env.reset()
    n_steps = 20000
    step = 0
    r=rospy.Rate(1)
    while (not rospy.is_shutdown()) and (step < n_steps):   
        action, _ = model.predict(obs, deterministic=True)
        # action =  np.zeros(shape=(5,))
        # print("Step {}".format(step + 1))
        # print("Action: ", action)
        obs, reward, done, info = env.step(action)
        # print('obs=', obs, 'reward=', reward, 'done=', done)
        step += 1
        reward_mean = (reward_mean*(step -1) + reward)/step
        # print('OUTPUT:',reward_mean, reward, step)

        #env.render(mode='console')
        if done:
            # Note that the VecEnv resets automatically
            # when a done signal is encountered
            print("Goal reached!", "reward=", reward)
            # break  
            continue
        r.sleep()