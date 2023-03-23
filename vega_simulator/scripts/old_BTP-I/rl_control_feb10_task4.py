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
from stable_baselines3.common.callbacks import CallbackList,CheckpointCallback,EvalCallback,StopTrainingOnRewardThreshold,ProgressBarCallback
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

        #creating all subscibers
        rospy.Subscriber("/steady_system/state",Float32MultiArray,self.state_callback,queue_size=1)
        rospy.Subscriber("/actual_system/velocity",Float32MultiArray,self.velocity_callback,queue_size=1)


        #defining the messages and state varaibles
        self.forces_msg=Float32MultiArray()
        self.target_msg = Float32MultiArray()
        self.reset_msg = Bool()
        self.measured_state = np.array([0,0,0,0])
        self.velocity = np.array([0,0])
        self.theta = 0.02
        self.clck = False

        self.n = 0
        self.n_limit = 15
        self.radius = 0.5

        self.t_x =  self.radius*np.sin(self.theta)
        self.t_y =  self.radius*(1-np.cos(self.theta))
        self.target = np.array([self.t_x,self.t_y]).astype(np.float32)

        # Define action and observation space
        self.observation_space = spaces.Box(low=-1, high=1, shape = (4,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1, high=1, shape = (5,), dtype=np.float32)
        # They must be gym.spaces objects

        #starting the controller by starting with zero input for first timestep so that step is obtained for next step
        sleep(2)
        self.forces_msg.data=np.zeros(shape=(5,))
        # self.force_pub.publish(self.forces_msg)

    def state_callback(self,msg):

        #get current state
        pose=np.array(msg.data).reshape((2,))
        self.measured_state = np.concatenate((self.target, pose))
        self.check_state = True

        #Publish the Target or Goal after we get the steady 
        self.target_msg.data = self.target
        self.target_pub.publish(self.target_msg)
        # rospy.loginfo("received new state from steaady state checker")
        # rospy.loginfo(self.measured_state)

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
        rospy.loginfo("resetting the environment")

        self.target = np.array([self.t_x,self.t_y]).astype(np.float32)
        # observation should match observation space
        return np.array(self.measured_state).astype(np.float32)

    def step(self, action):

        start = time.time()
        self.check_state = False
        action =  np.zeros(shape=(5,)) + action*3
        torques = np.zeros(shape=(5,))
        torques[4] = action[4]
        torques[3] = action[3]-action[4]
        torques[2] = action[2]-action[3]
        torques[1] = action[1]-action[2]
        torques[0] = action[0]-action[1]

        self.forces_msg.data= torques
        self.force_pub.publish(self.forces_msg)

        while not (self.check_state):
            continue

        x = self.measured_state[2]
        y = self.measured_state[3]
        end = time.time() 

        tar_vel = 0.5
        dt = end-start

        # print("time elapsed:",dt)

        if( self.n> self.n_limit):
            self.radius = np.random.uniform(0.2,0.6)
            self.n = 0
            self.n_limit = np.random.uniform(10,20)
        self.n += 1

        # self.theta = np.pi * np.random.uniform(0, 2)

        if(self.theta <= 1.38) and (self.clck == False):
            self.theta += tar_vel*dt
        elif(self.theta > 1.38) and (self.clck == False):
            self.theta -= tar_vel*dt
            self.clck = True
        elif(self.theta >= -1.38) and (self.clck == True):
            self.theta -= tar_vel*dt
        elif (self.theta < -1.38) and (self.clck == True):
            self.theta += tar_vel*dt
            self.clck = False

        self.t_x = self.radius*np.sin(self.theta)
        self.t_y = self.radius*(1-np.cos(self.theta))

        #Idea for target point moving in a box enclosed workspace
        # tar_vx = tar_vel*np.cos(self.theta)
        # tar_vy = tar_vel*np.sin(self.theta)

        # if(((self.t_y + tar_vy *dt)> 0.5) or ((self.t_y + tar_vy *dt) < 0.0)):
        #     self.dir_y *=  -1
        # if(((self.t_x + tar_vx * dt)> 0.3) or ((self.t_x + tar_vx * dt)< -0.3)):
        #     self.dir_x *= -1

        # self.t_x += tar_vx * dt *self.dir_x
        # self.t_y += tar_vy *dt* self.dir_y

        self.target = np.array([self.t_x,self.t_y]).astype(np.float32)
        
        distance = np.sqrt(((x - self.target[0])**2+ (y - self.target[1])**2))
        vel_diff = np.sqrt(((self.velocity[0] - self.radius*tar_vel*np.cos(self.theta))**2+ (self.velocity[1] - self.radius*tar_vel*np.sin(self.theta))**2))

        # print("distance:", distance)
        # print("velocity:",vel_diff)
        # print("robot vel:",self.velocity)

        #check if we reached the goal
        done = bool(distance < 0.01)

        # Null reward everywhere except when reaching the goal (left of the grid)
        reward = - 20*distance  -(vel_diff)*10

        info = {}
        return np.array(self.measured_state).astype(np.float32), reward, done, info


    def close(self):
        pass

if __name__ == '__main__':

    rospy.init_node('Controller_node', anonymous = True)

    n_sections = int(sys.argv[1])
    # float(sys.argv[2]),int(sys.argv[3])

    env = vega_env(n_sections)
    # obs = env.reset()
    # print(obs)
    # If the environment don't follow the interface, an error will be thrown
    # check_env(env, warn=True)
    # wrap it
    # env = make_vec_env(lambda: env, n_envs=1)
    # # Train the agent

    # #if no pretrained model
    # # model = SAC('MlpPolicy', env, verbose=1, learning_starts = 10, batch_size=64, gamma = 0.9, use_sde = True, tensorboard_log ="/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/results")
    
    # #continue training from a saved checkpoint
    # model = SAC.load("/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/config/logs/Feb10_Extend_moving_goal_20000_steps", tensorboard_log="/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/results")
    # model.set_env(env)

    # # Save a checkpoint every 5000 steps
    # log_dir = os.getcwd()
    # # print(log_dir)
    # checkpoint_callback = CheckpointCallback(save_freq=5000, save_path=log_dir +'/logs/',name_prefix='Feb10_Task4')

    # # Separate evaluation env
    # eval_env = vega_env(n_sections)
    # eval_env = make_vec_env(lambda: eval_env, n_envs=1)
    # # Stop training when the model reaches the reward threshold
    # callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=-2, verbose=1)
  
    # eval_callback = EvalCallback(eval_env, callback_on_new_best=callback_on_best, verbose=1)
    # # Almost infinite number of timesteps, but the training will stop
    # # early as soon as the reward threshold is reached

    # callback = CallbackList([ProgressBarCallback(),checkpoint_callback,eval_callback ])

    # model.learn(total_timesteps=int(2e5), tb_log_name = "Feb10_Task4", callback=callback)

    # # # save the model
    # model.save(log_dir + "/Feb10_Task4")

    # # load the trained model
    model = SAC.load("/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/config/Feb10_Task4")    
    # # Save the policy independently from the model
    # # Note: if you don't save the complete model with `model.save()`
    # # # you cannot continue training afterward
    # policy = model.policy
    # policy.save("Feb10_Task4")
    ######### independent attempt to test########
    # start a new episode

    obs = env.reset()
    n_steps = 20000
    step = 0
    r=rospy.Rate(1)
    while (not rospy.is_shutdown()) and (step < n_steps):   
        action, _ = model.predict(obs, deterministic=True)
        # print("Step {}".format(step + 1))
        # print("Action: ", action)
        obs, reward, done, info = env.step(action)
        # print('obs=', obs, 'reward=', reward, 'done=', done)
        step += 1
        #env.render(mode='console')
        if done:
            # Note that the VecEnv resets automatically
            # when a done signal is encountered
            # print("Goal reached!", "reward=", reward)
            # break  
            continue
        r.sleep()