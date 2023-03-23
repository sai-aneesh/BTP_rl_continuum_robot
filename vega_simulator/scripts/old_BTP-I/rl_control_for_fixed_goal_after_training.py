#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
from math import ceil, pi
from std_msgs.msg import Float32MultiArray, Float32, Bool
from time import sleep
import numpy as np
import os
import gym
from gym import spaces
from time import sleep
import stable_baselines3
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

        #creating all subscibers
        rospy.Subscriber("/steady_system/state",Float32MultiArray,self.state_callback,queue_size=1)
        # rospy.Subscriber("/actual_system/velocity",Float32MultiArray,self.velocity_callback,queue_size=1)


        #defining the messages and state varaibles
        self.forces_msg=Float32MultiArray()
        self.reset_msg = Bool()
        self.measured_state = np.array([0,0,0,0])
        self.velocity = np.array([0,0])

        # t_x = float(np.random.uniform(-0.5,0.5))
        # t_y = float(np.random.uniform(0,0.5))
        self.target = np.array([0.24,0.04])
        # rospy.loginfo(self.target)

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

        #set new target position
        # t_x = float(np.random.uniform(-0.5,0.5))
        # t_y = float(np.random.uniform(0,0.5))
        # self.target = np.array([t_x,t_y])
        # rospy.loginfo(self.target)

        # observation should match observation space
        return np.array(self.measured_state).astype(np.float32)

    def step(self, action):
        # r=rospy.Rate(0.3)
        self.check_state = False
        # print(action.shape)
        # print('action',action)
        action =  np.zeros(shape=(5,)) + action*3
        # print("action:",action)
        torques = np.zeros(shape=(5,))
        # print(torques)
        torques[4] = action[4]
        torques[3] = action[3]-action[4]
        torques[2] = action[2]-action[3]
        torques[1] = action[1]-action[2]
        torques[0] = action[0]-action[1]
        # print(torques)

        self.forces_msg.data= torques
        # rospy.loginfo("published the force to steady state checker")
        self.force_pub.publish(self.forces_msg)

        while not (self.check_state):
            continue

        x = self.measured_state[2]
        y = self.measured_state[3]

        distance = np.sqrt(((x - self.target[0])**2+ (y - self.target[1])**2))

        #check if we reached the goal
        done = bool(distance < 0.01)

        # Null reward everywhere except when reaching the goal (left of the grid)
        # reward = - 10*distance  -(self.velocity[0]**2 + self.velocity[1]**2)*100

        reward = - 10*distance 
        info = {}
        return np.array(self.measured_state).astype(np.float32), reward, done, info


    def close(self):
        pass

if __name__ == '__main__':

    rospy.init_node('Controller_node', anonymous = True)

    n_sections = int(sys.argv[1])
    # float(sys.argv[2]),int(sys.argv[3])

    env = vega_env(n_sections)
    #obs = env.reset()
    #print(obs)
    # If the environment don't follow the interface, an error will be thrown
    # check_env(env, warn=True)
    # # wrap it
    # env = make_vec_env(lambda: env, n_envs=1)
    # # Train the agent
    # # model = SAC('MlpPolicy', env, verbose=1).learn(total_timesteps=int(2e4))
    # model = SAC('MlpPolicy', env, verbose=1, learning_starts = 10, batch_size=64, gamma = 0.5, use_sde = True, tensorboard_log ="/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/results").learn(total_timesteps=int(1e4), progress_bar=True, tb_log_name = "SAC")

    # # save the model
    log_dir = os.getcwd()
    print(log_dir)
    # model.save(log_dir + "SAC_Vega")

    # load the trained model
    model = SAC.load("/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/SAC_Vega.zip")    
    # Save the policy independently from the model
    # Note: if you don't save the complete model with `model.save()`
    # # you cannot continue training afterward
    # policy = model.policy
    # # policy.save("SAC_policy_vega")
    ######### independent attempt to test########
    # start a new episode
    obs = env.reset()
    n_steps = 200
    step = 0
    r=rospy.Rate(1)
    while (not rospy.is_shutdown()) and (step < n_steps):   
        action, _ = model.predict(obs, deterministic=True)
        print("Step {}".format(step + 1))
        print("Action: ", action)
        obs, reward, done, info = env.step(action)
        print('obs=', obs, 'reward=', reward, 'done=', done)
        step += 1
        #env.render(mode='console')
        if done:
            # Note that the VecEnv resets automatically
            # when a done signal is encountered
            print("Goal reached!", "reward=", reward)
            break
        r.sleep()


######################################################## now python code for moving goal
#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
from math import ceil, pi
from std_msgs.msg import Float32MultiArray, Float32, Bool
from time import sleep
import numpy as np
import os
import gym
from gym import spaces
from time import sleep
import stable_baselines3
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

        #creating all subscibers
        rospy.Subscriber("/steady_system/state",Float32MultiArray,self.state_callback,queue_size=1)
        # rospy.Subscriber("/actual_system/velocity",Float32MultiArray,self.velocity_callback,queue_size=1)


        #defining the messages and state varaibles
        self.forces_msg=Float32MultiArray()
        self.reset_msg = Bool()
        self.measured_state = np.array([0,0,0,0])
        self.velocity = np.array([0,0])

        # t_x = float(np.random.uniform(-0.5,0.5))
        # t_y = float(np.random.uniform(0,0.5))
        self.target = np.array([0.22,0.04])
        # rospy.loginfo(self.target)

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
        rospy.loginfo("received new state from steaady state checker")
        rospy.loginfo(self.measured_state)

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

        # set new target position
        t_x = float(np.random.uniform(-0.5,0.5))
        t_y = float(np.random.uniform(0,0.25))
        self.target = np.array([t_x,t_y])
        rospy.loginfo(self.target)

        # observation should match observation space
        return np.array(self.measured_state).astype(np.float32)

    def step(self, action):
        # r=rospy.Rate(0.3)
        self.check_state = False
        # print(action.shape)
        # print('action',action)
        action =  np.zeros(shape=(5,)) + action*3
        # print("action:",action)
        torques = np.zeros(shape=(5,))
        # print(torques)
        torques[4] = action[4]
        torques[3] = action[3]-action[4]
        torques[2] = action[2]-action[3]
        torques[1] = action[1]-action[2]
        torques[0] = action[0]-action[1]
        # print(torques)

        self.forces_msg.data= torques
        rospy.loginfo("published the force to steady state checker")
        self.force_pub.publish(self.forces_msg)

        while not (self.check_state):
            continue

        x = self.measured_state[2]
        y = self.measured_state[3]

        distance = np.sqrt(((x - self.target[0])**2+ (y - self.target[1])**2))

        #check if we reached the goal
        done = bool(distance < 0.03)

        # Null reward everywhere except when reaching the goal (left of the grid)
        # reward = - 10*distance  -(self.velocity[0]**2 + self.velocity[1]**2)*100

        reward = - 10*distance 
        info = {}
        return np.array(self.measured_state).astype(np.float32), reward, done, info


    def close(self):
        pass

if __name__ == '__main__':

    rospy.init_node('Controller_node', anonymous = True)

    n_sections = int(sys.argv[1])
    # # float(sys.argv[2]),int(sys.argv[3])

    env = vega_env(n_sections)
    # obs = env.reset()
    # # print(obs)
    # # If the environment don't follow the interface, an error will be thrown
    # check_env(env, warn=True)
    # # wrap it
    # env = make_vec_env(lambda: env, n_envs=1)
    # # Train the agent
    # # model = SAC('MlpPolicy', env, verbose=1).learn(total_timesteps=int(2e4))
    # model = SAC('MlpPolicy', env, verbose=1, learning_starts = 10, batch_size=64, gamma = 0.5, use_sde = True, tensorboard_log ="/home/aneesh/catkin_ws/src/Soft_robo_sim/vega_simulator/results").learn(total_timesteps=int(2e4), progress_bar=True, tb_log_name = "SAC")

    # # # save the model
    log_dir = os.getcwd()
    # print(log_dir)
    # model.save(log_dir + "/SAC_Vega")

    # load the trained model
    model = SAC.load(log_dir + "/SAC_Vega")    
    # Save the policy independently from the model
    # Note: if you don't save the complete model with `model.save()`
    # you cannot continue training afterward
    # policy = model.policy
    # policy.save(log_dir + "/SAC_policy_vega")
    # Retrieve the environment
    env = model.get_env()
    # Evaluate the policy
    # mean_reward, std_reward = evaluate_policy(policy, env, n_eval_episodes=10,deterministic=True)  #lways some error with this line
    # print(f"mean_reward={mean_reward:.2f} +/- {std_reward}")
    # Load the policy independently from the model
    # saved_policy = MlpPolicy.load(log_dir + "/SAC_policy_vega")
    # Evaluate the loaded policy
    # mean_reward, std_reward = evaluate_policy(saved_policy, env, n_eval_episodes=10,deterministic=True)
    # print(f"mean_reward={mean_reward:.2f} +/- {std_reward}")

    ######### independent attempt to test########
    # start a new episode
    obs = env.reset()
    n_steps = 200
    step = 0
    r=rospy.Rate(1)
    while (not rospy.is_shutdown()) and (step < n_steps):   
        action, _ = model.predict(obs, deterministic=True)
        print("Step {}".format(step + 1))
        print("Action: ", action)
        obs, reward, done, info = env.step(action)
        print('obs=', obs, 'reward=', reward, 'done=', done)
        step += 1
        #env.render(mode='console')
        if done:
            print("Goal reached!", "reward=", reward)
            break
        r.sleep()

