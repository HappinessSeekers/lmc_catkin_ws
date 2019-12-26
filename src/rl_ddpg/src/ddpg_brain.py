#!/usr/bin/env python
# coding:utf-8
import tensorflow as tf
import numpy as np
import sys


class DDPG(object):
    def __init__(self,
                 a_dim,
                 s_dim,
                 a_bound,
                 gamma=0.8,
                 lr_a=0.001,
                 lr_c=0.002,
                 tau=0.01,
                 memory_size=5000,
                 batch_size=128,
                 var_start=0.5,
                 var_replace_ratio=0.9995,
                 is_restore=False,
                 is_save=False,
                 save_path="/data/model/",
                 restore_path=None
                 ):
        
        self.a_dim, self.s_dim, self.a_bound = a_dim, s_dim, a_bound
        self.memory_size = memory_size
        self.gamma = gamma
        self.lr_a = lr_a
        self.lr_c = lr_c
        self.tau = tau
        self.memory = np.zeros((self.memory_size, s_dim * 2 + a_dim + 1), dtype=np.float32)
        self.memory_counter = 0
        self.batch_size = batch_size
        self.var = 2 * var_start * self.a_bound
        self.var_replace_ratio = var_replace_ratio
        self.is_restore = is_restore
        self.is_save = is_save
        self.save_path = sys.path[0] + save_path
        if restore_path is not None:
            self.restore_path = sys.path[0] + restore_path
        else:
            self.restore_path = self.save_path
        # put placeholder
        self.S = tf.placeholder(tf.float32, [None, s_dim], 's')
        self.S_ = tf.placeholder(tf.float32, [None, s_dim], 's_')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')
        # build network
        with tf.variable_scope('Actor'):
            self.a = self._build_a(self.S, scope='eval', trainable=True)
            self.a_ = self._build_a(self.S_, scope='target', trainable=False)
        with tf.variable_scope('Critic'):
            # assign self.a = a in memory when calculating q for td_error,
            # otherwise the self.a is from Actor when updating Actor
            self.q = self._build_c(self.S, self.a, scope='eval', trainable=True)
            self.q_ = self._build_c(self.S_, self.a_, scope='target', trainable=False)
        # networks parameters
        self.ae_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval')
        self.at_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target')
        self.ce_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval')
        self.ct_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target')
        # sess 
        self.sess = tf.Session()
        # target net replacement
        self.soft_replace = [tf.assign(t, (1 - self.tau) * t + self.tau * e)
                             for t, e in zip(self.at_params + self.ct_params, self.ae_params + self.ce_params)]
        q_target = self.R + self.gamma * self.q_
        # in the feed_dic for the td_error, the self.a should change to actions in memory
        self.td_error = tf.losses.mean_squared_error(labels=q_target, predictions=self.q)
        self.ctrain = tf.train.AdamOptimizer(self.lr_c).minimize(self.td_error, var_list=self.ce_params)
        # calculate loss
        self.a_loss = - tf.reduce_mean(self.q)  # maximize the q
        self.atrain = tf.train.AdamOptimizer(self.lr_a).minimize(self.a_loss, var_list=self.ae_params)
        self.sess.run(tf.global_variables_initializer())
        # loss his 
        self.c_cost_his = []
        self.a_cost_his = []
        self.c_cost = 0.0
        self.a_cost = 0.0
        self.average_reward = 0.0
        self.average_reward_his =[]
        # saver and restore
        self.saver = tf.compat.v1.train.Saver()
        self.restore()

    def _build_a(self, s, scope, trainable):
        with tf.variable_scope(scope, initializer=tf.contrib.layers.xavier_initializer()):
            net1 = tf.layers.dense(s, 30, activation=tf.nn.relu, name='l1', trainable=trainable)
            net2 = tf.layers.dense(net1, 15, activation=tf.nn.relu, name='l2', trainable=trainable)
            net3 = tf.layers.dense(net2, 5, activation=tf.nn.relu, name='l3', trainable=trainable)
            a = tf.layers.dense(net2, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            return tf.multiply(a, self.a_bound, name='scaled_a')

    def _build_c(self, s, a, scope, trainable):
        with tf.variable_scope(scope, initializer=tf.contrib.layers.xavier_initializer()):
            n_l1 = 30
            n_l2 = 15
            w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            l1 = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            l2 = tf.layers.dense(l1, n_l2, trainable=trainable)
            return tf.layers.dense(l2, 1, trainable=trainable)  # Q(s,a)

    def choose_action(self, s):
        action = self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]
        action = np.clip(np.random.normal(action, self.var), -self.a_bound, self.a_bound)
        return action

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.memory_counter % self.memory_size  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.memory_counter += 1

    def lr_var_replace(self):
        if self.memory_counter > self.memory_size:
            self.var *= self.var_replace_ratio
        if (self.memory_counter == self.memory_size * 2) or (self.memory_counter == self.memory_size * 4) or(self.memory_counter == self.memory_size * 8):
            self.lr_a = self.lr_a * 0.5
            self.lr_c = self.lr_a * 0.5

    def learn(self):
        # soft target replacement
        self.sess.run(self.soft_replace)
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
            # print(self.var)
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        self.lr_var_replace()
        bt = self.memory[sample_index, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]

        bs_ = bt[:, -self.s_dim:]
        _, self.a_cost = self.sess.run([self.atrain, self.a_loss], feed_dict={self.S: bs})
        _, self.c_cost = self.sess.run([self.ctrain, self.td_error], {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})
        self.average_reward = np.mean(br)
        self.average_reward_his.append(self.average_reward)
        self.a_cost_his.append(self.a_cost)
        self.c_cost_his.append(self.c_cost)

    def plot_a_cost(self, length = 0):
        import matplotlib.pyplot as plt
        if length == 0:
            plt.plot(np.arange(len(self.a_cost_his)-20), self.a_cost_his[20:])
        else:
            if length >= 0:
                length_plot = min(len(self.a_cost_his)-20, length)
                plt.plot(np.arange(length_plot), self.a_cost_his[20:length_plot+20])
            else:
                length = -length 
                length_plot = min(len(self.a_cost_his), length)
                plt.plot(np.arange(length_plot), self.a_cost_his[-length_plot:])
        plt.ylabel('A_Cost')
        plt.xlabel('training steps')
        plt.show()

    def plot_c_cost(self, length=0):
        import matplotlib.pyplot as plt
        if length == 0:
            plt.plot(np.arange(len(self.c_cost_his)), np.log(self.c_cost_his[:]))
        else:
            if length >= 0:
                length_plot = min(len(self.c_cost_his), length)
                plt.plot(np.arange(length_plot), np.log(self.c_cost_his[:length_plot]))
            else:
                length_plot = min(len(self.c_cost_his), -length)
                plt.plot(np.arange(length_plot), np.log(self.c_cost_his[-length_plot:]))
        plt.ylabel('C_Cost')
        plt.xlabel('training steps')
        plt.show()

    def plot_average_reward(self, length=0):
        import matplotlib.pyplot as plt
        if length == 0:
            plt.plot(np.arange(len(self.average_reward_his)), self.average_reward_his[:])
        else:
            if length >= 0:
                length_plot = min(len(self.average_reward_his), length)
                plt.plot(np.arange(length_plot), self.average_reward_his[:length_plot])
            else:
                length_plot = min(len(self.average_reward_his), -length)
                plt.plot(np.arange(length_plot), self.average_reward_his[-length_plot:])
        plt.ylabel('average_reward')
        plt.xlabel('training steps')
        plt.show()

    
    def save(self):
        if self.is_save is True:
            self.saver.save(self.sess, self.save_path + 'model.ckpt')
            print("save successfully")
            a_cost_his = np.array(self.a_cost_his)
            np.save(self.save_path+"a_cost_his.npy", a_cost_his)
            c_cost_his = np.array(self.c_cost_his)
            np.save(self.save_path+"c_cost_his.npy", c_cost_his)
            average_reward_his = np.array(self.average_reward_his)
            np.save(self.save_path+"average_reward_his.npy", average_reward_his)
        else:
            pass

    def restore(self):
        if self.is_restore is True:
            ckpt = tf.train.get_checkpoint_state(self.restore_path)
            if ckpt and ckpt.model_checkpoint_path:
                self.saver.restore(self.sess, ckpt.model_checkpoint_path)
                print("restore successfully")
            self.a_cost_his = np.load(self.restore_path+"a_cost_his.npy").tolist()
            self.c_cost_his = np.load(self.restore_path+"c_cost_his.npy").tolist()
            self.average_reward_his = np.load(self.restore_path+"average_reward_his.npy").tolist()
        else:
            pass



        