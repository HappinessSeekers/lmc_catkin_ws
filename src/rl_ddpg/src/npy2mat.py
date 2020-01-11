#!/usr/bin/env python
# coding:utf-8

import numpy as np
from scipy import io
import sys
def npy2mat(path):
    a_cost_his = np.load(path+"a_cost_his.npy").tolist()
    c_cost_his = np.load(path+"c_cost_his.npy").tolist()
    average_reward_his = np.load(path+"average_reward_his.npy").tolist()
    io.savemat(path+'train_his.mat',{'a_cost_his':a_cost_his,'c_cost_his':c_cost_his,'average_reward_his':average_reward_his})

if __name__ == '__main__':
    path = sys.path[0] + "/data/train/"+"202001081020"+"/model/"
    npy2mat(path)