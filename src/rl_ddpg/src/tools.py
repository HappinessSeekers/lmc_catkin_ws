#!/usr/bin/env python
# coding=utf-8
import os
import sys
import csv
import time

class Record(object):
    def __init__(self,name,record_path="/data/train/data_example/record"):
        self.record_path = sys.path[0] + record_path + name + ".csv"
        self.name = name 
        self.dataList = []
    def add(self,data):
        self.dataList.append(data)
    def clear(self):
        self.dataList = []
    def save(self):
        with open(self.record_path,'w') as f:
            f_csv = csv.writer(f)
            f_csv.writerows(self.dataList)

def getTime():
    return time.strftime('%Y%m%d%H%M',time.localtime()) 

def create_dir(path):
    os.mkdir(sys.path[0]+path)
    os.mkdir(sys.path[0]+path+'/model/')
    os.mkdir(sys.path[0]+path+'/record/')
    
def create_test_dir(path):
    os.mkdir(sys.path[0]+path)
    os.mkdir(sys.path[0]+path+'/record/')