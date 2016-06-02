__author__ = 'kehlinswain'
import time

def micros():
   return time.time()*1000000 #- time.time()/1000000

def elapsed_micros(start_time):
    curent_time = time.time()*1000000
    elapsed_time = start_time - time.time()*1000000
    return elapsed_time
