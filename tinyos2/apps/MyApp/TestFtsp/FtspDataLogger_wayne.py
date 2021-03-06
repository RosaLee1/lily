#!/usr/bin/env python

import sys, time
import tos
import numpy as np

AM_TEST_FTSP_MSG = 137

class FtspMsg(tos.Packet):
    def __init__(self, packet = None):
        tos.Packet.__init__(self,
                            [('src_addr',            'int', 2),  # node id
                             ('counter',             'int', 2),  # ref number
                             ('local_rx_timestamp',  'int', 4),
                             ('global_rx_timestamp', 'int', 4),  # local time of each node
                             ('skew_times_1000000',  'int', 4),
                             ('is_synced',           'int', 1),  # only node #2's state needs attention, node #1's state is always 0.
                                                                 # z1 returns: state 0 - synchronized; state 1 - not synchronized
                                                                 # this python script further divide state 0 to two sub-states:
                                                                 # - state 2: collecting timestamps for slope calculation
                                                                 # - state 3: time jump
                             ('ftsp_root_addr',      'int', 2),
                             ('ftsp_seq',            'int', 1),
                             ('ftsp_table_entries',  'int', 2)],
                            packet)

if '-h' in sys.argv:
    print "Usage:", sys.argv[0], "serial@/dev/ttyUSB0:115200"
    sys.exit()

am = tos.AM()
#system setup
sample_num = 10
node1_sample = 10
start_point = 0
jump = False
calculate_ready = False #get 5 offset and error<5 and no jump
error_threshold = 10

# set node1t & 2t an 2 x 5(sample) zeros array
node1t = [[0]*node1_sample for i in range(2)]
node2t = [[0]*sample_num for i in range(2)]
#setup offset for error_gap calculation
#offset = [0,0]
offset = [[0]*sample_num for i in range(3)] #10*3 array
error_offset = [[0]*(sample_num-1) for i in range(2)] 
error_skew = 0
# set all array from tuple to list
'''
node1t = list(node1t)
node2t = list(node2t)
offset = list(offset)
error_offset = list(error_offset)
'''
count1 = 0
count2 = 0
count_offset = 0
#print countoffset
data_count = 0
match = 0
while True:
    p = am.read()
    if p and p.type == AM_TEST_FTSP_MSG:
        msg = FtspMsg(p.data)
        #save node1's counter and time
        if msg.src_addr == 1 :
            node1t[0][count1%node1_sample] = msg.counter
            node1t[1][count1%node1_sample] = msg.global_rx_timestamp
            count1 += 1
            print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced #print Node1
        
        elif msg.src_addr == 2 and msg.is_synced == 0: #when node 2 synchronized synchronized
            #store node2's counter and time to array
            if calculate_ready == True:
                #print localtime + offset + error_gap*(msg.counter - startpoint)
                #msg.global_rx_timestamp = msg.global_rx_timestamp + sum(error_offset[1]) + error_skew*(msg.counter - start_point - sum(error_offset[0]))
                msg.global_rx_timestamp = msg.global_rx_timestamp + offset[1][0]+ sum(error_offset[1]) + error_skew*(msg.counter - start_point - sum(error_offset[0]))
                print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced
            else: #save data into array
                node2t[0][count2%sample_num] = msg.counter
                node2t[1][count2%sample_num] = msg.global_rx_timestamp
                count2 += 1
                data_count += 1 #set to 0 if jump
                #print 'node2t: ',node2t
                #check we are ready or not
                #get 5 no jump data
                if data_count >= 5:
                    #print 'enter data_got_5'
                    jump = False
                    match = 0
                    offset[2] = [0]*sample_num
                    #print 'node1t: ', node1t
                    for i in range(0,sample_num):
                        for j in range(0,node1_sample): #everytime we find a match we decide calculate error or not
                            if node2t[0][i] == node1t[0][j] != 0:  #check seq, if we can find a match, if no, skip
                                #find one match, and data_count+=1
                                offset[0][count_offset] = node1t[0][j] #store sequence number
                                #print offset[0][count_offset]
                                offset[1][count_offset] = node1t[1][j] - node2t[1][i] #store offset
                                #print 'offset[1] : ', offset[1]
                                offset[2][count_offset] = 1 #label matched seq#
                                #offset = tuple(temp)
                                count_offset = (count_offset+1)%sample_num
                    #we dont want to take current number
                    for i in range(0,sample_num):
                        if offset[0][i] == msg.counter: #if current seq number is in array, ignore by set offset[2][i] = 0
                            offset[2][i] = 0

                    #now we get all data in offset, now sort offset
                    #print 'offset before zip: ', offset
                    temp = offset
                    temp = zip(*temp)
                    temp.sort(key=lambda x:x[0]) #sort first row
                    temp = zip(*temp)
                    offset = list(map(list,temp))
                    #print 'offset after zip:', offset
                    #print 'after zip:', offset
                    #print offset
                    start_point = offset[0][0] #the start sequence number
                    #put sorted number into error_offset 9*2
                    error_offset = [[0]*(sample_num-1) for i in range(2)] #set error to zeros
                    for i in range(0,sample_num - 2):
                        get_one = False
                        for j in range( i+1, sample_num-1):
                            if offset[2][i] == 1 and offset[2][j] == 1 and get_one == False: #matched sequence number
                                error_offset[0][i] = offset[0][j] - offset[0][i] #sequence number gap
                                error_offset[1][i] = offset[1][j] - offset[1][i] #time error gap
                                get_one = True
                                if abs(error_offset[1][i]) > error_threshold:
                                    data_count = 0 #reset availible data
                                    jump = True
                                    calculate_ready = False
                    #calculate error_skew
                    if jump == False:
                        #print 'sumoffset and error offset: ', sum(error_offset[1]), sum(error_offset[0])
                        #print 'error_offset[0]', error_offset[0]
                        if sum(error_offset[0]) != 0:
                            error_skew = float(sum(error_offset[1]))/float(sum(error_offset[0]))
                            error_skew = int(error_skew)
                            #print 'error_skew: ', error_skew
                            #print 'error_offset: ', error_offset
                            calculate_ready == True
                            msg.global_rx_timestamp = msg.global_rx_timestamp + offset[1][0]+ sum(error_offset[1]) + error_skew*(msg.counter - start_point - sum(error_offset[0]))
                            print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced
                    else:
                        msg.is_synced = 3 #jump occured
                        #calculate_ready = False
                        print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced
                else:
                    msg.is_synced = 2 #collecting data
                    print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced

        else:
            print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced
        




