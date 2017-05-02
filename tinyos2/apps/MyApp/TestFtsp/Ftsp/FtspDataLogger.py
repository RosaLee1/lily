#!/usr/bin/env python

import sys, time
import tos

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
                             ('ftsp_root_addr',      'int', 2),
                             ('ftsp_seq',            'int', 1),
                             ('ftsp_table_entries',  'int', 2)],
                            packet)

if '-h' in sys.argv:
    print "Usage:", sys.argv[0], "serial@/dev/ttyUSB0:115200"
    sys.exit()

am = tos.AM()

while True:
    p = am.read()
    if p and p.type == AM_TEST_FTSP_MSG:
        msg = FtspMsg(p.data)
        print int(time.time()), msg.src_addr, msg.counter, msg.global_rx_timestamp, msg.is_synced
        #print msg
