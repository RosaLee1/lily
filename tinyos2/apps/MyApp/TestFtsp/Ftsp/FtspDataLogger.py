#!/usr/bin/env python

import sys
import tos

AM_TEST_FTSP_MSG = 137

class FtspMsg(tos.Packet):
    def __init__(self, packet = None):
        tos.Packet.__init__(self,
                            [('src_addr',            'int', 2),  # node id
                             ('counter',             'int', 2),  # ref number
                             ('local_rx_timestamp',  'int', 4),
                             ('rss',  'int', 2)],
                            packet)

if '-h' in sys.argv:
    print "Usage:", sys.argv[0], "serial@/dev/ttyUSB0:115200"
    sys.exit()

am = tos.AM()

while True:
    p = am.read()
    if p and p.type == AM_TEST_FTSP_MSG:
        msg = FtspMsg(p.data)
        print msg.src_addr, msg.counter, msg.local_rx_timestamp, msg.rss
        #print msg
