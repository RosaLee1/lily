#!/bin/bash
# a shell script containing useful functions

# tinyos print helper function
tosprint(){
    java net.tinyos.tools.PrintfClient -comm serial@/dev/ttyUSB$1:115200
}

# tinyos listen helper function
toslisten(){
    java net.tinyos.tools.Listen -comm serial@/dev/ttyUSB$1:115200
}

# tinyos log helper function
toslog(){
    python FtspDataLogger.py serial@/dev/ttyUSB$1:115200 
}

# tinyos rss helper function
rss(){
    java TestSerial -comm serial@/dev/ttyUSB$1:115200
}
