#!/bin/bash
# a shell script containing useful functions

# tinyos print helper function
tosprint(){
    java net.tinyos.tools.PrintfClient -comm serial@/dev/$1:115200
}

# tinyos log helper function
toslog(){
    java net.tinyos.tools.PrintfClient -comm serial@/dev/$1:115200 > rss.log
}

# tinyos listen helper function
toslisten(){
    java net.tinyos.tools.Listen -comm serial@/dev/$1:115200
}
