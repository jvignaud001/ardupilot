#!/usr/bin/expect -f
spawn scp ../ParametresDrone.conf pi@172.16.10.10:~/ardupilot/build/navio2/bin/
expect "pi@172.16.10.10's password: "
send "raspberry\r"
interact

