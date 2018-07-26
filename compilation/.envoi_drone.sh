#!/usr/bin/expect -f

spawn scp ../build/navio2/bin/arducopter-quad pi@172.16.10.10:~/ardupilot/build/navio2/bin/
expect "pi@172.16.10.10's password: "
send "raspberry\r"
interact

