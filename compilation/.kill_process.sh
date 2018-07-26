#!/usr/bin/expect -f

spawn ssh pi@172.16.10.10 "sudo pkill arducopter+"
expect "pi@172.16.10.10's password: "
send "raspberry\r"
interact
