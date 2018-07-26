#!/usr/bin/expect -f

spawn ssh pi@172.16.10.10 "sudo reboot"
expect "pi@172.16.10.10's password: "
send "raspberry\r"
interact
