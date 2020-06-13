#!/bin/bash
scp *.c* *.h Makefile root@192.168.1.124:~/qr/
ssh root@192.168.1.124 "echo \"CPPFLAGS += -UGUI\" >> /root/qr/Makefile"
ssh root@192.168.1.124 make -C /root/qr -f /root/qr/Makefile
exit 0
