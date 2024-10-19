#!/usr/bin/bash
#
#
# Script used to capture Data
# ----------------------------------------------------------------------
# cd..
# cd..
# cd..
# cd program files
# cd wireshark
# path
# dumpcap -P -i 4 -w D:/Lidardaten/vergleich.pcap -b filesize:700000
# PAUSE

# Another script
# ----------------------------------------------------------------------
# HOST=192.168.1.201
# tcpdump -i enp2s0f1 -nn -vvv  -net host ${HOST}  -w /var/tcpdump/capture-%H-%M-%S.pcap -G 1

# Pcap Payload info
# ----------------------------------------------------------------------
# Source : 192.168.1.201
# Destiation 255.255.255.255 (Broadcast)
# UDP Frame: 1248 Bytes
# UDP payload(Data) size : 1206 Bytes
# Src/Dst port : 2368


# Conda Env : 
# conda create -n pcap-proc python=3.8 -y


