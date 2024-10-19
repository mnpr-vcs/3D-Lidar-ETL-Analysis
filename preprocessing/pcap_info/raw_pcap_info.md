# Raw network packets(`*.pcap`) containing Lidar data

## Data Captured as Network packets (`*.pcap`)

**Script used:**

```
cd..
cd..
cd..
cd program files
cd wireshark
path
dumpcap -P -i 4 -w D:/Lidardaten/vergleich.pcap -b filesize:700000
PAUSE
```

## General Captured `Vlp-16` Packet Info

```
# Source : 192.168.1.201 (Factory Set IP address )
# Destiation 255.255.255.255 (Broadcast)
# IP Frame : 1234 Byte [UDP Frame + ... ]
# UDP Frame: 1214 Byte [UDP Payload + ... ]
# UDP Payload : 1206 Byte 
    - 12 X 100 byte blocks with each block[ ... ]
        - timestamp(uint)
        - azimuth(uint) * 100
        - 16 X [distance(uint) * 500 , intensity(byte)]
    - 4 byte Timestamp, 2 byte factory
# Src/Dst port : 2368
```

## Packet analysis using [`dpkt`](https://dpkt.readthedocs.io/en/latest/)

**Print-info**

- `conda activate <env name>` 
- `python pcap_info.py`

*if `pretty_print=False`*

```
Timestamp:  2021-10-25 13:58:38.663520
Ethernet Frame:  60:76:88:38:75:ce ff:ff:ff:ff:ff:ff 2048
IP: 192.168.1.201 -> 255.255.255.255   (len=1234 ttl=255 DF=1 MF=0 offset=0)
Dtype Data payload : <class 'bytes'>, Length : 1206
```

*if `pretty_print=True`*

```
Ethernet(
  dst=b'\xff\xff\xff\xff\xff\xff',  # ff:ff:ff:ff:ff:ff
  src=b'`v\x888u\xce',  # 60:76:88:38:75:ce
  type=2048,
  data=IP(
    v=4,
    hl=5,
    tos=0,
    len=1234,
    id=0,
    rf=0,
    df=1,
    mf=0,
    offset=0,
    ttl=255,
    p=17,  # UDP
    sum=46249,  # 0xb4a9
    src=b'\xc0\xa8\x01\xc9',  # 192.168.1.201
    dst=b'\xff\xff\xff\xff',  # 255.255.255.255
    opts=b'',
    data=UDP(
      sport=2368,
      dport=2368,
      ulen=1214,
      sum=0,
      data=b'\xff\xee\x9b0\xb5\x01\x0f\x00\x00\x18\x8d\x02\x11\x00\ ... "'
    )  # UDP
  )  # IP
)  # Ethernet
``