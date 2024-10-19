import glob
import config
import socket
import datetime

import dpkt
from dpkt.utils import mac_to_str

pcap_files_list = glob.glob("{base}/*.pcap".format(base=config.default['PCAP_PATH']))
first_file = pcap_files_list[1]


def inet_to_str(inet):
    try:
        return socket.inet_ntop(socket.AF_INET, inet)
    except ValueError:
        return socket.inet_ntop(socket.AF_INET6, inet)


def get_pcap_len(pcap_path):
    """Print out information about each packet in a pcap
       Args:
           pcap_path
       Returns:
           pcap_length (int)
    """
    
    pcap_length = 0
    with open(pcap_path, 'rb') as f:
        try:
            pcap_reader = dpkt.pcap.Reader(f)
        except Exception as ex:
            print(str(ex))
            return 0
    
        for _, _ in enumerate(pcap_reader):
            pcap_length += 1     
                      
    return pcap_length

def get_packet_info(pcap, pretty_print=False):
    """Print out information about each packet in a pcap
       Args:
           pcap: dpkt pcap reader object (dpkt.pcap.Reader)
    """
    # For each packet in the pcap process the contents
    for timestamp, packet_buffer in pcap:
        
        eth = dpkt.ethernet.Ethernet(packet_buffer)
        
        # if Ethernet data contains an IP packet
        if not isinstance(eth.data, dpkt.ip.IP):
            print('Non IP Packet type not supported %s\n' % eth.data.__class__.__name__)
            continue
        
        ip = eth.data
        udp = ip.data

        if pretty_print:
            eth.pprint()
            return 0
        
        # Info : Timestamp, Eth, IP, UDP frames        
        print('Timestamp: ', str(datetime.datetime.utcfromtimestamp(timestamp)))
        print('Ethernet Frame: ', mac_to_str(eth.src), mac_to_str(eth.dst), eth.type)
        print('IP: %s -> %s   (len=%d ttl=%d DF=%d MF=%d offset=%d)' %
        (inet_to_str(ip.src), inet_to_str(ip.dst), ip.len, ip.ttl, ip.df, ip.mf, ip.offset))
        print(f'Dtype Data payload : {type(udp.data)}, Length : {len(udp.data)}\n')
        

print(f'Pcap Length: {get_pcap_len(first_file)}')

with open(first_file, 'rb') as f:
    
    pcap = dpkt.pcap.Reader(f)
    get_packet_info(pcap)
    # get_packet_info(pcap, pretty_print=True)