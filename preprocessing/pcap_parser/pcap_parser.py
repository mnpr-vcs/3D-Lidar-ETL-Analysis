import os
import glob
import yaml
import dpkt
import socket
import datetime
import numpy as np

from dataclasses import dataclass
from typing import List, Tuple, Iterator
from dpkt.utils import mac_to_str, inet_to_str

from .point_cloud import PointCloud


# Vlp-16 Params
# ---------------------------------------------------------------------------------------
LASERS = 16
BLOCKS = 12

HEADER_SIZE = 42
PACKET_SIZE = 1248

OMEGA = np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15])
OMEGA_RAD = np.deg2rad(OMEGA)
OMEGA_COS = np.cos(OMEGA_RAD)
OMEGA_SIN = np.sin(OMEGA_RAD)

AZIMUTH_RES = 0.01
DISTANCE_RES = 0.002


pkt_stream = List[Tuple[float, bytes]] # (ts, data_buffer)

@dataclass
class FiringData:
    """[summary]
    """
    
    azimuth: np.ndarray
    distance: np.ndarray
    intensity: np.ndarray
    

def parse_data_packets(buffer: bytes):
    """[summary]

    Args:
        buffer (bytes): [description]

    Returns:
        [type]: [description]
    """
    
    azimuth = np.ndarray((BLOCKS,), np.uint16, buffer, 2, (100,)) * AZIMUTH_RES
    azimuth = np.repeat(azimuth, LASERS).reshape(BLOCKS, LASERS)
    distance = np.ndarray((BLOCKS, LASERS), np.uint16, buffer, 4, (100, 3)) * DISTANCE_RES
    intensity = np.ndarray((BLOCKS, LASERS), np.uint8, buffer, 6, (100,3))
    
    return azimuth, distance, intensity


def yield_firings(buffer: bytes) -> Iterator[FiringData]:
    """[summary]

    Args:
        buffer (bytes): [description]

    Yields:
        Iterator[FiringData]: [description]
    """
    
    azimuth, distance, intensity = parse_data_packets(buffer)
    for i in range(BLOCKS):
        firing = FiringData(azimuth[i], distance[i], intensity[i])
        
        yield firing


def spherical_to_cartesian(firings: List[FiringData]) -> np.ndarray:
    """[summary]

    Args:
        firings (List[FiringData]): [description]

    Returns:
        np.ndarray: [description]
    """
    
    len_firings = len(firings)
    xyzi = np.zeros((len_firings* LASERS, 4))
    
    for idx, firing in enumerate(firings):
        
        azimuth_rad = np.deg2rad(firing.azimuth)
        r_cos_omega = firing.distance * OMEGA_COS
        
        start_idx = LASERS * idx
        end_idx = start_idx + LASERS
        
        xyzi[start_idx:end_idx, 0] = r_cos_omega * np.sin(azimuth_rad)
        xyzi[start_idx:end_idx, 1] = r_cos_omega * np.cos(azimuth_rad)
        xyzi[start_idx:end_idx, 2] = firing.distance * OMEGA_SIN
        xyzi[start_idx:end_idx, 3] = firing.intensity
    
    return xyzi

def yield_point_clouds(packet_stream : pkt_stream) -> Iterator[PointCloud]:
    """[summary]

    Args:
        packet_stream (pkt_stream): [description]

    Yields:
        Iterator[PointCloud]: [description]
    """
    
    prev_azimuth = 0
    firings_buffer : List[FiringData] = []
    
    for timestamp, packet in packet_stream:
        
        if len(packet) != PACKET_SIZE:
                continue
        
        for firing in yield_firings(packet[42:]):
            if prev_azimuth > firing.azimuth[0]:
                
                xyzi = spherical_to_cartesian(firings_buffer)
                xyzi = xyzi[np.where(np.count_nonzero(xyzi[:, :3], axis=1))]
                point_cloud = PointCloud.from_numpy(xyzi)
                
                firings_buffer = []
                
                yield point_cloud
                
            firings_buffer.append(firing)
            prev_azimuth = firing.azimuth[0]
            
    xyzi = spherical_to_cartesian(firings_buffer)
    xyzi = xyzi[np.where(np.count_nonzero(xyzi[:, :3], axis=1))]
    point_cloud = PointCloud.from_numpy(xyzi)
    
    yield point_cloud
    
def count_rotations(packet_stream: pkt_stream) -> int:
    """[summary]

    Args:
        packet_stream (pkt_stream): [description]

    Returns:
        int: [description]
    """

    count = 1
    prev_max_azi = 0

    for timestamp, packet in packet_stream:
        
        if len(packet) != PACKET_SIZE:
            continue
        
        # use same binary parsing as in parse_data_packet()
        min_azi, max_azi = np.ndarray((2,), np.uint16, packet, HEADER_SIZE + 2, (1100,))
        if (max_azi < min_azi or prev_max_azi > min_azi):
            count += 1
        prev_max_azi = max_azi

    return count
    



