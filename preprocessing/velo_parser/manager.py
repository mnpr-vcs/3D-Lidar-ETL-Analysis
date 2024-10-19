import os
import dpkt
import datetime
import numpy as np

from pathlib import Path
from tqdm import tqdm

from vlp_16 import VLP16
from utils import write_pcd, write_txt

class VelodyneManager():

    def __init__(self, type, pcap_path, out_dir, params):

        self.params = params
        self.out_dir = out_dir
        self.pcap_path = Path(pcap_path)
        
        self.lidar = None
        self.lidar_type = type
      
        self.frame_nr = 0
        self.indicies = None
        self.datetime = None
        self.distances = None
        self.timestamps = None
        
        self.pos_X = None
        self.pos_Y = None
        self.pos_Z = None
        self.intensities = None
        
        self.cur_azimuth = None
        self.last_azimuth = None


        if "velodynepuck16" == type.lower():
            self.lidar = VLP16()

    def get_pcap_length(self):
        """
        Get length of pcap file contents
        """
    
        try:
            pcap_file = open(self.pcap_path, 'rb')
            lidar_reader = dpkt.pcap.Reader(pcap_file)
        except Exception as ex:
            print(str(ex))
            return 0

        counter = 0
        for _, _ in enumerate(lidar_reader):
            counter += 1
        pcap_file.close()
        
        return counter

    def run(self):
        """
        Extract point cloud from pcap
        """

        pcap_len = self.get_pcap_length()
        if pcap_len <= 0:
            return

        try:
            fpcap = open(self.pcap_path, 'rb')
            self.lidar_reader = dpkt.pcap.Reader(fpcap)
        except Exception as ex:
            print(str(ex))
            return

        if not self.create_folders():
            return
        
        pbar = tqdm(total=pcap_len)
        for idx, (ts, buf) in enumerate(self.lidar_reader):

            if idx < self.params['from']:
                continue
            if 0 < self.params['to'] < idx:
                break

            self.datetime = datetime.datetime.utcfromtimestamp(ts)

            eth = dpkt.ethernet.Ethernet(buf)
            data = eth.data.data.data

            # Handle Data-Frame (Point clouds)
            if eth.data.data.sport == self.params['data-port']:
                self.process_data_frame(data, ts, idx)

            pbar.update(1)


    def process_data_frame(self, data, timestamp, index):

        cur_X, cur_Y, cur_Z, cur_intensities, cur_azimuth, cur_timestamps, cur_distances = self.lidar.process_data_frame(data, index)
    
        # number of sequences
        n_seq = int(len(cur_X) / self.lidar.count_lasers)

        cur_indicies = np.tile(np.arange(self.lidar.count_lasers), n_seq)

        # initilaise states
        if index == 0 or self.pos_X is None:
            self.pos_X = cur_X
            self.pos_Y = cur_Y
            self.pos_Z = cur_Z
            self.intensities = cur_distances
            self.timestamps = cur_timestamps
            self.distances = cur_distances
            self.indicies = cur_indicies
            
        if self.cur_azimuth is None:    
            self.cur_azimuth = cur_azimuth 
            self.last_azimuth = cur_azimuth 

        # update current azimuth before checking for roll over
        self.cur_azimuth = cur_azimuth

        # check if a frame is finished
        idx_rollovr = self.is_roll_over()

        # handle rollover (full 360° frame)
        if idx_rollovr is not None:

            if idx_rollovr > 0:
                self.pos_X = np.hstack((self.pos_X, cur_X[0:idx_rollovr+1]))
                self.pos_Y = np.hstack((self.pos_Y, cur_Y[0:idx_rollovr+1]))
                self.pos_Z = np.hstack((self.pos_Z, cur_Z[0:idx_rollovr+1]))
                self.intensities = np.hstack((self.intensities, cur_intensities[0:idx_rollovr+1]))
                self.timestamps = np.hstack((self.timestamps, cur_timestamps[0:idx_rollovr+1]))
                self.distances = np.hstack((self.distances, cur_distances[0:idx_rollovr+1]))
                self.indicies = np.hstack((self.indicies, cur_indicies[0:idx_rollovr+1]))

            min, sec, micro = self.time_from_lidar(self.timestamps[0])
            self.datetime = self.datetime.replace(minute=min, second=sec, microsecond=int(micro))
 
            if self.params['text']:
                fpath = "{}/{:06d}.txt".format(self.txt_path, self.frame_nr)
                write_txt(fpath, self.timestamps, self.pos_X, self.pos_Y, self.pos_Z, self.intensities)

            if self.params['ply']:
                fpath = "{}/{:06d}.pcd".format(self.pcl_path, self.frame_nr)
                write_pcd(fpath, self.pos_X, self.pos_Y, self.pos_Z, self.intensities)

            # reset states
            if idx_rollovr > 0:
                self.pos_X = cur_X[idx_rollovr+1:]
                self.pos_Y = cur_Y[idx_rollovr+1:]
                self.pos_Z = cur_Z[idx_rollovr+1:]
                self.intensities = cur_intensities[idx_rollovr+1:]
                self.timestamps = cur_timestamps[idx_rollovr+1:]
                self.distances = cur_distances[idx_rollovr+1:]
                self.indicies = cur_indicies[idx_rollovr+1:]
            else:
                self.pos_X = cur_X
                self.pos_Y = cur_Y
                self.pos_Z = cur_Z
                self.intensities = cur_intensities
                self.timestamps = cur_timestamps
                self.distances = cur_distances
                self.indicies = cur_indicies

            self.frame_nr += 1

            # reset roll over check
            self.cur_azimuth = None
            return

        self.pos_X = np.hstack((self.pos_X, cur_X))
        self.pos_Y = np.hstack((self.pos_Y, cur_Y))
        self.pos_Z = np.hstack((self.pos_Z, cur_Z))
        self.intensities = np.hstack((self.intensities, cur_intensities))
        self.timestamps = np.hstack((self.timestamps, cur_timestamps))
        self.distances = np.hstack((self.distances, cur_distances))
        self.indicies = np.hstack((self.indicies, cur_indicies))

        self.last_azimuth = cur_azimuth

    def is_roll_over(self):
        """
        Check if one frame is completed, therefore 360° rotation of the lidar
        """
        diff_cur = self.cur_azimuth[0:-1] - self.cur_azimuth[1:]
        diff_cur_last = self.cur_azimuth - self.last_azimuth

        res_cur = np.where(diff_cur > 0.)[0]
        res_cur_last =  np.where(diff_cur_last < 0.)[0]
        
        if res_cur.size > 0:
            index = res_cur[0]
            return index
        elif res_cur_last.size > 0:
            index = res_cur_last[0]
            return index
        else:
            return None

    def time_from_lidar(self, timestamp):
        """
        convert the timestamp [top of the hour in microsec] of a firing into
        minutes, seconds and microseconds
        """
        try:
            micro = timestamp % (1000*1000)
            min_float = (timestamp / (1000*1000*60)) % 60
            min = int(min_float)
            sec = int((timestamp / (1000*1000)) % 60)
            min = int(min)
        except Exception as ex:
            print(ex)
        return min, sec, micro

    def create_folders(self):
        self.out_path = Path("{}/{}".format(self.out_dir, self.lidar_type.lower()))

        # creating output dir
        try:
            os.makedirs(self.out_path.absolute())
        except Exception as ex:
            print(str(ex))
            return False

        # create point cloud dirs
        self.pcl_path = Path("{}/{}".format(self.out_path, "data_pcl"))
        try:
            os.makedirs(self.pcl_path.absolute())
        except Exception as ex:
            print(str(ex))
            return False

        # if text-files are desired, create text-file dir
        if self.params['text']:
            self.txt_path = Path("{}/{}".format(self.out_path, "data_ascii"))
            try:
                os.makedirs(self.txt_path.absolute())
            except Exception as ex:
                print(str(ex))
                return False
        return True

