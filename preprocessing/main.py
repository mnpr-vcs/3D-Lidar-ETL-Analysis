import glob
import dpkt
import numpy as np
import open3d as o3d

import pcap_parser.transforms as Tx
import pcap_parser.parse_pcap as utils


# list of pcap files in directory
# ---------------------------------------------------------------------------------------
PCAP_PATH='../data/ferry_pcap/'
pcap_files = glob.glob("{base}/*.pcap".format(base=PCAP_PATH))
file_path = pcap_files[1]

# pipeline = Tx.Compose([
#     Tx.AxisRotate(
#         axis=[0, 0, 1],
#         angle=0.7070
#         ),
#     Tx.PolygonCrop(polygon=[
#             [0.0, -10.0],
#             [-4.0, 0.0],
#             [-4.0, 5.0],
#             [-20.0, 10.0],
#             [-20.0, -12.0],
#             [0.0, -32.0]
#         ]),
#     Tx.Translate(x=5, y=-10)
# ])


with open(file_path, "rb") as f:
    packet_stream = dpkt.pcap.Reader(f)

    # feed the stream into cloud generator
    pc_generator = utils.yield_point_clouds(packet_stream)

    # do something with the clouds
    for i, pc in enumerate(pc_generator):
        # pc = pipeline.apply(pc)
        # pc.data.astype(np.float32).tofile(f"data/cloud_{i}.bin")
        
        pcd_data = pc.data.astype(np.float32)
        pcd = o3d.geometry.PointCloud()
        v3d = o3d.utility.Vector3dVector
        pcd.points = v3d(pcd_data)
        o3d.io.write.write_point_cloud(f"data/cloud_{i}.pcd", pcd)
        
