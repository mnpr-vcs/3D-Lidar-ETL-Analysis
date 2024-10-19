import numpy as np

from sklearn.neighbors import BallTree
from skimage.measure import points_in_poly
from scipy.spatial.transform import Rotation as ScipyRotation

from .point_cloud import PointCloud


class Compose:
    def __init__(self, transforms):
        self.transforms = transforms

    def apply(self, pc: PointCloud):
        for t in self.transforms:
            pc = t.apply(pc)
        return pc


class Translate:
    def __init__(self, x=0, y=0, z=0):
        
        self.x = x
        self.y = y
        self.z = z

    def apply(self, pc: PointCloud):
        """translate .xyz of the point cloud, this
        implementation creates outputs
        """
        new_data = pc.data + np.array([self.x, self.y, self.z, 0])
        
        return PointCloud.from_numpy( new_data, capacity=new_data.shape[0])


class CartesianClip:
    def __init__(
            self,
            x_range=[-np.inf, np.inf],
            y_range=[-np.inf, np.inf],
            z_range=[-np.inf, np.inf],
            inverse=False):
        
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.inverse = inverse

    def apply(self, pc: PointCloud):
        """
        Points that lay right at the boundary are not included
        """
        x, y, z = pc.xyz.T
        
        x_mask = (x > self.x_range[0]) & (x < self.x_range[1])
        y_mask = (y > self.y_range[0]) & (y < self.y_range[1])
        z_mask = (z > self.z_range[0]) & (z < self.z_range[1])
        
        mask = x_mask & y_mask & z_mask

        if self.inverse:
            mask = np.invert(mask)

        new_data = pc.data[mask, :]
        
        return PointCloud.from_numpy(new_data, capacity=new_data.shape[0])


class PolygonCrop:
    def __init__(
            self,
            polygon,
            z_range=[-np.inf, np.inf],
            inverse=False):
        
        self.polygon = polygon
        self.z_range = z_range
        self.inverse = inverse

    def apply(self, pc: PointCloud):
        """
        Clip the point cloud outside the given polygon
        """
        xy_mask = points_in_poly(pc.data[:, :2], self.polygon)
        mask = xy_mask
        if self.z_range[0] < self.z_range[1]:
            z = pc.data[:, 2]
            z_mask = (z > self.z_range[0]) & (z < self.z_range[1])
            mask = xy_mask & z_mask

        if self.inverse:
            mask = np.invert(mask)

        new_data = pc.data[mask, :]
        return PointCloud.from_numpy(
            new_data,
            capacity=new_data.shape[0])


class AxisRotate:
    def __init__(self, axis, angle):
        
        self.axis = np.asarray(axis)
        self.axis = self.axis / np.linalg.norm(self.axis)
        self.angle = angle

    def _get_matrix(self):
        """
        Get rotation matrix from quaterion defined by
        axis and CCW rotation angle
        """
        qw = np.cos(self.angle / 2)
        qx, qy, qz = np.sin(self.angle / 2) * self.axis
        return ScipyRotation.from_quat([qx, qy, qz, qw])

    def apply(self, pc: PointCloud):
        """
        Rotate around axis using quaternion
        """
        R = self._get_matrix()
        new_xyz = R.apply(pc.xyz)
        new_data = np.hstack([new_xyz, pc.intensity])
        return PointCloud.from_numpy(
            new_data,
            capacity=new_data.shape[0])


class CloudSubtractor:
    def __init__(self, subtracted: PointCloud, radius: float=0.2, leaf_size: np.uint=10):
        if subtracted.size == 0:
            raise ValueError("Cant use empty point cloud for subtraction.")
        if radius <= 0:
            raise ValueError("Cant use nonpositive KD-tree search radius.")

        self.subtracted_kdtree = BallTree(subtracted.xyz, leaf_size)
        self.radius = radius

    def apply(self, pc: PointCloud):
        if pc.size == 0:
            return pc

        intersection_count = self.subtracted_kdtree.query_radius(
            pc.xyz,
            r=self.radius,
            count_only=True
        )
        new_data = pc.data[intersection_count==0, :]
        return PointCloud.from_numpy(
            new_data,
            capacity=new_data.shape[0]
            )
