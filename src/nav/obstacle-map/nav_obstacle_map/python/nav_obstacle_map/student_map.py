"""学生仅需完成本文件的两个函数；接口和语义见 README。"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class GridSpec:
    resolution: float = 0.1
    width: int = 100
    height: int = 80
    origin_x: float = -1.0
    origin_y: float = -1.0
    z_min: float = 0.15
    z_max: float = 1.5
    inflation_radius: float = 0.3

    def __post_init__(self):
        values = (self.resolution, self.origin_x, self.origin_y,
                  self.z_min, self.z_max, self.inflation_radius)
        if (not all(np.isfinite(v) for v in values) or self.resolution <= 0
                or not isinstance(self.width, int) or not isinstance(self.height, int)
                or self.width <= 0 or self.height <= 0
                or self.width * self.height > 250000
                or self.z_min > self.z_max or self.inflation_radius < 0):
            raise ValueError('invalid grid specification')


def project_cloud(points: np.ndarray, spec: GridSpec) -> np.ndarray:
    """TODO A：N×3 米制世界点云 -> (height,width) uint8，值为 0 或 100。

    过滤非有限坐标，保留 z_min <= z <= z_max。
    左下角原点，floor 索引；地图上、右边界不包含；越界忽略。
    同一列只要存在一个有效点，对应单元即为 100，其他为 0。
    每帧仅含当前雷达扫描；spec 是随机器人移动的局部窗口。
    每帧从零重建，不累积历史、不做衰减或射线遮挡推理。
    """
    grid = np.zeros((spec.height, spec.width), dtype = np.uint8)
    points = np.asarray(points, dtype = np.float64).reshape(-1, 3)

    points = points[np.isfinite(points).all(axis = 1)]
    z = points[:, 2]
    points = points[(z >= spec.z_min) & (z <= spec.z_max)]

    ix = np.floor((points[:, 0] - spec.origin_x) / spec.resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - spec.origin_y) / spec.resolution).astype(np.int64)

    inside = (ix >= 0) & (ix < spec.width) & (iy >= 0) & (iy < spec.height)
    ix = ix[inside]
    iy = iy[inside]

    grid[iy, ix] = 100

    return grid

def inflate_grid(occupied: np.ndarray, spec: GridSpec) -> np.ndarray:
    """TODO B：返回独立副本，不修改输入，shape/dtype/二值编码不变。

    对每个原始占据格，标记中心距离 <= inflation_radius (+1e-9 米容差)
    的其他格为 100，使用欧氏圆盘，边界截断；radius=0 保持输入。
    禁止一边写输出一边把新增占据格当作膨胀种子，否则会连锁填满。
    """
    out = occupied.copy()

    r = spec.inflation_radius
    res = spec.resolution
    h, w = occupied.shape

    dmax = int(np.ceil(r / res))

    offsets = []
    for dy in range(-dmax, dmax + 1):
        for dx in range(-dmax, dmax + 1):
            dist = np.hyhot(dx, dy) * res
            if dist <= r + 1e-9:
                offsets.append((dy,dx))

    iy, ix = np.nonzero(occupied == 100)

    for dy, dx in offsets:
        ny = iy + dy
        nx = ix + dx
        ok = (ny >= 0) & (ny < h) & (nx >= 0) & (nx < w)
        out[ny[ok], nx[ok]] = 100

    return out
    
