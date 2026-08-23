"""
GPS(위경도) <-> 로컬 평면좌표(m) 변환 유틸.

대회 코스는 좁으므로(수백 m 이내) 지구 곡률 무시하고
'기준점(origin) 기준 평면 근사(equirectangular)'로 충분하다.
- x축: East(동), y축: North(북), 단위: meter
- 기준점은 코스에서 처음 저장한 좌표로 잡으면 된다.
"""
import math

# 위도 1도당 거리(m)는 거의 일정. 경도 1도당 거리는 위도에 따라 cos배.
_LAT_M_PER_DEG = 111_320.0


def latlon_to_xy(lat, lon, lat0, lon0):
    """(lat, lon)을 기준점(lat0, lon0)에 대한 로컬 (x=East, y=North) meter로."""
    dlat = lat - lat0
    dlon = lon - lon0
    x = dlon * _LAT_M_PER_DEG * math.cos(math.radians(lat0))  # East
    y = dlat * _LAT_M_PER_DEG                                  # North
    return x, y


def xy_to_latlon(x, y, lat0, lon0):
    """로컬 (x=East, y=North) meter를 다시 (lat, lon)으로 (디버그/저장용)."""
    dlat = y / _LAT_M_PER_DEG
    dlon = x / (_LAT_M_PER_DEG * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def dist_m(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)
