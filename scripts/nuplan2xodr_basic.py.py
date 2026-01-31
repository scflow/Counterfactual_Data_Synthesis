import geopandas as gpd
import numpy as np
from shapely.geometry import LineString
from lxml import etree
import math
import pyproj

class NuPlanToOpenDRIVE_V4:
    def __init__(self, gpkg_path):
        self.gpkg_path = gpkg_path
        self.root = None
        self.transformer = None
        # 记录关键的投影参数
        self.utm_zone = None
        self.is_northern = True
        self.offset_x = 0.0
        self.offset_y = 0.0
        
    def init_projection(self, lat, lon):
        """
        根据经纬度确定 UTM 带号，并初始化投影器
        """
        self.utm_zone = int((lon + 180) / 6) + 1
        self.is_northern = lat >= 0
        
        crs_wgs84 = pyproj.CRS("EPSG:4326")
        # 标准 UTM 投影
        crs_utm = pyproj.CRS(proj='utm', zone=self.utm_zone, ellps='WGS84', 
                             north=self.is_northern, units='m')
        
        self.transformer = pyproj.Transformer.from_crs(crs_wgs84, crs_utm, always_xy=True)
        print(f"坐标系统: UTM Zone {self.utm_zone} {'N' if self.is_northern else 'S'}")

    def get_georeference_string(self):
        """
        【核心】生成 OpenDRIVE 的 geoReference 字符串
        利用 PROJ 格式，通过修改 +x_0 和 +y_0 (False Easting/Northing) 
        来抵消我们在脚本里做的平移操作。
        """
        # 标准 UTM 的 False Easting 是 500,000米
        # 标准 UTM 的 False Northing 是 0 (北半球) 或 10,000,000 (南半球)
        std_false_easting = 500000.0
        std_false_northing = 0.0 if self.is_northern else 10000000.0
        
        # 我们新的“伪原点”需要抵消掉 offset
        # 公式：New_FE = Std_FE - Offset_X
        new_false_easting = std_false_easting - self.offset_x
        new_false_northing = std_false_northing - self.offset_y
        
        # 构造 PROJ4 字符串
        # 这一行字告诉 GIS 软件：如何把 xodr 里的 (0,0) 映射回经纬度
        proj_str = (f"+proj=utm +zone={self.utm_zone} +ellps=WGS84 +datum=WGS84 "
                    f"+units=m +x_0={new_false_easting:.4f} +y_0={new_false_northing:.4f} +no_defs")
        
        return proj_str

    def read_data(self):
        print(f"正在读取: {self.gpkg_path} ...")
        layers_to_try = ['main.baseline_paths', 'baseline_paths', 'lanes_polygons']
        for layer in layers_to_try:
            try:
                self.gdf = gpd.read_file(self.gpkg_path, layer=layer)
                break
            except:
                continue
                
        if not hasattr(self, 'gdf') or self.gdf.empty:
            raise ValueError("未找到有效数据图层")

        # 检查坐标并初始化投影
        first_geom = self.gdf.geometry.iloc[0]
        p_sample = first_geom.coords[0] if first_geom.geom_type == 'LineString' else first_geom.exterior.coords[0]
        
        if -180 <= p_sample[0] <= 180:
            self.init_projection(p_sample[1], p_sample[0])
            self.needs_projection = True
        else:
            print("警告：原始数据似乎不是经纬度，无法自动生成对齐的 geoReference。")
            self.needs_projection = False

    def create_header(self):
        self.root = etree.Element("OpenDRIVE")
        header = etree.SubElement(self.root, "header")
        header.set("revMajor", "1")
        header.set("revMinor", "4")
        header.set("name", "NuPlan_Georeferenced")
        header.set("version", "1.0")
        
        # 【新增】写入地理参考信息
        if self.needs_projection:
            geo_ref = etree.SubElement(header, "geoReference")
            # 写入 CDATA 格式的 PROJ 字符串
            geo_ref.text = etree.CDATA(self.get_georeference_string())

    def convert(self):
        # 1. 预处理：计算 Offset
        if self.needs_projection:
            # 取第一条线的起点作为基准
            first_row = self.gdf.iloc[0].geometry
            p_start = first_row.coords[0] if first_row.geom_type == 'LineString' else first_row.exterior.coords[0]
            
            # 计算其 UTM 坐标
            utm_x, utm_y = self.transformer.transform(p_start[0], p_start[1])
            
            # 记录这个偏移量
            self.offset_x = utm_x
            self.offset_y = utm_y
            print(f"计算出基准偏移量: X={self.offset_x:.2f}, Y={self.offset_y:.2f}")
        
        # 2. 创建头部 (包含 GeoReference)
        self.create_header()
        
        road_id = 1
        
        for index, row in self.gdf.iterrows():
            geom = row.geometry
            if geom.geom_type != 'LineString': continue
            
            raw_points = list(geom.coords)
            clean_points = []
            
            # 坐标转换 + 平移 + 清洗
            prev_p = None
            for p in raw_points:
                # 转 UTM
                px, py = p[0], p[1]
                if self.needs_projection:
                    px, py = self.transformer.transform(px, py)
                
                # 减 Offset (核心步骤)
                px -= self.offset_x
                py -= self.offset_y
                
                curr_p = (px, py)
                
                # 距离过滤 (防止点重合)
                if prev_p:
                    dist = math.sqrt((px-prev_p[0])**2 + (py-prev_p[1])**2)
                    if dist < 0.02: continue 
                
                clean_points.append(curr_p)
                prev_p = curr_p
                
            if len(clean_points) < 2: continue

            # 构建 Road XML (同 V3)
            road = etree.SubElement(self.root, "road")
            road.set("name", f"Road_{index}")
            road.set("id", str(road_id))
            road.set("junction", "-1")
            
            etree.SubElement(road, "link")
            plan_view = etree.SubElement(road, "planView")
            
            s_cursor = 0.0
            for i in range(len(clean_points) - 1):
                p1 = clean_points[i]
                p2 = clean_points[i+1]
                length = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
                hdg = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                
                geo = etree.SubElement(plan_view, "geometry")
                geo.set("s", f"{s_cursor:.4f}")
                geo.set("x", f"{p1[0]:.4f}")
                geo.set("y", f"{p1[1]:.4f}")
                geo.set("hdg", f"{hdg:.6f}")
                geo.set("length", f"{length:.4f}")
                etree.SubElement(geo, "line")
                s_cursor += length
            
            road.set("length", f"{s_cursor:.4f}")
            self._create_simple_lanes(road) # 简化的车道函数
            road_id += 1

    def _create_simple_lanes(self, road):
        # 简化的车道生成代码，为了节省篇幅合并写在这里
        lanes = etree.SubElement(road, "lanes")
        ls = etree.SubElement(lanes, "laneSection", s="0.0")
        etree.SubElement(etree.SubElement(ls, "center"), "lane", id="0", type="none", level="false")
        right = etree.SubElement(ls, "right")
        l = etree.SubElement(right, "lane", id="-1", type="driving", level="false")
        etree.SubElement(l, "width", sOffset="0.0", a="3.0", b="0", c="0", d="0")

    def save(self, output_path):
        tree = etree.ElementTree(self.root)
        tree.write(output_path, pretty_print=True, xml_declaration=True, encoding="UTF-8")
        print(f"转换完成: {output_path}")

if __name__ == "__main__":
    converter = NuPlanToOpenDRIVE_V4("data/maps/us-ma-boston/9.12.1817/map.gpkg") # 替换你的文件
    try:
        converter.read_data()
        converter.convert()
        converter.save("us-ma-boston.xodr")
    except Exception as e:
        print(e)