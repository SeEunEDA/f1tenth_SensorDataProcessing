#!/usr/bin/env python3
# ROS2 Foxy - rclpy FTG (Find The Gap)
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def moving_median(x: np.ndarray, k: int) -> np.ndarray:
    if k <= 1 or k % 2 == 0:
        return x
    pad = k // 2
    xp = np.pad(x, (pad, pad), mode='edge')
    out = np.empty_like(x)
    for i in range(len(x)):
        out[i] = np.median(xp[i:i + k])
    return out


def moving_mean(x: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return x
    kernel = np.ones(k, dtype=np.float32) / float(k)
    return np.convolve(x, kernel, mode='same')


def largest_true_run(mask: np.ndarray):
    """True/False 마스크에서 가장 긴 True 구간 [s,e] 반환"""
    best_len, best_s, best_e = 0, None, None
    s = None
    for i, v in enumerate(mask):
        if v and s is None:
            s = i
        if (not v or i == len(mask) - 1) and s is not None:
            e = i if v and i == len(mask) - 1 else i - 1
            L = e - s + 1
            if L > best_len:
                best_len, best_s, best_e = L, s, e
            s = None
    return best_s, best_e


class FTGNode(Node):
    def __init__(self):
        super().__init__('ftg_node')

        # Parameters
        gp = self.declare_parameters('', [
            ('scan_topic', '/scan'),
            ('drive_topic', '/drive'),
            ('cmd_vel_topic', '/cmd_vel'),
            ('frame_id', 'base_link'),
            ('wheelbase', 0.33),
            ('max_steering_deg', 28.0),
            ('fov_deg', 180.0),
            ('clip_min', 0.02),
            ('clip_max', 10.0),
            ('median_k', 5),
            ('mean_k', 1),
            ('downsample', 2),
            ('danger_dist', 0.50),
            ('bubble_radius', 0.30),
            ('bestpoint_smooth_k', 5),
            ('steer_lpf_alpha', 0.6),
            ('max_speed', 2.5),
            ('min_speed', 0.5),
            ('k_steer_speed', 1.0),
            ('k_dist_speed', 1.5),
            ('front_window_deg', 15.0),
            ('publish_ackermann', True),
            ('publish_twist', False),
            ('publish_markers', True),
        ])
        self.p = {p.name: p.value for p in gp}

        # Publishers/Subscribers
        self.pub_drive = self.create_publisher(AckermannDriveStamped, self.p['drive_topic'], 10)
        self.pub_twist = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'ftg/markers', 10)

        self.sub_scan = self.create_subscription(LaserScan, self.p['scan_topic'], self.lidar_callback, 10)

        self.prev_angle = 0.0
        self.get_logger().info(f"FTG ready. scan={self.p['scan_topic']} → drive/cmd_vel")

    # ===================== 핵심 알고리즘 함수 3종 =====================

    def preprocess_lidar(self, scan: LaserScan):
        """
        (개념) LiDAR 전처리:
          1) NaN/Inf 치환 + [clip_min, clip_max] 클리핑
          2) 전방 FOV 제한(±fov/2)
          3) median/mean smoothing
          4) downsample
        반환: (rng[float32], ang[float32])
        """
        rmin, rmax = scan.range_min, scan.range_max
        ang_min, ang_inc = scan.angle_min, scan.angle_increment
        n = len(scan.ranges)

        rng = np.asarray(scan.ranges, dtype=np.float32)
        rng = np.where(np.isfinite(rng), rng, rmax)
        rng = np.clip(rng, max(self.p['clip_min'], rmin), min(self.p['clip_max'], rmax))

        # 전방 FOV 슬라이스
        fov = math.radians(self.p['fov_deg'])
        des_min = -fov / 2.0
        des_max = +fov / 2.0
        idx_s = max(0, int(math.floor((des_min - ang_min) / ang_inc)))
        idx_e = min(n - 1, int(math.ceil((des_max - ang_min) / ang_inc)))
        rng = rng[idx_s:idx_e + 1]
        angs = ang_min + ang_inc * np.arange(idx_s, idx_e + 1, dtype=np.float32)

        # smoothing
        median_k = int(self.p['median_k'])
        mean_k = int(self.p['mean_k'])
        if median_k > 1 and median_k % 2 == 1:
            rng = moving_median(rng, median_k)
        if mean_k > 1:
            rng = moving_mean(rng, mean_k)

        # downsample
        step = max(1, int(self.p['downsample']))
        rng = rng[::step]
        angs = angs[::step]

        return rng, angs, ang_inc * step

    def find_max_gap(self, rng: np.ndarray, angs: np.ndarray, ang_inc_eff: float):
        """
        (개념) 가장 큰 '빈 공간' 구간 찾기:
          - 위험거리 이하(False), 안전거리 초과(True) 마스크 구성
          - 최근접 물체 주변에 '버블'을 각도폭으로 확장하여 False
          - True가 연속된 가장 긴 구간을 반환
        """
        danger = float(self.p['danger_dist'])
        free_mask = rng > danger

        if len(rng) == 0:
            return None, None, free_mask, None

        # 최근접물체 + 버블 확장
        i_close = int(np.argmin(rng))
        d_close = float(rng[i_close])
        if d_close > 1e-3:
            bubble_r = float(self.p['bubble_radius'])
            ang_width = math.atan2(bubble_r, d_close)  # 각도 폭
            nb = int(math.ceil(ang_width / max(ang_inc_eff, 1e-6)))
            L = max(0, i_close - nb)
            R = min(len(free_mask) - 1, i_close + nb)
            free_mask[L:R + 1] = False

        s, e = largest_true_run(free_mask)
        return s, e, free_mask, i_close

    def find_best_point(self, s: int, e: int, rng: np.ndarray, angs: np.ndarray):
        """
        (개념) 갭 내부 최적 목표점 선택:
          - 갭 내부 거리 곡선을 이동평균으로 스무딩
          - 최대값(가장 멀리) 위치를 '베스트 포인트'로 선택
        """
        if s is None:
            return None, None
        gap_rng = rng[s:e + 1]
        gap_ang = angs[s:e + 1]
        k = int(self.p['bestpoint_smooth_k'])
        sm = moving_mean(gap_rng, k) if k > 1 else gap_rng
        ib = int(np.argmax(sm))
        return float(gap_rng[ib]), float(gap_ang[ib])

    # ========================== 콜백/출력 ===========================

    def lidar_callback(self, scan: LaserScan):
        rng, angs, ang_inc_eff = self.preprocess_lidar(scan)
        s, e, free_mask, i_close = self.find_max_gap(rng, angs, ang_inc_eff)
        best_r, best_ang = self.find_best_point(s, e, rng, angs)

        # 조향각 LPF + 제한
        if best_ang is None:
            target_angle = 0.0
        else:
            alpha = float(self.p['steer_lpf_alpha'])
            target_angle = alpha * best_ang + (1.0 - alpha) * self.prev_angle
        self.prev_angle = target_angle

        max_deg = math.radians(float(self.p['max_steering_deg']))
        target_angle = float(np.clip(target_angle, -max_deg, +max_deg))

        # 속도 계획 (조향/전방거리 기반)
        max_speed = float(self.p['max_speed'])
        min_speed = float(self.p['min_speed'])
        k_steer = float(self.p['k_steer_speed'])
        v_by_steer = max_speed * max(0.0, 1.0 - k_steer * abs(target_angle) / max(max_deg, 1e-6))

        front_win = math.radians(float(self.p['front_window_deg']))
        front_mask = np.abs(angs) < front_win
        front_min = float(np.min(rng[front_mask])) if np.any(front_mask) else float(np.min(rng)) if len(rng) else 0.0
        k_dist = float(self.p['k_dist_speed'])
        v_by_dist = min(max_speed, k_dist * max(0.0, front_min - float(self.p['danger_dist'])))

        target_speed = float(np.clip(min(v_by_steer, v_by_dist), min_speed, max_speed))

        # 출력
        self.publish_cmd(target_angle, target_speed)

        # RViz 마커
        if self.p['publish_markers']:
            self.publish_markers(scan.header, angs, rng, free_mask, s, e, target_angle,
                                 (best_r, best_ang) if best_ang is not None else None)

    def publish_cmd(self, steer_rad: float, speed: float):
        if self.p['publish_ackermann']:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p['frame_id']
            msg.drive.steering_angle = float(steer_rad)  # [rad]
            msg.drive.speed = float(speed)               # [m/s]
            self.pub_drive.publish(msg)

        if self.p['publish_twist']:
            tw = Twist()
            tw.linear.x = float(speed)
            L = float(self.p['wheelbase'])
            tw.angular.z = float(speed) * math.tan(float(steer_rad)) / max(L, 1e-6)
            self.pub_twist.publish(tw)

    def publish_markers(self, header, angs, rng, free_mask, s, e, target_angle, best_polar):
        arr = MarkerArray()

        def color(r, g, b, a=0.9):
            c = ColorRGBA(); c.r, c.g, c.b, c.a = r, g, b, a; return c

        # 선택 조향 화살표
        m_arrow = Marker()
        m_arrow.header = header
        m_arrow.ns = 'ftg'
        m_arrow.id = 1
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.scale.x = 0.6
        m_arrow.scale.y = 0.05
        m_arrow.scale.z = 0.05
        m_arrow.color = color(0.1, 0.8, 0.1)
        m_arrow.pose.orientation.z = math.sin(target_angle/2.0)
        m_arrow.pose.orientation.w = math.cos(target_angle/2.0)
        arr.markers.append(m_arrow)

        # 갭(초록)
        m_gap = Marker(); m_gap.header=header; m_gap.ns='ftg'; m_gap.id=2
        m_gap.type = Marker.POINTS; m_gap.scale.x=m_gap.scale.y=0.03
        m_gap.color = color(0.2, 1.0, 0.2)

        def pol2pt(r,a):
            from geometry_msgs.msg import Point
            p=Point(); p.x=float(r*math.cos(a)); p.y=float(r*math.sin(a)); p.z=0.0; return p

        if s is not None:
            for i in range(s, e+1):
                m_gap.points.append(pol2pt(rng[i], angs[i]))
        arr.markers.append(m_gap)

        # 차단(빨강)
        m_block = Marker(); m_block.header=header; m_block.ns='ftg'; m_block.id=3
        m_block.type=Marker.POINTS; m_block.scale.x=m_block.scale.y=0.02
        m_block.color = color(1.0, 0.2, 0.2)
        for i, ok in enumerate(free_mask):
            if not ok:
                m_block.points.append(pol2pt(rng[i], angs[i]))
        arr.markers.append(m_block)

        # 베스트 포인트(파랑)
        if best_polar is not None:
            br, ba = best_polar
            m_best = Marker(); m_best.header=header; m_best.ns='ftg'; m_best.id=4
            m_best.type=Marker.SPHERE; m_best.scale.x=m_best.scale.y=m_best.scale.z=0.08
            m_best.color = color(0.2, 0.4, 1.0)
            from geometry_msgs.msg import Point
            m_best.pose.position = pol2pt(br, ba)
            arr.markers.append(m_best)

        self.pub_markers.publish(arr)


def main():
    rclpy.init()
    node = FTGNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
