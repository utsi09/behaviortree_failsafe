#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from carla_msgs.msg import CarlaEgoVehicleStatus
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from warning_mode_interfaces.action import WarningMode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import carla
import time
import math
import threading
import asyncio
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

LIDAR_TIMEOUT = 0.5    # 무신호 감지 임계 (초)
CHECK_PERIOD  = 0.1    # 타임아웃 검사 주기 (초)
PUBLISH_RATE  = 10.0   # 제어용 Python API 호출 주기 (Hz)

# 액션 라이브러리 사용해서 behavior Tree로 부터 액션 goal을 받으면 (0, 저속운전 , 1. 갓길 이동 , 2. 차선 평행 회전 , 3. 핸드파킹)

### 위험도 파라미터 #######
K = 3.0 #P에 대한 가중치 ##
lamb = 0.7   # λ      ##
TH = 100              ##
########################

def force_all_traffic_lights_green(client):
    world = client.get_world()
    lights = world.get_actors().filter("traffic.traffic_light")

    for light in lights:
        light.set_state(carla.TrafficLightState.Green)
        light.set_green_time(9999.0)
        light.freeze(True)
        print(f"신호등 {light.id} → 초록불 고정")


def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

class LidarFailSafe(Node):
    def __init__(self):
        super().__init__('lidar_failsafe')
        
        # 현재 액션 모드를 저장할 변수 추가
        self.current_mode = -1  # 초기값 -1 (비활성 상태)

        self.action_server = ActionServer( #액션서버 정의
            self,
            WarningMode,
            'warning_mode',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # 액션 실행 상태 플래그
        self.action_running = False
        self.action_future = None
        self.action_goal_handle = None

        # ① /lidar_alive, /risk_level 퍼블리셔 추가
        self.alive_pub = self.create_publisher(Bool, '/lidar_alive', 10)
        self.risk_pub = self.create_publisher(Float64,'/risk_level',10)
        # ② ROS: Lidar 구독
        self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.lidar_cb,
            10)

        # ③ ROS: 차량 속도(Status) 구독
        self.vehicle_speed = 0.0
        self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/hero/vehicle_status',
            self.status_cb,
            10)

        # CARLA Python API 연결
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        force_all_traffic_lights_green(self.client) #강제 초록불

        # HERO 차량 찾기 및 Autopilot 비활성
        self.hero = None
        for v in self.world.get_actors().filter('vehicle.*'):
            print(v.id, v.attributes.get('role_name'))
            if v.attributes.get('role_name') == 'hero':
                self.get_logger().info(f"[DEBUG] 차량 ID={v.id}, role_name={v.attributes.get('role_name')}")
                self.hero = v
                #self.hero.set_autopilot(False) #emp
                break
        if not self.hero:
            self.get_logger().error('Hero 차량을 찾을 수 없습니다!')

        # 상태 변수
        self.last_stamp = time.time()
        self.in_fail    = False
        self.current_risk = 0.0
        self.has_parked = False

        # 타이머 설정
        self.create_timer(CHECK_PERIOD, self.check_timeout)
        self.create_timer(1.0 / PUBLISH_RATE, self.publish_ctrl) # 모드 확인해서 실행
        self.create_timer(0.1, self.publish_risk)
        self.create_timer(1.0, self.next_line)

        # 초기화 - waypoint와 차선 정보
        self.waypoint = None
        self.right_lane_marking = None
        self.left_lane_marking = None


    ## 액션 서버 콜백 함수 추가
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: mode={goal_request.mode}')
        if self.current_mode == goal_request.mode and self.action_running:
            self.get_logger().info('중복 goal 거절')
            return GoalResponse.REJECT
        # 액션 모드 저장 - 목표 수락 시 현재 모드 설정
        self.current_mode = goal_request.mode
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.current_mode = -1
        self.has_parked = False
        self.in_fail = False
        if self.hero:
            self.hero.set_autopilot(True)
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        mode = goal_handle.request.mode
        self.get_logger().info(f'Executing goal: mode {mode}')
        
        # if self.current_risk < 4: #risk 정상 복귀시 모드 종료
        #     # 액션 종료 처리
        #     goal_handle.succeed()
        #     # 액션 완료 후 모드 초기화
        #     self.current_mode = -1
        #     result = WarningMode.Result()
        #     result.success = True
        #     return result

        ##########################  mode 0  ##########################
        if mode == 0:
            try:
                start = time.time()
                while rclpy.ok():
                    if not self.hero:
                        raise RuntimeError('hero is None')
                    self.hero.apply_control(carla.VehicleControl(throttle=0.3))
                    await asyncio.sleep(0.5)
            except Exception as e:
                self.get_logger().error(str(e))
                goal_handle.abort()
                return WarningMode.Result(success=False)

            self.current_mode = -1          # <– 추가
            goal_handle.succeed()
            return WarningMode.Result(success=True)

                
        # elif mode == 1:
        #     self.get_logger().info('모드 1: 갓길 이동 모드 시작')
        #     start = time.time()

        #     while time.time() - start < 30.0 and rclpy.ok() and not self.has_parked:
        #         await asyncio.sleep(0.5)

        #     result = WarningMode.Result()
        #     print(self.has_parked) 
        #     if self.has_parked:
        #         self.get_logger().info('갓길 주차 완료 → 액션 성공')
        #         goal_handle.succeed()
        #         result.success = True
        #     else:
        #         self.get_logger().warn('30초 내 주차 실패 → 액션 실패')
        #         goal_handle.abort()
        #         result.success = False

        #     return result
        ##########################  mode 1  ##########################
        if mode == 1:
            while rclpy.ok() and not self.has_parked:
                await asyncio.sleep(0.5)

            result = WarningMode.Result()
            if self.has_parked:
                self.current_mode = -1
                goal_handle.succeed()
                result.success = True
            # else:
            #     self.current_mode = -1
            #     goal_handle.abort()
            #     result.success = False
            
            return result


        
        # 액션 종료 처리
        goal_handle.succeed()
        
        # 액션 완료 후 모드 초기화
        self.current_mode = -1
        self.action_running = False   
        result = WarningMode.Result()
        result.success = True
        return result

    #################################################################################################################

    def get_lane_lotation(self):
        if self.waypoint:
            self.lane_yaw = self.waypoint.transform.rotation.yaw #차 yaw와 오른쪽 차선 yaw 일치시키기
        

    def next_line(self):
        if self.hero:
            self.waypoint = self.world.get_map().get_waypoint(self.hero.get_location(), project_to_road=True, lane_type=carla.LaneType.Any)
            self.right_lane_marking = self.waypoint.right_lane_marking
            self.left_lane_marking = self.waypoint.left_lane_marking
            self.get_logger().info(f"왼쪽 차선: {self.left_lane_marking.type}, 오른쪽 차선: {self.right_lane_marking.type}")


    def publish_risk(self): # 위험도 토픽 퍼블리시 함수
        risk_msg = Float64()
        risk_msg.data = self.current_risk
        self.risk_pub.publish(risk_msg)

    def lidar_cb(self, msg):
        # 라이다 메시지 수신 시점 갱신
        self.last_stamp = time.time() #받았을때 시간을 객체에 저장

        # alive 토픽에 True 발행
        alive_msg = Bool()
        alive_msg.data = True
        self.alive_pub.publish(alive_msg)

        # 만약 이전에 실패 상태였다면 복구 처리
        if self.in_fail:
            self.get_logger().info('Lidar 복구 — 정상 주행으로 전환')
            self.in_fail = False
            self.has_parked = False
            if self.hero:
                self.hero.set_autopilot(True)
            # 모드 초기화
            self.current_mode = -1

    def status_cb(self, msg):
        self.vehicle_speed = msg.velocity
        #self.get_logger().info(f'현재 속도: {self.vehicle_speed:.2f} m/s')

    def check_timeout(self):
        t = time.time() - self.last_stamp #(현재 시간 - 최근 수신 시간)
        alive = (t < LIDAR_TIMEOUT)
        
        self.current_risk = K * math.exp(lamb*t) # 위험도 계산 (라이다만 고려)
        self.get_logger().info(f'현재 위험도: {self.current_risk}')
        # ─── /lidar_alive 퍼블리시 ───
        alive_msg = Bool()
        alive_msg.data = alive
        self.alive_pub.publish(alive_msg)

        # ─── 무응답 타임아웃 진입 ───
        if not self.in_fail and self.current_risk > TH:
            self.get_logger().warn(f'위험도 초과 {self.current_risk} — 급정지 모드')
            self.in_fail = True
            if self.hero:
                self.hero.set_autopilot(False)

    def publish_ctrl(self):
        # 모드가 1일 때만 실행 (갓길 이동 모드)
        if self.current_mode == 1:
            if not self.in_fail or not self.hero:
                return
                
            if self.has_parked:
                return

            if not self.waypoint or not self.left_lane_marking or not self.right_lane_marking:
                self.next_line()  # waypoint가 없으면 업데이트
                return
                
            left_type = self.left_lane_marking.type
            right_type = self.right_lane_marking.type

            # 왼쪽 Solid + 오른쪽 None → 평행 맞추고 정지
            if left_type == carla.LaneMarkingType.Solid and right_type == carla.LaneMarkingType.NONE:
                # 차량과 차선의 yaw 차이 계산
                hero_yaw = self.hero.get_transform().rotation.yaw
                lane_yaw = self.waypoint.transform.rotation.yaw
                angle_diff = abs(normalize_angle(hero_yaw - lane_yaw))

                # yaw 차이가 크면 조향 보정
                if angle_diff > 3.0:
                    steer = max(-1.0, min(1.0, normalize_angle(lane_yaw - hero_yaw) / 45.0))
                    ctrl = carla.VehicleControl(throttle=0.2, steer=steer, brake=0.0)
                    self.hero.apply_control(ctrl)
                    self.get_logger().info(f"▶ 평행 맞추는 중 (angle_diff={angle_diff:.2f})")
                    return

                # yaw 일치하면 정지
                ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True)
                self.hero.apply_control(ctrl)
                self.has_parked = True
                self.get_logger().info("▶▶▶ 주차 조건 + 방향 일치 → 차량 정지 및 핸드브레이크")
                return

            # 아직 주차 조건 미달 → 우측 이동 계속
            ctrl = carla.VehicleControl(throttle=0.3, steer=0.1, brake=0.0)
            self.hero.apply_control(ctrl)
            self.get_logger().info("▶ 갓길 탐색 중: 우측으로 이동")


def main():
    rclpy.init()
    node = LidarFailSafe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()