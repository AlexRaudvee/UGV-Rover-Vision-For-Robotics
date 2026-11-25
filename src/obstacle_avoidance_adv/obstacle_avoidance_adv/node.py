"""LiDAR-first avoidance with gyro turns and forward-speed mission odometry."""
from collections import deque
from dataclasses import fields
import json
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry,Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from .planner import Controller,Settings,Scan,View,wrap,rotation
from .motion import MotionLimiter


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_adv')
        defaults=dict(vars(Settings()),scan_topic='/scan',odom_topic='/odom',imu_topic='/imu/estimation',
                      detection_topic='/detection_improved/obstacle/detections',
                      command_topic='/obstacle_avoidance_adv/cmd_vel',base_frame='base_footprint',
                      input_timeout=1.5,depth_timeout=3.,scan_memory_s=.30,control_rate=20.,
                      linear_acceleration=.5,resume_clear_time=.2)
        for key,value in defaults.items():self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.value=lambda key:self.get_parameter(key).value
        for key in ('input_timeout','depth_timeout','scan_memory_s','control_rate'):
            if not math.isfinite(self.value(key)) or self.value(key)<=0:raise ValueError(key+' must be finite and positive')
        self.controller=Controller(Settings(**{f.name:self.value(f.name) for f in fields(Settings)}))
        self.motion=MotionLimiter(self.value('linear_acceleration'),self.value('angular_acceleration'),self.value('resume_clear_time'))
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pose=np.zeros(3);self.history=deque(maxlen=600);self.scans=deque(maxlen=5)
        self.latest_scan=self.latest_odom=None;self.depth=None;self.scan_token=None
        self.gyro_stamp=None;self.gyro_rate=0.;self.gyro_angle=0.;self.gyro_origin=0.;self.gyro_bias=0.
        self.bias_samples=deque(maxlen=40);self.received={};self.odom_stamp=None;self.velocity=0.
        self.wheel_v=0.;self.wheel_received=-math.inf;self.speed_source='waiting'
        self.global_origin=np.zeros(3);self.last_sent=(0.,0.);self.last_status=0.;self.hold_since=None
        self.readiness='Waiting for LiDAR, gyro and odometry';self.view=None
        self.direction_mismatch_since=None
        self.command=self.create_publisher(Twist,self.value('command_topic'),1)
        self.status=self.create_publisher(String,'~/status',1);self.path=self.create_publisher(Path,'~/planned_path',1)
        self.create_subscription(LaserScan,self.value('scan_topic'),self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Imu,self.value('imu_topic'),self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,self.value('odom_topic'),self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom/wheels',self.on_wheels,qos_profile_sensor_data)
        self.create_subscription(String,self.value('detection_topic'),self.on_depth,1)
        self.create_service(Trigger,'~/start',self.start);self.create_service(Trigger,'~/stop',self.stop)
        self.create_timer(1/self.value('control_rate'),self.tick)
        self.get_logger().info('IDLE: LiDAR/gyro manoeuvres; no cube fit or depth-validity gate. Start service required.')

    def now(self):return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def on_scan(self,msg):
        if self.latest_scan is not None and self.stamp(msg)<=self.stamp(self.latest_scan):return
        self.latest_scan=msg;self.received['scan']=self.now()

    def on_wheels(self,msg):
        v=float(msg.twist.twist.linear.x)
        if math.isfinite(v) and abs(v)<=1.6:self.wheel_v=v;self.wheel_received=self.now()

    def on_imu(self,msg):
        try:
            tf=self.buffer.lookup_transform(self.value('base_frame'),msg.header.frame_id,Time())
            v=msg.angular_velocity
            rate=float(np.diff(transform_points(np.array([[0.,0.,0.],[v.x,v.y,v.z]]),tf.transform),axis=0)[0,2])
            stamp=self.stamp(msg)
            if not math.isfinite(rate) or abs(rate)>4:return
            if self.gyro_stamp is not None and stamp<=self.gyro_stamp:return
            if not self.controller.active and abs(self.wheel_v)<.01 and abs(self.velocity)<.02 and abs(rate)<.03:
                self.bias_samples.append(rate);self.gyro_bias=float(np.median(self.bias_samples))
            corrected=rate-self.gyro_bias
            if self.gyro_stamp is not None and 0<stamp-self.gyro_stamp<=self.value('input_timeout'):
                self.gyro_angle+=(self.gyro_rate+corrected)*.5*(stamp-self.gyro_stamp)
            self.gyro_stamp=stamp;self.gyro_rate=corrected;self.received['gyro']=self.now()
            if self.controller.active:self.pose[2]=wrap(self.gyro_angle-self.gyro_origin)
        except (TransformException,ValueError):pass

    def on_odom(self,msg):
        stamp=self.stamp(msg);v=float(msg.twist.twist.linear.x)
        if not math.isfinite(v) or abs(v)>1.6:return
        if self.odom_stamp is not None and stamp<=self.odom_stamp:return
        self.latest_odom=msg;self.received['odom']=self.now();self.speed_source='fused_forward_velocity'
        if self.now()-self.wheel_received<=self.value('input_timeout'):
            if abs(self.wheel_v)<.012:v=0.;self.speed_source='wheel_stationary'
            elif abs(v-self.wheel_v)>.25+.3*abs(self.wheel_v):v=self.wheel_v;self.speed_source='wheel_fallback'
        if self.controller.active and self.odom_stamp is not None:
            dt=stamp-self.odom_stamp
            if 0<dt<=self.value('input_timeout'):
                self.pose[:2]+=(v+self.velocity)*.5*dt*np.array([math.cos(self.pose[2]),math.sin(self.pose[2])])
        self.odom_stamp=stamp;self.velocity=v

    def on_depth(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get('fresh') and value.get('schema_version')==1:self.depth=value
            else:self.depth=None
        except (ValueError,TypeError):self.depth=None

    def pose_at(self,stamp):
        if not self.history:return self.pose.copy()
        times=np.array([t for t,p in self.history]);poses=np.array([p for t,p in self.history])
        if stamp<times[0]:return poses[0].copy()
        return np.array([np.interp(stamp,times,poses[:,0]),np.interp(stamp,times,poses[:,1]),
                         np.interp(stamp,times,np.unwrap(poses[:,2]))])

    def depth_points(self):
        data=self.depth
        if data is None:return np.empty((0,2))
        try:
            stamp=float(data['stamp']['sec'])+float(data['stamp']['nanosec'])/1e9
            if not -.1<=self.now()-stamp<=self.value('depth_timeout'):return np.empty((0,2))
            tf=self.buffer.lookup_transform(self.value('base_frame'),data['frame_id'],Time())
            points=[]
            for item in data.get('blockers',[]):
                distance=float(item['distance_m']);low=float(item['angle_min']);high=float(item['angle_max'])
                if not all(math.isfinite(v) for v in (distance,low,high)) or not .1<distance<10 or not -1.5<low<=high<1.5:continue
                z=distance*math.cos(float(item['bearing']));angles=np.linspace(low,high,9)
                local=np.column_stack((-z*np.tan(angles),np.zeros(9),np.full(9,z)))
                points.append(transform_points(local,tf.transform)[:,:2])
            if not points:return np.empty((0,2))
            pose=self.pose_at(stamp)
            return np.vstack(points)@rotation(pose[2]).T+pose[:2]
        except (KeyError,ValueError,TypeError,TransformException):return np.empty((0,2))

    def prepare(self):
        now=self.now();timeout=self.value('input_timeout')
        for key in ('scan','gyro','odom'):
            if now-self.received.get(key,-math.inf)>timeout:raise ValueError(key+' missing/stale')
        for key,stamp in (('scan',self.stamp(self.latest_scan)),('gyro',self.gyro_stamp),('odom',self.odom_stamp)):
            if stamp is None or not -.1<=now-stamp<=timeout:raise ValueError(key+' acquisition missing/stale')
        if self.count_publishers(self.value('command_topic'))!=1:raise ValueError('Another velocity publisher owns the command topic')
        msg=self.latest_scan;stamp=self.stamp(msg);token=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if token!=self.scan_token:
            tf=self.buffer.lookup_transform(self.value('base_frame'),msg.header.frame_id,Time())
            previous=self.scans[-1] if self.scans and stamp-self.scans[-1].stamp<.5 else None
            cfg=self.controller.cfg
            scan=Scan(msg,tf.transform,self.pose_at(stamp),stamp,previous,cfg.half_length,cfg.half_width)
            if np.isfinite(scan.ranges).sum()<3:raise ValueError('LiDAR has insufficient observed rays')
            self.scans.append(scan);self.scan_token=token
        scans=[s for s in self.scans if stamp-s.stamp<=self.value('scan_memory_s')]
        self.view=View(scans,self.pose,self.depth_points(),self.controller.cfg,now-stamp)
        return self.view

    def start(self,request,response):
        if self.controller.active:
            response.success=False;response.message='Mission already active';return response
        try:
            self.prepare()
            if abs(self.velocity)>.03 or abs(self.gyro_rate)>.08:raise ValueError('Stop the rover before capturing the route origin')
            self.pose=np.zeros(3);self.gyro_origin=self.gyro_angle;self.history.clear();self.scans.clear();self.scan_token=None
            p=self.latest_odom.pose.pose;self.global_origin=np.array([p.position.x,p.position.y,yaw(p.orientation)])
            self.history.append((self.now(),self.pose.copy()));self.controller.start(self.pose,time.monotonic())
            self.motion.reset();self.hold_since=None
            self.direction_mismatch_since=None
            response.success=True;response.message='Mission started; gyro quarter turns; output: '+self.value('command_topic')
            if self.value('command_topic')!='/cmd_vel':response.message+=' (preview only)'
        except (ValueError,TransformException) as error:
            response.success=False;response.message=str(error);self.controller.reason='Start rejected: '+str(error)
        return response

    def stop(self,request,response):
        self.controller.stop();self.motion.reset();self.send(0.,0.)
        response.success=True;response.message='Mission stopped';return response

    def send(self,v,w):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.command.publish(msg)
        self.last_sent=(float(v),float(w))

    def tick(self):
        now=time.monotonic();self.history.append((self.now(),self.pose.copy()))
        active=self.controller.active
        try:
            view=self.prepare();self.readiness='Ready';self.hold_since=None
            if active:
                v,w=self.controller.step(self.pose,view,now,self.gyro_rate,self.velocity)
                mismatch=(abs(self.last_sent[1])>.15 and self.gyro_rate*self.last_sent[1]<-.015)
                if mismatch:
                    if self.direction_mismatch_since is None:self.direction_mismatch_since=now
                    if now-self.direction_mismatch_since>.5:v,w=self.controller.fail('Commanded turn opposes physical gyro rotation; check motor/IMU sign')
                else:self.direction_mismatch_since=None
                if now-self.controller.progress_at>20 and max(abs(x) for x in self.last_sent)>.1:
                    v,w=self.controller.fail('Commanded motion made no measured progress')
                self.send(*self.motion.update(v,w,now))
        except (ValueError,TransformException) as error:
            self.readiness=str(error)
            if active:
                self.send(0.,0.);self.motion.reset();self.controller.reason='Input hold: '+str(error)
                if self.hold_since is None:self.hold_since=now
                if 'publisher' in str(error) or now-self.hold_since>5.:self.controller.fail(str(error))
        if now-self.last_status>.5:
            self.last_status=now
            self.status.publish(String(data=json.dumps(dict(state=self.controller.state,reason=self.controller.reason,
                readiness=self.readiness,side='right' if self.controller.side<0 else 'left' if self.controller.side>0 else None,
                passed_obstacles=self.controller.passes,command_topic=self.value('command_topic'),
                command_velocity=list(self.last_sent),motion_status=self.motion.status,route_position_m=self.pose[:2].tolist(),
                heading_deg=math.degrees(self.pose[2]),yaw_rate=self.gyro_rate,gyro_bias=self.gyro_bias,
                localization='gyro_heading_and_forward_velocity',speed_source=self.speed_source,
                front_clearance_m=None if self.view is None or not math.isfinite(self.view.clearance()) else self.view.clearance(),
                lidar_age_s=None if self.latest_scan is None else self.now()-self.stamp(self.latest_scan),
                bypass_candidates=self.controller.side_choices))))
            path=Path();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
            targets=[self.pose[:2]]
            if self.controller.target is not None:targets.append(self.controller.target)
            if hasattr(self.controller,'goal'):targets.append(self.controller.goal)
            for point in targets:
                world=np.asarray(point)@rotation(self.global_origin[2]).T+self.global_origin[:2]
                item=PoseStamped(header=path.header);item.pose.position.x,item.pose.position.y=map(float,world);item.pose.orientation.w=1.
                path.poses.append(item)
            self.path.publish(path)


def main(args=None):
    rclpy.init(args=args);node=AvoidanceNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.controller.active:node.send(0.,0.)
        node.destroy_node();rclpy.try_shutdown()
