"""EKF-guided goals with LiDAR avoidance and continuous wheel/gyro bypass motion."""
from collections import deque
from dataclasses import fields
import json
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist
from nav_msgs.msg import Odometry,Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,QoSProfile,qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from l2s3_curling.path_plan import TARGETS
from .planner import Controller,Settings,Scan,View,wrap,rotation,relative_pose,ekf_goal_guidance
from .motion import MotionLimiter


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_adv')
        defaults=dict(vars(Settings()),scan_topic='/scan',odom_topic='/odom',imu_topic='/imu/estimation',
                      target_run=0,
                      ekf_pose_topic='/marker_ekf/pose',ekf_frame='map',ekf_timeout=1.0,
                      max_position_std_m=.50,max_heading_std_deg=20.0,
                      detection_topic='/detection_improved/obstacle/detections',
                      command_topic='/obstacle_avoidance_adv/cmd_vel',base_frame='base_footprint',
                      input_timeout=1.5,depth_timeout=3.,scan_memory_s=.30,control_rate=20.,
                      linear_acceleration=.5,resume_clear_time=.2)
        for key,value in defaults.items():self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.value=lambda key:self.get_parameter(key).value
        # Zero preserves the distance mission; runs 1..5 use the shared assignment targets.
        target_run=self.value('target_run')
        if type(target_run) is not int or target_run not in (0,*TARGETS):
            raise ValueError('target_run must be 0 (forward distance) or 1..5 (curling target)')
        self.target_name=TARGETS[target_run][0] if target_run else 'Forward-distance endpoint'
        for key in ('input_timeout','depth_timeout','scan_memory_s','control_rate','ekf_timeout',
                    'max_position_std_m','max_heading_std_deg'):
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
        # Observe the EKF separately: these messages never overwrite self.pose.
        self.latest_ekf_pose=None;self.ekf_received=None
        self.field_reference=None;self.field_received=None
        self.comparison_reference=None
        # Diagnostic history uses odometry acquisition times, separate from scan history.
        self.comparison_history=deque(maxlen=600)
        # The map goal stays fixed; only its representation in the motion frame changes.
        self.goal_map=None;self.navigation_anchor=None
        self.navigation_status=dict(ready=False,reason='No active mission',
                                    target_run=target_run,target_name=self.target_name)
        self.command=self.create_publisher(Twist,self.value('command_topic'),1)
        self.status=self.create_publisher(String,'~/status',1);self.path=self.create_publisher(Path,'~/planned_path',1)
        self.create_subscription(LaserScan,self.value('scan_topic'),self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Imu,self.value('imu_topic'),self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,self.value('odom_topic'),self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom/wheels',self.on_wheels,qos_profile_sensor_data)
        self.create_subscription(String,self.value('detection_topic'),self.on_depth,1)
        self.create_subscription(PoseWithCovarianceStamped,self.value('ekf_pose_topic'),self.on_ekf_pose,1)
        # The field server supplies the saved start; avoid duplicating its YAML values.
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/football_field/state',self.on_field_state,retained)
        self.create_service(Trigger,'~/start',self.start);self.create_service(Trigger,'~/stop',self.stop)
        self.create_timer(1/self.value('control_rate'),self.tick)
        self.get_logger().info('IDLE: EKF goal guidance with LiDAR/gyro bypasses. Start service required.')

    def now(self):return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def on_ekf_pose(self,msg):
        # Preserve the full field-frame message, including acquisition time and covariance.
        self.latest_ekf_pose=msg
        self.ekf_received=time.monotonic()

    def on_field_state(self,msg):
        try:
            data=json.loads(msg.data)
            start=data['start']
            pose=np.array([start['start_x_m'],start['start_y_m'],
                           math.radians(start['start_heading_deg'])],dtype=float)
            if data['frame_id']!=self.value('ekf_frame') or not np.isfinite(pose).all():
                raise ValueError('Invalid field frame or starting pose')
            if abs(wrap(pose[2]))>1e-6:
                raise ValueError('Pose comparison requires zero starting heading')
            # A changed session/revision identifies a reset of the field alignment.
            self.field_reference=((data['session_id'],data['revision']),pose)
            self.field_received=time.monotonic()
        except (ValueError,KeyError,TypeError):
            self.field_reference=None;self.field_received=None

    def capture_comparison_reference(self):
        # Freeze the assumed start for this mission; never redefine it from an EKF fix.
        self.comparison_reference=None
        if (self.field_reference is not None and self.field_received is not None
                and time.monotonic()-self.field_received<=self.value('input_timeout')):
            key,start=self.field_reference
            self.comparison_reference=(key,start.copy())

    def ekf_diagnostics(self):
        """Report EKF quality and a timestamp-matched comparison without modifying motion."""
        report=dict(available=self.latest_ekf_pose is not None,valid=False,fresh=False,
                    within_uncertainty_limits=False,uncertainty_reason='No valid EKF pose',
                    max_position_std_m=self.value('max_position_std_m'),
                    max_heading_std_deg=self.value('max_heading_std_deg'),
                    reason='No EKF pose received',comparison=dict(ready=False,reason='No valid EKF pose'))
        msg=self.latest_ekf_pose
        if msg is None:
            return report
        stamp=self.stamp(msg)
        age=self.now()-stamp
        receipt_age=time.monotonic()-self.ekf_received
        report.update(frame_id=msg.header.frame_id,stamp_s=stamp,age_s=age,receipt_age_s=receipt_age)
        try:
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            values=np.array([p.x,p.y,q.x,q.y,q.z,q.w])
            covariance=np.asarray(msg.pose.covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
            if msg.header.frame_id!=self.value('ekf_frame'):
                raise ValueError('Unexpected EKF frame')
            if not np.isfinite(values).all() or not np.isfinite(covariance).all():
                raise ValueError('Non-finite EKF pose or planar covariance')
            norm=float(np.linalg.norm(values[2:]))
            if abs(norm-1.)>.05:
                raise ValueError('Invalid EKF orientation quaternion')
            if (np.any(np.diag(covariance)<=0)
                    or not np.allclose(covariance,covariance.T,rtol=0.,atol=1e-9)
                    or np.linalg.eigvalsh(covariance).min() < -1e-9):
                raise ValueError('Invalid EKF planar covariance')
            # Normalize a near-unit quaternion for heading extraction without changing the message.
            x,y,z,w=values[2:]/norm
            heading=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            field_pose=np.array([p.x,p.y,heading])
            std=np.sqrt(np.diag(covariance))
            # The largest x/y eigenvalue accounts for uncertainty along any direction,
            # including a diagonal when x and y errors are correlated.
            position_std=math.sqrt(float(np.linalg.eigvalsh(covariance[:2,:2]).max()))
            if not math.isfinite(position_std):
                raise ValueError('Invalid EKF position uncertainty')
            heading_std=math.degrees(float(std[2]))
            uncertainty_errors=[]
            if position_std>self.value('max_position_std_m'):
                uncertainty_errors.append(f'EKF position uncertainty {position_std:.2f} m exceeds limit '
                                          f'{self.value("max_position_std_m"):.2f} m')
            if heading_std>self.value('max_heading_std_deg'):
                uncertainty_errors.append(f'EKF heading uncertainty {heading_std:.2f} deg exceeds limit '
                                          f'{self.value("max_heading_std_deg"):.2f} deg')
            fresh=stamp>0 and -.1<=age<=self.value('ekf_timeout') and receipt_age<=self.value('ekf_timeout')
            report.update(valid=True,fresh=fresh,reason='' if fresh else 'EKF pose is stale or future-dated',
                          position_m=field_pose[:2].tolist(),heading_deg=math.degrees(heading),
                          sigma_x_m=float(std[0]),sigma_y_m=float(std[1]),
                          sigma_position_m=position_std,sigma_heading_deg=heading_std,
                          within_uncertainty_limits=not uncertainty_errors,
                          uncertainty_reason='; '.join(uncertainty_errors))
            if fresh and uncertainty_errors:
                report['reason']=report['uncertainty_reason']
            # Compare only while the legacy estimate is integrating an active mission.
            if not fresh:
                raise ValueError(report['reason'])
            if not self.controller.active:
                raise ValueError('Legacy mission pose only advances during an active mission')
            if self.comparison_reference is None:
                raise ValueError('No assumed start captured for this mission')
            key,start=self.comparison_reference
            if (self.field_reference is None or key!=self.field_reference[0]
                    or not np.array_equal(start,self.field_reference[1])):
                raise ValueError('Field alignment changed; comparison reference is no longer valid')
            if time.monotonic()-self.field_received>self.value('input_timeout'):
                raise ValueError('Field alignment metadata is stale')
            # Never clamp an unmatched timestamp or compare acquisition time with receipt time.
            samples=self.comparison_history
            if len(samples)<2 or not samples[0][0]<=stamp<=samples[-1][0]:
                raise ValueError('No legacy pose history brackets the EKF acquisition time')
            times=np.array([t for t,pose in samples])
            poses=np.array([pose for t,pose in samples])
            upper=min(max(1,int(np.searchsorted(times,stamp))),len(times)-1)
            if times[upper]-times[upper-1]>.5:
                raise ValueError('Legacy pose history has a gap around the EKF acquisition time')
            legacy=np.array([np.interp(stamp,times,poses[:,0]),np.interp(stamp,times,poses[:,1]),
                             np.interp(stamp,times,np.unwrap(poses[:,2]))])
            relative=relative_pose(field_pose,start)
            delta=relative-legacy
            delta[2]=wrap(delta[2])
            report['comparison']=dict(ready=True,reason='',stamp_s=stamp,
                assumed_start_map=start.tolist(),ekf_relative_pose=relative.tolist(),
                legacy_relative_pose=legacy.tolist(),delta_position_m=delta[:2].tolist(),
                distance_difference_m=float(np.linalg.norm(delta[:2])),delta_heading_deg=math.degrees(delta[2]))
        except (ValueError,TypeError,np.linalg.LinAlgError) as error:
            if not report['valid']:
                report['reason']=str(error)
            report['comparison']['reason']=str(error)
        return report

    def require_ekf_pose(self):
        """Require a usable pose and the configured field reference before navigation."""
        report=self.ekf_diagnostics()
        if not report['valid'] or not report['fresh']:
            raise ValueError('EKF input unavailable: '+report['reason'])
        if not report['within_uncertainty_limits']:
            raise ValueError(report['uncertainty_reason'])
        if (self.field_reference is None or self.field_received is None
                or time.monotonic()-self.field_received>self.value('input_timeout')):
            raise ValueError('Waiting for fresh assumed-start field metadata')
        return report

    def update_navigation_goal(self,report):
        """Use EKF corrections for goal guidance while keeping bypass/scan coordinates continuous."""
        if self.goal_map is None or self.comparison_reference is None:
            raise ValueError('Navigation has no fixed map goal or assumed start')
        key,start=self.comparison_reference
        if key!=self.field_reference[0] or not np.array_equal(start,self.field_reference[1]):
            raise ValueError('Field alignment changed; stop and start a new mission')
        if self.navigation_anchor is not None and report['stamp_s']<self.navigation_anchor['stamp_s']:
            raise ValueError('EKF acquisition time moved backwards; start a new mission')

        comparison=report['comparison']
        if comparison['ready']:
            # Only combine estimates with matching acquisition times.
            self.navigation_anchor=dict(
                pose=np.array([*report['position_m'],math.radians(report['heading_deg'])]),
                motion=np.array(comparison['legacy_relative_pose']),
                stamp_s=report['stamp_s'],received=self.ekf_received)
        elif not comparison['reason'].startswith('No legacy pose history brackets'):
            raise ValueError('EKF alignment unavailable: '+comparison['reason'])

        # A new EKF sample can precede its matching wheel sample by one callback.
        # Retain the last matched alignment only while its own timestamp and receipt are fresh.
        anchor=self.navigation_anchor
        if anchor is None:
            raise ValueError('Waiting for timestamp-matched EKF and wheel/gyro poses')
        age=self.now()-anchor['stamp_s']
        if not -.1<=age<=self.value('ekf_timeout') or time.monotonic()-anchor['received']>self.value('ekf_timeout'):
            raise ValueError('Timestamp-matched EKF alignment is stale')
        goal,pose_map=ekf_goal_guidance(self.goal_map,anchor['pose'],anchor['motion'],self.pose)
        self.controller.set_goal(goal)
        self.navigation_status=dict(ready=True,reason='',frame_id=self.value('ekf_frame'),
            target_run=self.value('target_run'),target_name=self.target_name,
            goal_map_m=self.goal_map.tolist(),goal_motion_m=goal.tolist(),
            pose_map=pose_map.tolist(),ekf_stamp_s=anchor['stamp_s'],ekf_age_s=age,
            distance_to_goal_m=float(np.linalg.norm(self.goal_map-pose_map[:2])))

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
        if self.controller.active:
            # Snapshot the existing estimator without replacing it with the EKF.
            self.comparison_history.append((stamp,self.pose.copy()))

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
            self.require_ekf_pose()
            if abs(self.velocity)>.03 or abs(self.gyro_rate)>.08:raise ValueError('Stop the rover before capturing the route origin')
            self.pose=np.zeros(3);self.gyro_origin=self.gyro_angle;self.history.clear();self.scans.clear();self.scan_token=None
            self.comparison_history.clear()
            p=self.latest_odom.pose.pose;self.global_origin=np.array([p.position.x,p.position.y,yaw(p.orientation)])
            self.capture_comparison_reference()
            if self.comparison_reference is None:
                raise ValueError('Assumed-start field metadata became stale before mission start')
            # Curling coordinates are absolute: the assumed start never shifts these targets.
            target_run=self.value('target_run')
            if target_run:
                self.goal_map=np.array(TARGETS[target_run][1:],dtype=float)
            else:
                self.goal_map=self.comparison_reference[1][:2]+np.array([self.controller.cfg.goal_distance,0.])
            self.navigation_anchor=None
            self.navigation_status=dict(ready=False,reason='Waiting for timestamp-matched EKF and wheel/gyro poses',
                                        goal_map_m=self.goal_map.tolist(),target_run=target_run,
                                        target_name=self.target_name)
            self.history.append((self.now(),self.pose.copy()));self.controller.start(self.pose,time.monotonic())
            # Initial visualisation uses the assumed start; motion still waits for an EKF match.
            self.controller.set_goal(self.goal_map-self.comparison_reference[1][:2])
            self.motion.reset();self.hold_since=None
            self.direction_mismatch_since=None
            response.success=True;response.message=('Mission started; '+self.target_name+
                '; EKF goal guidance and gyro bypass turns; output: '+self.value('command_topic'))
            if self.value('command_topic')!='/cmd_vel':response.message+=' (preview only)'
        except (ValueError,TransformException) as error:
            response.success=False;response.message=str(error);self.controller.reason='Start rejected: '+str(error)
        return response

    def stop(self,request,response):
        self.controller.stop();self.motion.reset();self.send(0.,0.)
        self.navigation_anchor=None
        self.navigation_status.update(ready=False,reason='Mission stopped')
        response.success=True;response.message='Mission stopped';return response

    def send(self,v,w):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.command.publish(msg)
        self.last_sent=(float(v),float(w))

    def goal_visualization(self):
        """Publish the actual fixed goal, or its idle preview, in field coordinates."""
        goal=self.goal_map
        if goal is None:
            target_run=self.value('target_run')
            if target_run:
                goal=np.array(TARGETS[target_run][1:],dtype=float)
            elif self.field_reference is not None:
                goal=self.field_reference[1][:2]+np.array([self.controller.cfg.goal_distance,0.])
        if goal is None:
            return None
        return dict(frame_id=self.value('ekf_frame'),position_m=goal.tolist(),
                    target_run=self.value('target_run'),target_name=self.target_name)

    def tick(self):
        now=time.monotonic();self.history.append((self.now(),self.pose.copy()))
        active=self.controller.active
        try:
            view=self.prepare()
            report=self.require_ekf_pose()
            if active:
                self.update_navigation_goal(report)
            else:
                self.navigation_status.update(ready=False,reason='No active mission')
            # Clear the hold timer only after every required input has recovered.
            self.readiness='Ready';self.hold_since=None
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
            self.navigation_status.update(ready=False,reason=str(error))
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
                localization='marker_ekf_goal_guidance',motion_localization='gyro_heading_and_forward_velocity',
                speed_source=self.speed_source,navigation=self.navigation_status,
                goal=self.goal_visualization(),
                ekf=self.ekf_diagnostics(),
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
