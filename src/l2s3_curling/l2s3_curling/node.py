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
from sensor_msgs.msg import CompressedImage,Imu,LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from .path_plan import TARGETS
from .planner import Controller,Settings,Scan,View,wrap,rotation,relative_pose,ekf_goal_guidance,turn_direction_mismatch
from .motion import MotionLimiter
from .visualization import motion_intent
from .decision_log import CAMERA_TOPICS, DecisionLog


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class ScanUnavailable(ValueError):
    """LiDAR input is unavailable and may recover without restarting the mission."""


class CurlingControllerNode(Node):
    def __init__(self):
        super().__init__('curling_controller')
        defaults=dict(vars(Settings()),scan_topic='/scan',odom_topic='/odom',imu_topic='/imu/estimation',
                      target_run=0,
                      ekf_pose_topic='/marker_ekf/pose',ekf_frame='map',ekf_timeout=1.0,
                      max_position_std_m=.80,max_heading_std_deg=30.0,
                      command_topic='/curling_controller/cmd_vel',base_frame='base_footprint',
                      input_timeout=1.5,scan_memory_s=.30,control_rate=20.,
                      linear_acceleration=.5,resume_clear_time=.2,decision_log_dir='',
                      relocalize_distance_m=1.0,relocalize_position_std_m=.20,
                      relocalize_heading_std_deg=8.,relocalize_marker_age_s=1.5,
                      relocalize_duration_s=8.)
        for key,value in defaults.items():self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.value=lambda key:self.get_parameter(key).value
        # Zero preserves the distance mission; runs 1..5 use the shared assignment targets.
        target_run=self.value('target_run')
        if type(target_run) is not int or target_run not in (0,*TARGETS):
            raise ValueError('target_run must be 0 (forward distance) or 1..5 (curling target)')
        self.target_name=TARGETS[target_run][0] if target_run else 'Forward-distance endpoint'
        for key in ('input_timeout','scan_memory_s','control_rate','ekf_timeout',
                    'max_position_std_m','max_heading_std_deg','relocalize_distance_m',
                    'relocalize_position_std_m','relocalize_heading_std_deg',
                    'relocalize_marker_age_s','relocalize_duration_s'):
            if not math.isfinite(self.value(key)) or self.value(key)<=0:raise ValueError(key+' must be finite and positive')
        if self.value('relocalize_duration_s') > 8.:
            raise ValueError('relocalize_duration_s may not exceed 8 seconds')
        self.controller=Controller(Settings(**{f.name:self.value(f.name) for f in fields(Settings)}))
        self.motion=MotionLimiter(self.value('linear_acceleration'),self.value('angular_acceleration'),self.value('resume_clear_time'))
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pose=np.zeros(3);self.history=deque(maxlen=600);self.scans=deque(maxlen=5)
        self.latest_scan=self.latest_odom=None;self.scan_token=None
        self.gyro_stamp=None;self.gyro_rate=0.;self.gyro_angle=0.;self.gyro_origin=0.;self.gyro_bias=0.
        self.bias_samples=deque(maxlen=40);self.received={};self.odom_stamp=None;self.velocity=0.
        self.wheel_v=0.;self.wheel_received=-math.inf;self.speed_source='waiting'
        self.global_origin=np.zeros(3);self.last_sent=(0.,0.);self.last_status=0.;self.hold_since=None
        self.readiness='Waiting for LiDAR, gyro and odometry';self.view=None
        self.scan_hold = False
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
        self.local_started_at=None;self.last_visualization=0.
        self.arrival=None
        self.marker_state = None
        self.relocalization = None
        self.relocalization_result = None
        self.waiting_after_relocalization = False
        self.last_decision_key = None
        log_dir = self.value('decision_log_dir')
        self.decision_log = DecisionLog(log_dir, self.get_logger().warning) if log_dir else None
        if self.decision_log is not None:
            for name, topic in CAMERA_TOPICS.items():
                self.create_subscription(CompressedImage, topic,
                    lambda message, name=name: self.decision_log.on_image(name,message), qos_profile_sensor_data)
        self.command=self.create_publisher(Twist,self.value('command_topic'),1)
        self.status=self.create_publisher(String,'~/status',1);self.path=self.create_publisher(Path,'~/planned_path',1)
        self.visualization_pub=self.create_publisher(String,'~/visualization',1)
        self.create_subscription(LaserScan,self.value('scan_topic'),self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Imu,self.value('imu_topic'),self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,self.value('odom_topic'),self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom/wheels',self.on_wheels,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,self.value('ekf_pose_topic'),self.on_ekf_pose,1)
        self.create_subscription(String,'/marker_ekf/state',self.on_marker_state,1)
        self.create_subscription(String,'/marker_pan_tracker/state',self.on_camera_state,1)
        self.camera_search = self.create_publisher(String,'/localization/relocalize_camera',1)
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

    def on_marker_state(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            self.marker_state = (time.monotonic(), data)
        except (ValueError, TypeError):
            return

    def on_camera_state(self, msg):
        if self.relocalization is None:
            return
        try:
            state = json.loads(msg.data)
            mode = state.get('mode')
            actions = self.relocalization['camera_actions']
            if isinstance(mode, str) and (not actions or actions[-1]['mode'] != mode):
                actions.append(dict(stamp_s=self.now(), mode=mode, pan_deg=state.get('pan_deg'),
                                    target_id=state.get('search_target_id')))
        except (ValueError, TypeError):
            return

    def recent_marker_state(self, pose_report, now):
        if self.marker_state is None or self.comparison_reference is None:
            return None
        received, data = self.marker_state
        key, _ = self.comparison_reference
        if (not data.get('initialised') or now-received > self.value('ekf_timeout')
                or data.get('frame_id') != self.value('ekf_frame')
                or (data.get('field_session'), data.get('field_revision')) != key):
            return None
        try:
            pose_stamp = float(data['pose_stamp_s'])
            lag = pose_report['stamp_s'] - pose_stamp
            if not -1e-6 <= lag <= self.value('ekf_timeout'):
                return None
            age = data['seconds_since_marker']
            age = None if age is None else float(age)
            if age is not None and (not math.isfinite(age) or age < 0.):
                return None
            stamps = {label: float(stamp) for label, stamp in data['accepted_marker_stamps_s'].items()}
            if not all(math.isfinite(stamp) and 0. < stamp <= pose_stamp+1e-6 for stamp in stamps.values()):
                return None
        except (ValueError, TypeError, KeyError, AttributeError):
            return None
        return age, stamps

    @staticmethod
    def relocalization_pose(report):
        if report is None:
            return None
        return {key: report[key] for key in ('position_m', 'heading_deg', 'stamp_s',
                                             'sigma_position_m', 'sigma_heading_deg')}

    def maybe_start_relocalization(self, report, distance, now):
        state = self.controller.state
        if (self.relocalization_result is not None or state not in ('APPROACH', 'REJOIN')
                or self.controller.backoff_start is not None
                or distance > self.value('relocalize_distance_m')):
            return
        marker_state = self.recent_marker_state(report, now)
        marker_age, stamps = marker_state if marker_state is not None else (None, {})
        stamp = self.now()
        marker_window = self.value('relocalize_marker_age_s')
        recent_tags = sorted(label for label, image_stamp in stamps.items()
                             if stamp - image_stamp <= marker_window)
        duration = self.value('relocalize_duration_s')
        label = max(stamps, key=stamps.get) if stamps else ''
        tag_suffix = label.rsplit(':', 1)[-1]
        self.relocalization = dict(resume_state=state, start_monotonic=now, start_stamp_s=stamp,
                                   deadline=now+duration, accepted_tags=set(), camera_actions=[],
                                   before=self.relocalization_pose(report))
        self.controller.change('RELOCALIZE', now, 'Stopped for camera relocalization')
        self.motion.reset()
        self.send(0., 0.)
        self.camera_search.publish(String(data=json.dumps(dict(deadline_s=stamp+duration,
            marker_id=int(tag_suffix) if tag_suffix.isascii() and tag_suffix.isdecimal() else None))))
        self.get_logger().info('RELOCALIZE started '+json.dumps(dict(state=state, distance_m=distance,
            duration_s=duration, marker_age_s=marker_age, last_marker=label or None,
            recent_tags=recent_tags, recent_tag_window_s=marker_window,
            before=self.relocalization['before'])))
        return True

    def update_relocalization(self, report, now):
        attempt = self.relocalization
        marker_state = self.recent_marker_state(report, now)
        if marker_state is not None:
            _, stamps = marker_state
            attempt['accepted_tags'] = {label for label, stamp in stamps.items()
                                        if stamp > attempt['start_stamp_s']}
        if now >= attempt['deadline']:
            self.end_relocalization('timeout', now, report)

    def end_relocalization(self, outcome, now, report=None, resume=True):
        if self.relocalization is None:
            return
        attempt = self.relocalization
        self.relocalization = None
        self.camera_search.publish(String(data='{"deadline_s":0}'))
        after = self.relocalization_pose(report)
        before = attempt['before']
        self.relocalization_result = dict(outcome=outcome, elapsed_s=now-attempt['start_monotonic'],
            accepted_tags=sorted(attempt['accepted_tags']), camera_actions=attempt['camera_actions'], before=before, after=after,
            position_shift_m=math.dist(before['position_m'], after['position_m']) if after else None)
        self.get_logger().info('RELOCALIZE ended '+json.dumps(self.relocalization_result))
        if resume:
            self.controller.change(attempt['resume_state'], now, f'Relocalization {outcome}; resuming navigation')
            self.controller.progress_at = now

    def relocalization_status(self):
        if self.relocalization is None:
            return self.relocalization_result
        return dict(outcome='searching', accepted_tags=sorted(self.relocalization['accepted_tags']),
                    remaining_s=max(0., self.relocalization['deadline']-time.monotonic()))

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
        if (self.field_reference is None or self.field_received is None
                or time.monotonic()-self.field_received>self.value('input_timeout')):
            raise ValueError('Waiting for fresh assumed-start field metadata')
        return report

    def fused_goal_distance(self,report):
        """Use the published FUSED pose itself as the arrival authority, in map coordinates."""
        if self.goal_map is None or self.comparison_reference is None:
            raise ValueError('Navigation has no fixed map goal or assumed start')
        key,start=self.comparison_reference
        if key!=self.field_reference[0] or not np.array_equal(start,self.field_reference[1]):
            raise ValueError('Field alignment changed; stop and start a new mission')
        if self.navigation_anchor is not None and report['stamp_s']<self.navigation_anchor['stamp_s']:
            raise ValueError('EKF acquisition time moved backwards; start a new mission')
        return float(np.linalg.norm(self.goal_map-np.array(report['position_m'])))

    def update_navigation_goal(self,report):
        """Use EKF corrections for goal guidance while keeping bypass/scan coordinates continuous."""
        fused_distance=self.fused_goal_distance(report)
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
            distance_to_goal_m=fused_distance,arrival_source='FUSED',
            fused_pose_map=[*report['position_m'],math.radians(report['heading_deg'])],
            projected_distance_to_goal_m=float(np.linalg.norm(self.goal_map-pose_map[:2])))

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

    def pose_at(self,stamp):
        if not self.history:return self.pose.copy()
        times=np.array([t for t,p in self.history]);poses=np.array([p for t,p in self.history])
        if stamp<times[0]:return poses[0].copy()
        return np.array([np.interp(stamp,times,poses[:,0]),np.interp(stamp,times,poses[:,1]),
                         np.interp(stamp,times,np.unwrap(poses[:,2]))])

    def prepare(self):
        now=self.now();timeout=self.value('input_timeout')
        if self.count_publishers(self.value('command_topic'))!=1:raise ValueError('Another velocity publisher owns the command topic')
        for key in ('gyro','odom','scan'):
            error_type = ScanUnavailable if key == 'scan' else ValueError
            if now-self.received.get(key,-math.inf)>timeout:raise error_type(key+' missing/stale')
        for key,stamp in (('gyro',self.gyro_stamp),('odom',self.odom_stamp),('scan',self.stamp(self.latest_scan))):
            error_type = ScanUnavailable if key == 'scan' else ValueError
            if stamp is None or not -.1<=now-stamp<=timeout:raise error_type(key+' acquisition missing/stale')
        msg=self.latest_scan;stamp=self.stamp(msg);token=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if token!=self.scan_token:
            tf=self.buffer.lookup_transform(self.value('base_frame'),msg.header.frame_id,Time())
            previous=self.scans[-1] if self.scans and stamp-self.scans[-1].stamp<.5 else None
            cfg=self.controller.cfg
            scan=Scan(msg,tf.transform,self.pose_at(stamp),stamp,previous,cfg.half_length,cfg.half_width)
            if np.isfinite(scan.ranges).sum()<3:raise ScanUnavailable('LiDAR has insufficient observed rays')
            self.scans.append(scan);self.scan_token=token
        scans=[s for s in self.scans if stamp-s.stamp<=self.value('scan_memory_s')]
        self.view=View(scans,self.pose,self.controller.cfg,now-stamp)
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
            self.arrival=None
            self.relocalization_result = None
            self.waiting_after_relocalization = False
            self.navigation_status=dict(ready=False,reason='Waiting for timestamp-matched EKF and wheel/gyro poses',
                                        goal_map_m=self.goal_map.tolist(),target_run=target_run,
                                        target_name=self.target_name)
            self.history.append((self.now(),self.pose.copy()));self.controller.start(self.pose,time.monotonic())
            self.local_started_at=self.now()
            # Initial visualisation uses the assumed start; motion still waits for an EKF match.
            self.controller.set_goal(self.goal_map-self.comparison_reference[1][:2])
            self.motion.reset();self.hold_since=None
            self.scan_hold = False
            self.direction_mismatch_since=None
            self.last_decision_key = None
            if self.decision_log is not None:
                self.decision_log.new_run()
            response.success=True;response.message=('Mission started; '+self.target_name+
                '; EKF goal guidance and gyro bypass turns; output: '+self.value('command_topic'))
            if self.value('command_topic')!='/cmd_vel':response.message+=' (preview only)'
        except (ValueError,TransformException) as error:
            response.success=False;response.message=str(error);self.controller.reason='Start rejected: '+str(error)
        return response

    def stop(self,request,response):
        self.controller.stop();self.motion.reset();self.send(0.,0.)
        self.end_relocalization('operator_stop', time.monotonic(), resume=False)
        self.navigation_anchor=None
        self.navigation_status.update(ready=False,reason='Mission stopped')
        response.success=True;response.message='Mission stopped';return response

    def send(self,v,w):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.command.publish(msg)
        self.last_sent=(float(v),float(w))

    def log_decision(self, previous_state, decision, stamp):
        if self.decision_log is None:
            return
        if decision is None:
            self.last_decision_key = None
            return
        key = (decision['kind'], self.controller.state, decision['check'])
        if key == self.last_decision_key:
            return
        record = dict(decision, stamp_s=stamp, previous_state=previous_state,
                      state=self.controller.state, reason=self.controller.reason,
                      target_run=self.value('target_run'), pose_local=self.pose.tolist(),
                      fused_pose_map=self.navigation_status.get('fused_pose_map'),
                      command_velocity=list(self.last_sent))
        def add_ages(value):
            if isinstance(value, dict):
                if 'source' in value and 'stamp_s' in value:
                    point_stamp = value['stamp_s']
                    value['age_s'] = stamp-point_stamp if point_stamp is not None else None
                for child in tuple(value.values()):
                    add_ages(child)
            elif isinstance(value, list):
                for child in value:
                    add_ages(child)

        add_ages(record)
        path = self.decision_log.enqueue(record)
        if path is not None:
            self.last_decision_key = key
            self.get_logger().info(f'Avoidance decision {previous_state} -> {self.controller.state} '
                                   f'({decision["kind"]}): {self.controller.reason}; {path}')

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

    def motion_visualization(self):
        """Keep dead reckoning fixed to the start, but place intent using EKF alignment."""
        ekf=self.ekf_diagnostics()
        result=dict(frame_id=self.value('ekf_frame'),state=self.controller.state,
                    reason=self.controller.reason,readiness=self.readiness,motion_status=self.motion.status,
                    ekf_warning=ekf['uncertainty_reason'] if ekf['fresh'] else '',
                    ekf=ekf,
                    relocalization=self.relocalization_status(),
                    navigation=self.navigation_status,
                    command_velocity=list(self.last_sent),goal=self.goal_visualization(),
                    local_pose=None,fused_pose=None,local_fresh=False,comparison=None,intent=None,
                    local_timeout_s=self.value('input_timeout'),fused_timeout_s=self.value('ekf_timeout'),
                    session_id=None,revision=None)
        reference=self.comparison_reference
        same_reference=(reference is not None and self.field_reference is not None
                        and reference[0]==self.field_reference[0]
                        and np.array_equal(reference[1],self.field_reference[1]))
        if same_reference:
            key,start=reference
            result.update(session_id=key[0],revision=key[1])
            # This is deliberately the ORIGINAL alignment, never the current EKF transform.
            result['local_pose']=[float(start[0]+self.pose[0]),float(start[1]+self.pose[1]),float(self.pose[2])]
            stamp=self.comparison_history[-1][0] if self.comparison_history else self.local_started_at
            if stamp is not None:
                age=self.now()-stamp
                fresh=-.1<=age<=self.value('input_timeout')
                if self.controller.active:
                    gyro_age=self.now()-self.gyro_stamp if self.gyro_stamp is not None else math.inf
                    fresh=fresh and -.1<=gyro_age<=self.value('input_timeout')
                    if math.isfinite(gyro_age):
                        age=max(age,gyro_age)
                result.update(local_age_s=age,local_fresh=fresh)
            comparison=ekf.get('comparison',{})
            if comparison.get('ready'):
                result['comparison']=dict(distance_m=comparison['distance_difference_m'],
                                          heading_deg=comparison['delta_heading_deg'])
        if ekf['valid'] and ekf['fresh']:
            result['fused_pose']=[*ekf['position_m'],math.radians(ekf['heading_deg'])]
            result['fused_age_s']=ekf['age_s']
            # Show the actual published FUSED pose/distance, which is also the arrival authority.
            if result['goal'] is not None:
                result['distance_to_goal_m']=float(np.linalg.norm(
                    np.array(result['goal']['position_m'])-np.array(result['fused_pose'][:2])))
        if result['local_fresh'] and result['fused_pose'] is not None and self.navigation_anchor is not None:
            intent=motion_intent(self.controller,self.pose,self.last_sent,
                self.readiness if self.readiness!='Ready' else self.motion.status if not any(self.last_sent) else '')
            # Anchor the action to the corrected robot pose. Its waypoints remain local internally.
            angle=result['fused_pose'][2]-self.pose[2]
            origin=np.array(result['fused_pose'][:2])
            intent['path']=[(origin+rotation(angle)@(np.array(p)-self.pose[:2])).tolist() for p in intent['path']]
            result['intent']=intent
        return result

    def publish_motion_path(self,visualization):
        """Replace the rough waypoint chain with a short map-frame command preview."""
        path=Path();path.header.frame_id=self.value('ekf_frame')
        path.header.stamp=self.get_clock().now().to_msg()
        intent=visualization.get('intent')
        # An in-place rotation is an icon/arc, not a translated robot-centre trajectory.
        if intent and intent['kind'] in ('drive','reverse'):
            points=intent['path']
            for i,point in enumerate(points):
                item=PoseStamped(header=path.header)
                item.pose.position.x,item.pose.position.y=point
                a,b=(points[i],points[i+1]) if i+1<len(points) else (points[i-1],points[i])
                heading=math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if intent['kind']=='reverse' else 0.)
                item.pose.orientation.z=math.sin(heading/2);item.pose.orientation.w=math.cos(heading/2)
                path.poses.append(item)
        self.path.publish(path)

    def tick(self):
        now=time.monotonic();self.history.append((self.now(),self.pose.copy()))
        active=self.controller.active
        try:
            report=None;arrived=False;arrival_distance=None
            if active:
                # Check arrival before scan readiness, local-history matching or any bypass action.
                report=self.require_ekf_pose()
                arrival_distance=self.fused_goal_distance(report)
                if self.relocalization is not None:
                    self.update_relocalization(report, now)
                if self.relocalization is None:
                    self.maybe_start_relocalization(report, arrival_distance, now)
                previous_state=self.controller.state
                # Let an in-progress search finish before acting on a single correction.
                awaiting_final_search = (self.relocalization_result is None and
                    arrival_distance <= self.value('relocalize_distance_m'))
                arrived=(self.relocalization is None and not awaiting_final_search and
                         self.controller.finish_if_at_goal(arrival_distance,now,'FUSED'))
                if arrived:
                    self.motion.reset();self.send(0.,0.)
                    self.readiness='Ready';self.hold_since=None
                    self.arrival=dict(source='FUSED',previous_state=previous_state,
                        pose_map=[*report['position_m'],math.radians(report['heading_deg'])],
                        goal_map_m=self.goal_map.tolist(),distance_m=arrival_distance,
                        tolerance_m=self.controller.cfg.goal_tolerance_m,stamp_s=report['stamp_s'],
                        sigma_position_m=report['sigma_position_m'],sigma_heading_deg=report['sigma_heading_deg'],
                        uncertainty_warning=report['uncertainty_reason'])
                    self.navigation_status.update(ready=True,reason=self.controller.reason,
                        distance_to_goal_m=arrival_distance,arrival_source='FUSED',
                        pose_map=self.arrival['pose_map'],goal_map_m=self.goal_map.tolist())
                    self.get_logger().info(f'{previous_state} -> DONE: FUSED position '
                        f'{report["position_m"]}, goal {self.goal_map.tolist()}, '
                        f'distance {arrival_distance:.3f} m <= {self.controller.cfg.goal_tolerance_m:.3f} m; '
                        f'FUSED uncertainty {report["sigma_position_m"]:.2f} m / '
                        f'{report["sigma_heading_deg"]:.2f} deg; sent zero velocity')
            if not arrived:
                view=self.prepare()
                report=report if report is not None else self.require_ekf_pose()
                if active:
                    self.update_navigation_goal(report)
                else:
                    self.navigation_status.update(ready=False,reason='No active mission')
                # Clear the hold timer only after every required input has recovered.
                self.readiness='Ready';self.hold_since=None
                if self.scan_hold:
                    self.controller.progress_at = now
                    self.scan_hold = False
                self.waiting_after_relocalization = False
                if active and self.relocalization is not None:
                    remaining = self.relocalization['deadline'] - now
                    tags = len(self.relocalization['accepted_tags'])
                    self.controller.reason = f'Camera search: {remaining:.1f} s left, {tags} accepted tags'
                    self.send(0.,0.)
                elif active:
                    decision_state = self.controller.state
                    planner_distance = arrival_distance
                    if awaiting_final_search:
                        # Finish the committed manoeuvre before the required final search.
                        planner_distance = max(arrival_distance, self.controller.cfg.goal_tolerance_m + 1.)
                    v,w=self.controller.step(self.pose,view,now,self.gyro_rate,self.velocity,
                                             arrival_distance=planner_distance)
                    mismatch=turn_direction_mismatch(self.last_sent,(v,w),self.gyro_rate)
                    if mismatch:
                        if self.direction_mismatch_since is None:self.direction_mismatch_since=now
                        if now-self.direction_mismatch_since>.75:
                            v,w=self.controller.fail(f'In-place turn sign mismatch: command '
                                f'{self.last_sent[1]:+.2f} rad/s, gyro {self.gyro_rate:+.2f} rad/s; '
                                'check motor/IMU sign')
                    else:self.direction_mismatch_since=None
                    if now-self.controller.progress_at>20 and max(abs(x) for x in self.last_sent)>.1:
                        v,w=self.controller.fail('Commanded motion made no measured progress')
                    self.send(*self.motion.update(v,w,now))
                    self.log_decision(decision_state, self.controller.decision, self.now())
        except (ValueError,TransformException) as error:
            self.readiness=str(error)
            self.navigation_status.update(ready=False,reason=str(error))
            if active:
                self.controller.rejoin_clear_since = None
                self.send(0.,0.);self.motion.reset();self.controller.reason='Input hold: '+str(error)
                if isinstance(error, ScanUnavailable):
                    self.scan_hold = True
                    self.hold_since = None
                elif self.scan_hold:
                    self.scan_hold = False
                    self.hold_since = None
                if self.hold_since is None:self.hold_since=now
                if 'publisher' in str(error) or (self.relocalization is None
                        and not self.scan_hold and not self.waiting_after_relocalization and now-self.hold_since>5.):
                    self.controller.fail(str(error))
                    self.end_relocalization('input_fault', now, resume=False)
                elif self.relocalization is not None and now >= self.relocalization['deadline']:
                    self.end_relocalization('timeout', now)
                    self.hold_since = now
                    self.waiting_after_relocalization = True
        # Smooth diagnostic overlays without increasing the verbose status-log frequency.
        if now-self.last_visualization>.1:
            self.last_visualization=now
            visualization=self.motion_visualization()
            self.visualization_pub.publish(String(data=json.dumps(visualization)))
            self.publish_motion_path(visualization)
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
                arrival=self.arrival,
                relocalization=self.relocalization_status(),
                ekf=self.ekf_diagnostics(),
                front_clearance_m=None if self.view is None or not math.isfinite(self.view.clearance()) else self.view.clearance(),
                lidar_age_s=None if self.latest_scan is None else self.now()-self.stamp(self.latest_scan),
                bypass_candidates=self.controller.side_choices))))


def main(args=None):
    rclpy.init(args=args);node=CurlingControllerNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.controller.active:node.send(0.,0.)
        node.end_relocalization('shutdown', time.monotonic(), resume=False)
        if node.decision_log is not None:node.decision_log.close()
        node.destroy_node();rclpy.try_shutdown()
