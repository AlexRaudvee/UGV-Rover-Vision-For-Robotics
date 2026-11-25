"""Stop, quarter turn, sidestep, quarter turn, pass, then diagonal to the goal."""
from dataclasses import dataclass
import math
import numpy as np


GAP_BLOCKED_S = 3.   # Something must block the gap this long before the rover gives up on it.
SIDESTEP_BLOCKED_S = 5.   # A blocked sidestep waits this long, then turns back and re-plans.
BACKOFF_M = .30           # The most the rover backs up to be able to turn (re-checking behind as it goes)...
BACKOFF_SPEED = .05       # ...and how fast, in m/s. The only reverse movement the controller makes.
BACKOFF_GIVE_UP_S = 10.   # Back-up not enough this long: look around instead (FAULT only if it moves with the rover).
LOOK_WAIT_S = 5.          # STOP with both sides blocked waits this long, then looks around...
ESCAPE_M = .5             # ...and drives this far towards the first clear direction it finds.
LOOK_STEP_DEG = 5         # The look-around checks the LiDAR scan every this many degrees.
ARC_SPEED = .08           # Forward speed while turning when a turn on the spot is not allowed.


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


@dataclass
class Settings:
    goal_distance: float = 8.0
    speed: float = 1.3
    avoidance_speed: float = .25
    angular_speed: float = .65
    half_length: float = .13
    half_width: float = .15
    side_margin: float = .08
    stop_clearance: float = .30
    emergency_clearance: float = .08
    sidestep_min: float = .50
    sidestep_max: float = 1.20
    pass_distance: float = 1.0
    braking_deceleration: float = .5
    angular_acceleration: float = 1.2
    reaction_time: float = .20
    heading_tolerance: float = .04
    distance_tolerance: float = .035
    settle_time: float = .25
    minimum_coverage: float = .65
    mission_timeout: float = 180.
    # Gap between two obstacles. Look for one when an obstacle is this close ahead.
    # The rover drives to a point in front of the gap (at most gap_max_angle off
    # the route and gap_max_offset sideways), turns parallel to the route, and
    # drives through until gap_exit beyond obstacles up to obstacle_length long.
    # Obstacles must lie within gap_side_window on both sides of its corridor.
    gap_lookahead: float = 1.5
    gap_max_angle: float = .70
    gap_max_offset: float = .80
    gap_runup: float = .30
    gap_exit: float = .50
    gap_side_window: float = .60
    obstacle_length: float = .50
    # Room on each side of the rover in a gap, used instead of side_margin there.
    # The smallest gap is 2 * (half_width + gap_margin). While driving through,
    # only something directly in the rover's path stops it.
    gap_margin: float = .06

    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in vars(self).values()):
            raise ValueError('Controller settings must be finite and positive')
        if not 5<=self.goal_distance<=10 or not self.avoidance_speed<=self.speed<=1.3:
            raise ValueError('Use goal_distance 5..10 m and avoidance_speed <= speed <= 1.3 m/s')
        if self.angular_speed>1. or self.emergency_clearance>=self.stop_clearance:
            raise ValueError('Invalid turn speed or obstacle clearances')
        if self.sidestep_min>self.sidestep_max or self.minimum_coverage>1:
            raise ValueError('Invalid sidestep limits or coverage fraction')

    @property
    def radius(self):
        return math.hypot(self.half_length,self.half_width)+self.side_margin


class Scan:
    """One scan in mission coordinates; small gaps are allowed, blind sectors are not rays."""
    def __init__(self,msg,transform,pose,stamp,previous=None,half_length=.13,half_width=.15):
        self.stamp=stamp;self.pose=np.array(pose);self.origin=np.array([transform.translation.x,transform.translation.y])
        q=transform.rotation
        self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ranges=np.asarray(msg.ranges,dtype=float)
        if (len(ranges)<36 or not math.isfinite(msg.angle_increment) or msg.angle_increment==0
                or not 0<msg.range_min<msg.range_max):
            raise ValueError('Malformed LiDAR scan')
        self.angle_min=msg.angle_min;self.increment=msg.angle_increment
        self.ranges=ranges.copy()
        finite=np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=msg.range_max)
        angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        local=np.column_stack((ranges[finite]*np.cos(angles[finite]),ranges[finite]*np.sin(angles[finite])))
        base=local@rotation(self.yaw).T+self.origin
        own=(abs(base[:,0])<half_length)&(abs(base[:,1])<half_width)
        indices=np.flatnonzero(finite);finite[indices[own]]=False
        base=base[~own];indices=indices[~own]
        # A real face has adjacent returns. An isolated ray must recur in the
        # previous scan instead of becoming an obstacle from one noisy sample.
        supported=np.zeros(len(base),dtype=bool)
        if len(base)>1:
            gaps=np.diff(indices)
            expected=np.minimum(np.linalg.norm(base[:-1],axis=1),np.linalg.norm(base[1:],axis=1))*abs(msg.angle_increment)*gaps
            near=(np.linalg.norm(np.diff(base,axis=0),axis=1)<.06+1.5*expected)&(gaps<=3)
            supported[:-1]|=near;supported[1:]|=near
        world=base@rotation(pose[2]).T+pose[:2]
        if previous is not None and len(previous.points) and len(world):
            unsupported=np.flatnonzero(~supported)
            if len(unsupported):
                supported[unsupported]=np.min(np.linalg.norm(world[unsupported,None,:]-previous.raw_points[None,:,:],axis=2),axis=1)<.06
        self.raw_points=world
        self.points=world[supported]
        self.ranges[~finite]=np.nan
        self.ranges[indices[~supported]]=np.nan
        self.ranges[np.isposinf(ranges)]=msg.range_max

    def observed(self,world_point):
        base=(world_point-self.pose[:2])@rotation(self.pose[2])
        point=(base-self.origin)@rotation(self.yaw)
        angle=math.atan2(point[1],point[0]);middle=self.angle_min+(len(self.ranges)-1)*self.increment/2
        index=int(round((middle+wrap(angle-middle)-self.angle_min)/self.increment))
        if not 0<=index<len(self.ranges):return False
        reach=max(1,int(math.radians(6)/abs(self.increment)))
        lo=max(0,index-reach);hi=min(len(self.ranges),index+reach+1)
        return bool(np.isfinite(self.ranges[lo:hi]).any())


class View:
    def __init__(self,scans,pose,depth,cfg,age):
        self.pose=np.asarray(pose);self.cfg=cfg;self.age=max(0.,age);self.scans=scans
        world=[s.points for s in scans]
        if len(depth):world.append(np.asarray(depth))
        self.world=np.vstack(world) if world else np.empty((0,2))
        self.points=(self.world-self.pose[:2])@rotation(self.pose[2])

    def clearance(self,heading=0.,margin=None):
        p=self.points@rotation(heading)
        margin=self.cfg.side_margin if margin is None else margin
        # Only points ahead of the bumper (less 2 cm) are in front. Points beside the front
        # half of the body cannot be hit by driving straight on; turns are checked by
        # rotation_clear. Run 10e: a box beside the rover gave "Emergency front clearance"
        # with nothing in front.
        front=p[(p[:,0]>self.cfg.half_length-.02)&(abs(p[:,1])<self.cfg.half_width+margin)]
        return float(front[:,0].min()-self.cfg.half_length) if len(front) else math.inf

    def coverage(self,heading,distance):
        distance=max(self.cfg.half_length+.2,distance)
        targets=np.column_stack((np.full(9,distance),np.linspace(-self.cfg.half_width,self.cfg.half_width,9)))
        world=targets@rotation(heading+self.pose[2]).T+self.pose[:2]
        return sum(any(s.observed(point) for s in self.scans) for point in world)/len(world)

    def segment_clear(self,heading,distance):
        p=self.points@rotation(heading)
        return (not np.any((p[:,0]>0)&(p[:,0]<distance+self.cfg.half_length)&
                           (abs(p[:,1])<self.cfg.half_width+self.cfg.side_margin))
                and self.coverage(heading,distance)>=self.cfg.minimum_coverage)

    def rotation_clear(self):
        return not np.any(np.linalg.norm(self.points,axis=1)<self.cfg.radius)

    def side_distance(self,side):
        front=self.clearance()+self.cfg.half_length
        p=self.points[(self.points[:,0]>max(.05,front-.12))&
                      (self.points[:,0]<front+.30)&(abs(self.points[:,1])<.9)]
        edge=float(np.max(side*p[:,1])) if len(p) else .20
        return max(self.cfg.sidestep_min,edge+self.cfg.half_width+self.cfg.side_margin+.05)

    def gap_sides(self):
        """Measured (right edge, left edge) of the gap around the rover, in its own frame, or None.

        Uses LiDAR points beside and just ahead of the rover: the nearest on each side,
        taking the third nearest so a single noisy return does not decide. Returns None
        unless both sides are seen and the opening is at least the rover's width.
        """
        p=self.points;near=(p[:,0]>-.05)&(p[:,0]<.90)
        left=np.sort(p[near&(p[:,1]>0)&(p[:,1]<.60),1]);right=np.sort(p[near&(p[:,1]<0)&(p[:,1]>-.60),1])[::-1]
        if len(left)<3 or len(right)<3:return None
        r,l=float(right[2]),float(left[2])
        return (r,l) if l-r>=2*self.cfg.half_width else None

    def escape_direction(self,route,free,excluded=()):
        """First clear direction, in degrees-from-the-route steps of LOOK_STEP_DEG, as an angle
        from the route (radians), or None. `route` is the route direction relative to the
        robot; `free` (+1 left, -1 right) is tried first at each step. Clear: nothing within
        stop_clearance + ESCAPE_M ahead of the bumper across the rover plus side margin,
        and that strip observed by the LiDAR (segment_clear)."""
        order=[0]
        for k in range(LOOK_STEP_DEG,181,LOOK_STEP_DEG):
            order+=[free*k] if k==180 else [free*k,-free*k]
        for deg in order:
            if round(deg)%360 in {e%360 for e in excluded}:continue
            relative=wrap(route+math.radians(deg))
            if self.clearance(relative)>self.cfg.stop_clearance+ESCAPE_M and self.segment_clear(relative,ESCAPE_M):
                return math.radians(deg)
        return None

    def freer_side(self):
        """+1 (left) or -1 (right): the side with fewer LiDAR points within 1.2 m; right on a tie."""
        p=self.points;near=(np.linalg.norm(p,axis=1)<1.2)&(p[:,0]>-.3)
        return 1 if np.sum(near&(p[:,1]>0))<np.sum(near&(p[:,1]<0)) else -1

    def rear_clear(self,distance):
        """Nothing measured within `distance` behind the rover, across its width plus 2 cm."""
        p=self.points
        return not np.any((p[:,0]<-self.cfg.half_length+.02)&(p[:,0]>-self.cfg.half_length-distance)&
                          (abs(p[:,1])<self.cfg.half_width+.02))

    def gap_plan(self,heading):
        """Plan a pass through a gap between two obstacles, or return None.

        `heading` is the blocked route direction relative to the robot. In route
        coordinates (x along the route), a lateral offset y is a gap when a corridor
        parallel to the route at y is clear from an entry point in front of the
        obstacles until gap_exit beyond them, obstacles lie on both sides of it, and
        the drive from the robot to the entry point is clear. The middle of the run
        of such offsets nearest the route wins.
        Returns (entry heading relative to robot, entry distance, through distance).
        """
        cfg=self.cfg;front=self.clearance(heading)+cfg.half_length
        # Only obstacles within the lookahead form a gap (run 10e planned a "gap" in front
        # of obstacles 8 m away from STOP, with its entry point 5.5 m ahead).
        if not math.isfinite(front) or front-cfg.half_length>cfg.gap_lookahead:return None
        q=self.points@rotation(heading)
        offsets=np.arange(-cfg.gap_max_offset,cfg.gap_max_offset+1e-9,.02)
        plans=[self.gap_offset(q,heading,front,y) for y in offsets]
        # Every check is per offset, so the chosen middle of a run is itself usable.
        runs,start=[],None
        for i,plan in enumerate(plans+[None]):
            if plan is not None and start is None:start=i
            if plan is None and start is not None:runs.append(offsets[(start+i-1)//2]);start=None
        if not runs:return None
        y=min(runs,key=abs);entry,through=self.gap_offset(q,heading,front,y)
        return heading+math.atan2(y,entry),math.hypot(entry,y),through

    def gap_offset(self,q,heading,front,y):
        """(entry, through) when lateral offset y (route coordinates) is a usable gap, else None.

        The obstacles beside the gap may be staggered, up to obstacle_length nearer than
        the obstacle on the route. The entry point is placed in front of the nearest of
        them, so the rover lines up before reaching any of them.
        """
        cfg=self.cfg;corridor=cfg.half_width+cfg.gap_margin;d=q[:,1]-y
        if abs(y)<corridor:return None
        near_row=(q[:,0]>front-cfg.obstacle_length)&(q[:,0]<front+cfg.obstacle_length+.05)
        left=near_row&(d>=corridor)&(d<corridor+cfg.gap_side_window)
        right=near_row&(d<=-corridor)&(d>-corridor-cfg.gap_side_window)
        if not (left.any() and right.any()):return None
        near=min(front,float(q[left|right,0].min()))
        entry=near-cfg.half_length-cfg.stop_clearance-cfg.gap_runup
        if entry<=0 or abs(math.atan2(y,entry))>cfg.gap_max_angle:return None
        through=front+cfg.obstacle_length+cfg.gap_exit-entry
        channel=(q[:,0]>entry-cfg.half_length)&(q[:,0]<entry+through+cfg.half_length)
        if np.any(channel&(abs(d)<corridor)):return None
        return (entry,through) if self.gap_reachable(heading,entry,through,y) else None

    def gap_reachable(self,heading,entry,through,y):
        """The drive to the entry point is clear, also for the stop clearance beyond it that the
        forward speed limit keeps, and the scan has observed the far side of the gap."""
        # segment_clear reaches half_length past the distance, so this covers the bumper
        # at the entry point plus stop_clearance.
        if not self.segment_clear(heading+math.atan2(y,entry),
                                  math.hypot(entry,y)+self.cfg.stop_clearance):return False
        far=np.array([[entry+through*f,y] for f in (.5,1.)])@rotation(heading+self.pose[2]).T
        return all(any(s.observed(p+self.pose[:2]) for s in self.scans) for p in far)


class Controller:
    def __init__(self,cfg=None):
        self.cfg=cfg or Settings();self.state='IDLE';self.reason='Call start when LiDAR, gyro and odometry are available'
        self.side=0;self.passes=0;self.target=None;self.side_choices={};self.side_blocked_since=None;self.backoff_start=None
        self.stop_blocked_since=None;self.look_free=-1;self.look_excluded=set();self.look_target=None;self.now=0.

    @property
    def active(self):return self.state not in ('IDLE','DONE','FAULT')

    def start(self,pose,now):
        self.goal=np.array([self.cfg.goal_distance,0.]);self.started=now;self.passes=0;self.side=0
        self.leg_heading=0.;self.target=self.goal.copy();self.change('APPROACH',now,'Driving to original endpoint')
        self.progress_pose=np.array(pose);self.progress_at=now

    def change(self,state,now,reason):
        self.state=state;self.entered=now;self.reason=reason;self.settled_since=None

    def stop(self):
        self.state='IDLE';self.reason='Mission stopped';self.target=None

    def fail(self,reason):
        self.state='FAULT';self.reason=reason;return 0.,0.

    def speed_limit(self,available,ceiling,age):
        a=self.cfg.braking_deceleration;t=self.cfg.reaction_time+age
        return min(ceiling,max(0.,math.sqrt((a*t)**2+2*a*max(0.,available))-a*t))

    def turn(self,pose,view,now,yaw_rate,next_state):
        error=wrap(self.turn_heading-pose[2])
        if not view.rotation_clear():
            # Run 10g: TURN_OUT waited here for good. Back up until the turn is possible.
            return self.back_off(pose,view,now,'Something inside the turning circle')
        # Hysteresis: once inside the tolerance, settling continues within twice it.
        # Otherwise gyro drift at the band edge restarts the settle timer forever.
        band=self.cfg.heading_tolerance*(2 if self.settled_since is not None else 1)
        if abs(error)<=band:
            if abs(yaw_rate)<.08:
                if self.settled_since is None:self.settled_since=now
                if now-self.settled_since>=self.cfg.settle_time:
                    self.leg_start=pose[:2].copy();self.change(next_state,now,'Turn finished using gyro angle')
            return 0.,0.
        self.settled_since=None
        speed=min(self.cfg.angular_speed,math.sqrt(2*self.cfg.angular_acceleration*abs(error)),max(.10,1.8*abs(error)))
        self.reason=f'Turning: {math.degrees(error):.1f} degrees remaining'
        return 0.,math.copysign(speed,error)

    def forward(self,pose,view,heading,remaining,ceiling,margin=None):
        error=wrap(heading-pose[2])
        if abs(error)>.20:
            if view.rotation_clear():
                self.reason=f'{self.state}: turning {math.degrees(error):+.0f} degrees towards the target'
                return 0.,float(np.clip(1.8*error,-self.cfg.angular_speed,self.cfg.angular_speed))
            # Something within the turning circle (run 10f: after the gap, in REJOIN, it waited
            # here silently for good). If the way ahead is clear, turn while creeping forward,
            # which moves the rover away from it; otherwise back up until it can turn.
            if view.clearance()>self.cfg.stop_clearance+.15:
                self.reason=f'{self.state}: too close to turn on the spot; turning while moving'
                return ARC_SPEED,float(np.clip(1.5*error,-.3,.3))
            return self.back_off(pose,view,self.now,'Something inside the turning circle')
        clearance=view.clearance(error,margin)
        allowed=min(remaining,clearance-self.cfg.stop_clearance)
        speed=self.speed_limit(allowed,ceiling,view.age)
        horizon=self.cfg.half_length+self.cfg.stop_clearance+(self.cfg.reaction_time+view.age)*speed+speed**2/(2*self.cfg.braking_deceleration)
        if view.coverage(error,horizon)<self.cfg.minimum_coverage:
            self.reason='Forward LiDAR sector not observed; waiting';return 0.,0.
        self.reason=f'{self.state}: clearance {clearance:.2f} m, remaining {remaining:.2f} m'
        return speed*max(0.,math.cos(error)),float(np.clip(1.5*error,-.3,.3)) if abs(error)>.025 else 0.

    def step(self,pose,view,now,yaw_rate=0.,measured_speed=0.):
        if not self.active:return 0.,0.
        self.now=now
        pose=np.asarray(pose)
        if now-self.started>self.cfg.mission_timeout:return self.fail('Mission timeout')
        if np.linalg.norm(pose[:2]-self.progress_pose[:2])>.03 or abs(wrap(pose[2]-self.progress_pose[2]))>.05:
            self.progress_pose=pose.copy();self.progress_at=now
        if now-self.progress_at>25 and abs(measured_speed)>.03:return self.fail('No position progress despite measured motion')
        margin=0. if self.state=='GAP_THROUGH' else None
        if view.clearance(0.,margin)<self.cfg.emergency_clearance:
            # Something this close ahead is also inside the turning circle, so the only way
            # out is back (runs 10d and 10g waited here for good).
            return self.back_off(pose,view,now,'Emergency front clearance')
        # A back-up continues until the rover can turn again; only then is it finished.
        if view.rotation_clear():self.backoff_start=None
        if self.state in ('APPROACH','REJOIN'):
            delta=self.goal-pose[:2];distance=float(np.linalg.norm(delta))
            if distance<=.08:
                self.change('DONE',now,'Original endpoint reached');return 0.,0.
            heading=math.atan2(delta[1],delta[0]);error=wrap(heading-pose[2])
            if abs(error)<.20 and view.clearance(error)<=self.cfg.gap_lookahead and self.enter_gap(pose,view,heading,now):
                return 0.,0.
            if abs(error)<.20 and view.clearance(error)<=self.cfg.stop_clearance+self.cfg.distance_tolerance:
                self.leg_heading=heading;self.change('STOP',now,'Stopping before obstacle');return 0.,0.
            return self.forward(pose,view,heading,distance,self.cfg.speed)
        if self.state=='STOP':
            if abs(measured_speed)>.05:return 0.,0.
            if self.settled_since is None:self.settled_since=now
            if now-self.settled_since<self.cfg.settle_time:return 0.,0.
            if self.enter_gap(pose,view,self.leg_heading,now):return 0.,0.
            candidates=[];self.side_choices={}
            for side in (-1,1):
                distance=view.side_distance(side);angle=side*math.pi/2
                clear=distance<=self.cfg.sidestep_max and view.segment_clear(angle,distance)
                self.side_choices['right' if side<0 else 'left']=dict(distance_m=distance,clear=clear)
                if clear:candidates.append((distance,side))
            if not candidates:
                # Wait briefly in case it is a person; then look around instead of staring
                # at the same blocked view until the mission timeout (run 10e).
                if self.stop_blocked_since is None:self.stop_blocked_since=now
                if now-self.stop_blocked_since>=LOOK_WAIT_S:
                    self.stop_blocked_since=None
                    return self.start_look(pose,view,now)
                self.reason='Both sidestep corridors blocked/unobserved; waiting for clearance';return 0.,0.
            self.stop_blocked_since=None
            shortest=min(p[0] for p in candidates)
            distance,self.side=next((p for p in candidates if p[1]<0 and p[0]<=shortest+.05),min(candidates))
            self.side_length=distance;self.turn_heading=wrap(self.leg_heading+self.side*math.pi/2)
            self.target=pose[:2]+distance*np.array([math.cos(self.turn_heading),math.sin(self.turn_heading)])
            self.change('TURN_OUT',now,'Committed to quarter turn and sidestep');return 0.,0.
        if self.state=='TURN_OUT':return self.turn(pose,view,now,yaw_rate,'SIDESTEP')
        if self.state=='SIDESTEP':
            progress=float((pose[:2]-self.leg_start)@np.array([math.cos(self.turn_heading),math.sin(self.turn_heading)]))
            remaining=self.side_length-progress
            if remaining<=self.cfg.distance_tolerance:
                self.turn_heading=self.leg_heading
                self.change('TURN_FORWARD',now,'Sidestep complete; turning back parallel to route');return 0.,0.
            if view.clearance()<=self.cfg.stop_clearance+self.cfg.distance_tolerance:
                # Run 10b waited here until the mission timeout. After SIDESTEP_BLOCKED_S,
                # turn back to the route and re-plan from PASS, which stops and chooses a
                # new side (possibly the other one) if the obstacle is still ahead.
                if self.side_blocked_since is None:self.side_blocked_since=now
                if now-self.side_blocked_since>=SIDESTEP_BLOCKED_S:
                    self.side_blocked_since=None;self.turn_heading=self.leg_heading
                    self.change('TURN_FORWARD',now,'Sidestep blocked; turning back to the route to re-plan');return 0.,0.
                self.reason='Sidestep path became blocked; waiting';return 0.,0.
            self.side_blocked_since=None
            return self.forward(pose,view,self.turn_heading,remaining,self.cfg.avoidance_speed)
        if self.state=='TURN_FORWARD':return self.turn(pose,view,now,yaw_rate,'PASS')
        if self.state=='PASS':
            progress=float((pose[:2]-self.leg_start)@np.array([math.cos(self.leg_heading),math.sin(self.leg_heading)]))
            remaining=self.cfg.pass_distance-progress
            if remaining<=self.cfg.distance_tolerance:
                self.passes+=1;self.target=self.goal.copy()
                self.change('REJOIN',now,'Pass complete; diagonal directly to original endpoint');return 0.,0.
            if view.clearance()<=self.cfg.stop_clearance+self.cfg.distance_tolerance:
                self.change('STOP',now,'New obstacle in passing corridor');return 0.,0.
            return self.forward(pose,view,self.leg_heading,remaining,self.cfg.avoidance_speed)
        if self.state=='GAP':
            # Re-aim at the entry point every cycle so the rover arrives on the gap's centre line.
            delta=self.gap_entry-pose[:2];fixed=np.array([math.cos(self.gap_heading),math.sin(self.gap_heading)])
            remaining=float(delta@fixed)
            if remaining<=self.cfg.distance_tolerance:
                self.leg_start=self.gap_entry.copy();self.turn_heading=self.leg_heading
                self.target=self.gap_entry+self.gap_through*np.array([math.cos(self.leg_heading),math.sin(self.leg_heading)])
                self.change('GAP_ALIGN',now,'In front of the gap; turning parallel to the route')
                return 0.,0.
            heading=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>.10 else self.gap_heading
            error=wrap(heading-pose[2])
            if abs(error)<.20 and view.clearance(error)<=self.cfg.stop_clearance+self.cfg.distance_tolerance:
                # The speed limit will not drive closer. Within 10 cm of the entry point,
                # line up from here: the run-up corrects the small sideways error.
                if remaining<=.10:
                    # Keep the planned entry: it defines the gap's centre line, and the
                    # few centimetres short are added to the drive through automatically.
                    self.leg_start=self.gap_entry.copy();self.turn_heading=self.leg_heading
                    self.target=self.gap_entry+self.gap_through*np.array([math.cos(self.leg_heading),math.sin(self.leg_heading)])
                    self.change('GAP_ALIGN',now,'Close to the gap entry; turning parallel to the route');return 0.,0.
                if self.gap_blocked_since is None:self.gap_blocked_since=now
                if now-self.gap_blocked_since>=GAP_BLOCKED_S:
                    self.change('STOP',now,'Drive to the gap entry blocked; stopping');return 0.,0.
                self.reason='Drive to the gap entry blocked; waiting';return 0.,0.
            self.gap_blocked_since=None
            return self.forward(pose,view,heading,remaining,self.cfg.avoidance_speed)
        if self.state=='GAP_ALIGN':
            # Gyro turn on the spot; turn() keeps leg_start at the planned entry point.
            entry=self.leg_start;command=self.turn(pose,view,now,yaw_rate,'GAP_THROUGH')
            self.leg_start=entry
            if self.state=='GAP_THROUGH':self.reason='Aligned with the gap; driving through';self.gap_blocked_since=None
            return command
        if self.state=='GAP_THROUGH':
            along=np.array([math.cos(self.leg_heading),math.sin(self.leg_heading)]);left=np.array([-along[1],along[0]])
            offset=pose[:2]-self.leg_start;remaining=self.gap_through-float(offset@along)
            if remaining<=self.cfg.distance_tolerance:
                self.passes+=1;self.target=self.goal.copy()
                self.change('REJOIN',now,'Through the gap; diagonal directly to original endpoint');return 0.,0.
            # Steer for the middle of the gap as the LiDAR measures it, aiming 0.4 m ahead.
            # Odometry drifts by a few cm over the approach (run 10d entered 10 cm off
            # centre), so the planned centre line is only used while a side is not seen.
            sides=view.gap_sides()
            if sides is not None:
                heading=wrap(pose[2]+math.atan2((sides[0]+sides[1])/2,.40))
            else:
                heading=wrap(self.leg_heading-math.atan2(float(offset@left),.40))
            # Only gentle corrections inside the gap: a larger error would make forward()
            # turn on the spot, which a box beside the rover does not allow.
            error=float(np.clip(wrap(heading-pose[2]),-.15,.15));heading=wrap(pose[2]+error)
            # The corridor was planned with the gap margin. Inside the gap the sides are
            # known to be close, so only something directly in the rover's path (its own
            # width) blocks it, and it must stay blocked for GAP_BLOCKED_S before the rover
            # gives up on the gap; until then it waits. This keeps single noisy LiDAR
            # points from the gap's sides from stopping it.
            if abs(error)<.20 and view.clearance(error,0.)<=self.cfg.stop_clearance+self.cfg.distance_tolerance:
                if self.gap_blocked_since is None:self.gap_blocked_since=now
                if now-self.gap_blocked_since>=GAP_BLOCKED_S:
                    self.change('STOP',now,'Gap blocked; stopping');return 0.,0.
                self.reason='Something in the gap ahead; waiting';return 0.,0.
            self.gap_blocked_since=None
            return self.forward(pose,view,heading,remaining,self.cfg.avoidance_speed,0.)
        if self.state=='LOOK':return self.turn(pose,view,now,yaw_rate,'LOOK_CHECK')
        if self.state=='LOOK_CHECK':return self.choose_look(pose,view,now)
        if self.state=='ESCAPE':
            direction=np.array([math.cos(self.escape_heading),math.sin(self.escape_heading)])
            remaining=ESCAPE_M-float((pose[:2]-self.leg_start)@direction)
            if remaining<=self.cfg.distance_tolerance:
                self.target=self.goal.copy()
                self.change('REJOIN',now,'Moved to a new view; diagonal to the original endpoint');return 0.,0.
            if view.clearance()<=self.cfg.stop_clearance+self.cfg.distance_tolerance:
                self.change('STOP',now,'Escape path blocked; stopping');return 0.,0.
            return self.forward(pose,view,self.escape_heading,remaining,self.cfg.avoidance_speed)
        return self.fail('Unexpected controller state')

    def start_look(self,pose,view,now):
        """Both sides blocked: find a way out from the 360 degree LiDAR scan, standing still."""
        self.look_free=view.freer_side();self.look_excluded=set()
        return self.choose_look(pose,view,now)

    def choose_look(self,pose,view,now):
        """Check every LOOK_STEP_DEG from the current scan, nearest the route first and the
        freer side first (5 freer, 5 other, 10 freer, ... 180). Facing the first clear one
        already: ESCAPE. Otherwise turn to it (LOOK, then LOOK_CHECK checks again). If no
        direction is clear, turn 90 degrees towards the freer side to see past anything
        hiding part of the scan, and check again: it never gives up (only the mission
        timeout ends it)."""
        facing_clear=view.clearance()>self.cfg.stop_clearance+ESCAPE_M and view.segment_clear(0.,ESCAPE_M)
        if self.state=='LOOK_CHECK' and self.look_target is not None and facing_clear:
            angle=self.look_target    # turned to the chosen direction and it is still clear: commit
        else:
            angle=view.escape_direction(wrap(self.leg_heading-pose[2]),self.look_free,self.look_excluded)
        if angle is None:
            self.look_target=None;self.turn_heading=wrap(pose[2]+self.look_free*math.pi/2)
            self.change('LOOK',now,'No clear direction in the scan; turning 90 degrees to look again');return 0.,0.
        target=wrap(self.leg_heading+angle)
        if abs(wrap(target-pose[2]))<=2*self.cfg.heading_tolerance:
            self.escape_heading=pose[2];self.leg_start=pose[:2].copy()
            self.target=pose[:2]+ESCAPE_M*np.array([math.cos(pose[2]),math.sin(pose[2])])
            self.change('ESCAPE',now,f'Clear towards {math.degrees(angle):+.0f} degrees from the route; moving there');return 0.,0.
        self.look_target=angle;self.turn_heading=target
        self.change('LOOK',now,f'Clear towards {math.degrees(angle):+.0f} degrees from the route; turning there');return 0.,0.

    def next_look(self,pose,view,now):
        """A LOOK turn could not be made (boxed in): try a different direction."""
        if self.look_target is not None:self.look_excluded.add(round(math.degrees(self.look_target)))
        return self.choose_look(pose,view,now)

    def back_off(self,pose,view,now,why):
        """Too close to drive on or to turn on the spot (something within emergency_clearance
        ahead, or inside the turning circle). After 1 s back straight up, slowly, by at most
        BACKOFF_M if nothing is behind, until a turn on the spot is possible; then the
        state's own logic carries on. If that is not enough within BACKOFF_GIVE_UP_S, look
        around (or, when already looking, skip to the next direction) instead of waiting."""
        if self.backoff_start is None:self.backoff_start=pose[:2].copy();self.backoff_since=now
        if now-self.backoff_since<1.:
            self.reason=f'{why}; stopped';return 0.,0.
        backed=float(np.linalg.norm(pose[:2]-self.backoff_start))
        if backed>=BACKOFF_M or not view.rear_clear(.15):
            if now-self.backoff_since>=BACKOFF_GIVE_UP_S:
                self.backoff_start=None
                if why.startswith('Emergency') and backed>=BACKOFF_M:
                    # Backed up the full distance with nothing behind and it is still just
                    # ahead: it moves with the rover. Looking around cannot fix that.
                    return self.fail(f'{why} even after backing up {backed:.2f} m: something moves with '
                                     'the rover; check for a cable or part in front of the LiDAR')
                if self.state!='LOOK':return self.start_look(pose,view,now)
                return self.next_look(pose,view,now)
            self.reason=f'{why}; cannot back up further';return 0.,0.
        self.reason=f'{why}; backing up';return -BACKOFF_SPEED,0.

    def enter_gap(self,pose,view,heading,now):
        """Switch to GAP when a gap between two obstacles fits the rover; heading is the route in mission frame."""
        plan=view.gap_plan(wrap(heading-pose[2]))
        if plan is None:return False
        relative,distance,through=plan
        self.leg_heading=heading;self.gap_heading=wrap(pose[2]+relative)
        self.gap_through=through;self.gap_blocked_since=None
        self.gap_entry=pose[:2]+distance*np.array([math.cos(self.gap_heading),math.sin(self.gap_heading)])
        self.target=self.gap_entry.copy()
        side='left' if wrap(self.gap_heading-heading)>0 else 'right'
        self.change('GAP',now,f'Gap found on the {side}; driving to its entry');return True
