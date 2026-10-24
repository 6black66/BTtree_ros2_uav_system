#!/usr/bin/env python3
"""Exercise the independently implemented adapter against isolated mock PX4 DDS topics."""
import argparse
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from msgs.msg import FlightCommand, FlightCommandResult, FlightSetpoint, FlightState, FlightLinkState
from msgs.srv import FlightCommand as CommandService
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleGlobalPosition, VehicleAttitude,
                          VehicleLandDetected, VehicleCommand, VehicleCommandAck, OffboardControlMode, TrajectorySetpoint)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable',required=True)
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    rclpy.init()
    node=rclpy.create_node('px4_adapter_integration')
    best=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
    latched=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def native(direction,name,cls):
        version=getattr(cls,'MESSAGE_VERSION',0)
        return '/px4_mock/fmu/'+direction+'/'+name+(f'_v{version}' if version else '')
    def publisher(name,cls): return node.create_publisher(cls,native('out',name,cls),best)
    status_pub=publisher('vehicle_status',VehicleStatus)
    local_pub=publisher('vehicle_local_position',VehicleLocalPosition)
    global_pub=publisher('vehicle_global_position',VehicleGlobalPosition)
    attitude_pub=publisher('vehicle_attitude',VehicleAttitude)
    land_pub=publisher('vehicle_land_detected',VehicleLandDetected)
    ack_pub=publisher('vehicle_command_ack',VehicleCommandAck)
    setpoint_pub=node.create_publisher(FlightSetpoint,'/uav_test/px4/setpoint',10)
    request_pub=node.create_publisher(FlightCommand,'/uav_test/px4/command',10)
    service=node.create_client(CommandService,'/uav_test/px4/command_service')
    results,states,links,commands,trajectories,modes=[],[],[],[],[],[]
    status=VehicleStatus(system_id=1,component_id=1,vehicle_type=VehicleStatus.VEHICLE_TYPE_ROTARY_WING,
                         nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL,arming_state=VehicleStatus.ARMING_STATE_DISARMED)
    local=VehicleLocalPosition(x=10.,y=20.,z=-3.,vx=1.,vy=2.,vz=-.5,xy_valid=True,z_valid=True,
                               v_xy_valid=True,v_z_valid=True,xy_global=True,z_global=True,ref_lat=30.,ref_lon=120.,ref_alt=100.)
    settings={'status':True,'local':True,'refresh':None,'ack':'accept','apply':True,'drop_first':False,'trajectory_baseline':0}
    command_counts={}
    def command_received(command):
        commands.append((time.monotonic(),command))
        if command.command==176 and command.param2==6 and status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            assert len(trajectories)==settings['trajectory_baseline'], 'trajectory injected before PX4 entered Offboard'
        key=command.command
        command_counts[key]=command_counts.get(key,0)+1
        if settings['ack']=='drop' or (settings['drop_first'] and command.confirmation==0): return
        code=2 if settings['ack']=='deny' else 0
        ack=VehicleCommandAck(command=command.command,result=code,from_external=True,
                              target_system=command.source_system,target_component=command.source_component)
        ack.timestamp=node.get_clock().now().nanoseconds//1000
        ack_pub.publish(ack)
        if code==0 and settings['apply']:
            if command.command==VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
                status.arming_state=VehicleStatus.ARMING_STATE_ARMED if command.param1 else VehicleStatus.ARMING_STATE_DISARMED
            elif command.command==VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                status.nav_state=({6:VehicleStatus.NAVIGATION_STATE_OFFBOARD,3:VehicleStatus.NAVIGATION_STATE_POSCTL}.get(int(command.param2),
                    {3:VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,5:VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
                     6:VehicleStatus.NAVIGATION_STATE_AUTO_LAND}.get(int(command.param3),VehicleStatus.NAVIGATION_STATE_POSCTL)))
    subscriptions=[
        node.create_subscription(FlightCommandResult,'/uav_test/px4/command_result',results.append,100),
        node.create_subscription(FlightState,'/uav_test/px4/state',states.append,best),
        node.create_subscription(FlightLinkState,'/uav_test/px4/link_state',links.append,latched),
        node.create_subscription(VehicleCommand,native('in','vehicle_command',VehicleCommand),command_received,best),
        node.create_subscription(TrajectorySetpoint,native('in','trajectory_setpoint',TrajectorySetpoint),
                                 lambda message:trajectories.append((time.monotonic(),message)),best),
        node.create_subscription(OffboardControlMode,native('in','offboard_control_mode',OffboardControlMode),
                                 lambda message:modes.append((time.monotonic(),message)),best),
    ]
    last_feedback=last_setpoint=0
    process=None
    with tempfile.TemporaryDirectory(prefix='px4-adapter-test-') as temp:
        log=open(Path(temp)/'adapter.log','w+')
        def start(readonly=False):
            nonlocal process
            process=subprocess.Popen([args.executable,'--ros-args','-r','__ns:=/uav_test',
                '-p',f'config_file:={args.config}','-p','fmu_namespace:=/px4_mock',
                '-p','status_timeout_ms:=300','-p','local_timeout_ms:=200','-p','offboard_warmup_ms:=1100',
                '-p','ack_timeout_ms:=150','-p','command_timeout_ms:=1200',
                '-p',f'command_enabled:={str(not readonly).lower()}'],stdout=log,stderr=log)
        def send_setpoint(identifier='goal',source='planner',**fields):
            message=FlightSetpoint(command_id=identifier,source=source,ttl_ms=250,**fields)
            message.header.stamp=node.get_clock().now().to_msg()
            setpoint_pub.publish(message)
        def send_command(identifier,command,source='planner'):
            if command=='offboard': settings['trajectory_baseline']=len(trajectories)
            message=FlightCommand(command_id=identifier,source=source,command=command)
            message.header.stamp=node.get_clock().now().to_msg()
            request_pub.publish(message)
        def has(identifier,state): return any(r.command_id==identifier and r.status==state for r in results)
        def step():
            nonlocal last_feedback,last_setpoint
            t=time.monotonic()
            if t-last_feedback>.03:
                stamp=node.get_clock().now().nanoseconds//1000
                if settings['status']:
                    status.timestamp=stamp; status_pub.publish(status)
                if settings['local']:
                    local.timestamp=stamp; local_pub.publish(local)
                global_pub.publish(VehicleGlobalPosition(timestamp=stamp,lat=30.,lon=120.,alt=103.,lat_lon_valid=True,alt_valid=True))
                attitude_pub.publish(VehicleAttitude(timestamp=stamp,q=[1.,0.,0.,0.]))
                land_pub.publish(VehicleLandDetected(timestamp=stamp,landed=False))
                last_feedback=t
            if settings['refresh'] and t-last_setpoint>.04:
                send_setpoint(**settings['refresh']); last_setpoint=t
            rclpy.spin_once(node,timeout_sec=.005)
            if process.poll() is not None: raise AssertionError(f'adapter exited: {process.returncode}')
        def wait(predicate,timeout=6):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                step()
                if predicate(): return
            raise AssertionError('timeout waiting for adapter behavior')
        def spin_for(seconds):
            end=time.monotonic()+seconds
            wait(lambda:time.monotonic()>=end,seconds+1)
        def stop():
            process.send_signal(signal.SIGINT); process.wait(timeout=4)
            assert process.returncode==0
        try:
            start()
            wait(lambda:states and states[-1].local_position_valid and service.service_is_ready() and setpoint_pub.get_subscription_count()>0)
            assert (states[-1].x,states[-1].y,states[-1].z)==(20.,10.,3.)
            assert (states[-1].vx,states[-1].vy,states[-1].vz)==(2.,1.,.5)
            assert states[-1].global_position_valid and abs(states[-1].yaw-math.pi/2)<1e-6
            assert not commands and not trajectories and not modes, 'startup must not control PX4'
            settings['refresh']={'identifier':'goal','mode':0,'x':1.,'y':2.,'z':3.,'use_yaw':True,'yaw':0.}
            wait(lambda:has('goal','accepted'))
            spin_for(.15)
            assert not trajectories, 'setpoints alone must not start Offboard stream'
            bad=FlightSetpoint(command_id='unstamped',source='planner')
            setpoint_pub.publish(bad); wait(lambda:has('unstamped','rejected'))
            send_command('offboard','offboard')
            wait(lambda:has('offboard','confirmed'))
            mode_command=next(t for t,c in commands if c.command==176 and c.param2==6)
            assert mode_command-modes[0][0]>=1.0, 'continuous warmup missing'
            trajectory=trajectories[0][1]
            assert list(trajectory.position)==[2.,1.,-3.] and all(math.isnan(v) for v in trajectory.velocity)
            assert all(math.isnan(v) for v in trajectory.acceleration) and abs(trajectory.yaw-math.pi/2)<1e-6
            assert modes[0][1].position and not modes[0][1].velocity
            assert not any(c.command==400 for _,c in commands), 'no automatic arming'
            future=service.call_async(CommandService.Request(command_id='arm',source='planner',command='arm'))
            wait(future.done); assert future.result().accepted
            wait(lambda:has('arm','confirmed'))
            arm=next(c for _,c in commands if c.command==400)
            assert arm.param1==1. and arm.param2==0 and arm.from_external and arm.target_system==1
            settings['refresh']={'identifier':'velocity','mode':1,'vx':3.,'vy':4.,'vz':.5,'use_yaw_rate':True,'yaw_rate':.3}
            wait(lambda:any(all(math.isnan(v) for v in m.position) and list(m.velocity)==[4.,3.,-.5] for _,m in trajectories))
            wait(lambda:modes[-1][1].velocity and not modes[-1][1].position)
            assert abs(trajectories[-1][1].yawspeed+.3)<1e-6
            send_setpoint('intruder',source='other'); wait(lambda:has('intruder','rejected'))
            send_setpoint('bad_speed',mode=1,vx=100.); wait(lambda:has('bad_speed','rejected'))
            settings['refresh']=None
            wait(lambda:has('velocity','expired') and links and not links[-1].streaming_setpoints)
            wait(lambda:status.nav_state==VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)
            spin_for(.2); count=len(trajectories); spin_for(.2); assert len(trajectories)==count
            # ACK without the matching real state must not confirm a mode transition.
            wait(lambda:links[-1].offboard_session=='idle')
            spin_for(.15)
            settings['refresh']={'identifier':'second','mode':0,'x':2.,'y':3.,'z':4.}
            wait(lambda:has('second','accepted'))
            settings['apply']=False
            send_command('offboard2','offboard')
            wait(lambda:has('offboard2','acknowledged'))
            assert not has('offboard2','confirmed')
            status.nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD
            wait(lambda:has('offboard2','confirmed'))
            settings['apply']=True
            status.nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
            wait(lambda:has('second','cancelled') and not links[-1].streaming_setpoints)
            count=len(commands); spin_for(.3)
            assert len(commands)==count and not links[-1].streaming_setpoints, 'manual mode change must not auto-reenter'
            settings['refresh']=None
            # Retry missing ACK, then confirm only after real state agrees.
            settings['drop_first']=True
            send_command('retry','position_mode')
            wait(lambda:has('retry','confirmed'))
            assert any(c.confirmation==1 for _,c in commands)
            settings['drop_first']=False; settings['ack']='deny'
            send_command('denied','arm'); wait(lambda:has('denied','rejected'))
            assert next(r for r in results if r.command_id=='denied' and r.status=='rejected').px4_result==2
            settings['ack']='drop'
            send_command('no_ack','position_mode'); wait(lambda:has('no_ack','timeout'))
            settings['ack']='accept'
            # Local estimator freshness and vehicle type are enforced.
            settings['local']=False
            wait(lambda:states and not states[-1].local_position_valid)
            send_setpoint('stale_estimator'); wait(lambda:has('stale_estimator','rejected'))
            settings['local']=True
            status.vehicle_type=VehicleStatus.VEHICLE_TYPE_FIXED_WING
            wait(lambda:states[-1].vehicle_type==VehicleStatus.VEHICLE_TYPE_FIXED_WING)
            send_setpoint('fixed_wing'); wait(lambda:has('fixed_wing','rejected'))
            status.vehicle_type=VehicleStatus.VEHICLE_TYPE_ROTARY_WING
            settings['status']=False
            wait(lambda:not states[-1].connected)
            send_command('offline','arm'); wait(lambda:has('offline','rejected'))
            settings['status']=True
            wait(lambda:states[-1].connected)
            assert not links[-1].streaming_setpoints
            stop()
            # Verify the XML/CLI read-only switch with the same independent mock.
            states.clear(); start(readonly=True)
            wait(lambda:states and states[-1].connected and states[-1].local_position_valid)
            send_setpoint('readonly_goal'); wait(lambda:has('readonly_goal','rejected'))
            send_command('readonly_arm','arm'); wait(lambda:has('readonly_arm','rejected'))
            stop()
            print('PX4 DDS integration passed: independent native topic mock, ENU/NED, warmup, NaNs, explicit arming, ACK+state confirmation, lease arbitration, TTL hold, manual takeover, retry/deny/timeout, estimator/link freshness, vehicle limits, read-only')
        except BaseException:
            log.flush(); log.seek(0); print(log.read()); raise
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=4)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            log.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
