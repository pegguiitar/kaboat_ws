"""Run the existing avoidance controller once to its latched entry goal."""
import time,math,json,collections
from pathlib import Path as DiskPath
import rclpy
from geometry_msgs.msg import Twist,PointStamped
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data as q
rclpy.init();n=rclpy.create_node('dock_existing_ctrl_trial');begin=time.monotonic();latest={};received={};valid_since=None
folder=DiskPath('/tmp/kaboat_dock_trials');folder.mkdir(exist_ok=True);file=folder/(time.strftime('%Y%m%d_%H%M%S')+'_existing.jsonl');log=file.open('w',buffering=1)
commands=[];points=[];pwm=[];armed=None;goal=None;start_xy=None;status_after_arm=False

def emit(kind,value):
 d={'elapsed_s':round(time.monotonic()-begin,3),'event':kind,'value':value};log.write(json.dumps(d,default=str)+'\n')
 if kind in ('phase','arm','progress','stop'):print(json.dumps(d,default=str),flush=True)
def cb(t,m):
 global valid_since
 now=time.monotonic();received[t]=now
 if isinstance(m,Path):
  latest['path']=m
  if m.poses:
   if valid_since is None:valid_since=now
  else:valid_since=None
 elif isinstance(m,PointStamped):latest['goal']=[m.point.x,m.point.y]
 elif isinstance(m,Odometry):
  p=m.pose.pose.position;a=m.pose.pose.orientation;latest['xy']=[p.x,p.y]
  latest['yaw_deg']=math.degrees(math.atan2(2*(a.w*a.z+a.x*a.y),1-2*(a.y*a.y+a.z*a.z)))
  points.append([now,p.x,p.y,latest['yaw_deg']]);emit('odom',points[-1])
 elif isinstance(m,Twist):commands.append([now,m.linear.x,m.angular.z]);emit('command',commands[-1])
 elif isinstance(m,DiagnosticArray):
  for d in m.status:
   if d.name=='Thruster System':
    latest['actuator']={v.key:v.value for v in d.values};received['actuator']=now
    pwm.append([now,int(latest['actuator']['left_pwm']),int(latest['actuator']['right_pwm'])]);emit('actuator',latest['actuator'])
 else:latest['status']=m.data;emit('status',m.data)
for t,k in [('/dock/bspline_path',Path),('/dock/final_waypoint',PointStamped),('/odom',Odometry),('/cmd_vel',Twist),('/diagnostics',DiagnosticArray),('/dock/avoid/status',String)]:n.create_subscription(k,t,lambda m,t=t:cb(t,m),q)
start=n.create_client(Trigger,'/dock/start');stop=n.create_client(Trigger,'/dock/stop');pc=n.create_client(GetParameters,'/dock_ctrl/get_parameters')
mode=n.create_publisher(String,'/teleop/esp_mode',10);pf=None;settings=False;mode_sent=False;af=None;last_attempt=0;last_report=0;outcome='waiting'
try:
 emit('phase','existing B-spline avoidance controller; waiting for valid entry path')
 while True:
  rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
  if pf is None and pc.service_is_ready():pf=pc.call_async(GetParameters.Request(names=['drive_enabled','goal_topic','max_linear_cmd','require_goal_updates_when_running','dock_final_switch_distance_m','dock_stop_goal_topic']))
  if pf is not None and pf.done() and not settings:
   v=pf.result().values
   if not v[0].bool_value or v[1].string_value!='/dock/approach_point' or abs(v[2].double_value-.25)>1e-6 or v[3].bool_value or abs(v[4].double_value-3.)>1e-6 or v[5].string_value!='/dock/stop_point':
    raise RuntimeError('live controller is not the expected existing-controller entry configuration')
   settings=True
  a=latest.get('actuator',{});path=latest.get('path')
  ready=(now-received.get('actuator',0)<2 and a.get('backend_open')=='True' and a.get('last_send_ok')=='True'
         and a.get('mode') in ('auto','timeout') and a.get('teleop_enabled')!='True')
  good=(path is not None and path.poses and valid_since is not None and now-valid_since>=.4
        and now-received.get('/dock/bspline_path',0)<.3)
  if armed is None:
   if now-begin>75:outcome='no valid path/actuator readiness';break
   if settings and ready and good and start.service_is_ready():
    if not mode_sent and mode.get_subscription_count()>0:
     mode.publish(String(data='computer_control_mode'));mode_sent=True;emit('phase','computer mode requested, neutral output until start')
    elif mode_sent and a.get('esp_mode')=='computer_control_mode' and af is None and now-last_attempt>.5:
     af=start.call_async(Trigger.Request());last_attempt=now
   if af is not None and af.done():
    response=af.result();emit('arm',{'success':response.success,'message':response.message})
    if response.success:armed=now;start_xy=latest.get('xy');goal=latest.get('goal');outcome='running'
    else:af=None
  else:
   goal=latest.get('goal',goal)
   status=latest.get('status','')
   if received.get('/dock/avoid/status',0)>armed and status.startswith(('WAIT:','STOP:','ERROR:','ESTOP:','FINISHED:')):outcome=status;break
   if now-armed>60:outcome='observation window ended; controller stopped by monitor';break
   if not ready or now-received.get('/dock/avoid/status',0)>1.:outcome='actuator/controller feedback unavailable';break
  if now-last_report>5:
   emit('progress',{'armed':armed is not None,'status':latest.get('status'),'xy':latest.get('xy'),
       'entry':goal or latest.get('goal'),'pwm':[a.get('left_pwm'),a.get('right_pwm')]});last_report=now
except Exception as exc:outcome=f'{type(exc).__name__}: {exc}'
finally:
 if stop.service_is_ready():
  f=stop.call_async(Trigger.Request());deadline=time.monotonic()+2
  while time.monotonic()<deadline and not f.done():rclpy.spin_once(n,timeout_sec=.05)
  emit('stop',f.result().message if f.done() and f.result() else 'stop requested')
 deadline=time.monotonic()+2
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.05);a=latest.get('actuator',{})
  if a.get('left_pwm')=='1500' and a.get('right_pwm')=='1500' and time.monotonic()-received.get('actuator',0)<1.5:break
 active=[c for c in commands if armed is not None and c[0]>=armed and abs(c[1])+abs(c[2])>1e-6]
 result={'outcome':outcome,'armed':armed is not None,'entry_xy':goal,'initial_xy':start_xy,'final_xy':latest.get('xy'),
    'nonzero_commands':len(active),'max_linear_cmd':max((abs(c[1]) for c in active),default=0.),
    'max_angular_cmd':max((abs(c[2]) for c in active),default=0.),'final_distance_m':math.dist(latest['xy'],goal) if goal and 'xy' in latest else None,
    'actuator':latest.get('actuator'),'last_status':latest.get('status'),'log':str(file)}
 log.write(json.dumps({'summary':result})+'\n');log.close();print(json.dumps(result),flush=True)
 (folder/'latest_existing_summary.json').write_text(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
