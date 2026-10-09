from pathlib import Path
import collections,json,sqlite3
import numpy as np,yaml,pinocchio as pin
from scipy.spatial.transform import Rotation
from scipy.signal import welch,butter,sosfiltfilt,correlate,correlation_lags
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG=Path('/tmp/wbmm_force_diag_04');OUT=Path('/tmp/wbmm_force_diag04_analysis');OUT.mkdir(exist_ok=True)
c=sqlite3.connect(f'file:{next(BAG.glob("*.db3"))}?mode=ro',uri=True)
topicmap={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')}
start,end=c.execute('select min(timestamp),max(timestamp) from messages').fetchone()
rows=collections.defaultdict(list)
for i,t,b in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 n,typ=topicmap[i];rows[n].append(((t-start)*1e-9,deserialize_message(b,typ)))
c.close();epoch=start*1e-9

def stamp(s):return (s.sec*10**9+s.nanosec-start)*1e-9
def ser(n,f):return np.array([t for t,m in rows[n]]),np.array([f(m) for t,m in rows[n]])
def stats(x):
 x=np.asarray(x);x=x[np.isfinite(x)]
 return dict(zip(['min','p50','p95','p99','max'],map(float,np.percentile(x,[0,50,95,99,100])))) if len(x) else None
def interp(t,ts,v):return np.column_stack([np.interp(t,ts,v[:,i]) for i in range(v.shape[1])])
def at(t,ts):return np.clip(np.searchsorted(ts,t,side='right')-1,0,len(ts)-1)
def trans(n):
 out=[];last=None
 for t,m in rows[n]:
  if m.data!=last:out.append({'t':t,'state':m.data});last=m.data
 return out
def wvec(m):return [m.wrench.force.x,m.wrench.force.y,m.wrench.force.z,m.wrench.torque.x,m.wrench.torque.y,m.wrench.torque.z]
tq,q=ser('/joint_states',lambda m:[m.position[m.name.index(f'joint_{i}')] for i in range(1,7)])
_,qv=ser('/joint_states',lambda m:[m.velocity[m.name.index(f'joint_{i}')] for i in range(1,7)])
tc,cmd=ser('/arm_controller/commands',lambda m:m.data)
to,obs=ser('/mobile_manipulator_mpc_observation',lambda m:m.state.value)
_,ot=ser('/mobile_manipulator_mpc_observation',lambda m:m.time)
tt,target=ser('/mobile_manipulator_ee_target',lambda m:m.state_trajectory[0].value)
tw,w=ser('/whole_body_force_control/processed_wrench',wvec)
tr,raw=ser('/fts_broadcaster/wrench',wvec)
tb,bc=ser('/cmd_vel',lambda m:[m.linear.x,m.angular.z])
td,od=ser('/wheel/odometry',lambda m:[m.pose.pose.position.x,m.pose.pose.position.y,Rotation.from_quat([m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w]).as_euler('xyz')[2],m.twist.twist.linear.x,m.twist.twist.angular.z])
tcor,cor=ser('/whole_body_force_control/correction',lambda m:m.data)
model=pin.buildModelFromUrdf('/home/a/WBMM/src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf');data=model.createData();frame=model.getFrameId('tool0')
def kin(q):
 pin.computeJointJacobians(model,data,q);pin.updateFramePlacements(model,data)
 J=pin.getFrameJacobian(model,data,frame,pin.LOCAL_WORLD_ALIGNED).copy();J[3:]*=.30
 return data.oMf[frame].translation.copy(),data.oMf[frame].rotation.copy(),np.linalg.svd(J,compute_uv=False)[-1]
positions=[];rotations=[];sigma=[]
for x in obs:
 p,R,s=kin(x[3:9]);rz=Rotation.from_euler('z',x[2]).as_matrix();positions.append(rz@p+np.r_[x[:2],0]);rotations.append(rz@R);sigma.append(s)
positions=np.array(positions);rotations=np.array(rotations);sigma=np.array(sigma)
target_at=target[at(to,tt)];poserr=np.linalg.norm(positions-target_at[:,:3],axis=1)
angerr=Rotation.from_matrix(np.einsum('nij,njk->nik',Rotation.from_quat(target_at[:,3:]).as_matrix().transpose(0,2,1),rotations)).magnitude()
tp=np.array([t for t,m in rows['/mobile_manipulator_mpc_policy']]);policies=[m for t,m in rows['/mobile_manipulator_mpc_policy']]
inputs=np.array([m.input_trajectory[0].value for m in policies]);pt=np.array([m.init_observation.time for m in policies]);policy_age=ot-pt[at(to,tp)]
logs=[dict(t=stamp(m.stamp),receipt=t,name=m.name,level=m.level,message=m.msg) for t,m in rows['/rosout']]
states={n:trans(n) for n in ['/whole_body_force_control/states','/whole_body_force_control/force_sensor_states','/mobile_manipulator_force_execution_state']}
tf=collections.defaultdict(list)
for t,m in rows['/tf']:
 for x in m.transforms:tf[x.child_frame_id].append([t,stamp(x.header.stamp)])
tf={k:np.array(v) for k,v in tf.items()}
raw_stamp=np.array([stamp(m.header.stamp) for t,m in rows['/fts_broadcaster/wrench']]);proc_stamp=np.array([stamp(m.header.stamp) for t,m in rows['/whole_body_force_control/processed_wrench']])
summary={'epoch':epoch,'duration':(end-start)*1e-9,'topics':{n:len(v) for n,v in rows.items()},'states':states,'logs':logs,'initial':{'state':obs[0].tolist(),'sigma':float(sigma[0]),'target':target[0].tolist(),'processed_wrench':w[0].tolist(),'base_command':bc[0].tolist(),'odom':od[0].tolist()},'tf':{k:{'count':len(v),'stamp_gap_ms':stats(np.diff(v[:,1])*1000)} for k,v in tf.items()},'force_age_ms':stats((tw-proc_stamp)*1000),'force_interarrival_ms':stats(np.diff(tw)*1000),'policy_interval_ms':stats(np.diff(tp)*1000),'policy_age_ms':stats(policy_age*1000),'poserr_mm':stats(poserr*1000),'angerr_deg':stats(angerr*180/np.pi),'sigma':stats(sigma),'yaw_max_step':float(max(abs(np.diff(obs[:,2]))))}
windows=[]
for a,b in [(a,a+10) for a in range(0,125,10)]:
 d={'window':[a,b]}
 for name,t,v in [('base_cmd',tb,bc),('odom_velocity',td,od[:,3:]),('force',tw,w[:,:3]),('cor_velocity',tcor,cor[:,21:24]),('qvel',tq,qv)]:
  z=v[(t>=a)&(t<b)]
  if len(z):d[name]={'mean':np.mean(z,axis=0).tolist(),'rms':np.sqrt(np.mean(z*z,axis=0)).tolist(),'min':np.min(z,axis=0).tolist(),'max':np.max(z,axis=0).tolist()}
 s=(to>=a)&(to<b)
 if np.any(s):d.update(sigma_min=float(min(sigma[s])),position_error_mm_max=float(max(poserr[s])*1000),state_start=obs[s][0].tolist(),state_end=obs[s][-1].tolist())
 sel=(tt>=a)&(tt<b)
 if np.any(sel):d['target_change']=(target[sel][-1,:3]-target[sel][0,:3]).tolist()
 windows.append(d)
summary['windows']=windows
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
np.savez_compressed(OUT/'series.npz',tq=tq,q=q,qv=qv,tc=tc,cmd=cmd,to=to,obs=obs,ot=ot,tt=tt,target=target,tw=tw,w=w,tr=tr,raw=raw,tb=tb,bc=bc,td=td,od=od,tcor=tcor,cor=cor,positions=positions,rotations=rotations,sigma=sigma,poserr=poserr,angerr=angerr,tp=tp,inputs=inputs,pt=pt,policy_age=policy_age,raw_stamp=raw_stamp,proc_stamp=proc_stamp)
print(json.dumps({k:v for k,v in summary.items() if k not in ['topics','windows']},indent=2))
print('WINDOWS')
for d in windows:
 if 'base_cmd' not in d:continue
 print(d['window'],'base cmd mean/range',np.round(d['base_cmd']['mean'],4),np.round(d['base_cmd']['min'],4),np.round(d['base_cmd']['max'],4),'odom rms',np.round(d['odom_velocity']['rms'],4),'Fmean',np.round(d['force']['mean'],3),'target delta',np.round(d.get('target_change',[]),4),'sigma',round(d.get('sigma_min',0),5))
