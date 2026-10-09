from pathlib import Path
import runpy,json,collections
import numpy as np
from scipy.signal import butter,sosfiltfilt,welch
from scipy.optimize import least_squares
G=runpy.run_path('/tmp/wbmm_force_diag04_analysis/analyze_force_diag04.py')
globals().update(G)
fault=next(x['t'] for x in states['/whole_body_force_control/states'] if x['state'].startswith('FAULT'))
active=(to<fault)
more={'fault_time':fault,'active_metrics':{'position_mm':stats(poserr[active]*1000),'orientation_deg':stats(angerr[active]*180/np.pi),'sigma':stats(sigma[active])}}
mask=np.linalg.norm(w[:,:3],axis=1)==0
bounds=np.where(np.diff(np.r_[False,mask,False].astype(int)))[0]
zero_runs=[(float(tw[a]),float(tw[b-1]),int(b-a)) for a,b in zip(bounds[::2],bounds[1::2])]
more['zero_force_runs']=zero_runs
more['first_nonzero_force']=float(tw[np.flatnonzero(~mask)[0]])
# Exact stamped command pairs. Recorder arrival timestamps on unstamped
# /cmd_vel cannot measure sender->driver transport latency.
trecv,recv=ser('/tracer_base_node/command_received',lambda m:[stamp(m.header.stamp),m.twist.linear.x,m.twist.angular.z])
tsend,send=ser('/tracer_base_node/command_dispatched',lambda m:[stamp(m.header.stamp),m.twist.linear.x,m.twist.angular.z])
assert len(recv)==len(send) and np.array_equal(recv[:,1:],send[:,1:])
more['driver']={'pairs':len(recv),'sdk_call_duration_ms':stats((send[:,0]-recv[:,0])*1000),'receive_interval_ms':stats(np.diff(recv[:,0])*1000),'dispatch_interval_ms':stats(np.diff(send[:,0])*1000),'sdk_negative_duration_count':int(np.sum(send[:,0]<recv[:,0]))}
more['state_dimension']=dict(collections.Counter(len(m.state.value) for t,m in rows['/mobile_manipulator_mpc_observation']))
ix=at(to,td);more['velocity_state_to_latest_odom_difference']=stats(np.linalg.norm(obs[:,9:11]-od[ix,3:5],axis=1))
more['odom_child_frames']=dict(collections.Counter(m.child_frame_id for t,m in rows['/wheel/odometry']))
# Recover effective dynamics coefficients from solver trajectories, as an
# independent check of which model actually produced the recorded policies.
fits=[]
for axis in (0,1):
 reg=[];rhs=[]
 for m in policies[::10]:
  times=np.asarray(m.time_trajectory);xs=np.asarray([x.value for x in m.state_trajectory]);us=np.asarray([u.value for u in m.input_trajectory])
  if len(xs)!=len(times) or len(us)!=len(times) or xs.shape[1]!=11:continue
  h=np.diff(times);valid=(h>1e-5)&(h<.03)
  vel=xs[:,9+axis]
  reg.extend(np.c_[.5*(us[:-1,axis]+us[1:,axis]),-.5*(vel[:-1]+vel[1:])][valid]);rhs.extend((np.diff(vel)/h)[valid])
 A=np.asarray(reg);y=np.asarray(rhs);coef=np.linalg.lstsq(A,y,rcond=None)[0]
 fits.append({'axis':axis,'segments':len(y),'tau':float(1/coef[1]),'gain':float(coef[0]/coef[1]),'derivative_rmse':float(np.sqrt(np.mean((A@coef-y)**2)))})
more['policy_dynamics_recovered']=fits
# Time windows covering start, settled release, sustained drag and fault approach.
windows=[]
for label,a,b in [('initial_zero',max(1.647,to[0]),more['first_nonzero_force']),('settled_zero',62,68),('motion',5,100),('approach_fault',100,fault),('fault_tail',fault,124.81)]:
 item={'label':label,'window':[a,b]}
 ss=(to>=a)&(to<b);cc=(tb>=a)&(tb<b);dd=(td>=a)&(td<b);ff=(tw>=a)&(tw<b)
 item.update(position_mm=stats(poserr[ss]*1000),orientation_deg=stats(angerr[ss]*180/np.pi),sigma=stats(sigma[ss]))
 if np.any(ss):
  delta=obs[ss][-1,:3]-obs[ss][0,:3];item['base_net_xy_m']=float(np.linalg.norm(delta[:2]));item['base_yaw_change_rad']=float(delta[2]);item['tcp_net_m']=float(np.linalg.norm(positions[ss][-1]-positions[ss][0]));item['arm_joint_change_rad']=(obs[ss][-1,3:9]-obs[ss][0,3:9]).tolist()
  dxy=np.diff(obs[ss][:,:2],axis=0);item['base_path_length_m']=float(np.sum(np.linalg.norm(dxy,axis=1)))
 for name,z in [('command',bc[cc]),('reported_velocity',od[dd,3:]),('force',w[ff,:3])]:
  item[name]={'mean':np.mean(z,axis=0).tolist(),'rms':np.sqrt(np.mean(z*z,axis=0)).tolist(),'max_abs':np.max(abs(z),axis=0).tolist()}
 qq=(tq>=a)&(tq<b);item['joint_speed_peak_rad_s']=np.max(abs(qv[qq]),axis=0).tolist()
 windows.append(item)
more['windows']=windows
# Position and rotation error timeline for fault causality; measured posture,
# processed force and planned arm speeds at the last 5 s.
snapshots=[]
for mark in [90,95,98,100,101,102,103,104,fault-.008,fault,fault+.008,fault+.1,fault+.3,fault+.6,fault+1]:
 i=int(at(mark,to));k=int(at(mark,tw));j=int(at(mark,tb));pidx=int(at(mark,tp))
 snapshots.append({'t':float(to[i]),'position_mm':float(poserr[i]*1000),'orientation_rad':float(angerr[i]),'sigma':float(sigma[i]),'q_rad':obs[i,3:9].tolist(),'base_yaw_rad':float(obs[i,2]),'actual_velocity':obs[i,9:11].tolist(),'command':bc[j].tolist(),'force':w[k,:3].tolist(),'plan_input':inputs[pidx].tolist()})
more['fault_snapshots']=snapshots
iz=np.flatnonzero((tb>=fault)&(np.linalg.norm(bc,axis=1)==0))[0]
more['stop']={'first_zero_command_t':float(tb[iz]),'first_zero_delay_ms':float((tb[iz]-fault)*1000)}
# Motion settles below conservative small velocity thresholds continuously .3 s.
for name,axis,threshold in [('linear',3,.005),('angular',4,.01)]:
 for i in np.flatnonzero(td>=fault):
  s=(td>=td[i])&(td<td[i]+.3)
  if np.all(abs(od[s,axis])<=threshold):
   more['stop'][name+'_settled_delay_s']=float(td[i]-fault);break
s=(tc>=tb[iz]);more['stop']['hold_command_max_change_rad']=float(np.max(np.linalg.norm(cmd[s]-cmd[s][0],axis=1)))
s=(to>=fault)&(to<=fault+1);more['stop']['base_displacement_first_second_m']=float(np.linalg.norm(obs[s][-1,:2]-obs[s][0,:2]))
# Dynamic lag on genuine low-frequency motion. Exclude inactive/fault tail,
# and report command high-frequency residual separately from intended motion.
dyn=[]
for a,b in [(5,50),(70,100),(95,fault-.2)]:
 grid=np.arange(a,b,.02)
 for axis,name in [(0,'linear'),(1,'angular')]:
  actual=np.interp(grid,td,od[:,3+axis]);cmdgrid=np.interp(grid,tb,bc[:,axis])
  delays=np.arange(0,.601,.02);corr=[]
  for delay in delays:
   shifted=np.interp(grid-delay,tb,bc[:,axis]);corr.append(np.corrcoef(shifted,actual)[0,1])
  i=int(np.nanargmax(corr));dyn.append({'window':[a,b],'axis':name,'effective_correlation_lag_s':float(delays[i]),'correlation':float(corr[i]),'instantaneous_correlation':float(np.corrcoef(cmdgrid,actual)[0,1])})
more['response_correlation']=dyn
# Oscillation comparison with diag03; RMS high-pass >0.5Hz on uniform 50Hz
# grid is a descriptive measure. The human forces/paths differ between bags.
osc=[]
for name,path,a,b in [('diag03','/tmp/wbmm_force_diag03_analysis/series.npz',20,30.10),('diag04_motion','/tmp/wbmm_force_diag04_analysis/series.npz',5,100),('diag04_settled_zero','/tmp/wbmm_force_diag04_analysis/series.npz',62,68)]:
 d=np.load(path);grid=np.arange(a,b,.02)
 for axis,label in [(0,'linear'),(1,'angular')]:
  u=np.interp(grid,d['tb'],d['bc'][:,axis]);y=np.interp(grid,d['td'],d['od'][:,3+axis])
  filt=butter(3,.5,btype='highpass',fs=50,output='sos');uh=sosfiltfilt(filt,u);yh=sosfiltfilt(filt,y)
  f,P=welch(u,50,nperseg=min(len(u),512));sel=(f>=.5)&(f<=3);peak=float(f[sel][np.argmax(P[sel])]);band=float(np.trapz(P[sel],f[sel]))
  osc.append({'window':name,'axis':label,'command_highpass_rms':float(np.sqrt(np.mean(uh*uh))),'feedback_highpass_rms':float(np.sqrt(np.mean(yh*yh))),'command_peak_hz_05to3':peak,'command_bandpower_05to3':band})
more['oscillation_descriptive_comparison']=osc
(OUT/'details.json').write_text(json.dumps(more,indent=2)+'\n')
np.savez_compressed(OUT/'driver_series.npz',trecv=trecv,recv=recv,tsend=tsend,send=send)
print('DETAILS',json.dumps(more,indent=2))
