"""Fit effective no-dead-time response to read-only diag03 aligned telemetry.

Usage: python3 fit_first_order.py base_aligned.csv output.json
Columns: time_s,command_v,reported_v,command_w,reported_w.
Training [10,24) s; validation >=24 s. Initialize from first measured velocity.
"""
import json, sys
import numpy as np
from scipy.optimize import least_squares

data = np.loadtxt(sys.argv[1], delimiter=',')
t = data[:,0]
result = {'description':'Effective first-order fit without explicit dead time; combines actuator and feedback response, not a pure physical inertia identification.', 'fits':{}}
def predict(params, times, commands, initial):
    tau, gain = params
    out = np.empty(len(times));out[0] = initial
    for i in range(1,len(times)):
        a = np.exp(-(times[i]-times[i-1])/tau)
        out[i] = a*out[i-1]+(1-a)*gain*commands[i-1]
    return out
for name, command, feedback in [('linear',1,2),('angular',3,4)]:
    train = (t>=10)&(t<24);valid = t>=24
    fit = least_squares(lambda p:predict(p,t[train],data[train,command],data[train,feedback][0])-data[train,feedback], [.3,1.], bounds=([.02,.2],[1.5,2.]))
    actual = data[valid,feedback]
    predicted = predict(fit.x,t[valid],data[valid,command],actual[0])
    result['fits'][name] = {'tau_s':float(fit.x[0]),'gain':float(fit.x[1]),
        'validation_rmse':float(np.sqrt(np.mean((predicted-actual)**2))),
        'instantaneous_rmse':float(np.sqrt(np.mean((data[valid,command]-actual)**2)))}
open(sys.argv[2],'w').write(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
