import importlib.util, numpy as np
spec=importlib.util.spec_from_file_location("cal","/tmp/calibrate_fisheye.py"); cal=importlib.util.module_from_spec(spec); spec.loader.exec_module(cal)
cams=["left","right","bright","bleft"]
K=np.array([[530.,0.,960.],[0.,530.,600.],[0.,0.,1.]])
cals={c:{"K":K.copy(),"D":np.zeros((4,1)),"xi":1.0} for c in cams}
I=np.eye(4)
cal.write_yaml("/tmp/placeholder_float.yaml",cams,cals,{("left","right"):I,("right","bright"):I,("bright","bleft"):I},"omni",(1920,1200))
print(open("/tmp/placeholder_float.yaml").read()[:500])
