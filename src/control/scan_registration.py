"""Conservative local scan alignment; estimates never command motors."""
import math
import numpy as np

def align_scans(source,target,initial=(0.,0.,0.)):
    source=np.asarray(source,dtype=float);target=np.asarray(target,dtype=float)
    if source.ndim!=2 or target.ndim!=2 or source.shape[1]!=2 or target.shape[1]!=2 or min(len(source),len(target))<50:
        raise ValueError('At least 50 two-dimensional points per scan required')
    if not np.isfinite(source).all() or not np.isfinite(target).all():raise ValueError('Nonfinite scan')
    eigen=np.linalg.eigvalsh(np.cov(target.T))
    if eigen[0]/max(eigen[1],1e-9)<.03:
        return {'accepted':False,'reason':'Nearly collinear scene; translation along wall is ambiguous'}
    angle=initial[2];rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    translation=np.array(initial[:2],dtype=float)
    for iteration in range(50):
        transformed=source@rotation.T+translation
        distances=((transformed[:,None,:]-target[None,:,:])**2).sum(axis=2)
        nearest=distances.argmin(axis=1);error=np.sqrt(distances[np.arange(len(source)),nearest])
        keep=error<=min(.10,float(np.quantile(error,.75)))
        if keep.sum()<50:return {'accepted':False,'reason':'Insufficient correspondences'}
        a=transformed[keep];b=target[nearest[keep]];ac=a.mean(axis=0);bc=b.mean(axis=0)
        u,_,vt=np.linalg.svd((a-ac).T@(b-bc));delta=vt.T@u.T
        if np.linalg.det(delta)<0:vt[-1,:]*=-1;delta=vt.T@u.T
        shift=bc-delta@ac;rotation=delta@rotation;translation=delta@translation+shift
        if np.linalg.norm(shift)<1e-5 and abs(math.atan2(delta[1,0],delta[0,0]))<1e-5:break
    transformed=source@rotation.T+translation
    distance=np.sqrt(((transformed[:,None,:]-target[None,:,:])**2).sum(axis=2).min(axis=1))
    overlap=float(np.mean(distance<.04));trimmed=np.sort(distance)[:max(1,int(len(distance)*.75))];mean=float(trimmed.mean())
    yaw=math.atan2(rotation[1,0],rotation[0,0]);accepted=overlap>=.7 and mean<.025 and np.linalg.norm(translation)<=.3 and abs(yaw)<=math.radians(15)
    return {'accepted':bool(accepted),'translation_m':translation.tolist(),'yaw_rad':yaw,'trimmed_error_m':mean,'overlap':overlap,'iterations':iteration+1,'applied_to_controller':False}
