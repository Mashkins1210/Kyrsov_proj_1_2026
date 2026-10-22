"""Match reviewed training photographs; never infer identity from color alone."""
import cv2
import numpy as np

class LandmarkMatcher:
    def __init__(self, reference):
        self.orb=cv2.ORB_create(nfeatures=1500,edgeThreshold=12,fastThreshold=10)
        self.reference=reference
        self.keypoints,self.descriptors=self.orb.detectAndCompute(cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY),None)
        if self.descriptors is None or len(self.keypoints)<20:
            raise ValueError('Reference has too few visual features')

    def detect(self, frame):
        # USB/decoder faults can leave most of a YUYV frame solid green.
        sample=frame[::4,::4]
        green=(sample[:,:,0]<3)&(sample[:,:,1]>80)&(sample[:,:,2]<3)
        if float(np.mean(green))>.25:return None
        points,descriptors=self.orb.detectAndCompute(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),None)
        if descriptors is None:return None
        pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(self.descriptors,descriptors,k=2)
        good=[pair[0] for pair in pairs if len(pair)==2 and pair[0].distance<.72*pair[1].distance]
        if len(good)<12:return None
        source=np.float32([self.keypoints[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        destination=np.float32([points[m.trainIdx].pt for m in good]).reshape(-1,1,2)
        homography,mask=cv2.findHomography(source,destination,cv2.RANSAC,3)
        if homography is None or mask is None:return None
        inliers=int(mask.sum())
        if inliers<12 or inliers/len(good)<.5:return None
        source_inliers=source[mask.ravel().astype(bool)]
        reference_area=self.reference.shape[0]*self.reference.shape[1]
        coverage=abs(cv2.contourArea(cv2.convexHull(source_inliers)))/reference_area
        if coverage<.15:return None
        h,w=self.reference.shape[:2]
        polygon=cv2.perspectiveTransform(np.float32([[0,0],[w,0],[w,h],[0,h]]).reshape(-1,1,2),homography)
        if not np.isfinite(polygon).all() or not cv2.isContourConvex(polygon):return None
        area=abs(cv2.contourArea(polygon));fh,fw=frame.shape[:2]
        if not 1500<area<fh*fw*.8:return None
        if np.any(polygon[:,:,0]<0) or np.any(polygon[:,:,0]>=fw) or np.any(polygon[:,:,1]<0) or np.any(polygon[:,:,1]>=fh):return None
        return {'polygon':polygon.reshape(-1,2).tolist(),'inliers':inliers,'matches':len(good),'reference_coverage':float(coverage),'source':'reviewed_visual_template','recognition_verified':False}

