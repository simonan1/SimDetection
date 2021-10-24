import cv2 as cv
import numpy as np

MIN_MATCH_COUNT = 7 #for valid match at matchTwoImages
NN_RATIO = 0.6 # for NN ratio test at matchTwoImages was 0.8
QUALITY_LEVEL = 0.05 # for goodFeaturesToTrack was 0.01
MIN_DISTANCE = 10 # for goodFeaturesToTrack was 10

HFOV = 58.3#54.49#23.47 #70.42   #[deg] logitech c920, diagonal 78 deg
HRES = 640 #[pix] logitech c920  640x480

knn_matcher = cv.BFMatcher()

def extractFeaturesDescriptions(img):
    sift = cv.xfeatures2d.SIFT_create() # pip3 install opencv-contrib-python==3.4.2.16 
    kps, des = sift.detectAndCompute(img,None)
    return kps, des 

def matchTwoImagesKnn(des1,des2):
    global knn_matcher

    matches = knn_matcher.knnMatch(des1,des2,k=2)
    n_match = len(matches); n_good = 0; good =[]; 

    if n_match < MIN_MATCH_COUNT: n_match, n_good, good
    # Apply ratio test
    for m,n in matches:
    	if m.distance < NN_RATIO*n.distance:		
    		good.append(m)
    n_good = len(good)
    # print('Length matches is '+ str(n_match) + ', Length of good is '+ str(n_good))
    return n_match, n_good, good   

def computeHomography(best_matches, kp1, kp2):
	src_pts = np.float32([ kp2[m.trainIdx].pt for m in best_matches ]).reshape(-1,1,2)
	dst_pts = np.float32([ kp1[m.queryIdx].pt for m in best_matches ]).reshape(-1,1,2)

	M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,ransacReprojThreshold = 5,maxIters = 2000,confidence = 0.995)
	matchesMask = mask.ravel().tolist()
	# print(matchesMask)
	n_inliers = sum(matchesMask) # inl_pts = dst_pts[np.array(matchesMask)]
	match_list = [ kp1[m.queryIdx] for m in best_matches ]

	# inlier_matches_list = [item for i, item in enumerate(best_matches) if matchesMask[i]] # list of matches :from best matches select inliers only according to the mask
	# in_kp = [ kp1[m.queryIdx] for m in inlier_matches_list ]# list of keypoints

	# #for tuning 
	# dst_octave =   [ kp1[m.queryIdx].octave for m in best_matches ]
	# dst_response = [ kp1[m.queryIdx].response for m in best_matches ]
	# dst_size =     [ kp1[m.queryIdx].size for m in best_matches ]
	# best_kp_param = zip(dst_octave,dst_response,dst_size)

	return M,n_inliers,match_list,#inlier_matches_list,in_kp,best_kp_param

def computePoint(homo_mat,x,y):
	pt_in_ref = np.float32( [[x,y]] ).reshape(-1,1,2) #pt_in_ref = np.float32(ref_pnts[4,:]).reshape(-1,1,2)
	point_in_scene =  cv.perspectiveTransform(pt_in_ref,homo_mat)
	return point_in_scene

def pixToAzimuth(h_pix,h_fov,h_res):
	ifov = h_fov/h_res
	pix_from_center_to_right = h_pix - h_res/2
	# print("pix_from_center_to_right = ",pix_from_center_to_right)
	az_deg = ifov*pix_from_center_to_right
	return az_deg