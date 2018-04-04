#!/usr/bin/env python
import numpy as np
import rospy
import math
from boun_affordance_service.srv import *
from collections import deque
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import *

from boun_affordance_service.msg import Affordance
from boun_affordance_service.msg import AffordanceList

import tf
import copy


sample_pub = rospy.Publisher('samplePts', PoseArray, queue_size=10)
hdd_edge_pub = rospy.Publisher('hdd_edges', PoseArray, queue_size=10)
sample_poses_stamped = PoseArray()
tf_listener = tf.TransformListener()


def calc(p, hdd_vertex, N=4):
    dist_to_edges = []
    hdd_vertex = np.array(hdd_vertex)
    for i in range(0, len(hdd_vertex)):

        vec0 = hdd_vertex[i]
        vec1 = hdd_vertex[(i+1) % N]

        en = vec1-vec0
        pa = vec0 - p

        if np.dot(en, en) > 0:
            c = en*(np.dot(en, pa)/np.dot(en, en))
        else:
            c = 0

        d = pa - c
        d2 = math.sqrt(np.dot(d, d))
        #print "Distance to pt:" + str(d2)
        dist_to_edges.append(d2)
    min_dist = min(dist_to_edges)
    return min_dist > 0.01, min_dist, dist_to_edges.index(min_dist)


def calc_alt(p,hdd_vertex, N=4):
    dist_to_edges = []
    for i in range(0, len(hdd_vertex)):

        V1 = hdd_vertex[i]
        V2 = hdd_vertex[(i+1) % N]

        print V1,V2, V1[1]
        disto = abs((V2[1] - V1[1])*p[0]-(V2[0] - V1[0])*p[1] + (V2[0]*V1[1]) + (V2[1]*V1[0]))/math.sqrt((V2[1] - V1[1])**2 + (V2[0] - V1[0])**2)

        print "Distance to pt:" + str(disto)
        dist_to_edges.append(disto)
        min_dist = min(dist_to_edges)
    return min_dist > 0.01, min_dist, dist_to_edges.index(min_dist)


def dist2(p1, p2):
    return math.sqrt( (p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def sign(c1, c2):
    return 1 if c2[0]-c1[0] > 0 else -1


def polygon_edge_sample_original(polygon, sample_count=10):
    edge_total = 0

    N = len(polygon.points);

    corners = []
    for pt in polygon.points:
        corners.append([pt.x, pt.y])

    for j in range(N):
        edge_total += dist2(corners[j], corners[(j + 1) % N])

    bin_size = edge_total / sample_count

    current_point = copy.deepcopy(corners[0])
    current_edge = 0

    edge_dir = [0, 0]
    edge_dir[1] = -(corners[0][1] - corners[1][1])
    edge_dir[0] = -(corners[0][0] - corners[1][0])

    enorm = math.sqrt(edge_dir[0] ** 2 + edge_dir[1] ** 2)

    edge_dir[1] *= bin_size / enorm
    edge_dir[0] *= bin_size / enorm
    current_edge_len = dist2(corners[0], corners[1])

    sample_points = [[0 for x in range(2)] for y in range(sample_count)]

    for j in range(sample_count):
        current_edge_len -= bin_size
        sample_points[j] = copy.deepcopy(current_point)

        nextPoint = [current_point[0] + edge_dir[0], current_point[1] + edge_dir[1]]

        if current_edge_len < 0:
            current_edge = (current_edge + 1) % N
            edge_dir[1] = -(corners[current_edge][1] - corners[(current_edge + 1) % N][1])
            edge_dir[0] = -(corners[current_edge][0] - corners[(current_edge + 1) % N][0])
            enorm = math.sqrt(edge_dir[0] ** 2 + edge_dir[1] ** 2)

            offset = [0, 0]
            offset[0] = edge_dir[0] * (-current_edge_len / enorm)
            offset[1] = edge_dir[1] * (-current_edge_len / enorm)

            edge_dir[1] *= bin_size / enorm
            edge_dir[0] *= bin_size / enorm

            current_edge_len = dist2(corners[current_edge], corners[(current_edge + 1) % N]) + current_edge_len
            current_point[0] = corners[current_edge][0] + offset[0]
            current_point[1] = corners[current_edge][1] + offset[1]
        else:
            current_point = copy.deepcopy(nextPoint)

    print sample_points
    return sample_points

def pdist2(pt1,pt2):
    return math.sqrt( (pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2 )

def polygon_edge_sample(polygon, sample_count=5):
    N = len(polygon.points)
    print "N is : " , N
    """
    current_point = copy.deepcopy(polygon.points[0])
    current_edge = 0

    edge_dir = [-(polygon.points[0].x - polygon.points[1].x) , -(polygon.points[0].y - polygon.points[1].y)]
    current_edge_len = pdist2(polygon.points[0],polygon.points[1])
    enorm = math.sqrt(edge_dir[0] ** 2 + edge_dir[1] ** 2)

    edge_dir[1] *= bin_size / enorm
    edge_dir[0] *= bin_size / enorm
    
    bin_size = current_edge_len / sample_count
    """

    sample_points = []
    confidence = []
    angles = []

    #(ei+1) % N
    for ei in range(N):
        edge_dir = [-(polygon.points[ei].x - polygon.points[ (ei+1) % N ].x) , -(polygon.points[ei].y - polygon.points[(ei+1) % N].y)]
        current_edge_len = pdist2(polygon.points[ei],polygon.points[(ei+1) % N])
        enorm = math.sqrt(edge_dir[0] ** 2 + edge_dir[1] ** 2)
        
        bin_size = current_edge_len / sample_count
        percent_step = 1/math.floor(sample_count/2)
        cf = [ 1 - percent_step* abs(math.floor(sample_count/2)-ix ) for ix in range(sample_count)]
        #print "CF is" ,cf
        confidence.extend(cf)
        edge_dir[1] *= bin_size /enorm
        edge_dir[0] *= bin_size /enorm


        edge_normal = [edge_dir[1], -edge_dir[0]]

        current_point = polygon.points[ei]

        for jj in range(sample_count):
            sample_points.append(current_point)
            angles.append(edge_normal)
            current_point = copy.deepcopy(current_point)
            current_point.x += edge_dir[0]
            current_point.y += edge_dir[1]


    sample_pts = [ [pt.x, pt.y] for pt in sample_points ]

    #print sample_points
    print "Confidence : " , confidence
    return sample_pts, angles,  confidence


def handle_asc_service(req):
    #print "Affordance Service Request"
    #print "Poly : " + str(req.hdd_polygon)

    ptsArray = req.hdd_polygon.polygon.points

    hdd_vertex = deque()
    for hdpt in ptsArray:
        stamped_pt = PointStamped()
        stamped_pt.header.frame_id = req.hdd_polygon.header.frame_id
        stamped_pt.point = hdpt
        print('TF FRAME : ', req.pcb_polygon.header.frame_id)
        transformed_hdpt = tf_listener.transformPoint(req.pcb_polygon.header.frame_id, stamped_pt)
        hdd_vertex.append([transformed_hdpt.point.x, transformed_hdpt.point.y])

    pcb = req.pcb_polygon.polygon #Polygon()
    #pcb.points = [Point32(0.1, 0.1, 0), Point32(0.1, 1, 0), Point32(1.8, 1, 0), Point32(1.8, 0.1, 0)]
    samples,normals,confidence = polygon_edge_sample(pcb)


    affordance_pts = PoseArray()
    affordance_pts_poses = []
    affordance_pts_confidences = []
    computed_affordances = AffordanceList()

    sample_poses = []
    for idx,pt in enumerate(samples):
        sample_poses.append(Pose(Point(pt[0], pt[1], ptsArray[0].z), Quaternion()))
        isok, mindist, minIndex = calc(pt, hdd_vertex, 4)
        if isok:
            affordance_pts_poses.append(Pose(Point(pt[0], pt[1], ptsArray[0].z),Quaternion()))
            affordance_pts_confidences.append(confidence[idx])
        #print isok, mindist, minIndex

    sample_poses_stamped.poses = sample_poses
    sample_poses_stamped.header.frame_id = req.pcb_polygon.header.frame_id
    sample_pub.publish(sample_poses_stamped)

    affordance_pts.poses = affordance_pts_poses
    affordance_pts.header.frame_id = req.pcb_polygon.header.frame_id

    #print "Affordance!!"

    leverAffordances = Affordance()
    leverAffordances.affordance_type = "leverup"
    leverAffordances.affordance_poses = affordance_pts
    leverAffordances.confidence_values = affordance_pts_confidences
    #print "Lever done!"

    screwAffordances = Affordance()
    screwAffordances.affordance_type = "screw"
    screwAffordances.affordance_poses = None


    #print "screw done!"

    computed_affordances.affordances.append(leverAffordances)

    if screwAffordances.affordance_poses:
        computed_affordances.affordances.append(screwAffordances)


    #print "Computed Affordances!"

    resp = ComputeLeverAffordancesResponse(0,computed_affordances)
    
    #print "Response given!"

    return resp
def asc_server():

    rospy.init_node('asc_server')
    tf_listener = tf.TransformListener()

    s = rospy.Service('asc_service', ComputeLeverAffordances, handle_asc_service)
    print('Affordance service is ready!')
    rospy.spin()


if __name__ == "__main__":
    asc_server()
