#!/usr/bin/python

# basic python imports
import numpy as np
from math import tan,atan,degrees,radians

# ros imports
import rospy
from std_msgs.msg import Int32
# from sensor_msgs.msg import CameraInfo

# custom ros msg output
from camera_lidar_fusion.msg import FusedPerson,FusedPersonArray,LidarPointCloud

# the lidar listener class
from LidarListener import *
# the dnn detection listener class
from DnnListener import *

# begin of main
def fusion_main():
    # initialize
    RosNodeName = 'camera_lidar_fusion'
    print 'initializing {}...'.format(RosNodeName)
    rospy.init_node(RosNodeName,anonymous=True)


    # refresh rate of data fusion, can be tuned
    hz = 5.0
    rate = rospy.Rate(hz)

    # publishers
    RosTopicName = '/human_state_measurement'
    pubFusion = rospy.Publisher('/'+RosNodeName+RosTopicName, FusedPersonArray, queue_size=10)
    RosTopicName = '/num_detected_human'
    pubPersonCount = rospy.Publisher('/'+RosNodeName+RosTopicName, Int32, queue_size=10)
    # more publishers for lidar point clouds
    RosTopicName = '/lidar_points'	# lidar point cloud in front of the vehicle
    pubLidarPoints = rospy.Publisher('/'+RosNodeName+RosTopicName, LidarPointCloud, queue_size=10)

    # msg output init
    FusedMsg = FusedPersonArray()
    # more msg init for lidar
    LidarPoints = LidarPointCloud()

    # grab sensor data
    lidar_listener = LidarListener()
    dnn_listener = DnnListener()

    print 'running...'

    # sleep for 0.1s for init
    rate_init = rospy.Rate(10.0)
    rate_init.sleep()

    while not rospy.is_shutdown():

        # data from sensors
        NumOfPerson = dnn_listener.PersonCount
        
        # init output
	CurrentTime = rospy.Time.now()
        FusedMsg.header.stamp = CurrentTime    # may use other tstamps
        FusedMsg.persons = []
	# init lidar cloud output
	LidarPoints.header.stamp = CurrentTime
	LidarPoints.range_data = []
		
        # PersonMsg = []
        FusedClusters = [[]]*NumOfPerson
		
	# added May 14 2019
	# front view angle for raw lidar cloud points
	FrontViewAngle = 60.			
	
        # place holder for data fusion
        # for each detected person:
        # step 1: transfer the camera pixel position to lidar angle
        # step 2: fuse the corresponding lidar measurement points
        # step 3: data association and state filtering (another ROS node)

        # find the angular person position in astra
        PersonPositionAngle = []
        for i in range(NumOfPerson):
            PersonPosition = dnn_listener.DetectedPerson[i]
            xMin = PersonPosition[0]
            xMax = PersonPosition[1]
            # narrow the search angle if set to positive value
            ThetaScale = 0.0
            ThetaMin = pixel2angle(xMin)+ThetaScale
            ThetaMax = pixel2angle(xMax)-ThetaScale
            PersonPositionAngle.append((ThetaMin,ThetaMax))
        # end for
        
        # find the corresponding lidar datapoints
        LidarPersonDataList = []
        for i in range(NumOfPerson):
            LidarPersonDataList.append([])
        PersonDataCount = [0.0]*NumOfPerson
        for LidarRangeData in lidar_listener.RangeData:
            degree = LidarRangeData[0]
            # added May 14 2019
            # lidar cloud point output in the front viewing angle
            if -1.*FrontViewAngle <= degree <= FrontViewAngle:
		dist = LidarRangeData[1]
		point = FusedPerson()
		point.distance = dist
		point.angle = degree
                point.label = 1
                LidarPoints.range_data.append(point)
            # Original code
            for i in range(NumOfPerson):
                if PersonPositionAngle[i][0] <= degree <= PersonPositionAngle[i][1]:
                    LidarPersonDataList[i].append(LidarRangeData)
                    PersonDataCount[i] += 1.0
                    # added May 14 2019
                    # if -1.*FrontViewAngle <= degree <= FrontViewAngle:
                    #    point.label = -1
            # endfor
            # added May 16 2019
            # all static obstacles outside the human position angle
            # if point.label == 1:
            #    LidarPoints.range_data.append(point)
        # endfor
        # added May 14 2019
	pubLidarPoints.publish(LidarPoints)

        # fuse the lidar datapoints
        # version 0.0.1: take average
        # measurement association: discard the invalid measurements
        # discard the data points far away from the others
        # asusme most data points are correct, may cause problem if there are more background data points
        RangeDiscardThres = 0.5         # cluster the lidar distance by threshold
        for i in range(NumOfPerson):
            # init ros msg
            PersonMsg = FusedPerson()
            
            # unclustered lidar data
            LidarPersonData = LidarPersonDataList[i]
            DataLength = len(LidarPersonData)
            LidarPersonData.sort(key=sortRangeKey,reverse=True)

            # cluster the data by interval threshold
            ClusterIndex = [0]
            ClusterLength = []
            ClusterData = []
            for j in range(DataLength-1):
                if LidarPersonData[j][1]-LidarPersonData[j+1][1]>RangeDiscardThres:
                    ClusterIndex.append(j)
                    ClusterLength.append(ClusterIndex[-1]-ClusterIndex[-2])
                    ClusterData.append(LidarPersonData[ClusterIndex[-2]:ClusterIndex[-1]])
            ClusterIndex.append(DataLength)
            ClusterLength.append(ClusterIndex[-1]-ClusterIndex[-2])
            ClusterData.append(LidarPersonData[ClusterIndex[-2]:ClusterIndex[-1]])

            # average each of the clusters
            NumOfClusters = len(ClusterLength)
            for j in range(NumOfClusters):
                FusedClusters[i].append(FusedPerson())
                ClusterSeg = ClusterData[j]
                NormFactor = len(ClusterSeg)
                FusedClusters[i][j].angle = 0.0
                FusedClusters[i][j].distance = 0.0
                for Data in ClusterSeg:
                    FusedClusters[i][j].angle += Data[0]/NormFactor
                    FusedClusters[i][j].distance += Data[1]/NormFactor

            # use the largest cluster as person data
            # update note: output all the fused clusters and perform data association
            PersonDataLength = max(ClusterLength)
            PersonDataIndex = ClusterLength.index(PersonDataLength)
            LidarPersonData = FusedClusters[i][PersonDataIndex]
            PersonMsg.angle = LidarPersonData.angle
            PersonMsg.distance = LidarPersonData.distance
            # publish msg
            FusedMsg.persons.append(PersonMsg)
        # endfor

        # publish fused message
        pubFusion.publish(FusedMsg)
        pubPersonCount.publish(NumOfPerson)

        # printing output
        print '========================================output section======================================='
        print 'time stamp: ', lidar_listener.tStamp
        print 'num of detected person: ', NumOfPerson
        #for i in range(NumOfPerson):
        #    print 'angles of detected person:', FusedMsg.persons[i].angle
        #    print 'distance of detected person: ', FusedMsg.persons[i].distance
        #    for j in range(NumOfClusters):
        #        print '--------clustered data points----------'
        #        print 'angle:', FusedClusters[i][j].angle
        #        print 'distance: ', FusedClusters[i][j].distance
        
        rate.sleep()

# end of main

############################################################################################################################
# external functions

def pixel2angle(x):
    # convert x(u) pix to angle 
    # astra specs, 640*480, can be read from CameraInfo
    uMax = 640.0
    # vMaxAstra = 480.0
    FieldOfView = 60.0
    
    uMax = 0.5*uMax   # half image size
    FieldOfView = radians(0.5*FieldOfView)   # half angle range
    theta = atan((uMax-x)/uMax*tan(FieldOfView))
    theta = -degrees(theta)     # clockwise rotation
    return theta

def sortRangeKey(data):
    # key for range sorting
    return data[1]

#############################################################################################################################
# running
if __name__ == '__main__':
    try:
        fusion_main()
    except rospy.ROSInterruptException:
        pass


# simple running version
# fusion_main()
