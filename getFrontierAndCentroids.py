#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 28 21:34:39 2021

@author: facestomperx
"""
#--------Include modules---------------
from copy import copy
#import rclpy
#from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2
import scipy.stats

from geometry_msgs.msg import Point


occ_bins_all_edge = [-1,0.1,100]
occ_bins_occupied = [-1,52,100] #51
occ_bins_within = [-1,0,100] 
original_occ_bins = [-1, 0, 50,100]
cannyThresh = 250
ALTHRESH = 10
DILATE_PIXELS = 5
ERODE_PIXELS = 7

def worldToMap(coords, world_origin, resolution):
    #print("World to map")
    #convert_offset = 0.5
    if (coords.x < world_origin.x or coords.y < world_origin.y):
        print("Does this imply smth")
    mx = round((coords.x - world_origin.x) / resolution)
    my = round((coords.y - world_origin.y) / resolution)
    return (my,mx)


def binMap(mapData):
    data=mapData.data
    w=mapData.info.width
    h=mapData.info.height
    checkOcc = np.uint8(np.array(data).reshape(h,w))
    #print(checkOcc.shape)
    occ_counts, edges, binnum = scipy.stats.binned_statistic(np.array(data), np.nan, statistic='count', bins=occ_bins_occupied)
    
    """
    for idx,x in enumerate(binnum):
        if (binnum[idx] == 1):
            binnum[idx] = 0
        elif (binnum[idx] == 2):
            binnum[idx] = 25
        else:
            binnum[idx] = 255
     """
    for idx,x in enumerate(binnum):
        if (binnum[idx] == 1):
           binnum[idx] = 0
        else:
           binnum[idx] = 255
    binned_grid = np.uint8(binnum.reshape(h,w))
    return binned_grid

def widenBin(binned_grid, pixels):
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(pixels,pixels))
    return cv2.dilate(binned_grid,element)

def getfrontier(mapData,botPos, binned_grid):
    print("Getting frontiers")
    data=mapData.data
    w=mapData.info.width
    h=mapData.info.height
    #print("Map height: ", h)
    #print("Map width: ", w)
    resolution=mapData.info.resolution
    Xstartx=mapData.info.origin.position.x
    Xstarty=mapData.info.origin.position.y
    checkOcc = np.uint8(np.array(data).reshape(h,w))
    
    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ERODE_PIXELS,ERODE_PIXELS))
    #img4 = cv2.erode(binned_grid,element)
    #return binned_grid
    #ret,img2 = cv2.threshold(checkOcc,2,255,0)
    
    
    #This should always be thicker, cuz it' the one being minused off. On the other hand, the canny output cant be too thick cuz that's the one that remains
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(DILATE_PIXELS,DILATE_PIXELS))
    img4 = cv2.dilate(binned_grid,element)
    
    
    
    #return img4 100, 350
    canny_output = cv2.Canny(checkOcc, 50, 350)#cv2.Canny(checkOcc, 225, 250)#cv2.Canny(checkOcc, 100, 150), 125, 350   25,350   #cv2.Canny(checkOcc, 225, 250) #works on gazebo
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(4,4))  #(3,3) maybe too thick so may produce random spots. Thicker may form longer contoues but that isn't priority. Correctness is priority
    # (1,1) looks clean but calculating moments inaccurate, will result in [0,0] cuz not enclosed
    #(2,2) abit thick but abit no choice ah. Is there better ways to find moments?
    # Possibility: (2,2) for moments, (1,1) for final
    #Correction: Those white spots appear anyways even with 2,2
    
    
    thick_canny_output = cv2.dilate(canny_output ,element)
#    thick_canny_output = edge_output
    
    
    #return canny_output
    #getUnknownEdgesGrid(checkOcc,canny_output,w,h)
    edge_output = getUnknownEdgesGrid2(thick_canny_output,img4,w,h)
    
    
    #to surpress small frontiers?
    #https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html 
    # This link kinda prompts the possiblity of making my own kernel
    #https://stackoverflow.com/questions/51009126/opencv-how-to-correctly-apply-morphologyex-operation
    
    """
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    edge_output = cv2.dilate(edge_output,element)
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))   #(2,2)   
    edge_output = cv2.erode(edge_output,element)
    """
    
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3,3))
    edge_output = cv2.morphologyEx(edge_output, cv2.MORPH_OPEN, kernel)

    
    #return edge_output
    contours, hierarchy = cv2.findContours(edge_output,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    
    # erode abit so that can see the exact frontier? Not exacly working
    
    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))  #MORPH_ELLIPSE,(1,1),MORPH_CROSS
    #edge_output = cv2.erode(edge_output,element)
    
    
    
    #main_contour = contours[3]
    
    #contoured_image = cv2.drawContours(edge_output, [main_contour], 0, (0,255,0), 1)
    contoured_image = cv2.drawContours(edge_output, contours, -1, (255,255,255), 1) #when it outputs 0,0 means failed to find moment?
    #return contoured_image
    all_pts=[]
    all_world_pts = []
    #print(len(contours))
    if len(contours)>0:
        for i in range(0,len(contours)):
                cnt = contours[i]
                #print("Cnt: ", cnt)
                M = cv2.moments(cnt)
                cx = int(M['m10']/(M['m00']+ 1e-5))
                cy = int(M['m01']/(M['m00']+ 1e-5))
                if(cx == 0 and cy == 0 or cx >= h or cy >= w):
                    #discard frontier
                    continue
                xr=cx*resolution+Xstartx
                yr=cy*resolution+Xstarty
                pt=[np.array((cy,cx))]
                w_pt = Point()
                w_pt.x = xr
                w_pt.y = yr
                #w_pt = [np.array((xr,yr))]
                if len(all_pts)>0:
                    all_pts=np.vstack([all_pts,pt])
                    all_world_pts.append(w_pt)
                    #all_world_pts = np.vstack([all_world_pts,w_pt])
                else:
                    all_pts=pt
                    all_world_pts = [w_pt]
    #print(all_pts)
    
    res = np.zeros((h,w),dtype=np.uint8)
    
    for points in all_pts:
        res[points[0]][points[1]] = 255
        
    #print("World origin: ", Xstartx, Xstarty)
    """
    for world_points in all_world_pts:
        print(world_points[0], " ", world_points[1])
    """
    
    
    
    
    #print("Bot position in map: ", botPos)
    mBotPos = worldToMap(botPos,mapData.info.origin.position,resolution)
    #print("Bot position in cells: ", mBotPos[0], mBotPos[1])
    #print("Bot position in map again: ", utils.mapToWorld2(mBotPos, mapData.info.origin.position,resolution))
    
    res[mBotPos[0]][mBotPos[1]] = 255
    #just for makring positions
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(7,7)) #5,5 3,3
    res = cv2.dilate(res,element)
    contoured_image += res
    
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(2,2)) #5,8,5MORPH_CROSS, MORPH_ELLIPSE (1,1)
    img5 = cv2.dilate(binned_grid,element)
    
    
    
    return contoured_image,all_pts, all_world_pts, mBotPos,binned_grid,img5 #img4,binned_grid,canny_output,thick_canny_output, img5
    #o=cv2.bitwise_not(o) 
    #res = cv2.bitwise_and(o,edges)
    

def mapToWorld(coords,world_origin, resolution):
    #I believe my World to map is working, but map to world is fked
    wx = world_origin.x + coords[0] * resolution
    wy = world_origin.y + coords[1] * resolution
    return (wx,wy)


""" def worldToMap(coords, world_origin, resolution):
    if (coords.x < world_origin.x or coords.y < world_origin.y):
        print("Does this imply smth")
    mx = int((coords.x - world_origin.x) // resolution)
    my = int((coords.y - world_origin.y) // resolution)
    return (mx,my)
 """


def getUnknownEdgesGrid2(canny_grid, binned_grid, width, height):
    #must line up properly so nah 
    res = []
    print(canny_grid.shape)
    print(binned_grid.shape)
    for row in range(height):
        temprow = []
        for col in range(width):
            if (canny_grid[row][col]  < binned_grid[row][col]):
                temp = 0
            else:
                temp = canny_grid[row][col] - binned_grid[row][col]
            temprow.append(temp)
        res.append(temprow)
        #print(binned_grid[row])
    return np.array(res,dtype=np.uint8)


def getUnknownEdgesGrid(occupancy_grid, edges_grid, width, height):
    EXPLORE_RANGE = 3
    padded_occupancy = copy(occupancy_grid)
    horizontal_padding = np.zeros((height,EXPLORE_RANGE),dtype=np.uint8)
    vertical_padding = np.zeros((EXPLORE_RANGE,width + EXPLORE_RANGE * 2),dtype=np.uint8)
    padded_occupancy = np.hstack((horizontal_padding,padded_occupancy,horizontal_padding))
    padded_occupancy = np.vstack((vertical_padding,padded_occupancy,vertical_padding))
    checked_cells = set() #set of coordinates in tuple
    res = [] #grid of edges
    for row in range(height):
        for col in range(width):
            if(edges_grid[row][col] != 0):
                print(row,col)
                checkPixels(occupancy_grid,EXPLORE_RANGE,(row,col))
    
def checkPixels(occupancy_grid,explore_range,coord):
    #heck care about zero index, cuz got enough padding. Actually good point hor, pad image then wont have zero index
    print("Checking coord:" + str(coord[0]) + " " + str(coord[1]))
    curPixel_x = coord[0]
    curPixel_y = coord[1]
    #check for vertical boundary
    if(occupancy_grid[curPixel_x-explore_range][curPixel_y] != occupancy_grid[curPixel_x+explore_range][curPixel_y]):
        print(occupancy_grid[curPixel_x-explore_range][curPixel_y])
        print(occupancy_grid[curPixel_x+explore_range][curPixel_y])
    
    
