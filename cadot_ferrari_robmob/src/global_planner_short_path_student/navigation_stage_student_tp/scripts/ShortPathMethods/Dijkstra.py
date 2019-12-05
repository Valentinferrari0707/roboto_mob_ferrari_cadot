__author__ = 'Jacques saraydaryan'

from AbstractShortPath import AbstractShortPath
from visualization_msgs.msg import MarkerArray
import math
import rospy
# import sys
# sys.path.append('../')



class Dijsktra(AbstractShortPath):
    def __init__(self):
        print ''

    def goto(self, source, target, matrix,pub_marker,marker_array):
        ### TODO
        ###########################################################
        ################### Function Paramters ###################
        ###########################################################
        ### source: coordinate of the robot position source['x'] return the x position, source['y'] return the y position
        ###
        ### target: coordinate of the target position target['x'] return the x position, target['y'] return the y position
        ###
        ### matrix: rescaled map (including obstacles) matrix[i][j] return the value of the cell i,j of the matrix
        ###
        ### elf.MAP_OBSTACLE_VALUE: value of an obstacle into the matrix (-100)
        ###
        ### pub_marker: marker publisher to visualize information into rviz (usage pub_marker.publish(marker_array) )
        ###
        ### marker_array: marker container where new markers new to be added
        ###
        ###########################################################
        ################### Function Toolboxes ###################
        ###########################################################
        #   # create a visual information
        #   self.createFontierUnitMarker(v, marker_array)
        #
        #    # publish visual information
        #    pub_marker.publish(marker_array)
        #
        #    # create a visual information
        #    self.createClosedMarker(u, marker_array)
        #
        #
        #
        #
        #
        #
        #                       TODO
        #
        #
        #
        #
        #
        #
        ###
        ### prev:  disctionary holding node precedence
        ### CAUTION prev dictionary has to be completed as follow:
        ###
        ### prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
        ###
        ### where v['x'] return the x position of the node v in the resized map
        ### where v['y'] return the y position of the node v in the resized map
        
        # Dictionary that holds the previous node reference
        prev = {}
        # Dictionary that holds node score
        fscore = {}
        # List that holds the nodes to process
        frontier = []
        INF=9999

        # Condition to stop the path finding algo
        isEnd=False
        print 'start processing'

        for i in range (len(matrix)): 
            for j in range (len(matrix[0])):
                fscore[str(i)+'_'+ str(j)]=INF
                frontier.append({'x': i, 'y' : j})
        fscore[str(source['x'])+ '_' + str(source ['y'])]=0
        print('end initalisation phase')
        target_idx = str(target['x']) + '_' + str(target['y'])


        while len(frontier) !=0 and not isEnd:

            u=self.minScore(fscore, frontier)
            print 'current Node:'+str(u)
            frontier.remove(u)
            self.createClosedMarker(u, marker_array)
            currentNeighbors=self.getNeighbors(u,matrix)

            for v in currentNeighbors:
                
                # check that the current node has not already be processed
                if self.inU(v,frontier): 
                    # create a visual information
                    self.createFontierUnitMarker(v, marker_array)

                    current_score=fscore[str(u['x'])+'_'+str (u ['y'])]+ self.length(u, v)

                    if current_score < fscore[str(v['x'])+'_'+str (v ['y'])] :
                        self.createFontierUnitMarker(v, marker_array) 
                        fscore[str(v['x'])+'_'+str (v['y'])]=current_score
                        prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
                    if str(v['x']) + '_' + str(v['y']) == str(target['x']) + '_' + str(target['y']):
                        # end the path computation
                        isEnd=True


            pub_marker.publish(marker_array)
            marker_array=MarkerArray()
            # wait before next iteration
            rospy.sleep(0.2)
        print str(prev)
        return prev

    def minScore(self, fscore, frontier) :
        min=9999
        min_coord=''
        for n in frontier : 
            if(fscore[str(n['x'])+'_'+str (n ['y'])] < min): 
                min=fscore[str(n['x'])+'_'+str (n ['y'])]
                min_coord=n
        return min_coord


    def getNeighbors(self, currentNode, matrix):
        """ Compute Neighbors of the current point, Return the list of the point neighbors in Cfree"""
        x_c = currentNode['x']
        y_c = currentNode['y']
        neighbors = []
        self.checkAndAdd(neighbors, x_c + 1, y_c, matrix)
        self.checkAndAdd(neighbors, x_c, y_c + 1, matrix)
        self.checkAndAdd(neighbors, x_c - 1, y_c, matrix)
        self.checkAndAdd(neighbors, x_c, y_c - 1, matrix)
        return neighbors

    def checkAndAdd(self, neighbors, x, y, matrix):
        """ Check that the candidate neighbor is valid == not an obstacle, in current bound, add the nieghbor node to the node list"""
        if (x > 0 and x < self.map_width and y > 0 and y < self.map_height):
            if (matrix[y][x] != self.MAP_OBSTACLE_VALUE):
                neighbors.append({'x': x, 'y': y})
        return neighbors


    def inU(self,v,frontier):
        """ Check if the node is into the list, return boolean """
        return v in frontier

    def length(self, u, v):
        """Compute the distance between """
        return abs(v['x'] - u['x']) + abs(v['y'] - u['y'])
