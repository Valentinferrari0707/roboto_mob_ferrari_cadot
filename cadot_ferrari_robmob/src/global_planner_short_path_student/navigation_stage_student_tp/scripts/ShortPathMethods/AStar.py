__author__ = 'Jacques saraydaryan'

from AbstractShortPath import AbstractShortPath
import math
import rospy
from visualization_msgs.msg import MarkerArray
# sys.path.append('../')



class AStar(AbstractShortPath):
    def __init__(self):
        print ''

    def goto(self, source, target, matrix,pub_marker,marker_array):
        prev={}
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


        prev = {}
        # Dictionary that holds node scores
        fscore, gscore = {}, {}
        # List that holds the nodes to process
        closedList, openList = [], []
        INF=9999

        # Condition to stop the path finding algo
        isEnd=False
        print 'start processing'

        for i in range (len (matrix)):
            for j in range (len (matrix[0])):
                gscore[str(i)+ "_"+ str(j)]=INF
                fscore[str(i)+"_"+ str(j)]=INF

        fscore[str(source['x'])+"_"+str(source['y'])]=0     
        gscore[str(source['x'])+"_"+str(source['y'])]=0
        openList=[source]
        target_idx=str(target['x'])+"_"+str(target['y'])
        while len(openList)!= 0 and not isEnd : 
            u=self.minScore(fscore, openList)
            u_idx=str(u['x'])+'_'+str(u['y'])
            print u_idx, target_idx
            if u_idx==target_idx:
                break
            
            openList.remove(u)
            closedList.append(u)
            self.createClosedMarker(u, marker_array)
            currentNeighbor=self.getNeighbors(u, matrix)

            for v in currentNeighbor:
                v_idx=str(v['x'])+"_"+str(v['y'])

                if self.inU(v, closedList):
                    continue
                v_score=gscore[u_idx]+self.length(u,v)

                if not self.inU(v, openList):
                    openList.append(v)
                elif v_score >= gscore[v_idx]:
                    continue
                
                self.createFontierUnitMarker(v, marker_array)
                gscore[v_idx]=v_score
                fscore[v_idx]=gscore[v_idx]+ self.hn(matrix ,v, target)
                prev[v_idx]=u_idx

            
                pub_marker.publish(marker_array)
                marker_array=MarkerArray()
                    # wait before next iteration
                rospy.sleep(0.02)

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

    def length(self, u, v):
        """Compute the distance between """
        return abs(v['x'] - u['x']) + abs(v['y'] - u['y'])


    def inU(self,v,frontier):
        """ Check if the node is into the list, return boolean """
        return v in frontier

    def hn(self,matrix,source,destination):
        """Compute the distance between the given node and the target, the result is an estimation of the distance without taking into account obstacles """
        return math.sqrt(math.pow(source['x']-destination['x'],2)+math.pow(source['y']-destination['y'],2))