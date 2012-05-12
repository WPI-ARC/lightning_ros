import roslib
roslib.load_manifest("lightning")
import rospy
import actionlib

from tools.PathTools import InvalidSectionWrapper

import math
import string
import random
import pickle
import os.path
import sys
import time

TWO_PI = 2*math.pi
START, GOAL = 0, -1
ROOT_NAME = "root_"
SG_ROOT_NAME = "sgroot_"
PATH_TREE_NAME = "info_tree.pickle"
SG_TREE_NAME = "info_sgtree.pickle"
DRAW_PATH_NAME = "draw_points"
LIBRARY_DIR_PREFIX = "paths_"
INFO_FILE_NAME = "_lib_info"

class TreeNode:
    def __init__(self, name, parent):
        self.name = name;
        self.left, self.right = None, None;
        self.parent = parent;

    def isLeaf(self):
        return self.left is None and self.right is None;

class PathTreeNode(TreeNode):
    def __init__(self, name, parent, splitValue=0, splitIndex=0, splitState=START):
        TreeNode.__init__(self, name, parent);
        self.splitValue = splitValue;
        self.splitIndex = splitIndex;
        self.splitState = splitState; #START if splitIndex is for a start state value, GOAL if it is for goal state value

class SGTreeNode(TreeNode):
    def __init__(self, name, parent):
        TreeNode.__init__(self, name, parent);

class PathLibrary:
    def __init__(self, pathLibraryDir, stepSize, nodeSize=100, sgNodeSize=1000, dtwDist=10):
        #variables that are the same for all libraries
        if pathLibraryDir[-1] == '/':
            pathLibraryDir = pathLibraryDir[:-1]
        self.pathLibraryDir = pathLibraryDir
        self.stepSize = stepSize
        self.nodeSize = nodeSize
        self.sgNodeSize = sgNodeSize
        self.dtwDist = dtwDist
        self.invalidSectionWrapper = InvalidSectionWrapper()
        self.SPLIT_PATHS_FUNCT = self._largestRangeSplit #self._consecutiveSplit
        
        #variables that are different for each library
        self.nextPathId = 0
        self.currentLibName = ""
        self.currentNumDims = 0
        self.tree, self.sgTree = None, None
        self.sgCache = dict()

    def _initPathsFile(self, filename):
        f = open(self._getFullFilename(filename), 'w');
        f.write("%s\n" % (string.zfill('0', len(str(self.nodeSize)))));
        f.close();
    
    def _initSGsFile(self, sgNodeName):
        f = open(self._getFullFilename(sgNodeName), 'w');
        f.write("%s\n" % (string.zfill('0', len(str(self.sgNodeSize)))));
        f.close();

    def _readLibrarySize(self):
        count = 0
        for f in os.listdir(self._getFullLibName()):
            if f.find(SG_ROOT_NAME) != -1:
                count += self._readNumSGsFromFile(f)        
        return count

    def _getNextPathId(self):
        maxId = 0
        for f in os.listdir(self._getFullLibName()):
            if f.find(SG_ROOT_NAME) != -1:
                maxIdForFile = self._getMaxPathId(f)
                if maxIdForFile > maxId:
                    maxId = maxIdForFile
        return maxId+1

    def _getMaxPathId(self, filename):
        f = open(self._getFullFilename(filename), 'r')
        numSGs = int(f.next().strip())
        currentMax = 0
        for line in f:
            pid = int(line.strip().split(" ")[0])
            if pid > currentMax:
                currentMax = pid
        f.close()
        return currentMax
        
    def _loadTrees(self):
        treeLoadFile = open(self._getFullFilename(PATH_TREE_NAME), 'r');
        self.tree = pickle.load(treeLoadFile);
        treeLoadFile.close();
        sgTreeLoadFile = open(self._getFullFilename(SG_TREE_NAME), 'r');
        self.sgTree = pickle.load(sgTreeLoadFile);
        sgTreeLoadFile.close();

    def _storeCurrentSGTree(self):
        sgTreeFile = open(self._getFullFilename(SG_TREE_NAME), 'w')
        pickle.dump(self.sgTree, sgTreeFile);
        sgTreeFile.close();
        
    def _storeCurrentPathTree(self):
        treeFile = open(self._getFullFilename(PATH_TREE_NAME), 'w')
        pickle.dump(self.tree, treeFile);
        treeFile.close();

    def _storeCurrentTrees(self):
        self._storeCurrentPathTree();
        self._storeCurrentSGTree();

    def _removePathFile(self, filename):
        os.remove(self._getFullFilename(filename));
    
    def _removeSGFile(self, filename):
        os.remove(self._getFullFilename(filename));

    def _getRobotNameAndJoints(self, libName):
        f = open(self._getFullFilename(INFO_FILE_NAME, libName), 'r')
        robotName = f.next().strip()
        jointNames = f.next().strip().split('|')
        return (robotName, jointNames)

    def _findLibrary(self, robotName, jointNames):
        for lib in os.listdir(self.pathLibraryDir):
            if self._getRobotNameAndJoints(lib) == (robotName, jointNames):
                return lib
        return None

    def _loadLibrary(self, robotName, jointNames):
        lib = self._findLibrary(robotName, jointNames)
        if lib is None:
            return False
        if lib != self.currentLibName:
            self.currentLibName = lib
            self._loadTrees()
            self.nextPathId = self._getNextPathId()
            self.currentNumDims = len(jointNames)
        return True

    def _createAndLoadNewLibrary(self, robotName, jointNames):
        takenIds = set()
        for lib in os.listdir(self.pathLibraryDir):
            takenIds.add(int(lib.split("_")[1]))
        newId = 0
        while True:
            if newId in takenIds:
                newId += 1
            else:
                break
        self.currentLibName = "%s%i_%s" % (LIBRARY_DIR_PREFIX, newId, robotName)
        rospy.loginfo("Path library: creating new library: %s" % (self.currentLibName))
        os.mkdir(self._getFullLibName())
        infoFile = open(self._getFullFilename(INFO_FILE_NAME), 'w')
        infoFile.write("%s\n" % (robotName))
        infoFile.write("%s\n" % ('|'.join(jointNames)))
        self.tree = PathTreeNode(ROOT_NAME, None);
        self.sgTree = SGTreeNode(SG_ROOT_NAME, None);
        self._storeCurrentTrees();
        self._initPathsFile(self.tree.name);
        self._initSGsFile(self.sgTree.name);
        self.nextPathId = 1
        self.currentNumDims = len(jointNames)
    
    def _deleteLibraryFiles(self, libName):
        for f in os.listdir(self._getFullLibName(libName)):
            os.remove(self._getFullFilename(f, libName))

    def deleteLibrary(self, robotName, jointNames):
        lib = self._findLibrary(robotName, jointNames)
        if lib is None:
            rospy.loginfo("Path library: library for robot %s and joints %s does not exist" % (robotName, jointNames))
            return False
        else:
            rospy.loginfo("Path library: deleting library for robot %s and joints %s" % (robotName, jointNames))
            self._deleteLibraryFiles(lib)
            os.rmdir(self._getFullLibName(lib))
            return True
        
    def deletePathById(self, pid, robotName, jointNames):
        self._loadLibrary(robotName, jointNames)
        current = None
        newPaths = []
        for f in os.listdir(self._getFullLibName()):
            if f.find(ROOT_NAME) == 0:
                for pathId, path in self._getPaths(f):
                    if pathId == pid:
                        current = f
                        break
            if current is not None:
                rospy.loginfo("Path library: deleting path in %s" % (current))
                for pathId, path in self._getPaths(current):
                    if pathId != pid:
                        newPaths.append((pathId, path))
                self._makePathsFile(current, newPaths)
                sgNodeName = self._getSGLeafByPathName(current).name
                sgs = self._getSGs(sgNodeName)
                newSGs = []
                for sgWithId in sgs[current]:
                    if sgWithId[0] != pid:
                        newSGs.append(sgWithId)
                sgs[current] = newSGs
                self.sgCache = sgs
                self._makeSGsFile(self._getSGLeafByPathName(current).name, sgs)
                return True
        rospy.loginfo("Path library: path with id %i does not exist in library for robot %s and joints %s" % (pid, robotName, jointNames))
        return False
                         
    #to force store a path, set prevPath to None
    def storePath(self, newPath, robotName, jointNames, prevPath=None):
        if not self._loadLibrary(robotName, jointNames):
            self._createAndLoadNewLibrary(robotName, jointNames)
        newPath = self._normalizePath(newPath);
        rospy.loginfo("Path Library: got a path of %i points to store" % (len(newPath)))
        if prevPath is None or len(prevPath) == 0 or self._DTWDistance(prevPath, newPath) > self.dtwDist:
            leafNode = self._getPathLeafBySG(newPath[0], newPath[-1]);
            self._addPath(leafNode.name, newPath);
            rospy.loginfo("Path Library: done storing path with id %i", self.nextPathId-1)
        else:
            rospy.loginfo("Path Library: did not need to store this path")

    def retrievePath(self, s, g, n, robotName, groupName, jointNames):
        if not self._loadLibrary(robotName, jointNames):
            rospy.loginfo("Path library: No paths corresponding to robot %s with joints %s" % (robotName, jointNames))
            return ([], [], [])
        s, g = list(s), list(g)
        leafNode = self._getPathLeafBySG(s, g)
        #self._loadSGCache(leafNode.name) #load the cache from the file just in case the paths were updated
        closestN = self._findClosestNinAll(s, g, leafNode, n)
        if len(closestN) == 0:
            rospy.loginfo("Path library: No paths corresponding to %s" % (leafNode.name));
            return ([], [], []);
        pids, pathNames, paths = zip(*closestN)
        projections = [self._pathProjection(path, s, g) for path in paths]
        projectionsTuples = [tuple([tuple(pt) for pt in proj]) for proj in projections]
        rospy.loginfo("Path library: start %s, goal %s" % (str(s), str(g)))
        startTime = time.clock()
        allInvalidSections = self.invalidSectionWrapper.getInvalidSectionsForPaths(projections, groupName);
        rospy.loginfo("Path library: took %f seconds to do collision checking" % (time.clock() - startTime))
        rospy.loginfo("Path library: all invalid sections: %s" % (zip(pids, allInvalidSections)))
        bestPath = (float('inf'), None); #form is (v, path)
        for index, path in enumerate(projections):
            v = self._evaluateV(allInvalidSections[index]);
            if v < bestPath[0]:
                bestPath = (v, index);
        bestProjection = projections[bestPath[1]]
        pathRetrieved = paths[bestPath[1]]
        rospy.loginfo("Path library: retrieved path has pathid = %i, length = %i" % (pids[bestPath[1]], len(paths[bestPath[1]])))
        return (bestProjection, pathRetrieved, allInvalidSections[bestPath[1]])

    def _pathProjection(self, path, s, g):
        projection = []
        if path[0] != s:
            #rospy.loginfo("Path library: projecting start: original = %s, new = %s" % (path[0], s))
            projection.append(s)
            projection += self._getMiddleSteps(s, path[0])
        projection += path
        if path[-1] != g:
            projection += self._getMiddleSteps(path[-1], g)
            projection.append(g)
        return projection

    def _getMiddleSteps(self, p1, p2):
        middle = []
        dist = self._lineDist(p1, p2)
        if dist > self.stepSize:
            pathFraction = float(self.stepSize) / dist
            diffs = [self._angleBetween(p1[i], p2[i]) for i in xrange(self.currentNumDims)]
            for step in [pathFraction*i for i in xrange(1, int(math.ceil(1/pathFraction)))]:
                middle.append([p1[j]+step*diffs[j]*self._getDirectionMultiplier(p1[j], p2[j]) for j in xrange(self.currentNumDims)])
        return middle

    #get the direction to go around the circle to get from angle a to b
    def _getDirectionMultiplier(self, a, b):
        x, y = a % TWO_PI, b % TWO_PI
        if x <= y:
            return 1 if (0 <= y-x < TWO_PI/2) else -1
        else:
            return -1 if (0 <= x-y < TWO_PI/2) else 1

    #returns a list of (pid, pathName, path) tuples which correspond to the n paths that are closest to the target start and goal in terms of projection distance
    def _findClosestNinAll(self, targetStart, targetGoal, pathLeafNode, n):
        sgs = self._getSGsFromPathName(pathLeafNode.name);
        targetSG = [targetStart, targetGoal];
        if len(sgs) == 0:
            maxDist = float('inf')
            bestSGs = []
        else:
            bestSGs = [(pid, pathLeafNode.name, sg) for pid, sg in (sorted(sgs, key=(lambda sgWithId: self._projDist(targetSG, sgWithId[1]))))[:n]];
            maxDist = self._projDist(targetSG, bestSGs[-1][2]);
        counter = 0;
        nodes = [];
        currentNode = self.tree;
        counts = ([0 for i in xrange(self.currentNumDims)], [0 for i in xrange(self.currentNumDims)]);
        directions = self._getPathDirections(pathLeafNode.name);
        #get the nodes that need to be checked by taking the opposite directions of the leaf
        for i in xrange(len(directions)):
            counts = self._updateCounts(counts, currentNode, targetSG);
            if directions[i] == 'r':
                nodes.append((counts, currentNode.left));
                currentNode = currentNode.right;
            elif directions[i] == 'l':
                nodes.append((counts, currentNode.right));
                currentNode = currentNode.left;
        while len(nodes) > 0: 
            currentCount, currentNode = nodes.pop();
            if currentNode.name != pathLeafNode.name:
                if currentNode.isLeaf():
                    counter += 1;
                    bestSGs += [(pid, currentNode.name, sg) for pid, sg in self._getSGsFromPathName(currentNode.name)];
                    bestSGs = sorted(bestSGs, key=(lambda (pathId, pathName, sg): self._projDist(targetSG, sg)))[:n];
                    maxDist = self._projDist(targetSG, bestSGs[-1][2]);
                else:
                    if not self._canPrune(currentCount, currentNode.name, pathLeafNode.name, maxDist):
                        currentCount = self._updateCounts(currentCount, currentNode, targetSG);
                        nodes.append((currentCount, currentNode.left));
                        nodes.append((currentCount, currentNode.right));
        rospy.loginfo("Path library: Number of nodes checked: %i" % counter);
        rospy.loginfo("Path library: distances are %s" % ([(pathId, pathName, self._projDist(targetSG, sg)) for pathId, pathName, sg in bestSGs]))
        return sorted(self._getPathsOfSGs(bestSGs), key=(lambda (pid, pathName, p): self._totalDist(targetStart, targetGoal, p)));

    def _canPrune(self, counts, nodeName, targetNodeName, maxDist):
        shortDirs, longDirs = sorted([self._getPathDirections(name) for name in [nodeName, targetNodeName]], key=(lambda x: len(x)))
        beginIndex = (len(shortDirs)/(2*self.currentNumDims))*(2*self.currentNumDims)
        endStartIndex = min([beginIndex+self.currentNumDims, len(shortDirs)])
        startCounter = sum([counts[START][i%self.currentNumDims] for i in xrange(beginIndex, endStartIndex) if shortDirs[i] != longDirs[i]])
        goalCounter = sum([counts[GOAL][i%self.currentNumDims] for i in xrange(endStartIndex, len(shortDirs)) if shortDirs[i] != longDirs[i]])
        return startCounter**0.5 + goalCounter**0.5 >= maxDist

    def _updateCounts(self, counts, node, targetSG):
        #if updating the first dimension, then need to start over
        if node.splitState == START and node.splitIndex == 0:
            startRet = [0 for i in xrange(self.currentNumDims)]
            startRet[0] = self._angleBetween(node.splitValue, targetSG[START][0])**2
            goalRet = [0 for i in xrange(self.currentNumDims)]
            return (startRet, goalRet)
        else:
            newCount = (list(counts[START]), list(counts[GOAL]))
            newCount[node.splitState][node.splitIndex] = self._angleBetween(node.splitValue, targetSG[node.splitState][node.splitIndex])**2
            return newCount

    def reorganizePaths(self, oldPathsFile, newNodeSize=None):
        if newNodeSize is None:
            newNodeSize = self.nodeSize
        else:
            self.nodeSize = newNodeSize
        self._deleteLibraryFiles()
        self._initLib(nodeSize=newNodeSize)
        if oldPathsFile[-1] != '/':
            oldPathsFile += '/'
        allPathFiles = os.listdir(oldPathsFile)
        allPathFiles.remove(PATH_TREE_NAME)
        allPathFiles.remove(SG_TREE_NAME)
        currentPaths = []
        counter = 0
        for f in allPathFiles:
            if f.find(ROOT_NAME) == 0:
                pathList = self._getOldPaths(oldPathsFile+f)
                print "reorganize", f, len(pathList)
                counter += len(pathList)
                if len(currentPaths) + len(pathList) > newNodeSize:
                    self._storeMultiplePaths(currentPaths)
                    currentPaths = pathList
                else:
                    currentPaths += pathList
        if len(currentPaths) > 0:
            self._storeMultiplePaths(currentPaths)
        print "reorganized", counter+len(currentPaths), "paths"

    #add each path in allPaths to the correct node; used for reorganizing nodes
    def _storeMultiplePaths(self, allPaths):
        allPaths = [(pid, self._normalizePath(p)) for pid, p in allPaths];
        for i in xrange((len(allPaths)/self.nodeSize)+1):
            pathDict = dict(); #mapping from node name to list of paths to store at that node
            for pathWithId in allPaths[self.nodeSize*i:self.nodeSize*(i+1)]:
                pid, path = pathWithId
                leafName = self._getPathLeafBySG(path[0], path[-1]).name;
                if not pathDict.has_key(leafName):
                    pathDict[leafName] = [];
                pathDict[leafName].append(path);
            for key in pathDict.keys():
                self._addPaths(key, pathDict[key]);

    def _getOldPaths(self, filename):
        f = open(filename, 'r');
        numPaths = int(f.next().strip());
        paths = [];
        for line in f:
            pidString, pathString = line.strip().split(" ")
            paths.append((int(pidString), self._stringToPath(pathString)));
        f.close();
        return paths;

    #sets split index and split value of treenode, returns sorted list of paths
    #splits by using the parent.splitIndex + 1
    def _consecutiveSplit(self, treenode):
        paths = self._getPaths(treenode.name);
        if treenode.name == ROOT_NAME:
            treenode.splitState = START;
            treenode.splitIndex = 0;
        else:
            treenode.splitIndex = treenode.parent.splitIndex + 1;
            if treenode.splitIndex == self.currentNumDims:
                treenode.splitState = START if treenode.parent.splitState == GOAL else GOAL;
                treenode.splitIndex = 0;
            else:
                treenode.splitState = treenode.parent.splitState;
        return self._doPathSplit(treenode, paths);

    #sets split index and split value of treenode, returns sorted list of paths
    #splits by largest range along an index
    def _largestRangeSplit(self, treenode):
        paths = self._getPaths(treenode.name);
        maxRange = (None, float('-inf')); #tuple of (tuple indicating split index, range on that index)
        for state in [START, GOAL]:
            for index in xrange(self.currentNumDims):
                sortKey = lambda (pid, p): p[state][index];
                tempRange = self._angleBetween(sortKey(max(paths, key=sortKey)), sortKey(min(paths, key=sortKey)));
                if tempRange > maxRange[1]:
                    maxRange = ((state, index), tempRange);
        treenode.splitState = maxRange[0][0];
        treenode.splitIndex = maxRange[0][1];
        return self._doPathSplit(treenode, paths);

    #sets split value for the treenode and returns the split paths
    def _doPathSplit(self, treenode, paths):
        indexFunction = (lambda x: x[1][treenode.splitState][treenode.splitIndex])
        if treenode.name not in self.sgCache.keys():       
            self._loadSGCache(treenode.name)
        tempPaths = sorted(paths, key=(lambda t: indexFunction(t)))
        currentSplitValue = len(tempPaths)/2
        
        while currentSplitValue < len(tempPaths) and indexFunction(tempPaths[currentSplitValue-1]) == indexFunction(tempPaths[currentSplitValue]):
            currentSplitValue += 1

        if currentSplitValue == len(tempPaths):
            currentSplitValue = (len(tempPaths)/2)-1
            while currentSplitValue > 0 and indexFunction(tempPaths[currentSplitValue-1]) == indexFunction(tempPaths[currentSplitValue]):
                currentSplitValue -= 1
            if currentSplitValue == 0:
                rospy.loginfo("Too many start goals that are the same, not adding path to library.")
                treenode.splitValue = indexFunction(tempPaths[currentSplitValue-1])
                return (tempPaths, [])
        treenode.splitValue = (indexFunction(tempPaths[currentSplitValue-1])+indexFunction(tempPaths[currentSplitValue]))/2.0
        return (tempPaths[:currentSplitValue], tempPaths[currentSplitValue:]);
    
    def _readNumPathsFromFile(self, filename):
        f = open(self._getFullFilename(filename), 'r');
        ret = int(f.next().strip());
        f.close();
        return ret;

    def _readNumSGsFromFile(self, filename):
        f = open(self._getFullFilename(filename), 'r');
        ret = int(f.next().strip());
        f.close();
        return ret;

    def _splitPathNode(self, nodeName):
        leaf = self._findPathLeafByPathName(nodeName);
        leftPaths, rightPaths = self.SPLIT_PATHS_FUNCT(leaf);
        newLeft, newRight = PathTreeNode(name=leaf.name+"l", parent=leaf), PathTreeNode(name=leaf.name+"r", parent=leaf);
        leaf.left, leaf.right = newLeft, newRight;
        self._makePathsFile(newLeft.name, leftPaths);
        self._makePathsFile(newRight.name, rightPaths);
        self._updateSGs(leaf);
        self._removePathFile(leaf.name);
        self._storeCurrentPathTree();

    #when split a path node, need to update path names for start-goals
    def _updateSGs(self, pathNode):
        sgLeafName = self._getSGLeafByPathName(pathNode.name).name;
        if pathNode.name not in self.sgCache.keys():       
            self._loadSGCache(pathNode.name);
        sgs = self.sgCache; #cache gets updated while sgs is changed
        toUpdate = sgs.pop(pathNode.name);
        sgs[pathNode.name+'l'] = [];
        sgs[pathNode.name+'r'] = [];
        for sgWithId in toUpdate:
            pid, sg = sgWithId
            if sg[pathNode.splitState][pathNode.splitIndex] <= pathNode.splitValue:
                sgs[pathNode.name+'l'].append(sgWithId);
            else:
                sgs[pathNode.name+'r'].append(sgWithId);
        self._makeSGsFile(sgLeafName, sgs);

    def _makeSGsFile(self, filename, sgs):
        f = open(self._getFullFilename(filename), 'w');
        size = sum([len(sgList) for sgList in sgs.values()]);
        f.write("%s\n" % (string.zfill(str(size), len(str(self.sgNodeSize)))));
        for pathNodeName in sgs.keys():
            for pid, sg in sgs[pathNodeName]:
                f.write("%i %s %s\n" % (pid, pathNodeName, self._sgToString(sg)));
        f.close();
        
    # precondition: increased path count is less than or equal to self.nodeSize
    def _changePathCountInFile(self, filename, amt):
        f = open(self._getFullFilename(filename), 'r+');
        newVal = int(f.next().strip()) + amt;
        f.seek(0);
        f.write(string.zfill(str(newVal), len(str(self.nodeSize))));
        f.close();

    def _readPathsFromFile(self, filename):
        f = open(self._getFullFilename(filename), 'r')
        numPaths = int(f.next().strip())
        paths = []
        for line in f:
            pidString, pathString = line.strip().split(" ")
            paths.append((int(pidString), self._stringToPath(pathString)))
        f.close()
        return paths

    def _getPaths(self, filename):
        return self._readPathsFromFile(filename);

    def _writePathsToFile(self, filename, pathList):
        f = open(self._getFullFilename(filename), 'a');
        pathsWithIds = []
        for p in pathList:
            f.write("%i %s\n" % (self.nextPathId, self._pathToString(p)));
            pathsWithIds.append((self.nextPathId, p))
            self.nextPathId += 1
        f.close();
        return pathsWithIds

    def _writePathToFile(self, filename, path):
        return self._writePathsToFile(filename, [path])[0];

    def _addPaths(self, pathNodeName, pathList):
        numPathsBeforeAdd = self._readNumPathsFromFile(pathNodeName)
        sgNodeName = self._getSGLeafByPathName(pathNodeName).name
        numSGsBeforeAdd = self._readNumSGsFromFile(sgNodeName)
        pathsWithIds = self._writePathsToFile(pathNodeName, pathList)
        newSGs = [(pid, self._getSGFromPath(p)) for pid, p in pathsWithIds]
        if pathNodeName not in self.sgCache.keys():       
            self._loadSGCache(pathNodeName)
        if not self.sgCache.has_key(pathNodeName):
            self.sgCache[pathNodeName] = []
        self.sgCache[pathNodeName] += newSGs
        self._writeSGsToFile(pathNodeName, newSGs)
        #check if need to split path node
        if numPathsBeforeAdd + len(pathsWithIds) > self.nodeSize:
            self._splitPathNode(pathNodeName)
            #check if need to split start-goal node
            #saving numSGsBeforeAdd solves the problem of writing too large of a number to an sg file after a path node split
            if numSGsBeforeAdd + len(newSGs) > self.sgNodeSize:
                self._splitSGNode(sgNodeName)
        else:
            self._changePathCountInFile(pathNodeName, len(pathsWithIds))
            if numSGsBeforeAdd + len(newSGs) > self.sgNodeSize:
                self._splitSGNode(sgNodeName)
            else: #need to increment sg count here since path node split didn't do the update
                self._changeSGCountInFile(sgNodeName, len(newSGs))

    def _addPath(self, pathNodeName, path):
        self._addPaths(pathNodeName, [path])

    def _makePathsFile(self, filename, paths):
        f = open(self._getFullFilename(filename), 'w')
        f.write("%s\n" % (string.zfill(str(len(paths)), len(str(self.nodeSize)))))
        for pid, path in paths:
            f.write("%i %s\n" % (pid, self._pathToString(path)))
        f.close()

    def _pathToString(self, path):
        return '|'.join([','.join([str(j) for j in p]) for p in path])

    def _stringToPath(self, s):
        return [[float(j) for j in pathStr.split(',')] for pathStr in s.split('|')]

    def _getSGFromPath(self, path):
        return [path[0], path[-1]];

    #sgNodeName is the name of the start-goal node
    def _splitSGNode(self, sgNodeName):
        leftName, rightName = sgNodeName+'l', sgNodeName+'r';
        sgLeaf = self._findSGLeafBySGName(sgNodeName);
        newLeft, newRight = SGTreeNode(name=leftName, parent=sgLeaf), SGTreeNode(name=rightName, parent=sgLeaf);
        sgLeaf.left, sgLeaf.right = newLeft, newRight;
        sgs = self._getSGs(sgNodeName)
        leftSGs, rightSGs = dict(), dict()
        for pathNodeName in sgs.keys():
            if self._getPathDirections(pathNodeName).find(self._getSGDirections(leftName)) == 0:
                leftSGs[pathNodeName] = sgs[pathNodeName]
            else:
                rightSGs[pathNodeName] = sgs[pathNodeName]
        self._makeSGsFile(leftName, leftSGs)
        self._makeSGsFile(rightName, rightSGs)
        self._removeSGFile(sgNodeName);
        self._storeCurrentSGTree();
        self.sgCache = leftSGs

    def _changeSGCountInFile(self, sgNodeName, amt):
        f = open(self._getFullFilename(sgNodeName), 'r+');
        newVal = int(f.next().strip()) + amt;
        f.seek(0);
        f.write(string.zfill(str(newVal), len(str(self.sgNodeSize))));
        f.close();

    #return a dictionary of node name to start goals for node
    def _readSGsFromFile(self, sgNodeName):
        f = open(self._getFullFilename(sgNodeName), 'r')
        numSGs = int(f.next().strip())
        startGoals = dict()
        for line in f:
            pidString, nodeName, sgString = line.strip().split(" ")
            sg = self._stringToSG(sgString)
            if nodeName not in startGoals.keys():
                startGoals[nodeName] = []
            startGoals[nodeName].append((int(pidString), sg))
        f.close()
        return startGoals

    def _getSGs(self, sgNodeName):
        #possibly use the cache here somehow
        return self._readSGsFromFile(sgNodeName);

    def _writeSGsToFile(self, pathNodeName, sgs):
        f = open(self._pathNameToSGFile(pathNodeName), 'a');
        for pid, sg in sgs:
            f.write("%i %s %s\n" % (pid, pathNodeName, self._sgToString(sg)))
        f.close();

    def _sgToString(self, sg):
        return "%s" % ('|'.join([','.join([str(j) for j in p]) for p in sg]));

    def _stringToSG(self, s):
        return [[float(j) for j in pt.split(',')] for pt in s.split('|')];

    def _findPathLeafByPathName(self, name):
        directions = self._getPathDirections(name);
        currentNode = self.tree;
        for d in directions:
            if d == 'l':
                currentNode = currentNode.left;
            elif d == 'r':
                currentNode = currentNode.right;
        return currentNode;

    def _evaluateV(self, invalidSections):
        if len(invalidSections) == 0:
            return 0;
        else:
            return sum([section[1]-section[0]-1 for section in invalidSections]);
            
    def _getPathLeafBySG(self, s, g):
        currentNode = self.tree;
        while not currentNode.isLeaf():
            useState = s if currentNode.splitState == START else g;
            if useState[currentNode.splitIndex] <= currentNode.splitValue:
                currentNode = currentNode.left;
            else:
                currentNode = currentNode.right;
        return currentNode;

    def _getSGsFromPathName(self, pathNodeName):
        if pathNodeName not in self.sgCache.keys():
            self._loadSGCache(pathNodeName);
        return self.sgCache[pathNodeName];

    def _loadSGCache(self, pathNodeName):
        sgNodeName = self._getSGLeafByPathName(pathNodeName).name
        self.sgCache = self._getSGs(sgNodeName)
        if len(self.sgCache.keys()) == 0:
            self.sgCache[ROOT_NAME] = []
    
    #returns a list of (pid, pathName, path) corresponding to the list of (pid, pathName, sg) in sgs
    def _getPathsOfSGs(self, sgs):
        rospy.loginfo("Path library: %s" % (str(sgs)))
        sgDict = dict();
        ret = [];
        for pid, pathName, sg in sgs:
            if not sgDict.has_key(pathName):
                sgDict[pathName] = []
            sgDict[pathName].append(pid)
        for pathName in sgDict.keys():
            paths = self._getPaths(pathName)
            for pidFromSG in sgDict[pathName]:
                for pathWithId in paths:
                    pathId, path = pathWithId
                    if pidFromSG == pathId:
                        ret.append((pathId, pathName, path))
                        break
        return ret;

    def _getFullFilename(self, name, libName=None):
        if libName is None:
            libName = self.currentLibName
        return "%s/%s/%s" % (self.pathLibraryDir, libName, name);

    def _getFullLibName(self, libName=None):
        if libName is None:
            libName = self.currentLibName
        return "%s/%s" % (self.pathLibraryDir, libName)

    def _pathNameToSGFile(self, name):
        return "%s/%s/%s" % (self.pathLibraryDir, self.currentLibName, self._getSGLeafByPathName(name).name);

    def _getSGDirections(self, name):
        return name[len(SG_ROOT_NAME):];

    def _getPathDirections(self, name):
        return name[len(ROOT_NAME):];

    def _findSGLeafByDirs(self, directions):
        currentNode = self.sgTree;
        for d in directions:
            if currentNode.isLeaf():
                break;
            elif d == 'r':
                currentNode = currentNode.right;
            elif d == 'l':
                currentNode = currentNode.left;
        return currentNode;

    def _findSGLeafBySGName(self, name):
        return self._findSGLeafByDirs(self._getSGDirections(name));

    def _getSGLeafByPathName(self, name):
        return self._findSGLeafByDirs(self._getPathDirections(name));

    def _DTWDistance(self, path1, path2):
        n, m = len(path1), len(path2);
        table = [[0 for i in xrange(m+1)] for j in xrange(n+1)];
        for i in xrange(1, n+1):
            table[i][0] = float('inf');
        for i in xrange(1, m+1):
            table[0][i] = float('inf');
        table[0][0] = 0;

        for i in xrange(n):
            for j in xrange(m):
                cost = self._lineDist(path1[i], path2[j]);
                table[i+1][j+1] = cost + min(table[i][j+1], table[i+1][j], table[i][j]);

        return table[n][m];

    def _lineDist(self, p1, p2):
        return (sum([(self._angleBetween(p1[i], p2[i]))**2 for i in xrange(len(p1))]))**0.5;

    def _projDist(self, sg1, sg2):
        return self._lineDist(sg1[START], sg2[START]) + self._lineDist(sg1[GOAL], sg2[GOAL]);

    def _totalDist(self, targetStart, targetGoal, path):
        return self._lineDist(targetStart, path[0]) + self._lineDist(targetGoal, path[-1]);

    def _partialLineDist(self, p1, p2):
        return (sum([(self._angleBetween(p1[i], p2[i]))**2 for i in xrange(len(p1))**0.5 if p1[i] is not None]))**0.5;

    def _partialTotalDist(self, targetStart, targetGoal, path):
        return self._partialLineDist(targetStart, path[0]) + self._partialLineDist(targetGoal, path[-1]);

    def _angleBetween(self, a, b):
        """
        if a > b:
            temp = a
            a = b
            b = temp
        return b-a
        """
        a1, b1 = a % TWO_PI, b % TWO_PI;
        if a1 > b1: #make a1 the smaller one
            temp = a1;
            a1 = b1;
            b1 = temp;
        return min(b1-a1, a1+(TWO_PI-b1));

    def _normalizeAngle(self, angle):
        return angle;
        #return angle % TWO_PI;

    def _normalizePath(self, path):
        return [[self._normalizeAngle(angle) for angle in pt] for pt in path];

    ###functions for debugging###

    def findDistances(self, s, g, libName):
        dists = []
        for f in os.listdir(self._getFullLibName(libName)):
            if f.find("pickle") == -1 and f.find(SG_ROOT_NAME) == -1:
                paths = self._getPaths(f)
                for pid, p in paths:
                    dists.append(self._totalDist(s, g, p))
        dists.sort()
        return dists

    def checkAccuracy(self, numIters, n):
        RIGHT_ARM_JOINT_LIMITS = [(-1*math.pi/4-1.35, -1*math.pi/4+1.35), (-0.3536, 1.2963), (-1.55-2.2, -1.55+2.2), (-2.1213, -0.15), (-1*math.pi, math.pi), (-2.0, -0.1), (-1*math.pi, math.pi)];
        for i in xrange(numIters):
            start = [(pt[1]-pt[0])*random.random()+pt[0] for pt in RIGHT_ARM_JOINT_LIMITS];
            goal = [(pt[1]-pt[0])*random.random()+pt[0] for pt in RIGHT_ARM_JOINT_LIMITS];
            retrieved = self._retrievePathSimple(start, goal, n);
            actual = self.findDistances(start, goal)[0:n]
            retr = [self._totalDist(start, goal, path) for path in retrieved]
            print "Actual best %i distances: %s" % (n, str(actual))
            print "Retrieved %i distances: %s" % (n, str(retr))
            if actual != retr:
                print "Incorrect retrieval"
            else:
                print "Correct retrieval"

    def _retrievePathSimple(self, s, g, n):
        leafNode = self._getPathLeafBySG(s, g)
        #self._loadTrees()
        #self._loadSGCache(leafNode.name) #load the cache from the file
        closestN = self._findClosestNinAll(s, g, leafNode, n=n)
        pids, pathNames, paths = zip(*closestN)
        #closest = closestN[0]
        return paths

    def reorganizePathsIndividually(self, oldPathsFile, newNodeSize=None):
        if newNodeSize is None:
            newNodeSize = self.nodeSize;
        else:
            self.nodeSize = newNodeSize;
        self._deleteLibraryFiles();
        self._initLib(nodeSize=newNodeSize);
        if oldPathsFile[-1] != '/':
            oldPathsFile += '/';
        allPathFiles = os.listdir(oldPathsFile);
        allPathFiles.remove(PATH_TREE_NAME);
        allPathFiles.remove(SG_TREE_NAME);
        currentPaths = [];
        counter = 0;
        for f in allPathFiles:
            if f.find(ROOT_NAME) == 0:
                pathList = self._getOldPaths(oldPathsFile+f);
                print "reorganize", f, len(pathList);
                for p in pathList:
                    self.storePath(p);

    def getPathPoint(self, pid, index, libName):
        for f in os.listdir(self._getFullLibName(libName)):
            if f.find(ROOT_NAME) == 0:
                for pathId, path in self._getPaths(f):
                    if pathId == pid:
                        if index < len(path):
                            print "Path library: path (in %s) is %s" % (f, ' '.join([str(x) for x in path[index]]))
                            return path[index]
                        else:
                            print "Index is out of range of path (in %s), path is %i points long" % (f, len(path))
                            return None

    def deletePathBySG(self, s, g):
        #remove path from paths file
        leaf = self._getPathLeafBySG(s, g)
        paths = self._getPaths(leaf.name)
        newPaths = []
        for pathWithId in paths:
            pid, p = pathWithId
            if p[0] != s or p[-1] != g:
                newPaths.append(pathWithId)
        self._makePathsFile(leaf.name, newPaths)
        #remove path from sg file
        sgNodeName = self._getSGLeafByPathName(leaf.name).name
        sgs = self._getSGs(sgNodeName)
        newSGs = []
        for sgWithId in sgs[leaf.name]:
            pid, sg = sgWithId
            if sg[0] != s or sg[-1] != g:
                newSGs.append(sgWithId)
        sgs[leaf.name] = newSGs
        self._makeSGsFile(sgleaf.name, sgs)

if __name__ == "__main__":
    STEP_SIZE = 0.02
    if len(sys.argv) == 3 and sys.argv[1] == "reorganize":
        p = PathLibrary(STEP_SIZE, nodeSize=2, sgNodeSize=8)
        p.reorganizePaths(sys.argv[2])
    else:
        test_joints = ['a', 'b']
        p = PathLibrary(STEP_SIZE, nodeSize=2, sgNodeSize=8)
        for i in xrange(3):
            p.storePath([[random.random()*12,1],[1,2],[2,3]], "pr2", test_joints)
        p.deletePathById(1, "pr2", test_joints)
        for i in xrange(3):
            p.storePath([[random.random()*12,1],[1,2],[2,3]], "pr3", test_joints)
        p.deletePathById(1, "pr3", test_joints)
        for i in xrange(3):
            p.storePath([[random.random()*12,1],[1,2],[2,3]], "pr2", test_joints)
        p.deletePathById(3, "pr2", test_joints)
        p.retrievePath([], [], 0, "blah", "apple", ['a'])
