"""
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, University of California, Berkeley
# All rights reserved.
# Authors: Cameron Lee (cameronlee@berkeley.edu) and Dmitry Berenson (
berenson@eecs.berkeley.edu)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of California, Berkeley nor the names
of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""

import roslib
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

    def is_leaf(self):
        return self.left is None and self.right is None;

class PathTreeNode(TreeNode):
    def __init__(self, name, parent, split_value=0, split_index=0, split_state=START):
        TreeNode.__init__(self, name, parent);
        self.split_value = split_value;
        self.split_index = split_index;
        self.split_state = split_state; #START if split_index is for a start state value, GOAL if it is for goal state value

class SGTreeNode(TreeNode):
    def __init__(self, name, parent):
        TreeNode.__init__(self, name, parent);

class PathLibrary:
    def __init__(self, path_library_dir, step_size, node_size=100, sg_node_size=1000, dtw_dist=10):
        #variables that are the same for all libraries
        if path_library_dir[-1] == '/':
            path_library_dir = path_library_dir[:-1]
        self.path_library_dir = path_library_dir
        self.step_size = step_size
        self.new_node_size, self.new_sg_node_size = node_size, sg_node_size
        self.dtw_dist = dtw_dist
        self.invalid_section_wrapper = InvalidSectionWrapper()
        self.split_paths_function = self._largest_range_split #self._consecutive_split

        self._init_lib_vars()

    #variables that keep track of state for a single library
    #these variables when a new library is loaded
    def _init_lib_vars(self):
        self.current_lib_name = ""
        self.tree, self.sg_tree = None, None
        self.node_size = 0
        self.sg_node_size = 0
        self.next_path_id = 0
        self.current_num_dims = 0
        self.sg_cache = dict()

    def _init_paths_file(self, filename):
        f = open(self._get_full_filename(filename), 'w');
        f.write("%s\n" % (string.zfill('0', len(str(self.node_size)))));
        f.close();

    def _init_sgs_file(self, sg_node_name):
        f = open(self._get_full_filename(sg_node_name), 'w');
        f.write("%s\n" % (string.zfill('0', len(str(self.sg_node_size)))));
        f.close();

    def _read_library_size(self):
        count = 0
        for f in os.listdir(self._get_full_lib_name()):
            if f.find(SG_ROOT_NAME) != -1:
                count += self._read_num_sgs_from_file(f)
        return count

    def _get_next_path_id(self):
        max_id = 0
        for f in os.listdir(self._get_full_lib_name()):
            if f.find(SG_ROOT_NAME) != -1:
                max_id_for_file = self._get_max_path_id(f)
                if max_id_for_file > max_id:
                    max_id = max_id_for_file
        return max_id+1

    def _get_max_path_id(self, filename):
        f = open(self._get_full_filename(filename), 'r')
        num_sgs = int(f.next().strip())
        current_max = 0
        for line in f:
            pid = int(line.strip().split(" ")[0])
            if pid > current_max:
                current_max = pid
        f.close()
        return current_max

    def _load_trees(self):
        tree_load_file = open(self._get_full_filename(PATH_TREE_NAME), 'r');
        self.node_size, self.tree = pickle.load(tree_load_file);
        tree_load_file.close();
        sg_tree_load_file = open(self._get_full_filename(SG_TREE_NAME), 'r');
        self.sg_node_size, self.sg_tree = pickle.load(sg_tree_load_file);
        sg_tree_load_file.close();

    def _store_current_sg_tree(self):
        sg_tree_file = open(self._get_full_filename(SG_TREE_NAME), 'w')
        pickle.dump((self.sg_node_size, self.sg_tree), sg_tree_file);
        sg_tree_file.close();

    def _store_current_path_tree(self):
        tree_file = open(self._get_full_filename(PATH_TREE_NAME), 'w')
        pickle.dump((self.node_size, self.tree), tree_file);
        tree_file.close();

    def _store_current_trees(self):
        self._store_current_path_tree();
        self._store_current_sg_tree();

    def _remove_path_file(self, filename):
        os.remove(self._get_full_filename(filename));

    def _remove_sg_file(self, filename):
        os.remove(self._get_full_filename(filename));

    def _get_robot_name_and_joints(self, lib_name):
        f = open(self._get_full_filename(INFO_FILE_NAME, lib_name), 'r')
        robot_name = f.next().strip()
        joint_names = f.next().strip().split('|')
        return (robot_name, joint_names)

    def _find_library(self, robot_name, joint_names):
        for lib in os.listdir(self.path_library_dir):
            if lib.find(LIBRARY_DIR_PREFIX) == 0 and self._get_robot_name_and_joints(lib) == (robot_name, joint_names):
                return lib
        return None

    def _load_library(self, robot_name, joint_names):
        lib = self._find_library(robot_name, joint_names)
        if lib is None:
            return False
        if lib != self.current_lib_name:
            self.current_lib_name = lib
            self._load_trees()
            self.next_path_id = self._get_next_path_id()
            self.current_num_dims = len(joint_names)
        return True

    def _create_and_load_new_library(self, robot_name, joint_names):
        #figure out a unique library id
        taken_ids = set()
        for lib in os.listdir(self.path_library_dir):
            if lib.find(LIBRARY_DIR_PREFIX) == 0:
                taken_ids.add(int(lib.split("_")[1]))
        new_id = 0
        while True:
            if new_id in taken_ids:
                new_id += 1
            else:
                break
        self.current_lib_name = "%s%i_%s" % (LIBRARY_DIR_PREFIX, new_id, robot_name)
        rospy.loginfo("Path library: creating new library: %s" % (self.current_lib_name))
        os.mkdir(self._get_full_lib_name())
        info_file = open(self._get_full_filename(INFO_FILE_NAME), 'w')
        info_file.write("%s\n" % (robot_name))
        info_file.write("%s\n" % ('|'.join(joint_names)))
        self.tree = PathTreeNode(ROOT_NAME, None);
        self.sg_tree = SGTreeNode(SG_ROOT_NAME, None);
        self.node_size, self.sg_node_size = self.new_node_size, self.new_sg_node_size
        self._store_current_trees();
        self._init_paths_file(self.tree.name);
        self._init_sgs_file(self.sg_tree.name);
        self.next_path_id = 1
        self.current_num_dims = len(joint_names)
        self.sg_cache = dict()

    def _delete_library_files(self, lib_name):
        for f in os.listdir(self._get_full_lib_name(lib_name)):
            os.remove(self._get_full_filename(f, lib_name))

    def delete_library(self, robot_name, joint_names):
        lib = self._find_library(robot_name, joint_names)
        if lib is None:
            rospy.loginfo("Path library: library for robot %s and joints %s does not exist" % (robot_name, joint_names))
            return False
        else:
            rospy.loginfo("Path library: deleting library for robot %s and joints %s" % (robot_name, joint_names))
            self._delete_library_files(lib)
            os.rmdir(self._get_full_lib_name(lib))
            return True

    def delete_path_by_id(self, pid, robot_name, joint_names):
        self._load_library(robot_name, joint_names)
        current = None
        new_paths = []
        for f in os.listdir(self._get_full_lib_name()):
            if f.find(ROOT_NAME) == 0:
                for path_id, path in self._get_paths(f):
                    if path_id == pid:
                        current = f
                        break
            if current is not None:
                rospy.loginfo("Path library: deleting path in %s" % (current))
                for path_id, path in self._get_paths(current):
                    if path_id != pid:
                        new_paths.append((path_id, path))
                self._make_paths_file(current, new_paths)
                sg_node_name = self._get_sg_leaf_by_path_name(current).name
                sgs = self._get_sgs(sg_node_name)
                new_sgs = []
                for sg_with_id in sgs[current]:
                    if sg_with_id[0] != pid:
                        new_sgs.append(sg_with_id)
                sgs[current] = new_sgs
                self.sg_cache = sgs
                self._make_sgs_file(self._get_sg_leaf_by_path_name(current).name, sgs)
                return True
        rospy.loginfo("Path library: path with id %i does not exist in library for robot %s and joints %s" % (pid, robot_name, joint_names))
        return False

    #to force store a path, set prev_path to None
    def store_path(self, new_path, robot_name, joint_names, prev_path=None):
        if not self._load_library(robot_name, joint_names):
            self._create_and_load_new_library(robot_name, joint_names)
        new_path = self._normalize_path(new_path);
        rospy.loginfo("Path Library: got a path of %i points to store" % (len(new_path)))
        if prev_path is None or len(prev_path) == 0 or self._calc_dtw_distance(prev_path, new_path) > self.dtw_dist:
            leaf_node = self._get_path_leaf_by_sg(new_path[0], new_path[-1]);
            self._add_path(leaf_node.name, new_path);
            rospy.loginfo("Path Library: done storing path with id %i", self.next_path_id-1)
            return (True, self._read_library_size())
        else:
            rospy.loginfo("Path Library: did not need to store this path")
            return (False, self._read_library_size())

    def retrieve_path(self, s, g, n, robot_name, group_name, joint_names):
        if not self._load_library(robot_name, joint_names):
            rospy.loginfo("Path library: No paths corresponding to robot %s with joints %s" % (robot_name, joint_names))
            return ([], [], [])
        s, g = list(s), list(g)
        leaf_node = self._get_path_leaf_by_sg(s, g)
        closest_n = self._find_closest_n_in_all(s, g, leaf_node, n)
        if len(closest_n) == 0:
            rospy.loginfo("Path library: No paths corresponding to %s" % (leaf_node.name));
            return ([], [], []);
        pids, path_names, paths = zip(*closest_n)
        projections = [self._path_projection(path, s, g) for path in paths]
        rospy.loginfo("Path library: start %s, goal %s" % (str(s), str(g)))
        start_time = time.clock()
        all_invalid_sections = self.invalid_section_wrapper.get_invalid_sections_for_paths(projections, group_name);
        rospy.loginfo("Path library: took %f seconds to do collision checking" % (time.clock() - start_time))
        rospy.loginfo("Path library: all invalid sections: %s" % (zip(pids, all_invalid_sections)))
        best_path = (float('inf'), None); #form is (v, path)
        for index, path in enumerate(projections):
            v = self._evaluate_v(all_invalid_sections[index]);
            if v < best_path[0]:
                best_path = (v, index);
        best_projection = projections[best_path[1]]
        path_retrieved = paths[best_path[1]]
        rospy.loginfo("Path library: retrieved path has pathid = %i, length = %i" % (pids[best_path[1]], len(paths[best_path[1]])))
        return (best_projection, path_retrieved, all_invalid_sections[best_path[1]])

    def _path_projection(self, path, s, g):
        projection = []
        if path[0] != s:
            projection.append(s)
            projection += self._get_middle_steps(s, path[0])
        projection += path
        if path[-1] != g:
            projection += self._get_middle_steps(path[-1], g)
            projection.append(g)
        return projection

    def _get_middle_steps(self, p1, p2):
        middle = []
        dist = self._line_dist(p1, p2)
        if dist > self.step_size:
            path_fraction = float(self.step_size) / dist
            diffs = [self._angle_between(p1[i], p2[i]) for i in xrange(self.current_num_dims)]
            for step in [path_fraction*i for i in xrange(1, int(math.ceil(1/path_fraction)))]:
                middle.append([p1[j]+step*diffs[j]*self._get_direction_multiplier(p1[j], p2[j]) for j in xrange(self.current_num_dims)])
        return middle

    #get the direction to go around the circle to get from angle a to b
    def _get_direction_multiplier(self, a, b):
        x, y = a % TWO_PI, b % TWO_PI
        if x <= y:
            return 1 if (0 <= y-x < TWO_PI/2) else -1
        else:
            return -1 if (0 <= x-y < TWO_PI/2) else 1

    #returns a list of (pid, path_name, path) tuples which correspond to the n paths that are closest to the target start and goal in terms of projection distance
    def _find_closest_n_in_all(self, target_start, target_goal, path_leaf_node, n):
        sgs = self._get_sgs_from_path_name(path_leaf_node.name);
        target_sg = [target_start, target_goal];
        if len(sgs) == 0:
            max_dist = float('inf')
            best_sgs = []
        else:
            best_sgs = [(pid, path_leaf_node.name, sg) for pid, sg in (sorted(sgs, key=(lambda sg_with_id: self._proj_dist(target_sg, sg_with_id[1]))))[:n]];
            max_dist = self._proj_dist(target_sg, best_sgs[-1][2]);
        counter = 0;
        nodes = [];
        current_node = self.tree;
        counts = ([0 for i in xrange(self.current_num_dims)], [0 for i in xrange(self.current_num_dims)]);
        directions = self._get_path_directions(path_leaf_node.name);
        #get the nodes that need to be checked by taking the opposite directions of the leaf
        for i in xrange(len(directions)):
            counts = self._update_counts(counts, current_node, target_sg);
            if directions[i] == 'r':
                nodes.append((counts, current_node.left));
                current_node = current_node.right;
            elif directions[i] == 'l':
                nodes.append((counts, current_node.right));
                current_node = current_node.left;
        while len(nodes) > 0:
            current_count, current_node = nodes.pop();
            if current_node.name != path_leaf_node.name:
                if current_node.is_leaf():
                    counter += 1;
                    best_sgs += [(pid, current_node.name, sg) for pid, sg in self._get_sgs_from_path_name(current_node.name)];
                    best_sgs = sorted(best_sgs, key=(lambda (path_id, path_name, sg): self._proj_dist(target_sg, sg)))[:n];
                    max_dist = self._proj_dist(target_sg, best_sgs[-1][2]);
                else:
                    if not self._can_prune(current_count, current_node.name, path_leaf_node.name, max_dist):
                        current_count = self._update_counts(current_count, current_node, target_sg);
                        nodes.append((current_count, current_node.left));
                        nodes.append((current_count, current_node.right));
        rospy.loginfo("Path library: Number of nodes checked: %i" % counter);
        rospy.loginfo("Path library: distances are %s" % ([(path_id, path_name, self._proj_dist(target_sg, sg)) for path_id, path_name, sg in best_sgs]))
        return sorted(self._get_paths_of_sgs(best_sgs), key=(lambda (pid, path_name, p): self._total_dist(target_start, target_goal, p)));

    def _can_prune(self, counts, node_name, target_node_name, max_dist):
        short_dirs, long_dirs = sorted([self._get_path_directions(name) for name in [node_name, target_node_name]], key=(lambda x: len(x)))
        begin_index = (len(short_dirs)/(2*self.current_num_dims))*(2*self.current_num_dims)
        end_start_index = min([begin_index+self.current_num_dims, len(short_dirs)])
        start_counter = sum([counts[START][i % self.current_num_dims] for i in xrange(begin_index, end_start_index) if short_dirs[i] != long_dirs[i]])
        goal_counter = sum([counts[GOAL][i % self.current_num_dims] for i in xrange(end_start_index, len(short_dirs)) if short_dirs[i] != long_dirs[i]])
        return start_counter**0.5 + goal_counter**0.5 >= max_dist

    def _update_counts(self, counts, node, target_sg):
        #if updating the first dimension, then need to start over
        if node.split_state == START and node.split_index == 0:
            start_ret = [0 for i in xrange(self.current_num_dims)]
            start_ret[0] = self._angle_between(node.split_value, target_sg[START][0])**2
            goal_ret = [0 for i in xrange(self.current_num_dims)]
            return (start_ret, goal_ret)
        else:
            new_count = (list(counts[START]), list(counts[GOAL]))
            new_count[node.split_state][node.split_index] = self._angle_between(node.split_value, target_sg[node.split_state][node.split_index])**2
            return new_count

    #work in progress: do not call this
    def reorganize_paths(self, old_paths_file):
        self._load_node_sizes()
        self._delete_library_files()

        self._init_lib(node_size=new_node_size)
        if old_paths_file[-1] != '/':
            old_paths_file += '/'
        all_path_files = os.listdir(old_paths_file)
        all_path_files.remove(PATH_TREE_NAME)
        all_path_files.remove(SG_TREE_NAME)
        current_paths = []
        counter = 0
        for f in all_path_files:
            if f.find(ROOT_NAME) == 0:
                path_list = self._get_old_paths(old_paths_file+f)
                print "reorganize", f, len(path_list)
                counter += len(path_list)
                if len(current_paths) + len(path_list) > new_node_size:
                    self._store_multiple_paths(current_paths)
                    current_paths = path_list
                else:
                    current_paths += path_list
        if len(current_paths) > 0:
            self._store_multiple_paths(current_paths)
        print "reorganized", counter+len(current_paths), "paths"

    #add each path in all_paths to the correct node; used for reorganizing nodes
    def _store_multiple_paths(self, all_paths):
        all_paths = [(pid, self._normalize_path(p)) for pid, p in all_paths];
        for i in xrange((len(all_paths)/self.node_size)+1):
            path_dict = dict(); #mapping from node name to list of paths to store at that node
            for path_with_id in all_paths[self.node_size*i:self.node_size*(i+1)]:
                pid, path = path_with_id
                leaf_name = self._get_path_leaf_by_sg(path[0], path[-1]).name;
                if not path_dict.has_key(leaf_name):
                    path_dict[leaf_name] = [];
                path_dict[leaf_name].append(path);
            for key in path_dict.keys():
                self._add_paths(key, path_dict[key]);

    def _get_old_paths(self, filename):
        f = open(filename, 'r');
        num_paths = int(f.next().strip());
        paths = [];
        for line in f:
            pid_string, path_string = line.strip().split(" ")
            paths.append((int(pid_string), self._string_to_path(path_string)));
        f.close();
        return paths;

    #sets split index and split value of tree_node, returns sorted list of paths
    #splits by using the parent.split_index + 1
    def _consecutive_split(self, tree_node):
        paths = self._get_paths(tree_node.name);
        if tree_node.name == ROOT_NAME:
            tree_node.split_state = START;
            tree_node.split_index = 0;
        else:
            tree_node.split_index = tree_node.parent.split_index + 1;
            if tree_node.split_index == self.current_num_dims:
                tree_node.split_state = START if tree_node.parent.split_state == GOAL else GOAL;
                tree_node.split_index = 0;
            else:
                tree_node.split_state = tree_node.parent.split_state;
        return self._do_path_split(tree_node, paths);

    #sets split index and split value of tree_node, returns sorted list of paths
    #splits by largest range along an index
    def _largest_range_split(self, tree_node):
        paths = self._get_paths(tree_node.name);
        max_range = (None, float('-inf')); #tuple of (tuple indicating split index, range on that index)
        for state in [START, GOAL]:
            for index in xrange(self.current_num_dims):
                sort_key = lambda (pid, p): p[state][index];
                temp_range = self._angle_between(sort_key(max(paths, key=sort_key)), sort_key(min(paths, key=sort_key)));
                if temp_range > max_range[1]:
                    max_range = ((state, index), temp_range);
        tree_node.split_state = max_range[0][0];
        tree_node.split_index = max_range[0][1];
        return self._do_path_split(tree_node, paths);

    #sets split value for the tree_node and returns the split paths
    def _do_path_split(self, tree_node, paths):
        index_function = (lambda x: x[1][tree_node.split_state][tree_node.split_index])
        if tree_node.name not in self.sg_cache.keys():
            self._load_sg_cache(tree_node.name)
        temp_paths = sorted(paths, key=(lambda t: index_function(t)))
        current_split_value = len(temp_paths)/2

        #send all paths with same value at split index to the left
        while current_split_value < len(temp_paths) and index_function(temp_paths[current_split_value-1]) == index_function(temp_paths[current_split_value]):
            current_split_value += 1

        if current_split_value == len(temp_paths):
            current_split_value = (len(temp_paths)/2)-1
            while current_split_value > 0 and index_function(temp_paths[current_split_value-1]) == index_function(temp_paths[current_split_value]):
                current_split_value -= 1
            if current_split_value == 0:
                rospy.loginfo("Path library: too many start goals that are the same, not allowing last added path")
                return (sorted(temp_paths, key=lambda path_with_id: path_with_id[0])[:-1], [])
        tree_node.split_value = (index_function(temp_paths[current_split_value-1])+index_function(temp_paths[current_split_value]))/2.0
        return (temp_paths[:current_split_value], temp_paths[current_split_value:]);

    def _read_num_paths_from_file(self, filename):
        f = open(self._get_full_filename(filename), 'r');
        ret = int(f.next().strip());
        f.close();
        return ret;

    def _read_num_sgs_from_file(self, filename):
        f = open(self._get_full_filename(filename), 'r');
        ret = int(f.next().strip());
        f.close();
        return ret;

    def _split_path_node(self, node_name):
        leaf = self._find_path_leaf_by_path_name(node_name);
        left_paths, right_paths = self.split_paths_function(leaf);
        new_left, new_right = PathTreeNode(name=leaf.name+"l", parent=leaf), PathTreeNode(name=leaf.name+"r", parent=leaf);
        leaf.left, leaf.right = new_left, new_right;
        self._make_paths_file(new_left.name, left_paths);
        self._make_paths_file(new_right.name, right_paths);
        self._updateSGs(leaf);
        self._remove_path_file(leaf.name);
        self._store_current_path_tree();

    #when split a path node, need to update path names for start-goals
    def _updateSGs(self, pathNode):
        sg_leaf_name = self._get_sg_leaf_by_path_name(pathNode.name).name;
        if pathNode.name not in self.sg_cache.keys():
            self._load_sg_cache(pathNode.name);
        sgs = self.sg_cache; #cache gets updated while sgs is changed
        toUpdate = sgs.pop(pathNode.name);
        sgs[pathNode.name+'l'] = [];
        sgs[pathNode.name+'r'] = [];
        for sg_with_id in toUpdate:
            pid, sg = sg_with_id
            if sg[pathNode.split_state][pathNode.split_index] <= pathNode.split_value:
                sgs[pathNode.name+'l'].append(sg_with_id);
            else:
                sgs[pathNode.name+'r'].append(sg_with_id);
        self._make_sgs_file(sg_leaf_name, sgs);

    def _make_sgs_file(self, filename, sgs):
        f = open(self._get_full_filename(filename), 'w');
        size = sum([len(sgList) for sgList in sgs.values()]);
        f.write("%s\n" % (string.zfill(str(size), len(str(self.sg_node_size)))));
        for path_node_name in sgs.keys():
            for pid, sg in sgs[path_node_name]:
                f.write("%i %s %s\n" % (pid, path_node_name, self._sg_to_string(sg)));
        f.close();

    # precondition: increased path count is less than or equal to self.node_size
    def _change_path_count_in_file(self, filename, amt):
        f = open(self._get_full_filename(filename), 'r+');
        newVal = int(f.next().strip()) + amt;
        f.seek(0);
        f.write(string.zfill(str(newVal), len(str(self.node_size))));
        f.close();

    def _read_paths_from_file(self, filename):
        f = open(self._get_full_filename(filename), 'r')
        num_paths = int(f.next().strip())
        paths = []
        for line in f:
            pid_string, path_string = line.strip().split(" ")
            paths.append((int(pid_string), self._string_to_path(path_string)))
        f.close()
        return paths

    def _get_paths(self, filename):
        return self._read_paths_from_file(filename);

    def _write_paths_to_file(self, filename, path_list):
        f = open(self._get_full_filename(filename), 'a');
        paths_with_ids = []
        for p in path_list:
            f.write("%i %s\n" % (self.next_path_id, self._path_to_string(p)));
            paths_with_ids.append((self.next_path_id, p))
            self.next_path_id += 1
        f.close();
        return paths_with_ids

    def _write_path_to_file(self, filename, path):
        return self._write_paths_to_file(filename, [path])[0];

    def _add_paths(self, path_node_name, path_list):
        num_paths_before_add = self._read_num_paths_from_file(path_node_name)
        sg_node_name = self._get_sg_leaf_by_path_name(path_node_name).name
        num_sgs_before_add = self._read_num_sgs_from_file(sg_node_name)
        paths_with_ids = self._write_paths_to_file(path_node_name, path_list)
        new_sgs = [(pid, self._get_sg_from_path(p)) for pid, p in paths_with_ids]
        if path_node_name not in self.sg_cache.keys():
            self._load_sg_cache(path_node_name)
        if not self.sg_cache.has_key(path_node_name):
            self.sg_cache[path_node_name] = []
        self.sg_cache[path_node_name] += new_sgs
        self._write_sgs_to_file(path_node_name, new_sgs)
        #check if need to split path node
        if num_paths_before_add + len(paths_with_ids) > self.node_size:
            self._split_path_node(path_node_name)
            #check if need to split start-goal node
            #saving num_sgs_before_add solves the problem of writing too large of a number to an sg file after a path node split
            if num_sgs_before_add + len(new_sgs) > self.sg_node_size:
                self._split_sg_node(sg_node_name)
        else:
            self._change_path_count_in_file(path_node_name, len(paths_with_ids))
            if num_sgs_before_add + len(new_sgs) > self.sg_node_size:
                self._split_sg_node(sg_node_name)
            else: #need to increment sg count here since path node split didn't do the update
                self._change_sg_count_in_file(sg_node_name, len(new_sgs))

    def _add_path(self, path_node_name, path):
        self._add_paths(path_node_name, [path])

    def _make_paths_file(self, filename, paths):
        f = open(self._get_full_filename(filename), 'w')
        f.write("%s\n" % (string.zfill(str(len(paths)), len(str(self.node_size)))))
        for pid, path in paths:
            f.write("%i %s\n" % (pid, self._path_to_string(path)))
        f.close()

    def _path_to_string(self, path):
        return '|'.join([','.join([str(j) for j in p]) for p in path])

    def _string_to_path(self, s):
        return [[float(j) for j in path_str.split(',')] for path_str in s.split('|')]

    def _get_sg_from_path(self, path):
        return [path[0], path[-1]];

    #sg_node_name is the name of the start-goal node
    def _split_sg_node(self, sg_node_name):
        left_name, right_name = sg_node_name+'l', sg_node_name+'r';
        sg_leaf = self._find_sg_leaf_by_sg_name(sg_node_name);
        new_left, new_right = SGTreeNode(name=left_name, parent=sg_leaf), SGTreeNode(name=right_name, parent=sg_leaf);
        sg_leaf.left, sg_leaf.right = new_left, new_right;
        sgs = self._get_sgs(sg_node_name)
        left_sgs, right_sgs = dict(), dict()
        for path_node_name in sgs.keys():
            if self._get_path_directions(path_node_name).find(self._get_sg_directions(left_name)) == 0:
                left_sgs[path_node_name] = sgs[path_node_name]
            else:
                right_sgs[path_node_name] = sgs[path_node_name]
        self._make_sgs_file(left_name, left_sgs)
        self._make_sgs_file(right_name, right_sgs)
        self._remove_sg_file(sg_node_name);
        self._store_current_sg_tree();
        self.sg_cache = left_sgs

    def _change_sg_count_in_file(self, sg_node_name, amt):
        f = open(self._get_full_filename(sg_node_name), 'r+');
        newVal = int(f.next().strip()) + amt;
        f.seek(0);
        f.write(string.zfill(str(newVal), len(str(self.sg_node_size))));
        f.close();

    #return a dictionary of node name to start goals for node
    def _read_sgs_from_file(self, sg_node_name):
        f = open(self._get_full_filename(sg_node_name), 'r')
        num_sgs = int(f.next().strip())
        start_goals = dict()
        for line in f:
            pid_string, node_name, sg_string = line.strip().split(" ")
            sg = self._string_to_sg(sg_string)
            if node_name not in start_goals.keys():
                start_goals[node_name] = []
            start_goals[node_name].append((int(pid_string), sg))
        f.close()
        return start_goals

    def _get_sgs(self, sg_node_name):
        #possibly use the cache here somehow
        return self._read_sgs_from_file(sg_node_name);

    def _write_sgs_to_file(self, path_node_name, sgs):
        f = open(self._path_name_to_sg_file(path_node_name), 'a');
        for pid, sg in sgs:
            f.write("%i %s %s\n" % (pid, path_node_name, self._sg_to_string(sg)))
        f.close();

    def _sg_to_string(self, sg):
        return "%s" % ('|'.join([','.join([str(j) for j in p]) for p in sg]));

    def _string_to_sg(self, s):
        return [[float(j) for j in pt.split(',')] for pt in s.split('|')];

    def _find_path_leaf_by_path_name(self, name):
        directions = self._get_path_directions(name);
        current_node = self.tree;
        for d in directions:
            if d == 'l':
                current_node = current_node.left;
            elif d == 'r':
                current_node = current_node.right;
        return current_node;

    def _evaluate_v(self, invalid_sections):
        if len(invalid_sections) == 0:
            return 0;
        else:
            return sum([section[1]-section[0]-1 for section in invalid_sections]);

    def _get_path_leaf_by_sg(self, s, g):
        current_node = self.tree;
        while not current_node.is_leaf():
            use_state = s if current_node.split_state == START else g;
            if use_state[current_node.split_index] <= current_node.split_value:
                current_node = current_node.left;
            else:
                current_node = current_node.right;
        return current_node;

    def _get_sgs_from_path_name(self, path_node_name):
        if path_node_name not in self.sg_cache.keys():
            self._load_sg_cache(path_node_name);
        return self.sg_cache[path_node_name];

    def _load_sg_cache(self, path_node_name):
        sg_node_name = self._get_sg_leaf_by_path_name(path_node_name).name
        self.sg_cache = self._get_sgs(sg_node_name)
        if len(self.sg_cache.keys()) == 0:
            self.sg_cache[ROOT_NAME] = []

    #returns a list of (pid, path_name, path) corresponding to the list of (pid, path_name, sg) in sgs
    def _get_paths_of_sgs(self, sgs):
        rospy.loginfo("Path library: %s" % (str(sgs)))
        sg_dict = dict();
        ret = [];
        for pid, path_name, sg in sgs:
            if not sg_dict.has_key(path_name):
                sg_dict[path_name] = []
            sg_dict[path_name].append(pid)
        for path_name in sg_dict.keys():
            paths = self._get_paths(path_name)
            for pid_from_sg in sg_dict[path_name]:
                for path_with_id in paths:
                    path_id, path = path_with_id
                    if pid_from_sg == path_id:
                        ret.append((path_id, path_name, path))
                        break
        return ret;

    def _get_full_filename(self, name, lib_name=None):
        if lib_name is None:
            lib_name = self.current_lib_name
        return "%s/%s/%s" % (self.path_library_dir, lib_name, name);

    def _get_full_lib_name(self, lib_name=None):
        if lib_name is None:
            lib_name = self.current_lib_name
        return "%s/%s" % (self.path_library_dir, lib_name)

    def _path_name_to_sg_file(self, name):
        return "%s/%s/%s" % (self.path_library_dir, self.current_lib_name, self._get_sg_leaf_by_path_name(name).name);

    def _get_sg_directions(self, name):
        return name[len(SG_ROOT_NAME):];

    def _get_path_directions(self, name):
        return name[len(ROOT_NAME):];

    def _find_sg_leaf_by_dirs(self, directions):
        current_node = self.sg_tree;
        for d in directions:
            if current_node.is_leaf():
                break;
            elif d == 'r':
                current_node = current_node.right;
            elif d == 'l':
                current_node = current_node.left;
        return current_node;

    def _find_sg_leaf_by_sg_name(self, name):
        return self._find_sg_leaf_by_dirs(self._get_sg_directions(name));

    def _get_sg_leaf_by_path_name(self, name):
        return self._find_sg_leaf_by_dirs(self._get_path_directions(name));

    def _calc_dtw_distance(self, path1, path2):
        n, m = len(path1), len(path2);
        table = [[0 for i in xrange(m+1)] for j in xrange(n+1)];
        for i in xrange(1, n+1):
            table[i][0] = float('inf');
        for i in xrange(1, m+1):
            table[0][i] = float('inf');
        table[0][0] = 0;

        for i in xrange(n):
            for j in xrange(m):
                cost = self._line_dist(path1[i], path2[j]);
                table[i+1][j+1] = cost + min(table[i][j+1], table[i+1][j], table[i][j]);

        return table[n][m];

    def _line_dist(self, p1, p2):
        return (sum([(self._angle_between(p1[i], p2[i]))**2 for i in xrange(len(p1))]))**0.5;

    def _proj_dist(self, sg1, sg2):
        return self._line_dist(sg1[START], sg2[START]) + self._line_dist(sg1[GOAL], sg2[GOAL]);

    def _total_dist(self, target_start, target_goal, path):
        return self._line_dist(target_start, path[0]) + self._line_dist(target_goal, path[-1]);

    def _angle_between(self, a, b):
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

    def _normalize_angle(self, angle):
        return angle;
        #return angle % TWO_PI;

    def _normalize_path(self, path):
        return [[self._normalize_angle(angle) for angle in pt] for pt in path];

    ###functions for debugging###

    def find_distances(self, s, g, lib_name):
        dists = []
        for f in os.listdir(self._get_full_lib_name(lib_name)):
            if f.find("pickle") == -1 and f.find(SG_ROOT_NAME) == -1:
                paths = self._get_paths(f)
                for pid, p in paths:
                    dists.append(self._total_dist(s, g, p))
        dists.sort()
        return dists

    def check_accuracy(self, num_iters, n):
        RIGHT_ARM_JOINT_LIMITS = [(-1*math.pi/4-1.35, -1*math.pi/4+1.35), (-0.3536, 1.2963), (-1.55-2.2, -1.55+2.2), (-2.1213, -0.15), (-1*math.pi, math.pi), (-2.0, -0.1), (-1*math.pi, math.pi)];
        for i in xrange(num_iters):
            start = [(pt[1]-pt[0])*random.random()+pt[0] for pt in RIGHT_ARM_JOINT_LIMITS];
            goal = [(pt[1]-pt[0])*random.random()+pt[0] for pt in RIGHT_ARM_JOINT_LIMITS];
            retrieved = self._retrieve_path_simple(start, goal, n);
            actual = self.find_distances(start, goal)[0:n]
            retr = [self._total_dist(start, goal, path) for path in retrieved]
            print "Actual best %i distances: %s" % (n, str(actual))
            print "Retrieved %i distances: %s" % (n, str(retr))
            if actual != retr:
                print "Incorrect retrieval"
            else:
                print "Correct retrieval"

    def _retrieve_path_simple(self, s, g, n):
        leaf_node = self._get_path_leaf_by_sg(s, g)
        #self._load_trees()
        #self._load_sg_cache(leaf_node.name) #load the cache from the file
        closest_n = self._find_closest_n_in_all(s, g, leaf_node, n=n)
        pids, path_names, paths = zip(*closest_n)
        #closest = closest_n[0]
        return paths

    def reorganize_paths_individually(self, old_paths_file, new_node_size=None):
        if new_node_size is None:
            new_node_size = self.node_size;
        else:
            self.node_size = new_node_size;
        self._delete_library_files();
        self._init_lib(node_size=new_node_size);
        if old_paths_file[-1] != '/':
            old_paths_file += '/';
        all_path_files = os.listdir(old_paths_file);
        all_path_files.remove(PATH_TREE_NAME);
        all_path_files.remove(SG_TREE_NAME);
        current_paths = [];
        counter = 0;
        for f in all_path_files:
            if f.find(ROOT_NAME) == 0:
                path_list = self._get_old_paths(old_paths_file+f);
                print "reorganize", f, len(path_list);
                for p in path_list:
                    self.store_path(p);

    def get_path_point(self, pid, index, lib_name):
        for f in os.listdir(self._get_full_lib_name(lib_name)):
            if f.find(ROOT_NAME) == 0:
                for path_id, path in self._get_paths(f):
                    if path_id == pid:
                        if index < len(path):
                            print "Path library: path (in %s) is %s" % (f, ' '.join([str(x) for x in path[index]]))
                            return path[index]
                        else:
                            print "Index is out of range of path (in %s), path is %i points long" % (f, len(path))
                            return None

if __name__ == "__main__":
    STEP_SIZE = 0.02
    if len(sys.argv) == 3 and sys.argv[1] == "reorganize":
        p = PathLibrary(STEP_SIZE, node_size=2, sg_node_size=8)
        p.reorganize_paths(sys.argv[2])
    else:
        test_joints = ['a', 'b']
        p = PathLibrary(STEP_SIZE, node_size=2, sg_node_size=8)
        for i in xrange(3):
            p.store_path([[random.random()*12,1],[1,2],[2,3]], "pr2", test_joints)
        p.delete_path_by_id(1, "pr2", test_joints)
        for i in xrange(3):
            p.store_path([[random.random()*12,1],[1,2],[2,3]], "pr3", test_joints)
        p.delete_path_by_id(1, "pr3", test_joints)
        for i in xrange(3):
            p.store_path([[random.random()*12,1],[1,2],[2,3]], "pr2", test_joints)
        p.delete_path_by_id(3, "pr2", test_joints)
        p.retrieve_path([], [], 0, "blah", "apple", ['a'])
