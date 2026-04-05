#!/usr/bin/env python3

import rospy
import roslaunch
import rospkg

import os
import yaml

""" 
# 配置文件路径
# 配置yaml配置文件路径, 需要此文件位于的scripts文件夹与yaml文件所在的config文件夹在同一目录
# 配置launch文件路径，需要此文件位于的scripts文件夹与launch文件所在的launch文件夹在同一目录
"""
yaml_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "launch_registry.yaml"))
launch_dir_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "launch"))

VALID_STATES = {
    "STOP",
    "STARTING",
    "RUNNING",
    "STOPPING",
    "ERROR",
}


class LaunchManager(object):
    def __init__(self):
        self.registry = {}
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.build_registry()
    def build_registry(self):
        config_file = yaml_path
        with open(config_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
            data = config.get("modules", {})
        for module_name, module_info in data.items():
            self.registry[module_name] = {
                "package": module_info.get("package", ""),
                "launch": module_info.get("launch", ""),
                "state": "STOP",
                "parent": None
                }
        

    def start_module(self, name):
        if self.get_module_status(name) == "STARTING" or "RUNNING" or None:
            return False
        try:
            entry = self.registry[name]
            entry["state"] = "STARTING"
            parent = roslaunch.parent.ROSLaunchParent( self.uuid, [entry["launch_file"]] )
            parent.start(auto_terminate=False)
            entry["parent"] = parent
            entry["state"] = "RUNNING"
            rospy.loginfo("module [%s] start", name)
            return True
        except Exception as e:
            entry["state"] = "ERROR"
            entry["parent"] = None
            rospy.logerr("Start module [%s] failed: %s", name, str(e))
            return False

    def stop_module(self, name):
        if self.get_module_status(name) != "RUNNING":
            return False
        entry = self.registry[name]
        if entry["state"] == "STOPPED":
            return True
        try:
            entry["state"] = "STOPPING"
            if entry["parent"] is not None:
                entry["parent"].shutdown()
                entry["parent"] = None

            entry["state"] = "STOPPED"

            rospy.loginfo("Module [%s] stopped", name)
            return True
        except Exception as e:
            entry["state"] = "ERROR"
            rospy.logerr("Stop module [%s] failed: %s", name, str(e))
            return False

    def get_module_status(self, name):
        if name not in self.registry:
            return None
        return self.registry[name]["state"]
            

class LaunchManageNode:
    def __init__(self):
        rospy.init_node("launch_manager_node")
    
    