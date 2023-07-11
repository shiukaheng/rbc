#!/usr/bin/env python3

import roslaunch
import os
import threading

class MonitoredLaunchFileProcessListener(roslaunch.pmon.ProcessListener):
    def __init__(self, monitored_launchfile):
        self.monitored_launchfile = monitored_launchfile

    def process_died(self, name, exit_code):
        if self.monitored_launchfile.processes_exit_code is None:
            raise RuntimeError("Process died before launchfile was started, should not happen")
        if name not in self.monitored_launchfile.processes_exit_code:
            raise RuntimeError("Process died that was not in launchfile")
        
        self.monitored_launchfile.processes_exit_code[name] = exit_code
        self.monitored_launchfile._processes_exit_code_updated()

class MonitoredLaunchfile:
    def __init__(self, path):
        self.path = path
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.path], 
                                                        process_listeners=[MonitoredLaunchFileProcessListener(self)])
        self.processes_exit_code = None
        self.observers = set()

        self.all_processes_exited = threading.Event()

    def start(self):
        self.launch.start()
        self.processes_exit_code = {x.process_name: None for x in self.launch.config.nodes}

    def _processes_exit_code_updated(self):
        if all(code is None for code in self.processes_exit_code.values()):
            print("All processes started")
            return
        if any(code == 1 for code in self.processes_exit_code.values()):
            print("Some processes exited with error")
            self.notify_observers(False)
            return
        if all(code == 0 for code in self.processes_exit_code.values()):
            print("All processes exited with success")
            self.notify_observers(True)
            self.all_processes_exited.set()
        else:
            print("Unknown exit code combination")

    def subscribe(self, observer):
        self.observers.add(observer)

    def unsubscribe(self, observer):
        self.observers.discard(observer)

    def notify_observers(self, result):
        for observer in self.observers:
            observer.update(result)

    def kill(self):
        self.all_processes_exited.set()
    
    def stop(self):
        self.launch.shutdown()

class Observer:
    def update(self, result):
        print(f"All processes exited, success: {result}")

def main():
    raw_launchfile = "~/catkin_ws/src/rbc_state_manager/launch/test.launch"
    absolute_launchfile = os.path.expanduser(raw_launchfile)

    monitored_launchfile = MonitoredLaunchfile(absolute_launchfile)
    monitored_launchfile.start()

    observer = Observer()
    monitored_launchfile.subscribe(observer)

    monitored_launchfile.all_processes_exited.wait()

if __name__ == "__main__":
    main()
    
# Todo: Passing out errors / exit codes if something goes wrong