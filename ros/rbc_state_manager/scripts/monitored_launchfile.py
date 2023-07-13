#!/usr/bin/env python3

import roslaunch
import rospy
import os
import asyncio

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
        self._exit_future = asyncio.Future()

    def start(self):
        self.launch.start()
        self.processes_exit_code = {x.process_name: None for x in self.launch.config.nodes}

    def _processes_exit_code_updated(self):
        if all(code is None for code in self.processes_exit_code.values()):
            return
        if any(code == 1 for code in self.processes_exit_code.values()):
            self._exit_future.set_result(False)
            return
        if all(code == 0 for code in self.processes_exit_code.values()):
            self._exit_future.set_result(True)

    async def wait_launchfile_exit(self):
        return await self._exit_future


async def main():
    raw_launchfile = "~/catkin_ws/src/rbc_state_manager/launch/test.launch"
    absolute_launchfile = os.path.expanduser(raw_launchfile)
    
    rospy.init_node('monitored_launchfile_node', anonymous=True)
    
    monitored_launchfile = MonitoredLaunchfile(absolute_launchfile)
    monitored_launchfile.start()
    
    exit_status = await monitored_launchfile.wait_launchfile_exit()
    print(f"All processes exited, success: {exit_status}")

    rospy.signal_shutdown('All processes have exited')

if __name__ == "__main__":
    asyncio.run(main())