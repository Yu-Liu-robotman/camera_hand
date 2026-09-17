from tlibot_sdkpy.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
import sys
import time

from tlibot_sdkpy.idl.msg.dds import *
from tlibot_sdkpy.idl.default import *

if __name__ == '__main__':
   # print("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
        
    pub = ChannelPublisher("test_topic", Response)
    pub.Init()
    
    msg = default_Response()
    
    while True: 
        # msg.seq += 1
        # msg.timestamp_usx = int(time.time() * 1e6)
        # msg.command_space = 1
        # msg.robot_type = 1
        pub.Write(msg)
        time.sleep(1)
        