from tlibot_sdkpy.rpc.client import Client
from tlibot_sdkpy.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
import time
import json

class TestClient(Client):
    def __init__(self, serviceName, enabaleLease = True):
        super().__init__(serviceName, enabaleLease)
    
    def init(self):
        self._SetApiVerson("1.0.0")
        self._RegistApi(1001, 0)

    def Test(self, param: str):
        #p = {}
        #p["data"] = param
        #parameter = json.dumps(p)
        code, data = self._Call(1002, param)
        print("code:", code, "data:", data)
        return code
    
    
    
if __name__ == "__main__":    
    ChannelFactoryInitialize(0)
    
    client = TestClient("motion_service")    
    client.init()    
    #client.Test("aabbcc")
    #client.Test("second")
    #client.Test("third call")
    
    while True:
        time.sleep(1)