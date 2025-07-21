import requests
import time
import os
import ssl

class RotavaporController():
    ## ip等基础设置
    ip = "192.168.1.110"
    url = f"https://{ip}/api/v1"
    process = url + "/process"

    ## 用户名密码从平蒸仪操作屏幕上获取，不用改
    user = "rw"
    pwd = "1KWtaEcc"
    session = requests.Session()
    session.auth = (user, pwd)

    ## 默认证书
    session.verify = 'root_cert.crt'
    # session.verify = False

    def __init__(self):
        pass

    def pub_cmd(self, cmd):

        ## 三个仪器都只能通过改变globalStatus的值来统一同时启动停止
        # set_vacuum_msg = { 'globalStatus' : { 'running': True } }

        ## 设置加热温度：40，真空气压：1000，旋转转速：200， 总体状态：启动：否
        if cmd == "start":
            set_vacuum_msg = {
                'heating': {'set': 40},
                'vacuum': {'set': 200},
                'rotation': {'set': 200},
                'globalStatus': { 'running': True }
                }
        elif cmd == "stop":
            set_vacuum_msg = {
                'heating': {'set': 40},
                'vacuum': {'set': 200},
                'rotation': {'set': 200},
                'globalStatus': { 'running': False }
                }
        elif cmd == "air_start":
            set_vacuum_msg = {
                'heating': {'set': 40},
                'vacuum': {'set': 200},
                'rotation': {'set': 0},
                'globalStatus': { 'running': True }
                }
        elif cmd == "air_stop":
            set_vacuum_msg = {
                'heating': {'set': 40},
                'vacuum': {'set': 200},
                'rotation': {'set': 0},
                'globalStatus': { 'running': False }
                }
        elif cmd == "find_position_start":
            set_vacuum_msg = {
                'heating': {'set': 40},
                'vacuum': {'set': 200},
                'rotation': {'set': 60},
                'globalStatus': { 'running': True }
                }            
        elif cmd == "find_position_stop":
            set_vacuum_msg = {
                'heating': {'set': 40},
                'vacuum': {'set': 200},
                'rotation': {'set': 60},
                'globalStatus': { 'running': False }
                }  
        ## 发消息给平蒸仪
        set_vacuum_resp = self.session.put(self.process, json=set_vacuum_msg)

    def recv_info(self):

        ## 收平蒸仪消息
        res = self.session.get(self.process)

        ## 处理格式
        info = res.json()

        ## 提取并打印
        # system_name = info['globalStatus']
        # system_name = info
        # print(system_name)
        return info


# rc = RotavaporController()
# # rc.pub_cmd("find_position_start")
# rc.pub_cmd("start")
# time.sleep(10.0)
# rc.pub_cmd("find_position_stop")
