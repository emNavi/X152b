#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray,Int16
import rospy
import sys
import yaml
import time


#### TODO：
#@ 1.场景中的无人机数检测
#@ 2.能够分辨无人机
############
MAX_RATE = 200
SHOW = True
############
test_time_seq={}
wait_flag=False
times=0
send_flag=False
def callback(data):
    global drone_id,test_time_seq,SHOW
    if(len(data.data) == 5):
        req_num = data.data[0]
        req_id = data.data[1]
        req_time = data.data[2]
        cb_id = data.data[3]
        cb_time = data.data[4]
        if(abs(drone_id - req_id) < 10e-6 ):
            time_drift = (time.time()*1000 - req_time)/2+ req_time - cb_time
            test_time_seq.pop(str(round(req_num)))
            if(SHOW):
                print("ID {}######REQ-Callback############".format(str(round(req_num))))
                print("GoBack  delay ID {} to ID {} == {}ms ".format(
                drone_id,round(cb_id),(time.time()*1000 - req_time)))
                print("Estimate time drift  ID {} to ID {} == {}ms ".format(
                drone_id,round(cb_id),time_drift))
        else:
            print("ood")


    elif(len(data.data) == 3):
        req_num = data.data[0]
        req_id = data.data[1]
        req_time = data.data[2]
        if(abs(drone_id - req_id) > 10e-6):

            cb_id = drone_id
            cb_time = time.time()*1000

            data = Float64MultiArray()
            data.data = [req_num,req_id,req_time,cb_id,cb_time]
            ping_publisher.publish(data)
        else:
            # print("[ping]ERROR:Recv self request")
            pass
    else:
        print("[ping]ERROR:Msg len ({}) != 3/5".format(len(data.data)))


def test_cb(data):
    global drone_id,ping_publisher,test_time_seq,wait_flag,times,send_flag
    times = data.data
    # print("ID drone_id ping test {} times".format(times))

    # test_time_seq = {}
    # rate = rospy.Rate(300) # 10hz

    # for i in range(times):
    #     req_id = drone_id
    #     req_time = time.time()*1000
    #     ping_data = Float64MultiArray()
    #     ping_data.data = [i,req_id,req_time]
    #     test_time_seq[str(round(i))]=req_time
    #     ping_publisher.publish(ping_data)
    #     rate.sleep()
    # wait_flag = True
    send_flag = True


if __name__ == '__main__':
    rospy.init_node('udp_delay_test_node', anonymous=True)
    with open("/home/qyswarm/param_files/real_use/drone_param.yaml", "r") as stream:
        try:
            dictionary = yaml.safe_load(stream)
            drone_id = dictionary["drone_id"]            
        except yaml.YAMLError as exc:
            print(exc)
            sys.exit(1)
    print("Ping Node: self drone_id == {}".format(drone_id))

    ping_publisher = rospy.Publisher("/ping/send", Float64MultiArray, queue_size=10)
    rospy.Subscriber("/ping/recv", Float64MultiArray, callback)
    rospy.Subscriber("/ping/test", Int16, test_cb)

    rate = rospy.Rate(10) # 10hz
    wait_times =  10
    time_index = 0
    while not rospy.is_shutdown():
        if(wait_flag):
            time_index += 1
            if(time_index>wait_times):
                time_index=0
                wait_flag = False
                print("[Ping Result] Drop Rate : {}".format(len(test_time_seq)/times))
                print(test_time_seq)
        elif(send_flag):
            print("ID drone_id ping test {} times".format(times))

            test_time_seq = {}
            send_rate = rospy.Rate(MAX_RATE) # 10hz

            for i in range(times):
                req_id = drone_id
                req_time = time.time()*1000
                ping_data = Float64MultiArray()
                ping_data.data = [i,req_id,req_time]
                test_time_seq[str(round(i))]=req_time
                ping_publisher.publish(ping_data)
                send_rate.sleep()
            wait_flag=True
            send_flag=False
            print("wait recv cb")

        rate.sleep()
    rospy.spin()  # 保持ROS节点运行
