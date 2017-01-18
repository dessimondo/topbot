#!/usr/bin/env python
import SocketServer
import subprocess
import rospy
from duckietown_msgs.msg import BoolStamped, FSMState
flag = False

HOST, PORT = "0.0.0.0", 9002

class Timer(object):
    """When not seeing apriltags for X secs, set flag to true to send alert"""
    def __init__(self):
        self.node_name = 'send_alert_node'
        self.alert_sent = BoolStamped()
        self.alert_sent.data =False

        #self.sub_switch = rospy.Subscriber(~switch, BoolStamped, self.cbSwitch,queue_size=1)
        self.sub_alert = rospy.Subscriber("~detection", BoolStamped, self.callback, queue_size=1)
        self.pub_alert_sent = rospy.Publisher("/topbot/send_alert_node/alert_sent",BoolStamped,queue_size=1, latch=True)
        try:
            server = SocketServer.TCPServer((HOST, PORT), NodeTCPHandler)
            server.serve_forever()
        except KeyboardInterrupt:
            print NodeString, "keyboard interrupt!"
            server.shutdown()
            server.server_close()

    def callback(self, detection):
        print "send alert callback call"
        global flag
        if detection:
            print "flag true"
            flag = True
        else:
            print "shutdown timer"
            flag = False
#TCP Handler class, to process SocketServer.TCPServer.
class NodeTCPHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        #for testing only, set flag as true
        self.data = self.request.recv(1024).strip();
        print "{} wrote:".format(self.client_address[0])
        print "%s" %  self.data
        #print subprocess.Popen(self.data, shell=True, stdout=subprocess.PIPE).stdout.read()
        #return_code = subprocess.call(self.data)
        #s = self.data.split(";")
        global flag
        print"data connect"
        if flag == True:
            print "time's up"
            self.request.sendall("alert")
            self.alert_sent = BoolStamped()
            self.alert_sent.data = True
            self.pub_alert_sent.publish(self.alert_sent)
            flag = False
        else:
            print"flag false"
if __name__ == "__main__":
    rospy.init_node('Timer',anonymous=False)
    node = Timer()
    rospy.spin()
    #try:
    #    server = SocketServer.TCPServer((HOST, PORT), NodeTCPHandler)
    #    server.serve_forever()
    #except KeyboardInterrupt:
    #    print NodeString, "keyboard interrupt!"
    #    server.shutdown()
    #server.server_close()
