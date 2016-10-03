#!/usr/bin/env python  

'''
 Copyright (c) 2016, Juan Jimeno
 
 All rights reserved. 
 
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 
 
 * Redistributions of source code must retain the above copyright notice, 
 this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
 notice, this list of conditions and the following disclaimer in the 
 documentation and/or other materials provided with the distribution. 
 * Neither the name of  nor the names of its contributors may be used to 
 endorse or promote products derived from this software without specific 
 prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 POSSIBILITY OF SUCH DAMAGE. 
'''
 
import rospy
import tf
import localization as lx
import serial

def parse_data(data_string):
    range_dict={}
    if data_string != None:
        current_key = ""
        #iterate message string without first and last character of message
        for i in range(1,len(data_string)): 
            #anchor id is located at odd indexes       
            if i % 2 != 0:
                current_anchor = data_string[i]
                
            #anchor range is located at even indexes    
            else:
                current_range = data_string[i]
                
                #filter out data that doesn't make sense
                if float(current_range) < MAX_RANGE and float(current_range) > MIN_RANGE:
                    #populate dictionary
                    range_dict[current_anchor] = current_range
                else:
                    break
                    
        return range_dict

def get_transforms(anchors):
    transform_dict = {}
    #iterate and get the transforms for each anchor
    for anchor in anchors:
        try:
            (trans,rot) = listener.lookupTransform('/map', anchor, rospy.Time(0))
            transform_dict[anchor] = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass         
               
    return transform_dict
    
def get_target_location(transforms, ranges):
    P = lx.Project(mode="3D",solver="LSE")
    
    #define anchor locations
    for anchor in transforms:        
        P.add_anchor(anchor, transforms[anchor])
    t, label = P.add_target()

    #define anchor ranges
    for anchor in ranges:       
        t.add_measure(anchor,ranges[anchor])

    P.solve()
    B = t.loc

    return {'x':B.x, 'y':B.y, 'z':B.z}
    
def read_serial_data():
    #PROTOCOL: "$TOTAL_DEVICES_AROUND,ADDRESS_1,RANGE_1,ADDRESS_2,RANGE_2,ADDRESS_N,RANGE_N,TOTAL_DEVICES_SENT\r\n"
    data_string = ""
    parsed_data_string = ""
    
    #tell Arduino to send the data
    ser.write('+')
    # print "hello"
    #check the first character of the message 
    if ser.read() == '$':
        while True:
            c = ser.read()
            data_string += c
            # print data_string
            # print data_string
            #stop collecting characters at end of message
            if c == '\n':
                #omit start and end characters
                data_string = data_string.rstrip('$\r\n')  
                try:
                    #parse anchor ids and ranges from message
                    parsed_data_string = data_string.split(',')
                    total_anchors = int(parsed_data_string[0]) 
                                     
                    #exit when there's not enough anchors around
                    if total_anchors < MIN_ANCHOR or total_anchors == 0:
                        duration = rospy.get_time() - start_time
                        #prevent from complaining on sensor boot up
                        if duration > 5:
                            rospy.logwarn("Not enough anchors. Make sure anchors are powered on or try moving around near the anchors.")
                        break
                    else:
                        return parsed_data_string                       
                except:
                    rospy.logwarn("Error parsing reading serial data")                 
                    pass                      
            # elif c == '$':
            #     break
if __name__ == '__main__':

    rospy.init_node('lips')
    listener = tf.TransformListener()
    start_time = rospy.get_time()
    #create rosparameters
    MIN_RANGE = rospy.get_param('/lips/min_range', 0.5)
    MAX_RANGE = rospy.get_param('/lips/max_range', 10.0)
    MIN_ANCHOR = rospy.get_param('/lips/min_anchor', 2)
    FRAME_ID = rospy.get_param('/lips/frame_id', 'uwb_tag')
    SERIAL_PORT = rospy.get_param('/lips/serial_port', '/dev/ttyUSB0')
    
    #rosparam logs just to make sure parameters kicked in
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/min_range'), MIN_RANGE)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/max_range'), MAX_RANGE)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/min_anchor'), MIN_ANCHOR)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/frame_id'), FRAME_ID)
    rospy.loginfo("%s is %s", rospy.resolve_name('/lips/serial_sport'), SERIAL_PORT)
     
    ser = serial.Serial(SERIAL_PORT, 57600)
    rospy.loginfo("Connected to %s", ser.portstr)
 
    while not rospy.is_shutdown():
        rospy.sleep
        #get the range of each anchor
        ranges = parse_data(read_serial_data())
        
        #only perform calculations when there's valid ranges from the anchors
        if ranges != None:
            anchors = []
            
            #populate anchor dictionary 
            for anchor in ranges:
                anchors.append(anchor)
                
            #get transforms of each anchor
            transforms = get_transforms(anchors)
            
            if(len(transforms) >= MIN_ANCHOR):
                try:
                    #perform trilateration using anchor ranges
                    pos = get_target_location(transforms, ranges)
                except:
                    rospy.logwarn("Localization Error")

                #broadcast the tag's transform from map -> tag (FRAME_ID)   
                br = tf.TransformBroadcaster()
                br.sendTransform((pos['x'], pos['y'], pos['z']),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(),
                                FRAME_ID,
                                "map")