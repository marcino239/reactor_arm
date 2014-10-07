#!/usr/bin/env python

import rospy
import wx
import wx.lib.newevent
import xml.etree.ElementTree as ET

from optparse import OptionParser
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread
from dynamixel_msgs.msg import JointState as DXJointState
from std_msgs.msg import Float64

RANGE = 10000

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointStatePublisher():
    def __init__(self, deps):
        
        self.controller_list = deps
        self.joint_positions = {}
        
        self.joint_state_sub = {}
        self.joint_command_pub = {}
        self.joint_list = []
        self.joints = {}
        
        # create subscription list for each joint and command publishers
        for controller_name in deps:
            joint = get_param( '/' + controller_name + '/joint')
            
            self.joint_list.append( joint )
            
            s = '/' + controller_name + '/state' 
            print( 'subscribing to joint %s' % s )
            self.joint_positions[ joint ] = 0
            self.joint_state_sub[ joint ] = rospy.Subscriber( s, DXJointState, self.process_joint_states )

            s = '/' + controller_name + '/command' 
            print( 'publishing to joint %s' % s )
            self.joint_command_pub[ joint ] = rospy.Publisher( s, Float64, queue_size=5 )

        # process the URDF xml to check joint constrains
        description = get_param('robot_description')
        robot = ET.fromstring(description)

        for joint_name in self.joint_list:
            limits = robot.findall( ".//*[@name='" + joint_name + "']/limit" )
            if len( limits ) > 1:
                raise ROSException( 'malformed URDF: more than 1 joint: ' + joint_name )
            elif len( limits ) == 0:
                raise ROSException( 'URDF shows no joint: ' + joint )
        
            limit = limits[0]
            minval = float(limit.get('lower'))
            maxval = float(limit.get('upper'))
            
            zeroval = 0
            self.joints[ joint_name ] = {'min':minval, 'max':maxval, 'zero':zeroval}

        use_gui = get_param("use_gui", False)

        if use_gui:
            self.app = wx.App()
            self.gui = JointStatePublisherGui("Joint State Publisher", self)
            self.gui.Show()
        else:
            self.gui = None

        source_list = get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(rospy.Subscriber(source, JointState, self.source_cb))

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def process_joint_states(self, state_list):
        self.joint_positions[ state_list.name ] = state_list.current_pos

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
 
            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # post an event here instead of directly calling the update_sliders method, to switch to the wx thread
            wx.PostEvent(self.gui.GetEventHandler(), self.gui.UpdateSlidersEvent())
        

    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        dx_msg = Float64()

        # Publish Joint States
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()

            if delta > 0:
                self.update(delta)

            # Initialize msg.position, msg.velocity, and msg.effort.
            has_position = len(self.joints.items()) > 0
            has_velocity = False
            has_effort = False
            for (name,joint) in self.joints.items():
                if not has_position and 'position' in joint:
                    has_position = True
                if not has_velocity and 'velocity' in joint:
                    has_velocity = True
                if not has_effort and 'effort' in joint:
                    has_effort = True
            num_joints = len(self.joints.items())
            
            if has_position:
                msg.position = num_joints * [0.0]
            if has_velocity:
                msg.velocity = num_joints * [0.0]
            if has_effort:
                msg.effort = num_joints * [0.0]
            
            for i, name in enumerate(self.joint_list):
                msg.name.append(str(name))
                joint = None

                # Add Free Joint
                if name in self.joints:
                    joint = self.joints[name]
                    factor = 1
                    offset = 0
                
                if has_position and 'position' in joint:
                    pos = self.joint_positions[ name ]
                    msg.position[i] = pos
                    dx_msg.data = pos
                    
                if has_velocity and 'velocity' in joint:
                    msg.velocity[i] = joint['velocity'] * factor
                if has_effort and 'effort' in joint:
                    msg.effort[i] = joint['effort']

            self.pub.publish(msg)
            r.sleep()

    def update(self, delta):
        for name, joint in self.joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    joint['position'] = joint['max']
                    joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward

class JointStatePublisherGui(wx.Frame):
    def __init__(self, title, jsp):
        wx.Frame.__init__(self, None, -1, title, (-1, -1));
        self.jsp = jsp
        self.joint_map = {}
        panel = wx.Panel(self, wx.ID_ANY);
        box = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)
        
        ### Sliders ###
        for name in self.jsp.joint_list:
            if name not in self.jsp.joints:
                continue
            joint = self.jsp.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            row = wx.GridSizer(1,2)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)

            display = wx.TextCtrl (panel, value=str(0), 
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)

            row.Add(display, flag= wx.ALIGN_RIGHT| wx.ALIGN_CENTER_VERTICAL)
            box.Add(row, 1, wx.EXPAND)
            slider = wx.Slider(panel, -1, RANGE/2, 0, RANGE, 
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider.SetFont(font)
            box.Add(slider, 1, wx.EXPAND)

            self.joint_map[name] = {'slidervalue':0, 'display':display, 
                                    'slider':slider, 'joint':joint}

        self.UpdateSlidersEvent, self.EVT_UPDATESLIDERS = wx.lib.newevent.NewEvent()
        self.Bind(self.EVT_UPDATESLIDERS, self.updateSliders)
        
        ### Buttons ###
        self.ctrbutton = wx.Button(panel, 1, 'Center')
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        
        wx.EVT_BUTTON(self, 1, self.center_event)

        box.Add(self.ctrbutton, 0, wx.EXPAND)
        
        panel.SetSizer(box)
        self.center()
        box.Fit(self)
        self.update_values()


    def update_values(self):
        for (name,joint_info) in self.joint_map.items():
            purevalue = joint_info['slidervalue']
            joint = joint_info['joint']
            value = self.sliderToValue(purevalue, joint)
            joint['position'] = value
        self.update_sliders()

    def updateSliders(self, event):
        self.update_sliders()

    def update_sliders(self):
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['position'],
                                                           joint)
            joint_info['slider'].SetValue(joint_info['slidervalue'])
            joint_info['display'].SetValue("%.2f"%joint['position'])

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['zero'], joint)
        self.update_values()

    def sliderUpdate(self, event):
        for (name,joint_info) in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].GetValue()
        self.update_values()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])
        
    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue


if __name__ == '__main__':

    try:
        rospy.init_node('joint_state_publisher')

        #process dependant controller names
        parser = OptionParser()
        (options, args) = parser.parse_args(rospy.myargv()[1:])
        joint_controllers = args

        jsp = JointStatePublisher( joint_controllers )

        if jsp.gui is None:
            jsp.loop()
        else:
            Thread(target=jsp.loop).start()
            jsp.app.MainLoop()
        
    except rospy.ROSInterruptException: pass
