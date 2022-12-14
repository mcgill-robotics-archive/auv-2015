#!/usr/bin/env python
import argparse
import roslib
import sys
import numpy
import rospy
import cv2
import functools
import logging
import Queue
import collections
import thread as t
from threading import Thread
from plumber import *
from image_node import image_in
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from computer_vision.msg import ObjectImageLocation

class simple_pipe :
    def __init__(self, plan, camera="/camera/image_rect_color", threads=1, show=False, name="pipe"):
        """
        A simple pipeline is a composite of functions that will filter an image from a stream of ROS images.

        It is base on a single layer supplier-consumer model, where the consumer is the composite function.

        Args:
        plan := A string describing the layout of the pipeline (ex. "gray > scale" will turn image gray then scale it)
        camera := A string denoting the ROS topic that generates the image feed (default "/camera/image_rect_color" )
        threads := A number denoting the number of threads to be used
        show := A boolean denoting if a gui window should be shown to display the result of the filters
        name := A string name for the publisher node
        """
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        self.bridge = CvBridge()
        self.pipeline = [function_name.strip().split() for function_name in plan.split(">")]
        self.pipeline_args = [self.read_config(name_with_params[1]) if len(name_with_params)>1 else dict() for name_with_params in self.pipeline]
        self.pipeline = [ self.retrieve_function(lst[0]) for lst in self.pipeline ]
        self.queue_in = Queue.Queue()
        self.queue_out = Queue.PriorityQueue()
        self.out = collections.deque(maxlen=20)
        self.thread_quantity = threads if threads >= 1 else 1
        self.node_name = name
        self.publisher = rospy.Publisher( name, ObjectImageLocation, queue_size=100 )
        self.debuglisher = rospy.Publisher( "debug_"+name, Image, queue_size=100 )
        def callback(data):
            try:
              cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
              self.queue_in.put_nowait(filter_image(cv_image))
            except CvBridgeError, e:
              self.logger.debug("Error occurred in callback function : %s",e)
              rospy.logerr("Error occurred in callback function : %s",e)
        self.subscriber = rospy.Subscriber(camera,Image,callback)
        def work():
            process = self.pipe_process(self.pipeline, self.pipeline_args)
            while True:
                f_image = self.queue_in.get(block="true")
                try:
                    f_image.update_content(process(f_image).content)
                except Exception as e:
                    print e
                self.queue_in.task_done()
                self.queue_out.put_nowait(f_image)
        for i in range(self.thread_quantity):
            thread = Thread(target=work)
            thread.daemon = True
            thread.start()
        # if images are not consumed they will be disposed
        def outflow():
            while True:
                self.out.append(self.queue_out.get(block="true"))
        overflow_manager = Thread(target=outflow)
        overflow_manager.daemon = True
        overflow_manager.start()
        def display():
            while True:
                #print self.queue_out.qsize()
                image_out = self.queue_out.get(block=True)
                cv2.imshow("Image", image_out.content)
                if cv2.waitKey(5) == ord('q') :
                    # send keyboard interrupt to main thread and then end thread
                    t.interrupt_main()
                    self.queue_out.task_done()
                    exit(0)
                self.queue_out.task_done()
        if show:
            try:
                gui_thread = Thread(target=display)
                gui_thread.daemon = True
                gui_thread.start()
            except KeyboardInterrupt:
                rospy.exit()
    
    def retrieve_function(self, function_name):
        """
        Retrives the function specified by function_name in the plumber file

        Arg:
        function_name := the input name derived from the command line

        Return:
        the function corresponding to the function_name
        """
        try :
            f = getattr(plumber, function_name)
            rospy.loginfo("%s has been added to pipeline", function_name)
            return f
        except AttributeError as e:
            self.logger.info("A faulty function was given to pipe, %s was not added. Check error log for more info", function_name)
            self.logger.debug("%s was not added because : %s", function_name, e)
            rospy.logwarn("%s was not added because : %s", function_name, e)
            return lambda image:image # An identity function is returned to prevent breakage

    def pipe_process(self, pipeline, arguments):
        """
        This function produces a process, the composite function of filters

        Arg:
        pipeline := list of functions (filters)
        arguments := arguments for functions in the pipeline. Their indices should correspond to the indices of the filter in the pipeline

        Return:
        the function process:image->image which takes reduces all functions in pipeline and their respective arguments
        """
        
        def process(image):
            """
            This function is the process function for the pipeline.

            Arg:
            image := a filter image, it has a opencv image and a timestamp

            Return:
            filtered image
            """
            if not isinstance(image, filter_image):
                rospy.logerr("the image is not the right type it is : %s", type(image))
            for i, (filtering_op, args) in enumerate(zip(pipeline, arguments)):
                try:
                    image.content = filtering_op(**dict({"image":image.content, "publisher":self.publisher}.items()+ args.items() ))
                except TypeError as e :
                    rospy.logerr("A filter's output does not match the input of the subsequent filter : output of %s and input of %s", self.pipeline[i-1].__name__, self.pipeline[i].__name__)
                    del pipeline[i-1:i+1]
                    del arguments[i-1:i+1]
                    rospy.logwarn("Gracefully removed faulty filters")
            # send to debug publisher
            output_type = "8UC1"
            error_count = 0
            try:
                #ros_image = self.bridge.cv2_to_imgmsg(image.content, "8UC1")
                ros_image = self.bridge.cv2_to_imgmsg(image.content, output_type)
                self.debuglisher.publish(ros_image)
            except Exception as e:
                if not error_count:
                    output_type = str(e).rsplit(None, 1)[-1]
                    error_count = error_count + 1
                else:
                    print e
            return image
        return process

    def read_config(self, path):
        """
        Reads file at the path and parses it for arguments for its corresponding function.

        Arg:
        path := the path of the file

        Return:
        a dictionary of all argument names and values pair in file specified by path
        """
        arguments = dict()
        try:
            with open(path) as config_file:
                for line in config_file:
                    args = [arg.strip() for arg in line.split()]
                    if len( args ) >= 3:
                        lowercase = args[0].lower()
                        if lowercase == "string":
                            arguments[args[1]] = args[2:]
                        elif lowercase == "number":
                            arguments[args[1]] = [float(num) for num in args[2:]]
                        elif lowercase == "int" or lowercase == "integer":
                            arguments[args[1]] = [int(num) for num in args[2:]]
                        else:
                            # error parsing the line
                            rospy.logerr( "The line %s in file %s must conform to format <type> var_name value_0 ... value_n", line, path)
                        if len(args) == 3:
                            # take the value from the list for convenience in the filtering operations' argument reads
                            arguments[args[1]] = arguments[args[1]][0]
        except IOError as e:
            rospy.logerr("Cannot open file %s", path)
        return arguments
                        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Create an image processing pipeline in ROS system")
    parser.add_argument("plan", help='Filter names (ex. "gray > resize" )' )
    parser.add_argument("-c", "--camera", help="Camera feed message name (default: /camera/image_rect_color)")
    parser.add_argument("-t", "--threads", help="Number of threads to be used in this pipeline (default: 1)", type=int)
    parser.add_argument("-s", "--show", help="Show result in a window", action="store_true")
    parser.add_argument("-n", "--name", help="Name the ROS node (default: pipe")
    arguments = vars(parser.parse_args())
    plan = arguments["plan"]
    del arguments["plan"]
    arguments = { key:value for (key,value) in arguments.iteritems() if value }
    if "name" in arguments :
        arguments["name"] = arguments["name"].split()[0]
        name = arguments["name"]
    else :
        name = "pipe"
    rospy.init_node(name, anonymous=True)
    simple_pipe(plan, **arguments)
    try:
        rospy.spin()
    except KeyboardInterrupt:
       print "Shutting down"
    cv2.destroyAllWindows()
