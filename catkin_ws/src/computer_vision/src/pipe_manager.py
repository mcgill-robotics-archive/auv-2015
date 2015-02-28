from cv_bridge import CvBridge, CvBridgeError
import functools
import time
class pipe_piece(object) :
    def __init__(self, parent, operation= lambda image: image, name="", cons=None):
        """Creates a node in the pipe. A name can be given if this node is an important throttle point.
        Args:
        parent := the parent node of this node
        operation := a function that takes an image and returns an image ie operation(image) -> image
        name := a string to specify this node as a potential target for signals
        subtrees := a list of nodes subsequent to this node
        """
        self.name = name if not name else "root" ##aka is not empty or it will be root
        self.parent = parent
        self.cons = cons
        self.operation = operation
    def signal( self, operation, designated_node) :
        """Send a command down the pipe. Designate a node if it is intended for one node only.
        Args:
        operation := function to operate on this node
        """
        if self.name is designated_node or not designated_node:
            operation(self)
        self.cons.signal( self, operation, designated_node)
    def curry_funk(self, conses=[]):
        conses.append(self.operation)
        if self.cons :
            return self.cons.curry_funk(conses)
        else:
            print "Curry up"
            print conses
            return functools.reduce(lambda f, g: lambda x: f(g(x)), conses)
    
class pipe_section(pipe_piece) :
    def __init__(self, parent, operation=lambda image: image, name="", cons=None):
        super(pipe_section, self).__init__( parent, operation=lambda image: image, name="", cons=None)
        
class pipe_end(pipe_piece) :
    def __init__(self, parent, publisher, operation=lambda image: image, name="", cons=None):
        """
        Arg:
        publisher := is a 2-tuple (p, m)
            p := ros publisher
            m := function that will generate message to publish
                it is of form m(bridge, image)
        """
        self.bridge = CvBridge()
        self.publisher, self.message_generator = publisher
        super(pipe_end, self).__init__( parent, operation=lambda image: image, name="", cons=None)

class image :
    def __init__(self, cv_image, t = None) :
        self.stuffing = cv_image
        self.timestamp = time.time() if not t else t
    def copy(self) :
        image = image(this.content.copy())
        image.timestamp = this.timestamp
        return image
