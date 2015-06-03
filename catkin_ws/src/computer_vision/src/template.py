#import whatever you need to import here
import cv2
import plumber

# define your function
def filter_image(image):       # this would be filter_image(self,image) if you decided to remove @staticmethod
    # do something with your image
    # you can do all sorts of magic here but as an example, I'll just do scaling by 1/3
    return cv2.resize(image, None, fx=0.333, fy=0.333)

# That's it you have defined your first filtering operation
# let's say you want to publish the information

# hexadatum is a class in plumber file you can use to make your life easier
def generate_msg(image):
    # here from your image you will find the spam you want to publish
    spam = 1
    # hexadatum takes x,y,z,pitch,yaw,task, optional_description
    return plumber.hexadatum(spam,spam,spam,spam,spam,plumber.hexadatum.Target.NOTHING,"some optional msg")
    
