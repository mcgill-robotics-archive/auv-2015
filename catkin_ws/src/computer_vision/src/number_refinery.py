#!/usr/bin/env python
import cv2
import argparse       
import numpy as np
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="""Takes a folder containing extracted characters and iterates through them to create a sample set""")
    parser.add_argument("src", help="Source folder where the images are located.")
    parser.add_argument("-o", "--out", help="Name of output file")
    parser.add_argument("-v", "--verbose", help="Give more information about what is going on", action="store_true")
    arguments = vars(parser.parse_args())
    src, verbose = arguments["src"], arguments["verbose"]
    out = arguments["out"]+"_" if arguments["out"] else "out_"
    # setup constants
    waitkeys = {j:i for i,j in enumerate(range(48,58))}
    roi_size = 64
    feature_vector_size = roi_size ** 2
    # initialize numpy vector
    feature_vectors = np.empty((0,feature_vector_size))
    responses = []
    for image_file in os.listdir(src):
        if image_file.endswith(".png"):
            image = cv2.imread( os.path.join(src, image_file) )
            cv2.imshow("Showing file " + image_file, image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret,thresh = cv2.threshold(image,1,255,0)
            thresh = cv2.bitwise_not(thresh)
            wk = cv2.waitKey()
            if wk is 27:
                exit(0)
            elif wk in waitkeys:
                responses.append(int(chr(wk)))
                if verbose:
                    print "This number is registered as :", chr(wk)
                # for a lack of better set of features, we take all pixels
                feature_vector = thresh.reshape((1,feature_vector_size))
                feature_vectors = np.append(feature_vectors, feature_vector, 0)
            cv2.destroyAllWindows()
    responses = np.array(responses, np.float32)
    responses = responses.reshape((responses.size, 1))
    np.savetxt(os.path.join(src,out+"x.data"), feature_vectors)
    np.savetxt(os.path.join(src,out+"y.data"), responses)
                
            
