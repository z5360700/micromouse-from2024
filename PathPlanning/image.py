# import cv2
# cam = cv2.VideoCapture(1)
# result, image = cam.read()
# cv2.imshow('maze', image) 
# cv2.waitKey(0)


import cv2 
  
  
# define a video capture object 
vid = cv2.VideoCapture(1) 
  
while(True): 
      
    # Capture the video frame 
    # by frame 
    ret, frame = vid.read() 
  
    # Display the resulting frame 
    cv2.imshow('frame', frame) 
      
    # the 'q' button is set as the 
    # quitting button you may use any 
    # desired button of your choice 
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
  
# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv2.destroyAllWindows() 