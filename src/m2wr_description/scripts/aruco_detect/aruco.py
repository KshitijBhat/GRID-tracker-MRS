import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

def get_swathe(img,draw=True):
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    aruco_param = aruco.DetectorParameters_create()
    bounds,idx, rej = aruco.detectMarkers(gray,arucoDict,parameters=aruco_param)
    
    if draw:
        aruco.drawDetectedMarkers(img,bounds) 

    try:
        return np.array(bounds)[0][0],True
    except Exception:
        return None, False           

def main():
    address = "http://192.168.1.103:4747/mjpegfeed?640x480" 
    #### uncomment the following if you are using a webcam
    #address = 0
    cam = cv.VideoCapture(address)
    traj = []

    while True:
        bo,image = cam.read()  
        #image = image[::-1]  
        swathe, is_got = get_swathe(image,True)
        cv.imshow('Camera', cv.flip(image,1)) 
        
        if is_got:
            x,y = (swathe[0][0] + swathe[2][0])/2 , (swathe[0][1] + swathe[2][1])/-2
            xe,ye = (swathe[3][0] + swathe[2][0])/2 , (swathe[3][1] + swathe[2][1])/-2
            
            thetap = np.pi + np.arctan((swathe[1][1]-swathe[0][1])/(swathe[1][0]-swathe[0][0] + 1e-6))
            thetan = 3*np.pi/2 + np.arctan((swathe[1][1]-swathe[0][1])/(swathe[1][0]-swathe[0][0] + 1e-6))
            theta = thetap*(y>=ye) + thetan*(y<ye)
            pose = x,y,theta
            traj.append([*pose])    
        if cv.waitKey(1) & 0xFF == ord('x'):
            cv.destroyAllWindows()
            break

    [xo,yo,t0] = traj[0]
    [xf,yf,tf] = traj[-1]
    plt.figure()
    xs,ys,ts = zip(*traj)
    plt.plot(xs-xo,ys-yo,label='Tracked')
    plt.quiver(xo-xo,yo-yo, np.cos(t0), np.sin(t0),scale=12,color='g')
    plt.quiver(xf-xo,yf-yo, np.cos(tf), np.sin(tf),scale=12,color='r')

    plt.title('Tracked path')
    plt.legend()
    plt.grid() 
    plt.show()

if __name__ == '__main__':
    main()    
