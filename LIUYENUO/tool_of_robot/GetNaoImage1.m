function [outputArg1,outputArg2] = GetNaoImage(inputArg1,inputArg2)
% #***********************************************
% 
% #@function name：   GetNaoImage(IP,PORT,cameraID)
% #@parameters：     skip
% #@return value：   none
% #@function： The robot's built-in camera control module is called up to capture and hold the current scene.
% #           The robot's forehead camera cannot see the ball as it is approximately less than 0.6m away from the robot，
% #           so we need to switch the camera，cameraID=0，choose the upper one；
% #           cameraID=1，choose the lower one
def GetNaoImage(IP,PORT,cameraID):
    camProxy=ALProxy("ALVideoDevice",IP,PORT);
    resolition =2 ;    %VGA格式640*480
    colorSpace = 11; %RGB

    %select and turn on the camera
    camProxy.setParam(vision_definitions.kCameraSelectID,cameraID);
    videoClient = camProxy.subscribe("python_client",resolition,colorSpace,5);

    %acquire the picture of the camera
    %image [6] Contains image data passed as an array of ASCII characters.
    naoImage = camProxy.getImageRemote(videoClient);

    camProxy.unsubscribe(videoClient);
    %Get image size and pixel array。
    imageWidth=naoImage[0];
    imageHeight=naoImage[1];
    array=naoImage[6];
    %Create a PIL image from our pixel array.
    im = Image.fromstring("RGB",(imageWidth,imageHeight),array);
    %save the image.
    im.save("temp.jpg","JPEG")
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

