function [outputArg1,outputArg2] = DistAndDirect_cal(cxnum,rynum,colsum,rowsum,Head_angle,cameraID)
    distx=-(cxnum-colsum/2);
    disty=rynum-rowsu/2;
    disp(distx,disty);
    Picture_angle = disty*47.64/480;
    if cameraID ==0
        h = 0.62;
        Camera_angle = 12;
    else
        h = 0.57;
        Camera_angle = 38;
    end
    Total_angle = pi*(Picture_angle+Camera_angle)/180+Head_angle(0);
    d1 = h/math.tan(Total_angle);
    d2 = d1/cos(alpha);
end

