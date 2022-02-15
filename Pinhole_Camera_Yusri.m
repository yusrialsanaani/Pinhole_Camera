close all; 
%clear all; clc;
%% ------------------------------------------------------------------------
%% Virtual plane parameters
% X = Y = -1000:1000, taking 1 step, the total points = 2000 in both X & Y
x_total = 2000;
x_min = -x_total/2;
x_max = x_total/2;
%% ------------------------------------------------------------------------
%% Image plan parameters
image_width = 12.7; image_height = 12.7;
raw_pixel = 480; column_pixel = 640;
% Resolution = physical size / pixel dimensions
resolution_raw = image_width/raw_pixel;
resolution_column = image_height/column_pixel;

%% ------------------------------------------------------------------------
%% Homogeneous Coordinates
% If we consider the coordinates of the image point,(x',y',z'), as an 
% homogeneous coordinate vector [x';y';z';w']
% The coordinates of the object surface point,(x,y,z), can also be written as an
% homogeneous coordinate vector, [x;y;z;w] , where the weight (w) = 1
% as this point is in real 3D space (full sccale).
% We can represent the perspective projection as follows:
% [x'; y'; z'; w'] = P*[x; y; z; w]
% Perspective Projection Matrix: P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 1/f 0]
% [x';y';z';w'] = [x; y; z; z/f]
% The projection operation introduces some distortion in the world
% representation that appears as a scaling factor.
% perspective projection equations:
% [x';y';z'] = [x/w'; y/w'; z/w'] = [f*x/z; f*y/z; f']
%% ------------------------------------------------------------------------
%%                            Part 1
%%                    Create The Virtual Plane
tic
xyz_plane = zeros(x_total,x_total,6); %(x,y,z,H,S,V)
i = 1; j = 1;
for x = x_min:(x_max-1)
    for y = x_min:(x_max-1)
        xyz_plane(i,j,1) = x;
        xyz_plane(i,j,2) = y;
        xyz_plane(i,j,3) = 100*sin(x/150);
        xyz_plane(i,j,4) = 0.5*((x/1000)+1);
        xyz_plane(i,j,5) = 1;
        xyz_plane(i,j,6) = 0.5*((-y/1000)+1);
        j = j+1;
    end
    i = i+1; j=1;
end
% Extract xyz points and hsv color space points:
xyzPoints = xyz_plane(:,:,1:3); % Extract the xyz points of the virtual plane.
HSV = xyz_plane(:,:,4:6);       % Extract the hsv color space
RGB = hsv2rgb(HSV);             % Convert HSV into RGB color space

% Plot the virtual plane using HSV color space
ptCloud_HSV = pointCloud(xyzPoints, 'Color', HSV);
figure(1),pcshow(ptCloud_HSV), title('Virtual Plane in HSV'), xlabel('X'), ylabel('Y'),zlabel('Z');
set(gca,'linewidth', 1.5,'fontsize',16,'fontname','Times New Roman')
%grid of

% Plot the virtual plane using RGB color space
ptCloud_RGB = pointCloud(xyzPoints, 'Color', RGB);
figure(2),pcshow(ptCloud_RGB), title('Virtual Plane in RGB'), xlabel('X'), ylabel('Y'),zlabel('Z');
set(gca,'linewidth', 1.5,'fontsize',14,'fontname','Times New Roman','Color','none')
%% ------------------------------------------------------------------------
%%                                  Part 2
%%                            Create Virtual Image
% This part uses two functions to create the virtual images as follows:
% - P(f) is a function defined below to find the P matrix for different f values.
% - get_image(x_total,xyzPoints,P,Q,f,color_space) is a function to obtain
%   the virtual image by:
%      -> Implementing Non-Inverting Perspective Pojection Model
%      -> Obtaining the Perspective Projection Equations
%      -> Calculating Pixel Coordinates
%      -> Mapping the color patterns to the equivalant coordinations of the image plane
% - These functions are built in the end of this Script to allow us to
%   reproduce images at different parameters without repeating the same code.

% Focal lenths:
f5 = 5; f3 = 3; f7 = 7;
% Perspective Projection Matrix: 
P5 = PPM(f5); P3 = PPM(f3); P7 = PPM(f7);
% The homogeneous transformation:
Q_Robj1_Rcam1 = [1 0 0 0; 0 1 0 0; 0 0 1 1000; 0 0 0 1];

% Create Images at different focal lengths:
IM_Rcam1_RGB_f5 = get_image(x_total,xyzPoints,P5,Q_Robj1_Rcam1,f5,RGB);
IM_Rcam1_RGB_f3 = get_image(x_total,xyzPoints,P3,Q_Robj1_Rcam1,f3,RGB);
IM_Rcam1_RGB_f7 = get_image(x_total,xyzPoints,P7,Q_Robj1_Rcam1,f7,RGB);

% Create image with focal length = 5 and color space of HSV:
IM_Rcam1_HSV_f5 = get_image(x_total,xyzPoints,P5,Q_Robj1_Rcam1,f5,HSV);

% Visualize and save the output images
figure(3),imshow(IM_Rcam1_RGB_f5); title('IM\_Rcam1\_RGB\_f5');
imwrite(IM_Rcam1_RGB_f5,'IM_Rcam1_RGB_f5.jpg')

figure(4),imshow(IM_Rcam1_RGB_f3); title('IM\_Rcam1\_RGB\_f3');
imwrite(IM_Rcam1_RGB_f3,'IM_Rcam1_RGB_f3.jpg')

figure(5),imshow(IM_Rcam1_RGB_f7); title('IM\_Rcam1\_RGB\_f7');
imwrite(IM_Rcam1_RGB_f7,'IM_Rcam1_RGB_f7.jpg')

figure(6),imshow(IM_Rcam1_HSV_f5); title('IM\_Rcam1\_HSV\_f5');
imwrite(IM_Rcam1_HSV_f5,'IM_Rcam1_HSV_f5.jpg')

%% ------------------------------------------------------------------------
%%                                  Part 3
% This part uses the same functions as in part 1 but with different 
% homogeneous transformations.
P5 = PPM(f5); % Perspective Projection Matrix with f = 5.
% The homogeneous Transformations:
Q_Robj1_Rcam1 = [1 0 0 0; 0 1 0 0; 0 0 1 1000; 0 0 0 1];
Q_Rcam2_Rcam1 = [0.866 0 -0.5 400; 0 1 0 0; 0.5 0 0.866 -600; 0 0 0 1];
Q_Rcam3_Rcam1 = [0.7071 0 0.7071 -1200; 0 1 0 0; -0.7071 0 0.7071 0; 0 0 0 1];
% Q2 = inv(Q_Rcam2_Rcam1) * Q_Robj1_Rcam1;
% Q3 = inv(Q_Rcam3_Rcam1) * Q_Robj1_Rcam1;
Q2 = Q_Rcam2_Rcam1\Q_Robj1_Rcam1;
Q3 = Q_Rcam3_Rcam1\Q_Robj1_Rcam1;

% Create Image with Q2 transformation and focal length (f) = 5:
IM_Rcam2_RGB_f5 = get_image(x_total,xyzPoints,P5,Q2,f5,RGB);

% Create Image with Q3 transformation and focal length (f) = 5:
IM_Rcam3_RGB_f5 = get_image(x_total,xyzPoints,P5,Q3,f5,RGB);

% Visualize and save the output images
figure(7),imshow(IM_Rcam2_RGB_f5); title('IM\_Rcam2\_RGB\_f5');
imwrite(IM_Rcam2_RGB_f5,'IM_Rcam2_RGB_f5.jpg');

figure(8),imshow(IM_Rcam3_RGB_f5); title('IM\_Rcam3\_RGB\_f5');
imwrite(IM_Rcam3_RGB_f5,'IM_Rcam3_RGB_f5.jpg')
toc
%%
figure(9)
subplot(2,3,1);imshow(IM_Rcam1_RGB_f5); title('IM\_Rcam1\_RGB\_f5');
subplot(2,3,2);imshow(IM_Rcam1_RGB_f3); title('IM\_Rcam1\_RGB\_f3');
subplot(2,3,3);imshow(IM_Rcam1_RGB_f7); title('IM\_Rcam1\_RGB\_f7');
subplot(2,3,4);imshow(IM_Rcam1_HSV_f5); title('IM\_Rcam1\_HSV\_f5');
subplot(2,3,5); imshow(IM_Rcam2_RGB_f5); title('IM\_Rcam2\_RGB\_f5');
subplot(2,3,6); imshow(IM_Rcam3_RGB_f5); title('IM\_Rcam3\_RGB\_f5');
%%
figure(10)
subplot(1,2,1); pcshow(ptCloud_HSV), title('Virtual Plane in HSV'), xlabel('X'), ylabel('Y'),zlabel('Z');
subplot(1,2,2); pcshow(ptCloud_RGB), title('Virtual Plane in RGB'), xlabel('X'), ylabel('Y'),zlabel('Z');
%% ------------------------------------------------------------------------
%%                              Functions

%% __________________ Perspective Projection Matrix (PPM) _________________
% Function to calculate Perspective Projection Matrix (PPM) at given focal length (f)
function P = PPM(f)
P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 1/f 0];
end
%%
%% ___________________ Function To Obtain The Virtual Image _______________

function image = get_image(x_total,xyzPoints,P,Q,f,color_space)
%%             Implementing Non-Inverting Perspective Pojection Model
% Project points from the undulated surface (virtual plane) onto the image plane of the virtual camera.
% [x'; y'; z'; w'] = P*[x; y; z; w]
% Perspective Projection Matrix: P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 1/f 0]
%% Image plan parameters
image_width = 12.7; image_height = 12.7;
raw_pixel = 480; column_pixel = 640;
% Resolution = physical size / pixel dimensions
resolution_raw = image_width/raw_pixel;
resolution_column = image_height/column_pixel;
%%
w = ones(x_total, x_total);   % w = 1 as this point is in real 3D space (full sccale).
xyzw = cat(3, xyzPoints, w);
xyz_camera = zeros(x_total,x_total,4);
for i = 1:x_total
    for j = 1:x_total
        xyz_camera(i,j,:) = P * Q * squeeze(xyzw(i,j,:));
    end
end
%%                          Perspective Projection Equations
%                              [x';y';z';w'] = [x; y; z; z/f]
% Retrieve the perspective projection equations by deviding x',y',z' by the scaling factor (w'):
%                        [x';y';z'] = [x/w'; y/w'; z/w'] = [f*x/z; f*y/z; f']
xyz_camera(:,:,1:2) = xyz_camera(:,:,1:2)./xyz_camera(:,:,4);
xyz_camera(:,:,3) = f;
%%                           Pixel Coordinates Calculation
% Calculate the pixel coordinates of every projection over the pixel map by considering the image
% plane physical dimension, the number of pixels per row and colomn in the
% output image.
xyz_camera(:,:,1) = round(xyz_camera(:,:,1)./resolution_column)+(column_pixel/2);
xyz_camera(:,:,2) = round(xyz_camera(:,:,2)./resolution_raw)+(raw_pixel/2);

%% Mapping the color patterns to the equivalant coordinations of the image plane
% Propagate the color patterns along with the (X,Y,Z) coordinates of every sample point.
% Initializing the background of the output image map with a shading of medium gray
image = 0.5*ones(column_pixel,raw_pixel,3);
% Retrieve the transformed coordination in image plane range: 
% - Columns: between 0 and 641
% - Raws: between 0 and 480

for i = 1:x_total
    for j = 1:x_total
        if (xyz_camera(i,j,1)>0) && (xyz_camera(i,j,2)>0) &&...
                (xyz_camera(i,j,1) <= column_pixel) && (xyz_camera(i,j,2) <= raw_pixel)
            image(xyz_camera(i,j,1), xyz_camera(i,j,2),:) = color_space(i,j,:);
        end
    end
end
image = permute(image,[2 1 3]);
end
