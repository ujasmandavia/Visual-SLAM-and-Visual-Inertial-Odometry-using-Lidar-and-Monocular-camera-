% L-K Optical Flow
%% Load images
bag_obj = rosbag('rosbag_image_included.bag');
%Extract images
img_all = readMessages(select(bag_obj, 'Topic', '/raspicam/image_raw'));

img_count = size(img_all, 1);
img = cell(img_count,1);
img_time = zeros(img_count, 1);
img_time_init = img_all{1}.Header.Stamp.Sec;

for i = 1:img_count
    img{i} = readImage(img_all{i});
    img_time(i) = img_all{i}.Header.Stamp.Sec - img_time_init + img_all{i}.Header.Stamp.Nsec * 1e-9;
end
%% Select images for computation
prev = 5;
curr = 10;
img1 = im2double(rgb2gray(img{prev}));
img2 = im2double(rgb2gray(img{curr}));
[height,width]=size(img1);
%% Derivative of Gaussian filter
%stdev: 1.4; size: 5x5
img1_temp=imgaussfilt(img1,3,'FilterSize',7);
img2_temp=imgaussfilt(img2,3,'FilterSize',7);
fx=[-1,0,1];
fy=[-1;0;1];
I2x=filter2(fx,img2_temp);
I2y=filter2(fy,img2_temp);
%% Compute the temporal gradient
It=img2_temp-img1_temp;
%% Solve the equations
%Region size: 5
I2x2=I2x.^2;
I2y2=I2y.^2;
I2xy=I2x.*I2y;
I2xt=I2x.*It;
I2yt=I2y.*It;
h=fspecial('average',5);
S2x2=filter2(h,I2x2);
S2y2=filter2(h,I2y2);
S2xy=filter2(h,I2xy);
S2xt=filter2(h,I2xt);
S2yt=filter2(h,I2yt);
vx=zeros(height,width,'double');
vy=zeros(height,width,'double');
for i=4:height-3
    for j=4:width-3
        M2=[S2x2(i,j) S2xy(i,j);S2xy(i,j) S2y2(i,j)];
        b2=-[S2xt(i,j);S2yt(i,j)];
        %Set threshold
        if det(M2)>1e-7
            temp=M2\b2;
            vx(i,j)=temp(1);
            vy(i,j)=temp(2);
        end
    end
end
%% Display the optical flow
imshow(img{curr});
hold on;
quiver(vx,vy,'AutoScaleFactor',5);
%% Display the 2 images at the same time
points1 = detectSURFFeatures(img1);
points2 = detectSURFFeatures(img2);

[f1, vpts1] = extractFeatures(img1, points1);
[f2, vpts2] = extractFeatures(img2, points2);

indexPairs = matchFeatures(f1, f2) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));

figure; showMatchedFeatures(img1, img2, matchedPoints1, matchedPoints2);
legend('matched points 1','matched points 2');