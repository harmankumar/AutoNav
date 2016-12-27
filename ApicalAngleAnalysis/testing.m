image1 = '';
image2 = '';


fx = 1039.8;
fy = 1037.6;
u = 642.9;
v = 351.5;


%{
img1 = imread(image1);
img2 = imread(image2);
%}

imgstr1 = 'image000309';
imgstr2 = 'image000319';
extension = '.jpg';
[points1, boardSize1,imagesUsed1] = detectCheckerboardPoints('image000309.jpg');
[points2, boardSize2,imagesUsed2] = detectCheckerboardPoints('image000319.jpg');

%{points1 = load('FlyCapture2Test-15322706-0.pgm.txt');
%{points2 = load('FlyCapture2Test-15322706-10.pgm.txt');
img1 = imread(strcat(imgstr1,extension));
img2 = imread(strcat(imgstr2,extension));
figure; ax = axes;
showMatchedFeatures(img1,img2,points1,points2,'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');

points1(:,1) = (points1(:,1) - u)/fx;
points2(:,1) = (points2(:,1) - u)/fx;

points1(:,2) = (points1(:,2) - v)/fy;
points2(:,2) = (points2(:,2) - v)/fy;


for i=1:size(points1,1)
    A(i,1) = points2(i,1)*points1(i,1);
    A(i,2) = points2(i,1)*points1(i,2);
    A(i,3) = points2(i,1);
    A(i,4) = points2(i,2)*points1(i,1);
    A(i,5) = points2(i,2)*points1(i,2);
    A(i,6) = points2(i,2);
    A(i,7) = points1(i,1);
    A(i,8) = points1(i,2);
    A(i,9) = 1;
end

[u, s, v] = svd(A);
e = v(:,end);
E = [e(1) e(2) e(3); e(4) e(5) e(6); e(7) e(8) e(9)];

[a, b, c] = svd(E);
b(3, 3) = 0;
E = a*b*c';     %{ Low rank Approx. to E

X = eye(3, 4);
f = [0 -1 0 ;1 0 0; 0 0 1];

u3 = a(:,end);

P1 = [a*f*c' u3];
P2 = [a*f*c' -u3];
P3 = [a*f'*c' u3];
P4 = [a*f'*c' -u3];

bucket = [0 0 0 0];

for i=1:size(points1,1)

    A1 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P1(3,:) - P1(1,:) ; points2(i,2)*P1(3,:) - P1(2,:)]; 
    [u1, s1, v1] = svd(A1);
    wc1 = v1(:, end); 
    wc1 = wc1/wc1(4);
    %{wc1dash = P1*wc1;%{[P1(:,1:3)' -P1(:,1:3)'*P1(:,4)]*wc1;
    wc1dash = [P1(:,1:3)' -P1(:,1:3)'*P1(:,4)]*wc1;
    pointcoord = wc1(1:3);  %{ Coord of the point
    wc1arr(i,:) = pointcoord;
    
    A2 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P2(3,:) - P2(1,:) ; points2(i,2)*P2(3,:) - P2(2,:)]; 
    [u2, s2, v2] = svd(A2);
    wc2 = v2(:, end); 
    wc2 = wc2/wc2(4);
    %{wc2dash = P2*wc2;%{[P2(:,1:3)' -P2(:,1:3)'*P2(:,4)]*wc2;
    wc2dash = [P2(:,1:3)' -P2(:,1:3)'*P2(:,4)]*wc2;
    pointcoord = wc2(1:3);  %{ Coord of the point
    wc2arr(i,:) = pointcoord;
    
    A3 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P3(3,:) - P3(1,:) ; points2(i,2)*P3(3,:) - P3(2,:)]; 
    [u3, s3, v3] = svd(A3);
    wc3 = v3(:, end);
    wc3 = wc3/wc3(4);
    %{wc3dash = P3*wc3;%{[P3(:,1:3)' -P3(:,1:3)'*P3(:,4)]*wc3;
    wc3dash = [P3(:,1:3)' -P3(:,1:3)'*P3(:,4)]*wc3;
    pointcoord = wc3(1:3);  %{ Coord of the point
    wc3arr(i,:) = pointcoord;
    
    A4 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P4(3,:) - P4(1,:) ; points2(i,2)*P4(3,:) - P4(2,:)];
    [u4, s4, v4] = svd(A4);
    wc4 = v4(:, end);
    wc4 = wc4/wc4(4);
    %{wc4dash = P4*wc4;%{[P4(:,1:3)' -P4(:,1:3)'*P4(:,4)]*wc4;
    wc4dash = [P4(:,1:3)' -P4(:,1:3)'*P4(:,4)]*wc4;
    pointcoord = wc4(1:3);  %{ Coord of the point
    wc4arr(i,:) = pointcoord;
    
    if(wc1(3) > 0 && wc1dash(3) > 0)
        bucket(1) = bucket(1)+1;
    end
    if(wc2(3) > 0 && wc2dash(3) > 0)
        bucket(2) = bucket(2)+1;
    end
    if(wc3(3) > 0 && wc3dash(3) > 0)
        bucket(3) = bucket(3)+1;
    end
    if(wc4(3) > 0 && wc4dash(3) > 0)
        bucket(4) = bucket(4)+1;
    end    
end


%{ Epipolar lines, baseline, epical angle.
    
%{img1 = imread(image1);
%{img2 = imread(image2);

maxi = bucket(1);
maxind = 1;
bestcoord = wc1;
bestcoorddash = wc1dash;

if(bucket(2) > maxi)
    maxi = bucket(2);
    maxind = 2;
    bestcoord = wc2;
    bestcoorddash = wc2dash;      
end
if(bucket(3) > maxi)
    maxi = bucket(3);
    maxind = 3;
    bestcoord = wc3;
    bestcoorddash = wc3dash;    
end
if(bucket(4) > maxi)
    maxi = bucket(4);
    maxind = 4;
    bestcoord = wc4;
    bestcoorddash = wc4dash;    
end




for i=1:size(points1,1)
    
    if(maxind == 1)
        A1 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P1(3,:) - P1(1,:) ; points2(i,2)*P1(3,:) - P1(2,:)]; 
        [u1, s1, v1] = svd(A1);
        wc1 = v1(:, end); 
        wc1 = wc1/wc1(4);
        wc1dash = [P1(:,1:3)' -P1(:,1:3)'*P1(:,4)]*wc1;
        %{wc1dash = P1*wc1;%{[P1(:,1:3)' -P1(:,1:3)'*P1(:,4)]*wc1;
        pointcoord = wc1(1:3);  %{ Coord of the point
        pointarr(i,:) = pointcoord;
        orig = [0; 0; 0] - pointcoord;
        bestcoord = P1(:, end) - pointcoord;
        angle = acos(dot(orig, bestcoord)/(norm(bestcoord)*norm(orig)))*(180/pi);
        finmat(i) = angle;
        
    end
    
    if(maxind == 2)
        A2 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P2(3,:) - P2(1,:) ; points2(i,2)*P2(3,:) - P2(2,:)];
        [u2, s2, v2] = svd(A2);
        wc2 = v2(:, end); 
        wc2 = wc2/wc2(4);
        wc2dash = [P2(:,1:3)' -P2(:,1:3)'*P2(:,4)]*wc2;        
        %{wc2dash = P2*wc2;%{[P2(:,1:3)' -P2(:,1:3)'*P2(:,4)]*wc2;        
        pointcoord = wc2(1:3);
        pointarr(i,:) = pointcoord;
        orig = [0; 0; 0] - pointcoord;
        bestcoord = P2(:, end) - pointcoord;
        angle = acos(dot(orig, bestcoord)/(norm(bestcoord)*norm(orig)))*(180/pi);
        finmat(i) = angle;
    end
    if(maxind == 3)
        A3 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P3(3,:) - P3(1,:) ; points2(i,2)*P3(3,:) - P3(2,:)]; 
        [u3, s3, v3] = svd(A3);
        wc3 = v3(:, end);
        wc3 = wc3/wc3(4);
        %{wc3dash = P3*wc3;%{[P3(:,1:3)' -P3(:,1:3)'*P3(:,4)]*wc3;
        wc3dash = [P3(:,1:3)' -P3(:,1:3)'*P3(:,4)]*wc3;
        pointcoord = wc3(1:3);
        pointarr(i) = wc3;
        pointarr(i,:) = pointcoord;
        orig = [0; 0; 0] - pointcoord;
        bestcoord = P3(:, end) - pointcoord;
        angle = acos(dot(orig, bestcoord)/(norm(bestcoord)*norm(orig)))*(180/pi);
        finmat(i) = angle;   
    end
    if(maxind == 4)
        A4 = [points1(i,1)*X(3,:) - X(1,:); points1(i,2)*X(3,:) - X(2,:) ; points2(i,1)*P4(3,:) - P4(1,:) ; points2(i,2)*P4(3,:) - P4(2,:)];
        [u4, s4, v4] = svd(A4);
        wc4 = v4(:, end);
        wc4 = wc4/wc4(4);
        wc4dash = [P4(:,1:3)' -P4(:,1:3)'*P4(:,4)]*wc4;
        %{wc4dash = P4*wc4;%{[P4(:,1:3)' -P4(:,1:3)'*P4(:,4)]*wc4;
        pointcoord = wc4(1:3);
        pointarr(i,:) = pointcoord;
        orig = [0; 0; 0] - pointcoord;
        bestcoord = P4(:, end) - pointcoord;
        angle = acos(dot(orig, bestcoord)/(norm(bestcoord)*norm(orig)))*(180/pi);
        finmat(i) = angle;    
    end
end


img1 = imread(strcat(imgstr1,extension));
img2 = imread(strcat(imgstr2,extension));


fx = 1232.96698291042;
fy = 1252.81548168609;
u = 622.729667462387;
v = 516.690238598557;

number = 5;
pointsX = (rand(1,number,1)*0.3);
pointsY = (rand(1,number,1)*0.3);

for i=1:number
    eqnLine = E*([pointsX(i) pointsY(i) 1.0]');
    %{disp(eqnLine);
    
    shapeInserter1 = vision.ShapeInserter('Shape','Circles','BorderColor','White','Fill',true,'FillColor','White');
    shapeInserter2 = vision.ShapeInserter('Shape','Lines','BorderColor','White');
    
    x1 = -1.0;
    x2 = 1.0;
    syms y1
    eqn = [x1 y1 1.0]*eqnLine == 0;
    soly1 = solve(eqn, y1);
    
    syms y2
    eqn = [x2 y2 1.0]*eqnLine == 0;
    soly2 = solve(eqn, y2);
    

    lines = int32(double([fx*x1+u,fy*soly1+v,fx*x2+u,fy*soly2+v]));
    
    J2 = step(shapeInserter2, img2, lines);
    %{figure;
    %{imshow(J2);
    
    circles = int32([fx*pointsX(i)+u,fy*pointsY(i)+v,10]);
    %{disp(circles);
    J1 = step(shapeInserter1, img1, circles);
    %{figure;
    %{imshow(J1);
    
    imwrite(J1,strcat(imgstr1,int2str(i),'_.jpg'));
    imwrite(J2,strcat(imgstr2,int2str(i),'_.jpg'));

end

figure;
axis equal;
scatter3(pointarr(:,1),pointarr(:,2),pointarr(:,3),'r');
hold on;

XYZ = pointarr;
cm = mean(XYZ,1);
XYZ0 = bsxfun(@minus,XYZ,cm);
[U,S,V] = svd(XYZ0,0);
diag(S)
P = V(:,3);
myz = ( (dot(P,cm) - P(1))*pointarr(1,:) - P(2)*pointarr(2,:))/P(3);

for i = 1:size(pointarr,1)
    zz = myz(1)*pointarr(i, 1) + myz(2)*pointarr(i, 2) + myz(3);
    pointarr(i,3)=zz;
end

%{ scatter3(pointarr(:,1),pointarr(:,2),pointarr(:,3),'g');

%{  Plot the angles.
%{  F estimation is correct, point lies on the epipolar line.
%{  Relate translation between cameras
%{  Angles are small

