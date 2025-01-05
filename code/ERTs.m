clc;
close all;
clear all;

load('simexp_1124.mat');
figure(1);
imshow(m2);
hold on;

% for i = 1:size(sa1,1)-1
%     for j = i:size(sa1,1)
%         d = sqrt((sa1(i,1)-sa1(j,1))^2+(sa1(i,2)-sa1(j,2))^2);
%         if d<60
%             result = edgecollision(sa1(i,1:2),sa1(j,1:2),m2);
%             if result == 0
%                 plot([sa1(i,1),sa1(j,1)],[sa1(i,2),sa1(j,2)],'color',[128/255, 128/255, 128/255]);
%             end
%         end
%     end
% end
% 
% for i = 1:size(sa2,1)-1
%     for j = i:size(sa2,1)
%         d = sqrt((sa2(i,1)-sa2(j,1))^2+(sa2(i,2)-sa2(j,2))^2);
%         if d<60
%             result = edgecollision(sa2(i,1:2),sa2(j,1:2),m2);
%             if result == 0
%                 plot([sa2(i,1),sa2(j,1)],[sa2(i,2),sa2(j,2)],'color',[128/255, 128/255, 128/255]);
%             end
%         end
%     end
% end

plot(s1(1,1),s1(1,2),'. b', 'markersize',100);
plot(g1(1,1),g1(1,2),'o b', 'markersize',25);

plot(s2(1,1),s2(1,2),'. r', 'markersize',100);
plot(g2(1,1),g2(1,2),'o r', 'markersize',25);

plot(s3(1,1),s3(1,2),'. m', 'markersize',100);
plot(g3(1,1),g3(1,2),'o m', 'markersize',25);

p1 = [s1;
    317.5,111.7;
    353.1, 206.8;
    386, 333;
    481.4, 415.8;
    532.6, 589;
    607.1, 643.6;
    g1];
p2 = [s2;
    195.9, 583.8;
    315.6, 588.9;
    406.9, 586.6;
    532.6, 589;
    643.7, 586;
    771.3, 587.8;
    818.3, 584;
    g2];
p3 = [s3;
    384.4,904.2;
    244,888.4;
    185.8,888.7;
    93.8, 886;
    g3];
for i = 1:size(p1,1)-1
    plot([p1(i,1),p1(i+1,1)],[p1(i,2),p1(i+1,2)],'b','linewidth',1);
end

for i = 1:size(p2,1)-1
    plot([p2(i,1),p2(i+1,1)],[p2(i,2),p2(i+1,2)],'r','linewidth',1);
end

for i = 1:size(p3,1)-1
    plot([p3(i,1),p3(i+1,1)],[p3(i,2),p3(i+1,2)],'m','linewidth',1);
end

pa1 = [s1];
speed = 10;
x = s1(1,1);
y = s1(1,2);
m = 0;
d = 0;
for i = 2:size(p1,1)
    thetax = (p1(i,1)-p1(i-1,1))/(sqrt((p1(i-1,1)-p1(i,1))^2+(p1(i-1,2)-p1(i,2))^2));
    thetay = (p1(i,2)-p1(i-1,2))/(sqrt((p1(i-1,1)-p1(i,1))^2+(p1(i-1,2)-p1(i,2))^2));
    if d>0
        m = speed - d;
        x = p1(i-1,1)+m*thetax;
        y = p1(i-1,2)+m*thetay;
        pa1 = [pa1;x, y];
    end
    d = sqrt((p1(i-1,1)-p1(i,1))^2+(p1(i-1,2)-p1(i,2))^2)-m;
    k = floor(d/speed);
    for j = 1:k
        x = p1(i-1,1)+j*speed*thetax;
        y = p1(i-1,2)+j*speed*thetay;
        pa1 = [pa1;x, y];
    end
    d = d-k*speed;
end
pa1 = [pa1;g1];

%%
pa2 = [s2];
x = s2(1,1);
y = s2(1,2);
m = 0;
d = 0;
for i = 2:size(p2,1)
    thetax = (p2(i,1)-p2(i-1,1))/(sqrt((p2(i-1,1)-p2(i,1))^2+(p2(i-1,2)-p2(i,2))^2));
    thetay = (p2(i,2)-p2(i-1,2))/(sqrt((p2(i-1,1)-p2(i,1))^2+(p2(i-1,2)-p2(i,2))^2));
    if d>0
        m = speed - d;
        x = p2(i-1,1)+m*thetax;
        y = p2(i-1,2)+m*thetay;
        pa2 = [pa2;x, y];
    end
    d = sqrt((p2(i-1,1)-p2(i,1))^2+(p2(i-1,2)-p2(i,2))^2)-m;
    k = floor(d/speed);
    for j = 1:k
        x = p2(i-1,1)+j*speed*thetax;
        y = p2(i-1,2)+j*speed*thetay;
        pa2 = [pa2;x, y];
    end
    d = d-k*speed;
end
pa2 = [pa2;g2];

%%
pa3 = [s3];
x = s3(1,1);
y = s3(1,2);
m = 0;
d = 0;
for i = 2:size(p3,1)
    thetax = (p3(i,1)-p3(i-1,1))/(sqrt((p3(i-1,1)-p3(i,1))^2+(p3(i-1,2)-p3(i,2))^2));
    thetay = (p3(i,2)-p3(i-1,2))/(sqrt((p3(i-1,1)-p3(i,1))^2+(p3(i-1,2)-p3(i,2))^2));
    if d>0
        m = speed - d;
        x = p3(i-1,1)+m*thetax;
        y = p3(i-1,2)+m*thetay;
        pa3 = [pa3;x, y];
    end
    d = sqrt((p3(i-1,1)-p3(i,1))^2+(p3(i-1,2)-p3(i,2))^2)-m;
    k = floor(d/speed);
    for j = 1:k
        x = p3(i-1,1)+j*speed*thetax;
        y = p3(i-1,2)+j*speed*thetay;
        pa3 = [pa3;x, y];
    end
    d = d-k*speed;
end
pa3 = [pa3;g3];
%%
% 

robot1 = scatter(s1(1,1), s1(1,2), 50, 'b', 'filled');
robot2 = scatter(s2(1,1), s2(1,2), 50, 'r', 'filled');
robot3 = scatter(s3(1,1), s3(1,2), 50, 'm', 'filled');
v = VideoWriter('circle_animation.mp4', 'MPEG-4');
open(v);

for i = 1:100
    if i <= size(pa1,1)
        robot1.XData = pa1(i,1);
        robot1.YData = pa1(i,2);
    end
    if i <= size(pa2,1)
        robot2.XData = pa2(i,1);
        robot2.YData = pa2(i,2);
    end
    if i <= size(pa3,1)
        robot3.XData = pa3(i,1);
        robot3.YData = pa3(i,2);
    end
    frame = getframe(gcf);
    writeVideo(v, frame);
    pause(0.1);
    
end

close(v);
hold off;
%%
function result = edgecollision(p1, p2, map)
    result = 0;
    n = 30;
    a = (p2(1,1)-p1(1,1))/n;
    b = (p2(1,2)-p1(1,2))/n;
    for i = 1:n
        c = p1(1,1)+i*a;
        d = p1(1,2)+i*b;
        if map(ceil(d), ceil(c)) ~= 255 || map(floor(d), ceil(c)) ~= 255 || map(ceil(d), floor(c)) ~= 255 || map(floor(d), floor(c)) ~= 255
            result = 1;
            break;
        end
    end
end