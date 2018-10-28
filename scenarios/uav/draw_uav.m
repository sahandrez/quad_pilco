%% draw_uav.m
% *Summary:* Draw the uav with cost and rpms
%
%    function draw_uav(latent, plant,t2,cost,text1, text2, activeFigure, videoCapture)
%
%
% *Input arguments:*
%
%   latent     state of the uav (including the torques)
%   plant      plant structure
%     .dt      sampling time
%     .dyno    state indices that are passed ont the cost function
%   t2         supersampling frequency (in case you want smoother plots)
%   cost       cost structure
%     .fcn     function handle (it is assumed to use saturating cost)
%     .<>      other fields that are passed to cost
%   text1      (optional) text field 1
%   text2      (optional) text field 2
%   activeFigure    current active figure to capture the movie
%   videoWriter     VideoWriter class
%
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%

function draw_uav(latent,plant,t2,cost,text1, text2, activeFigure, videoWriter)
%% Draw the UAV

clf; set(gca,'FontSize',16);

t1 = plant.dt;

qq = latent;

clear q;
xi = t1*(0:size(qq,1)-1); xn = 0:t2:(size(qq,1)-1)*t1;
for i = 1:size(qq,2), q(:,i) = interp1(xi,qq(:,i),xn); end

%% Code from the Simulator

global PROP_POSNS

X = q;
Rbumper = 0.11;
Cbumper = PROP_POSNS(:,1);
Abumper = deg2rad(11);

% Create body-fixed centers of 4 bumpers + virtual bumper
load('locations');

c1 = PROP_POSNS(:,1);
c2 = PROP_POSNS(:,2);
c3 = PROP_POSNS(:,3);
c4 = PROP_POSNS(:,4);

n1_b = invar2rotmat('Z',deg2rad(45))'*invar2rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
n2_b = invar2rotmat('Z',deg2rad(135))'*invar2rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
n3_b = invar2rotmat('Z',deg2rad(-135))'*invar2rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
n4_b = invar2rotmat('Z',deg2rad(-45))'*invar2rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];

cR = Cbumper;

clear p1 p2 p3 p4;

% Create body-fixed points of spherical bumper
[sx,sy,sz] = sphere;
sx = sx(9:13,:);
sy = sy(9:13,:);
sz = sz(9:13,:)+PROP_POSNS(3,1);
sr = Rbumper;
sxR = zeros(size(sx));
syR = zeros(size(sy));
szR = zeros(size(sz));

% Create body-fixed points of Spiri body
p1 = [0.08;0.0115;0]-CoM;
p2 = [0;0.0575;0]-CoM;
p3 = [-0.1;0;0]-CoM;
p4 = [0;-0.0575;0]-CoM;
p5 = [0.08;-0.0115;0]-CoM;

% Create body-fixed axes
po = [0;0;0];
px = [0.1;0;0];
py = [0;0.1;0];
pz = [0;0;0.1];

% Calculate axes ranges for plotting
axis_min = min([min(X(:,7))-0.4,min(X(:,8))-0.4,min(X(:,9))-0.4]);
axis_max = max([max(X(:,7))+0.4,max(X(:,8))+0.4,max(X(:,9))+0.4]);

for i=1:size(q,1)
    % Rotate body-fixed points to world-frame points
    quat = [q(i,10);q(i,11);q(i,12);q(i,13)];
    quat = quat/norm(quat);
    R = quat2rotmat(quat);
    T = [q(i,7);q(i,8);q(i,9)];
    
    p1_p = R'*p1 + T;
    p2_p = R'*p2 + T;
    p3_p = R'*p3 + T;
    p4_p = R'*p4 + T;
    p5_p = R'*p5 + T;
    
    c1_p = R'*c1 + T;
    c2_p = R'*c2 + T;
    c3_p = R'*c3 + T;
    c4_p = R'*c4 + T;
    cR_p = R'*cR + T;
    
    po_p = R'*po + T;
    px_p = R'*px + T;
    py_p = R'*py + T;
    pz_p = R'*pz + T;
    
    pts = [p1_p p2_p p3_p p4_p p5_p p1_p];
    
    % Plot Spiri body points
    plot3(pts(1,:),pts(2,:),pts(3,:),'Color',[154 215 227]/255,'LineWidth',2);
    hold on;
    plot3(T(1),T(2),T(3),'rx','MarkerSize',8); %Centre of mass
    
    % Plot Spiri 2-d bumpers
    normal = cross(p1_p-p2_p,p2_p-p3_p);
    
    n1_w = R'*n1_b;
    n2_w = R'*n2_b;
    n3_w = R'*n3_b;
    n4_w = R'*n4_b;
    
    plotCircle3D(c1_p,n1_w,Rbumper);
    plotCircle3D(c2_p,n2_w,Rbumper);
    plotCircle3D(c3_p,n3_w,Rbumper);
    plotCircle3D(c4_p,n4_w,Rbumper);
    
    
    % Plot Spiri spherical bumper
    for j = 1:size(sx,1)
        for k = 1:size(sx,2)
            sxR(j,k) = R(:,1)'*[sx(j,k);sy(j,k);sz(j,k)];
            syR(j,k) = R(:,2)'*[sx(j,k);sy(j,k);sz(j,k)];
            szR(j,k) = R(:,3)'*[sx(j,k);sy(j,k);sz(j,k)];
        end
    end
    
    % Plot body-fixed axes
    xpts = [po_p px_p];
    ypts = [po_p py_p];
    zpts = [po_p pz_p];
    
    plot3(xpts(1,:),xpts(2,:),xpts(3,:),'r-','LineWidth',1);
    plot3(ypts(1,:),ypts(2,:),ypts(3,:),'g-','LineWidth',1);
    plot3(zpts(1,:),zpts(2,:),zpts(3,:),'b-','LineWidth',1);
    
    % Figure settings
    %     axis([axis_min,axis_max,axis_min,axis_max,axis_min,axis_max]);
    axis([-3 3 -3 3 -0.5 3]);
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
       
    grid on;
    axis square;
    
    %% draw controls:
    rpm1  = q(i,end-3);
    rpm2  = q(i,end-2);
    rpm3  = q(i,end-1);
    rpm4  = q(i,end);
    rpmMax = 10000;
    
    L = cost.fcn(cost,q(i,plant.dyno)',zeros(length(plant.dyno)));
    
    oo = [4 -3.07 0]/6.4;
    o1 = [-2 2 2.0]; 
    o2 = [-2 2 1.6]; 
    o3 = [-2 2 1.2];
    o4 = [-2 2 0.8];
    o5 = [-2 2 0.4];
    
    % RPM 1
    o0 = rpm1/rpmMax;
    plot3([o1(1) o1(1)+o0*oo(1)],[o1(2) o1(2)+o0*oo(2)],[o1(3) o1(3)+o0*oo(3)],'r','LineWidth',4)
    plot3([o1(1)-oo(1) o1(1)+oo(1) o1(1)+oo(1) o1(1)-oo(1) o1(1)-oo(1)],...
        [o1(2)-oo(2) o1(2)+oo(2) o1(2)+oo(2) o1(2)-oo(2) o1(2)-oo(2)],...
        [o1(3)+0.04 o1(3)+0.04 o1(3)-0.04 o1(3)-0.04 o1(3)+0.04], 'r');
    plot3([o1(1) o1(1)],[o1(2) o1(2)],o1(3)+[-0.06 0.06],'r');
    
    % RPM 2
    o0 = rpm2/rpmMax;
    plot3([o2(1) o2(1)+o0*oo(1)],[o2(2) o2(2)+o0*oo(2)],[o2(3) o2(3)+o0*oo(3)],'r','LineWidth',4)
    plot3([o2(1)-oo(1) o2(1)+oo(1) o2(1)+oo(1) o2(1)-oo(1) o2(1)-oo(1)],...
        [o2(2)-oo(2) o2(2)+oo(2) o2(2)+oo(2) o2(2)-oo(2) o2(2)-oo(2)],...
        [o2(3)+0.04 o2(3)+0.04 o2(3)-0.04 o2(3)-0.04 o2(3)+0.04], 'r');
    plot3([o2(1) o2(1)],[o2(2) o2(2)],o2(3)+[-0.06 0.06],'r');
    
    % RPM 3
    o0 = rpm3/rpmMax;
    plot3([o3(1) o3(1)+o0*oo(1)],[o3(2) o3(2)+o0*oo(2)],[o3(3) o3(3)+o0*oo(3)],'r','LineWidth',4)
    plot3([o3(1)-oo(1) o3(1)+oo(1) o3(1)+oo(1) o3(1)-oo(1) o3(1)-oo(1)],...
        [o3(2)-oo(2) o3(2)+oo(2) o3(2)+oo(2) o3(2)-oo(2) o3(2)-oo(2)],...
        [o3(3)+0.04 o3(3)+0.04 o3(3)-0.04 o3(3)-0.04 o3(3)+0.04], 'r');
    plot3([o3(1) o3(1)],[o3(2) o3(2)],o3(3)+[-0.06 0.06],'r');
    
    % RPM 4
    o0 = rpm4/rpmMax;
    plot3([o4(1) o4(1)+o0*oo(1)],[o4(2) o4(2)+o0*oo(2)],[o4(3) o4(3)+o0*oo(3)],'r','LineWidth',4)
    plot3([o4(1)-oo(1) o4(1)+oo(1) o4(1)+oo(1) o4(1)-oo(1) o4(1)-oo(1)],...
        [o4(2)-oo(2) o4(2)+oo(2) o4(2)+oo(2) o4(2)-oo(2) o4(2)-oo(2)],...
        [o4(3)+0.04 o4(3)+0.04 o4(3)-0.04 o4(3)-0.04 o4(3)+0.04], 'r');
    plot3([o4(1) o4(1)],[o4(2) o4(2)],o4(3)+[-0.06 0.06],'r');
    
    % Loss 
    o0 = 3*L-1.5;
    plot3([o5(1)-oo(1) o5(1)+oo(1)],[o5(2)-oo(2) o5(2)+oo(2)],[o5(3)-oo(3) o5(3)+o0*oo(3)],'k','LineWidth',4)
    plot3([o5(1)-oo(1) o5(1)+oo(1) o5(1)+oo(1) o5(1)-oo(1) o5(1)-oo(1)],...
        [o5(2)-oo(2) o5(2)+oo(2) o5(2)+oo(2) o5(2)-oo(2) o5(2)-oo(2)],...
        [o5(3)+0.04 o5(3)+0.04 o5(3)-0.04 o5(3)-0.04 o5(3)+0.04], 'k');    
    
    text(1,1,4,text1);
    text(1,1,3.5,text2);
    
    drawnow
    hold off;
    
    % capture the frame and write the video
    frame = getframe(activeFigure);
    writeVideo(videoWriter, frame);
end

