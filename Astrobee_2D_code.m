clc
clear all

% define parameters
m = 1; %mass: kg
a = 1; % edge length: m
Izz = 1/12*m*a^2; % moment of inertia
t_end = 4; % simulation time
C = 4e-4; % damping

% initial conidtion
% the astrobee will be rest at origin with zero rotation angle in the
% begining
vx0 = 0; vy0 = 0; omega0 = 0;
x0 = 0; y0 = 0; theta0 = 0;

% applied thrust
T0 = 1; % magnitude of thrust
ang_T = 0; % angle of thrust (horizon if it is 0)
r_CP0 = a.*[0.5,0]';

% Discretize time
t0 = 0;
segment = 10000;
t = linspace(t0,t_end,segment);
dt = t(2)-t(1);

% Simulation part
clear x y vx vy theta omega
% assign initial condiitons
x(1) = x0; y(1) = y0; theta(1) = theta0; 
vx(1) = vx0; vy(1) = vy0; omega(1) = omega0; 

% loop: body of simulation
for i = 1:segment -1 
    % If thrust is not rotating with RB
    T(:,i) = [T0*cos(ang_T),T0*sin(ang_T)]; 
    
    %If thrust is rotating with RB
    % T(:,i) = rotation_matrix(theta(i))*[T0*cos(ang_T);T0*sin(ang_T)]; %
    
    % Total forces
    Fx(i) = T(1,i);
    Fy(i) = T(2,i);
    
    % Total moment around COM
    r_CP = rotation_matrix(theta(i))*r_CP0;
    P_loc(:,i) = r_CP + [x(i),y(i)]'; % this line provides location of P, and is for animation purpose only
    Mc_3d = cross([r_CP;0],[T(:,i);0]); % note that in matlab cross() only works for 3D vectors
    Mc(i) = Mc_3d(3); % only care about the z component

    vx(i+1) = vx(i) + Fx(i)/m*dt;
    vy(i+1) = vy(i) + Fy(i)/m*dt;
    omega(i+1) = omega(i) + Mc(i)/Izz*dt - C*omega(i);

    x(i+1) = x(i) + vx(i)*dt;
    y(i+1) = y(i) + vy(i)*dt;
    theta(i+1) = theta(i) + omega(i)*dt;
end

%% Plotting code
fig = figure(1);
set(fig,'Position',[0,0,1200,500]);
folder_name = 'lecture24_videos';
mkdir(folder_name);
% use current name as file name
filename = [folder_name,'/',char(datetime('now','Format','MM-dd_HH_mm_ss'))];
myVideo = VideoWriter(filename,'MPEG-4');
framerate = 20; 
myVideo.FrameRate = framerate;
open(myVideo);
% plot
for i = 1:50:segment-1
    clf % comment this line if you want to see the trace
    hold on
    % draw square
    square_plotter(x(i),y(i),theta(i),a);
    % draw arrow
    quiver(P_loc(1,i),P_loc(2,i),T(1,i),T(2,i),'-r');

    xlim([min(x)-a,max(x)+a]);
    ylim([min(y)-a,max(y)+a]);
    axis equal
    pause(0.001);
    frame = getframe(gcf);
    writeVideo(myVideo, frame);
end
close(myVideo);

function R = rotation_matrix(ang)
    R = [cos(ang),-sin(ang);+sin(ang),cos(ang)];
end

function square_plotter(xc,yc,ang,a)
    initial_vertice = [-1,-1;-1,1;1,1;1,-1]'.*a/2;
    rotated_vertice = rotation_matrix(ang)*initial_vertice;
    fill(rotated_vertice(1,:) + xc, rotated_vertice(2,:) + yc, 'b')
end
