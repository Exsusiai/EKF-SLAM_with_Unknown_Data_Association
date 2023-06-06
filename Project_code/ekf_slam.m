load('test_3.mat');

%% config
format compact

% control parameters
V= 3; % m/s
MAXG= 30*pi/180; % radians, maximum steering angle (-MAXG < g < MAXG)
RATEG= 20*pi/180; % rad/s, maximum rate of change in steer angle
WHEELBASE= 4; % metres, vehicle wheel-base
DT_CONTROLS= 0.025; % seconds, time interval between control signals

% control noises
sigmaV= 0.3; % m/s
sigmaG= (3.0*pi/180); % radians
Q= [sigmaV^2 0; 0 sigmaG^2];

% observation parameters
MAX_RANGE= 30.0; % metres
DT_OBSERVE= 8*DT_CONTROLS; % seconds, time interval between observations

% observation noises
sigmaR= 0.1; % metres
sigmaB= (1.0*pi/180); % radians
R= [sigmaR^2 0; 0 sigmaB^2];

% data association innovation gates (Mahalanobis distances)
GATE_REJECT= 4.0; % maximum distance for association
GATE_AUGMENT= 25.0; % minimum distance for creation of new feature

% waypoint proximity
AT_WAYPOINT= 1.0; % metres, distance from current waypoint at which to switch to next waypoint
NUMBER_LOOPS= 3; % number of loops through the waypoint list

% initialise states
xtrue= zeros(3,1);
x= zeros(3,1);
P= zeros(3);

% initialise other variables and constants
dt= DT_CONTROLS; % change in time between predicts
dtsum= 0; % change in time since last observation
ftag= 1:size(lm,2); % identifier for each landmark
% da_table= zeros(1,size(lm,2)); % data association table 
iwp= 1; % index to first waypoint 
G= 0; % initial steer angle
data= initialise_store(x,P,x); %  stored data for off-line
QE= Q; RE= R; 
% if SWITCH_SEED_RANDOM, randn('state',SWITCH_SEED_RANDOM), end
error_dist=[];


%% setup plots

fig=figure;
plot(lm(1,:),lm(2,:),'bo')
hold on, axis equal
plot(wp(1,:),wp(2,:), 'g', wp(1,:),wp(2,:),'g*')
xlabel('metres'), ylabel('metres')
set(fig, 'name', 'EKF-SLAM Simulator')
h= setup_animations;
veh= [0 -WHEELBASE -WHEELBASE; 0 -2 2]; % vehicle animation
plines=[]; % for laser line animation
pcount=0;

%% main loop 
while iwp ~= 0
%% compute steering
    cwp= wp(:,iwp);
    d2= (cwp(1)-xtrue(1))^2 + (cwp(2)-xtrue(2))^2;
    if d2 < AT_WAYPOINT^2 % determine if current waypoint reached
        iwp= iwp+1; % switch to next
        if iwp > size(wp,2) % reached final waypoint, flag and return
            iwp=0;
        else
            cwp= wp(:,iwp); % next waypoint
        end
    end
    
    if iwp==0 && NUMBER_LOOPS > 1
        iwp=1; 
        NUMBER_LOOPS= NUMBER_LOOPS-1; 
    else
          % compute change in G to point towards current waypoint
        deltaG= pi_to_pi(atan2(cwp(2)-xtrue(2), cwp(1)-xtrue(1)) - xtrue(3) - G);
        
        % limit rate
        maxDelta= RATEG*dt;
        if abs(deltaG) > maxDelta
            deltaG= sign(deltaG)*maxDelta;
        end
        
        % limit angle
        G= G+deltaG;
        if abs(G) > MAXG
            G= sign(G)*MAXG;
        end
    end % perform loops: if final waypoint reached, go back to first
    
    xtrue= [xtrue(1) + V*dt*cos(G+xtrue(3,:));  %calculate vehicle model
            xtrue(2) + V*dt*sin(G+xtrue(3,:));
            pi_to_pi(xtrue(3) + V*dt*sin(G)/WHEELBASE)];

     Vn = V;
     Gn = G;
     Vn= Vn + randn(1)*sqrt(Q(1,1));% add control noise
     Gn= Gn + randn(1)*sqrt(Q(2,2));
    %% EKF predict step

    [x,P]= predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
    
    %% EKF update step
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        z= get_observations(xtrue, lm, ftag, MAX_RANGE);

    % add_observation_noise
        len= size(z,2);
        if len > 0
            z(1,:)= z(1,:) + randn(1,len)*sqrt(R(1,1));
            z(2,:)= z(2,:) + randn(1,len)*sqrt(R(2,2));
        end
        
        [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT); 
        
        [x,P]= update(x,P,zf,RE,idf); 

        [x,P]= augment(x,P, zn,RE); 
    end
    
    % offline data store
    data= store_data(data, x, P, xtrue);
    error_dist(end+1) = sqrt((x(1)-xtrue(1))^2+(x(2)-xtrue(2))^2);

    %% plots
    xt= transformtoglobal(veh,xtrue);
    xv= transformtoglobal(veh,x(1:3));
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
    set(h.xv, 'xdata', xv(1,:), 'ydata', xv(2,:))
    set(h.xf, 'xdata', x(4:2:end), 'ydata', x(5:2:end))
    ptmp= make_covariance_ellipses(x(1:3),P(1:3,1:3));
    pcov(:,1:size(ptmp,2))= ptmp;
    if dtsum==0
        set(h.cov, 'xdata', pcov(1,:), 'ydata', pcov(2,:)) 
        pcount= pcount+1;
        if pcount == 15
            set(h.pth, 'xdata', data.path(1,1:data.i), 'ydata', data.path(2,1:data.i))
            pcount=0;
        end
        if ~isempty(z)
            plines= make_laser_lines (z,x(1:3));
            set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:))
            pcov= make_covariance_ellipses(x,P);
        end
    end
    drawnow
end

data= finalise_data(data);
set(h.pth, 'xdata', data.path(1,:), 'ydata', data.path(2,:))   

figure;
plot(error_dist);
xlabel('time');
ylabel('error/m'); 
% 
%

function h= setup_animations()
    
    h.xt= patch(0,0,'b'); % vehicle true
    h.xv= patch(0,0,'r'); % vehicle estimate
    h.pth= plot(0,0,'k.','markersize',2); % vehicle path estimate
    h.obs= plot(0,0,'r'); % observations
    h.xf= plot(0,0,'r+'); % estimated features
    h.cov= plot(0,0,'r'); % covariance ellipses
end
%
%

function p= make_laser_lines (rb,xv)
    % compute set of line segments for laser range-bearing measurements
    if isempty(rb), p=[]; return, end
    len= size(rb,2);
    lnes(1,:)= zeros(1,len)+ xv(1);
    lnes(2,:)= zeros(1,len)+ xv(2);
    lnes(3:4,:)= transformtoglobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
    len= size(lnes,2)*3 - 1;
    p= zeros(2, len);
    
    p(:,1:3:end)= lnes(1:2,:);
    p(:,2:3:end)= lnes(3:4,:);
    p(:,3:3:end)= NaN;
end
%
%

function p= make_covariance_ellipses(x,P)
    % compute ellipses for plotting state covariances
    N= 10;
    inc= 2*pi/N;
    phi= 0:inc:2*pi;
    
    lenx= length(x);
    lenf= (lenx-3)/2;
    p= zeros (2,(lenf+1)*(N+2));
    
    ii=1:N+2;
    p(:,ii)= make_ellipse(x(1:2), P(1:2,1:2), 2, phi);
    
    ctr= N+3;
    for i=1:lenf
        ii= ctr:(ctr+N+1);
        jj= 2+2*i; jj= jj:jj+1;
        
        p(:,ii)= make_ellipse(x(jj), P(jj,jj), 2, phi);
        ctr= ctr+N+2;
    end
end

%
%

function p= make_ellipse(x,P,s, phi)
    % make a single 2-D ellipse of s-sigmas over phi angle intervals 
    r= sqrtm(P);
    a= s*r*[cos(phi); sin(phi)];
    p(2,:)= [a(2,:)+x(2) NaN];
    p(1,:)= [a(1,:)+x(1) NaN];
end

%
%

function data= initialise_store(x,P, xtrue)
    % offline storage initialisation
    data.i=1;
    data.path= x;
    data.true= xtrue;
    data.state(1).x= x;
    %data.state(1).P= P;
    data.state(1).P= diag(P);
end

%
%

function data= store_data(data, x, P, xtrue)
    % add current data to offline storage
    CHUNK= 5000;
    if data.i == size(data.path,2) % grow array in chunks to amortise reallocation
        data.path= [data.path zeros(3,CHUNK)];
        data.true= [data.true zeros(3,CHUNK)];
    end
    i= data.i + 1;
    data.i= i;
    data.path(:,i)= x(1:3);
    data.true(:,i)= xtrue;
    data.state(i).x= x;
    %data.state(i).P= P;
    data.state(i).P= diag(P);
end

%
%

function data= finalise_data(data)
    % offline storage finalisation
    data.path= data.path(:,1:data.i);
    data.true= data.true(:,1:data.i);
end


function angle = pi_to_pi(angle)
    angle = mod(angle, 2*pi);
    
    i=find(angle>pi);
    angle(i)=angle(i)-2*pi;
    
    i=find(angle<-pi);
    angle(i)=angle(i)+2*pi;
end



