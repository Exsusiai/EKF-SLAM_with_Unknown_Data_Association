function [z,H]= observe_model(x, idf)

Nxv= 3; % number of vehicle pose states
fpos= Nxv + idf*2 - 1; % position of xf in state
H= zeros(2, length(x));

% auxiliary values
dx= x(fpos)  -x(1); 
dy= x(fpos+1)-x(2);
d2= dx^2 + dy^2;
d= sqrt(d2);
xd= dx/d;
yd= dy/d;
xd2= dx/d2;
yd2= dy/d2;

% predict z
z= [d;
    atan2(dy,dx) - x(3)];

% calculate H
H(:,1:3)        = [-xd -yd 0; yd2 -xd2 -1];
H(:,fpos:fpos+1)= [ xd  yd;  -yd2  xd2];
