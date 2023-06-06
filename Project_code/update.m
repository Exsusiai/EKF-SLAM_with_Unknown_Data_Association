function [x,P]= update(x,P,z,R,idf)

lenz= size(z,2);
lenx= length(x);
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(x, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end
        
PHt= P*H';
S= H*PHt + RR;

S= (S+S')*0.5; % make symmetric
SChol= chol(S);

SCholInv= inv(SChol); % triangular matrix
W1= PHt * SCholInv;
W= W1 * SCholInv';

x= x + W*v; % update 
P= P - W1*W1';