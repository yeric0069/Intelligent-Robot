function [ineqvalue, eqvalue] = nconst(eta, prob_info)

N = prob_info.N;
T = prob_info.T;
Nx = prob_info.Nx;
Nu = prob_info.Nu;
x0 = prob_info.x0;
num = prob_info.num;

delta = T/N;

x = reshape(eta(1:N*Nx,1), Nx, N);

x = [x0 x];

u = reshape(eta((N*Nx+1):(N*Nx+N*Nu)), Nu, N);

% eqvalue constrain
eqvalue = zeros(N*Nx,1); % initialisation

for index=1:N
    eqvalue( ((index-1)*Nx+1):index*Nx,1) = ...
    x(:,index+1) - x(:,index) - delta*sys_h(x(:,index), u(:,index), prob_info); 
end 

% ineqvalue constrain
ineqvalue = zeros(N,num); % initialisation

for index= 1:N
    for idx =1:num
        r = prob_info.radii(idx);
        c1 = prob_info.centres(idx,1); 
        c2 = prob_info.centres(idx,2);

        ineqvalue(index,idx) = r^2 - (x(1,index+1)-c1)^2 - (x(2,index+1)-c2)^2;
    end
end