function value = cost(eta, prob_info)

    N = prob_info.N;
    Nx = prob_info.Nx;
    x0 = prob_info.x0;
    goal = prob_info.goal;
    wh = prob_info.wh;
    
    % reshape this matrix as Nx*N
    x = reshape(eta(1:N*Nx,1), Nx, N);
    
    % add initial state x0
    x = [x0 x];
    
    % calculate the errors
    value = (x(1,N+1)-goal(1,1))^2 +...
            (x(2,N+1)-goal(2,1))^2 +...
            wh*(goal(3,1)-x(3,N+1))^2 +...
            x(4,N+1)^2 +...
            x(5,N+1);