function J = f_num_diff(fun, x, h, cdiff)
% returns the Jacobian J given a function and states
if cdiff
    n = size(x,1);
    H = [-h*eye(n), h*eye(n)];
    H = permute(H, [1 3 2]);
    X = x + H;
    for i = 1:2*n
        Y(:,i) = fun(X(:,:,i));
    end
    J = (Y(:,n+1:2*n)-Y(:,1:n))/(2*h);
else
    n = size(x,1);
    H = [zeros(n,1), h*eye(n)];
    H = permute(H, [1 3 2]);
    X = x + H;
    for i = 1:n+1
        Y(:,i) = fun(X(:,:,i));
    end
    J = (Y(:,2:n+1)-Y(:,1))/(h);
end