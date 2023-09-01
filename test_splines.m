

x = [0 1 2 3 4 5 6 7]
y = [1 0 1 -1 -2 -1 3 1]

M = [1 4 1 0;
    -3 0 3 0;
    3 -6 3 0;
    -1 3 -3 1] / 6;

C = [6 0 0 0;
    5 3 -3 1;
    1 3 3 -2;
    0 0 0 1] / 6

t_=[]
ys_=[]
Mu_=[]
for i=2:size(x,2)-2
    t = linspace(x(i),x(i+1),100);
    ut=t-x(i);
    u = [ones(size(ut));ut;ut.^2;ut.^3];
    % Definition of B splines using standard basis function
    Mu = u' * M * y(i-1:i+2)';
    % Definition of B splines using cumulative basis function
    Cu = C * u;
    ys = y(i-1) + sum(diff(y(i-1:i+2))*Cu(2:4,:),1);

    t_ = [t_ t];
    ys_ = [ys_ ys];
    Mu_ = [Mu_ Mu'];
end

load('stest')
figure(1)
plot(x,y,'ob')
hold on
plot(t_,Mu_,'xk')
plot(stest(:,1),stest(:,2),'-b')
hold off
figure(2)
plot(x,y,'ob')
hold on
plot(t_,ys_,'xk')
plot(stest(:,1),stest(:,3),'-b')
hold off



