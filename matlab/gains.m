%% 6th order: h1 and h3
A6 = [zeros(5,1), eye(5); zeros(1,6)];
B6 = [zeros(5,1);1];

K1 = place(A6,B6,[-1 -1.5 -2 -2.5 -3 -3.5]*1);
% K3 = place(A6,B6,[-1.5 -2 -2.5 -3 -3.5 -4]);

%% 2nd order: h2
A2 = [zeros(1,1), eye(1); zeros(1,2)];
B2 = [zeros(1,1);1];

K2 = place(A2,B2,[-1 -1.2]);

%% 5th order: h3 vel control
A5 = [zeros(4,1), eye(4); zeros(1,5)];
B5 = [zeros(4,1);1];

K3 = place(A5,B5,[-1 -1.5 -2 -2.5 -3]*1);


