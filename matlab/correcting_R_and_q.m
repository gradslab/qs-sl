%% Projecting q on S2
[U,S,V] = svd([q(1);q(2);q(3)]);
q_matriax  = U*V.';
q = q_matriax(:,1); %q corrected
q__1 = q(1); q__2 = q(2); q__3 = q(3);
% q__3 = sign(q__3)*sqrt(1-q__1^2-q__2^2);
%% Projecting R on SO3
[U,S,V] = svd([r11, r12, r13; ...
    r21, r22, r23; ...
    r31, r32, r33]);
R  = U*V.';

