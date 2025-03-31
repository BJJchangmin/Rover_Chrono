rng(42);


%%%%%%%%%%%%%%
% A = grf_x{1,1}(1:50)' Data Form
%%%%%%%%%%%%%%

start_time = 0;
end_time = 5;
s_time_idx = find(t == start_time); 
e_time_idx = find(t == end_time);

time = t(s_time_idx:e_time_idx);
[m,n] = size(time);

W_data = grf_z{1,1}(s_time_idx:e_time_idx);

T_data = drive_torque{1,1}(s_time_idx:e_time_idx);

A = ones(m,n);
th1_data = deg2rad(10.*A); % I have to make data

% slip ratio
s_data = slip_ratio{1,1}(s_time_idx:e_time_idx);

%shearing radius
r_s = 0.25; 

%True Data
ydata = grf_x{1,1}(s_time_idx:e_time_idx);

initial_guess = [1; 1];

% Constant value is not in here
xdata =[W_data; T_data; th1_data; s_data]; 

objFun = @(param) sum((ydata - DPmodel(xdata(1,:),xdata(2,:),xdata(3,:),xdata(4,:),r_s, param)).^2);

lb = [-10;-10];
ub = [10; 10];



options = optimoptions('fmincon', ...
    'Display', 'iter', ...               % 반복 과정 출력
    'StepTolerance', 1e-15, ...           % Step 크기 허용오차 조절 (더 정밀하게)
    'OptimalityTolerance', 1e-15, ...     % 1차 도함수 허용오차 조절
    'FiniteDifferenceStepSize', 1e-12, ... % 수치 미분 단계 크기 조절
    'Algorithm', 'sqp');                  % 알고리즘 변경 (sqp 사용)


[estimated_params,fval,exitflag,output] = fmincon(objFun, initial_guess, [], [], [], [], lb, ub, [], options);

% 결과 출력
disp('Identify Parameter (c1, c2):');
disp(estimated_params);

