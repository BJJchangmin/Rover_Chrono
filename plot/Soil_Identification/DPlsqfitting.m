
wheel_num = 1;
% Step 1: 실험 데이터 생성 (예시 데이터)
W_data = grt_x{wheel_num};             % 수직 하중 값
T_data = [10, 11, 9, 12, 10];                % 토크 값
th1_data = deg2rad([30, 32, 28, 31, 29]);     % 입구 각도 (radian 단위)
s_data = [0.1, 0.12, 0.08, 0.11, 0.09];       % 슬립비 값
r_s = 0.15;                                  % shearing radius (상수)


% 실제 파라미터 (예시)
true_params = [0.5; 0.2];

% 합성 DP 데이터 생성 (실제 데이터 대신)
nData = length(W_data);
DP_true = zeros(1, nData);
for i = 1:nData
    DP_true(i) = DPmodel(W_data(i), T_data(i), th1_data(i), s_data(i), r_s, true_params);
end
% 약간의 노이즈 추가
noise = 0.01 * randn(size(DP_true));
ydata = DP_true + noise;



% Constant value is not in here
xdata =[W_data; T_data; th1_data; s_data]; 

modelFun = @(param, xdata)DPmodel(xdata(1,:), ...
    xdata(2,:),xdata(3,:),xdata(4,:),r_s, param);


initial_guess = [0.5; 0.2];

options = optimset('Display','iter');
estimated_params = lsqcurvefit(modelFun, initial_guess, xdata, ydata,[],[],options );

% 결과 출력
disp('Identify Parameter (c1, c2):');
disp(estimated_params);

for i = 1:nData
    DP_res(i) = DPmodel(W_data(i), T_data(i), th1_data(i), s_data(i), r_s, estimated_params);
end

residual = ydata - DP_res;
MSE = mean(residual.^2);

figure;
plot(residual, 'o-');
xlabel('Data Index');
ylabel('Residual');
title('Residual Plot');
grid on;
