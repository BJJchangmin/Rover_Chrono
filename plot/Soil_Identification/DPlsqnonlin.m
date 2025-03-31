rng(42);

% 데이터 개수
nData = 20;

% 수직 하중 값 (W)
W_base = [80, 85, 90, 95, 100]; % 기존 데이터
W_data = repmat(W_base, 1, 4) + randi([-5, 5], 1, nData); 

% 토크 값 (T)
T_base = [10, 11, 9, 12, 10]; % 기존 데이터
T_data = repmat(T_base, 1, 4) + randi([-2, 2], 1, nData); 

% 입구 각도 (theta1) (degree -> radian 변환)
th1_base = deg2rad([30, 32, 28, 31, 29]); % 기존 데이터
th1_data = repmat(th1_base, 1, 4) + deg2rad(randi([-3, 3], 1, nData)); 

% 슬립비 (s)
s_base = [0.1, 0.12, 0.08, 0.11, 0.09]; % 기존 데이터
s_data = repmat(s_base, 1, 4) + 0.01 * randn(1, nData); 
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
ydata = DP_true;


initial_guess = [0.5; 0.2];


% Constant value is not in here
xdata =[W_data; T_data; th1_data; s_data]; 

residualFun = @(param) ydata - DPmodel(xdata(1,:),xdata(2,:),xdata(3,:),xdata(4,:),r_s, param);

% 최적화 옵션 설정
options = optimset('Display', 'iter');

% lsqnonlin 실행
estimated_params = lsqnonlin(residualFun, initial_guess, [], [], options);


% 결과 출력
disp('식별된 파라미터 (c1, c2):');
disp(estimated_params);


