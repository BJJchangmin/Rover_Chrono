

%%%%%%%%%% Soil Parameter %%%%%%%%%%

%What is unit problem/
k_c = 0.14*10^4;
k_phi = 0.82*10^6;

ks = k_c/0.25 +k_phi;
% ks = 2500;

c= 0.017*10^4;
fric_ang = 35*pi/180;
janosi_k = 1.78*10^-2;

%Tunning factor
c_1 = 0.5; % Effect W,DP,T
c_2 = 0.1;   % Only Effect DP

n0 = 1.0; % Don't Touch. Only Effect Normal Force
n1 = 0.7;  % Don't Touch. Only Effect Normal Force



%%%%%%%%%% DATA %%%%%%%%%%

% Data Setting2
start_time = 1.5;
end_time = 10; 
s_time_idx = find(t == start_time); 

e_time_idx = find(t == end_time);

time = t(s_time_idx:e_time_idx);
[row,col] = size(time);

Leg_num = 1;

% F_n Data
W= grf_z{1,Leg_num}(s_time_idx:e_time_idx);
W = lowpass(W, 100, 1000);

%DP
DP= grf_x{1,Leg_num}(s_time_idx:e_time_idx);
DP = lowpass(DP, 100, 1000);

% T Data
T = drive_torque{1,Leg_num}(s_time_idx:e_time_idx);
T = lowpass(T, 100, 1000);

% th_1 data
A = ones(row,col);
th_1 = deg2rad(34.5);
th_1 = th_1.*A; % Tunning Parameter

%slip ratio data
s = slip_ratio{1,Leg_num}(s_time_idx:e_time_idx);

% 데이터 크기 확인
N = length(s);  

% Low-pass filter 적용 (각 요소별)


s = lowpass(s, 100, 1000);





%%%%%%%%%% Constant %%%%%%%%%%


r = 0.25;
r_s = 0.25 +0.5*0.005 ;

b=0.25;

th_1_n = th_s_model(s,1);

th_m = (c_1+c_2.*s).*th_1;


%%%%%%%%%%
c1 = c_1;
c2 = c_2;

A = (cos(th_m)-1)./th_m + (cos(th_m)-cos(th_1))./(th_1-th_m);

B = sin(th_m)./th_m + (sin(th_m)-sin(th_1))./(th_1-th_m);

C = th_1./2;
disp(A);

% I_DP = (A.^2 + B.^2).*T./(r_s.*A.*C) - B.*W./A;
term1 = B./A;
term2 = (A.*A + B.*B)./(r_s.*A.*C);
I_DP = (-(A.*A + B.*B)./(r_s.*A.*C).*T + B.*W./A);

sigma_m = ks.*(r.*(cos(th_m)-cos(th_1))).^(n0);

D_exp = -r_s.*( (th_1-th_m) - (1-s).*(sin(th_1)-sin(th_m)))./janosi_k;

D = (1-exp(D_exp));

tau_m = (c+sigma_m.*tan(fric_ang)).*D;


%Plus,Minus Check
X = r.*b.*sigma_m;
Y = r_s.*b.*tau_m;

cal_W = A.*X + B.*Y;
cal_DP = -B.*X + A.*Y;
cal_T = r_s.*C.*Y;

%%%%%%%%%%



Tparam = [ c, fric_ang, janosi_k]; % c , phi, janosi_k
Ideal_T = Tmodel(W, th_1, s, r_s, b, th_1_n, th_m, Tparam);


DPParam = [ c_1, c_2]; % c_1 , c_2
Ideal_DP = DPmodel(W, T, th_1, s, r_s, DPParam);
% Ideal_DP = I_DP;

WParam = [ks, n0, n1];
Ideal_W = Wmodel( T, th_1, s, r, r_s, b, th_m, WParam);
Ideal_W = lowpass(Ideal_W, 100, 1000);


Dri_observer = 0.25.*((T - 0.4.*W)./r_s) + 200;
% Dri_observer = -30*((T - 0.4.*W)./r_s) ;
Dri_observer = lowpass(Dri_observer, 10, 1000);



 %%%%%%%%%%%%%%%%%%%% DATA PLOT %%%%%%%%%%%%%%%%%%%

%Plotting Parameter
lw =1;   %Line Width
FT = 7; %Title Fonte Size
sgT= 18; % subtitle plot title
Faxis = 12.5; %Axis Fonte Size
fl =10 ; % Legend Fonte Size
Ms = 3 ; %Mark Size

% figure (20)
% subplot(3,1,1);
% plot(t(s_time_idx:e_time_idx),W,'k-','LineWidth', lw);
% hold on
% plot(t(s_time_idx:e_time_idx),Ideal_W,'r-','LineWidth', lw);
% grid on;
% ylabel('Normal Force (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
% 
% subplot(3,1,2);
% plot(t(s_time_idx:e_time_idx),T,'k-','LineWidth', lw);
% hold on
% plot(t(s_time_idx:e_time_idx),Ideal_T,'r-','LineWidth', lw);
% grid on;
% ylabel('Motor Torque (Nm)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
% 
% subplot(3,1,3);
% plot(t(s_time_idx:e_time_idx),DP,'k-','LineWidth', lw);
% hold on
% plot(t(s_time_idx:e_time_idx),Ideal_DP,'r-','LineWidth', lw);
% hold on
% % plot(t(s_time_idx:e_time_idx),Dri_observer,'b-','LineWidth', lw);
% grid on;
% ylabel('Driving Force (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블

% figure("Name","Motor Torque")
% plot(t(s_time_idx:e_time_idx),T,'k-','LineWidth', lw);
% hold on
% plot(t(s_time_idx:e_time_idx),Ideal_T,'r-','LineWidth', lw);
% grid on;
% ylabel('Motor Torque (Nm)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블

figure(1)
plot(t(s_time_idx:e_time_idx),W,'k-','LineWidth', lw);
hold on
% plot(t(s_time_idx:e_time_idx),Ideal_W,'r-','LineWidth', lw);
hold on
plot(t(s_time_idx:e_time_idx),cal_W,'b-','LineWidth', lw);
grid on;
ylabel('Force (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
legend('Sensor','est','model','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
title('Normal Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(2)
plot(t(s_time_idx:e_time_idx),T,'k-','LineWidth', lw);
hold on
% plot(t(s_time_idx:e_time_idx),Ideal_T,'r-','LineWidth', lw);
hold on
plot(t(s_time_idx:e_time_idx),cal_T,'b-','LineWidth', lw);
grid on;
ylabel('Torque (Nm)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
legend('Sensor','est','model','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
title('Motor Torque','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(3)
plot(t(s_time_idx:e_time_idx),DP,'k-','LineWidth', lw);
hold on
% plot(t(s_time_idx:e_time_idx),Ideal_DP,'r-','LineWidth', lw);
hold on
plot(t(s_time_idx:e_time_idx),cal_DP,'b-','LineWidth', lw);
% hold on
% plot(t(s_time_idx:e_time_idx),Dri_observer,'g-','LineWidth', lw);
grid on;
ylabel('Force (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
legend('Sensor','est','model','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
title('Driving Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');






setFigurePositions(3);







