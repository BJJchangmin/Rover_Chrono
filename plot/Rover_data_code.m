close all;

filename{1} = 'data_FL.csv';
filename{2} = 'data_FR.csv';
filename{3} = 'data_RL.csv';
filename{4} = 'data_RR.csv';
filename{5} = 'data_trunk.csv';

%Leg Data Load  
for i = 1:1:4 
    Arr_Leg{i} = table2array(readtable(filename{i}));
end

%Trunk Data Load
Arr_trunk = table2array(readtable(filename{5}));


t = Arr_Leg{1}(:,1);

for i = 1:1:4

    sus_pos_ref{i} = Arr_Leg{i}(:,2);
    sus_pos{i} = Arr_Leg{i}(:,3);
    sus_vel{i} = Arr_Leg{i}(:,4);
    sus_torque{i} = Arr_Leg{i}(:,5);

    steer_pos_ref{i} = Arr_Leg{i}(:,6);
    steer_pos{i} = Arr_Leg{i}(:,7);
    steer_vel{i} = Arr_Leg{i}(:,8);
    steer_torque{i} = Arr_Leg{i}(:,9);

    drive_vel_ref{i} = Arr_Leg{i}(:,10);
    drive_pos{i} = Arr_Leg{i}(:,11);
    drive_vel{i} = Arr_Leg{i}(:,12);
    drive_torque{i} = Arr_Leg{i}(:,13);

%     drive_vel_ref{i} = Arr_Leg{i}(:,10);
%     drive_pos{i} = Arr_Leg{i}(:,11);
%     drive_vel{i} = Arr_Leg{i}(:,12);
%     drive_torque{i} = Arr_Leg{i}(:,13);

    slip_ratio{i} = Arr_Leg{i}(:,14);

    grf_x{i} = Arr_Leg{i}(:,15);
    grf_y{i} = Arr_Leg{i}(:,16);
    grf_z{i} = Arr_Leg{i}(:,17);

    grt_x{i} = Arr_Leg{i}(:,18);
    grt_y{i} = Arr_Leg{i}(:,19);
    grt_z{i} = Arr_Leg{i}(:,20);

    sinkage{i} = Arr_Leg{i}(:,21);

    slip_ref{i} = Arr_Leg{i}(:,22);

    dri_pid_ctrl{i} = Arr_Leg{i}(:,23);
    soil_comp_ctrl{i} = Arr_Leg{i}(:,24);

    steer_pid_ctrl{i} = 1*Arr_Leg{i}(:,25);
    steer_ff_ctrl{i} =  1*Arr_Leg{i}(:,26);
    steer_dob_ctrl{i} = 1*Arr_Leg{i}(:,27);


    alpha{i} = Arr_Leg{i}(:,28);




end

Trunk_x_vel = Arr_trunk(:,1);
Trunk_y_vel = Arr_trunk(:,2);
Trunk_z_vel = Arr_trunk(:,3);

Trunk_x_ang_vel = Arr_trunk(:,4);
Trunk_y_ang_vel = Arr_trunk(:,5);
Trunk_z_ang_vel = Arr_trunk(:,6);

Trunk_x_pos = Arr_trunk(:,7);
Trunk_y_pos = Arr_trunk(:,8);
Trunk_z_pos = Arr_trunk(:,9);
x_offset = Trunk_x_pos(1);
y_offset = Trunk_y_pos(1); 

Trunk_x_acc = Arr_trunk(:,10);
Trunk_y_acc = Arr_trunk(:,11);
Trunk_z_acc = Arr_trunk(:,12);

Trunk_x_vel_ref = Arr_trunk(:,13);
Trunk_y_vel_ref = Arr_trunk(:,14);
Trunk_yaw_rate_ref = Arr_trunk(:,15);

steer_pos_a  = steer_pos_ref{1};





%%%%%%%%%%%%%%%%%%%%% TRUNK STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Ts = t(2,1)-t(1,1);
for i = 1:length(sus_pos_ref)
    t(i,1) = (i-1)*Ts;
end

[row,col] = size(t);

steer_vel_cmd =Tustin(20,steer_pos_a,t,0);
steer_acc_cmd = Tustin(20,steer_vel_cmd,t,0);

%%%%%%%%%%%%%%%%%%%% DATA PLOT %%%%%%%%%%%%%%%%%%%

%Plotting Parameter for Paper
% lw =1.5;   %Line Width
% FT = 7; %Title Fonte Size
% sgT= 8; % subtitle plot title
% Faxis = 6+6; %Axis Fonte Size
% fl =5 ; % Legend Fonte Size
% Ms = 3 ; %Mark Size
% p_width = 300; %Plot Width
% p_height = 150; %Plot Heiht
% p_width_Paper = 240; %Plot Width
% p_height_Paper = 120; %Plot Heiht
%Plotting Parameter for Paper

%Plotting Parameter for Presentation
% lw =1.5;   %Line Width
% FT = 7; %Title Fonte Size
% sgT= 12; % subtitle plot title
% Faxis = 6+4; %Axis Fonte Size
% fl =7 ; % Legend Fonte Size
% Ms = 3 ; %Mark Size
% p_width = 300; %Plot Width
% p_height = 150; %Plot Heiht
% p_width_Paper = 400; %Plot Width
% p_height_Paper = 200; %Plot Heiht

% Plotting Parameter for DeBugging
lw =1.5;   %Line Width
FT = 7; %Title Fonte Size
sgT= 15; % subtitle plot title
Faxis = 15; %Axis Fonte Size
fl =10 ; % Legend Fonte Size
Ms = 3 ; %Mark Size
p_width_Paper = 600; %Plot Width
p_height_Paper = 300; %Plot Heiht

sim_st = 1;
sim_end = 14;


%%%Need to change total plot
% function Tracking_graph

% figure(1)
% for i = 1:1:4
%     subplot(2,2,i);
%     plot(t,sus_pos_ref{i},'b-','LineWidth', lw);
%     hold on
%     plot(t,sus_pos{i},'r-','LineWidth',lw);
%     grid on;
%     legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
%     ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% end
% sgtitle('Sus Joint Position Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');


figure(2)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,drive_torque{i},'k-','LineWidth', lw);
    hold on
    plot(t,dri_pid_ctrl{i}*40,'r-','LineWidth', lw);
    hold on
    plot(t,soil_comp_ctrl{2}*40,'b-','LineWidth', lw);
%     plot(t,sus_torque{i},'b-','LineWidth', lw);
%     hold on;
%     plot(t,steer_torque{i},'g-','LineWidth', lw);
%     hold on;
%     plot(t,drive_torque{i},'r-','LineWidth', lw);
    grid on;
    xlim([sim_st sim_end]);
    legend('Total','FB','FF','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\tau$ (Nm)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Driving Motor Control Input ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(4)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,drive_vel_ref{i},'b-','LineWidth', lw);
    hold on
    plot(t,drive_vel{i},'r-','LineWidth',lw);
    grid on;
    xlim([sim_st sim_end]);
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Drive Joint Velocity Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(5)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,rad2deg(steer_pos_ref{i}),'k-','LineWidth', lw);
    hold on
    plot(t,rad2deg(steer_pos{i}),'r-','LineWidth',lw);
    grid on;
    xlim([sim_st sim_end]);
%     legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Steer Joint Position Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(6)
plot(t,Trunk_x_vel_ref,'-k','LineWidth', lw);
hold on
plot(t,Trunk_x_vel,'r-','LineWidth', lw);
grid on;
ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlim([sim_st sim_end]);
ylabel('Velocity (m/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Trunk Velocity','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(7)
plot(t,Trunk_y_vel_ref,'-k','LineWidth', lw);
hold on
plot(t,Trunk_y_vel,'r-','LineWidth', lw);
grid on;
ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlim([sim_st sim_end]);
ylabel('Velocity (m/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Trunk y Velocity','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');




% subplot(3,1,2);
% plot(t,Trunk_y_vel,'r-','LineWidth', lw);
% grid on;
% ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% 
% subplot(3,1,3); 
% plot(t,Trunk_z_vel,'r-','LineWidth', lw);
% grid on;
% ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% sgtitle('Trunk Velocity','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

% figure(16)
% subplot(3,1,1);
% plot(t,Trunk_x_ang_vel,'r-','LineWidth', lw);
% grid on;
% ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% 
% subplot(3,1,2);
% plot(t,Trunk_y_ang_vel,'r-','LineWidth', lw);
% grid on;
% ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% 
% subplot(3,1,3);
% plot(t,Trunk_z_ang_vel,'r-','LineWidth', lw);
% grid on;
% ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
% sgtitle('Trunk Angular Velocity','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(8)
plot(-(Trunk_y_pos-y_offset),(Trunk_x_pos-x_offset),'r','LineWidth',lw*2);
grid on;
xlim([-1 1]);
ylabel('y (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('x (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Trunk Map','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

% figure(8)
% plot(t,Trunk_x_vel,'r-','LineWidth', lw);
% hold on;
% plot(t,drive_vel{1}*0.25,'k-','LineWidth', lw);
% grid on;
% ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
% legend('Trunk','Motor_vel*radius','z dir','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
% title('Trunk_x_vel','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex')

figure(9)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,slip_ref{i},'k-','LineWidth', lw);
    hold on
    plot(t,slip_ratio{i},'r-','LineWidth', lw);
    grid on;
    xlim([sim_st sim_end]);
    ylim([-1 1]);
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('%(percent)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블

end
sgtitle('Slip Ratio','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(10)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,grf_x{i},'k-','LineWidth', lw);
    hold on
    plot(t,grf_y{i},'r-','LineWidth', lw);
    hold on
    plot(t,grf_z{i},'b-','LineWidth', lw);
    grid on;
    xlim([sim_st sim_end]);
    ylabel('Force (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
end
legend('x dir','y dir','z dir','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
sgtitle('Ground Reaction Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(11)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,steer_torque{i},'k-','LineWidth', lw);
    hold on;
    plot(t, steer_pid_ctrl{i},'r-','LineWidth', lw )
    hold on;
    plot(t, steer_ff_ctrl{i},'b-','LineWidth', lw )
    hold on;
    plot(t,steer_dob_ctrl{i},'m-','LineWidth', lw)
    grid on;
    xlim([sim_st sim_end]);
%     legend('Total','FB','FF','dhat','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\tau$ (Nm)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Steering Motor Control Input ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



figure(12)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,rad2deg(alpha{i}),'k-','LineWidth', lw);
    grid on;
    ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
    
end
legend('alpha','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
sgtitle('Tire Side Slip Angle','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(13)
plot(t,Trunk_yaw_rate_ref,'-k','LineWidth', lw);
hold on
plot(t,Trunk_z_ang_vel,'r-','LineWidth', lw);
grid on;
ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlim([sim_st sim_end]);
ylabel('$\gamma$ (rad/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Yaw rate Angle','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');


setFigurePositions(3,p_width_Paper,p_height_Paper);









