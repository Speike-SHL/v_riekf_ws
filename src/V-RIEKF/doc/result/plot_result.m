clear;clc;close all;
% 读取文件数据
ground_truth = load('ground_truth.txt');
riekf_esti = load('riekf_esti.txt');

% 线条颜色、线宽等参数 (请根据需要修改)
truth_color = '--k'; % 蓝色
esti_color = 'r'; % 红色
true_line_width = 2;
esti_line_width = 1.5;

% 提取时间、位姿、速度和位置数据
t = ground_truth(:, 1);
true_quat = ground_truth(:, 2:5);
true_vel = ground_truth(:, 6:8);
true_pos = ground_truth(:, 9:11);

esti_quat = riekf_esti(:, 2:5);
esti_vel = riekf_esti(:, 6:8);
esti_pos = riekf_esti(:, 9:11);

% 计算欧拉角 (roll, pitch, yaw),拼凑出来的,应该有更好方法
[~, ~, true_eul_yaw] = quat2angle(true_quat,"ZYX");
[~, ~, esti_eul_yaw] = quat2angle(esti_quat,"ZYX");
[true_eul_pitch, ~, ~] = quat2angle(true_quat,"YXZ");
[esti_eul_pitch, ~, ~] = quat2angle(esti_quat,"YXZ");
[~, true_eul_roll, ~] = quat2angle(true_quat,"XZY");
[~, esti_eul_roll, ~] = quat2angle(esti_quat,"XZY");
true_eul_yaw = true_eul_yaw * 180 / pi;
true_eul_pitch = true_eul_pitch * 180 / pi;
true_eul_roll = true_eul_roll * 180 / pi;
esti_eul_yaw = esti_eul_yaw * 180 / pi;
esti_eul_pitch = esti_eul_pitch * 180 / pi;
esti_eul_roll = esti_eul_roll * 180 / pi;

% 计算轨迹误差
trajectory_error = sqrt(sum((true_pos - esti_pos).^2, 2));

% 第一个图：三维轨迹误差, 一会在导出设置中设置所有字体为15磅
% figure('Position',[0,0,1280,960]);
figure;
plot3(true_pos(:, 1), true_pos(:, 2), true_pos(:, 3), truth_color, 'LineWidth', true_line_width);
hold on; grid on;
esti_pos(end, 1) = NaN; esti_pos(end, 2) = NaN; esti_pos(end, 3) = NaN;
% plot3(esti_pos(:, 1), esti_pos(:, 2), esti_pos(:, 3), esti_color, 'LineWidth', line_width);
patch(esti_pos(:, 1), esti_pos(:, 2), esti_pos(:, 3), trajectory_error, 'EdgeColor','interp','linewidth', esti_line_width);
colormap cool;
c = colorbar('Ticks',[0.0017,0.1549,0.3462],'TickLabels',[0.0017,0.1549,0.3462]); c.Label.String = 'APE';
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis([-10,10,-10,10,-2,8]);
legend('Ground Truth', 'RIEKF Estimate');
set(gca, fontsize=19);

% 第二个图：姿态误差
figure('Position',[0,0,1280,960]);
subplot(3, 1, 1);
plot(t, true_eul_roll-esti_eul_roll, esti_color, 'LineWidth', esti_line_width);
grid on;
ylabel('$$\phi$$ (deg)','Interpreter','latex');   %roll
title('Euler angles Error'); set(gca, fontsize=19);

subplot(3, 1, 2);
plot(t, true_eul_pitch-esti_eul_pitch, esti_color, 'LineWidth', esti_line_width);
grid on; ylim([-1.5 1.5]); yticks([-1.5 -0.75 0 0.75 1.5])
ylabel('$$\theta$$ (deg)','Interpreter','latex'); %pitch
set(gca, fontsize=19);

subplot(3, 1, 3);
plot(t, true_eul_yaw-esti_eul_yaw, esti_color, 'LineWidth', esti_line_width);
grid on;
xlabel('$$t$$ (s)','Interpreter','latex'); ylabel('$$\psi$$ (deg)','Interpreter','latex'); %yaw
set(gca, fontsize=19);


% 第三个图：速度误差
figure('Position',[0,0,1280,960]);
% legend('Ground Truth', 'RIEKF Estimate');
subplot(3, 1, 1);
plot(t, true_vel(:, 1)-esti_vel(:, 1), esti_color, 'LineWidth', esti_line_width);
grid on;
ylabel('$$v_x$$ (m/s)','Interpreter','latex');
title('Velocity Error'); set(gca, fontsize=19);

subplot(3, 1, 2);
plot(t, true_vel(:, 2)-esti_vel(:, 2), esti_color, 'LineWidth', esti_line_width);
grid on;
ylabel('$$v_y$$ (m/s)','Interpreter','latex');
set(gca, fontsize=19);

subplot(3, 1, 3);
plot(t, true_vel(:, 3)-esti_vel(:, 3), esti_color, 'LineWidth', esti_line_width);
grid on; ylim([-0.08 0.08]); yticks([-0.08 -0.04 0 0.04 0.08]);
xlabel('$$t$$ (s)','Interpreter','latex'); ylabel('$$v_z$$ (m/s)','Interpreter','latex');
set(gca, fontsize=19);


% 第四个图：位置误差
figure('Position',[0,0,1280,960]);
% legend('Ground Truth', 'RIEKF Estimate');
subplot(3, 1, 1);
plot(t, true_pos(:, 1)-esti_pos(:, 1), esti_color, 'LineWidth', esti_line_width);
grid on; axis([0,60,-0.1,0.3])
ylabel('$$p_x$$ (m)','Interpreter','latex');
title('Position Error'); set(gca, fontsize=19);

subplot(3, 1, 2);
plot(t, true_pos(:, 2)-esti_pos(:, 2), esti_color, 'LineWidth', esti_line_width);
grid on; ylim([-0.3 0.3]); yticks([-0.3 -0.15 0 0.15 0.3]);
ylabel('$$p_y$$ (m)','Interpreter','latex');
set(gca, fontsize=19);

subplot(3, 1, 3);
plot(t, true_pos(:, 3)-esti_pos(:, 3), esti_color, 'LineWidth', esti_line_width);
grid on; ylim([-0.08 0.08]); yticks([-0.08 -0.04 0 0.04 0.08])
xlabel('$$t$$ (s)','Interpreter','latex'); ylabel('$$p_z$$ (m)','Interpreter','latex');
set(gca, fontsize=19);



