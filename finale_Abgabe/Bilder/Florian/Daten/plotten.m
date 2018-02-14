
%% Plotten Theta_x und Theta_x_dot
clear all; 
clc; 

A = load('Winkel_Rauschen.txt');

figure();

t1=A(:,1).*1e-6;
theta_x = A(:,2);
theta_y = A(:,3);
theta_x_dot = A(:,4);
theta_y_dot = A(:,5);

%Plot Theta_x
subplot(2,1,1);
plot(t1,theta_x,'linewidth',1.2);
xlim([0.015 0.025]);
set(gca,'xtick',0.015:0.0025:0.025);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')), '.', ','));
%set(gca,'xtick',[0,1]);
set(gca,'XTickLabel',{});
set(gca,'ytick',-3:1:1);
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')), '.', ','))
ylim([-3 1]);
ylabel('$\vartheta_{x}$ [$^{\circ}$]','Interpreter','latex');
grid on;

%Plot Theta_x_dot
subplot(2,1,2);
plot(t1,theta_x_dot, 'linewidth',1.2);
xlim([0.015 0.025]);
ylim([-1 1]);
set(gca,'ytick',-0.5:.5:1);
set(gca,'xtick',0.015:0.0025:0.025);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')), '.', ','));
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')), '.', ','))
xlabel('t [$s$]','Interpreter','latex');
ylabel('$\dot{\vartheta_{x}}$ [$\frac{^{\circ}}{s}$]','Interpreter','latex');
grid on;


%% Rauschen Simulink
clear all; 
clc; 
A = load('Winkel_Rauschen.txt');
B = load('Theta_0_deg_with_Noise.txt');
C = load('Real_Torques_0_deg_with_Noise.txt');

figure();

t1=A(:,1).*1e-6;
t2=B(:,1);
t3=C(:,1);

theta_y = A(:,3);
theta_x_sim = B(:,2).*(360/(2*pi)); 
T1_sim = C(:,2);

%Plot Theta_x_sim
subplot(2,1,1);
plot(t2,theta_x_sim, 'linewidth',1.2);
xlim([0 4]);
set(gca,'xtick',0:1:4);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')), '.', ','));
%set(gca,'xtick',[0,1]);
set(gca,'XTickLabel',{});
set(gca,'ytick',-1:0.5:1);
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')), '.', ','))
ylim([-1 1]);
ylabel('$\vartheta_{x}$ [$^{\circ}$]','Interpreter','latex');
grid on;

%Plot T1_sim
subplot(2,1,2);
plot(t3,T1_sim,'r','linewidth',1.2);
xlim([0 4]);
ylim([-.1 .1]);
%set(gca,'ytick',-.05:.01:0.05);
set(gca,'xtick',0:1:4);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')),'.', ','));
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')),'.', ','))
xlabel('t [$s$]','Interpreter','latex');
ylabel('$T_{1}$ [$Nm$]','Interpreter','latex');
grid on;

%% Rauschen, Abtastung, Simulink
clear all; 
clc; 

B = load('Theta_10_deg_xy_with_Noise_Abtastung_DELAY.txt');
C = load('Real_Torques_10_deg_xz_with_Noise_Abtastung_DELAY.txt');

figure();

t2=B(:,1);
t3=C(:,1);

theta_x_sim = B(:,2).*(360/(2*pi)); 
T1_sim = C(:,2);

%Plot Theta_x_sim
subplot(2,1,1);
plot(t2,theta_x_sim, 'linewidth',1.2);
xlim([2 3]);
ylim([-.9 .1]);
set(gca,'xtick',2:0.25:3);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')), '.', ','));
%set(gca,'xtick',[0,1]);
set(gca,'XTickLabel',{});
%set(gca,'ytick',-0.05:0.025:0.05);
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')), '.', ','))
ylabel('$\vartheta_{x}$ [$^{\circ}$]','Interpreter','latex');
grid on;

%Plot T1_sim
subplot(2,1,2);
plot(t3,T1_sim,'r','linewidth',1.2);
xlim([2 3]);
ylim([-.05 .05]);
set(gca,'ytick',-.04:.02:0.05);
set(gca,'xtick',2:0.25:3);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')), '.', ','));
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')), '.', ','))
xlabel('t [$s$]','Interpreter','latex');
ylabel('$T_{1}$ [$Nm$]','Interpreter','latex');
grid on;



%%Plotten Filter über Winkel und Drehmomente

clear all; 
clc; 

A= load('State_Vals_FilteredUnfiltered_06_02.txt');
B = load('Real_Torques_10_deg_xz_with_Noise_Abtastung_DELAY.txt');

figure();

t1=A(:,1);
%t3=C(:,1);

theta_x = A(:,2);
theta_x_filtered = A(:,6);


%Plot Theta_x_sim
subplot(2,1,1);
plot(t1,theta_x,'b','linewidth',1.2)
hold on; 
plot(t1,theta_x_filtered,'r','linewidth',1.2);
xlim([31 33]);
ylim([-1 0]);
set(gca,'xtick',31:0.5:333);
set(gca, 'XTickLabel', strrep(cellstr(get(gca, 'XTickLabel')), '.', ','));
%set(gca,'xtick',[0,1]);
%set(gca,'XTickLabel',{});
%set(gca,'ytick',-0.05:0.025:0.05);
set(gca, 'YTickLabel', strrep(cellstr(get(gca, 'YTickLabel')), '.', ','))
ylabel('$\vartheta_{x}$ [$^{\circ}$]','Interpreter','latex');
grid on;




















% %Virtuelle Drehmomente Plotten
% figure(); 
% t=Virtual_Torques_FULL.time; 
% 
% %Plot T1
% subplot(3,1,1);
% plot(t,Virtual_Torques_FULL.signals(1).values);
% title('T_{x}');
% xlabel('t in s');
% ylabel('M in Nm');
% grid on;
% 
% %Plot T2
% subplot(3,1,2); 
% plot(t,Virtual_Torques_FULL.signals(2).values);
% title('T_{y}');
% xlabel('t in s');
% ylabel('M in Nm');
% grid on;
% 
% %Plot T3
% subplot(3,1,3); 
% plot(t,Virtual_Torques_FULL.signals(3).values);
% title('T_{z}');
% xlabel('t in s');
% ylabel('M in Nm');
% grid on;
% 
% suptitle('Virtuelle Drehmomente mit Rauschen');
% 
% %Angles
% figure(); 
% t=Phi_Struct.time; 
% 
% %Plot Theta_x
% t=Theta_Struct.time;
% subplot(4,1,1); 
% plot(t,Theta_Struct.signals(1).values);
% title('Winkel $$\theta_{x}$$','Interpreter','latex');
% xlabel('t in s');
% ylabel('$$\theta_{x}$$ in rad','Interpreter','latex');
% grid on;
% 
% %Plot Theta_x
% subplot(4,1,2); 
% plot(t,Theta_Struct.signals(2).values);
% title('Winkelgeschwindigkeit $$\dot{\theta_{x}}$$','Interpreter','latex');
% xlabel('t in s');
% ylabel('$$\dot{\theta_{x}}$$ in rad/s','Interpreter','latex');
% grid on;
% 
% %Plot Theta_y
% t=Theta_Struct.time;
% subplot(4,1,3); 
% plot(t,Theta_Struct.signals(3).values);
% title('Winkel $$\theta_{y}$$','Interpreter','latex');
% xlabel('t in s');
% ylabel('$$\theta_{y}$$ in rad','Interpreter','latex');
% grid on;
% 
% %Plot Theta_y
% subplot(4,1,4); 
% plot(t,Theta_Struct.signals(4).values);
% title('Winkelgeschwindigkeit $$\dot{\theta_{y}}$$','Interpreter','latex');
% xlabel('t in s');
% ylabel('$$\dot{\theta_{y}}$$ in rad/s','Interpreter','latex');
% grid on;
% 
% 
% suptitle('Winkel und Winkelgeschwindigkeiten mit Rauschen ');
% %suptitle(sprintf('Reale Winkel- und Winkelgeschwindigkeiten mit Rauschen;\n $$y_{0} = \frac{\PI}{32}$$;\n K=[-0.0316  -14.4092   -0.2409   -3.4304]'));

