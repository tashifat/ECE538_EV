%%
%This script includes the solutions to question 2 and 3. 
%%

close all;clear all; clc;
ftp_data = readtable("D:\WINTER_2022\ECE538\HW01\ftpcol.txt");

t = ftp_data.TestTime_Secs;
v_mph = ftp_data.TargetSpeed_Mph;
v_kmh = v_mph.*1.60934;
v_ms = v_kmh./3.6;


%%
Prrated = 110e3; %max rotor power in W
Trrated = 320; %max rotor torque in Nm
vmax = 150/3.6; % max speed in m/s
r = 0.316; %wheel radius
ng = 8.19; %gear ratio
m = 1757.67; %vehicle mass in kg

w_axle = v_ms./r;
w_shaft = w_axle*ng;
w_shaft_rpm = (60/(2*pi))*(w_shaft);

waxlemax = vmax/r; %max axle angular speed
wrmax = waxlemax*ng; %max rotor angular speed
Nrmax = wrmax*60/(2*pi); % Maximum speed in rpm
wrrated = Prrated/Trrated; % Rated speed (rad/s)
Nrrated = wrrated/2/pi*60 % Rated speed (Rpm)
p = 8; % Number of poles
k = 0.3; % machine constant (Nm/A)
Rs = 0.02; % stator resistance (ohm)
Ls = 0.2*10^-3; % phase inductance (H)
Iphm = 2*k/p/Ls % Equivalent magnetizing current
Tnl = 1; % no-load torque


A = 25.890*4.448; %coastdown parameter A in N
B = 0.34490*9.950; %coastdown parameter B in N/(m/s)
C = 0.019450*22.26; %coastdown parameter C in N/((m/s)^2)
Effg = 0.97; %Assumed gear efficiency
J = 3; %Assumed axle-reference MOI
%%
dv = gradient(v_ms); %finding the change in speed
dt=0.01;
acc_ms = dv./dt; %calculating acceleration
F_axle = A + B.*v_ms + C.*v_ms.^2;
T_axle = F_axle.*r + J*(gradient(w_axle)./dt);

%Below part is for the Motor Shaft Torque calculation for acceleration,
%defeleration and standalone position of vehicle.
T_shaft = zeros(length(dv),1);
for i=1:length(dv)
    if(dv(i) > 0)
        T_shaft(i) = T_axle(i)/(ng*Effg);
    elseif (dv(i) < 0)
        T_shaft(i) = (T_axle(i)*Effg)/ng;
    elseif (dv(i) == 0)
        T_shaft(i) = 0;
    end
end


figure(1)
plot(t,w_shaft_rpm, 'b-', 'linewidth', 2,'color','#c9321e')
set(gca,'fontname','times', 'FontSize',13)
xlabel('Time (s)', 'fontname','times', 'FontSize',14);
ylabel('Motor Speed (RPM)', 'fontname','times', 'FontSize',14);
title('Motor Speed vs Time over an FTP Drive Cycle','fontname','times', 'FontSize',18);
xlim([0 length(t)])
exportgraphics(gcf,'rpm_vs_time.png','Resolution',600)

figure(2)
plot(t,T_shaft, 'linewidth', 1.5,'color','#D95319')
set(gca,'fontname','times', 'FontSize',13)
xlabel('Time (s)', 'fontname','times', 'FontSize',14);
ylabel('Torque (Nm)', 'fontname','times', 'FontSize',14);
title('Motor Torque vs Time over an FTP Drive Cycle','fontname','times', 'FontSize',18);
xlim([0 length(t)])
exportgraphics(gcf,'torque_vs_time.png','Resolution',600)
%%
tr =[1:1:Trrated]; % list of torque values in increments of 1Nm
Nr =[1:100:Nrmax]; % speed values in increment 100rpm
[X,Y] =meshgrid(Nr,tr); 
Pr =X*pi/30.*Y; 
Pr(Pr>Prrated)=0;
Iphd =-Iphm*(1-Nrrated./X); 
Iphd(Nr>Nrrated)=0;
Pin=Pr+3*Rs*((Y./k/3).^2)+(Tnl.*X.*(pi/30))+3*Rs*((Iphd).^2);
Eff=Pr./Pin; 
Tlim=Prrated./(Nr.*(pi/30)); 
Tlim(Tlim>Trrated)=Trrated;
%%
%finding efficiency for different speed-torque combination.
EFF_1875 = zeros(1875,1);
EFF_1875_2 = zeros(1875,1);
for i = 1:1875
    a(i) = round(abs(T_shaft(i)))+1; %finding the nearest torque index position
    b(i) = round(abs(w_shaft_rpm(i)/100))+1;%finding the nearest speed index position
    EFF_1875(i) = Eff(a(i),b(i)); %from the torque-speed index position, finding the efficiency.
    EFF_1875_2(i) = Eff(a(i),b(i));
end
EFF_1875(EFF_1875<0.1) = 1; % eliminating zeros and close to zero efficiencies to avoid sudden peaks in the input power.

%%
Pout_Motor = (T_shaft.*w_shaft)/1000;
Pin_Motor = Pout_Motor./EFF_1875;
figure(3)
yyaxis right
plot(t, EFF_1875_2*100, 'linestyle', '-','linewidth', 1,'color','#808080')
xlim([0 length(t)])
ylabel('Efficiency (%)', 'fontname','times', 'FontSize',14);
ytickformat('percentage')
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = '#808080';

yyaxis left
plot(t,Pout_Motor,'linewidth', 2,'color','k')
hold on
plot(t,Pin_Motor,'linestyle', '--', 'linewidth', 1.5,'color','#35cd00')
xlim([0 length(t)])
set(gca,'fontname','times', 'FontSize',13)
xlabel('Time (s)', 'fontname','times', 'FontSize',14);
ylabel('Power (kW)', 'fontname','times', 'FontSize',14);
title('Motor Power vs Time with Instantenous Efficiency','fontname','times', 'FontSize',18);
legend('Output Power', 'Input Power', 'Efficiency','NumColumns',3)
exportgraphics(gcf,'POWER.png','Resolution',600)
%%
Ein_Motor = cumtrapz(Pin_Motor); %integrating input power over time
Eout_Motor = cumtrapz(Pout_Motor); %integrating output power over time

figure(4)
plot(t,Ein_Motor, 'linewidth', 1.5)
hold on
plot(t,Eout_Motor, 'linewidth', 1.5,'color','#D95319')
set(gca,'fontname','times', 'FontSize',13)
xlabel('Time (s)', 'fontname','times', 'FontSize',14);
ylabel('Energy (kJ)', 'fontname','times', 'FontSize',14);
title('Motor Energy vs Time','fontname','times', 'FontSize',18);
xlim([0 length(t)])
legend('Input Energy', 'Output Energy', 'location', 'northwest')
exportgraphics(gcf,'energy.png','Resolution',600)