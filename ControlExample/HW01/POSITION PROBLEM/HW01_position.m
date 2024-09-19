clear all
clc

% This file considers the position control problem
% to stop the motor at a given angular position
% Student Name: Giuditta Sigona
% Created on 21/03/2022
% Last version xx/yy/zz


% matlab colors
NMatlabRed      = [0.8500   0.3250   0.0980];
NMatlabYellow   = [0.929    0.694    0.125 ];
NMatlabBlue     = [0        0.4470   0.7410];  
NMatlabViolet   = [0.4940   0.1840   0.5560];
NMatlabGreen    = [0.4660   0.6740   0.1880];
NMatlabCyan     = [0.3010   0.7450   0.9330];
NMatlabBordeaux = [0.6350   0.0780   0.1840]; 

s = tf('s');
om = logspace(-2,5,1000);
t = 0:0.001:10;

% Parameters (matlab example)
R = 2.0;                % Ohms
L = 0.5;                % Henrys
K = 0.1;               % torque constant and back emf constant
b = 0.2;               % Nms
J = 0.02;               % kg.m^2/s^2

h1 = tf(Km,[L R]);            % armature
h2 = tf(1,[J Kf]);            % eqn of motion

dcm = ss(h2) * [h1 , 1];      % w = h2 * (h1*Va + Td)
dcm = feedback(dcm,Kb,1,1);   % close back emf loop

Tl = 0;   % disturbance T_load torque initially = 0

P1 = tf(K,[L R]);            % armature
P2 = tf(1,[J b]);            % eqn of motion

P1.u = 'v';
P1.y = 'T';
% P2.u = 'Td';
P2.u = 'T';
P2.y = 'ω';

% Sum1 = sumblk('e = v-ω');
% Sum2 = sumblk('Td = T + Tl');

Pe = connect(P1,P2,{'v'},{'ω'})       % open loop 
Cont = tf(0.5); % cont = K
Plant1 = feedback(Pe, Cont);   % close back emf loop
tf(Plant1)

%adding the integrator to get the position

integrator = tf(1/s);
Plant2 = series(Plant1, integrator);
tf(Plant2)
SF0 = loopsens(Pe, Cont); 
 
% Cont2 = tf(1);
% SF0 = loopsens(Plant2, Cont2);  % all the sensitivities functions

[magS0,phaseS0]=bode(SF0.Si,om);
[magT0,phaseT0]=bode(SF0.Ti,om);
[magL0,phaseL0]=bode(SF0.Li,om);
%[magC0,phaseC0]=bode(Cont,om);

figure(1)
subplot(211)
Plot1 = semilogx(om,20*log10(magS0(1,:)),...
    om,20*log10(magL0(1,:)),...
    om,20*log10(magT0(1,:))); grid
set(Plot1,'LineWidth',2)
yline(0,'--')
yline(-3, '--')
legA = legend('S','L','T','Location','SouthEast');
set(legA,'FontSize',12)
title('Sensitivity and Complementary Sensitivity functions')
ylim([-60,30])
xlim([1.e-1,1.e+3])

subplot(212)
y1 = step(SF0.Ti,t);
Plot2 = plot(t,y1); grid
set(Plot2,'LineWidth',2);
legend('y1','FontName','courier','FontSize',12,'Location','SouthEast')
xlabel('$t$','Interpreter','latex','FontSize',14)
title('Step response','FontName','courier','FontSize',14)

% ----------  Minimizing the weighted sensitivity function -----------
% choose a weight function on the basis of the obtained sensitivity function 
% the optimal controller which minimize the min || . || is such that
% | wBS1(jω) S(jω)| = constant 

A1 = 7.9*(10^-3);    % low frequency behavior of specs of |S| (the value associated to S(0) )
wBS1 = 1.8;   % is the lower bound on B3S
M1 = 3;    % = 9.9 db = 3 is the upper bound for |S(jω)|, generally belongs to [1.4, 2]
              % peak of S
wS1 = (s/M1 +wBS1)/(s+wBS1*A1) %should be A.S. and M.P.


% wBT1 = ; % upper bound 
% Mt1 = ; %peak of T(jw)
% At = ;
% wT = (s+wBT1/Mt1)/(At*s + wBT1); 

%Controller design through mixsyn 
[C1,CL1,GAM1] = mixsyn(Plant2,wS1,[]);     %controller with only Ws
GAM1
SF1 = loopsens(Plant2,C1);
tf(C1)

%add a constant weight Wu
wu = 0.3;  % same as wT or a constant
[C2,CL2,GAM2] = mixsyn(Plant2,wS1,wu,[]);       %controller with Ws and Wu
GAM2    %constant gamma
SF1a = loopsens(Plant2,C2);
tf(C2)

[magwS1,phasewS1]=bode(wS1,om);          % weight
[magS1,phaseS1]=bode(SF1.Si,om);
[magT1,phaseT1]=bode(SF1.Ti,om);
[magL1,phaseL1]=bode(SF1.Li,om);
[magC1,phaseC1]=bode(C1,om);             % optimal controller
[magwSS1,phasewSS1]=bode(wS1*SF1.Si,om); % weighted sensitivity


figure(2)
subplot(211)
Plot3 = semilogx(om,20*log10(magS1(1,:)),...
    om,20*log10(magL1(1,:)),om,20*log10(magT1(1,:)),...
    om,20*log10(magC1(1,:)),om,-20*log10(magwS1(1,:)),'--'); grid
set(Plot3,'LineWidth',2)
set(Plot3(4),'Color',NMatlabGreen)
set(Plot3(5),'Color',NMatlabBordeaux)
yline(0,'--')
yline(-3,'--')
legC = legend('$|S(j\omega)|$','$|L(j\omega)|$','$|T(j\omega)|$','$|C(j\omega)|$','$|1/w_S(j\omega)|$',...
    'Interpreter','latex','Location','SouthEast');
set(legC,'FontSize',12)
title(['C1 with A= ' num2str(A1) '     wBS= ' num2str(wBS1) ...
           '     M= ' num2str(M1)]);
ylim([-60,50])
xlim([1.e-2,1.e+5])
subplot(212)
Plot4 = semilogx(om,20*log10(magS1(1,:)),om,20*log10(magwSS1(1,:)),...
                 om,-20*log10(magwS1(1,:)),'--'); grid
set(Plot4,'LineWidth',2)
leg4 = legend('$|S(j\omega)|$','$|w_S(j\omega)S(j\omega)|$','$|1/w_S(j\omega)|$','Location','SouthEast',...
        'Interpreter', 'latex');
set(leg4,'FontSize',12)

figure(3)
subplot(211)
y1 = step(SF1.Ti,t);
Plot2 = plot(t,y1); grid
set(Plot2,'LineWidth',2);
legend('y1','FontName','courier','FontSize',12,'Location','SouthEast')
xlabel('$t$','Interpreter','latex','FontSize',14)
title('Step response with H-inf','FontName','courier','FontSize',14)


figure(4)
subplot(211)
y1 = step(SF1a.Ti,t);
Plot2 = plot(t,y1); grid
set(Plot2,'LineWidth',2);
legend('y1','FontName','courier','FontSize',12,'Location','SouthEast')
xlabel('$t$','Interpreter','latex','FontSize',14)
title('Step response with H-inf','FontName','courier','FontSize',14)

%in the absence of particular limiting situations, the optimal controller
%is such that | wBS1(jω) S(jω)| = constant 

%----------------------------------------------------------------
%confronto con un PID

C2 = pid(0.5, 0.5,0);

SF2 = loopsens(Plant1,C2);

[magwS1,phasewS1]=bode(wS1,om);          % weight
[magS1,phaseS1]=bode(SF1.Si,om);
[magT1,phaseT1]=bode(SF1.Ti,om);
[magL1,phaseL1]=bode(SF1.Li,om);
[magC1,phaseC1]=bode(C2,om);             % optimal controller
[magwSS1,phasewSS1]=bode(wS1*SF1.Si,om); % weighted sensitivity


figure(5)
subplot(211)
Plot3 = semilogx(om,20*log10(magS1(1,:)),...
    om,20*log10(magL1(1,:)),om,20*log10(magT1(1,:)),...
    om,20*log10(magC1(1,:)),om,-20*log10(magwS1(1,:)),'--'); grid
set(Plot3,'LineWidth',2)
set(Plot3(4),'Color',NMatlabGreen)
set(Plot3(5),'Color',NMatlabBordeaux)
yline(0,'--')
yline(-3,'--')
legC = legend('$|S(j\omega)|$','$|L(j\omega)|$','$|T(j\omega)|$','$|C(j\omega)|$','$|1/w_S(j\omega)|$',...
    'Interpreter','latex','Location','SouthEast');
set(legC,'FontSize',12)
title(['Controller PID']);
ylim([-60,50])
xlim([1.e-2,1.e+5])
subplot(212)
Plot4 = semilogx(om,20*log10(magS1(1,:)),om,20*log10(magwSS1(1,:)),...
                 om,-20*log10(magwS1(1,:)),'--'); grid
set(Plot4,'LineWidth',2)
leg4 = legend('$|S(j\omega)|$','$|w_S(j\omega)S(j\omega)|$','$|1/w_S(j\omega)|$','Location','SouthEast',...
        'Interpreter', 'latex');
set(leg4,'FontSize',12)

figure(6)
subplot(211)
y1 = step(SF2.Ti,t);
Plot2 = plot(t,y1); grid
set(Plot2,'LineWidth',2);
legend('y1','FontName','courier','FontSize',12,'Location','SouthEast')
xlabel('$t$','Interpreter','latex','FontSize',14)
title('Step response','FontName','courier','FontSize',14)


