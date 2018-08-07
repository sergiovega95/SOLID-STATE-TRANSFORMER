clear all
%PARAMETROS RECTIFICADOR / ETAPA AC-DC

%Tensión de entrada al H-Bridge y Frecuencia de la red [V] [Rad/s]
Vfase_MV=13200/sqrt(3);
W=120*pi;

%Filtro inductivo a la entrada [H] [Ohm]
L_HB=50e-3;
R_HB=120*pi*L_HB*0.05;
C_HB=50000/(0.6*60*pi*(11397^2));
R_load=11397^2/(50e3/3);

%Frecuencia de conmutación y tiempo de simulación de la etapa[Hz] [s]
Fsw_HB=15e3;
Ts_CH=1/(90*Fsw_HB);
%Tensión de salida de cada HB y Número de niveles [V] [módulos]
Vout_HB=11397;
Nm=1;
E=Nm*Vout_HB;

%Corriente nominal de cada HB [A]
Inom_HB=(50e3/(13200*sqrt(3)))*sqrt(2);

%Modulación
m=0.9;

%Saturación del controlador
uLmt_I=0.05;
uLmt_V=Inom_HB;

%Código para sintonizar el controlador del lazo de corriente
F_cutI=Fsw_HB/100; %%/80
F_lagI=F_cutI/2.5; %%/3
Fsw=Fsw_HB;
a=2*pi*F_cutI*1j;
H_infI=abs(1/(exp(-a*1/(2*Fsw))*E/(L_HB*a+R_HB)*(1+2*pi*F_lagI/a)));
s=tf('s');
Giddd=E/(L_HB*s+R_HB);
anguloI=angle(exp(-a*1/(2*Fsw)*E/(L_HB*a+R_HB)*(1+2*pi*F_lagI/a)));
TsmI=exp(-s*1/(2*Fsw));
HpiI=-H_infI*sign(anguloI)*(1+2*pi*F_lagI/s);
Kc_I=H_infI;
tauI_I=1/(2*pi*F_lagI);
H_current=Giddd*TsmI*HpiI/(1+Giddd*TsmI*HpiI);

%Código para sintonizar el controlador del lazo de tensión
F_cutV=Fsw_HB/15;
F_LagV=F_cutV/2.5;
b=2*pi*F_cutV*1j;
H_infV=abs(1/(exp(-b*1/(2*Fsw))*(m*(b*L_HB+R_HB)/(3*b*C_HB*(b*L_HB+R_HB)))*(1+2*pi*F_LagV/b)));
anguloV=angle(exp(-b*1/(2*Fsw))*(m*(b*L_HB+R_HB)/(3*b*C_HB*(b*L_HB+R_HB)))*(1+2*pi*F_LagV/b));
Gvid=m*(s*L_HB+R_HB)/(3*s*C_HB*(s*L_HB+R_HB));
TsmV=exp(-s*1/(2*Fsw));
HpiV=-H_infV*sign(anguloV)*(1+2*pi*F_LagV/s);

% hold on
% options = bodeoptions;
% options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.
% bode(Gvid*TsmV,options);
% margin(Gvid*TsmV*HpiV)
Kc_V=H_infV;
tauI_V=1/(2*pi*F_LagV);
% hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Parametros del DAB/ ETAPA DC-DC

%Potencia nominal del SST [W]
Pnom=50e3;
Wred=120*pi;

%Tensión de fase en el lado de alta del SST [V]
Vfase_MV=13200/sqrt(3);
Vfase_LV=220/sqrt(3);

%Valores nominales en alta y baja tensión [V]
VDAB1=11397;
Vmax_DAB1=(80/105)*15000;
Vmin_DAB1=(100/95)*sqrt(2)*Vfase_MV;
VDAB2=393;
Vmin_DAB2=sqrt(6)*(1.1/0.95)*Vfase_LV;
Vmax_DAB2=Vmin_DAB2*1.1;

%Relación de transformación del transformador
Ntr=VDAB1/VDAB2;

%Corriente nominal del DAB
Inom=(50e3/3)/(220*sqrt(3))*sqrt(2);

%Frecuencia de portadora 
Fsw_DAB=30e3;
D=0.25;
Ts=1/(200*Fsw_DAB); %%% PERIODO DE SIMULACION DEL SISTEMA
Fs=1/Ts;
F_DAB=Fsw_DAB;
T_DAB=1/Fsw_DAB;

%Inductancia y resistencia del transformador [H] [Ohm] REFERIDOS A BAJA
L_DAB=(1/Ntr)*VDAB1*VDAB2*D*(1-D)/(2*Fsw_DAB*(Pnom/3));
R_DAB=30e3*2*pi*L_DAB*0.1;

%Capacitancias en baja y alta [F]
CDAB1=50*(Pnom/3)/(Vmin_DAB1^2*Fsw_DAB);
CDAB2=50*(Pnom/3)/(Vmin_DAB2^2*Fsw_DAB);

%Resistencia de carga en el lado de baja [Ohm]
R_LOAD2=VDAB2^2/(Pnom/3);

%Sintonizacion del controlador lazo de corriente
F_cutI_DAB=Fsw_DAB/100;
F_lagI_DAB=F_cutI_DAB/5;
sI_DAB=2*pi*F_cutI_DAB*1j;

Tsm_I_DAB=exp(-sI_DAB*1/(2*Fsw_DAB));
Gid_I_DAB=2*VDAB2/(sI_DAB*L_DAB+R_DAB);

H_infI_DAB=abs(1/(Tsm_I_DAB*Gid_I_DAB*(1+2*pi*F_lagI_DAB/sI_DAB)));
Ang_I_DAB=angle(Tsm_I_DAB*Gid_I_DAB*(1+2*pi*F_lagI_DAB/sI_DAB));

s=tf('s');
Gid_DAB=2*VDAB2/((s*L_DAB+R_DAB));
Tsm_DAB=exp(-s*1/(2*Fsw_DAB));
Hpi_DAB_I=-sign(Ang_I_DAB)*H_infI_DAB*(1+2*pi*F_lagI_DAB/s);

% hold on
% options = bodeoptions;
% options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.
% bode(Tsm_DAB*Gid_DAB,options);
% margin(Tsm_DAB*Gid_DAB*Hpi_DAB_I);
% hold off
KcI_DAB=H_infI_DAB;
tauII_DAB=1/(2*pi*F_lagI_DAB);


% KcI_DAB=0.001678696079423;
% tauII_DAB=1/29.031454349120892; %Multiplicar por 50 en la simulación
% HpiI_DAB=KcI_DAB+(1/tauII_DAB)*1/s;
% H1=Gid_DAB*Tsm_DAB*HpiI_DAB/(1+Gid_DAB*Tsm_DAB*HpiI_DAB);

%Sintonizacion del controlador lazo de tensión
F_cutV_DAB=Fsw_DAB/10;
F_lagV_DAB=F_cutV_DAB/5000;
sV_DAB=2*pi*F_cutV_DAB*1j;

Tsm_V_DAB=exp(-sV_DAB*1/(2*Fsw_DAB));
Gvi_V_DAB=R_LOAD2*(1-2*D)/(sV_DAB*R_LOAD2*CDAB2+1);
H_openL_current=-sign(Ang_I_DAB)*H_infI_DAB*(1+2*pi*F_lagI_DAB/sV_DAB)*Tsm_V_DAB*(2*VDAB2/(sV_DAB*L_DAB+R_DAB));

H_infV_DAB=abs(1/(Gvi_V_DAB*(1+2*pi*F_lagV_DAB/sV_DAB)*H_openL_current/(1+H_openL_current)));
Ang_V_DAB=angle(Tsm_V_DAB*Gvi_V_DAB*(1+2*pi*F_lagV_DAB/sV_DAB));

Gvi_DAB=R_LOAD2*(-2*D+1)/(s*R_LOAD2*CDAB2+1);
Hpi_DAB_V=-sign(Ang_V_DAB)*H_infV_DAB*(1+2*pi*F_lagV_DAB/s);
H_current=Tsm_DAB*Gid_DAB*Hpi_DAB_I/(1+Tsm_DAB*Gid_DAB*Hpi_DAB_I);


% figure
% hold on
% options = bodeoptions;
% options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.
% bode(Gvi_DAB*H_current,options);
% margin(Gvi_DAB*H_current*Hpi_DAB_V);
% hold off
KcV_DAB=H_infV_DAB;
tauIV_DAB=1/(2*pi*F_lagV_DAB);
uLmt_dDAB=0.5;
uLmt_IDAB=1e3*VDAB2/R_LOAD2;

%%%%%%%%%%%%%%%%%%%%%%%%%

%%%PARAMETROS DEL INVERSOR NPC / ETAPA DC-AC

%Tension de salida DAB /Entrada del NPC
Entrada=393;

%Tension de unos de los buses de continua
Vp=Entrada/2;

%Potencia nominal del trafo
Potencia_nominal=50e3;

%Carga ---- Carga nominal 0.968 ohms
R=0.968;

%Voltaje pico senoidal de salida
Voltaje_pico=(220*sqrt(2))/sqrt(3);

%Amplitud de la moduladora para pruebas en lazo abierto
m=(2*Voltaje_pico)/Entrada;

% frecuencia de la portadora o de trabajo del NPC
Fsw_NPC=5040;

% frecuencia de la red
w=120*pi;

% Capacitor DC link
C_3P4L=(Potencia_nominal)/(2*pi*60*0.1*((Entrada)^2));

%Filtro LC

%Frecuencia de corte del Filtro
corte=Fsw_NPC/5;
%Corriente Maxima por fase
Imax=(50e3*sqrt(3))/(220*3);
%Inductacia del Filtro
L=((0.03*Entrada))/((2*pi*60)*(Imax));
%Capacitancia del Filtro
a=(2*pi*corte)^2
ca=1/(L*a)

%%Funcion de transferencia del controlador
num=[0,Entrada];
dem=[0,2];
planta_V_d=tf(num,dem); 

%Constantes del PID , Obtenidas en matlab con la herramienta PID TUNNER
s=tf('s');
Kp=0.00138;
Ki=2.5641;

capacitor=(2*Imax)/(0.01*Entrada*Fsw_NPC)

red=1/60;
