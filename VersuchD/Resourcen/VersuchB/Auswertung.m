
clc
clf

stellgroessensprung % stellgroessen und Stoergrössenscript laden
stoergroessensprung




%% Parameterbestimmung:

rY = t25Y/t75Y;
PY = -18.56075*rY + 0.57311/(rY- 0.20747) + 4.16423;
XY = 14.2797*rY^3 - 9.3891*rY^2 + 0.25437*rY + 1.32148;
T2Y = (t75Y-t25Y)/(XY*(1+1/PY));
T1Y = T2Y/PY;
TtY = T1Y*T2Y;
DY = (T1Y+T2Y)/(2*sqrt(TtY));
TDY = 2*sqrt(TtY)*DY;    %Formeln und werte aus dem VL-Script

% Uebetragungsfunktion zur ausgabe im Command Window als Char speichern und dann ausgeben 
tfGpYPa = sprintf('GpYPa = tf(%d,[%d %d 1]) (Parameterbestimmung)',KpY,TtY,TDY); 

disp(tfGpYPa)
GpYPa = tf(KpY,[TtY TDY 1]);

%selbe für die Stoergroesse
rZ = t25Z/t75Z;
PZ = -18.56075*rZ + 0.57311/(rZ- 0.20747) + 4.16423;
XZ = 14.2797*rZ^3 - 9.3891*rZ^2 + 0.25437*rZ + 1.32148;
T2Z = (t75Z-t25Z)/(XZ*(1+1/PZ));
T1Z = T2Z/PZ;
TTZ = T1Z*T2Z;
DZ = (T1Z+T2Z)/(2*sqrt(TTZ));
TDZ = 2*sqrt(TTZ)*DZ;

tfGpZPa = sprintf('GpZPa = tf(%d,[%d %d 1]) (Parameterbestimmung)',KpZ,TTZ,TDZ);

disp(tfGpZPa)
GpZPa = tf([KpZ],[TTZ,TDZ,1]);

%% Schwarze:       (Annahme: PT2)

%Formeln aus dem VL-Script
a10 = 1.88;
a50 = 0.596;
a90 = 0.257;

tYSwa = (a10*t10Y + a50*t50Y + a90*t90Y)/3;
TtYSwa = tYSwa * tYSwa;
TDYSwa = 2 * tYSwa;


tfGpYSwa = sprintf('GpYSwa = tf(%d,[%d %d 1]) (Schwarze)',KpY, TtYSwa,TDYSwa);
disp(tfGpYSwa)
GpYSwa = tf([KpY],[TtYSwa,TDYSwa,1])

TZSwa = (a10*t10Z + a50*t50Z + a90*t90Z)/3;
TTZSwa = TZSwa * TZSwa;
TDZSwa = 2 * TZSwa;

tfGpZSwa = sprintf('GpZSwa = tf(%d,[%d %d 1]) (Schwarze)',KpZ, TTZSwa,TDZSwa);
disp(tfGpZSwa)
GpZSwa = tf([KpZ],[TTZSwa TDZSwa 1])


%% plotten 

%Sprungantwort berechnen
GpYPaRes = step(GpYPa,tY);
GpYSwaRes = step(GpYSwa,tY);


% figure(2)
% hold on
% stairs(tY,GpYPaRes,"--","DisplayName","GpYPaRes")
% stairs(tY,GpYSwaRes,"--","DisplayName","GpYSwaRes")
% 
% figure(4)
% hold on

GpZPaRes = step(GpZPa ,tZ);
GpZSwaRes = step(GpZSwa,tZ);

% stairs(tZ,GpZPaRes,"--","DisplayName","GpZPaRes")
% stairs(tZ,GpZSwaRes,"--","DisplayName","GpZSwaRes")

%% Strejc : 

%Scripte laden
StrejcY
StrejcZ


%% die EndFigur

figure(9);hold on, grid on, legend show;

stairs(tY,GpYPaRes,"b-","DisplayName","GpYPaRes")
stairs(tY,GpYSwaRes,"r-","DisplayName","GpYSwaRes")
stairs(stellgroessen.time,GpYStrFinalRes,"g-","DisplayName","GpYStrRes")
yline(KpY,'b-','DisplayName','KpY')
stairs(stellgroessen.time,stellgroessen.stellgroesse,'c','DisplayName',"Stellgroesse")
stairs(stellgroessen.time,stellgroessen.regelgroesse,'y','DisplayName',"Regelgroesse")


% figure(10);hold on, grid on, legend show;
% 
% stairs(tZ,GpZPaRes,"b-","DisplayName","GpZPaRes")
% stairs(tZ,GpZSwaRes,"r-","DisplayName","GpZSwaRes")
% stairs(stoergroessen.time,GpZStrFinalRes,"g-","DisplayName","GpZStrRes")
% yline(KpZ,'b-','DisplayName','KpZ')
% plot(stoergroessen.time,stoergroessen.stoergroesse,'c','DisplayName',"Stoergroesse");
% plot(stoergroessen.time,stoergroessen.regelgroesse,'y','DisplayName',"Regelgroesse");




%% Totzeit Stellgroessensprung


%figure(7); 
% clf
% hold on;

% stairs(tY,GpYPaRes,"--","DisplayName","GpYPaRes");
% stairs(tY,stellgroessen.stellgroesse,'r',"DisplayName","Stellgroesse");
% stairs(tY,stellgroessen.regelgroesse,'b',"DisplayName","Regelgroesse");

s = tf('s'); % s variable ist s im Bildbereich



for i = 0:0.2:2

    totzeit = exp(-i * s); %Totzeitformel 

    GpYPaT = GpYPa * totzeit; % Übertragungsfunktion mit Totzeit multiplizeren

    GpYPaTRes = step(GpYPaT,tY); %Sprungantowrt berechnen

%     plot(tY,GpYPaTRes,"DisplayName","[Totzeit="+i+"]"),legend show,grid on;

end

%% Totzeit Stoergoessensprung

% figure(8);
% clf
% hold on;
% 
% stairs(tZ,GpZPaRes,"--","DisplayName","GpZPaRes"),legend show,grid on;
% stairs(tZ,stoergroessen.stoergroesse,'r',"DisplayName","Stoergroesse");
% stairs(tZ,stoergroessen.regelgroesse,'b',"DisplayName","Regelgroesse");

s = tf('s');



for i = 0:0.2:2

    totzeit = exp(-i * s);

    GpZPaT = GpZPa * totzeit;

    GpZPaTRes = step(GpZPaT,tZ);

%     plot(tZ,GpZPaTRes,"DisplayName","[Totzeit="+i+"]");

end








