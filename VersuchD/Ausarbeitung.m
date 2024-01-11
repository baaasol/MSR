clear
clc
clf

addpath('Resourcen\VersuchB');
addpath('Resourcen\VersuchB\Gruppe1-1_Stellgrößensprung');
addpath('Resourcen\VersuchB\Gruppe1_1_(Stellgröße-)Störgröße');
addpath('Resourcen\VersuchB\StrejcY.m')

Auswertung

%% Tsumme

TsumVecPos = numel(GpYPaRes);
TsumVecNeg = numel(GpYPaRes);
SumPos = 0;
SumNeg = 0;
Tsum = 0;
TsumRT = 0;

for i = 1:1:numel(GpYPaRes)

    SumPos = SumPos + GpYPaRes(i);
    SumNeg = SumNeg + (KpY - GpYPaRes(numel(GpYPaRes) - i+1));

    TsumVecPos(i) = SumPos;
    TsumVecNeg(numel(GpYPaRes) - i+1) = SumNeg;

end

abs_diff = abs(TsumVecPos - TsumVecNeg);

[min_diff, min_index] = min(abs_diff);
Tsum = min_index;
TsumRT = Tsum*TintervalY

% PID Parameter:
KrTsum = 1/KpY
TiTsum = 0.7*TsumRT
TdTsum = 0.17*TsumRT
KiTsum = KrTsum/TiTsum
TaTsum = TdTsum/5
KdTsum = TdTsum*KrTsum

GpPIDTsum = KrTsum + tf(KiTsum,[1 0]) + tf([KdTsum 0],[TaTsum 1]);
GpPIDTsum

sys = step(GpPIDTsum,tY);




figure(11), clf, hold on, grid on, legend show, 
plot(tY,TsumVecPos);
plot(tY,TsumVecNeg);
plot(tY,abs_diff);
figure(12), clf, hold on, grid on, legend show,
plot(tY,GpYPaRes,"b-","DisplayName","GpYPaRes");
plot(tY,sys,"c-","DisplayName","Tsumme");


%% Latzel n = 3

%aus der Tabelle
TiT = 2.47;
TdT = 0.66;
KpKr = 2.543;

TiLa = 2.47 * tYSwa
TdLa = 0.66 * tYSwa

KrLa = KpKr/KpY


KrLa = 1/KpY
KiLa = KrLa/TiLa
KdLa = TdLa*KrLa


GpPIDLa = KrLa + tf(KrLa,[TiLa 0]) + tf([KrLa*TdLa 0],[TdLa/5 1]);
GpPIDLa
sys = step(GpPIDLa,tY);
plot(tY,sys,"g-","DisplayName","Latzel");

%% Strejc

k = (str.T1+str.T2-str.Te)/str.Te

KrStr = 1/KpY + (k^2+1)/2*k 
TiStr = (((k^2+1)*(k+1))/(k^2+k+1))*str.Te

KiStr = KrStr/TiStr
KdStr = 0

GpPIStr = tf([KrStr KrStr/TiStr],[1 0])

sys = step(GpPIStr,tY);
plot(tY,sys,"r-","DisplayName","Strejc");

%% Kompensationsregler

%figure(13), hold on, grid on, legend show

%handschriftlich bestimmt
GpPIKom = tf([1.647 0.237 7.28e-3],[0.689 0.0307 0])

sys = step(GpPIKom,tY);
KrKom = sys(1)
KiKom = sys(500)-sys(1) 

TiKom = KrKom/KiKom
KdKom = 0


plot(tY,step(GpPIKom,tY),"k-","DisplayName","Kompensationsregler")



%% Das ist leider unnötig 
% 
% 
% % Tsum
% 
% opt = stepDataOptions;
% opt.InputOffset = 4;
% opt.StepAmplitude = 2;
% 
% GpYSwaGpPIDTsum = feedback(GpYSwa*GpPIDTsum,1);
% 
% GpYSwaGpPIDTsumRes = step(GpYSwaGpPIDTsum, tY, opt);
% 
% %Latzel
% 
% GpYSwaGpPIDLa = feedback(GpYSwa*GpPIDLa,1);
% 
% GpYSwaGpPIDLaRes = step(GpYSwaGpPIDLa, tY, opt);
% 
% %Strejc
% 
% GpYSwaGpPIStr = feedback(GpYSwa*GpPIStr,1);
% 
% GpYSwaGpPIStrRes = step(GpYSwaGpPIStr, tY, opt);
% 
% %Kompensationsregler
% 
% GpYSwaGpPIKom = feedback(GpYSwa*GpPIKom,1);
% 
% GpYSwaGpPIKomRes = step(GpYSwaGpPIKom, tY, opt);
% 
% 
% % Plot the step response
% figure(13), clf, hold on, grid on, legend show
% plot(tY, GpYSwaGpPIDTsumRes,"b-","DisplayName","Tsumme");
% plot(tY, GpYSwaGpPIKomRes,"c-","DisplayName","Kompensationsregler");
% plot(tY, GpYSwaGpPIDLaRes,"g-","DisplayName","Latzel");
% plot(tY, GpYSwaGpPIStrRes,"r-","DisplayName","Strejc");
% title('Regelkreissprungantwort');
% xlabel('Time');
% ylabel('Output');
% 

out = sim("Stoergroessensprung.slx");

PIDSwaTsumY  = out.PIDSwaTsumYRes.';
PIDSwaTsumZ  = out.PIDSwaTsumZRes1.';
PIDSwaStrY   = out.PIDSwaStrYRes.';
PIDSwaStrZ   = out.PIDSwaStrZRes1.';
PIDSwaLaY    = out.PIDSwaLaYRes1.';
PIDSwaLaZ    = out.PIDSwaLaZRes.';
PIDSwaKomY   = out.PIDSwaKomYRes1.';
PIDSwaKomZ   = out.PIDSwaKomZRes.';

stellgrY      = out.StellgrY.';
%stoergrY      = out.StoergrY.';

stoergrY = 5.5 * ones(1, length(stellgrY ));

%stellgrZ      = out.StellgrZ.';
stoergrZ      = out.StoergrZ.';

stellgrZ  = 5 * ones(1, length(stoergrZ ));


tSimulink=(1:1:length(out.PIDSwaKomYRes1));
tSimulinkRt = tSimulink * 0.001;

figure(13), clf, hold on, grid on, legend show
plot(tSimulinkRt, PIDSwaTsumY,"b-","DisplayName","Tsumme Y");
plot(tSimulinkRt, PIDSwaKomY,"c-","DisplayName","Kompensationsregler");
plot(tSimulinkRt, PIDSwaLaY,"g-","DisplayName","Latzel");
plot(tSimulinkRt, PIDSwaStrY,"r-","DisplayName","Strejc");
plot(tSimulinkRt, stellgrY,"y-","DisplayName","stellgroesse");
plot(tSimulinkRt, stoergrY,"y-","DisplayName","stoergroesse");
title('Strecken Simulation Stellgrößensprung');
xlabel('Time');
ylabel('Output');

figure(14), clf, hold on, grid on, legend show
plot(tSimulinkRt, PIDSwaTsumZ,"b-","DisplayName","Tsumme Y");
plot(tSimulinkRt, PIDSwaKomZ,"c-","DisplayName","Kompensationsregler");
plot(tSimulinkRt, PIDSwaLaZ,"g-","DisplayName","Latzel");
plot(tSimulinkRt, PIDSwaStrZ,"r-","DisplayName","Strejc");
plot(tSimulinkRt, stellgrZ,"y-","DisplayName","stellgroesse");
plot(tSimulinkRt, stoergrZ,"y-","DisplayName","stoergroesse");
title('Strecken Simulation Störgrößensprung');
xlabel('Time');
ylabel('Output');

%% Umlaufdauer T bestimmen

% Strejc
[y,x] = max(PIDSwaStrY(400000:end))
tUmStr1 = x * 0.001
[y,x] = max(PIDSwaStrY(415000:end))
tUmStr2 = (x + 15000) * 0.001
tUmStr = tUmStr2-tUmStr1


% Kompensationsregler
[y,x] = max(PIDSwaKomY(400000:end))
tUmKom1 = x * 0.001
[y,x] = max(PIDSwaKomY(500000:end))
tUmKom2 = (x + 100000) * 0.001
tUmKom = tUmKom2-tUmKom1

%% Relative Überschwingweite

% Strejc
maxY = max(PIDSwaStrY(400000:end))
ySchwingStr = (maxY / 6 - 1) * 100

%Kompensationsregler
maxY = max(PIDSwaKomY(400000:end))
ySchwingKom = (maxY / 6 - 1) * 100

% T-Summen Verfahren
maxY = max(PIDSwaTsumY(400000:end))
ySchwingTsum = (maxY / 6 - 1) * 100


%% Einschwingen auf Beharrungszustand
% 
% % Kom
% % Define the threshold (e.g., 0.05 of the last value)
% thresholdHigh = 0.05 + 5;
% 
% % Find the first value above the threshold starting from the end
% aboveThresholdIndices = find(PIDSwaKomZ > thresholdHigh, 1, 'last')
% firstAboveThresholdValue = PIDSwaKomZ(aboveThresholdIndices)
% 
% thresholdLow = 5 - 0.05;
% 
% % Find the first value above the threshold starting from the end
% belowThresholdIndices = find(PIDSwaKomZ < thresholdLow, 1, 'last')
% firstbelowThresholdValue = PIDSwaKomZ(belowThresholdIndices)
% 
% % Strejc
% 
% % Find the first value above the threshold starting from the end
% aboveThresholdIndices = find(PIDSwaStrZ > thresholdHigh, 1, 'last')
% firstAboveThresholdValue = PIDSwaStrZ(aboveThresholdIndices)
% 
% % Find the first value above the threshold starting from the end
% belowThresholdIndices = find(PIDSwaStrZ < thresholdLow, 1, 'last')
% firstbelowThresholdValue = PIDSwaStrZ(belowThresholdIndices)
% 
% % Tsumme
% 
% % Find the first value above the threshold starting from the end
% aboveThresholdIndices = find(PIDSwaTsumZ > thresholdHigh, 1, 'last')
% firstAboveThresholdValue = PIDSwaTsumZ(aboveThresholdIndices)
% 
% % Find the first value above the threshold starting from the end
% belowThresholdIndices = find(PIDSwaTsumZ < thresholdLow, 1, 'last')
% firstbelowThresholdValue = PIDSwaTsumZ(belowThresholdIndices)
% 
% %Latzel
% 
% % Find the first value above the threshold starting from the end
% aboveThresholdIndices = find(PIDSwaLaZ > thresholdHigh, 1, 'last')
% firstAboveThresholdValue = PIDSwaTsumZ(aboveThresholdIndices)
% 
% % Find the first value above the threshold starting from the end
% belowThresholdIndices = find(PIDSwaLaZ < thresholdLow, 1, 'last')
% firstbelowThresholdValue = PIDSwaLaZ(belowThresholdIndices)
% 

%% Einschwingen auf Beharrungszustand

% Kom

disp("Kompensationsregler")

% Define the threshold (e.g., 0.05 of the last value)
thresholdHigh = 0.05 + 5;

% Find the first value above the threshold starting from the end
aboveThresholdIndices = find(PIDSwaKomZ > thresholdHigh, 1, 'last')
firstAboveThresholdValue = PIDSwaKomZ(aboveThresholdIndices)
timeAboveThresholdIndices = aboveThresholdIndices * 0.001 - 400

thresholdLow = 5 - 0.05;


% Find the first value above the threshold starting from the end
belowThresholdIndices = find(PIDSwaKomZ < thresholdLow, 1, 'last')
firstbelowThresholdValue = PIDSwaKomZ(belowThresholdIndices)
timeBelowThresholdIndices = belowThresholdIndices * 0.001 - 400

% Strejc

disp("Strejc")

% Find the first value above the threshold starting from the end
aboveThresholdIndices = find(PIDSwaStrZ > thresholdHigh, 1, 'last')
firstAboveThresholdValue = PIDSwaStrZ(aboveThresholdIndices)
timeAboveThresholdIndices = aboveThresholdIndices * 0.001 - 400

% Find the first value above the threshold starting from the end
belowThresholdIndices = find(PIDSwaStrZ < thresholdLow, 1, 'last')
firstbelowThresholdValue = PIDSwaStrZ(belowThresholdIndices)
timeBelowThresholdIndices = belowThresholdIndices * 0.001 - 400

% Tsumme

disp("T-Summen Verfahren")

% Find the first value above the threshold starting from the end
aboveThresholdIndices = find(PIDSwaTsumZ > thresholdHigh, 1, 'last')
firstAboveThresholdValue = PIDSwaTsumZ(aboveThresholdIndices)
timeAboveThresholdIndices = aboveThresholdIndices * 0.001 - 400

% Find the first value above the threshold starting from the end
belowThresholdIndices = find(PIDSwaTsumZ < thresholdLow, 1, 'last')
firstbelowThresholdValue = PIDSwaTsumZ(belowThresholdIndices)
timeBelowThresholdIndices = belowThresholdIndices * 0.001 - 400

%Latzel

disp("Latzel")

% Find the first value above the threshold starting from the end
aboveThresholdIndices = find(PIDSwaLaZ > thresholdHigh, 1, 'last')
firstAboveThresholdValue = PIDSwaTsumZ(aboveThresholdIndices)
timeAboveThresholdIndices = aboveThresholdIndices * 0.001 - 400

% Find the first value above the threshold starting from the end
belowThresholdIndices = find(PIDSwaLaZ < thresholdLow, 1, 'last')
firstbelowThresholdValue = PIDSwaLaZ(belowThresholdIndices)
timeBelowThresholdIndices = belowThresholdIndices * 0.001 - 400




