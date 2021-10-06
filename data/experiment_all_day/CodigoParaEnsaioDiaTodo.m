%% Temperatura ambiente dia 02/05/21
%       6h 7h 8h 9h 10 11 12 13 14 15 16 17 18
temp = [18 19 19 22 26 25 27 29 30 28 28 26 25];

% Temperatura medida no painel
%       6h 7h 8h 9h 10 11 12 13 14 15 16 17 18
temp = [18 20 24 31 40 40 47 49 47 39 30 26];0

%% Open Serial Port
serial = serial('COM3');
set(serial,'BaudRate',921600);
set(serial,'InputBufferSize',4);
set(serial,'Timeout',1);
fopen(serial);

%% Linha da Matrix para salvar
%Linha     =   1  2  3  4  5  6  7   8  9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
HoraDoDdia = [6.5 7 7.5 8 8.5 9 9.5 10 10.5 11 11.5 12 12.5 13 13.5 14 14.5 15 15.5 16 16.5 17 17.5 18];
m = 24;

%% Close serial port
fclose(serial);
clear serial
clc

%% Estrutura FIXA%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Posição perpendicular
fprintf(serial,'t');
pause (8);
fprintf(serial,'p');

%Enable ControleVpv
fprintf(serial,'v');

%Enable MPPT
fprintf(serial,'m');

N=200;
for n=1:N
    
    fprintf(serial,'{');
    TensionPainel_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'$');
    ReferenciaVPainel_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'#');
    ControlVPainelOut_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'}');
    CurrentPainel_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'!');
    TensionLoad_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'@');
    CurrentLoad_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'a');
    SensorX_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'b');
    SensorY_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'c');
    ControlX_FIX(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'d');
    ControlY_FIX(m,n) = fread(serial, 1, 'float32');

    pause(0.05); %fs = 20Hz
    
end

%% Estrutura SEGUIDORA%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Rastreia sol
fprintf(serial,'r');

%Enable ControleVpv
fprintf(serial,'v');

%Enable MPPT
fprintf(serial,'m');

N=200;
for n=1:N
    
    fprintf(serial,'{');
    TensionPainel_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'$');
    ReferenciaVPainel_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'#');
    ControlVPainelOut_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'}');
    CurrentPainel_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'!');
    TensionLoad_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'@');
    CurrentLoad_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'a');
    SensorX_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'b');
    SensorY_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'c');
    ControlX_SEG(m,n) = fread(serial, 1, 'float32');
    fprintf(serial,'d');
    ControlY_SEG(m,n) = fread(serial, 1, 'float32');

    pause(0.05); %fs = 20Hz
    
end

%Stop rastreamento sol
fprintf(serial,'s');


%% Plotar variaveis para verificação

tempo1 = 0:0.05:N*0.05-0.05; %fs = 20Hz

Power_FIX(m,:) = TensionPainel_FIX(m,:).*CurrentPainel_FIX(m,:);
Power_SEG(m,:) = TensionPainel_SEG(m,:).*CurrentPainel_SEG(m,:);

%Fixa%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
hold on
plot(tempo1,TensionPainel_FIX(m,:),'LineWidth', 1.5)
plot(tempo1,ReferenciaVPainel_FIX(m,:),'LineWidth', 1.5)
grid on

figure(2)
plot(tempo1,Power_FIX,'LineWidth', 1.5)

figure(3)
hold on
plot(tempo1,SensorX_FIX(m,:),'LineWidth', 1.5)
plot(tempo1,SensorY_FIX(m,:),'LineWidth', 1.5)
grid on

%Seguindo%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
hold on
plot(tempo1,TensionPainel_SEG(m,:),'LineWidth', 1.5)
plot(tempo1,ReferenciaVPainel_SEG(m,:),'LineWidth', 1.5)
grid on

figure(2)
plot(tempo1,Power_SEG,'LineWidth', 1.5)

figure(3)
hold on
plot(tempo1,SensorX_SEG(m,:),'LineWidth', 1.5)
plot(tempo1,SensorY_SEG(m,:),'LineWidth', 1.5)
grid on
