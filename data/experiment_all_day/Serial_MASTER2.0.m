serial = serial('COM3');
set(serial,'BaudRate',921600);
set(serial,'InputBufferSize',4);
set(serial,'Timeout',1);
fopen(serial);

%Enable ControleVpv%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(serial,'v');

%Enable MPPT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(serial,'m');

%Ensaiar função de transferencia%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%fprintf(serial,'f')

%Habilita P&O modificado p/ carregar a bateria%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%fprintf(serial,'g')

%N=500;
N=100;
for n=1:N
    
    fprintf(serial,'{');
    TensionPainel(n) = fread(serial, 1, 'float32');

    fprintf(serial,'$');
    ReferenciaVPainel(n) = fread(serial, 1, 'float32');
    
    fprintf(serial,'#');
    ControlVPainelOut(n) = fread(serial, 1, 'float32');

    fprintf(serial,'}');
    CurrentPainel(n) = fread(serial, 1, 'float32');
    
    fprintf(serial,'!');
    TensionLoad(n) = fread(serial, 1, 'float32');
    
    fprintf(serial,'@');
    CurrentLoad(n) = fread(serial, 1, 'float32');
    
    
    fprintf(serial,'a');
    SensorX(n) = fread(serial, 1, 'float32');

    fprintf(serial,'b');
    SensorY(n) = fread(serial, 1, 'float32');
    
    fprintf(serial,'c');
    ControlX(n) = fread(serial, 1, 'float32');
    
    fprintf(serial,'d');
    ControlY(n) = fread(serial, 1, 'float32');

    pause(0.01); %fs = 100Hz

end

tempo1 = 0:0.01:N*0.01-0.01; %fs = 100Hz
%tempo1 = 0:1e-3:N*1e-3-1e-3; %fs = 1kHz

figure(1)
hold on
plot(tempo1,TensionPainel,'LineWidth', 1.5)
plot(tempo1,ReferenciaVPainel,'LineWidth', 1.5)
grid on

figure(2)
plot(tempo1,ControlVPainelOut,'LineWidth', 1.5)
grid on

figure(3)
plot(tempo1,TensionPainel.*CurrentPainel,'LineWidth', 1.5)
grid on

Power = TensionPainel.*CurrentPainel;

%plot(tempo1,SensorX,'LineWidth', 1.5)
%plot(tempo1,SensorY,'LineWidth', 1.5)

%Rastreia sol%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(serial,'r'); 

%Stop rastreamento sol
fprintf(serial,'s'); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Posição perpendicular %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(serial,'t');
pause (8);
fprintf(serial,'p');


%Close serial port
fclose(serial);

clear serial
clc
