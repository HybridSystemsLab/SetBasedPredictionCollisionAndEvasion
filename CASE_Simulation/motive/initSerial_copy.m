function [s, flag] = initSerial_copy() 
pause on; 
try 
     ppmValues = [1500, 1500, 1500, 1500, 1500, 1500]; 
     numChannels = size(ppmValues); 
     %numChannels = numChannels(2); //////////////////
 
 
     %Serial setup 
     s = serial('COM6');%('/dev/tty.usbmodem1411');    % define serial port 
     %s = serial(comPort); 
     s.BaudRate=9600;               % define baud rate 
     s.DataBits=8;                  %default config of arduino 8 data bits, one stop bit, no parity 
     s.StopBits=1; 
     s.Parity= 'none'; 
     set(s, 'terminator', 'LF'); 
 
 
     fopen(s);              % open serial port 
     %display(['Serial Open']); 
     pause(2) 
     w = 'X'; 
 
 
 
 
     %try                             % use try catch to ensure fclose 
     while(w ~= 'A')     
         %w=fscanf(s,'%s');  
         w=fread(s,1,'uchar');              % must define the input % d or %s, etc. 
     end 
 

     display(['ArduinoPPM found']); 
     flag = 1;
      
 catch ME 
     fclose(s); 
     flag = 0;
 end 
