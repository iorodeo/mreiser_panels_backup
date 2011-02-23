function send_serial(string_to_send, waitingAck)
%send_serial is a function for sending and receving serial port info from matlab. 
%
%send_serial(string_to_send, waitingAck)
%        string_to_send is the command to be sent 
%        waitingAck is a flag for the function to wait for a acknowledgement or not
%        waitingAck default value is 0 when only user inputs only one
%        argument for send_serial. 
%        If waitingAck = 0, the function doesn't wait for data from the
%        receiver after sending the command.
%        If waitingAck = 1; the function wait for 0.1 second and then check
%        whether the data from the receiver are ready. If the data are
%        ready, the data are displayed.
%        If waitingAck = 2; the function wait until the data from the receiver 
%        are ready. If the data are ready, the data are displayed.

global serialPort

if nargin <2
        waitingAck = 0;
end

try
    fwrite(serialPort, string_to_send, 'uchar');

    
catch ME

    init_serial;
    fwrite(serialPort, string_to_send, 'uchar');
end




