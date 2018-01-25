u = UDP_msgr('192.168.1.177',10001,10002);
fprintf('\nReceiving sensor array data from Arduino ... \n')
N=100;
reverseStr = [];
t0 = tic;
msg_m1 = ' m1 s1=%.0f,\n m1 s2=%.0f,\n m1 s3=%.0f,\n m1 s4=%.0f';
msg_m2 = ' m2 s1=%.0f,\n m2 s2=%.0f,\n m2 s3=%.0f,\n m2 s4=%.0f';
while 1==1
    data = u.receiveDataMsg(8,'uint8');
    t = toc(t0);
    if ~isempty(data)
        msg = sprintf(...
            [msg_m1,'\n',msg_m2],...
            data(1),data(2),data(3),data(4),data(5),data(6),data(7),data(8));
            fprintf([reverseStr, msg]);
            reverseStr = repmat(sprintf('\b'), 1, length(msg));
    else
        msg = [];
    end
end