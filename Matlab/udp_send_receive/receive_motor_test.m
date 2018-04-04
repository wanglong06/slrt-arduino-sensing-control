u = UDP_msgr('192.168.1.177',10001,10002);
fprintf('\nReceiving data from Arduino ... \n')
N=100;
reverseStr = [];
t0 = tic;
while 1==1
    data = u.receiveDataMsg(6);
    t = toc(t0);
    u.send([1,1,1,1,2*sin(2*pi/10*t),1]);
    if ~isempty(data)
        msg = sprintf(...
            ' q1=%.2f,\n q2=%.2f,\n q3=%.2f,\n u1=%.2f,\n u2=%.2f,\n u3=%.2f,\n',...
            data(1),data(2),data(3),data(4),data(5),data(6));
            fprintf([reverseStr, msg]);
            reverseStr = repmat(sprintf('\b'), 1, length(msg));
    else
        msg = [];
    end
end