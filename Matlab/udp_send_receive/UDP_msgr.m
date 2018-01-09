classdef UDP_msgr
    %%  UDP messenger
    %   Long Wang, 2018/1/8
    %   For Testing purposes
    properties
        udpOBJ; % udp object by Matlab
    end
    
    methods
        function obj = UDP_msgr(RemoteIP,RemotePort,LocalPort)
            if nargin<3
                LocalPort = RemotePort;
            end
            obj.udpOBJ = udp(RemoteIP,RemotePort,'LocalPort',LocalPort);
            try
                fopen(obj.udpOBJ);
            catch
                instrreset;
                obj.udpOBJ = udp(RemoteIP,RemotePort,'LocalPort',LocalPort);
                fopen(obj.udpOBJ);
            end
        end
        function send(obj,data,format)
            if nargin<3
                format = 'double';
            end
            % data = swapbytes(data);
            fwrite(obj.udpOBJ,data,format);
        end
        function data = receiveStringMsg(obj,dataArraySize)
            data  = char(fread(obj.udpOBJ,dataArraySize,'char'));
        end
        function [data,received] = receiveDataMsg(obj,dataArraySize)
            if nargin<2
                dataArraySize = 1;
            end
            packet_READ = 0;
            data = [];
            while (obj.udpOBJ.BytesAvailable ~= 0)
                packet_READ = 1;
                data  = fread(obj.udpOBJ,4*dataArraySize);
            end
            received = packet_READ*(length(data)==dataArraySize);
            data = obj.unpackUDP_Msg(uint8(data));
        end
        function close(obj)
            fclose(obj.udpOBJ);
        end
    end
    methods (Static)
        function data_unpacked = unpackUDP_Msg(data)
            %   Assuming the data is [4n x 1]
            dataLen = length(data)/4;
            data_unpacked = zeros(dataLen,1);
            for i = 1:dataLen
                start_i = 1+(i-1)*4;
                data_unpacked(i) = typecast(data(start_i:start_i+3)','single');
            end
        end
    end
    
end

