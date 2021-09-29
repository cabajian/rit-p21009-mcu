clear device;
device = serialport("COM13",115200);
M_ob_acc = ["x" "y" "z"];
M_ob_eul = ["h" "r" "p"];
M_imu_acc = ["x" "y" "z"];
M_imu_gyr = ["x" "y" "z"];
n = 0;

while (1)
    [sname,sdata,x,y,z] = readDataLine(device);
    if (contains(sname, "STOP"))
        fname = sprintf('MatlabObj/%s%d.dat','obAcc',n);
        writematrix(M_ob_acc,fname,'Delimiter',';');
        fname = sprintf('MatlabObj/%s%d.dat','obEul',n);
        writematrix(M_ob_eul,fname,'Delimiter',';');
        fname = sprintf('MatlabObj/%s%d.dat','imuAcc',n);
        writematrix(M_imu_acc,fname,'Delimiter',';');
        fname = sprintf('MatlabObj/%s%d.dat','imuGyr',n);
        writematrix(M_imu_gyr,fname,'Delimiter',';');
        M_ob_acc = ["x" "y" "z"];
        M_ob_eul = ["h" "r" "p"];
        M_imu_acc = ["x" "y" "z"];
        M_imu_gyr = ["x" "y" "z"];
        n = n + 1;
    else
        switch (sname)
            case "OBH"
                if (sdata == "ACC")
                    M_ob_acc = cat(1,M_ob_acc,[x y z]); 
                elseif (sdata == "EUL")
                    M_ob_eul = cat(1,M_ob_eul,[x y z]);
                end
            case "IMUL"
                if (sdata == "ACC")
                    M_imu_acc = cat(1,M_imu_acc,[x y z]); 
                elseif (sdata == "GYR")
                    M_imu_gyr = cat(1,M_imu_gyr,[x y z]);
                end
        end
    end
end




function [sname,sdata, x,y,z] = readDataLine(serial_device)
    data = readline(serial_device);
    if (isempty(data))
        sname L= "";
        sdata = "";
        x = 0;
        y = 0;
        z = 0;
    else
        data = strtrim(data);
        if (contains(data, "START"))
            sname = "START";
            sdata = "";
            x = 0;
            y = 0;
            z = 0;
        elseif (contains(data, "STOP"))
            sname = "STOP";
            sdata = "";
            x = 0;
            y = 0;
            z = 0;  
        else
            line = split(data, ">");
             if (length(line) < 3)
                sname = "UNKNOWN";
                sdata = "";
                x = 0;
                y = 0;
                z = 0;
            else
                sname = line(1);
                sdata = line(2);
                switch (length(line)-2)
                    case 1
                        x = line(3);
                        y = 0;
                        z = 0;
                    case 2
                        x = line(3);
                        y = line(4);
                        z = 0;
                    case 3
                        x = line(3);
                        y = line(4);
                        z = line(5);
                    otherwise
                        x = 0;
                        y = 0;
                        z = 0;
                end
            end
        end
    end
end