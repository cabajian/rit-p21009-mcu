device = serialport("COM13",115200);
M_ob_acc = ["x" "y" "z"];
M_ob_eul = ["h" "r" "p"];
M_imu_acc = ["x" "y" "z"];
M_imu_gyr = ["x" "y" "z"];
n = 0;
DELIM = ">";
TEST_START = "START";
TEST_END = "END";
LOG_UNKNOWN = "UNKNOWN";

while (1)
    [sname,sdata,x,y,z] = readDataLine(device);
    if (sname == TEST_END)
        fname = sprintf('%s%d.dat','obAcc',n);
        writematrix(M_ob_acc,fname,'Delimiter',';');  
        fname = sprintf('%s%d.dat','imuAcc',n);
        writematrix(M_ob_acc,fname,'Delimiter',';');  
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
                    M_imu_accel = cat(1,M_imu_acc,[x y z]); 
                elseif (sdata == "GYR")
                    M_imu_gyro = cat(1,M_imu_gyr,[x y z]);
                end
        end
    end
end




function [sname,sdata, x,y,z] = readDataLine(serial_device)
    data = readline(serial_device);
    if (data == TEST_START)
        sname = TEST_START;
        sdata = "";
        x = 0;
        y = 0;
        z = 0;
    else
        line = split(data, delim);
        if (length(line) < 3)
            sname = LOG_UNKNOWN;
            sdata = "";
            x = 0;
            y = 0;
            z = 0;
        else
            sname = line(1);
            sdata = line(2);
            vals = split(line(3));
            vals(cellfun('isempty',vals)) = []; % remove empty cells
            switch (length(vals))
                case 1
                    x = vals(1);
                    y = 0;
                    z = 0;
                case 2
                    x = vals(1);
                    y = vals(2);
                    z = 0;
                case 3
                    x = vals(1);
                    y = vals(2);
                    z = vals(3);
                otherwise
                    x = 0;
                    y = 0;
                    z = 0;
            end
        end
    end
end