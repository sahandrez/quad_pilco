function [rpmRadian] = rpm2rad(rpm)
    % converts from RPM to rad / s
    rpmRadian = rpm * 2 * pi / 60;
end