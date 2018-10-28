%% dynamics_uav.m
% *Summary:* Implements ths ODE for simulating the UAV 
% dynamics, where an input rpm can be applied to three active actuators, 
%
%    function dz = dynamics_uav(t, z, rpm1, rpm2, rpm3)
%
%
% *Input arguments:*
%
%	t     current time step (called from ODE solver)
%   z     state                                                    [13 x 1]
%   rpm1  RPM applied to motor 1
%   rpm2  RPM applied to motor 2
%   rpm3  RPM applied to motor 3
%
% *Output arguments:*
%   
%   dz    state derivative wrt time        
%
%   Note: It is assumed that the state variables are of the order in
%   settings_uav.m
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%

function dz = dynamics_uav(t, z, rpm1, rpm2, rpm3)
%% Code
    global g m I Jr PROP_POSNS Kt Dt dt;        
    
    rpmControl = [rpm1(t); rpm2(t); rpm3(t); 0];
    rpmPrev = [rpm1(t-1); rpm2(t-1); rpm3(t-1); 0];
    rpmDeriv = (rpm2rad(rpmControl) - rpm2rad(rpmPrev)) / dt;

    if isreal(rpmControl) == 0
        error('Control rpm is imaginary');
    end

    if isreal(z) == 0
        error('State is imaginary');
    end
   
    dz = zeros(13,1);
    % define rotation matrix
    R = quat2rotmat(z(10:13));

    % define forces
    fGravity = R*[0; 0; -m*g];                  % force of gravity, body frame
    fThrust = [0; 0; -Kt*sum(rpmControl.^2)];   % force of thrust, body frame

    % compute moments
    Mx = -Kt*PROP_POSNS(2,:)*(rpmControl.^2) - z(5) * Jr * sum(rpm2rad(rpmControl));
    My =  Kt*PROP_POSNS(1,:)*(rpmControl.^2) + z(4) * Jr * sum(rpm2rad(rpmControl));
    Mz =        Dt*[-1 1 -1 1]*(rpmControl.^2) - Jr*[-1 1 -1 1]* rpm2rad(rpmDeriv);

    dz(1)=z(4)+z(5)*sin(z(1))*tan(z(2))+z(6)*cos(z(1))*tan(z(2));
    dz(2)=z(5)*cos(z(1))-z(6)*sin(z(1));
    dz(3)=1/cos(z(2))*(z(5)*sin(z(1))+z(6)*cos(z(1)));
    dz(1:3)   = (fGravity + fThrust  -  m*cross(z(4:6), z(1:3)))/m;
    dz(4:6)   = inv(I)*([Mx; My; Mz] -  cross(z(4:6), I * z(4:6)));
    dz(7:9)   = R' * z(1:3);
    dz(10:13) = -0.5 * quatmultiply( [0; z(4:6)] , z(10:13));
end
