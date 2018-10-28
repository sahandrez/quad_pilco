function []= initparams()

    global g m I Ixx Iyy Izz
    
    global Jr Dt Kt PROP_POSNS u2RpmMat pZ pXY dZ dXY

    g = 9.81;           % gravity
    m = 1.096;          % mass (kg) 

    % inertial properties in body frame
    Ixx = 0.01121976;
    Iyy = 0.01122668;
    Izz = 0.021082335;
    
    Ixy = -0.0000562297;
    Iyz = -0.0000044954;
    Izx = -0.00000001418;

    I = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 

    Jr = 2.20751e-5;    %Propeller moment of inertia about rotation axis (kg m^2)

    % loads geometry of Spiri
    load('locations.mat');
    
    % prop locations relative to CoM
    PROP_POSNS = [p1, p2, p3, p4] - repmat(CoM,1,4);

    Kt = 8.7e-8;        % Thrust coefficient
    Dt = 8.7e-9;        % Drag coefficient
    
    u2RpmMat = inv([-Kt                   -Kt                   -Kt                   -Kt;              
                    -Kt*PROP_POSNS(2,1) -Kt*PROP_POSNS(2,2) -Kt*PROP_POSNS(2,3) -Kt*PROP_POSNS(2,4);
                     Kt*PROP_POSNS(1,1)  Kt*PROP_POSNS(1,2)  Kt*PROP_POSNS(1,3)  Kt*PROP_POSNS(1,4);
                    -Dt                    Dt                   -Dt                    Dt                 ]);
    
    % position and velocity proportional gain matricies
    dZ = 0;
    dXY = 0;
    pZ = 0;
    pXY = 0;

end

