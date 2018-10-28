%% settings_uav.m
% *Summary:* Script to set up the uav scenario
%
% Copyright (C) 2008-2013 by 
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
%% High-Level Steps
% # Define state and important indices
% # Set up scenario
% # Set up the plant structure
% # Set up the policy structure
% # Set up the cost structure
% # Set up the GP dynamics model structure
% # Parameters for policy optimization
% # Plotting verbosity
% # Some array initializations


%% Code

warning('off','all'); format short; format compact

% include some paths
try
  rd = '../../';
  addpath([rd 'base'],[rd 'util'],[rd 'gp'],[rd 'control'],[rd 'loss']);
catch
end

rand('seed',1); randn('seed',1);

% 1. Define state and important indices

% 1a. Full state representation (including all augmentations)
% - non-angle velocities
% - angular velocities
% - non-angles
% - quaternions
% - controls
%

% 1a. Full state representation (including all augmentations)
%  1  u         UAV velocity in x direction
%  2  v         UAV velocity in y direction
%  3  w         UAV velocity in z direction
%  4  p         UAV body rate (pitch/roll/yaw)
%  5  q         UAV body rate (pitch/roll/yaw)
%  6  r         UAV body rate (pitch/roll/yaw)
%  7  x         UAV x position
%  8  y         UAV y position
%  9  z         UAV z position
%  10 q0        UAV altitude Quaternion 0
%  11 q1        UAV altitude Quaternion 1
%  12 q2        UAV altitude Quaternion 2
%  13 q3        UAV altitude Quaternion 3
%  14 rpm1      Control RPM input for motor 1
%  15 rpm2      Control RPM input for motor 2
%  16 rpm3      Control RPM input for motor 3
%  17 rpm4      Control RPM input for motor 4

% 1b. Important indices
% odei  indicies for the ode solver
% augi  indicies for variables augmented to the ode variables
% dyno  indicies for the output from the dynamics model and indicies to loss
% angi  indicies for variables treated as angles (using sin/cos representation)
% dyni  indicies for inputs to the dynamics model
% poli  indicies for variables that serve as inputs to the policy
% difi  indicies for training targets that are differences (rather than values)

odei = [1 2 3 4 5 6 7 8 9 10 11 12 13];     % varibles for the ode solver
augi = [];                                  % variables to be augmented
dyno = [1 2 3 4 5 6 7 8 9 10 11 12 13];     % variables to be predicted (and known to loss)
angi = [];                                  % angle variables
dyni = [1 2 3 4 5 6 7 8 9 10 11 12 13];     % variables that serve as inputs to the dynamics GP
poli = [1 2 3 4 5 6 7 8 9];     % variables that serve as inputs to the policy
difi = [1 2 3 4 5 6 7 8 9];     % variables that are learned via differences     

% 2. Set up the scenario
global dt;
dt = 0.025;                    % [s] sampling time
T = 2.0;                      % [s] initial prediction horizon time
H = ceil(T/dt);               % prediction steps (optimization horizon)
maxH = ceil(10.0/dt);         % max pred horizon
s = (0.1 * ones(1,13)).^2;
S0 = diag(s);                 % initial state variance, 95% is +/- 11.4 degrees
mu0 = [0 0 0 0 0 0 0 0 0 0 -1 0 0]';    % initial state mean
N = 40;                       % number of controller optimizations
J = 10;                       % initial J trajectories of length H
K = 1;                        % number of initial states for which we optimize

% 3. Set up the plant structure
plant.dynamics = @dynamics_uav;             % dynamics ODE function
% plant.constraint = inline('abs(x(7))>4 | abs(x(8))>4 | abs(x(9))>4 | x(9)<-0.5 | sum(abs(quat2angle([x(10) x(11) x(12) x(13)])) > [pi pi/3 pi/2])');    % ODE constraint
% plant.constraint = inline('x(9)<-0.5 | sum(abs(quat2angle([x(10) x(11) x(12) x(13)])) > [pi 80*pi/180 pi/2])');    % ODE constraint
plant.constraint = inline('x(9)<-0.5');    % ODE constraint
plant.noise = diag(ones(1,13)*0.01.^2);     % measurement noise
plant.dt = dt;
plant.ctrl = @zoh;                  % controller is zero order hold
plant.odei = odei;                  % indices to the varibles for the ODE solver
plant.augi = augi;                  % indices of augmented variables
plant.angi = angi;
plant.poli = poli;
plant.dyno = dyno;
plant.dyni = dyni;
plant.difi = difi;
plant.prop = @propagated;           % handle to function that propagates state over time

% 4. Set up the policy structure

% Linear Policy
% policy.fcn = @(policy,m,s)conCat(@conlin,@gSat,policy,m,s);   % linear policy
% policy.maxU = [10000 10000 10000 10000];                      % max. amplitude of rpms 
% policy.p.w = 1e-2*randn(length(policy.maxU),length(poli));    % weight matrix
% policy.p.b = zeros(length(policy.maxU),1);                    % bias

% RBF Policy
policy.fcn = @(policy,m,s)conCat(@congp,@gSat,policy,m,s);      % RBF policy
policy.maxU = [10000 10000 10000 10000];                        % max. amplitude of rpms 
mm = mu0;   ss = S0;    nc = 100;
policy.p.inputs = gaussian(mm(poli), ss(poli,poli), nc)';       % init. location of 
policy.p.targets = 0.1*randn(nc, length(policy.maxU));          % init. policy targets 
policy.p.hyp = ...                                              % initialize policy
  repmat(log([ones(1,9) 1 0.01]'), 1,4);              % hyper-parameters


                                                            
% 5. Set up the cost structure
cost.fcn = @loss_uav;                       % cost function
cost.gamma = 1;                             % discount factor
cost.width = 1;                             % cost function width
cost.expl = 0;                              % exploration parameter (UCB)
% Note to self: this line was not in the original code
cost.target = [0 0 0 0 0 0 0 0 2 0 -1 0 0]';    % target state

% 6. Set up the GP dynamics model structure
dynmodel.fcn = @gp1d;                % function for GP predictions
dynmodel.train = @train;             % function to train dynamics model
dynmodel.induce = zeros(300,0,1);    % shared inducing inputs (sparse GP)



% 7. Parameters for policy optimization
opt.length = -100;                   % max. number of function evaluations
opt.MFEPLS = 20;                     % max. number of function evaluations
                                     % per line search
opt.verbosity = 2;                   % verbosity: specifies how much 
                                     % information is displayed during
                                     % policy learning. Options: 0-3
opt.method = 'BFGS';                 % optimization algorithm
trainOpt = [300 300];                % defines the max. number of line searches
                                     % when training the GP dynamics models
                                     % trainOpt(1): full GP,
                                     % trainOpt(2): sparse GP (FITC)
                                     

% 8. Plotting verbosity
plotting.verbosity = 2;              % 0: no plots
                                     % 1: some plots
                                     % 2: all plots

% 9. Some initializations
x = []; y = [];
fantasy.mean = cell(1,N); fantasy.std = cell(1,N);
realCost = cell(1,N); M = cell(N,1); Sigma = cell(N,1);