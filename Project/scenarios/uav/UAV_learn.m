%% uav_learn.m
% *Summary:* Script to learn a controller for unicycling
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Last modified: 2013-03-27
%
%% High-Level Steps
% # Load parameters
% # Create J initial trajectories by applying random controls
% # Controlled learning (train dynamics model, policy learning, policy
% application)

%% Code

% 1. Initialization
clear all; close all;
settings_uav;                   % load scenario-specific settings
initparams;
basename = 'uav_';              % filename used for saving data

% the main figure in which the uav is shown
% videoFigure = figure('units','normalized','outerposition',[0 0 1 1]);

videoFigure = figure;

% the VideoWriter class for creating the video
videoWriter = VideoWriter('1.avi'); 
open(videoWriter);

% 2. Initial J random rollouts
for jj = 1:J                                        % get the first observations
  [xx, yy, realCost{jj}, latent{jj}] = ...
    rollout(gaussian(mu0, S0), struct('maxU',policy.maxU/5), H, plant, cost);
  x = [x; xx]; y = [y; yy];
  if plotting.verbosity > 0      % visualization of trajectory
    if ~ishandle(1); figure(videoFigure); else set(0,'CurrentFigure',1); end; clf(1);
    draw_rollout_uav;
  end
end

z(odei,:) = bsxfun(@plus, mu0, chol(S0)'*randn(length(odei),1000));   % compute
mu0Sim = mean(z,2); S0Sim = cov(z');         % of augmented start state by MCMC
mu0Sim(odei) = mu0; S0Sim(odei,odei) = S0;        % Put in known correct values
mu0Sim = mu0Sim(dyno); S0Sim = S0Sim(dyno,dyno); clear z i;

% 3. Controlled learning (N iterations)
for j = 1:N
  trainDynModel;   % train (GP) dynamics model
  learnPolicy;     % learn policy
  applyController; % apply controller to system  
  disp(['controlled trial # ' num2str(j)]);
  if plotting.verbosity > 0      % visualization of trajectory
    if ~ishandle(1); figure(videoFigure); else set(0,'CurrentFigure',videoFigure); end; clf(1);
    draw_rollout_uav;
  end
end
