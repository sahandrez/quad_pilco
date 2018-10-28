%% draw_rollout_uav.m
% *Summary:* Script to draw a rollout of the uav
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
%% High-Level Steps
% # For each time step, plot the observed trajectory, the applied torques,
% and the incurred cost

%% Code

if j > 0
  
  draw_uav(xx, plant, plant.dt/10, cost, ...
    ['trial # ' num2str(j+J) ', T=' num2str(H*dt) ' sec'], ...
    ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
    ' sec'], videoFigure, videoWriter);  
else
  draw_uav(xx, plant, plant.dt/10, cost, ...
    ['(random) trial # ' num2str(jj) ', T=' num2str(H*dt) ' sec'], ...
    ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
    ' sec'], videoFigure, videoWriter);
end
