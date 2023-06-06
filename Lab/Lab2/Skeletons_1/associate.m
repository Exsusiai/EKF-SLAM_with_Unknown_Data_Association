% This function performs the ML data association
%           S_bar(t)                 4XM
%           z(t)                     2Xn
%           association_ground_truth 1Xn | ground truth landmark ID for
%           every measurement  1 (actually not used)
% Outputs: 
%           outlier                  1Xn    (1 means outlier, 0 means not outlier) 
%           Psi(t)                   1XnXM
%           c                        1xnxM (actually not ever used)
function [outlier, Psi, c] = associate(S_bar, z, association_ground_truth)
    if nargin < 3
        association_ground_truth = [];
    end

    global lambda_psi % threshold on average likelihood for outlier detection
    global Q % covariance matrix of the measurement model
    global M % number of particles
    global N % number of landmarks
   
    % YOUR IMPLEMENTATION


end
