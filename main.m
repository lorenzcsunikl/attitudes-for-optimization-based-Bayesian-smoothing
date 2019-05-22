% Copyright 2019 Michael Lorenz <lorenz@cs.uni-kl.de>
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

clearvars;
clc;
addpath('./0RotationCalculus')
addpath('./0Solvers')
addpath('./Smoothing')
close all;
%% INITIALIZATION of measurements

%global constants
y.n_samp = 399;         % number of gyroscope measurements
y.g = [0; 0; -9.81];    % gravity
y.m = [1; 0; 0];        % magnetic field
y.T = 1/100.0;          % sampling time
y.n_magres = 1;         % number of magnetometer measurements used

%  initialisation of Covariances:
smoother.Q  = 1.6e-5 *eye(3);   % orienation process noise
smoother.Ra = 0.0015 *eye(3);   % accelermoter noise
smoother.Rm = 0.0015 *eye(3);   % magnetormeter noise

%% choose Scenario
scenario = {'Singularity' 'Random'};
iSce = 2; % choose Scenario    

% load measurements and ground truth
if(iSce == 2)
    rng('shuffle')
    [QGT, Gyr, Acc, Mag] = simulateRandomIMUdata( y.n_samp+1 );
else
    [QGT, Gyr, Acc, Mag] = loadSingularitydata();
end

%% choose Parametrization
parametrizations = { 'Quaternion' 'RotationVector' ...
                    'ErrorState-RotVec'  'ErrorState-MRP' ...
                    'MRP'  'MRP-reprojected'  'MRP-regularized'};
iParam = 4; % choose Parametrization
[smoother, y] = configureFunctor( parametrizations{iParam} , smoother, y );

% add noises on measurements
y.gyr = Gyr(1:y.n_samp,:)   + smoother.Q(1,1)* randn(y.n_samp,3);
y.acc = Acc(1:y.n_samp+1,:) + smoother.Ra(1,1)*randn(y.n_samp+1,3);
y.mag = Mag(1:y.n_samp+1,:) + smoother.Rm(1,1)*randn(y.n_samp+1,3);
y.QT = QGT; % Groundtruth


% initialize prior
prioroffDegree = 5;
axisoff = generateRandomAxis( );
y.qprior = quatprod(QGT(1,:), axisAngle2quat(axisoff *pi /180 *prioroffDegree)) ; 
Pqprior= prioroffDegree *eye(3);

% add Priors and weights to functor
smoother = addPrior(smoother , y.qprior, Pqprior);
smoother = addWeight(smoother, y) ;

% compute inital optimization state
initDeviation = 15; % [ 5 15 30 45 60: 30:180];
if iSce==2
    y.Q = quatprod(QGT(1:y.n_samp+1,:), axisAngle2quat( repmat(generateRandomAxis( 1),y.n_samp+1,1 ) *pi /180 *initDeviation) );
else
    y.Q = quatprod(QGT(1:y.n_samp+1,:), axisAngle2quat( generateRandomAxis( y.n_samp+1) *pi /180 *initDeviation) );
end

x = computeInitialState(parametrizations{iParam}, y.Q  );

%% START OPTIMIZATION
maxiter = 100;
smoother.verbose = false;
smoother.linesearch = true;

result = WLSoptimize('GaussNewton', x, y, smoother, maxiter);

dispresult = @(out) disp( [ out.name '  iteration:' num2str(out.iter) ' cost: ' num2str(out.cost(out.iter)) ' normgrad: ' num2str(out.normGrad(out.iter))]);
dispresult(result)

%% plot error and cost
figure
subplot 211
errorangle = NaN(result.iter,1);
for iiter = 1 : result.iter
errorangle(iiter) = rms( quat2angle(quatprod( QGT , quatuniInv( reshape(result.Q(iiter,:),4,[]).' ) ) ) * 180/pi);
end
plot(errorangle)
xlabel('iteraton [1] ')
ylabel('RMS Error [°]')
title(result.name)
grid on

subplot 212
plot(result.cost)
xlabel('iteraton [1]')
ylabel('Cost [1]')
grid on

%% plot groundtruth quaternion and estimated one
figure
plot( reshape(result.q, 4,[]).' , '--o', 'LineWidth', 0.7)
hold on
plot(QGT, 'LineWidth', 2)
legend( 'q0-Estim', 'q1-Estim', 'q2-Estim', 'q3-Estim',  'q0-GT', 'q1-GT', 'q2-GT', 'q3-GT')
title([ result.name ' - Estimated and Groundtruth trajectory as Quaternion'])
