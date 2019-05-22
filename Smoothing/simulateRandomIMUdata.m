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

function [QT, Gyr, Acc, Mag, T] = simulateRandomIMUdata( n_samp )
% get IMU data and groundtruth accordingly to the random scenario.
if nargin() == 0
    n_samp = 100;
end
f = 100;
T = 1/ f;

upscale = 20;
n_sim = upscale*n_samp;
fsim = upscale*f;
Tsim = 1/fsim;

axis = generateRandomAxis(1)*( rand(1)*pi +pi*1/9) ;
amplitude =  rand(1)*pi/2; 
gyr_sim = repmat(axis, n_sim, 1 ) + amplitude * sin( repmat(linspace(0, rand(1)*10*pi, n_sim), 1, 1).' ).*repmat(generateRandomAxis(1), n_sim,1)  ;
%% integrate 
Qsim = zeros(n_sim, 4);

prioroffDegree = rand(1)*2*pi;
q_init = axisAngle2quat(generateRandomAxis(1) *prioroffDegree );
Qsim(1,:) = q_init(:).';
for ii = 2 : n_sim
    Qsim(ii,:) = quatprod( Qsim(ii-1,:), quatexp(Tsim/2* gyr_sim(ii,:)));
end
%% downsample and add gravity + magnetic field
QT  = Qsim( 1: upscale: end, :);
Gyr = gyr_sim( 1: upscale: end, :);
Gyr(end,:) = [];
Acc = NaN(n_samp-1, 3);
Mag = NaN(n_samp-1, 3);

for ii = 1: n_samp
    Rbn = quat2rot(QT(ii,:)).';
    Acc(ii,:) = transpose( Rbn *[0; 0; 9.81] ) ;
    Mag(ii,:) = transpose( Rbn *[1; 0; 0]) ;
end

end