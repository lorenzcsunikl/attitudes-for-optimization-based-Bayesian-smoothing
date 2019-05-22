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

function [res] = smo_f(x,y,smo)
% [res] = smo_quat_f(x,y,smo)
statesize = y.stateSize;
ressize = 3;
nres = ressize + ressize*(2*y.n_samp+1) +y.n_magres * ressize;

regularization = false;
if isfield(smo, 'regularization')
   if smo.regularization
      nres = nres + y.n_samp + 1;
      regularization = true;
   end
end


res = zeros( nres, 1);

% functions to handle indices
increaseLastrindex = @(ri)(ri+ressize);
getRindex = @(lri) ( lri+1 : lri + ressize);
getsTindex = @(it) (it-2)*statesize+1 : statesize*it;
getsMindex = @(it) (it-1)*statesize+1 : statesize*it;


imag=0;
lastrindex = ressize;

%prior
t = 1;
res(1:ressize) = smo.pri( x(1:statesize), y, 1 );

% first acc measurement
indexsM = getsMindex(t);
rindex = getRindex( lastrindex);
res(rindex) = smo.res_ha(  x(indexsM), y, t);
lastrindex = increaseLastrindex( lastrindex);

if regularization
    rindex = lastrindex+1;
    res(rindex) = smo.res_uc( x(indexsM), y, t);
    lastrindex = lastrindex+1;
end

%dynamics and measurements
for t = 2 : y.n_samp+1

indexsT = getsTindex(t);
indexsM = getsMindex(t);

rindex = getRindex( lastrindex);
res(rindex) = smo.res_f(  x(indexsT), y, t);
lastrindex = increaseLastrindex( lastrindex);

rindex = getRindex( lastrindex);
res(rindex) = smo.res_ha( x(indexsM), y, t);
lastrindex = increaseLastrindex( lastrindex);

if (y.n_magres > 0 && y.n_magres > imag &&  t == floor( y.n_samp / (y.n_magres+1)  * (imag+1) ) ) 
    rindex = getRindex( lastrindex);
    res(rindex) = smo.res_hm( x(indexsM), y, t);
    lastrindex = increaseLastrindex( lastrindex);
    imag= imag+1;
end

if regularization
    rindex = lastrindex+1;
    res(rindex) = smo.res_uc( x(indexsM), y, t);
    lastrindex = lastrindex+1;
end

end

end

