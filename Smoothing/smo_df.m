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

function [jac] = smo_df(x, y, smo)
ressize = 3;

nres = ressize* ( 1 + 2*y.n_samp + y.n_magres );
nx =  y.stateSize*(y.n_samp+1);
regularization = false;
if isfield(smo, 'regularization')
   if smo.regularization
      nres = nres + y.n_samp + 1;
      regularization = true;
   end
end
nzmax = nres*nx;
jac = spalloc( nres , nx, nzmax );

%% functions to handle indicies
increaseLastrindex = @(ri)(ri+ressize);
getRindex = @(lri) ( lri+1 : lri + ressize);
getsTindex = @(it) (it-2)*y.stateSize+1 : y.stateSize*it;
getsMindex = @(it) (it-1)*y.stateSize+1 : y.stateSize*it;

df_f = smo.res_f;
df_ha = smo.res_ha;
df_hm = smo.res_hm;

%% prior);
jac(1:ressize, 1:y.stateSize) = sparse(numericalJacobian( x(1:y.stateSize), @ smo.pri , y, 1, 10^(-6) ) );
lastrindex = ressize;
% acc measurement
t = 1;
rindex = getRindex( lastrindex);
sindex_m = getsMindex(t);
jac(rindex, sindex_m ) = sparse( numericalJacobian(x(sindex_m), df_ha, y, t, 10^(-6) ) ) ;  
lastrindex = increaseLastrindex( lastrindex);
if regularization  
    rindex =  lastrindex +1 ;
    sindex_m = getsMindex(t);
    jac(rindex, sindex_m ) = sparse( numericalJacobian(x(sindex_m), smo.res_uc, y, t, 10^(-6) ) ) ; 
    lastrindex = lastrindex +1;
end

%% dynamic and measurement functions
imag = 0;
for t = 2 : y.n_samp+1
rindex = getRindex( lastrindex);
sindex_t = getsTindex(t);
jac(rindex, sindex_t ) = sparse(numericalJacobian(x(sindex_t), df_f, y, t, 10^(-6) ) ); %smo.A( x(sindex_t), y, t) );
lastrindex = increaseLastrindex( lastrindex);

rindex = getRindex( lastrindex);
sindex_m = getsMindex(t);
jac(rindex, sindex_m ) = sparse( numericalJacobian(x(sindex_m), df_ha, y, t, 10^(-6) ) ) ;  % jac(rindex, sindex_m ) = sparse( smo.H_a( x(sindex_m) ) );
lastrindex = increaseLastrindex( lastrindex);

if (y.n_magres > 0 && y.n_magres > imag && t == floor( y.n_samp / (y.n_magres+1)  * (imag+1) ) )
    rindex = getRindex( lastrindex);
    sindex_m = getsMindex(t);
    jac(rindex, sindex_m ) = sparse( numericalJacobian(x(sindex_m), df_hm, y, t, 10^(-6) ) ) ; % jac(rindex, sindex_m ) = sparse( smo.H_a( x(sindex_m) ) );
    lastrindex = increaseLastrindex( lastrindex);
    imag = imag +1;
end
if regularization  
    rindex =  lastrindex +1 ;
    sindex_m = getsMindex(t);
    jac(rindex, sindex_m ) = sparse( numericalJacobian(x(sindex_m), smo.res_uc, y, t, 10^(-6) ) ) ; % jac(rindex, sindex_m ) = sparse( smo.H_a( x(sindex_m) ) );
    lastrindex = lastrindex +1;
end

end

end