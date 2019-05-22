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

function [smoquat] = addQuatFunctions2Functor( smoquat )

% prior
smoquat.pri   = @(x,y, it) ( 2 * quatlog( quatprod( quatuniInv(y.qprior), x))  ); 

%% time update functions
smoquat.res_f = @(q, y, it)( shiftdim( 2/y.T * quatlog( quatprod( quatuniInv(q(1:4)) , q(5:8)) ) - y.gyr(it-1,:) ) );

%% measurement update functions
smoquat.res_ha = @(q, y, it) ( shiftdim( y.acc(it,:) ) + quat2rot(q).' * y.g);
smoquat.res_hm =@(q, y, it) ( shiftdim( y.mag(it,:) ) - quat2rot(q).' * y.m);

% regularizationontr
smoquat.res_uc =@(q, y, it) ( norm(q) -1  );


%%
smoquat.f  = @smo_f;
smoquat.df = @smo_df;

if isfield(smoquat , 'postprocessing')
    if smoquat.postprocessing
        smoquat.postprocess = @postprocessQuat;
    end
end

smoquat.x2Qvec = @(x,y) x;

end