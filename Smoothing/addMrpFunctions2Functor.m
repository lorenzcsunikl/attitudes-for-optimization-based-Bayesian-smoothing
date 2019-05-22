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

function [smomrp] = addMrpFunctions2Functor( smomrp )

% prior
smomrp.pri   = @(ter ,y, it) ( 2 * quatlog( quatprod( quatuniInv(y.qprior), mrp2quat( ter)))  );

% time update functions
smomrp.res_f    = @(a, y, it)( shiftdim( 1/y.T * mrp2axisAngle( mrpprod( mrpinv( a(1:3) ) ,  a(4:6) ) ) - y.gyr(it-1,:) ) );

% measurement update functions
smomrp.res_ha = @(a, y, it) ( shiftdim( y.acc(it,:) ) + mrp2rot(a).' * y.g);
smomrp.res_hm = @(a, y, it) ( shiftdim( y.mag(it,:) ) - mrp2rot(a).' * y.m);

 
smomrp.res_uc =@(a, y, it) ( sum(a.^4)/200^4  ); %exp(sqrt( sum( a.^2))-100));% + a.^4/100^4) ) ); %

%%
smomrp.f  = @smo_f;
smomrp.df = @smo_df;
if isfield(smomrp , 'postprocessing')
    if smomrp.postprocessing
        smomrp.postprocess = @postprocessMRP;
    end
end

smomrp.x2Qvec = @(x, y) reshape( mrp2quat( reshape(x, 3, []).').' , [], 1 ) ;

end