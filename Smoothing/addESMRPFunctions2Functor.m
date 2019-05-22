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

function [smodevmrp] = addESMRPFunctions2Functor( smodevmrp )

% prior
smodevmrp.pri   = @(a ,y, it) ( 2 * quatlog( quatprod( quatuniInv(y.qprior), quatprod(mrp2quat( a) , y.Q(1,:)))) );

% time update functions
smodevmrp.res_f = @(a, y, it)( shiftdim( 2/y.T * quatlog( quatprod( quatuniInv( quatprod( mrp2quat( a(1:3)), y.Q(it-1,:) ) ) , quatprod( mrp2quat( a(4:6)), y.Q(it,:) ) ) ) - y.gyr(it-1,:) ) );

% measurement update functions
smodevmrp.res_ha = @(a, y, it) ( shiftdim( y.acc(it,:) ) + quat2rot( quatprod(mrp2quat( a) , y.Q(it,:)) ).' * y.g);
smodevmrp.res_hm = @(a, y, it) ( shiftdim( y.mag(it,:) ) - quat2rot( quatprod(mrp2quat( a) , y.Q(it,:)) ).' * y.m);

smodevmrp.f  = @smo_f;
smodevmrp.df = @smo_df;
smodevmrp.postprocess = @postprocessDevTer;

smodevmrp.postprocessing = true;
smodevmrp.x2Qvec = @(x,y) reshape( y.Q.', [], 1 );
end