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
function [smoaxis] = addRotVecFunctions2Functor( smoaxis )

% prior
smoaxis.pri   = @(a ,y, it) ( 2 * quatlog( quatprod( quatuniInv(y.qprior), axisAngle2quat( a)))  );
% smoaxis.pri   = @(a0, a0pri) ( a0(:) - a0pri(:));

% time update functions
smoaxis.res_f = @(a, y, it)( shiftdim( 2/y.T * quatlog( quatprod( quatuniInv( axisAngle2quat( a(1:3) ) ) , axisAngle2quat( a(4:6) ) ) ) - y.gyr(it-1,:) ) );

% measurement update functions
smoaxis.res_ha = @(a, y, it) ( shiftdim( y.acc(it,:) ) + axisAngle2rot(a).' * y.g);
smoaxis.res_hm = @(a, y, it) ( shiftdim( y.mag(it,:) ) - axisAngle2rot(a).' * y.m);

smoaxis.f  = @smo_f;
smoaxis.df = @smo_df;

smoaxis.x2Qvec = @(x, y) reshape( axisAngle2quat( reshape(x, 3, []).').' , [], 1 ) ;


end
