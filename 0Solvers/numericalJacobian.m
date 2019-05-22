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

function [jac] = numericalJacobian(x, f, y, it, h)
x = x(:);
if(nargin()<5)
h = 1e-7;
end
z = shiftdim( f(x, y, it) );
jac = NaN( length(z), length(x));

for ii = 1: length(x)
    xh = x;
    xh(ii) = xh(ii) + h;
    zh = shiftdim( f( xh, y, it) );
    jac(:, ii) = (zh - z )./h ;
end

end