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

function Q = quatexp(A)
% exp of an unitquaternion:
 [r,c] = size(A);
if c == 3
    devisioner = vecnorm(A,2,2);
    devisioner( devisioner==0) = 1;
    Q = quatexphelper(A, devisioner);
elseif r == 3
    devisioner = vecnorm(A.',2,2);
    devisioner( devisioner==0) = 1;
    Q = quatexphelper(A.', devisioner);
else 
    error('dimension do not match')
end
end

function q = quatexphelper(a, dev)
q = ( [cos( vecnorm(a,2,2)), sin(vecnorm(a,2,2)) .* a ./ dev] );
end