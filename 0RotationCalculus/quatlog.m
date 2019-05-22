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

function [ logq ] = quatlog( Q )
% log of an unitquaternion:
[r,c] = size(Q);
if c == 4
    dev1 = Q(:,1);
    dev1(dev1==0) = 1;
    dev2 = vecnorm(Q(:,2:4),2,2);
    dev2( dev2 == 0) = 1;
    logq = qtloghelper(Q,dev1, dev2);
elseif r == 4
    dev1 = transpose( Q(1,:) );
    dev1(dev1==0) = 1;
    dev2 = transpose(vecnorm(Q(2:4,:),2,1) );
    dev2( dev2 == 0) = 1;
    logq = qtloghelper(Q',dev1, dev2);
else 
    error('dimension do not match')
end

end

function a = qtloghelper( q, devAng, devVec)
a = ( q(:,1) ./ abs(devAng) .* q(:,2:4)./ devVec .* asin( devVec));
end

