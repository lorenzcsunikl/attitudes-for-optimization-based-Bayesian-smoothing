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

function [ter] = quat2mrp( Q )
[r,c] = size(Q);
if c == 4
    ter = zeros(r,3);
    ter(:,1:3) = Q(:,2:4) ./ (Q(:,1)+1);
elseif r == 4
    ter = zeros(c,3);
    ter(:,1:3) = transpose( Q(2:4,:) ./ (Q(1,:)+1 ) );
else
    error('Dimensions do not match');
end
end