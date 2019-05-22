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

function [Q] = mrp2quat( T )
[r,c] = size(T);
if c == 3
    Tsq = sum(T.*T,2);
    preterm = 1./(Tsq+1);
    Q = zeros(r, 4);
    Q(:,1)   = preterm.*(1-Tsq);
    Q(:,2:4) = 2* preterm .*T(:,1:3);
elseif r == 3
    Tsq = sum(T.*T,1);
    preterm = 1 ./ (Tsq+1);
    Q = zeros(c, 4);
    Q(:, 1 ) = transpose( preterm .*(1-Tsq) );
    Q(:,2:4) = transpose( 2* preterm .*T(1:3,:) );
else
    error('Dimensions do not match');
end

end