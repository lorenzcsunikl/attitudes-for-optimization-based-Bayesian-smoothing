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

function [angle] = quat2angle( Q ) 
[r,c] = size(Q);
ang = @(q)( 2*acos( abs(q(:,1)) ));
if c == 4
angle = ang(Q);
elseif r == 4
angle = ang(Q');
else 
    error('dimension do not match')
end
end