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

function R = quat2rot(q)
% The matrix Q(q) from Body -> Nav
% oder auch simply R
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   R = [2*(q0^2+q1^2)-1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3)    2*(q0^2+q2^2)-1  2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2)-1];
end
