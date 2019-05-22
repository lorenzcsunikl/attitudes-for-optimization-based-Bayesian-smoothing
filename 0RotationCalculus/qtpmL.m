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

function qpmatL = qtpmL(qL)
[r,c] = size(qL);
if r == 4
  qv = qL(2:4);
  qpmatL = [ qL(1), transpose(-qv); qv, qL(1)*eye(3) + crossMatrix(qv)];
elseif c == 4
  qv = qL(2:4);
  qpmatL = [ qL(1), qv; transpose(-qv), qL(1)*eye(3) + crossMatrix(qv).'];
else 
    error('Dimension mismatch!')
end
end