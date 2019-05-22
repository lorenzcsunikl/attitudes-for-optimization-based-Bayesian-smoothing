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

function qpmatR = qtpmR(qR)
  q0 = qR(1);
  qv = qR(2:4);
  qvcrM = [ 0   -qv(3) qv(2);
          qv(3)   0   -qv(1);
         -qv(2) qv(1)   0];
  qpmatR = [ q0, transpose(-qv); qv, [q0 0 0; 0 q0 0; 0 0 q0]-qvcrM];
end