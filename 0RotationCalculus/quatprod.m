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

function [QP] = quatprod(Q, P)
% quaternion product q left quat and p right quat Q 4xnsamp
[r ,c ] = size(Q);
P = real(P);
if (r==4 && c == 1) || (r==1 && c == 4)
    DQ = qtpmL(Q(:));
    QP = DQ * P(:);
    QP = QP./norm(QP);
elseif c == 4
    nc = r;
    QP = zeros(r,c);
    for iq = 1:nc
        QP(iq,:) =  P(iq,:)*qtpmL(Q(iq,:));
    end
    QP = normr(QP);
else
    nc = c;
    QP = zeros(c,r);
    for iq = 1:nc
        QP(iq,:) = transpose( qtpmL(Q(:,iq)) *P(:,iq) );
    end
    QP = normv(QP);
end
end