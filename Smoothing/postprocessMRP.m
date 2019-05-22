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

function [x, smot, y] = postprocessMRP(x, smot, y)



%% reprojecting
if isfield(smot, 'reproject')
    if smot.reproject
        X = reshape(x,3,[]);
        dotX = sum(X.^2);
        boundary = 1;
        idxm = dotX >  boundary ;
        if any( idxm )
            X(:,idxm) = - X(:,idxm) ./ dotX(idxm);
        end
        x = reshape( X, [], 1);
        return;
    end
end

%% regularizing shift to boundary
X = reshape(x,3,[]);
nrmX = sqrt(sum(X.^2));
boundary = 200;
idxm = nrmX >  boundary ;
if any( idxm )
    X(:,idxm) =  X(:,idxm) ./ nrmX(idxm) * boundary;
end
x = reshape( X, [], 1);

end

