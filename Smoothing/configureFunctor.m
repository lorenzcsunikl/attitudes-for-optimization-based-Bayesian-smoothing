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

function [smoother, y] = configureFunctor( name , smoother, y )

if strcmp(name, 'Quaternion')
    smoother.name = 'Quaternion';
    smoother.regularization = true;
    smoother.E = 10^-5;
    smoother.postprocessing = true;
    smoother = addQuatFunctions2Functor( smoother );
    y.stateSize = 4;
    
elseif strcmp(name, 'RotationVector')
    smoother.name = 'RotationVector';
    smoother = addRotVecFunctions2Functor(smoother);
    y.stateSize = 3;
    
elseif strcmp(name, 'ErrorState-RotVec')
    smoother.name = 'ErrorState-RotVec';
    smoother= addESRotVecFunctions2Functor(smoother);
    y.stateSize = 3;
    
elseif strcmp(name, 'ErrorState-MRP')
    smoother.name = 'ErrorState-MRP';
    smoother= addESMRPFunctions2Functor(smoother);
    y.stateSize = 3;
    
elseif strcmp(name, 'MRP')
    smoother.name = 'MRP';
    smoother.postprocessing = false;
    smoother = addMrpFunctions2Functor(smoother);
    y.stateSize = 3;
    
elseif strcmp(name, 'MRP-reprojected')
    smoother.name = 'MRP-reprojected';
    smoother.postprocessing = true;
    smoother.reproject = true;
    smoother = addMrpFunctions2Functor(smoother);
    y.stateSize = 3;
    
elseif strcmp(name, 'MRP-regularized')
    smoother.name = 'MRP-regularized';
    smoother.E = 10^0;
    smoother.regularization = true;
    smoother.postprocessing = true;
    smoother = addMrpFunctions2Functor(smoother);
    y.stateSize = 3;
    
else
    error('Parametrization not known')
end

end