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

function x = computeInitialState(name, Q  )
if strcmp(name, 'Quaternion')
    x = reshape( Q.', [ ], 1 );
    
elseif strcmp(name, 'RotationVector')
    x = reshape( quat2axisangle( Q ).', [ ], 1 ) ;
    
elseif ( strcmp(name, 'ErrorState-RotVec') || strcmp(name, 'ErrorState-MRP') )
    x = reshape( zeros( length(Q),3), [], 1 ) ;

elseif ( strcmp(name, 'MRP') || strcmp(name, 'MRP-reprojected') || strcmp(name, 'MRP-regularized') )
    x = reshape( quat2mrp( Q ).', [ ], 1 ) ;
    
else
    error('Parametrization not known')
end

end

