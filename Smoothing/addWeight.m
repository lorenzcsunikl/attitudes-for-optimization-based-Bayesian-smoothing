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

function [functor] = addWeight(functor, y)
ressize = 3;
nres = ressize* ( 1 + 2*y.n_samp + y.n_magres );

regularization = false;
if isfield(functor, 'regularization')
   if functor.regularization
      nres = nres + y.n_samp + 1;
      regularization = true;
   end
end

if isfield(functor, 'weight')
    functor = rmfield(functor, 'weight');
end

functor.weight = spalloc(nres, nres, nres + nres.^2 -nres );

blocki = zeros(nres);
weighti = functor.Pprior \eye(ressize);

blocki(1:ressize, 1:ressize) = 0.5* (weighti * weighti.');
imag = 0;
index = ressize + 1: ressize + ressize;
nextindex = @(idx) idx +ressize;

blocki(index, index) = functor.Ra \eye(ressize);
index = nextindex(index);
if regularization
    blocki(index(1), index(1)) = functor.E \eye(1);
    index = index+1;
end
    
for t = 2 : y.n_samp+1
    blocki(index, index) = functor.Q \eye(ressize);
    index = nextindex(index);
    
    blocki(index, index) = functor.Ra \eye(ressize);
    index = nextindex(index);
    
    if (y.n_magres > 0 && imag < y.n_magres  && t == floor( y.n_samp / (y.n_magres+1)  * (imag+1) ) )
        blocki(index, index) = functor.Rm \eye(ressize);
        index = nextindex(index);
        imag = imag +1;
    end
    
    if regularization
        blocki(index(1), index(1)) = functor.E \eye(1);
        index = index+1;   
    end
end
weighti = sparse(blocki);
functor.weight = 0.5*(weighti+weighti');

end