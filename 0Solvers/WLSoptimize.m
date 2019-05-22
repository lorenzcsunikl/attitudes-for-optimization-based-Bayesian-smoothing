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

function [out] = WLSoptimize( optimizer, x, y, functor, maxiter, toldx, tolcost)
% [out] = WLSoptimize( optimizer, x, y, functor, maxiter, toldx, tolcost)
if nargin() < 7
    tolcost = 10^-2;
end
if nargin() < 6
    toldx = 10^-2;
end
if nargin() < 5
    maxiter = 100;
end

if isfield(functor, 'verbose')
    if functor.verbose
        disp( [ 'optimizing: ' functor.name ])
    end
end
warning ('off','all')
out.cost = NaN(maxiter,1);
out.normGrad = NaN(maxiter+1, 1);

X = NaN(maxiter+1, numel(x) );
Q = NaN(maxiter+1, (y.n_samp+1)*4 );

X(1, :) = x(:).';
Q(1, :) = functor.x2Qvec(x,y).';

iter     = 0;
normgradi = 100 * toldx;
costdiff = 100 * tolcost;

beta = 0.7;
maxline = 0.01;
levi = 1;
betalevi = 0.9;
tic;
while (  toldx < normgradi && maxiter > iter && tolcost < costdiff)
    
    resvec   = functor.f (x, y, functor);
    jac_res  = functor.df(x, y, functor);
    
    if isfield(functor, 'weight')
        out.cost(iter+1) = resvec.'  * functor.weight * resvec;
        hess = jac_res.' * functor.weight * jac_res;
        grad = jac_res.' * functor.weight * resvec;
    else
        out.cost(iter+1) = resvec.'  *resvec;
        hess = jac_res.' * jac_res;
        grad = jac_res.' * resvec;
    end
    
    if  strcmp( optimizer, 'GradientDescent')
        dx = -grad;
    elseif strcmp( optimizer, 'GaussNewton' ) 
        dx = (hess)\(-grad);
    elseif strcmp( optimizer,'ModGaussNewton')
        dx = (hess + levi* eye(size(hess)) )\(-grad);
        levi = betalevi*levi;
    else
        error('Choose correct optimizer: GaussNewton, GradientDescent or ModGaussNewton')
    end
    
    if isfield(functor, 'linesearch')
        if (functor.linesearch == 1)
            alpha = 1.0;
            gamma = 0.25;
            iline = 0;
			resvec = functor.f(x+alpha*dx, y, functor);
			if isfield(functor, 'weight')
              costline = resvec.'  * functor.weight * resvec;
            else
              costline= resvec.'  *resvec;
            end
            costtresh = out.cost(iter+1) + alpha * gamma * grad.' * dx;
				
            while( costline > costtresh && alpha > maxline)
                xl = x + alpha*dx;
                resvec = functor.f(xl, y, functor);
                
                if isfield(functor, 'weight')
                    costline = resvec.'  * functor.weight * resvec;
                else
                    costline= resvec.'  *resvec;
                end
                costtresh = out.cost(iter+1) + alpha * gamma * grad.' * dx;
                alpha = alpha * beta;
                iline = iline + 1;
            end
            dx = alpha*dx;
        end
    end % linesearch
    x = x + dx;
    
    if isfield(functor, 'postprocessing')
        if functor.postprocessing
            [x, functor, y] = functor.postprocess(x, functor, y);
        end
    end
    
    
    X(iter+2, : ) = x(:).';
    Q(iter+2, :)  = functor.x2Qvec(x,y);
    normgradi = norm( grad);
    out.normGrad(iter+1) = normgradi;
    if(iter ~=0)
        costdiff = abs( diff( out.cost(iter:iter+1) ) ) ;
    end
    iter = iter +1 ;
    if isfield(functor, 'verbose')
        if functor.verbose
            disp( [ 'iteration: ' num2str(iter) '       cost: ' num2str(out.cost(iter)) '       normgrad: ' num2str(normgradi) '  > toldx:' num2str(toldx) ])
        end
    end
end % iterative optimization
out.runtime = toc;

% why optimization ended
if( toldx > normgradi )
    out.wlsend = 1;
elseif (tolcost > costdiff)
    out.wlsend = 0;
else
    out.wlsend = -1;
end

out.X  = X(1:iter+1, :);
out.Q  = Q(1:iter+1, :);
out.q  = Q(iter+1, :).';
out.cost = out.cost(1:iter);
out.name = functor.name;


out.x = x;
out.iter = iter;
out.y = y;
end
