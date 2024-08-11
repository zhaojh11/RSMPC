%--------------------------------------------------------------------------
% Function that obtains the new constraints for the predicted states along N 
% by applying chance constraints:
%       Pr[xi(k+j+1) < = xmax_i] > = 1 - epsiloni j = 0, ..., N-1
%       Pr[xi(k+j+1) > = xmin_i] > = 1 - epsiloni j = 0, ..., N-1
% where the new deterministic constraints will be:
%       nmax = [nxmax(k+1); nxmax(k+2); ...; nxmax(k+N)]
%       nmin = [nxmin(k+1); nxmin(k+2); ...; nxmin(k+N)]
%       nmax and nmin are found by means of the evolution of the state error
%           covariance by means of his accumulated density functions
% for all states along N:
%       X = Mx*x(k) + Mv*V
%       nmin <= X <= nmax --------> nmin <= Mx*x(k) + Mv*V <= nmax
% Where
%       Acl: is a stable matrix Acl = A + B*K, size: nx_X_nx
%       G: disturbance matrix, size: nx_X_nw
%       N: prediction horizon
%       w(k): system disturbances vector, size nw_X_1
%             - they are independent (i.i.d.), with zero mean and normal distribution
%               w(k) distN(0,Qw).  Where covariance matrix Qw is size nw_X_nw
%             - may be truncated, e.g. |w(k)| <= wlim.  Where wlim is size nw_X_1
%   `   xmin, xmax: state constraints vectors, size nx_X_1
%       var: initial state variance.  size nx_X_nx, where nx is the state
%            vector length
%       epsilon: vector of allowed state violation probability levels.
%                vector with lenght nx
%--------------------------------------------------------------------------
% The inputs of this function are matrices and constants:
%       Acl, G, Qw, N, xmax, xmin, var and epsilon
% The outputs of the function are the new state constraints along N and total
% chance constraints number:
%       nmax, nmin and numcc
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Author: Edwin Alonso González Querubín
%         https://www.researchgate.net/profile/Edwin_Gonzalez_Querubin
%         https://es.mathworks.com/matlabcentral/profile/authors/15149689
% Research Group: Predictive Control and Heuristic Optimization (CPOH)
%                 http://cpoh.upv.es
% Unversity: Universidad Politécnica de Valencia
%            http://www.upv.es
% Version: Beta
% For new releases and bug fixing of this tool please visit:
% https://es.mathworks.com/matlabcentral/fileexchange/75803
%--------------------------------------------------------------------------

function [nmax, nmin, numcc] = chance_constraints(Acl, G, Qw, N, xmax, xmin, var, epsilon)

%--------------------------------------------------------------------------
    nx = size(Acl,1);
    SIGMA = zeros(N,nx);
    for i = 1:N
        var = Acl*var*Acl'+G*Qw*G';
        SIGMA(i,:) = sqrt(diag(var))';
    end
    nmax = zeros(nx, N);
    nmin = zeros(nx, N);
    numcc = 0;
    for i = 1:N
        for j = 1:nx
            if (isnan(xmax(j))) == 0
                nmax(j,i) = -norminv(1-epsilon(j), -xmax(j), SIGMA(i,j));
                numcc = numcc+1;
            else
                nmax(j,i) = NaN;
            end
            if (isnan(xmin(j))) == 0
                nmin(j,i) = norminv(1-epsilon(j), xmin(j), SIGMA(i,j));
                numcc = numcc+1;
            else
                nmin(j,i) = NaN;
            end
        end
    end
    auxnmax = zeros(N*nx, 1);
    auxnmin = zeros(N*nx, 1);
    for i = 1:N
       auxnmax((i-1)*nx+1:i*nx, 1) = nmax(:,i);
       auxnmin((i-1)*nx+1:i*nx, 1) = nmin(:,i);
    end
    nmax = auxnmax;
    nmin = auxnmin;
    

end
