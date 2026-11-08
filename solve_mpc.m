function [force, maxResidual] = solve_mpc(x, previousForce, reference, ...
    P, T, L, H, weightRatio, uMin, uMax, yMax, options)
%SOLVE_MPC Solve one position-tracking MPC optimization problem.
%
% The decision variables are changes in force over the next 25 samples.
% Predictions: Y = P*[x; previousForce] + T*du.
% Future forces: U = previousForce + L*du.
%
% Cost: sum(Qy*(predictedPosition-reference)^2 + Rdu*du^2).
% The entire cost is divided by Rdu; the optimal force is unchanged.

    % Predict future position if the force stays at its previous value.
    freePosition = P*[x; previousForce];
    gradient = 2*weightRatio*T'*(freePosition - reference);

    % Force must remain between uMin and uMax at every predicted sample.
    upperForce = (uMax-previousForce)*ones(size(L,1),1);
    lowerForce = (previousForce-uMin)*ones(size(L,1),1);

    % Also enforce the original sampled-position upper bound.
    upperPosition = yMax*ones(size(T,1),1)-freePosition;

    Aineq = [ L;
             -L;
              T ];
    bineq = [upperForce;
             lowerForce;
             upperPosition];

    [moves, ~, exitflag] = quadprog( ...
        H, gradient, Aineq, bineq, [], [], [], [], [], options);

    if exitflag <= 0 || isempty(moves)
        error('MPC could not find a valid input. Solver flag: %d.', exitflag);
    end

    % Check the numerical constraints, then apply the FIRST change only.
    maxResidual = max(0, max(Aineq*moves - bineq));
    if maxResidual > 1e-5
        error('MPC solution violates a constraint by %.3g.', maxResidual);
    end
    force = previousForce + moves(1);
end
