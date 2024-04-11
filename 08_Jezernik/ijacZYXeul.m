function invJ = ijacZYXeul(q)


	% q - Vrednost kotov v sklepih robota (vhod).
	% invJ - Inverzna analiticna Jacobijeva matrika, kjer je orientacija
	%        predstavljena z ZYX Eulerjevimi koti (izhod).

	% Izracun geometrijske Jacobijeve matrike (Jg).
		Jg = jacobi0(q) ;                                              %%% STUDENT %%%

	% Izracun lege vrha robota z ZYX Eulerjevimi koti za opis orientacije.
		X = q2ZYXeul(q) ;                                             %%% STUDENT %%%
		
	% Iz vektorja X izluscimo ustrezne komponente Eulerjevih kotov.
		phi      = X(4) ;                                               %%% STUDENT %%%
		theta   = X(5)   ;                                            %%% STUDENT %%%
		psi     = X(6)    ;                                           %%% STUDENT %%%

       


	% Izracun transformacijske matrike za pretvorbo iz geometrijske v
	% analiticno Jacobijevo matriko.
        T0= [0 -sin(phi) cos(phi)*cos(theta)
             0 cos(phi)  sin(phi)*cos(theta)
             1     0             -sin(theta)] ;                                              %%% STUDENT %%%

    
		Tr= [1 0 0 0 0 0;
             0 1 0 0 0 0;
             0 0 1 0 0 0;
             0 0 0 0 0 0;
             0 0 0 0 0 0;
             0 0 0 0 0 0];

    Tr(4:end, 4:end) = T0;                                        %%% STUDENT %%%
        
	% Izracun inverzne analiticne Jacobijeve matrike.
    % inverzni Jacobi [6x5]-->psevdo inverz: dq=[J'*J]^-1*J'
		invJ = inv(inv(Tr)*Jg) ;                                      %%% STUDENT %%%