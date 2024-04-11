function invJ = ijacZYZeul(q)
    %to workspace in plotas plot3(x,y,z)
	% q - Vrednost kotov v sklepih robota (vhod).
	% invJ - Inverzna analiticna Jacobijeva matrika, kjer je orientacija
	%        predstavljena z ZYZ Eulerjevimi koti (izhod).

	% Izracun geometrijske Jacobijeve matrike (Jg).
		Jg = jacobi0(q)                                              %%% STUDENT %%%

	% Izracun lege vrha robota z ZYZ Eulerjevimi koti za opis orientacije.
		X = q2ZYZeul(q);                                               %%% STUDENT %%%
		
	% Iz vektorja X izluscimo ustrezne komponente Eulerjevih kotov.
		phi      = X(4);                                         %%% STUDENT %%%
		theta   = X(5);                                         %%% STUDENT %%%
		psi     = X(6);                                         %%% STUDENT %%%

	% Izracun transformacijske matrike za pretvorbo iz geometrijske v
	% analiticno Jacobijevo matriko.
                                          %%% STUDENT %%
		Tr= [1 0 0 ;
             0 1 0 ;
             0 0 1 ]
             

    %Tr(4:end, 4:end) = T0                                              %%% STUDENT %%%

	% Izracun inverzne analiticne Jacobijeve matrike.
		invJ = inv(inv(Tr)*Jg);                                            %%% STUDENT %%%
        
    % preverjanje rezultatov za e.fe
%         izpis = round(invJ,4);
%         izpis=mat2str(izpis,2);
%         disp('REZULTAT invJa:')
%         disp([newline, izpis, newline])