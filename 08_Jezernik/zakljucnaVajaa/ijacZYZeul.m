function invJ = ijacZYZeul(q)

	% q - Vrednost kotov v sklepih robota (vhod).
	% invJ - Inverzna analiticna Jacobijeva matrika, kjer je orientacija
	%        predstavljena z ZYZ Eulerjevimi koti (izhod).

	% Izracun geometrijske Jacobijeve matrike (Jg).
		Jg = ;                                              %%% STUDENT %%%

	% Izracun lege vrha robota z ZYZ Eulerjevimi koti za opis orientacije.
		X = ;                                               %%% STUDENT %%%
		
	% Iz vektorja X izluscimo ustrezne komponente Eulerjevih kotov.
		fi      = ;                                         %%% STUDENT %%%
		theta   = ;                                         %%% STUDENT %%%
		psi     = ;                                         %%% STUDENT %%%

	% Izracun transformacijske matrike za pretvorbo iz geometrijske v
	% analiticno Jacobijevo matriko.
		Tr = ;                                              %%% STUDENT %%%

	% Izracun inverzne analiticne Jacobijeve matrike.
		invJ = ;                                            %%% STUDENT %%%
        
    % preverjanje rezultatov za e.fe
        izpis = round(invJ,4);
        izpis=mat2str(izpis,2);
        disp('REZULTAT invJa:')
        disp([newline, izpis, newline])