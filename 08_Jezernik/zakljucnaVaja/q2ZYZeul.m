function X = q2ZYZeul(q)

	% q - Vektor kotnih spremenljivk robota (vhod).
	% x - Lega vrha robota XYZ in orientacija z ZYZ Eulerjevimi koti (izhod).
    % Izracun transformacijskih matrik A.
		A = dirkinA(q);
	% Matrika lege vrha robota.
		T3 = dirkinT3(A);                                              %%% STUDENT %%%

	% Izluscimo ustrezne komponente rotacijske matrike.
		r23 = T3(2,3);                                             %%% STUDENT %%%
		r13 = T3(1,3);                                             %%% STUDENT %%%
		r33 = T3(3,3);                                             %%% STUDENT %%%
		r32 = T3(3,2);                                             %%% STUDENT %%%
		r31 = T3(3,1);       

    % Izracunamo ustrezne Eulerjeve kote.
        fi = atan2(r23,r13) ;                                              %%% STUDENT %%%
        theta = atan2(sqrt(  r13*r13 + r23*r23  ),r33);                                           %%% STUDENT %%%
        psi = atan2(r32,-r31);                                             %%% STUDENT %%%
        
	% Orientacija vrha predstavljena z vektorjem treh ZYZ Eulerjevih kotov.
		ea = [fi, theta, psi];                                              %%% STUDENT %%%

	% Lega vrha robota v smislu polozaja XYZ in orientacije predstavljene z
	% ZYZ Eulerjevimi koti.
		X = [T3(1,4),T3(2,4),T3(3,4),fi,theta,psi]';                                               %%% STUDENT %%%
        
     % preverjanje rezultatov za e.fe
%         izpis = round(X,4);
%         izpis=mat2str(izpis,2);
%         disp('REZULTAT X:')
%         disp([newline, izpis, newline])


                      

