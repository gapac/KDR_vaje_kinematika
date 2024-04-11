 function X = q2ZYXeul(q)

	% q - Vektor kotnih spremenljivk robota (vhod).
	% x - Lega vrha robota XYZ in orientacija z ZYX Eulerjevimi koti (izhod).

    % Izracun transformacijskih matrik A.
		A = dirkinA(q);
	% Matrika lege vrha robota.
		T6= dirkinT6(A);                                              %%% STUDENT %%%

	% Izluscimo ustrezne komponente rotacijske matrike.
		r21 = T6(2,1);                                             %%% STUDENT %%%
		r31 = T6(3,1);                                             %%% STUDENT %%%
		r32 = T6(3,2);                                             %%% STUDENT %%%
		r33 = T6(3,3);                                             %%% STUDENT %%%
		r11 = T6(1,1);                                             %%% STUDENT %%%

        %mogoce dodaj enacbo z tan2
        %phi = atan((T6(2,3)*T6(3,1)  -  T6(3,3)*T6(2,1))  /  (T6(3,3)*T6(2,2)  -  T6(2,3)*T6(3,2)))
        %theta = atan((-(T6(1,3))) / (sqrt(  0.5* ( T6(1,1)^2+T6(1,2)^2+T6(2,3)^2+T6(3,3)^2 )  )))
        %psi = atan((T6(1,2)*T6(3,1)  -  T6(1,1)*T6(3,2))  /  (T6(1,1)*T6(2,2)  -  T6(1,2)*T6(2,1)))
        
        phi = atan2(r21,r11);
        theta = atan2(-r31,sqrt(  r32*r32 + r33*r33  ));
        psi = atan2(r32,r33);
        
        
        
        
	% Orientacija vrha predstavljena z vektorjem treh ZYX Eulerjevih kotov.
		ea = [phi, theta, psi];                                              %%% STUDENT %%%

	% Lega vrha robota v smislu polozaja XYZ in orientacije predstavljene z
	% ZYX Eulerjevimi koti.
		X = [T6(1,4),T6(2,4),T6(3,4),phi,theta,psi]';                                               %%% STUDENT %%%


