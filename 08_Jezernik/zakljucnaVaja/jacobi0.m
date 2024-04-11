function J = jacobi0(q)

	% q - Vrednost kotov v sklepih robota (vhod).
	% J - Geometrijska Jacobijeva matrika (izhod).

	% Lista transformacijskih matrik A.
		A = dirkinA(q) ;                                               %%% STUDENT %%%

	% Transformacijska matrika lege vrha.
		T3 = dirkinT3(A);                                              %%% STUDENT %%%


	% Inicializacija prazne Jacobijeve matrike.
		Jp = zeros(3,3); % pozicijska podmatrika			    %%% STUDENT %%%
		Jo = zeros(3,3); % orientacijska podmatrika		    %%% STUDENT %%%

	% Inicializacija spremenljivk.
		z0 = [0,0,1]'; % vektor v smeri osi z_0
		p0 = [0,0,0]'; % vektor polozaja prvega sklepa glede na referencni koordinanti sistem
		Tn = eye(4); % zacetna vrednost matrike Tn
        
        z3 = T3(1:end-1,3);
        p3 = T3(1:end-1,4);

	%%% STUDENT %%%
	% 1. STOLPEC JACOBIJEVE PODMATRIKE

		% Izracun 1. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,1) = cross(z0, (p3-p0));                                        %%% STUDENT %%%
		% Izracun 1. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,1) = z0;                                        %%% STUDENT %%%

    % 2. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
		Tn = A(:,:,1);                                             %%% STUDENT %%%
		% Izracun 2. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,2) = Tn(1:3,3);                                        %%% STUDENT %%%
		% Izracun 2. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,2) = 0; 
        
    % 3. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
		Tn = Tn* A(:,:,2);                                             %%% STUDENT %%%
		% Izracun 3. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,3) = Tn(1:3,3);                                        %%% STUDENT %%%
		% Izracun 3. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,3) = 0;    
        
	% Celotna Jacobijeva matrika je sestavljena iz obeh podmatrik.
		J = Jp;											    %%% STUDENT %%%
        
     % preverjanje rezultatov za e.fe
%         izpis = round(J,4);
%         izpis=mat2str(izpis,2);
%         disp('REZULTAT Jg:')
%         disp([newline, izpis, newline])
        
        
        
        
        
        
        
        
        