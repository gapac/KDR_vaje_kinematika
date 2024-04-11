function A = dirkinA(q)

	% q - Vektor kotnih spremenljivk za sklepe (vhod).
	% A(:,:,1) ... - Lista matrik A (izhod).

	% Paramteri robota.
    d1 = 0.0; 				% Enote so metri.
    dv = 457.29; 			% Maksimalna vertikalna translacija.
    d2 = (67 + dv/2)/1000;	% Enote so metri.
    dh = 379.7; 			% Maksimalna horizontalna translacija.
    d3 = (247 + dh/2)/1000;	% Enote so metri.

    A = zeros(4,4,3);
    A(:,:,1) = hdh(q(1), d1,  0,   0);                                   %%% STUDENT %%%
    A(:,:,2) = hdh(-pi/2, d2+q(2), 0,   -pi/2);                                   %%% STUDENT %%%
    A(:,:,3) = hdh(0, d3+q(3),    0,  0);                                   %%% STUDENT %%%

