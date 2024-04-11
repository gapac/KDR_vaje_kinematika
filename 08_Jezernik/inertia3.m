function B = inertia3(q)

    % Funkcija izracuna vztrajnostno matriko za robot s tremi segmenti.
    % Vhod je vektor kotov v sklepih q.

    % Inicializacijs vztrajnostne matrike.
        B = zeros(3,3);

    % Upostevamo pogoj za pospesek, ki deluje na bazo robota.
        ddp0 = [0 0 0]';                                            %%% STUDENT %%%

    % Upostevamo pogoj za kotne pospeske v sklepih.
        dq = [0 0 0];                                              %%% STUDENT %%%

    
    % Izracunamo prvi stolpec vztrajnostne matrike B, pri cemer uporabimo
    % funkcijo za izracun inverzne dinamike robota za izracun navorov, 
    % ki predstavljajo stolpec matrike vztrajnosti.
        ddq = [1 0 0];                                             %%% STUDENT %%%
        B(:,1) = torque(ddq, dq, q, ddp0) ;                                           %%% STUDENT %%%

    % Izracunamo drugi stolpec vztrajnostne matrike B.
        ddq = [0 1 0];                                             %%% STUDENT %%%
        B(:,2) = torque(ddq, dq, q, ddp0);                                          %%% STUDENT %%%
        
    % Izracunamo tretji stolpec vztrajnostne matrike B.
        ddq = [0 0 1];                                             %%% STUDENT %%%
        B(:,3) = torque(ddq, dq, q, ddp0);                                          %%% STUDENT %%%
end