function ftb = ft2base(q, ft)

    % Funkcija preracuna sile in navore, ki jih izmeri senzor v koordinatni
    % sistem, ki je pritrjen v bazi robota.
    % Vhodna vektorja sta polozaj v sklepih q in sile in navori, ki jih
    % izmeri senzor ft.
    % Izhod funkcije je vektor sil in navorov izrazen v koordinatnem sistemu
    % baze robota ftb.

    % Transformacija od k.s. vrha robota do k.s. senzorja.
        t6F = [eye(3),[0; 0; 0.0395]; [0 0 0 1]];

    % Transformacija od k.s. senzorja do k.s. rocke.
        tFE = [0 0 -1 -0.17; 0 1 0 0; 1 0 0 0.0355;0 0 0 1] ;
    
    % Lista transformacijskih matrik A.
        A = dirkinA(q);
                                      %%% STUDENT %%%

    % Transformacijska matrika lege vrha.
        T6 = dirkinT6(A)   ;                                            %%% STUDENT %%%

    % Izracunajte transformacijsko matriko T za transformacijo sil in 
    % navorov med koordinatnim sistemom senzorja in baze robota.        
        TFE = T6*t6F;
    %     
        r = TFE(1:3,4)
        R = TFE(1:3,1:3) ;
                                                %%% STUDENT %%%
        Sr= [0         -r(3)   r(2);
               r(3)    0         -r(1);
               -r(2)   r(1)    0] ;
            
        T = [R       zeros(3,3);
             Sr*R    R]      ;                                       %%% STUDENT %%%
                                                                                          
    % Izhodni vektor sil in navorov.
        ftb = T*ft ;                                             %%% STUDENT %%%