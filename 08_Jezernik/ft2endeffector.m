function fte = ft2endeffector(q, ft)

    % Funkcija preracuna sile in navore, ki jih izmeri senzor v koordinatni
    % sistem, ki je pritrjen na vrhu robota v rocki.
    % Vhodna vektorja sta polozaji v sklepih q in sile ter navori, ki jih
    % izmeri senzor ft.
    % Izhod funkcije je vektor sil in navorov izrazen v koordinatnem sistemu
    % rocke fte.

    % Transformacija od k.s. vrha robota do k.s. senzorja.
        t6F = [eye(3),[0; 0; 0.0395]; [0 0 0 1]];

    % Transformacija od k.s. senzorja do k.s. rocke.
        tFE = [0 0 -1 -0.17; 0 1 0 0; 1 0 0 0.0355;0 0 0 1]; 

    % Izracunajte transformacijsko matriko T za transformacijo sil in 
    % navorov med koordinatnim sistemom senzorja in rocke.
        inv_TFE = inv(tFE);
    %     
        r = inv_TFE(1:3,4);
        R = inv_TFE(1:3,1:3) ;
                                                %%% STUDENT %%%
        Sr= [0         -r(3)   r(2);
               r(3)    0         -r(1);
               -r(2)   r(1)    0] ;
            
        T = [R       zeros(3,3);
             Sr*R    R];
    % Izhodni vektor sil in navorov.
        fte = T*ft ;                                     %%% STUDENT %%%