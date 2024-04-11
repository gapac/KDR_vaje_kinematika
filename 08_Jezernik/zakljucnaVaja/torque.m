function tau = torque(ddq, dq, q, ddp0)

% tau = torque(ddq, dq, q, ddp0)
% Funkcija izracuna inverzni dinamicni model za manipulator s tremi 
% prostostnimi stopnjami. 
% Vhodni vektorji so polozaji v sklepih q, sklepne hitrosti dq, 
% sklepni pospeski ddq in vektor ddp0, ki doloca pospesek baze robota
% Izhod funkcije je vektor sklepnih navorov tau

% mase segmentov robota
m = [0.3 2.5 7.0];               

% vektorji od sklepa i-1 do sklepa i
d1 = 0.0;
dv = 457.29; % Maksimalna vertikalna translacija
d2 = (67 + dv/2)/1000;
dh = 379.7; % Maksimalna horizontalna translacija
d3 = (247 + dh/2)/1000;
r = [[0 0 d1]' [0 0 d2]' [0 0 d3]'];

% vektorji tezisc segmentov glede na lokalni koordinatni sistem
rC = [[0 0 0]' [0 0.25 0]' [0 0.01 -0.45]'];

% vztrajnostne matrike segmentov
I = zeros(3,3,3);    
I(:,:,1) = [0.0001 0 0; 0 0.0002 0; 0 0 0.0002];
I(:,:,2) = [0.16 0 0; 0 0.001 0; 0 0 0.16]; 
I(:,:,3) = [1.89 0 0; 0 1.89 0; 0 0 0.001];

% parametri viskoznega trenja za posamezne sklepe
fv = [115 523 523];

% struktura z vsemi robotskimi parametri (npr. masa prvega segmenta je rob.m(1))
rob = struct('m', m, 'rC', rC, 'r', r, 'I', I, 'fv', fv);

% izracun transformacijskih matrik za segmente
A = zeros(4,4,3);
A(:,:,1) = hdh(q(1), d1,  0,   0);                                   %%% STUDENT %%%
A(:,:,2) = hdh(-pi/2, d2+q(2), 0,   -pi/2);                                   %%% STUDENT %%%
A(:,:,3) = hdh(0, d3+q(3),    0,  0);                                               %%% STUDENT %%%

% vektor v smeri osi z0
z0 = [0 0 1]';

% inicializacija vektorja navora
tau = zeros(3,1);

% dolocitev rotacijskih matrik kot podmatrik matrik A
R = zeros(3,3,3);
R(:,:,1) = A(1:3,1:3,1)  ;                                      %%% STUDENT %%%
R(:,:,2) = A(1:3,1:3,2)  ;                                      %%% STUDENT %%%
R(:,:,3) = A(1:3,1:3,3)   ;                                            %%% STUDENT %%%

% VSE INICIALIZACIJE V NADALJEVANJU SO IZVEDENE ZA VSE TRI SKLEPE IN
% SEGMENTE ROBOTA, ZATO SO UPORABLJENE MATRIKE DIMENZIJ 3x3, KJER VSAK
% STOLPEC PREDSTAVLJA VEKTOR, KI OPISUJE EN SEGMENT ALI SKLEP ROBOTA

% inicializacija vektorjev kotnih hitrosti segmentov
omega = zeros(3,3);

% inicializacija vektorjev kotnih pospeskov segmentov
domega = zeros(3,3);

% inicializacija vektorjev transl. pospeskov koordinatnih sistemov
% segmentov
ddp = zeros(3,3);

% inicializacija vektorjev transl. pospeskov tezisc segmentov
ddpC = zeros(3,3);

% inicializacija vektorjev sile v sklepih
f = zeros(3,3);

% inicializacija vektorjev navora v sklepih
mi = zeros(3,3);

% izracun kinematicnih velicin
for ii = 1:3
    if (ii == 1) % robni pogoj za prvi sklep - rotacijski
        % kotna hitrost segmenta
        omega(:,ii) = R(:,:,ii)' * (omega(:,ii) + dq(ii)*z0);                                     %%% STUDENT %%%
        % kotni pospesek segmenta        
        domega(:,ii) = R(:,:,ii)' * (ddq(ii)*z0);                                    %%% STUDENT %%%
        % translacijski pospesek segmenta        
        ddp(:,ii) =   R(:,:,ii)'*ddp0 + cross(domega(:,ii),r(:,ii)) + cross(omega(:,ii) , (cross(omega(:,ii),r(:,ii))) ) ;                               %%% STUDENT %%%        
    else
        % Translacijska sklepa
        % Kotna hitrost segmenta.
        omega(:,ii) = R(:,:,ii)' * omega(:,ii-1)   ;                             %%% STUDENT %%%
            % Kotni pospesek segmenta.        
        domega(:,ii)= R(:,:,ii)' * domega(:,ii-1) ;                            %%% STUDENT %%%
            % Translacijski pospesek segmenta.
        ddp(:,ii) =   R(:,:,ii)' * (ddp(:,ii-1)+ddq(ii)*z0) + cross(2*dq(ii)*omega(:,ii),R(:,:,ii)'*z0)  + cross(domega(:,ii),r(:,ii))  +  cross(omega(:,ii),cross(omega(:,ii),r(:,ii)));                            %%% STUDENT %%%  
    end
    % translacijski pospesek tezisca segmenta            
        ddpC(:,ii) =  ddp(:,ii) + cross(domega(:,ii),rC(:,ii)) + cross(omega(:,ii) , (cross(omega(:,ii),rC(:,ii))) );                                  %%% STUDENT %%%
end

% izracun dinamicnih velicin
for ii = 3:-1:1
    if (ii==3) % robni pogoj za silo na vrhu
         % Sila, ki deluje na sklep.
                f(:,ii) = R(:,:,ii) * f(:,ii) + m(ii)*ddpC(:,ii);                                 %%% STUDENT %%%
            % Navor, ki deluje na sklep.
                mi(:,ii) = cross(-f(:,ii) , (r(:,ii)+rC(:,ii)) )  +  R(:,:,ii)*mi(:,ii)  +  I(:,:,ii)*domega(:,ii)  +  cross(omega(:,ii) , (I(:,:,ii)*omega(:,ii)));                                %%% STUDENT %%%
    else
          % Sila, ki deluje na sklep.
                f(:,ii) = R(:,:,ii+1) * f(:,ii+1) + m(ii)*ddpC(:,ii) ;                                 %%% STUDENT %%%
            % Navor, ki deluje na sklep.
                mi(:,ii) = cross(-f(:,ii) , (r(:,ii)+rC(:,ii)) )  +  R(:,:,ii+1)*mi(:,ii+1)  +  cross((R(:,:,ii+1)*f(:,ii+1)),rC(:,ii))  +  I(:,:,ii)*domega(:,ii)  +  cross(omega(:,ii) , (I(:,:,ii)*omega(:,ii)));                                %%% STUDENT %%% ;                                %%% STUDENT %%%        
    end
    
    if(ii==1)
        % navor, ki deluje v osi sklepa (obremenjuje motor) - za rotacijski
        % sklep
            tau(ii) = mi(:,ii)'*R(:,:,ii)'*z0  +  fv(ii)*dq(ii);                                     %%% STUDENT %%%
    else
        % navor, ki deluje v osi sklepa (obremenjuje motor) - za
        % translacijska sklepa
            tau(ii) = f(:,ii)'*R(:,:,ii)'*z0  +  fv(ii)*dq(ii);                                     %%% STUDENT %%%
    end
end

% % preverjanje rezultatov
% disp('Izpis za preverjanje:')
% 
% % ddp
% izpis = round(ddp,4);
% izpis=mat2str(izpis,2);
% disp('ddp:')
% disp([newline, izpis, newline])
% 
% % ddpC
% izpis = round(ddpC,4);
% izpis=mat2str(izpis,2);
% disp('ddpC:')
% disp([newline, izpis, newline])
% 
% % f
% izpis = round(f,4);
% izpis=mat2str(izpis,2);
% disp('f:')
% disp([newline, izpis, newline])
% 
% % mi
% izpis = round(mi,4);
% izpis=mat2str(izpis,2);
% disp('mi:')
% disp([newline, izpis, newline])
% 
% % tau
% izpis = round(tau,4);
% izpis=mat2str(izpis,4);
% disp('tau:')
% disp([newline, izpis, newline])
