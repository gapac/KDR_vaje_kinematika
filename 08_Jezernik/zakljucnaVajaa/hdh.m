function A = hdh(th, d, a, al)

	% th, d, a, al - Denavit-Hartenbergovi parametri robota (vhod) - po vrsti:
	% Theta(rotz), d(transz), a(transx), Alpha(rotx).
	% A - homogena transformacijska matrika (izhod).

	% Kosinus kota THETA.
		ct = cos(th); 
	% Sinus kota THETA.
		st = sin(th); 
		
	% Matrika rotacije okrog osi z.
		rz = [ct -st 0 0
              st ct 0 0
              0 0 1 0 
              0 0 0 1];                                              %%% STUDENT %%%

	% Matrika translacije vzdolz osi z in x.
		trans = [1 0 0 a
                 0 1 0 0
                 0 0 1 d
                 0 0 0 1];                                           %%% STUDENT %%%
		
	% Kosinus kota ALPHA.
		cal = cos(al); 
	% Sinus kota ALPHA.
		sal = sin(al); 
		
	% Matrika rotacije okrog osi x.
		rx = [1 0 0 0
              0 cal -sal 0
              0 sal cal 0
              0 0 0 1];                                              %%% STUDENT %%%
		
	% Izracun homogene transformacijske matrike.
		A = rz*trans*rx;                                               %%% STUDENT %%%
        
        
% preverjanje rezultatov za e.fe
    izpis=mat2str(A,2);
    disp('REZULTAT HDH:')
    disp([newline, izpis, newline])
    
    
    
    
    
    
    
    
   