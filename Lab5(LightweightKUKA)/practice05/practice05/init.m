% File di inizializzazione 

% Link lengths
a1 = 1;
a2 = .5;
a3 = .3;
a4 = 1;
a5 = .5;
a6 = .3;
a7 = .1;
% DH parameters table for the two-link planar arm
DH_planar2link = [a1 0 0;
      a2 0 0];
     
% DH parameters table for the three-link planar arm
DH_planar3link = [a1 0 0;
      a2 0 0;
      a3 0 0];
  
% DH parameters table for the anthropomorphic arm
DH_anthrop = [a1 pi/2 0;
          a2 0 0;
          a3 0 0];
      
% Joint variable vector (delete the last element 
% if you choose the two-link planar arm)
theta = [0 pi/4 pi/4]';
%theta = [0 0]'; %-> planar two links 



% Final DH parameters table (replace the first element with the DH parametes table relative to the chosen arm)
% DH = [DH_planar3link theta];

DH = [0 0 0 0;
      0 0 0 0;
      0 0 0.4 0.5;
      0 0 0 0;
      0 0 0.39 0.5;
      0 0 0 0;
      0 0 0.078 0.8];
