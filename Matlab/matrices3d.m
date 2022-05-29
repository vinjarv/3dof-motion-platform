clear; clc
syms p r L
cp = cos(p); sp = sin(p);
cr = cos(r); sr = sin(r);

P = [L/2 -L/2 0
     L/(2*sqrt(3)) L/(2*sqrt(3)) -L/sqrt(3)
     0 0 0];
rx = [1 0 0
      0 cp -sp
      0 sp cp];
ry = [cr 0 sr
      0 1 0
      -sr 0 cr];
  
p_pr = ry*rx*P;

disp("Z coordinates for motor {1, 2, 3} from roll and pitch:")
pretty(p_pr(3,:))