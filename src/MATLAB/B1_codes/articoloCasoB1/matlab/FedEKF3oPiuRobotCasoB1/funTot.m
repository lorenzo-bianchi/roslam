function resto = funTot(x,pos,rhoVett)

deltaPos = pos-x;
resto = diag(deltaPos'*deltaPos) - rhoVett.^2';