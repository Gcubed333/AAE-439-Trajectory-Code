function F = solvePsi(psi,theta,T,L,D,W)
    DvDt = T*cos(psi-theta) - D - W*sin(theta);
    DthetaDt = T*sin(psi-theta) + L - W*cos(theta);

    C1 = T*cos(psi) - L*sin(theta) - D*cos(theta) - DthetaDt*sin(theta);
    C2 = T*sin(psi) + L*cos(theta) - D*sin(theta) - W +...
        DthetaDt*cos(theta);
    
    F(1) = C1 - DvDt*cos(theta);
    F(2) = C2 - DvDt*sin(theta);
end