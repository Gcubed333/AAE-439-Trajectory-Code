function g = getGravity(h)
   g = 1000*(3.986004418*10^5)/(6.3781370*10^3+(h/1000))^2;

end