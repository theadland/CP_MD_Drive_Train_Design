


function crv = fcrit(lfm,geo,res,i,x,E)


for n=1:res
    
    crv(1,n)=... % Deflection in y along x
         (-lfm(6,i)/6*xstep(x(n))^3 ... % Pulley weight
         - lfm(9,i)/6*xstep(x(n)-geo(1,1))^3 ... % left bearing reaction
         - lfm(4,i)/6*xstep(x(n)-(geo(4,1)/2))^3 ... % flywheel weight
         + lfm(1,i)/6*xstep(x(n)-(geo(1,1)+geo(2,1)))^3 ... cutting force
         - lfm(8,i)/6*xstep(x(n)-geo(4,1))^3 ... % right being reaction
         + 17.18*x(n)...
         - 25.848)/(E*geo(7,i));
end
   
for n=1
    crv(1,n)=... % Deflection in y along x
         (-lfm(6,i)/6*xstep(x(n))^3 ... % Pulley weight
         - lfm(9,i)/6*xstep(x(n)-geo(1,1))^3 ... % left bearing reaction
         - lfm(4,i)/6*xstep(x(n)-(geo(4,1)/2))^3 ... % flywheel weight
         + lfm(1,i)/6*xstep(x(n)-(geo(1,1)+geo(2,1)))^3 ... cutting force
         - lfm(8,i)/6*xstep(x(n)-geo(4,1))^3 ... % right being reaction
         + 25.848*x(n))/(E*geo(7,i));



end