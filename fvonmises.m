



function sms = fvonmises(lfm,x,i,mai,geo,res,sms)


for n = 1:1000

        % Shear forces in the y direction where x=0 is the left face of the
        % shaft
        sms(1,n)=... 
          -lfm(6,i)*step(x(n)) ... % Pulley weight
         - lfm(9,i)*step(x(n)-geo(1,1)) ... % left bearing reaction
         - lfm(4,i)*step(x(n)-(geo(4,1)/2)) ... % flywheel weight
         + lfm(1,i)*step(x(n)-(geo(1,1)+geo(2,1))) ... cutting force
         - lfm(8,i)*step(x(n)-geo(4,1)); % right being reaction

        % Shear forces in the z direction where x=0 is the left face of the
        % shaft
        sms(2,n)=...
         + lfm(7,i)*step(x(n)) ... % pulley force
         - lfm(10,i)*step(x(n)-geo(1,1)) ... % left bearing reaction
         + lfm(11,i)*step(x(n)-geo(4,1)); % right bearing reaction

        % Moment about the x axis
        sms(3,n)=...
         - (lfm(12,i) + lfm(2,i))*step(x(n))...
         + (lfm(12,i) + lfm(2,i))*step(x(n)-(geo(1,1)+geo(2,1)));

        % Moment about the y axis
        sms(4,n)=...
         + lfm(7,i)*xstep(x(n))...
         - lfm(10,i)*xstep(x(n)-geo(1,1))...
         + lfm(11,i)*xstep(x(n)-geo(4,1));       

        % Moment about the z axis
        sms(5,n)=... 
          -lfm(6,i)*xstep(x(n))...
         - lfm(9,i)*xstep(x(n)-geo(1,1))...
         - lfm(4,i)*xstep(x(n)-(geo(4,1)/2))...
         + lfm(1,i)*xstep(x(n)-(geo(1,1)+geo(2,1)))...
         - lfm(8,i)*xstep(x(n)-geo(4,1));

        % Axial stress
        %sms(6,n) =

        % Bending stress in x
        sms(7,n) = (sms(5,n)*geo(5,i)/geo(7,i)... % bending moment z
                 + sms(4,n)*geo(5,i)/geo(7,i))/144; % bending moment y

        % Shear stress in xy plane
        sms(8,n) = (sms(1,n)/geo(6,i))/144; % Direct
                
        % Shear stress in zx plane
        sms(9,n) = (sms(3,n)/geo(8,i)... % Torq
                 + sms(2,n)/geo(6,i))/144; % Direct

        % VonMises at radius for all x
        sms(11,n) = (((sms(7,n) + lfm(6,i))^2....
                    + 6*(sms(8,n)^2 + sms(9,n)^2))^.5)/sqrt(2);

        % Prevents discarding previous values
        sms(10,n) = sms(10,n);

 
                
    end