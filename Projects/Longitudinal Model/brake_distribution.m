% Vehicle data
m  = veh.m;
l  = veh.l;
lF = veh.lF;
lR = veh.lR;
hG  = veh.hG;
KbF = 2*bra.KbF; % [Nm/] Axle equivalent pressure to torque ratio
KbR = 2*bra.KbR; % Axle equivalent pressure to torque ratio
Re  = tir.Re;

% Create matices to plot the lines for front and rear longitudinal grip
muVect  = linspace(0,1.4,8)';
fxfVect = linspace(0,1,11)';
fxrVect = linspace(0,1,11)';

for ii=1:length(muVect)
    for jj=1:length(fxfVect)
        fxfMatr(ii,jj) = muVect(ii)*(lR + hG*fxrVect(jj)) / (l - muVect(ii)*hG);
        fxrMatr(ii,jj) = muVect(ii)*(lF - hG*fxfVect(jj)) / (l + muVect(ii)*hG);
    end
end

% Ideal braking disrtibution forces
mu    = linspace(0,2,1000)';
fxfId = mu./l.*(lR+mu.*hG);
fxrId = mu./l.*(lF-mu.*hG);

figure(54) % NORMALIZED FORCES
hold on
plot(fxfId,fxrId,'b','Linewidth',2)
for ii=2:length(muVect)-1
    plot(fxfMatr(ii,:),fxrVect,'--k','Linewidth',0.5)
    text(fxfMatr(ii,1)+0.015,fxrVect(1)+0.015,['\mu_{F}=' num2str(muVect(ii))],'Rotation', atan((fxrVect(end)-fxrVect(1))/(fxfMatr(ii,end)-fxfMatr(ii,1)))*180/pi )
    plot(fxfVect,fxrMatr(ii,:),'--k','Linewidth',0.5)
    text(fxfVect(1)+0.015,fxrMatr(ii,1)+0.01,['\mu_{R}=' num2str(muVect(ii))],'Rotation', atan((fxrMatr(ii,end)-fxrMatr(ii,1)))/(fxfVect(end)-fxfVect(1))*180/pi )
end
for ii=2:2:10
    plot([0,ii]./10,[ii,0]./10,'.-.k','Linewidth',0.5)
end
grid on;
xlabel('f_{xF} [-]');
ylabel('f_{xR} [-]');
xlim([0 1.0]);
ylim([0 0.5]);
legend('Ideal');

pFId = fxfId*(m*g)/(KbF/Re); % [bar]
pRId = fxrId*(m*g)/(KbR/Re); % [bar]

figure(55);
hold on
plot(pFId,pRId,'b','Linewidth',2);
for ii=2:length(muVect)-1
    plot((fxfMatr(ii,:))      *(m*g)/(KbF/Re), (fxrVect)           *(m*g)/(KbR/Re),'--k','Linewidth',0.5)
    text((fxfMatr(ii,1)+0.015)*(m*g)/(KbF/Re), (fxrVect(1)+0.015)  *(m*g)/(KbR/Re),['\mu_{F}=' num2str(muVect(ii))],'Rotation', atan((fxrVect(end)-fxrVect(1))/(fxfMatr(ii,end)-fxfMatr(ii,1)))*180/pi )
    plot((fxfVect)            *(m*g)/(KbF/Re), (fxrMatr(ii,:))     *(m*g)/(KbR/Re),'--k','Linewidth',0.5)
    text((fxfVect(1)+0.015)   *(m*g)/(KbF/Re), (fxrMatr(ii,1)+0.01)*(m*g)/(KbR/Re),['\mu_{R}=' num2str(muVect(ii))],'Rotation', atan((fxrMatr(ii,end)-fxrMatr(ii,1)))/(fxfVect(end)-fxfVect(1))*180/pi )
end
for ii=2:2:10
    plot([0,ii]./10,[ii,0]./10,'.-.k','Linewidth',0.5)
end
grid on;
xlabel('p_{F} [-]');
ylabel('p_{R} [-]');
xlim([0 1.0]*(m*g)/(KbF/Re));
ylim([0 0.5]*(m*g)/(KbR/Re));

% Pressure distributor
pF = pFId; % [bar]
pR = pF; % [bar] - follows pF until pDis
pDis = 121.5289; % [bar]
mDis = 0.08; % [bar/bar]

pR(pF > pDis) = (pF(pF > pDis) - pDis) * mDis + pDis; % After pDis, pR follows reduced slope
fxf = pF*(KbF/Re)/(m*g); % [bar]
fxr = pR*(KbR/Re)/(m*g); % [bar]

figure(55);
plot(pF,pR,'r','Linewidth',2);
legend('Ideal','Real');

figure(54);
plot(fxf,fxr,'r','Linewidth',2);
legend('Ideal','Real');

bra.pDis = pDis; % [bar]
bra.mDis = mDis; % [bar/bar]

clear m l lF lR hG KbF KbR Re...
      muVect fxfVect fxrVect fxfMatr fxrMatr...
      mu fxfId fxrId fxf fxr ...
      pFId pRId pF pR mDis pDis









