t = 0:0.1:20
a = sin(t)
b = cos(t)

%%
% posamezne celice, z contro enter pozenes tisto celico na kateri je kliknjeno

fh = figure(1) %figure handle
plot(t,a,'r')
hold all %%da ti ne zbrise signala
plot(t,b,'k--')
title('naslov')
xlabel('t / s')
ylabel('OZNAKA')
grid on
hold off %da ko pozenes narise na novo

fig2 = figure(2)
plot(a,b)
axis equal

fig3 = figure(3)
ax1 = subplot(2,1,1)
plot(t,a)
ax2 = subplot(2,1,2)
plot(t,b)

linkaxes([ax1,ax2], 'xy') %povezes kontrole