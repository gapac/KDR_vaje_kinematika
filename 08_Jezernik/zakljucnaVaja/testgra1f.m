%
close all
x=testgraf(1:end,1);
y=testgraf(1:end,2);
z=testgraf(1:end,3);
plot3(x,y,z)

hold all
x=xTrajec(1:end,2);
y=xTrajec(1:end,3);
z=xTrajec(1:end,4);
plot3(x,y,z)
    