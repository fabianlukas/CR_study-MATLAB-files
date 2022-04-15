function surfActionSpace(Au,Ar,data)
figure
surf(Au, Ar, data)
view(0,90)
h = colorbar;
h.Label.String = 'min distance';
xlabel('long force')
ylabel('torque')
end

