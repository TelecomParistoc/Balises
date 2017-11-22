X2 = 3000
X3 = 1500
Y3 = 2000

r1 = 1800
r2 = 2000
r3 = 900

x = (r1^2-r2^2+X2^2)/(2*X2)
y = (r1^2-r3^2+X3^2+Y3^2-2*X3*x)/(2*Y3)
z = sqrt(r1^2-x^2-y^2)

rectangle(gca(), "Position", [0,0,3000,2000])
rectangle(gca(), "Position", [-50,-50,100,100], "Curvature", 1, "FaceColor", "red")
rectangle(gca(), "Position", [2950,-50,100,100], "Curvature", 1, "FaceColor", "red")
rectangle(gca(), "Position", [1450,1950,100,100], "Curvature", 1, "FaceColor", "red")

rectangle(gca(), "Position", [-r1,-r1,2*r1,2*r1], "Curvature", 1, "EdgeColor", "blue")
rectangle(gca(), "Position", [3000-r2,-r2,r2*2,r2*2], "Curvature", 1, "EdgeColor", "blue")
rectangle(gca(), "Position", [1500-r3,2000-r3,r3*2,r3*2], "Curvature", 1, "EdgeColor", "blue")

rectangle(gca(), "Position", [x-50,y-50,100,100], "Curvature", 1, "FaceColor", "green")

axis(gca(), "equal")

#Kalman
x0 = 1400
y0 = 1200
#standard deviation of the noise of the position
qx = 30
qy = 30
dt = 0.03468
Treply = 211*10^(-6)
#clock offset errors 
e1 = 0.002
e2 = 0.002
e3 = 0.002

X = [x0; y0]
X(2)
A = eye(2)
P = zeros(2)
Q = [qx*dt,0;0,qy*dt]
R = Treply/2*diag([e1,e2,e3])
D = [1800;2000;900]

#update
Xproj = A*X
Pproj = Q
H = [X(1)/sqrt(X(1)^2+X(2)^2),X(2)/sqrt(X(1)^2+X(2)^2);(X(1)-X2)/sqrt((X(1)-X2)^2+X(2)^2),X(2)/sqrt((X(1)-X2)^2+X(2)^2);(X(1)-X3)/sqrt((X(1)-X3)^2+(X(2)-Y3)^2),(X(2)-Y3)/sqrt((X(1)-X3)^2+(X(2)-Y3)^2)]
K = Pproj*H'*inv(H*Pproj*H'+R)
X = Xproj+K*(D-[sqrt(X(1)^2+X(2)^2);sqrt((X(1)-X2)^2+X(2)^2);sqrt((X(1)-X3)^2+(X(2)-Y3)^2)])
P = (eye(2)-K*H)*Pproj
rectangle(gca(), "Position", [X(1)-50,X(2)-50,100,100], "Curvature", 1, "FaceColor", "blue")
