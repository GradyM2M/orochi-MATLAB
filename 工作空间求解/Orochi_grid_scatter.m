function [] = Orochi_grid_scatter(m,n,h)
xx=1;yy=1;zz=1;
H=(h-1)*m*n;
C=m*n*h;
N=m*n;
[x,y,z]=meshgrid(-(n-1)*xx/2:(n-1)*xx/2,-(n-1)*yy/2:(n-1)*yy/2,-(n-1)*zz/2:(n-1)*zz/2);
Cube=reshape(1:N*h,m,n,h);
MN=2*m*n-m-n;
X=zeros(C,1);Y=X;Z=X;
for i=1:m
    for j=1:n
        for k=1:h
            X(Cube(i,j,k))=x(i,j,k);
            Y(Cube(i,j,k))=y(i,j,k);
            Z(Cube(i,j,k))=z(i,j,k);
        end
    end
end
linkN=0;
Img=sqrt(-1);
A=zeros(H+h*MN,1);
for k=1:h-1
    for j=1:n
        for i=1:m
            linkN=linkN+1;
            A(linkN)=Cube(i,j,k)+Img*Cube(i,j,k+1);
        end
    end
end
for k=1:h
    for j=1:n-1
        for i=1:m
            linkN=linkN+1;
            A(linkN)=Cube(i,j,k)+Img*Cube(i,j+1,k);
        end
    end
    for j=1:n
        for i=1:m-1
            linkN=linkN+1;
            A(linkN)=Cube(i,j,k)+Img*Cube(i+1,j,k);
        end
    end
end
figure(2)
% plot3(X,Y,Z,'r.')
% axis equal
P1=real(A);P2=imag(A);
line([X(P1)';X(P2)'],[Y(P1)';Y(P2)'],[Z(P1)';Z(P2)'],'color','k')
xlabel('x');ylabel('y');zlabel('z');
end